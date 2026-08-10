"""Cross-engine validation bridge: fly quadsim's controller inside MuJoCo.

Builds a MuJoCo model of the same vehicle that :mod:`quadsim.dynamics` integrates
analytically — same mass, inertia, rotor geometry, drag-torque coefficient, and
per-motor thrust limits, all taken from :class:`quadsim.params.QuadParams` — and
runs the unmodified :class:`quadsim.controller.SE3Controller` against MuJoCo's
independent rigid-body engine. Agreement between the two engines on the same
trajectory is evidence that neither the dynamics derivation nor the integrator
hides an error (the industry-standard "sim-to-sim" check done before hardware).

Rotor convention (matches ``QuadrotorDynamics``): X configuration with
``d = L / sqrt(2)``; rotor sites ``r1=(+d,+d,0)``, ``r2=(-d,+d,0)``,
``r3=(-d,-d,0)``, ``r4=(+d,-d,0)``; rotors 1 and 3 spin CCW (reaction torque
``-c_tau`` per unit thrust about body z), rotors 2 and 4 spin CW (``+c_tau``).
Each MuJoCo actuator applies ``[0, 0, f_i]`` force and ``[0, 0, -+c_tau * f_i]``
torque at its rotor site, in the site (body) frame, with
``ctrlrange=[f_motor_min, f_motor_max]`` enforcing the same saturation the
mixer applies.

State conventions agree with quadsim exactly (verified by tests): free-joint
``qpos = [p (world), quat (w,x,y,z)]``; ``qvel[0:3]`` world-frame linear
velocity; ``qvel[3:6]`` BODY-frame angular velocity.

Requires the optional dependency ``mujoco`` (and ``imageio`` for video);
install with ``pip install quadsim[mujoco]``. On Apple-silicon Macs MuJoCo
needs a native arm64 Python build.
"""
from __future__ import annotations

from typing import Any, Callable, Optional, Tuple
from xml.sax.saxutils import escape

import numpy as np

import mujoco

from quadsim.dynamics import QuadrotorDynamics, QuadState
from quadsim.maths import quat_normalize
from quadsim.params import QuadParams
from quadsim.sim import History

_PROP_RADIUS = 0.0275  # m: 55 mm prop disc, visual only (docs/DESIGN.md prop size)


def quad_mjcf(
    params: QuadParams,
    mesh_path: Optional[str] = None,
    timestep: float = 5e-4,
    start_z: float = 1.0,
) -> str:
    """Return the MJCF XML for the quadsim vehicle as a MuJoCo model.

    The body carries an explicit ``<inertial>`` (mass ``m``, diagonal inertia
    from ``J``), four rotor sites at the mixer geometry, and one thrust
    actuator per rotor. Gravity, integrator (RK4), and zero air density match
    the quadsim world. A ground plane provides contact so a control failure
    ends in a physical crash rather than a fall through the floor.

    Note on inertia: the catalog Crazyflie values in ``QuadParams`` violate the
    rigid-body triangle inequality (``Jx + Jy >= Jz`` must hold for any real
    mass distribution, since ``Jx + Jy - Jz = 2 * integral(z^2 dm)``); they are
    identified, rounded numbers describing no realizable rigid body, and MuJoCo
    refuses them outright. (Its own ``balanceinertia`` fix is too blunt — it
    averages all three moments, distorting pitch/roll by ~34%.) This builder
    instead applies the minimal projection ``Jz <- min(Jz, Jx + Jy)`` — a 1%
    change on the yaw axis only, equivalent to treating the vehicle as the flat
    disc it nearly is. quadsim keeps integrating the catalog values, so the
    cross-engine comparison doubles as a small, honest model-mismatch
    experiment on the yaw axis.

    Args:
        params: quadrotor physical parameters (single source of truth).
        mesh_path: optional absolute path to a binary STL of the frame
            (millimeter units; e.g. ``out/frame_binary.stl``). Used as the
            visual body; when None, a simple cross of boxes is drawn instead.
        timestep: MuJoCo integrator timestep, s.
        start_z: initial height of the free joint, m.

    Returns:
        MJCF XML string accepted by ``mujoco.MjModel.from_xml_string``.
    """
    d = params.L / np.sqrt(2.0)
    c = params.c_tau
    J = np.asarray(params.J, dtype=float)
    jx, jy, jz = float(J[0, 0]), float(J[1, 1]), float(J[2, 2])
    jz = min(jz, jx + jy)  # minimal physical projection; see inertia note above
    # (x, y, spin sign) per rotor; sign is the z reaction torque per unit thrust.
    rotors = [(+d, +d, -1.0), (-d, +d, +1.0), (-d, -d, -1.0), (+d, -d, +1.0)]

    scene_assets = (
        '<texture type="skybox" builtin="gradient" rgb1="0.53 0.71 0.92" '
        'rgb2="0.86 0.92 0.98" width="256" height="256"/>'
        '<texture type="2d" name="grid" builtin="checker" rgb1="0.80 0.83 0.87" '
        'rgb2="0.62 0.67 0.73" width="512" height="512"/>'
        '<material name="grid" texture="grid" texrepeat="24 24" reflectance="0.1"/>'
    )
    if mesh_path is not None:
        mesh_file = escape(mesh_path, {'"': "&quot;"})
        asset = (
            f'<asset>{scene_assets}<mesh name="frame" file="{mesh_file}" '
            'scale="0.001 0.001 0.001"/></asset>'
        )
        body_geoms = [
            '<geom type="mesh" mesh="frame" pos="0 0 -0.004" '
            'rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>'
        ]
    else:
        asset = f"<asset>{scene_assets}</asset>"
        body_geoms = [
            f'<geom type="box" size="{params.L:.4f} 0.005 0.002" euler="0 0 0.7854" '
            'rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>',
            f'<geom type="box" size="{params.L:.4f} 0.005 0.002" euler="0 0 -0.7854" '
            'rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>',
        ]
    # Collision proxy (sphere at the hub) + translucent prop discs per rotor.
    body_geoms.append(
        '<geom type="sphere" size="0.012" rgba="0.1 0.1 0.1 0" '
        'contype="1" conaffinity="1"/>'
    )
    sites = []
    for i, (rx, ry, _) in enumerate(rotors, start=1):
        sites.append(f'<site name="rotor{i}" pos="{rx:.12g} {ry:.12g} 0" size="0.003"/>')
        body_geoms.append(
            f'<geom type="cylinder" size="{_PROP_RADIUS} 0.0008" '
            f'pos="{rx:.6f} {ry:.6f} 0.006" rgba="0.25 0.55 0.9 0.35" '
            'contype="0" conaffinity="0"/>'
        )
    actuators = []
    for i, (_, _, s) in enumerate(rotors, start=1):
        actuators.append(
            f'<motor name="m{i}" site="rotor{i}" '
            f'gear="0 0 1 0 0 {s * c:.12g}" '
            f'ctrlrange="{params.f_motor_min:.12g} {params.f_motor_max:.12g}"/>'
        )
    nl = "\n          "
    return f"""
<mujoco model="quadsim-x">
  <compiler angle="radian"/>
  <option timestep="{timestep}" gravity="0 0 {-params.g}" integrator="RK4"
          density="0" viscosity="0"/>
  {asset}
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <global offwidth="1280" offheight="720"/>
  </visual>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="6 6 0.1" material="grid"/>
    <body name="quad" pos="0 0 {start_z}">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="{params.m}" diaginertia="{jx} {jy} {jz}"/>
      <camera name="track" mode="trackcom" pos="-0.42 -0.42 0.26"
              xyaxes="0.707 -0.707 0 0.210 0.210 0.959"/>
      {nl.join(body_geoms)}
      {nl.join(sites)}
    </body>
  </worldbody>
  <actuator>
    {nl.join(actuators)}
  </actuator>
</mujoco>
"""


class MujocoQuadSim:
    """A quadsim vehicle living inside a MuJoCo world.

    Wraps ``MjModel``/``MjData`` and reuses :meth:`QuadrotorDynamics.mix` for
    thrust allocation, so command saturation is bit-identical to quadsim's.
    """

    def __init__(
        self,
        params: QuadParams,
        mesh_path: Optional[str] = None,
        timestep: float = 5e-4,
    ) -> None:
        """Build the model. Args as in :func:`quad_mjcf`."""
        self.params = params
        self.timestep = float(timestep)
        self.dyn = QuadrotorDynamics(params)
        self.model = mujoco.MjModel.from_xml_string(
            quad_mjcf(params, mesh_path=mesh_path, timestep=timestep)
        )
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

    def reset(self, state: QuadState) -> None:
        """Set the MuJoCo state from a :class:`QuadState` and zero the controls."""
        self.data.qpos[0:3] = state.p
        self.data.qpos[3:7] = quat_normalize(state.q)
        self.data.qvel[0:3] = state.v
        self.data.qvel[3:6] = state.w
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def state(self) -> QuadState:
        """Read the current MuJoCo state back as a :class:`QuadState`."""
        return QuadState(
            p=self.data.qpos[0:3].copy(),
            v=self.data.qvel[0:3].copy(),
            q=quat_normalize(self.data.qpos[3:7].copy()),
            w=self.data.qvel[3:6].copy(),
        )

    def apply(self, f: float, tau: np.ndarray) -> Tuple[float, np.ndarray]:
        """Allocate a wrench to the four rotor actuators (zero-order hold).

        Returns the post-saturation ``(f, tau)`` actually applied, exactly as
        ``QuadrotorDynamics`` reports it.
        """
        motors = self.dyn.mix(f, tau)
        self.data.ctrl[:] = motors
        return self.dyn.unmix(motors)

    def step(self, dt: float) -> None:
        """Advance MuJoCo by ``dt``, an exact integer number of internal timesteps.

        Raises:
            ValueError: if ``dt`` is not a positive integer multiple of
                ``timestep`` — rounding silently would advance the physics
                clock by ``n * timestep != dt`` per control step, skewing the
                plant against the controller/reference clock.
        """
        ratio = dt / self.timestep
        n_sub = int(round(ratio))
        if n_sub < 1 or abs(ratio - n_sub) > 1e-6 * n_sub:
            raise ValueError(
                f"dt={dt} must be a positive integer multiple of "
                f"timestep={self.timestep} (got dt/timestep={ratio})"
            )
        for _ in range(n_sub):
            mujoco.mj_step(self.model, self.data)


def simulate_mujoco(
    params: QuadParams,
    controller: Any,
    ref_fn: Callable[[float], Any],
    state0: QuadState,
    T: float,
    dt: float = 0.002,
    mesh_path: Optional[str] = None,
    timestep: float = 5e-4,
) -> History:
    """Closed-loop run of ``controller`` against MuJoCo physics; mirrors ``sim.simulate``.

    Same loop structure, sample times, and recorded quantities as
    :func:`quadsim.sim.simulate`, so the returned :class:`History` is directly
    comparable (and plottable with :mod:`quadsim.viz`).

    Args:
        params: quadrotor parameters.
        controller: object with ``compute(state, ref_fn, t) -> (f, tau)``.
        ref_fn: reference callable ``t -> FlatOutput``.
        state0: initial state.
        T: total simulated time, s.
        dt: control period, s (held zero-order over MuJoCo substeps).
        mesh_path: optional frame STL for the visual body (see :func:`quad_mjcf`).
        timestep: MuJoCo integrator timestep, s.
    """
    world = MujocoQuadSim(params, mesh_path=mesh_path, timestep=timestep)
    world.reset(state0)
    n = int(round(T / dt))
    t_arr = np.zeros(n)
    p_arr = np.zeros((n, 3))
    v_arr = np.zeros((n, 3))
    q_arr = np.zeros((n, 4))
    w_arr = np.zeros((n, 3))
    f_arr = np.zeros(n)
    tau_arr = np.zeros((n, 3))
    ref_p_arr = np.zeros((n, 3))
    ref_v_arr = np.zeros((n, 3))

    for k in range(n):
        t = k * dt
        state = world.state()
        f_cmd, tau_cmd = controller.compute(state, ref_fn, t)
        f_act, tau_act = world.apply(f_cmd, tau_cmd)
        ref = ref_fn(t)
        t_arr[k] = t
        p_arr[k] = state.p
        v_arr[k] = state.v
        q_arr[k] = state.q
        w_arr[k] = state.w
        f_arr[k] = f_act
        tau_arr[k] = tau_act
        ref_p_arr[k] = ref.pos
        ref_v_arr[k] = ref.vel
        world.step(dt)

    return History(
        t=t_arr, p=p_arr, v=v_arr, q=q_arr, w=w_arr, f=f_arr, tau=tau_arr,
        ref_p=ref_p_arr, ref_v=ref_v_arr,
    )


def render_history(
    hist: History,
    params: QuadParams,
    save: str,
    mesh_path: Optional[str] = None,
    fps: int = 50,
    width: int = 960,
    height: int = 544,
    camera: str = "track",
) -> str:
    """Replay a recorded :class:`History` through MuJoCo's renderer to an MP4.

    Only the recorded poses are replayed (``qpos = [p, q]`` per frame); no
    physics is re-run, so the video shows exactly the trajectory that was
    simulated — by either engine.

    Args:
        hist: recorded run (from :func:`simulate_mujoco` or ``sim.simulate``).
        params: quadrotor parameters (rebuilds the same model).
        save: output ``.mp4`` path.
        mesh_path: optional frame STL for the visual body.
        fps: video frame rate.
        width: frame width, px.
        height: frame height, px.
        camera: model camera name to render from.

    Returns:
        The ``save`` path.

    Raises:
        ValueError: if ``hist`` is empty, or ``width``/``height`` exceed the
            model's offscreen framebuffer (1280x720).
    """
    import imageio

    if len(hist.t) == 0:
        raise ValueError("hist is empty; nothing to render")
    if width > 1280 or height > 720:
        raise ValueError(
            f"{width}x{height} exceeds the model's 1280x720 offscreen "
            "framebuffer (see the <visual><global> clause in quad_mjcf)"
        )
    world = MujocoQuadSim(params, mesh_path=mesh_path)
    renderer = mujoco.Renderer(world.model, height=height, width=width)
    try:
        # End-inclusive so a short (even single-sample) History still yields
        # at least one frame.
        frame_times = np.arange(0.0, float(hist.t[-1]) + 0.5 / fps, 1.0 / fps)
        frames = []
        for ft in frame_times:
            k = int(np.searchsorted(hist.t, ft, side="right")) - 1
            k = max(0, min(k, len(hist.t) - 1))
            world.data.qpos[0:3] = hist.p[k]
            world.data.qpos[3:7] = quat_normalize(hist.q[k])
            mujoco.mj_forward(world.model, world.data)
            renderer.update_scene(world.data, camera=camera)
            frames.append(renderer.render())
        imageio.mimwrite(save, frames, fps=fps, quality=8)
    finally:
        renderer.close()
    return save
