"""Live interactive MuJoCo flight — YOUR controller, YOUR knobs, real time.

Opens the MuJoCo viewer with the SE(3) controller flying closed-loop, so you
can orbit the camera, grab the vehicle mid-flight and yank it (double-click
the body to select it, then Ctrl+drag to apply a force / Ctrl+right-drag a
torque — watch the controller fight back), and re-run with different gains,
speeds, or time scaling to SEE what each hyperparameter does.

macOS note: MuJoCo's viewer must run under its own launcher:

    .venv-mj/bin/mjpython demos/fly_mujoco_live.py                 # fig8, stock gains
    .venv-mj/bin/mjpython demos/fly_mujoco_live.py --traj hover
    .venv-mj/bin/mjpython demos/fly_mujoco_live.py --kp 0.25      # mushy position loop
    .venv-mj/bin/mjpython demos/fly_mujoco_live.py --kR 0.1      # weak attitude loop
    .venv-mj/bin/mjpython demos/fly_mujoco_live.py --avg-speed 4.5  # too fast: saturates
    .venv-mj/bin/mjpython demos/fly_mujoco_live.py --time-scale 0.25  # slow motion

Experiments worth running (each is one flag away):
  --kp/--kv scale the position/velocity gains: low kp = lazy, drifting corners;
    high kv relative to kp = overdamped crawl.
  --kR/--kw scale the attitude gains: low kR = the inner loop can't keep up
    with the outer loop's demands — wobble, then a crash into the (real) floor.
  --avg-speed raises the trajectory's aggressiveness until the 0.16 N/motor
    limit saturates and tracking falls apart — the actuator ceiling made visible.
  --time-scale 0.25 slows the world 4x so you can watch the nose lead the turn
    (yaw feedforward) frame by frame.

Export the model for the MuJoCo.app GUI instead (sliders for each motor,
physics parameter panels, no controller):

    .venv-mj/bin/python demos/fly_mujoco_live.py --export-xml out/quad_mujoco.xml
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)


def lemniscate_waypoints(points_per_lap: int = 16, a: float = 1.2,
                         z: float = 1.0) -> np.ndarray:
    """One lap of the Gerono lemniscate (same geometry as demo_figure8)."""
    theta = np.linspace(0.0, 2.0 * np.pi, points_per_lap + 1)
    x = a * np.sin(theta)
    y = a * np.sin(theta) * np.cos(theta)
    return np.column_stack([x, y, np.full(theta.shape, z)])


def main(argv: Optional[List[str]] = None) -> int:
    """Run the live viewer (or export the MJCF for the MuJoCo app)."""
    parser = argparse.ArgumentParser(
        description="Fly the SE(3) controller live in the MuJoCo viewer.")
    parser.add_argument("--traj", choices=["hover", "fig8"], default="fig8")
    parser.add_argument("--kp", type=float, default=1.0, help="position gain scale")
    parser.add_argument("--kv", type=float, default=1.0, help="velocity gain scale")
    parser.add_argument("--kR", type=float, default=1.0, help="attitude gain scale")
    parser.add_argument("--kw", type=float, default=1.0, help="body-rate gain scale")
    parser.add_argument("--avg-speed", type=float, default=2.5,
                        help="figure-eight average speed budget, m/s")
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="wall-clock rate: 0.25 = 4x slow motion")
    parser.add_argument("--mesh", default=os.path.join(PROJECT_ROOT, "out", "frame_binary.stl"),
                        help="binary STL for the visual frame ('' to disable)")
    parser.add_argument("--export-xml", default=None, metavar="PATH",
                        help="write the MJCF model (with a hover keyframe) for "
                             "the MuJoCo.app GUI and exit")
    parser.add_argument("--smoke", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    from quadsim.controller import SE3Controller
    from quadsim.dynamics import QuadState
    from quadsim.mujoco_bridge import MujocoQuadSim, quad_mjcf
    from quadsim.params import QuadParams
    from quadsim.trajectory import MinSnapTrajectory, hover_ref

    params = QuadParams()
    mesh = args.mesh if args.mesh and os.path.exists(args.mesh) else None

    if args.export_xml:
        xml = quad_mjcf(params, mesh_path=os.path.abspath(mesh) if mesh else None)
        h = params.weight / 4.0
        keyframe = (f'  <keyframe><key name="hover" qpos="0 0 1 1 0 0 0" '
                    f'ctrl="{h:.6f} {h:.6f} {h:.6f} {h:.6f}"/></keyframe>\n')
        xml = xml.replace("</mujoco>", keyframe + "</mujoco>")
        os.makedirs(os.path.dirname(os.path.abspath(args.export_xml)), exist_ok=True)
        with open(args.export_xml, "w") as fh:
            fh.write(xml)
        print(f"wrote {args.export_xml}")
        print("Open it in the MuJoCo app (drag the file onto the window).")
        print(f"Open-loop hover thrust is {h:.4f} N per motor — try to hold it "
              "with the Control sliders and see how long you last.")
        return 0

    m = params.m
    controller = SE3Controller(
        params,
        kp=args.kp * m * np.array([16.0, 16.0, 16.0]),
        kv=args.kv * m * np.array([8.0, 8.0, 8.0]),
        kR=args.kR * np.array([1000.0, 1000.0, 100.0]),
        kw=args.kw * np.array([63.0, 63.0, 20.0]),
    )

    if args.traj == "hover":
        ref, T_loop = hover_ref(np.array([0.0, 0.0, 1.0])), None
        state0 = QuadState.hover(np.array([0.2, -0.2, 0.4]))
    else:
        traj = MinSnapTrajectory(lemniscate_waypoints(), avg_speed=args.avg_speed,
                                 yaw_mode="velocity")
        T_loop = traj.T  # starts and ends at rest at the same point: loops seamlessly
        ref = lambda t: traj(t % T_loop)  # noqa: E731
        state0 = QuadState.hover(lemniscate_waypoints()[0], yaw=float(traj(0.0).yaw))

    world = MujocoQuadSim(params, mesh_path=mesh)
    world.reset(state0)
    dt = 0.002

    print(f"traj={args.traj}  gains scale kp/kv/kR/kw = "
          f"{args.kp}/{args.kv}/{args.kR}/{args.kw}  "
          + (f"avg_speed={args.avg_speed} m/s  lap={T_loop:.1f} s  " if T_loop else "")
          + f"time_scale={args.time_scale}")
    print("Viewer: double-click the body to select; Ctrl+drag = force, "
          "Ctrl+right-drag = torque. Space pauses physics.")

    if args.smoke:
        t = 0.0
        for _ in range(int(1.0 / dt)):
            f, tau = controller.compute(world.state(), ref, t)
            world.apply(f, tau)
            world.step(dt)
            t += dt
        err = float(np.linalg.norm(world.state().p - ref(t).pos))
        print(f"smoke ok, err={err:.3f} m")
        return 0

    import mujoco.viewer
    try:
        viewer = mujoco.viewer.launch_passive(world.model, world.data)
    except RuntimeError as exc:
        print(f"viewer failed to launch: {exc}")
        print("On macOS the viewer needs MuJoCo's launcher — run this script with "
              ".venv-mj/bin/mjpython instead of python.")
        return 1

    t = 0.0
    next_wall = time.perf_counter()
    last_report = 0.0
    with viewer:
        while viewer.is_running():
            f, tau = controller.compute(world.state(), ref, t)
            world.apply(f, tau)
            world.step(dt)
            t += dt
            viewer.sync()
            if t - last_report >= 2.0:
                err = float(np.linalg.norm(world.state().p - ref(t).pos))
                print(f"t={t:7.1f} s  tracking error = {1000.0 * err:6.1f} mm")
                last_report = t
            next_wall += dt / args.time_scale
            sleep = next_wall - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_wall = time.perf_counter()  # fell behind; don't spiral
    return 0


if __name__ == "__main__":
    sys.exit(main())
