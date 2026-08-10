"""Tests for quadsim.mujoco_bridge: model fidelity and cross-engine agreement.

Skipped entirely when the optional ``mujoco`` dependency is not installed
(e.g. in the x86_64 base venv on Apple-silicon Macs; use the arm64 venv).
"""
from __future__ import annotations

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from quadsim.controller import SE3Controller  # noqa: E402
from quadsim.dynamics import QuadState, QuadrotorDynamics  # noqa: E402
from quadsim.mujoco_bridge import MujocoQuadSim, simulate_mujoco  # noqa: E402
from quadsim.params import QuadParams  # noqa: E402
from quadsim.sim import simulate  # noqa: E402
from quadsim.trajectory import MinSnapTrajectory, hover_ref  # noqa: E402


def _lemniscate(points_per_lap: int = 16, a: float = 1.2, z: float = 1.0) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * np.pi, points_per_lap + 1)
    return np.column_stack(
        [a * np.sin(theta), a * np.sin(theta) * np.cos(theta), np.full(theta.shape, z)]
    )


def test_model_matches_params():
    params = QuadParams()
    world = MujocoQuadSim(params)
    body = world.model.body("quad")
    assert np.isclose(body.mass[0], params.m)
    # The builder projects the non-physical catalog Jz onto Jx + Jy (see the
    # quad_mjcf inertia note); x/y moments must be untouched.
    J = np.diag(params.J)
    expected = np.array([J[0], J[1], min(J[2], J[0] + J[1])])
    assert np.allclose(body.inertia, expected, rtol=1e-9)
    jx, jy, jz = body.inertia
    assert jx + jy >= jz - 1e-12
    assert world.model.nu == 4
    assert np.allclose(
        world.model.actuator_ctrlrange,
        [[params.f_motor_min, params.f_motor_max]] * 4,
    )


def test_free_fall_matches_gravity():
    params = QuadParams()
    world = MujocoQuadSim(params)
    world.reset(QuadState.hover(np.array([0.0, 0.0, 2.0])))
    t = 0.3
    world.step(t)
    s = world.state()
    assert np.allclose(s.v, [0.0, 0.0, -params.g * t], atol=1e-9)
    assert np.allclose(s.w, 0.0, atol=1e-12)
    assert np.isclose(s.p[2], 2.0 - 0.5 * params.g * t**2, atol=1e-6)


def test_hover_equilibrium():
    params = QuadParams()
    world = MujocoQuadSim(params)
    p0 = np.array([0.0, 0.0, 1.0])
    world.reset(QuadState.hover(p0))
    for _ in range(500):  # 1 s at dt=0.002
        world.apply(params.weight, np.zeros(3))
        world.step(0.002)
    s = world.state()
    assert np.linalg.norm(s.p - p0) < 1e-6
    assert np.linalg.norm(s.v) < 1e-6
    assert np.linalg.norm(s.w) < 1e-9


def test_allocation_matches_mixer_matrix():
    """Asymmetric motor thrusts must produce the A-matrix wrench in MuJoCo."""
    params = QuadParams()
    dyn = QuadrotorDynamics(params)
    world = MujocoQuadSim(params)
    world.reset(QuadState.hover(np.array([0.0, 0.0, 1.0])))
    motors = np.array([0.09, 0.07, 0.08, 0.06])
    world.data.ctrl[:] = motors
    mujoco.mj_forward(world.model, world.data)
    f, tau = dyn.unmix(motors)
    # Predict with the MODEL's (balanced) inertia — see quad_mjcf inertia note.
    J_model = np.diag(world.model.body("quad").inertia)
    alpha_pred = np.linalg.solve(J_model, tau)
    acc_pred = np.array([0.0, 0.0, -params.g + f / params.m])
    assert np.allclose(world.data.qacc[3:6], alpha_pred, rtol=1e-6, atol=1e-9)
    assert np.allclose(world.data.qacc[0:3], acc_pred, rtol=1e-6, atol=1e-9)


def test_closed_loop_hover_recovery():
    params = QuadParams()
    controller = SE3Controller(params)
    ref = hover_ref(np.array([0.0, 0.0, 1.0]))
    state0 = QuadState.hover(np.array([0.15, -0.1, 0.8]))
    hist = simulate_mujoco(params, controller, ref, state0, T=3.0, dt=0.002)
    assert hist.pos_error()[-1] < 0.01
    assert hist.rms_pos_error(t_from=2.0) < 0.005


def test_step_rejects_non_divisor_dt():
    """Silent dt/timestep rounding would skew the physics clock; must raise."""
    params = QuadParams()
    world = MujocoQuadSim(params, timestep=6e-4)
    world.reset(QuadState.hover(np.array([0.0, 0.0, 1.0])))
    with pytest.raises(ValueError, match="integer multiple"):
        world.step(0.002)  # 0.002 / 6e-4 = 3.33...
    world.step(0.0018)  # exact multiple is fine


def test_fallback_arms_lie_on_rotor_diagonals():
    """Without a mesh, the visual arm boxes must be rotated a true 45 degrees."""
    params = QuadParams()
    world = MujocoQuadSim(params)  # no mesh -> box-cross fallback
    quats = [g for g in world.model.geom_quat if abs(g[3]) > 1e-6]
    assert len(quats) == 2
    for q in quats:
        assert np.isclose(abs(q[3]), np.sin(np.pi / 8), atol=1e-6)  # 45 deg about z


def test_render_history_input_validation():
    from quadsim.mujoco_bridge import render_history
    from quadsim.sim import History

    params = QuadParams()
    empty = History(*(np.zeros((0,)) if n in ("t", "f") else np.zeros((0, k))
                      for n, k in [("t", 0), ("p", 3), ("v", 3), ("q", 4), ("w", 3),
                                   ("f", 0), ("tau", 3), ("ref_p", 3), ("ref_v", 3)]))
    with pytest.raises(ValueError, match="empty"):
        render_history(empty, params, "/tmp/never_written.mp4")
    one = History(t=np.zeros(1), p=np.zeros((1, 3)), v=np.zeros((1, 3)),
                  q=np.array([[1.0, 0, 0, 0]]), w=np.zeros((1, 3)), f=np.zeros(1),
                  tau=np.zeros((1, 3)), ref_p=np.zeros((1, 3)), ref_v=np.zeros((1, 3)))
    with pytest.raises(ValueError, match="framebuffer"):
        render_history(one, params, "/tmp/never_written.mp4", width=1920, height=1080)


def test_cross_engine_figure8_agreement():
    """quadsim's RK4 and MuJoCo must agree on the same closed-loop flight."""
    params = QuadParams()
    waypoints = _lemniscate()
    traj = MinSnapTrajectory(waypoints, avg_speed=2.5, yaw_mode="velocity")
    state0 = QuadState.hover(waypoints[0], yaw=float(traj(0.0).yaw))
    T = traj.T + 0.5
    hist_qs = simulate(params, SE3Controller(params), traj, state0, T, dt=0.002)
    hist_mj = simulate_mujoco(params, SE3Controller(params), traj, state0, T, dt=0.002)

    rms_qs = hist_qs.rms_pos_error()
    rms_mj = hist_mj.rms_pos_error()
    divergence = np.linalg.norm(hist_mj.p - hist_qs.p, axis=1)
    assert rms_mj < 0.12
    assert abs(rms_mj - rms_qs) < 0.03
    assert float(np.sqrt(np.mean(divergence**2))) < 0.08
