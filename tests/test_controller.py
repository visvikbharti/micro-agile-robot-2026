"""Tests for quadsim.controller: SE(3) geometric tracking control."""
from __future__ import annotations

import numpy as np

from quadsim.controller import SE3Controller, flat_to_rotation
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate
from quadsim.trajectory import FlatOutput, hover_ref


def test_flat_to_rotation_valid():
    rng = np.random.default_rng(0)
    for _ in range(20):
        F = rng.normal(size=3)
        F *= rng.uniform(0.2, 5.0) / np.linalg.norm(F)
        yaw = rng.uniform(-np.pi, np.pi)
        R = flat_to_rotation(F, yaw)
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-10)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-10)
        assert np.allclose(R[:, 2], F / np.linalg.norm(F), atol=1e-10)


def test_flat_to_rotation_guards():
    # near-zero force falls back to b3 = e3
    R0 = flat_to_rotation(np.zeros(3), 0.7)
    assert np.allclose(R0.T @ R0, np.eye(3), atol=1e-10)
    assert np.allclose(R0[:, 2], np.array([0.0, 0.0, 1.0]), atol=1e-10)
    # b3 parallel to the yaw heading triggers the rotated-b1c fallback
    R1 = flat_to_rotation(np.array([2.0, 0.0, 0.0]), 0.0)
    assert np.allclose(R1.T @ R1, np.eye(3), atol=1e-10)
    assert np.isclose(np.linalg.det(R1), 1.0, atol=1e-10)
    assert np.allclose(R1[:, 2], np.array([1.0, 0.0, 0.0]), atol=1e-10)


def test_hover_convergence():
    params = QuadParams()
    controller = SE3Controller(params)
    target = np.array([0.0, 0.0, 1.0])
    offset = np.array([0.3, 0.4, 0.0])  # ||offset|| = 0.5 m
    state0 = QuadState.hover(target + offset)
    hist = simulate(params, controller, hover_ref(target), state0, T=4.0, dt=0.002)
    assert hist.pos_error()[-1] < 0.01


def _circle_ref(t: float) -> FlatOutput:
    r, om, z0 = 1.0, 1.5, 1.0
    c, s = np.cos(om * t), np.sin(om * t)
    return FlatOutput(
        pos=np.array([r * c, r * s, z0]),
        vel=np.array([-r * om * s, r * om * c, 0.0]),
        acc=np.array([-r * om**2 * c, -r * om**2 * s, 0.0]),
        jerk=np.array([r * om**3 * s, -r * om**3 * c, 0.0]),
        snap=np.array([r * om**4 * c, r * om**4 * s, 0.0]),
        yaw=0.0,
        yaw_rate=0.0,
    )


def test_circle_tracking():
    params = QuadParams()
    controller = SE3Controller(params)
    state0 = QuadState(
        p=np.array([1.0, 0.0, 1.0]),
        v=np.array([0.0, 1.5, 0.0]),
        q=np.array([1.0, 0.0, 0.0, 0.0]),
        w=np.zeros(3),
    )
    hist = simulate(params, controller, _circle_ref, state0, T=8.0, dt=0.002)
    assert hist.rms_pos_error(t_from=4.0) < 0.08
