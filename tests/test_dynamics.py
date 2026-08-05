"""Tests for quadsim.dynamics: rigid-body model, mixer, saturation."""
from __future__ import annotations

import numpy as np

from quadsim.dynamics import QuadState, QuadrotorDynamics
from quadsim.params import QuadParams


def test_hover_equilibrium():
    params = QuadParams()
    dyn = QuadrotorDynamics(params)
    p0 = np.array([0.0, 0.0, 1.0])
    state = QuadState.hover(p0)
    f = params.m * params.g
    tau = np.zeros(3)
    dt = 0.002
    for _ in range(1000):
        state = dyn.step(state, f, tau, dt)
    assert np.linalg.norm(state.p - p0) < 1e-9
    assert np.linalg.norm(state.v) < 1e-9
    assert np.linalg.norm(state.w) < 1e-9


def test_free_fall():
    params = QuadParams()
    dyn = QuadrotorDynamics(params)
    z0 = 2.0
    state = QuadState.hover(np.array([0.0, 0.0, z0]))
    dt = 0.002
    n_steps = 250
    for _ in range(n_steps):
        state = dyn.step(state, 0.0, np.zeros(3), dt)
    t = n_steps * dt
    dz = state.p[2] - z0
    expected = -0.5 * params.g * t**2
    assert abs(dz - expected) <= 1e-6 * abs(expected)
    assert np.allclose(state.p[:2], 0.0, atol=1e-12)
    assert np.isclose(state.v[2], -params.g * t, rtol=1e-6)


def test_unmix_mix_identity():
    params = QuadParams()
    dyn = QuadrotorDynamics(params)
    f = 0.3
    tau = np.array([2e-4, -1.5e-4, 5e-5])
    motors = dyn.mix(f, tau)
    assert motors.shape == (4,)
    # feasible command: no motor at a limit, so the clip is inactive
    assert np.all(motors > params.f_motor_min)
    assert np.all(motors < params.f_motor_max)
    f2, tau2 = dyn.unmix(motors)
    assert np.isclose(f2, f, rtol=1e-9, atol=1e-12)
    assert np.allclose(tau2, tau, rtol=1e-9, atol=1e-12)


def test_hard_saturation():
    params = QuadParams()
    dyn = QuadrotorDynamics(params)
    motors = dyn.mix(10.0 * params.f_max, np.zeros(3))
    assert np.allclose(motors, params.f_motor_max, atol=1e-12)
    f_act, tau_act = dyn.unmix(motors)
    assert np.isclose(f_act, params.f_max, rtol=1e-12)
    assert np.allclose(tau_act, np.zeros(3), atol=1e-12)
