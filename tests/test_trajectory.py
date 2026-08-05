"""Tests for quadsim.trajectory: minimum-snap polynomial trajectories."""
from __future__ import annotations

import numpy as np

from quadsim.trajectory import MinSnapTrajectory

WAYPOINTS = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.6, 1.2],
        [2.0, -0.4, 1.5],
        [3.0, 0.0, 1.0],
    ]
)
SEGMENT_TIMES = np.array([0.7, 0.8, 0.6])


def _make_traj() -> MinSnapTrajectory:
    return MinSnapTrajectory(WAYPOINTS, segment_times=SEGMENT_TIMES)


def _knot_times() -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(SEGMENT_TIMES)])


def test_waypoint_interpolation():
    traj = _make_traj()
    for t_k, wp in zip(_knot_times(), WAYPOINTS):
        assert np.linalg.norm(traj.eval(t_k).pos - wp) <= 1e-5


def test_endpoint_derivatives_zero():
    traj = _make_traj()
    for t in (0.0, traj.T):
        fo = traj.eval(t)
        assert np.linalg.norm(fo.vel) < 1e-6
        assert np.linalg.norm(fo.acc) < 1e-6
        assert np.linalg.norm(fo.jerk) < 1e-6


def test_finite_difference_consistency():
    traj = _make_traj()
    rng = np.random.default_rng(0)
    h = 1e-4
    knots = _knot_times()
    times = []
    for a, b in zip(knots[:-1], knots[1:]):
        times.extend(rng.uniform(a + 0.02, b - 0.02, size=3))
    for t in times:
        vel_fd = (traj.eval(t + h).pos - traj.eval(t - h).pos) / (2 * h)
        acc_fd = (traj.eval(t + h).vel - traj.eval(t - h).vel) / (2 * h)
        jerk_fd = (traj.eval(t + h).acc - traj.eval(t - h).acc) / (2 * h)
        fo = traj.eval(t)
        assert np.linalg.norm(vel_fd - fo.vel) <= 1e-3 * (1.0 + np.linalg.norm(fo.vel))
        assert np.linalg.norm(acc_fd - fo.acc) <= 1e-3 * (1.0 + np.linalg.norm(fo.acc))
        assert np.linalg.norm(jerk_fd - fo.jerk) <= 1e-3 * (1.0 + np.linalg.norm(fo.jerk))


def test_continuity_at_knots():
    traj = _make_traj()
    eps = 1e-6
    for t_k in _knot_times()[1:-1]:
        lo = traj.eval(t_k - eps)
        hi = traj.eval(t_k + eps)
        assert np.linalg.norm(hi.pos - lo.pos) < 1e-4
        assert np.linalg.norm(hi.vel - lo.vel) < 1e-3
        assert np.linalg.norm(hi.acc - lo.acc) < 1e-2


def test_call_is_eval():
    traj = _make_traj()
    fo_call = traj(0.3)
    fo_eval = traj.eval(0.3)
    assert np.allclose(fo_call.pos, fo_eval.pos, atol=1e-14)
    assert np.allclose(fo_call.vel, fo_eval.vel, atol=1e-14)


def test_default_segment_times_and_T():
    traj = MinSnapTrajectory(WAYPOINTS, avg_speed=2.0)
    dists = np.linalg.norm(np.diff(WAYPOINTS, axis=0), axis=1)
    expected_T = float(np.sum(np.maximum(dists / 2.0, 0.5)))
    assert traj.T > 0.0
    assert np.isclose(traj.T, expected_T, atol=1e-9)


def test_two_waypoints():
    W2 = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
    traj = MinSnapTrajectory(W2)
    assert traj.T > 0.0
    assert np.linalg.norm(traj.eval(0.0).pos - W2[0]) <= 1e-5
    assert np.linalg.norm(traj.eval(traj.T).pos - W2[1]) <= 1e-5
    for t in (0.0, traj.T):
        fo = traj.eval(t)
        assert np.linalg.norm(fo.vel) < 1e-6
        assert np.linalg.norm(fo.acc) < 1e-6
        assert np.linalg.norm(fo.jerk) < 1e-6
    # eval clamps t to [0, T]
    assert np.allclose(traj.eval(-1.0).pos, traj.eval(0.0).pos, atol=1e-12)
    assert np.allclose(traj.eval(traj.T + 1.0).pos, traj.eval(traj.T).pos, atol=1e-12)
    # fixed yaw mode with default yaw_fixed
    assert traj.eval(0.5 * traj.T).yaw == 0.0
