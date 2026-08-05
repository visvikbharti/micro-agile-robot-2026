"""Tests for quadsim.maths: rotation/quaternion utilities."""
from __future__ import annotations

import numpy as np
import scipy.linalg

from quadsim.maths import (
    hat,
    vee,
    quat_normalize,
    quat_mul,
    quat_to_rot,
    rot_to_quat,
    quat_derivative,
    rot_log,
)


def _random_unit_quats(rng: np.random.Generator, n: int) -> np.ndarray:
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def test_hat_vee_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        v = rng.normal(size=3)
        u = rng.normal(size=3)
        M = hat(v)
        assert M.shape == (3, 3)
        assert np.allclose(M, -M.T, atol=1e-14)
        assert np.allclose(M @ u, np.cross(v, u), atol=1e-12)
        assert np.allclose(vee(M), v, atol=1e-12)


def test_quat_mul_matches_rotation_composition():
    rng = np.random.default_rng(0)
    for q1, q2 in zip(_random_unit_quats(rng, 20), _random_unit_quats(rng, 20)):
        R_prod = quat_to_rot(quat_mul(q1, q2))
        R_comp = quat_to_rot(q1) @ quat_to_rot(q2)
        assert np.allclose(R_prod, R_comp, atol=1e-12)


def test_quat_to_rot_orthonormal():
    rng = np.random.default_rng(0)
    for q in _random_unit_quats(rng, 20):
        R = quat_to_rot(q)
        assert R.shape == (3, 3)
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-12)


def test_rot_to_quat_roundtrip():
    rng = np.random.default_rng(0)
    for q in _random_unit_quats(rng, 30):
        R = quat_to_rot(q)
        q2 = rot_to_quat(R)
        assert q2[0] >= 0.0
        assert np.isclose(np.linalg.norm(q2), 1.0, atol=1e-12)
        assert np.allclose(quat_to_rot(q2), R, atol=1e-10)


def test_quat_normalize():
    rng = np.random.default_rng(0)
    for _ in range(10):
        q = rng.normal(size=4) * 3.0
        qn = quat_normalize(q)
        assert np.isclose(np.linalg.norm(qn), 1.0, atol=1e-12)
        assert np.allclose(qn, q / np.linalg.norm(q), atol=1e-12)
    # w < 0 keeps its sign as-is
    qn = quat_normalize(np.array([-2.0, 0.0, 0.0, 0.0]))
    assert np.allclose(qn, np.array([-1.0, 0.0, 0.0, 0.0]), atol=1e-14)


def test_quat_derivative():
    rng = np.random.default_rng(0)
    for q in _random_unit_quats(rng, 10):
        w = rng.normal(size=3)
        qd = quat_derivative(q, w)
        expected = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
        assert np.allclose(qd, expected, atol=1e-12)


def test_rot_log_small_angles():
    rng = np.random.default_rng(0)
    for norm in (1e-8, 1e-5, 1e-3, 0.3, 1.0):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        v = norm * axis
        R = scipy.linalg.expm(hat(v))
        assert np.allclose(rot_log(R), v, rtol=1e-6, atol=1e-9)


def test_rot_log_near_pi():
    rng = np.random.default_rng(0)
    for delta in (1e-3, 1e-5):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        v = (np.pi - delta) * axis
        R = scipy.linalg.expm(hat(v))
        assert np.allclose(rot_log(R), v, atol=1e-6)


def test_rot_log_identity():
    assert np.allclose(rot_log(np.eye(3)), np.zeros(3), atol=1e-12)
