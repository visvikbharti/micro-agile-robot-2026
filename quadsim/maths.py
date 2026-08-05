"""Rotation, quaternion, and skew-symmetric helpers for quadsim.

Conventions (see SPEC): quaternions are ``[w, x, y, z]`` (Hamilton, unit norm, body->world);
rotation matrices map body to world; angular velocity is expressed in the body frame.
"""
from __future__ import annotations

import numpy as np


def hat(v: np.ndarray) -> np.ndarray:
    """Return the (3,3) skew-symmetric matrix such that ``hat(v) @ u == np.cross(v, u)``."""
    v = np.asarray(v, dtype=float)
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def vee(M: np.ndarray) -> np.ndarray:
    """Return the (3,) inverse of :func:`hat`, averaging the off-diagonal entries of ``M``."""
    M = np.asarray(M, dtype=float)
    return 0.5 * np.array([
        M[2, 1] - M[1, 2],
        M[0, 2] - M[2, 0],
        M[1, 0] - M[0, 1],
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return ``q / ||q||`` as a (4,) unit quaternion (sign is kept as-is, even if w < 0)."""
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Return the (4,) Hamilton product ``q1 * q2`` of two ``[w, x, y, z]`` quaternions."""
    w1, x1, y1, z1 = np.asarray(q1, dtype=float)
    w2, x2, y2, z2 = np.asarray(q2, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Return the (3,3) rotation matrix (body->world) of a unit quaternion ``[w, x, y, z]``."""
    w, x, y, z = np.asarray(q, dtype=float)
    s = 2.0 / (w * w + x * x + y * y + z * z)
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
        [s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x)],
        [s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)],
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Return the (4,) unit quaternion ``[w, x, y, z]`` (with ``w >= 0``) of a rotation matrix.

    Uses Shepperd's method: branch on the largest of the trace and the diagonal entries so the
    division is always by a well-conditioned quantity.
    """
    R = np.asarray(R, dtype=float)
    d = np.diag(R)
    tr = d.sum()
    branch = int(np.argmax([tr, d[0], d[1], d[2]]))
    if branch == 0:
        s = 2.0 * np.sqrt(1.0 + tr)
        q = np.array([
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ])
    elif branch == 1:
        s = 2.0 * np.sqrt(1.0 + d[0] - d[1] - d[2])
        q = np.array([
            (R[2, 1] - R[1, 2]) / s,
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        ])
    elif branch == 2:
        s = 2.0 * np.sqrt(1.0 - d[0] + d[1] - d[2])
        q = np.array([
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
        ])
    else:
        s = 2.0 * np.sqrt(1.0 - d[0] - d[1] + d[2])
        q = np.array([
            (R[1, 0] - R[0, 1]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
        ])
    q = q / np.linalg.norm(q)
    if q[0] < 0.0:
        q = -q
    return q


def quat_derivative(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Return the (4,) quaternion time derivative ``0.5 * q * [0, wx, wy, wz]``.

    ``w`` is the angular velocity expressed in the body frame.
    """
    w = np.asarray(w, dtype=float)
    return 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))


def rot_log(R: np.ndarray) -> np.ndarray:
    """Return the (3,) axis-angle vector ``theta * axis`` (vee of the matrix log) of ``R``.

    Numerically robust near angle 0 and near pi: the rotation is first converted to a unit
    quaternion via Shepperd's method, then ``theta = 2 * atan2(||xyz||, w)`` with the returned
    angle in ``[0, pi]``.
    """
    q = rot_to_quat(R)
    xyz = q[1:4]
    n = np.linalg.norm(xyz)
    if n < 1e-12:
        return 2.0 * xyz
    return (2.0 * np.arctan2(n, q[0]) / n) * xyz
