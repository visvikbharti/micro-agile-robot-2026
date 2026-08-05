"""Geometric SE(3) tracking controller (Lee, Leok, McClamroch, CDC 2010).

Computes thrust and body torque to track a differentially flat reference
(:class:`quadsim.trajectory.FlatOutput`), with attitude feedforward built by
finite-differencing the commanded rotation as a pure function of time.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Optional, Tuple

import numpy as np

from quadsim.maths import hat, rot_log, vee
from quadsim.params import QuadParams
from quadsim.trajectory import FlatOutput

if TYPE_CHECKING:
    from quadsim.dynamics import QuadState


def flat_to_rotation(F: np.ndarray, yaw: float) -> np.ndarray:
    """Desired rotation matrix (body->world) from a desired world force and a yaw angle.

    The body z-axis is aligned with ``F``; the body x-axis is placed as close as
    possible to the heading ``[cos(yaw), sin(yaw), 0]``.

    Args:
        F: (3,) desired force vector in the world frame.
        yaw: desired yaw angle in rad.

    Returns:
        (3,3) rotation matrix ``R = [b1, b2, b3]`` with ``b3`` parallel to ``F``.
    """
    F = np.asarray(F, dtype=float)
    norm_F = float(np.linalg.norm(F))
    if norm_F < 1e-6:
        b3 = np.array([0.0, 0.0, 1.0])
    else:
        b3 = F / norm_F
    b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    c = np.cross(b3, b1c)
    if np.linalg.norm(c) < 1e-6:
        b1c = np.array([np.cos(yaw + np.pi / 2), np.sin(yaw + np.pi / 2), 0.0])
        c = np.cross(b3, b1c)
    b2 = c / np.linalg.norm(c)
    b1 = np.cross(b2, b3)
    return np.column_stack([b1, b2, b3])


class SE3Controller:
    """Geometric tracking controller on SE(3) with finite-difference attitude feedforward."""

    def __init__(
        self,
        params: QuadParams,
        kp: Optional[np.ndarray] = None,
        kv: Optional[np.ndarray] = None,
        kR: Optional[np.ndarray] = None,
        kw: Optional[np.ndarray] = None,
    ) -> None:
        """Create the controller.

        Args:
            params: quadrotor physical parameters.
            kp: (3,) position gains; default ``m * [16, 16, 16]``.
            kv: (3,) velocity gains; default ``m * [8, 8, 8]``.
            kR: (3,) attitude gains; default ``[1000, 1000, 100]``.
            kw: (3,) body-rate gains; default ``[63, 63, 20]``.
        """
        self.params = params
        m = params.m
        self.kp = m * np.array([16.0, 16.0, 16.0]) if kp is None else np.asarray(kp, dtype=float)
        self.kv = m * np.array([8.0, 8.0, 8.0]) if kv is None else np.asarray(kv, dtype=float)
        self.kR = np.array([1000.0, 1000.0, 100.0]) if kR is None else np.asarray(kR, dtype=float)
        self.kw = np.array([63.0, 63.0, 20.0]) if kw is None else np.asarray(kw, dtype=float)

    def compute(
        self,
        state: "QuadState",
        ref_fn: Callable[[float], FlatOutput],
        t: float,
        fd_dt: float = 1e-3,
    ) -> Tuple[float, np.ndarray]:
        """Compute total thrust and body torque for the current state and reference.

        Args:
            state: current quadrotor state (uses ``p``, ``v``, ``R``, ``w``).
            ref_fn: reference trajectory, a callable ``t -> FlatOutput``.
            t: current time in s.
            fd_dt: finite-difference step for the attitude feedforward, in s.

        Returns:
            ``(f, tau)``: total thrust (N, >= 0) and (3,) body torque (N m).
            Actuator saturation is applied by the dynamics, not here.
        """
        m = self.params.m
        g = self.params.g
        J = self.params.J
        e3 = np.array([0.0, 0.0, 1.0])
        R = state.R
        w = state.w

        ref = ref_fn(t)
        e_p = state.p - ref.pos
        e_v = state.v - ref.vel
        F_des = -self.kp * e_p - self.kv * e_v + m * g * e3 + m * ref.acc
        f = max(0.0, float(F_des @ (R @ e3)))
        R_cmd = flat_to_rotation(F_des, ref.yaw)

        def R_ff(s: float) -> np.ndarray:
            r = ref_fn(s)
            return flat_to_rotation(m * r.acc + m * g * e3, r.yaw)

        d = fd_dt
        R_ff_0 = R_ff(t)
        w_ff = rot_log(R_ff_0.T @ R_ff(t + d)) / d
        w_ff_m = rot_log(R_ff(t - d).T @ R_ff_0) / d
        a_ff = (w_ff - w_ff_m) / d

        e_R = 0.5 * vee(R_cmd.T @ R - R.T @ R_cmd)
        e_w = w - R.T @ R_cmd @ w_ff
        tau = (
            J @ (-self.kR * e_R - self.kw * e_w)
            + np.cross(w, J @ w)
            - J @ (hat(w) @ R.T @ R_cmd @ w_ff - R.T @ R_cmd @ a_ff)
        )
        return f, tau
