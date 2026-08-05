"""Rigid-body quadrotor dynamics with an X-configuration motor mixer and RK4 integration."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from quadsim.maths import quat_derivative, quat_normalize, quat_to_rot
from quadsim.params import QuadParams

_E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class QuadState:
    """Full quadrotor state.

    Attributes:
        p: (3,) position in the world frame, m.
        v: (3,) velocity in the world frame, m/s.
        q: (4,) attitude quaternion ``[w, x, y, z]`` (body->world, unit norm).
        w: (3,) angular velocity in the body frame, rad/s.
    """

    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    w: np.ndarray

    @property
    def R(self) -> np.ndarray:
        """(3,3) rotation matrix (body->world) of the attitude quaternion."""
        return quat_to_rot(self.q)

    @classmethod
    def hover(cls, p: np.ndarray, yaw: float = 0.0) -> "QuadState":
        """Return a state at rest at position ``p``, rotated by ``yaw`` about the world z-axis."""
        q = np.array([np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)])
        return cls(
            p=np.asarray(p, dtype=float).copy(),
            v=np.zeros(3),
            q=q,
            w=np.zeros(3),
        )

    def as_vector(self) -> np.ndarray:
        """Return the (13,) state vector ``[p, v, q, w]``."""
        return np.concatenate([self.p, self.v, self.q, self.w])

    @classmethod
    def from_vector(cls, x: np.ndarray) -> "QuadState":
        """Build a state from a (13,) vector in the order ``[p, v, q, w]``."""
        x = np.asarray(x, dtype=float)
        return cls(p=x[0:3].copy(), v=x[3:6].copy(), q=x[6:10].copy(), w=x[10:13].copy())


class QuadrotorDynamics:
    """Quadrotor rigid-body dynamics with motor mixing, saturation, and RK4 stepping.

    Mixer geometry (X configuration) with ``d = L / sqrt(2)`` and ``c = c_tau``: rotor positions
    in the body frame are ``r1=(+d,+d,0)``, ``r2=(-d,+d,0)``, ``r3=(-d,-d,0)``, ``r4=(+d,-d,0)``;
    rotors 1 and 3 spin CCW (reaction torque -z per unit thrust), rotors 2 and 4 spin CW (+z).
    The allocation matrix ``A`` maps motor thrusts to ``u = [f, tau_x, tau_y, tau_z]``.
    """

    def __init__(self, params: QuadParams):
        """Store parameters and precompute the allocation matrix, its inverse, and J^-1."""
        self.params = params
        d = params.L / np.sqrt(2.0)
        c = params.c_tau
        self.A = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [d, d, -d, -d],
            [-d, d, d, -d],
            [-c, c, -c, c],
        ])
        self._A_inv = np.linalg.inv(self.A)
        self._J = np.asarray(params.J, dtype=float)
        self._J_inv = np.linalg.inv(self._J)

    def mix(self, f: float, tau: np.ndarray) -> np.ndarray:
        """Return (4,) motor thrusts ``A^-1 @ [f, tau]`` clipped to the per-motor limits."""
        tau = np.asarray(tau, dtype=float)
        u = np.array([f, tau[0], tau[1], tau[2]])
        motors = self._A_inv @ u
        return np.clip(motors, self.params.f_motor_min, self.params.f_motor_max)

    def unmix(self, motors: np.ndarray) -> Tuple[float, np.ndarray]:
        """Return ``(f, tau)`` = ``A @ motors`` as a float and a (3,) array."""
        u = self.A @ np.asarray(motors, dtype=float)
        return float(u[0]), u[1:4].copy()

    def derivative(self, x: np.ndarray, f: float, tau: np.ndarray) -> np.ndarray:
        """Return the (13,) time derivative of the state vector ``x`` = ``[p, v, q, w]``.

        ``p' = v``; ``v' = -g e3 + (f/m) R e3``; ``q' = quat_derivative(q, w)``;
        ``w' = J^-1 (tau - w x (J w))``.
        """
        x = np.asarray(x, dtype=float)
        tau = np.asarray(tau, dtype=float)
        v = x[3:6]
        q = x[6:10]
        w = x[10:13]
        R = quat_to_rot(q)
        v_dot = -self.params.g * _E3 + (f / self.params.m) * (R @ _E3)
        q_dot = quat_derivative(q, w)
        w_dot = self._J_inv @ (tau - np.cross(w, self._J @ w))
        return np.concatenate([v, v_dot, q_dot, w_dot])

    def step(self, state: QuadState, f: float, tau: np.ndarray, dt: float) -> QuadState:
        """Advance the state by ``dt`` using classic RK4 with zero-order-hold actuation.

        Actuator saturation is applied ONCE via ``unmix(mix(f, tau))``; the saturated wrench is
        held constant over the step. The quaternion is renormalized after integration.
        """
        f_act, tau_act = self.unmix(self.mix(f, tau))
        x = state.as_vector()
        k1 = self.derivative(x, f_act, tau_act)
        k2 = self.derivative(x + 0.5 * dt * k1, f_act, tau_act)
        k3 = self.derivative(x + 0.5 * dt * k2, f_act, tau_act)
        k4 = self.derivative(x + dt * k3, f_act, tau_act)
        x_new = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        x_new[6:10] = quat_normalize(x_new[6:10])
        return QuadState.from_vector(x_new)
