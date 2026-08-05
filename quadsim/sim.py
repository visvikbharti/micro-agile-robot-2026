"""Closed-loop simulation driver: controller + dynamics + reference, recorded as a History."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from quadsim.dynamics import QuadrotorDynamics, QuadState
from quadsim.params import QuadParams


@dataclass
class History:
    """Recorded closed-loop time series (one row per control step).

    Attributes:
        t: (N,) sample times, s.
        p: (N,3) positions, world frame, m.
        v: (N,3) velocities, world frame, m/s.
        q: (N,4) attitude quaternions ``[w, x, y, z]``.
        w: (N,3) body-frame angular velocities, rad/s.
        f: (N,) actual (post-saturation) total thrusts, N.
        tau: (N,3) actual (post-saturation) body torques, N m.
        ref_p: (N,3) reference positions, m.
        ref_v: (N,3) reference velocities, m/s.
    """

    t: np.ndarray
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    w: np.ndarray
    f: np.ndarray
    tau: np.ndarray
    ref_p: np.ndarray
    ref_v: np.ndarray

    def pos_error(self) -> np.ndarray:
        """Return the (N,) Euclidean position tracking error ``||p - ref_p||`` per sample."""
        return np.linalg.norm(self.p - self.ref_p, axis=1)

    def rms_pos_error(self, t_from: float = 0.0) -> float:
        """Return the RMS of :meth:`pos_error` over all samples with ``t >= t_from``."""
        e = self.pos_error()
        mask = self.t >= t_from
        return float(np.sqrt(np.mean(e[mask] ** 2)))


def simulate(
    params: QuadParams,
    controller: Any,
    ref_fn: Callable[[float], Any],
    state0: QuadState,
    T: float,
    dt: float = 0.002,
) -> History:
    """Run a closed-loop simulation for duration ``T`` and return the recorded :class:`History`.

    Each step: ``(f, tau) = controller.compute(state, ref_fn, t)``; the SATURATED wrench actually
    applied is recorded (recomputed via the dynamics' ``unmix(mix(...))``); then the dynamics are
    stepped by ``dt``.

    Args:
        params: Quadrotor parameters.
        controller: Object with ``compute(state, ref_fn, t) -> (f, tau)`` (e.g. SE3Controller).
        ref_fn: Reference callable ``t -> FlatOutput`` (with ``.pos`` and ``.vel``).
        state0: Initial state.
        T: Total simulated time, s.
        dt: Control/integration time step, s.
    """
    dyn = QuadrotorDynamics(params)
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

    state = state0
    for k in range(n):
        t = k * dt
        f_cmd, tau_cmd = controller.compute(state, ref_fn, t)
        f_act, tau_act = dyn.unmix(dyn.mix(f_cmd, tau_cmd))
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
        state = dyn.step(state, f_cmd, tau_cmd, dt)

    return History(
        t=t_arr,
        p=p_arr,
        v=v_arr,
        q=q_arr,
        w=w_arr,
        f=f_arr,
        tau=tau_arr,
        ref_p=ref_p_arr,
        ref_v=ref_v_arr,
    )
