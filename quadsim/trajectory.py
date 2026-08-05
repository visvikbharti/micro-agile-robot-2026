"""Flat-output reference trajectories: hover and piecewise minimum-snap polynomials.

Implements the minimum-snap trajectory generation of Mellinger & Kumar (ICRA 2011):
piecewise 7th-order polynomials per axis through 3D waypoints, each segment expressed
in scaled time ``tau = (t - t_i) / T_i`` for numerical conditioning, solved as an
equality-constrained QP via its KKT system.

Conventions: world frame with z up, SI units, numpy float64, vectors shape ``(3,)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

_N_COEFF = 8            # 7th-order polynomial -> 8 coefficients per segment per axis
_YAW_SPEED_MIN = 0.15   # m/s: horizontal speed below which atan2(vy, vx) is unreliable
_YAW_GRID_HZ = 500.0    # Hz: density of the precomputed yaw grid


@dataclass
class FlatOutput:
    """Differentially flat output of a quadrotor reference at one time instant.

    Positions/derivatives are world-frame ``(3,)`` arrays in SI units; ``yaw`` is the
    heading angle about world z in rad and ``yaw_rate`` its time derivative in rad/s.
    """

    pos: np.ndarray   # (3,)
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))   # (3,)
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))   # (3,)
    jerk: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)
    snap: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)
    yaw: float = 0.0
    yaw_rate: float = 0.0


def hover_ref(p: np.ndarray, yaw: float = 0.0) -> Callable[[float], FlatOutput]:
    """Return a constant hover reference function ``t -> FlatOutput`` at position ``p``.

    All derivatives are zero; ``yaw`` is held fixed.
    """
    p0 = np.asarray(p, dtype=float).copy()
    yaw0 = float(yaw)

    def ref(t: float) -> FlatOutput:
        return FlatOutput(pos=p0.copy(), yaw=yaw0)

    return ref


def _falling(j: int, k: int) -> float:
    """Falling factorial ``j * (j - 1) * ... * (j - k + 1)`` (equals 1 for k == 0)."""
    out = 1.0
    for m in range(j, j - k, -1):
        out *= m
    return out


def _basis_row(k: int, tau: float) -> np.ndarray:
    """Row ``r`` with ``r @ c`` = k-th tau-derivative of ``sum_j c_j tau**j`` (8 coeffs)."""
    row = np.zeros(_N_COEFF)
    for j in range(k, _N_COEFF):
        row[j] = _falling(j, k) * tau ** (j - k)
    return row


class MinSnapTrajectory:
    """Piecewise 7th-order minimum-snap trajectory through 3D waypoints.

    Each segment i is a 7th-order polynomial per axis in scaled time
    ``tau = (t - t_i) / T_i in [0, 1]``. The coefficients minimize
    ``sum_i T_i**-7 * integral_0^1 ||d4p/dtau4||^2 dtau`` subject to waypoint positions,
    zero vel/acc/jerk at the two trajectory endpoints, and continuity of physical-time
    derivatives 1..4 at interior knots, solved per axis via the KKT system
    ``[[2H, A^T], [A, 0]] [c; lam] = [0; b]``.

    A trajectory instance is callable (``__call__ = eval``), so it is a valid ``ref_fn``.
    """

    def __init__(
        self,
        waypoints: np.ndarray,
        avg_speed: float = 2.0,
        segment_times: Optional[np.ndarray] = None,
        yaw_mode: str = "fixed",
        yaw_fixed: float = 0.0,
    ) -> None:
        """Build the trajectory.

        Args:
            waypoints: ``(M, 3)`` array, M >= 2; consecutive duplicates are invalid.
            avg_speed: m/s used for default segment times ``max(dist_i / avg_speed, 0.5)``.
            segment_times: optional ``(M - 1,)`` positive durations overriding the default.
            yaw_mode: ``"fixed"`` holds ``yaw_fixed``; ``"velocity"`` follows
                ``atan2(vy, vx)`` where the horizontal speed is at least 0.15 m/s,
                holding the nearest valid value across low-speed spans.
            yaw_fixed: rad, the yaw used in ``"fixed"`` mode.

        Raises:
            ValueError: on invalid inputs or if the waypoint fit error is >= 1e-6.
        """
        wp = np.asarray(waypoints, dtype=float)
        if wp.ndim != 2 or wp.shape[1] != 3 or wp.shape[0] < 2:
            raise ValueError("waypoints must be an (M, 3) array with M >= 2")
        dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
        if np.any(dists < 1e-12):
            raise ValueError("consecutive duplicate waypoints are invalid")
        n_seg = wp.shape[0] - 1
        if segment_times is None:
            if avg_speed <= 0.0:
                raise ValueError("avg_speed must be positive")
            times = np.maximum(dists / float(avg_speed), 0.5)
        else:
            times = np.asarray(segment_times, dtype=float)
            if times.shape != (n_seg,):
                raise ValueError("segment_times must have shape (M - 1,)")
            if np.any(times <= 0.0):
                raise ValueError("segment_times must be positive")
        if yaw_mode not in ("fixed", "velocity"):
            raise ValueError("yaw_mode must be 'fixed' or 'velocity'")
        self._wp = wp
        self._times = times
        self._knots = np.concatenate(([0.0], np.cumsum(times)))
        self._yaw_mode = yaw_mode
        self._yaw_fixed = float(yaw_fixed)
        self._coeffs = self._solve_coefficients()
        fit_err = max(
            float(np.linalg.norm(self._derivatives(self._knots[j])[0] - wp[j]))
            for j in range(wp.shape[0])
        )
        if fit_err >= 1e-6:
            raise ValueError("min-snap solve failed: waypoint fit error %.3e" % fit_err)
        if yaw_mode == "velocity":
            self._build_yaw_grid()

    @property
    def T(self) -> float:
        """Total trajectory duration in seconds."""
        return float(self._knots[-1])

    def eval(self, t: float) -> FlatOutput:
        """Evaluate the flat output at time ``t``, clamped to ``[0, T]``.

        Derivatives are with respect to physical time (``d^k/dt^k = T_i**-k d^k/dtau^k``).
        """
        t = float(min(max(float(t), 0.0), self.T))
        pos, vel, acc, jerk, snap = self._derivatives(t)
        if self._yaw_mode == "fixed":
            yaw = self._yaw_fixed
            yaw_rate = 0.0
        else:
            yaw = float(np.interp(t, self._yaw_t, self._yaw_vals))
            yaw_rate = float(np.interp(t, self._yaw_t, self._yaw_rates))
        return FlatOutput(pos=pos, vel=vel, acc=acc, jerk=jerk, snap=snap,
                          yaw=yaw, yaw_rate=yaw_rate)

    __call__ = eval

    def _solve_coefficients(self) -> np.ndarray:
        """Solve the per-axis KKT systems; return coefficients shaped (n_seg, 8, 3)."""
        n_seg = int(self._times.size)
        n_c = _N_COEFF * n_seg
        snap_block = np.zeros((_N_COEFF, _N_COEFF))
        for j in range(4, _N_COEFF):
            for k in range(4, _N_COEFF):
                snap_block[j, k] = _falling(j, 4) * _falling(k, 4) / (j + k - 7)
        hess = np.zeros((n_c, n_c))
        for i, t_i in enumerate(self._times):
            sl = slice(i * _N_COEFF, (i + 1) * _N_COEFF)
            hess[sl, sl] = snap_block * t_i ** (-7)
        rows: List[np.ndarray] = []
        rhs: List[np.ndarray] = []
        zero3 = np.zeros(3)
        for i in range(n_seg):
            row = np.zeros(n_c)
            row[i * _N_COEFF:(i + 1) * _N_COEFF] = _basis_row(0, 0.0)
            rows.append(row)
            rhs.append(self._wp[i])
            row = np.zeros(n_c)
            row[i * _N_COEFF:(i + 1) * _N_COEFF] = _basis_row(0, 1.0)
            rows.append(row)
            rhs.append(self._wp[i + 1])
        for k in (1, 2, 3):
            row = np.zeros(n_c)
            row[:_N_COEFF] = _basis_row(k, 0.0) * self._times[0] ** (-k)
            rows.append(row)
            rhs.append(zero3)
            row = np.zeros(n_c)
            row[-_N_COEFF:] = _basis_row(k, 1.0) * self._times[-1] ** (-k)
            rows.append(row)
            rhs.append(zero3)
        for i in range(n_seg - 1):
            for k in (1, 2, 3, 4):
                row = np.zeros(n_c)
                row[i * _N_COEFF:(i + 1) * _N_COEFF] = (
                    _basis_row(k, 1.0) * self._times[i] ** (-k))
                row[(i + 1) * _N_COEFF:(i + 2) * _N_COEFF] = (
                    -_basis_row(k, 0.0) * self._times[i + 1] ** (-k))
                rows.append(row)
                rhs.append(zero3)
        a_mat = np.vstack(rows)
        b_mat = np.vstack(rhs)
        n_con = a_mat.shape[0]
        kkt = np.zeros((n_c + n_con, n_c + n_con))
        kkt[:n_c, :n_c] = 2.0 * hess
        kkt[:n_c, n_c:] = a_mat.T
        kkt[n_c:, :n_c] = a_mat
        kkt_rhs = np.zeros((n_c + n_con, 3))
        kkt_rhs[n_c:] = b_mat
        try:
            sol = np.linalg.solve(kkt, kkt_rhs)
        except np.linalg.LinAlgError:
            sol = np.linalg.lstsq(kkt, kkt_rhs, rcond=None)[0]
        return sol[:n_c].reshape(n_seg, _N_COEFF, 3)

    def _segment_index(self, t: float) -> int:
        """Index of the segment containing time ``t`` (t assumed already in [0, T])."""
        i = int(np.searchsorted(self._knots, t, side="right")) - 1
        return min(max(i, 0), int(self._times.size) - 1)

    def _derivatives(self, t: float) -> List[np.ndarray]:
        """Position and physical-time derivatives 1..4 at ``t`` as five ``(3,)`` arrays."""
        t = float(min(max(float(t), 0.0), self.T))
        i = self._segment_index(t)
        t_i = float(self._times[i])
        tau = min(max((t - self._knots[i]) / t_i, 0.0), 1.0)
        coeffs = self._coeffs[i]
        return [(_basis_row(k, tau) @ coeffs) * t_i ** (-k) for k in range(5)]

    def _build_yaw_grid(self) -> None:
        """Precompute the dense yaw/yaw-rate grid for ``yaw_mode='velocity'``."""
        n = max(int(np.ceil(self.T * _YAW_GRID_HZ)) + 1, 2)
        t_grid = np.linspace(0.0, self.T, n)
        vel = np.array([self._derivatives(t)[1] for t in t_grid])
        speed_xy = np.hypot(vel[:, 0], vel[:, 1])
        valid = speed_xy >= _YAW_SPEED_MIN
        if not np.any(valid):
            yaw_raw = np.full(n, self._yaw_fixed)
        else:
            idx_valid = np.nonzero(valid)[0]
            yaw_valid = np.arctan2(vel[idx_valid, 1], vel[idx_valid, 0])
            all_idx = np.arange(n)
            left = np.clip(np.searchsorted(idx_valid, all_idx, side="right") - 1,
                           0, idx_valid.size - 1)
            right = np.clip(left + 1, 0, idx_valid.size - 1)
            use_right = np.abs(idx_valid[right] - all_idx) < np.abs(all_idx - idx_valid[left])
            yaw_raw = yaw_valid[np.where(use_right, right, left)]
        self._yaw_t = t_grid
        self._yaw_vals = np.unwrap(yaw_raw)
        self._yaw_rates = np.gradient(self._yaw_vals, t_grid)
