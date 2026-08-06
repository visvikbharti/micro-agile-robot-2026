"""Export quadsim min-snap trajectories to the Crazyflie high-level 'poly4d' format.

The Crazyflie high-level commander consumes a trajectory as a list of segments,
each holding a duration and 8 polynomial coefficients (ascending powers of
UNSCALED segment-local time in seconds) for x, y, z and yaw -- the
cflib / uav_trajectory 'poly4d' CSV layout, one row per segment:

    duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7      (33 floats)

``quadsim.trajectory.MinSnapTrajectory`` stores its segment coefficients in
SCALED time ``tau = (t - t_i) / T_i``, so each coefficient converts as

    c_unscaled[k] = c_scaled[k] / T_i**k

This module is pure numpy: it needs neither cflib nor hardware.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

from quadsim.trajectory import MinSnapTrajectory

N_COEFF = 8          # 7th-order polynomials: 8 coefficients per axis per segment
ROW_LEN = 1 + 4 * N_COEFF   # duration + x, y, z, yaw coefficient blocks

MAX_SPEED = 3.0      # m/s: validate_rows sanity limit; min-snap peaks ~2x the average
MAX_ACCEL = 8.0      # m/s^2: validate_rows sanity limit (CF thrust margin, ex gravity)

_YAW_FIT_SAMPLES = 60       # yaw samples per segment used for the polynomial fit
_YAW_FIT_TOL = 5e-3         # rad: residual below which a lower-order fit is accepted

CSV_HEADER = ",".join(
    ["duration"]
    + ["%s^%d" % (axis, k) for axis in ("x", "y", "z", "yaw") for k in range(N_COEFF)]
)


def _yaw_segment_coeffs(traj: MinSnapTrajectory, i: int) -> np.ndarray:
    """Yaw coefficients (ascending powers of tau) for segment ``i`` of ``traj``.

    For ``yaw_mode == 'fixed'`` the yaw is the constant ``traj._yaw_fixed``:
    all coefficients are zero except the constant term. For ``'velocity'`` mode
    a low-order polynomial is least-squares fitted to dense samples of the
    trajectory's (unwrapped, precomputed) yaw over the segment: the lowest
    degree whose max residual is below 5e-3 rad is used, up to degree 7.
    """
    if traj._yaw_mode == "fixed":
        coeffs = np.zeros(N_COEFF)
        coeffs[0] = traj._yaw_fixed
        return coeffs

    t0, t1 = float(traj._knots[i]), float(traj._knots[i + 1])
    tau = np.linspace(0.0, 1.0, _YAW_FIT_SAMPLES)
    yaw = np.array([traj.eval(t0 + s * (t1 - t0)).yaw for s in tau])
    coeffs = np.zeros(N_COEFF)
    for deg in range(1, N_COEFF):
        fit = np.polynomial.polynomial.polyfit(tau, yaw, deg)
        if np.max(np.abs(np.polynomial.polynomial.polyval(tau, fit) - yaw)) < _YAW_FIT_TOL:
            break
    coeffs[: fit.size] = fit
    return coeffs


def to_poly4d(traj: MinSnapTrajectory, yaw_from_traj: bool = True) -> List[List[float]]:
    """Convert a ``MinSnapTrajectory`` to poly4d segment rows.

    Args:
        traj: the trajectory to export.
        yaw_from_traj: if True, export the trajectory's yaw (constant polynomial
            in 'fixed' mode, per-segment polynomial fit in 'velocity' mode);
            if False, all yaw coefficients are zero (yaw held at 0).

    Returns:
        List of ``n_seg`` rows, each ``[duration, x^0..x^7, y^0..y^7,
        z^0..z^7, yaw^0..yaw^7]`` with coefficients in ascending powers of
        UNSCALED segment-local time (seconds).
    """
    rows: List[List[float]] = []
    powers = np.arange(N_COEFF, dtype=float)
    for i, t_i in enumerate(traj._times):
        t_i = float(t_i)
        unscale = t_i ** powers                      # T_i**k, k = 0..7
        row = [t_i]
        for axis in range(3):                        # x, y, z
            row.extend((traj._coeffs[i, :, axis] / unscale).tolist())
        if yaw_from_traj:
            yaw_scaled = _yaw_segment_coeffs(traj, i)
        else:
            yaw_scaled = np.zeros(N_COEFF)
        row.extend((yaw_scaled / unscale).tolist())
        rows.append(row)
    return rows


def save_csv(rows: Sequence[Sequence[float]], path: str) -> None:
    """Write poly4d rows to ``path`` in the standard uav_trajectory CSV format.

    One header line, then one comma-separated line of 33 floats per segment.
    """
    with open(path, "w") as f:
        f.write(CSV_HEADER + "\n")
        for row in rows:
            if len(row) != ROW_LEN:
                raise ValueError("each row must have %d values, got %d" % (ROW_LEN, len(row)))
            f.write(",".join("%.17g" % float(v) for v in row) + "\n")


def load_csv(path: str) -> List[List[float]]:
    """Load poly4d rows from a uav_trajectory CSV (header line optional)."""
    data = np.loadtxt(path, delimiter=",", skiprows=_header_lines(path), ndmin=2)
    if data.shape[1] != ROW_LEN:
        raise ValueError("expected %d columns in %s, got %d" % (ROW_LEN, path, data.shape[1]))
    return data.tolist()


def _header_lines(path: str) -> int:
    """1 if the first non-empty line of ``path`` is a non-numeric header, else 0."""
    with open(path) as f:
        for line in f:
            if line.strip():
                try:
                    float(line.split(",")[0])
                    return 0
                except ValueError:
                    return 1
    return 0


def total_duration(rows: Sequence[Sequence[float]]) -> float:
    """Sum of segment durations in seconds."""
    return float(sum(row[0] for row in rows))


def peak_speed_accel(
    rows: Sequence[Sequence[float]], samples_per_seg: int = 64
) -> Tuple[float, float]:
    """Peak ``||velocity||`` and ``||acceleration||`` over the whole trajectory.

    Differentiates each segment's x/y/z polynomials analytically and samples
    ``samples_per_seg`` points per segment (endpoints included).
    """
    polyval = np.polynomial.polynomial.polyval
    peak_v = 0.0
    peak_a = 0.0
    k = np.arange(N_COEFF, dtype=float)
    for row in rows:
        row = np.asarray(row, dtype=float)
        s = np.linspace(0.0, row[0], samples_per_seg)
        v2 = np.zeros_like(s)
        a2 = np.zeros_like(s)
        for axis in range(3):
            c = row[1 + axis * N_COEFF: 1 + (axis + 1) * N_COEFF]
            vel_c = (c * k)[1:]                     # d/ds coefficients
            acc_c = (vel_c * k[: N_COEFF - 1])[1:]  # d2/ds2 coefficients
            v2 += polyval(s, vel_c) ** 2
            a2 += polyval(s, acc_c) ** 2
        peak_v = max(peak_v, float(np.sqrt(v2.max())))
        peak_a = max(peak_a, float(np.sqrt(a2.max())))
    return peak_v, peak_a


def validate_rows(
    rows: Sequence[Sequence[float]],
    max_speed: float = MAX_SPEED,
    max_accel: float = MAX_ACCEL,
) -> Tuple[float, float]:
    """Validate poly4d rows before they get anywhere near a Crazyflie.

    Checks row width, finite values, strictly positive durations, and that the
    peak speed/acceleration stay under ``max_speed``/``max_accel``.

    Returns:
        ``(peak_speed, peak_accel)`` in m/s and m/s^2.

    Raises:
        ValueError: on any violated check.
    """
    if not len(rows):
        raise ValueError("trajectory has no segments")
    for i, row in enumerate(rows):
        if len(row) != ROW_LEN:
            raise ValueError(
                "segment %d has %d values, expected %d" % (i, len(row), ROW_LEN))
        arr = np.asarray(row, dtype=float)
        if not np.all(np.isfinite(arr)):
            raise ValueError("segment %d contains non-finite values" % i)
        if arr[0] <= 0.0:
            raise ValueError("segment %d duration %.6g s is not positive" % (i, arr[0]))
    peak_v, peak_a = peak_speed_accel(rows)
    if peak_v > max_speed:
        raise ValueError(
            "peak speed %.2f m/s exceeds the %.2f m/s sanity limit" % (peak_v, max_speed))
    if peak_a > max_accel:
        raise ValueError(
            "peak accel %.2f m/s^2 exceeds the %.2f m/s^2 sanity limit" % (peak_a, max_accel))
    return peak_v, peak_a


def evaluate_poly4d(rows: Sequence[Sequence[float]], t: float) -> Tuple[np.ndarray, float]:
    """Evaluate exported poly4d rows at time ``t`` (clamped to the trajectory span).

    Returns:
        ``(pos, yaw)`` where ``pos`` is a ``(3,)`` array in meters and ``yaw``
        is in radians, exactly as the Crazyflie firmware would evaluate them.
    """
    if not rows:
        raise ValueError("rows is empty")
    durations = np.array([row[0] for row in rows], dtype=float)
    starts = np.concatenate(([0.0], np.cumsum(durations)))
    t = float(min(max(float(t), 0.0), starts[-1]))
    i = min(max(int(np.searchsorted(starts, t, side="right")) - 1, 0), len(rows) - 1)
    s = min(t - starts[i], durations[i])             # unscaled segment-local time
    row = np.asarray(rows[i], dtype=float)
    s_pow = s ** np.arange(N_COEFF)
    pos = np.array([
        float(row[1 + axis * N_COEFF: 1 + (axis + 1) * N_COEFF] @ s_pow)
        for axis in range(3)
    ])
    yaw = float(row[1 + 3 * N_COEFF: 1 + 4 * N_COEFF] @ s_pow)
    return pos, yaw


def _smoke_test() -> None:
    """Round-trip a 5-waypoint min-snap trajectory through the poly4d export."""
    import os
    import tempfile

    waypoints = np.array([
        [0.0, 0.0, 0.5],
        [0.8, 0.2, 0.7],
        [1.0, 1.0, 1.0],
        [0.2, 1.2, 0.7],
        [0.0, 0.0, 0.5],
    ])
    for yaw_mode, yaw_fixed in (("fixed", 0.3), ("velocity", 0.0)):
        traj = MinSnapTrajectory(waypoints, avg_speed=1.0,
                                 yaw_mode=yaw_mode, yaw_fixed=yaw_fixed)
        rows = to_poly4d(traj)
        assert len(rows) == len(traj._times)
        assert all(len(row) == ROW_LEN for row in rows)
        assert abs(total_duration(rows) - traj.T) < 1e-12

        t_grid = np.linspace(0.0, traj.T, 200)
        pos_err = 0.0
        yaw_err = 0.0
        for t in t_grid:
            pos, yaw = evaluate_poly4d(rows, t)
            ref = traj.eval(t)
            pos_err = max(pos_err, float(np.max(np.abs(pos - ref.pos))))
            yaw_err = max(yaw_err, abs(yaw - ref.yaw))
        print("yaw_mode=%-8s  max|pos err| = %.3e  max|yaw err| = %.3e"
              % (yaw_mode, pos_err, yaw_err))
        assert pos_err < 1e-6, "position export error %.3e >= 1e-6" % pos_err
        assert yaw_err < 0.05, "yaw export error %.3e >= 0.05" % yaw_err

        peak_v, peak_a = validate_rows(rows)
        assert 0.0 < peak_v <= MAX_SPEED and 0.0 < peak_a <= MAX_ACCEL

        fd, path = tempfile.mkstemp(suffix=".csv")
        os.close(fd)
        try:
            save_csv(rows, path)
            reloaded = load_csv(path)
            assert np.max(np.abs(np.asarray(reloaded) - np.asarray(rows))) < 1e-12
        finally:
            os.remove(path)

    bad = [list(r) for r in rows]
    bad[0][0] = -1.0
    for corrupt, what in ((bad, "negative duration"),
                          ([list(r)[:-1] for r in rows], "short row")):
        try:
            validate_rows(corrupt)
        except ValueError:
            pass
        else:
            raise AssertionError("validate_rows accepted a %s" % what)
    print("cf_trajectory smoke test passed")


if __name__ == "__main__":
    _smoke_test()
