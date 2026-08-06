"""Sim-to-real correlation: score a logged Crazyflie flight against its reference.

Compares a position log from a real flight against either the poly4d trajectory
that was flown (the same CSV ``fly_minsnap.py --csv`` consumes) or a constant
hover setpoint, and reports the same style of metrics the simulator reports:
the RMS position error convention matches ``quadsim.sim.History.rms_pos_error``
(RMS of the per-sample Euclidean error norm). Intended as the automated
pass/fail gate in the flight-test plan: FT-1.x hovers use ``--hover``,
FT-2.x / FT-3.x trajectory flights use ``--ref``, and ``--max-rms`` turns the
result into an exit code a test script can check.

Expected flight-log CSV (header line required)::

    t,x,y,z          or          t,x,y,z,vbat

    t      time in seconds (any origin; see --t0 below)
    x,y,z  position in meters, world frame -- the Crazyflie's
           stateEstimate.x / stateEstimate.y / stateEstimate.z
    vbat   optional battery voltage in volts (pm.vbat)

Producing the log with cfclient: add a log configuration (Settings -> Logging
configurations) with variables ``stateEstimate.x``, ``stateEstimate.y``,
``stateEstimate.z`` and optionally ``pm.vbat``, period 100 ms or faster, start
it before takeoff, and save the resulting log as CSV. cfclient writes a
``Timestamp`` column in MILLISECONDS and column headers named after the log
variables; rename the columns to ``t,x,y,z(,vbat)`` and convert the timestamp
to seconds (``t = (ms - ms[0]) / 1000``) before running this tool. A scripted
flight can equally record the same variables through ``cflib``'s ``SyncLogger``
and write this format directly.

Time alignment: the log clock and the trajectory clock differ (logging starts
before ``start_trajectory``). ``--t0 OFFSET`` declares the log time at which
trajectory time zero occurs: every log sample is mapped to trajectory time
``t_traj = t - t0``, and the analysis is clipped to the overlap window
``0 <= t_traj <= total_duration(ref)`` (for ``--hover``, ``t_traj >= 0`` with
no upper clip). Samples outside the window -- takeoff, the go-to-start move,
landing -- are excluded from every metric. Pick ``t0`` from your flight
script's console output or by eye from the log; the reported RMS is a smooth
function of ``t0`` near the optimum, so +/-0.1 s of slop is visible but not
catastrophic.

Metric caveat, stated honestly: the "actual" position is the onboard Kalman
estimate (Flow deck v2 odometry), not ground truth. Flow-deck drift folds into
the reported error, so this measures estimate-vs-reference tracking, not
absolute-position accuracy. Without a motion-capture system that is the best
observable available.

Examples::

    python flight/analyze_log.py out/ft20_log.csv --ref out/traj.csv --t0 6.5
    python flight/analyze_log.py out/ft11_log.csv --hover 0 0 0.5 --t0 4.0
    python flight/analyze_log.py out/ft20_log.csv --ref out/traj.csv --t0 6.5 \
        --plot out/ft20_tracking.png --max-rms 0.15

Pure numpy + matplotlib (Agg): needs neither cflib nor hardware nor a display.
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cf_trajectory  # noqa: E402

REQUIRED_COLUMNS = ("t", "x", "y", "z")


# --- Log loading -----------------------------------------------------------------


def load_flight_log(path: str) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    """Load a flight-log CSV with header columns ``t,x,y,z`` and optional ``vbat``.

    Returns:
        ``(t, pos, vbat)`` where ``t`` is ``(N,)`` seconds, ``pos`` is ``(N, 3)``
        meters, and ``vbat`` is ``(N,)`` volts or None if the column is absent.

    Raises:
        ValueError: on an empty file, a missing/wrong header, fewer than 2
            samples, non-finite time, position, or vbat values, or a
            decreasing time column.
    """
    try:
        with warnings.catch_warnings():
            # Suppress genfromtxt's "Empty input file" UserWarning; the
            # ValueError below reports it cleanly.
            warnings.simplefilter("ignore")
            data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    except IndexError:
        # np.genfromtxt raises IndexError on a zero-byte/empty file; normalize
        # to ValueError so the CLI reports it like every other malformed log.
        raise ValueError("log %s is empty" % path)
    names = data.dtype.names
    if names is None or any(c not in names for c in REQUIRED_COLUMNS):
        raise ValueError(
            "log %s must have a header with columns t,x,y,z (got %r)" % (path, names))
    t = np.asarray(data["t"], dtype=float)
    pos = np.column_stack([data["x"], data["y"], data["z"]]).astype(float)
    vbat = np.asarray(data["vbat"], dtype=float) if "vbat" in names else None
    if t.size < 2:
        raise ValueError("log %s has %d sample(s); need at least 2" % (path, t.size))
    if not (np.all(np.isfinite(t)) and np.all(np.isfinite(pos))):
        raise ValueError("log %s contains non-finite t/x/y/z values" % path)
    if vbat is not None and not np.all(np.isfinite(vbat)):
        raise ValueError("log %s contains non-finite vbat values" % path)
    if np.any(np.diff(t) < 0.0):
        raise ValueError("log %s time column is not non-decreasing" % path)
    return t, pos, vbat


# --- Pure metric functions (numpy in, numbers out; no I/O) -----------------------


def overlap_mask(
    t_log: np.ndarray, t0: float, duration: Optional[float] = None
) -> np.ndarray:
    """Boolean mask of log samples inside the reference's time window.

    A sample at log time ``t`` maps to trajectory time ``t - t0``; it is kept
    when ``t - t0 >= 0`` and, if ``duration`` is given, ``t - t0 <= duration``.
    """
    t_traj = np.asarray(t_log, dtype=float) - float(t0)
    mask = t_traj >= 0.0
    if duration is not None:
        mask &= t_traj <= float(duration)
    return mask


def reference_positions(
    rows: Sequence[Sequence[float]], t_traj: np.ndarray
) -> np.ndarray:
    """(N, 3) reference positions: poly4d ``rows`` evaluated at each ``t_traj``.

    Uses ``cf_trajectory.evaluate_poly4d``, i.e. exactly what the Crazyflie
    firmware evaluates while flying the uploaded trajectory.
    """
    return np.array(
        [cf_trajectory.evaluate_poly4d(rows, float(t))[0] for t in t_traj]
    ).reshape(-1, 3)


def hover_reference(setpoint: Sequence[float], n: int) -> np.ndarray:
    """(n, 3) constant reference at ``setpoint`` (x, y, z in meters)."""
    return np.tile(np.asarray(setpoint, dtype=float).reshape(1, 3), (n, 1))


def position_metrics(pos: np.ndarray, ref_pos: np.ndarray) -> Dict[str, np.ndarray]:
    """Position tracking metrics over aligned ``pos`` and ``ref_pos`` (both (N, 3)).

    Returns a dict with:
        ``rms``: RMS of the per-sample Euclidean error norm (same convention as
            ``quadsim.sim.History.rms_pos_error``), m.
        ``max``: maximum error norm, m.
        ``axis_rms``: (3,) per-axis RMS error, m.
        ``final``: error norm at the last analyzed sample, m.
    """
    err = np.asarray(pos, dtype=float) - np.asarray(ref_pos, dtype=float)
    err_norm = np.linalg.norm(err, axis=1)
    return {
        "rms": float(np.sqrt(np.mean(err_norm ** 2))),
        "max": float(np.max(err_norm)),
        "axis_rms": np.sqrt(np.mean(err ** 2, axis=0)),
        "final": float(err_norm[-1]),
    }


def battery_metrics(vbat: np.ndarray) -> Dict[str, float]:
    """Battery stats over the analyzed window: mean, min, and total sag.

    ``sag`` is defined as ``vbat[0] - min(vbat)`` -- the drop from the voltage
    at the start of the analyzed window to the worst point under load, V.
    """
    vbat = np.asarray(vbat, dtype=float)
    return {
        "mean": float(np.mean(vbat)),
        "min": float(np.min(vbat)),
        "sag": float(vbat[0] - np.min(vbat)),
    }


# --- Reporting -------------------------------------------------------------------


def format_report(
    pm: Dict[str, np.ndarray],
    t_traj: np.ndarray,
    n_total: int,
    bm: Optional[Dict[str, float]] = None,
) -> str:
    """Render the metrics as an aligned two-column table (one string, no I/O)."""
    lines: List[str] = []

    def row(name: str, value: str) -> None:
        lines.append("%-26s %s" % (name, value))

    row("metric", "value")
    row("------", "-----")
    row("position RMS error", "%.4f m" % pm["rms"])
    row("max position error", "%.4f m" % pm["max"])
    row("per-axis RMS (x, y, z)",
        "%.4f / %.4f / %.4f m" % tuple(pm["axis_rms"]))
    row("final-position error", "%.4f m" % pm["final"])
    row("duration analyzed",
        "%.2f s  (%d of %d samples, traj t %.2f..%.2f s)"
        % (t_traj[-1] - t_traj[0], t_traj.size, n_total, t_traj[0], t_traj[-1]))
    if bm is not None:
        row("battery mean / min", "%.3f / %.3f V" % (bm["mean"], bm["min"]))
        row("battery sag (start-min)", "%.3f V" % bm["sag"])
    return "\n".join(lines)


def plot_comparison(
    t_traj: np.ndarray,
    pos: np.ndarray,
    ref_pos: np.ndarray,
    save: str,
    title: str = "",
) -> None:
    """Save a 4-panel reference-vs-actual PNG (x, y, z, error norm) to ``save``.

    Matches the ``quadsim.viz`` conventions: per-axis colors C0/C1/C2 with the
    reference dashed, ``tab:red`` error norm, light grids, Agg backend, 150 dpi,
    figure closed after saving. No display is required.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    err = np.linalg.norm(np.asarray(pos) - np.asarray(ref_pos), axis=1)
    fig, axs = plt.subplots(2, 2, figsize=(11.0, 7.5), sharex=True)
    for k, name in enumerate(("x", "y", "z")):
        ax = axs.flat[k]
        ax.plot(t_traj, pos[:, k], color="C%d" % k, linewidth=1.2, label="actual")
        ax.plot(t_traj, ref_pos[:, k], color="C%d" % k, linewidth=1.0,
                linestyle="--", label="reference")
        ax.set_ylabel("%s [m]" % name)
        ax.set_title("%s (reference dashed)" % name)
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)
    ax = axs.flat[3]
    ax.plot(t_traj, err, color="tab:red", linewidth=1.2)
    ax.set_ylabel("|p - ref_p| [m]")
    ax.set_title("position error norm")
    ax.grid(True, alpha=0.3)
    for ax in axs[1]:
        ax.set_xlabel("trajectory time t [s]")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    parent = os.path.dirname(save)
    if parent:
        os.makedirs(parent, exist_ok=True)
    fig.savefig(save, dpi=150)
    plt.close(fig)


# --- CLI -------------------------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare a logged Crazyflie flight against its reference "
                    "trajectory (sim-style metrics).")
    parser.add_argument("log", help="flight log CSV with columns t,x,y,z[,vbat]")
    parser.add_argument("--ref", default=None, metavar="POLY4D_CSV",
                        help="poly4d trajectory CSV that was flown "
                             "(the file fly_minsnap.py --csv consumes)")
    parser.add_argument("--hover", nargs=3, type=float, default=None,
                        metavar=("X", "Y", "Z"),
                        help="constant setpoint reference in meters "
                             "(for hover flights, e.g. FT-1.x)")
    parser.add_argument("--t0", type=float, default=0.0,
                        help="log time (s) at which trajectory time zero occurs; "
                             "analysis is clipped to the overlap window")
    parser.add_argument("--plot", default=None, metavar="OUT_PNG",
                        help="save a reference-vs-actual tracking figure here")
    parser.add_argument("--max-rms", type=float, default=None, metavar="M",
                        help="exit with code 1 if the position RMS error "
                             "exceeds this many meters (automated test gate)")
    args = parser.parse_args(argv)

    if (args.ref is None) == (args.hover is None):
        parser.error("exactly one of --ref or --hover is required")
    if args.hover is not None and not all(np.isfinite(v) for v in args.hover):
        parser.error("--hover values must be finite")

    try:
        t_log, pos, vbat = load_flight_log(args.log)
    except (OSError, ValueError) as exc:
        print("Invalid log: %s" % exc, file=sys.stderr)
        return 2

    if args.ref is not None:
        try:
            rows = cf_trajectory.load_csv(args.ref)
            cf_trajectory.validate_rows(rows)
        except (OSError, ValueError) as exc:
            print("Invalid reference: %s" % exc, file=sys.stderr)
            return 2
        duration = cf_trajectory.total_duration(rows)
        source = "poly4d %s (%d segments, %.2f s)" % (args.ref, len(rows), duration)
    else:
        rows = None
        duration = None
        source = "hover setpoint (%.2f, %.2f, %.2f) m" % tuple(args.hover)

    mask = overlap_mask(t_log, args.t0, duration)
    if int(np.count_nonzero(mask)) < 2:
        print("No overlap: fewer than 2 log samples fall in the reference window "
              "(check --t0; log spans t %.2f..%.2f s)" % (t_log[0], t_log[-1]),
              file=sys.stderr)
        return 2

    t_traj = t_log[mask] - args.t0
    if t_traj[-1] - t_traj[0] == 0.0:
        print("WARNING: analyzed window has zero duration (all analyzed "
              "timestamps are identical); metrics may be meaningless",
              file=sys.stderr)
    p = pos[mask]
    if rows is not None:
        ref_p = reference_positions(rows, t_traj)
    else:
        ref_p = hover_reference(args.hover, t_traj.size)

    pm = position_metrics(p, ref_p)
    bm = battery_metrics(vbat[mask]) if vbat is not None else None

    print("Log %s vs %s (t0 = %.2f s)" % (args.log, source, args.t0))
    print(format_report(pm, t_traj, t_log.size, bm))

    if args.plot is not None:
        plot_comparison(t_traj, p, ref_p, args.plot,
                        title="%s vs %s" % (os.path.basename(args.log), source))
        print("Plot saved to %s" % args.plot)

    # Fail closed: pass only if rms <= threshold, so a NaN RMS gates as FAIL
    # (nan > threshold is False, which would otherwise pass silently).
    if args.max_rms is not None and not pm["rms"] <= args.max_rms:
        print("FAIL: position RMS %.4f m exceeds --max-rms %.4f m"
              % (pm["rms"], args.max_rms), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
