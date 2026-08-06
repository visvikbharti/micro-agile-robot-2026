"""Tests for flight/analyze_log.py: sim-to-real correlation metrics and CLI gate.

Round-trip strategy: build a small MinSnapTrajectory, export it with
``cf_trajectory.to_poly4d`` / ``save_csv``, sample the reloaded rows with
``evaluate_poly4d`` into a synthetic "perfect log" (RMS must be ~0), then add a
known constant offset (RMS must equal the offset norm), and exercise --t0
alignment, overlap clipping, and the --max-rms exit-code gate through main().
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "flight"),
)
import analyze_log  # noqa: E402
import cf_trajectory  # noqa: E402

from quadsim.trajectory import MinSnapTrajectory  # noqa: E402

WAYPOINTS = np.array(
    [
        [0.0, 0.0, 0.5],
        [0.6, 0.0, 0.6],
        [0.6, 0.6, 0.5],
        [0.0, 0.0, 0.5],
    ]
)
OFFSET = np.array([0.02, -0.03, 0.05])


def _rows_and_csv(tmp_path):
    """Export a small min-snap trajectory to a poly4d CSV; return (rows, path)."""
    traj = MinSnapTrajectory(WAYPOINTS, avg_speed=0.5, yaw_mode="fixed", yaw_fixed=0.0)
    ref_path = str(tmp_path / "ref.csv")
    cf_trajectory.save_csv(cf_trajectory.to_poly4d(traj), ref_path)
    # Use the RELOADED rows everywhere, exactly as the CLI does via --ref.
    return cf_trajectory.load_csv(ref_path), ref_path


def _perfect_log(rows, dt=0.02, t_shift=0.0):
    """Sample poly4d rows into a fake perfect log: (t_log, pos).

    Samples strictly inside [0, duration) so every sample survives overlap
    clipping even after the float round-trip of the --t0 shift.
    """
    duration = cf_trajectory.total_duration(rows)
    t_traj = np.arange(0.0, duration, dt)
    pos = analyze_log.reference_positions(rows, t_traj)
    return t_traj + t_shift, pos


def _write_log(path, t, pos, vbat=None):
    cols = [t, pos[:, 0], pos[:, 1], pos[:, 2]]
    header = "t,x,y,z"
    if vbat is not None:
        cols.append(vbat)
        header += ",vbat"
    np.savetxt(str(path), np.column_stack(cols), delimiter=",",
               header=header, comments="", fmt="%.18e")


def test_perfect_log_zero_rms(tmp_path):
    rows, _ = _rows_and_csv(tmp_path)
    t_log, pos = _perfect_log(rows)
    pm = analyze_log.position_metrics(pos, analyze_log.reference_positions(rows, t_log))
    assert pm["rms"] < 1e-9
    assert pm["max"] < 1e-9
    assert pm["final"] < 1e-9
    assert np.all(pm["axis_rms"] < 1e-9)


def test_constant_offset_recovered(tmp_path):
    rows, _ = _rows_and_csv(tmp_path)
    t_log, pos = _perfect_log(rows)
    ref = analyze_log.reference_positions(rows, t_log)
    pm = analyze_log.position_metrics(pos + OFFSET, ref)
    expected = float(np.linalg.norm(OFFSET))
    assert abs(pm["rms"] - expected) < 1e-9
    assert abs(pm["max"] - expected) < 1e-9
    assert abs(pm["final"] - expected) < 1e-9
    assert np.allclose(pm["axis_rms"], np.abs(OFFSET), atol=1e-9)


def test_t0_alignment_and_overlap_clipping(tmp_path):
    rows, _ = _rows_and_csv(tmp_path)
    duration = cf_trajectory.total_duration(rows)
    t0 = 1.5
    t_log, pos = _perfect_log(rows, t_shift=t0)
    # Prepend/append junk samples outside the trajectory window (takeoff/landing).
    t_junk_pre = np.array([0.0, 0.7])
    t_junk_post = t_log[-1] + np.array([0.5, 1.0])
    junk = np.full((2, 3), 9.9)
    t_all = np.concatenate([t_junk_pre, t_log, t_junk_post])
    pos_all = np.vstack([junk, pos, junk])

    mask = analyze_log.overlap_mask(t_all, t0, duration)
    assert int(np.count_nonzero(mask)) == t_log.size  # junk excluded, log kept
    t_traj = t_all[mask] - t0
    pm = analyze_log.position_metrics(
        pos_all[mask], analyze_log.reference_positions(rows, t_traj))
    assert pm["rms"] < 1e-9

    # Wrong t0 must NOT be near zero: the reference is evaluated at shifted times.
    mask_bad = analyze_log.overlap_mask(t_all, 0.0, duration)
    t_bad = t_all[mask_bad]
    pm_bad = analyze_log.position_metrics(
        pos_all[mask_bad], analyze_log.reference_positions(rows, t_bad))
    assert pm_bad["rms"] > 0.01


def test_hover_reference():
    setpoint = [0.2, -0.1, 0.5]
    n = 50
    ref = analyze_log.hover_reference(setpoint, n)
    assert ref.shape == (n, 3)
    pm = analyze_log.position_metrics(ref + OFFSET, ref)
    assert abs(pm["rms"] - float(np.linalg.norm(OFFSET))) < 1e-12
    # overlap_mask with no duration keeps everything at/after t0
    t = np.linspace(0.0, 5.0, n)
    assert int(np.count_nonzero(analyze_log.overlap_mask(t, 2.0))) == \
        int(np.count_nonzero(t >= 2.0))


def test_battery_metrics():
    bm = analyze_log.battery_metrics(np.array([4.1, 4.0, 3.9, 3.95]))
    assert abs(bm["mean"] - 3.9875) < 1e-12
    assert abs(bm["min"] - 3.9) < 1e-12
    assert abs(bm["sag"] - 0.2) < 1e-12


def test_load_flight_log_validation(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b,c\n1,2,3\n")
    with pytest.raises(ValueError):
        analyze_log.load_flight_log(str(bad))
    single = tmp_path / "single.csv"
    single.write_text("t,x,y,z\n0.0,0.0,0.0,0.5\n")
    with pytest.raises(ValueError):
        analyze_log.load_flight_log(str(single))


def test_empty_log_file_is_valueerror_and_exit_2(tmp_path, capsys):
    empty = tmp_path / "empty.csv"
    empty.write_text("")
    with pytest.raises(ValueError):
        analyze_log.load_flight_log(str(empty))
    # Through the CLI it must exit 2 (malformed log), never traceback / exit 1.
    assert analyze_log.main([str(empty), "--hover", "0", "0", "0.5"]) == 2
    assert "Invalid log" in capsys.readouterr().err


def test_nonfinite_vbat_rejected(tmp_path, capsys):
    n = 5
    t = np.linspace(0.0, 1.0, n)
    pos = np.tile([0.0, 0.0, 0.5], (n, 1))
    vbat = np.array([4.0, 4.0, np.nan, 3.9, 3.9])
    log_path = tmp_path / "nan_vbat.csv"
    _write_log(log_path, t, pos, vbat=vbat)
    with pytest.raises(ValueError, match="vbat"):
        analyze_log.load_flight_log(str(log_path))
    assert analyze_log.main([str(log_path), "--hover", "0", "0", "0.5"]) == 2
    assert "Invalid log" in capsys.readouterr().err


def test_nonfinite_hover_rejected(tmp_path):
    n = 5
    t = np.linspace(0.0, 1.0, n)
    pos = np.tile([0.0, 0.0, 0.5], (n, 1))
    log_path = tmp_path / "log.csv"
    _write_log(log_path, t, pos)
    with pytest.raises(SystemExit) as exc:
        analyze_log.main([str(log_path), "--hover", "nan", "0", "0.5"])
    assert exc.value.code == 2


def test_max_rms_gate_fails_closed_on_nan(tmp_path, monkeypatch):
    n = 5
    t = np.linspace(0.0, 1.0, n)
    pos = np.tile([0.0, 0.0, 0.5], (n, 1))
    log_path = tmp_path / "log.csv"
    _write_log(log_path, t, pos)
    # Force a NaN RMS: the gate must FAIL (exit 1), not pass silently.
    monkeypatch.setattr(
        analyze_log, "position_metrics",
        lambda p, r: {"rms": float("nan"), "max": 0.0,
                      "axis_rms": np.zeros(3), "final": 0.0})
    assert analyze_log.main(
        [str(log_path), "--hover", "0", "0", "0.5", "--max-rms", "0.5"]) == 1


def test_zero_duration_window_warns(tmp_path, capsys):
    n = 5
    t = np.zeros(n)  # all identical timestamps: zero-duration window
    pos = np.tile([0.0, 0.0, 0.5], (n, 1))
    log_path = tmp_path / "zero_duration.csv"
    _write_log(log_path, t, pos)
    assert analyze_log.main([str(log_path), "--hover", "0", "0", "0.5"]) == 0
    err = capsys.readouterr().err
    assert "WARNING" in err and "zero duration" in err


def test_cli_threshold_gate(tmp_path, capsys):
    rows, ref_path = _rows_and_csv(tmp_path)
    t_log, pos = _perfect_log(rows, t_shift=2.0)
    log_path = tmp_path / "log.csv"
    vbat = np.linspace(4.1, 3.9, t_log.size)
    _write_log(log_path, t_log, pos + np.array([0.0, 0.0, 0.05]), vbat=vbat)

    base = [str(log_path), "--ref", ref_path, "--t0", "2.0"]
    # RMS is exactly 0.05 m: passes a 0.2 m gate, fails a 0.01 m gate.
    assert analyze_log.main(base + ["--max-rms", "0.2"]) == 0
    assert analyze_log.main(base + ["--max-rms", "0.01"]) == 1
    out = capsys.readouterr()
    assert "position RMS error" in out.out
    assert "battery mean / min" in out.out
    assert "exceeds --max-rms" in out.err

    # No overlap (t0 far beyond the log) -> exit 2.
    assert analyze_log.main([str(log_path), "--ref", ref_path, "--t0", "1e6"]) == 2
    # Exactly one of --ref/--hover is required (argparse exits with code 2).
    with pytest.raises(SystemExit) as exc:
        analyze_log.main([str(log_path)])
    assert exc.value.code == 2


def test_cli_hover_and_plot(tmp_path, capsys):
    n = 100
    t = np.linspace(0.0, 4.0, n)
    setpoint = np.array([0.0, 0.0, 0.5])
    pos = np.tile(setpoint, (n, 1)) + OFFSET
    log_path = tmp_path / "hover_log.csv"
    _write_log(log_path, t, pos)
    png = tmp_path / "tracking.png"

    code = analyze_log.main(
        [str(log_path), "--hover", "0", "0", "0.5", "--plot", str(png)])
    assert code == 0
    assert png.exists() and png.stat().st_size > 0
    out = capsys.readouterr().out
    assert "hover setpoint" in out
    assert "Plot saved" in out
