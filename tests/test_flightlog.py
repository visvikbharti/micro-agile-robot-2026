"""Tests for flight/flightlog.py: pure CSV logic round-tripped through analyze_log.

``FlightLogWriter`` is fed synthetic log-callback samples (ms timestamps +
variable dicts, exactly what a cflib ``LogConfig`` callback delivers) and the
resulting CSV is read back with ``analyze_log.load_flight_log`` — the consumer
whose format contract the logger exists to satisfy. No cflib required.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "flight"))

import analyze_log  # noqa: E402
import flightlog  # noqa: E402


def _synthetic_samples(n=100, t0_ms=987654, period_ms=flightlog.LOG_PERIOD_MS):
    """(timestamps_ms, xyz, vbat) mimicking 50 Hz callback data with a late t0."""
    rng = np.random.default_rng(7)
    timestamps = t0_ms + period_ms * np.arange(n)
    xyz = rng.uniform(-1.5, 1.5, size=(n, 3))
    vbat = np.linspace(4.1, 3.8, n)
    return timestamps, xyz, vbat


def _write_with_writer(path, timestamps, xyz, vbat):
    with open(path, "w") as f:
        writer = flightlog.FlightLogWriter(f)
        for ts, p, v in zip(timestamps, xyz, vbat):
            writer.add(int(ts), {
                "stateEstimate.x": p[0],
                "stateEstimate.y": p[1],
                "stateEstimate.z": p[2],
                "pm.vbat": v,
            })
    return writer


def test_import_does_not_pull_cflib():
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_round_trip_through_analyze_log(tmp_path):
    path = str(tmp_path / "log.csv")
    timestamps, xyz, vbat = _synthetic_samples()
    writer = _write_with_writer(path, timestamps, xyz, vbat)
    assert writer.n_rows == len(timestamps)

    t, pos, vbat_read = analyze_log.load_flight_log(path)
    assert t.shape == (len(timestamps),)
    # t is SECONDS relative to the FIRST sample, not the onboard ms clock origin
    assert t[0] == 0.0
    np.testing.assert_allclose(t, (timestamps - timestamps[0]) / 1000.0, atol=5e-4)
    np.testing.assert_allclose(pos, xyz, atol=5e-7)
    assert vbat_read is not None
    np.testing.assert_allclose(vbat_read, vbat, atol=5e-4)


def test_header_matches_analyze_log_contract(tmp_path):
    path = str(tmp_path / "log.csv")
    timestamps, xyz, vbat = _synthetic_samples(n=2)
    _write_with_writer(path, timestamps, xyz, vbat)
    with open(path) as f:
        assert f.readline().strip() == "t,x,y,z,vbat"


def test_one_log_block_fits_crtp_payload():
    # All four variables in ONE block: 4 floats = 16 bytes, under the ~26-byte
    # CRTP log payload; period 20 ms = 50 Hz.
    assert flightlog.LOG_VARIABLES == (
        "stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "pm.vbat")
    assert len(flightlog.LOG_VARIABLES) * 4 <= 26
    assert flightlog.LOG_PERIOD_MS == 20


def test_logger_construction_and_stop_need_no_cflib(tmp_path):
    logger = flightlog.CsvFlightLogger(object(), str(tmp_path / "x.csv"))
    logger.stop()  # stop before start is a safe no-op
    logger.stop()  # and idempotent
    assert not os.path.exists(str(tmp_path / "x.csv"))
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_on_data_callback_delegates_to_pure_writer(tmp_path):
    path = str(tmp_path / "cb.csv")
    logger = flightlog.CsvFlightLogger(object(), path)
    # Wire the pure half by hand: everything start() does except the LogConfig.
    logger._file = open(path, "w")
    logger._writer = flightlog.FlightLogWriter(logger._file)
    logger._on_data(1000, {"stateEstimate.x": 0.1, "stateEstimate.y": 0.2,
                           "stateEstimate.z": 0.3, "pm.vbat": 4.0}, None)
    logger._on_data(1020, {"stateEstimate.x": 0.2, "stateEstimate.y": 0.3,
                           "stateEstimate.z": 0.4, "pm.vbat": 3.99}, None)
    logger.stop()
    t, pos, vbat = analyze_log.load_flight_log(path)
    assert t.tolist() == [0.0, 0.02]
    np.testing.assert_allclose(pos[0], [0.1, 0.2, 0.3], atol=5e-7)
    np.testing.assert_allclose(pos[1], [0.2, 0.3, 0.4], atol=5e-7)
    np.testing.assert_allclose(vbat, [4.0, 3.99], atol=5e-4)
