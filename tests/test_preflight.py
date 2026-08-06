"""Tests for flight/preflight.py: FT-0 bench-check logic, with no cflib installed."""
from __future__ import annotations

import os
import subprocess
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "flight"))

import preflight  # noqa: E402

PASS, WARN, FAIL = preflight.PASS, preflight.WARN, preflight.FAIL


def test_import_does_not_pull_cflib():
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_defaults_match_fly_hover_conventions():
    assert preflight.DEFAULT_URI == "radio://0/80/2M/E7E7E7E7E7"
    assert preflight.DEFAULT_MIN_VOLTAGE == 3.9
    assert preflight.VAR_SPREAD_THRESHOLD == 0.001
    assert preflight.VAR_HISTORY_LEN == 10


# --- FT-0.1 radio link + firmware ------------------------------------------------


def test_link_pass_with_version():
    r = preflight.evaluate_link(True, "0123deadbeef")
    assert r.status == PASS
    assert r.test_id == "FT-0.1"
    assert "0123deadbeef" in r.measured


def test_link_warn_without_version():
    assert preflight.evaluate_link(True, None).status == WARN
    assert preflight.evaluate_link(True, "").status == WARN


def test_link_fail_disconnected():
    r = preflight.evaluate_link(False, None)
    assert r.status == FAIL


# --- FT-0.2 Flow deck v2 ---------------------------------------------------------


def test_flow_deck_detected():
    r = preflight.evaluate_flow_deck(1)
    assert r.status == PASS
    assert r.test_id == "FT-0.2"


def test_flow_deck_absent_or_missing_param():
    assert preflight.evaluate_flow_deck(0).status == FAIL
    assert preflight.evaluate_flow_deck(None).status == FAIL


# --- FT-0.3 battery voltage ------------------------------------------------------


def test_battery_fail_below_min():
    r = preflight.evaluate_battery(3.89, 3.9)
    assert r.status == FAIL
    assert r.test_id == "FT-0.3"
    assert "3.89" in r.measured


def test_battery_warn_band_boundaries():
    # WARN band is [min_v, min_v + 0.1)
    assert preflight.evaluate_battery(3.90, 3.9).status == WARN
    assert preflight.evaluate_battery(3.99, 3.9).status == WARN
    assert preflight.evaluate_battery(4.00, 3.9).status == PASS
    assert preflight.evaluate_battery(4.20, 3.9).status == PASS


def test_battery_respects_custom_min():
    assert preflight.evaluate_battery(3.95, 4.0).status == FAIL
    assert preflight.evaluate_battery(4.15, 4.0).status == PASS


# --- FT-0.4 estimator convergence ------------------------------------------------


def test_estimator_pass_when_converged():
    r = preflight.evaluate_estimator(True, 5.5, 0.0004, 0.001, 15.0)
    assert r.status == PASS
    assert r.test_id == "FT-0.4"
    assert "5.5" in r.measured
    assert "0.0004" in r.measured


def test_estimator_fail_on_timeout():
    r = preflight.evaluate_estimator(False, 15.0, 0.2, 0.001, 15.0)
    assert r.status == FAIL
    assert "0.2" in r.measured


# --- FT-0.5 attitude level -------------------------------------------------------


def test_attitude_pass_at_and_below_2deg():
    assert preflight.evaluate_attitude(0.0, 0.0).status == PASS
    assert preflight.evaluate_attitude(2.0, -2.0).status == PASS


def test_attitude_warn_between_2_and_5deg():
    assert preflight.evaluate_attitude(0.0, -3.0).status == WARN
    assert preflight.evaluate_attitude(5.0, 0.0).status == WARN


def test_attitude_fail_beyond_5deg():
    r = preflight.evaluate_attitude(-7.0, 0.5)
    assert r.status == FAIL
    assert r.test_id == "FT-0.5"


# --- Summary and exit code -------------------------------------------------------


def _result(status: str) -> preflight.CheckResult:
    return preflight.CheckResult("FT-0.9", "dummy", status, "x", "y")


def test_exit_code_zero_without_fail():
    assert preflight.exit_code([_result(PASS), _result(PASS)]) == 0
    assert preflight.exit_code([_result(PASS), _result(WARN)]) == 0


def test_exit_code_one_with_any_fail():
    assert preflight.exit_code([_result(PASS), _result(FAIL), _result(WARN)]) == 1


def test_format_summary_contents():
    results = [
        preflight.evaluate_link(True, "abc123"),
        preflight.evaluate_battery(3.92, 3.9),
        preflight.evaluate_flow_deck(0),
    ]
    text = preflight.format_summary(results)
    for test_id in ("FT-0.1", "FT-0.3", "FT-0.2"):
        assert test_id in text
    assert "Overall: FAIL (3 checks, 1 WARN, 1 FAIL)" in text


def test_format_summary_overall_warn_and_pass():
    assert "Overall: WARN" in preflight.format_summary([_result(WARN)])
    assert "Overall: PASS" in preflight.format_summary([_result(PASS)])


# --- Dry run ---------------------------------------------------------------------


def test_dry_run_exit_zero_and_plan(capsys):
    rc = preflight.main(["--dry-run"])
    assert rc == 0
    out = capsys.readouterr().out
    for test_id in ("FT-0.1", "FT-0.2", "FT-0.3", "FT-0.4", "FT-0.5"):
        assert test_id in out
    assert "3.90 V" in out  # default --min-voltage in the plan
    assert not any(m == "cflib" or m.startswith("cflib.") for m in sys.modules)


def test_dry_run_subprocess_needs_no_cflib():
    script = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "flight", "preflight.py")
    proc = subprocess.run([sys.executable, script, "--dry-run"],
                          capture_output=True, text=True)
    assert proc.returncode == 0
    assert "Dry run" in proc.stdout
