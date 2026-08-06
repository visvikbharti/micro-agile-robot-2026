"""Automated pre-flight bench checks (FT-0.x) for the Crazyflie 2.1+.

Runs the FT-0 bench and ground checks of the flight-test plan against a
connected Crazyflie and prints a PASS/WARN/FAIL summary table:

    FT-0.1  radio link established, firmware version readable
    FT-0.2  Flow deck v2 detected (``deck.bcFlow2`` parameter)
    FT-0.3  battery voltage (``pm.vbat``) at or above --min-voltage
    FT-0.4  Kalman estimator variance converges on the pad
            (same reset + variance-settling pattern as ``fly_hover.py``)
    FT-0.5  attitude sanity: roll/pitch near level on a flat pad

Exit code is 0 only if no check FAILs (WARNs are allowed). Run this before
every FT-1.x hover session (`flight/README.md` Part C).

``--dry-run`` prints the full check plan and exits 0 without importing cflib,
so it works with no radio, no Crazyflie, and even no cflib installed.

Thresholds and where they come from:

- Estimator variance: converged when the max-minus-min spread of each of
  ``kalman.varPX/varPY/varPZ`` over the last 10 samples (500 ms apart) drops
  below 0.001 — copied verbatim from ``fly_hover.py``. The 10 x 500 ms window
  needs >= 5 s of fresh samples before it can possibly report converged, so a
  --timeout below ~6 s will always FAIL.
- Battery: 1S LiPo, 4.2 V full. Default minimum 3.9 V resting keeps most of a
  flight in the pack; hover load sags it by roughly 0.1 V, so a resting
  voltage within 0.1 V of the minimum crosses it as soon as motors spin —
  hence the 0.1 V WARN band above the FAIL threshold.
- Attitude: an attitude estimate tilted by theta on a truly level pad becomes
  a lateral acceleration command of g*sin(theta) at takeoff: 2 deg -> 0.34
  m/s^2 (noticeable drift, WARN); above 5 deg -> 0.86 m/s^2 (abort, FAIL).
  Re-place the vehicle flat and power-cycle (`flight/README.md` Part C step 3:
  gyro bias calibrates at boot) before retrying.

Examples:
    python flight/preflight.py --dry-run
    python flight/preflight.py --uri radio://0/80/2M/E7E7E7E7E7
    python flight/preflight.py --min-voltage 4.0 --timeout 20
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"
DEFAULT_MIN_VOLTAGE = 3.9
DEFAULT_TIMEOUT_S = 15.0

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"

VOLTAGE_WARN_BAND = 0.10  # V above --min-voltage that still WARNs (sag margin)
VAR_SPREAD_THRESHOLD = 0.001  # same settling threshold as fly_hover.py
VAR_HISTORY_LEN = 10  # same 10-sample window as fly_hover.py
KALMAN_VARIANCE_VARS = ("kalman.varPX", "kalman.varPY", "kalman.varPZ")
LEVEL_PASS_DEG = 2.0  # |roll|,|pitch| at or below this: PASS
LEVEL_WARN_DEG = 5.0  # above PASS up to this: WARN; beyond: FAIL


@dataclass
class CheckResult:
    """Outcome of one FT-0.x check.

    Attributes:
        test_id: Flight-test ID, e.g. "FT-0.3".
        name: Short human-readable check name.
        status: One of PASS, WARN, FAIL.
        measured: Measured value, formatted for the summary table.
        threshold: Threshold the measurement was judged against.
        note: Optional advice or failure detail.
    """

    test_id: str
    name: str
    status: str
    measured: str
    threshold: str
    note: str = ""


# --- Pure evaluation logic (no cflib, no I/O; unit-tested) ----------------------


def evaluate_link(connected: bool, firmware_version: Optional[str]) -> CheckResult:
    """FT-0.1: radio link up and firmware version readable.

    WARN (not FAIL) when the link is up but the version parameters could not
    be read: flyable, but firmware/cflib protocol drift is a classic time sink
    (`flight/README.md` Part C step 2), so it is worth noticing.
    """
    if not connected:
        return CheckResult("FT-0.1", "radio link + firmware", FAIL,
                           "no link", "link up, version readable",
                           "check Crazyradio, power, and --uri")
    if not firmware_version:
        return CheckResult("FT-0.1", "radio link + firmware", WARN,
                           "link up, version unreadable",
                           "link up, version readable",
                           "update firmware and cflib together")
    return CheckResult("FT-0.1", "radio link + firmware", PASS,
                       "link up, fw %s" % firmware_version,
                       "link up, version readable")


def evaluate_flow_deck(bc_flow2: Optional[int]) -> CheckResult:
    """FT-0.2: Flow deck v2 detected via the ``deck.bcFlow2`` parameter.

    Without the deck, position is unobservable (`docs/DESIGN.md` section 9) and
    every autonomous script in ``flight/`` is unsafe to run — hard FAIL.
    """
    if bc_flow2 is None:
        return CheckResult("FT-0.2", "Flow deck v2", FAIL,
                           "deck.bcFlow2 missing", "deck.bcFlow2 == 1",
                           "parameter absent: very old firmware?")
    if bc_flow2 != 1:
        return CheckResult("FT-0.2", "Flow deck v2", FAIL,
                           "deck.bcFlow2 = %d" % bc_flow2, "deck.bcFlow2 == 1",
                           "reseat the deck pins; mount underneath")
    return CheckResult("FT-0.2", "Flow deck v2", PASS,
                       "deck.bcFlow2 = 1", "deck.bcFlow2 == 1")


def evaluate_battery(vbat: float, min_v: float,
                     warn_band: float = VOLTAGE_WARN_BAND) -> CheckResult:
    """FT-0.3: battery voltage at or above the minimum, with a WARN band.

    FAIL below ``min_v``; WARN in [min_v, min_v + warn_band) because hover
    load sags a 1S pack by roughly the band width; PASS at or above the band.
    """
    threshold = ">= %.2f V (warn < %.2f V)" % (min_v, min_v + warn_band)
    measured = "%.2f V" % vbat
    if vbat < min_v:
        return CheckResult("FT-0.3", "battery voltage", FAIL, measured,
                           threshold, "charge before flying")
    if vbat < min_v + warn_band:
        return CheckResult("FT-0.3", "battery voltage", WARN, measured,
                           threshold, "flyable, but expect a short flight")
    return CheckResult("FT-0.3", "battery voltage", PASS, measured, threshold)


def evaluate_estimator(converged: bool, elapsed_s: float, max_spread: float,
                       threshold: float = VAR_SPREAD_THRESHOLD,
                       timeout_s: float = DEFAULT_TIMEOUT_S) -> CheckResult:
    """FT-0.4: Kalman variance settled on the pad within the timeout.

    ``max_spread`` is the worst max-minus-min spread across the varPX/PY/PZ
    histories at the moment the wait ended (`fly_hover.py` convergence rule).
    """
    thresh_str = "spread <= %.4g within %.0f s" % (threshold, timeout_s)
    if converged:
        return CheckResult("FT-0.4", "estimator convergence", PASS,
                           "spread %.4g in %.1f s" % (max_spread, elapsed_s),
                           thresh_str)
    return CheckResult("FT-0.4", "estimator convergence", FAIL,
                       "spread %.4g after %.1f s" % (max_spread, elapsed_s),
                       thresh_str,
                       "keep the vehicle still on a textured floor; retry")


def evaluate_attitude(roll_deg: float, pitch_deg: float,
                      pass_max_deg: float = LEVEL_PASS_DEG,
                      warn_max_deg: float = LEVEL_WARN_DEG) -> CheckResult:
    """FT-0.5: roll/pitch near level on a flat pad.

    Judged on the worse of |roll| and |pitch|: PASS at or below
    ``pass_max_deg``, WARN up to ``warn_max_deg``, FAIL beyond.
    """
    tilt = max(abs(roll_deg), abs(pitch_deg))
    threshold = "|roll|,|pitch| <= %.1f deg (warn <= %.1f deg)" % (
        pass_max_deg, warn_max_deg)
    measured = "roll %+.1f deg, pitch %+.1f deg" % (roll_deg, pitch_deg)
    if tilt > warn_max_deg:
        return CheckResult("FT-0.5", "attitude level", FAIL, measured,
                           threshold, "re-place flat and power-cycle")
    if tilt > pass_max_deg:
        return CheckResult("FT-0.5", "attitude level", WARN, measured,
                           threshold, "pad not level, or stale bias calibration")
    return CheckResult("FT-0.5", "attitude level", PASS, measured, threshold)


def exit_code(results: "list[CheckResult]") -> int:
    """0 if no check FAILed (WARNs allowed), else 1."""
    return 1 if any(r.status == FAIL for r in results) else 0


def format_summary(results: "list[CheckResult]") -> str:
    """Render the summary table plus an overall verdict line."""
    lines = ["", "%-8s %-24s %-4s  %-34s %s"
             % ("test", "check", "", "measured", "threshold")]
    for r in results:
        lines.append("%-8s %-24s %-4s  %-34s %s"
                     % (r.test_id, r.name, r.status, r.measured, r.threshold))
        if r.note:
            lines.append("%-8s %-24s %-4s  -> %s" % ("", "", "", r.note))
    n_warn = sum(1 for r in results if r.status == WARN)
    n_fail = sum(1 for r in results if r.status == FAIL)
    verdict = FAIL if n_fail else (WARN if n_warn else PASS)
    lines.append("")
    lines.append("Overall: %s (%d checks, %d WARN, %d FAIL)"
                 % (verdict, len(results), n_warn, n_fail))
    return "\n".join(lines)


def format_check_plan(uri: str, min_v: float, timeout_s: float) -> str:
    """Render the FT-0 check plan (what --dry-run prints)."""
    rows = [
        ("FT-0.1", "radio link + firmware",
         "connect %s, read firmware version" % uri),
        ("FT-0.2", "Flow deck v2",
         "deck.bcFlow2 == 1"),
        ("FT-0.3", "battery voltage",
         "pm.vbat >= %.2f V (warn < %.2f V)" % (min_v, min_v + VOLTAGE_WARN_BAND)),
        ("FT-0.4", "estimator convergence",
         "kalman.varPX/PY/PZ spread <= %.4g over %d x 500 ms samples within %.0f s"
         % (VAR_SPREAD_THRESHOLD, VAR_HISTORY_LEN, timeout_s)),
        ("FT-0.5", "attitude level",
         "|roll|,|pitch| <= %.1f deg (warn <= %.1f deg)"
         % (LEVEL_PASS_DEG, LEVEL_WARN_DEG)),
    ]
    lines = ["FT-0 bench checks (pre-flight):"]
    for test_id, name, what in rows:
        lines.append("  %-8s %-24s %s" % (test_id, name, what))
    lines.append("Exit code 0 only if no check FAILs.")
    return "\n".join(lines)


# --- Hardware path: everything below imports cflib lazily -----------------------


def _read_firmware_version(cf) -> Optional[str]:
    """Return the firmware git revision as a hex string, or None if unreadable."""
    try:
        rev0 = int(cf.param.get_value("firmware.revision0"))
        rev1 = int(cf.param.get_value("firmware.revision1"))
        return "%04x%08x" % (rev1, rev0)
    except Exception:
        return None


def _read_param_int(cf, name: str) -> Optional[int]:
    """Return an integer parameter value, or None if the parameter is missing."""
    try:
        return int(float(cf.param.get_value(name)))
    except Exception:
        return None


def _read_log_once(scf, variables: "list[str]") -> "dict[str, float]":
    """Read one sample of the given float log variables."""
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger

    log_config = LogConfig(name="preflight", period_in_ms=100)
    for var in variables:
        log_config.add_variable(var, "float")
    with SyncLogger(scf, log_config) as logger:
        for _, data, _ in logger:
            return {var: float(data[var]) for var in variables}
    raise RuntimeError("log read returned no samples")


def _wait_for_estimator(scf, threshold: float, timeout_s: float):
    """Reset the Kalman estimator, then wait for its variance to settle.

    Same reset + 10-sample variance-spread pattern as ``fly_hover.py``, plus a
    timeout. Returns (converged, elapsed_s, max_spread).
    """
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger

    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")

    print("Waiting for estimator to converge (timeout %.0f s)..." % timeout_s)
    log_config = LogConfig(name="Kalman Variance", period_in_ms=500)
    for var in KALMAN_VARIANCE_VARS:
        log_config.add_variable(var, "float")
    var_hist = {k: [1000.0] * VAR_HISTORY_LEN for k in KALMAN_VARIANCE_VARS}
    t_start = time.time()
    max_spread = float("inf")
    with SyncLogger(scf, log_config) as logger:
        for _, data, _ in logger:
            spreads = []
            for name, hist in var_hist.items():
                hist.append(data[name])
                hist.pop(0)
                spreads.append(max(hist) - min(hist))
            max_spread = max(spreads)
            elapsed = time.time() - t_start
            if max_spread <= threshold:
                return True, elapsed, max_spread
            if elapsed >= timeout_s:
                return False, elapsed, max_spread
    return False, time.time() - t_start, max_spread


def run_checks(args) -> "list[CheckResult]":
    """Connect and run FT-0.1 .. FT-0.5. Requires cflib and a Crazyradio."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    print("Connecting to %s ..." % args.uri)
    try:
        scf_ctx = SyncCrazyflie(args.uri, cf=Crazyflie())
        scf_ctx.open_link()
    except Exception as exc:
        print("Connection failed: %s" % exc, file=sys.stderr)
        results = [evaluate_link(False, None)]
        for test_id, name in (("FT-0.2", "Flow deck v2"),
                              ("FT-0.3", "battery voltage"),
                              ("FT-0.4", "estimator convergence"),
                              ("FT-0.5", "attitude level")):
            results.append(CheckResult(test_id, name, FAIL, "not run",
                                       "n/a", "skipped: no link"))
        return results

    try:
        cf = scf_ctx.cf
        results = [evaluate_link(True, _read_firmware_version(cf))]

        # Each remaining check runs inside a try/except so an exception after
        # the link is up (log read, estimator wait, param read) turns into a
        # FAIL row for that check and the ones after it, instead of a
        # traceback -- mirroring the connection-failure fallback above.
        sample: "dict[str, float]" = {}

        def check_flow_deck() -> CheckResult:
            return evaluate_flow_deck(_read_param_int(cf, "deck.bcFlow2"))

        def check_battery() -> CheckResult:
            sample.update(_read_log_once(
                scf_ctx, ["pm.vbat", "stabilizer.roll", "stabilizer.pitch"]))
            return evaluate_battery(sample["pm.vbat"], args.min_voltage)

        def check_estimator() -> CheckResult:
            converged, elapsed, max_spread = _wait_for_estimator(
                scf_ctx, VAR_SPREAD_THRESHOLD, args.timeout)
            return evaluate_estimator(converged, elapsed, max_spread,
                                      VAR_SPREAD_THRESHOLD, args.timeout)

        def check_attitude() -> CheckResult:
            return evaluate_attitude(sample["stabilizer.roll"],
                                     sample["stabilizer.pitch"])

        checks = [("FT-0.2", "Flow deck v2", check_flow_deck),
                  ("FT-0.3", "battery voltage", check_battery),
                  ("FT-0.4", "estimator convergence", check_estimator),
                  ("FT-0.5", "attitude level", check_attitude)]
        for i, (test_id, name, check) in enumerate(checks):
            try:
                results.append(check())
            except Exception as exc:
                print("Check %s (%s) errored: %s" % (test_id, name, exc),
                      file=sys.stderr)
                results.append(CheckResult(test_id, name, FAIL,
                                           "error: %s" % exc, "n/a"))
                for later_id, later_name, _ in checks[i + 1:]:
                    results.append(CheckResult(
                        later_id, later_name, FAIL, "not run", "n/a",
                        "skipped: %s errored" % test_id))
                break
        return results
    finally:
        scf_ctx.close_link()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Pre-flight FT-0 bench checks for the Crazyflie 2.1+.")
    parser.add_argument("--uri", default=DEFAULT_URI, help="Crazyflie URI")
    parser.add_argument("--min-voltage", type=float, default=DEFAULT_MIN_VOLTAGE,
                        help="battery FAIL threshold in volts (default 3.9)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="estimator convergence timeout in seconds")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the check plan and exit (no cflib, no hardware)")
    args = parser.parse_args(argv)

    print(format_check_plan(args.uri, args.min_voltage, args.timeout))
    if args.dry_run:
        print("Dry run: not connecting to a Crazyflie.")
        return 0
    results = run_checks(args)
    print(format_summary(results))
    return exit_code(results)


if __name__ == "__main__":
    sys.exit(main())
