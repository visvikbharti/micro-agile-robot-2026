"""First flight: connect to a Crazyflie, arm, take off, hover, land.

Sequence: connect -> reset estimator -> arm -> take off to --height (default
0.5 m) -> hold for --hold seconds (default 5 s) -> land. Uses the high-level
commander.

``--log PATH`` streams ``stateEstimate.x/y/z`` + ``pm.vbat`` at 50 Hz to an
``analyze_log.py``-ready CSV during the flight (``flight/flightlog.py``).

Ctrl-C during the flight is caught and runs the normal land + stop sequence —
it is the controlled abort. The hard kill (motors off, vehicle falls) is
``flight/estop.py`` in a second terminal.

``--dry-run`` prints the flight plan and exits 0 without importing cflib, so it
works with no radio, no Crazyflie, and even no cflib installed.

Examples:
    python flight/fly_hover.py --dry-run
    python flight/fly_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --height 0.5
    python flight/fly_hover.py --hold 30 --log out/ft12_log.csv
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flightlog  # noqa: E402

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


def _wait_for_position_estimator(scf):
    """Block until the Kalman position variance settles (standard cflib pattern)."""
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger

    print("Waiting for estimator to converge...")
    log_config = LogConfig(name="Kalman Variance", period_in_ms=500)
    log_config.add_variable("kalman.varPX", "float")
    log_config.add_variable("kalman.varPY", "float")
    log_config.add_variable("kalman.varPZ", "float")
    var_hist = {k: [1000.0] * 10 for k in ("kalman.varPX", "kalman.varPY", "kalman.varPZ")}
    threshold = 0.001
    with SyncLogger(scf, log_config) as logger:
        for _, data, _ in logger:
            ok = True
            for name, hist in var_hist.items():
                hist.append(data[name])
                hist.pop(0)
                if max(hist) - min(hist) > threshold:
                    ok = False
            if ok:
                print("Estimator converged.")
                return


def reset_estimator(scf):
    """Reset the Kalman estimator and wait for it to converge."""
    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")
    _wait_for_position_estimator(scf)


def fly(args):
    """Connect and hover. Requires cflib, a Crazyradio, and a Crazyflie."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    print("Connecting to %s ..." % args.uri)
    with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
        cf = scf.cf
        cf.param.set_value("commander.enHighLevel", "1")
        reset_estimator(scf)

        logger = flightlog.CsvFlightLogger(scf, args.log) if args.log else None
        if logger is not None:
            logger.start()
        try:
            print("Arming...")
            cf.platform.send_arming_request(True)
            time.sleep(1.0)

            commander = cf.high_level_commander
            try:
                print("Takeoff to %.2f m" % args.height)
                commander.takeoff(args.height, 2.0)
                time.sleep(2.5)

                print("Holding for %.1f s" % args.hold)
                time.sleep(args.hold)
            except KeyboardInterrupt:
                print("\nCtrl-C: aborting — landing")

            print("Landing")
            commander.land(0.0, 2.0)
            time.sleep(2.5)
            commander.stop()
        finally:
            if logger is not None:
                logger.stop()
    print("Done.")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="First flight: take off, hover, land (high-level commander).")
    parser.add_argument("--uri", default=DEFAULT_URI, help="Crazyflie URI")
    parser.add_argument("--height", type=float, default=0.5,
                        help="hover height in meters")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="hover hold time in seconds")
    parser.add_argument("--log", default=None, metavar="PATH",
                        help="write a t,x,y,z,vbat CSV (analyze_log.py format) "
                             "during the flight")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the flight plan and exit (no cflib, no hardware)")
    args = parser.parse_args(argv)

    print("Flight plan: connect %s -> reset estimator -> arm -> takeoff %.2f m "
          "-> hold %.1f s -> land" % (args.uri, args.height, args.hold))
    if args.log:
        print("Logging t,x,y,z,vbat at 50 Hz to %s (analyze_log.py format)" % args.log)
    print("Ctrl-C during flight commands the normal landing sequence.")
    if args.dry_run:
        print("Dry run: not connecting to a Crazyflie.")
        return 0
    fly(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
