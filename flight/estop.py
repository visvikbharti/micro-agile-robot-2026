"""Emergency stop: connect to a Crazyflie and cut motor power NOW.

This is the panic button a solo operator keeps open, pre-typed, in a second
terminal for every flight session (`docs/FLIGHT_TEST_PLAN.md` §2.6, drilled on
the ground with props off as FT-0.6). Running it connects and sends the cflib
emergency stop (``cf.loc.send_emergency_stop()``): all motors off at once, the
vehicle falls where it is. That is the point — let it fall inside the net onto
the soft floor; never hand-catch (`docs/FLIGHT_TEST_PLAN.md` §2.3). For a
controlled abort, Ctrl-C in the flight script's own terminal commands a normal
landing instead.

Single-Crazyradio constraint: one dongle serves one process, so while a flight
script holds the radio this tool cannot open it. With one dongle, kill the
script first (second Ctrl-C / close its terminal — the firmware watchdog cuts
motors on link loss) and run this once the dongle frees, to confirm and safe
the vehicle. With a second Crazyradio, this fires instantly mid-flight on its
own dongle (`docs/FLIGHT_TEST_PLAN.md` §2.6 layer 2; drilled both ways in
FT-0.6 Trial B).

``--dry-run`` prints what it would do and exits 0 without importing cflib, so
it works with no radio, no Crazyflie, and even no cflib installed.

Examples:
    python flight/estop.py --dry-run
    python flight/estop.py --uri radio://0/80/2M/E7E7E7E7E7
"""

from __future__ import annotations

import argparse
import sys
import time

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


def send_emergency_stop(uri):
    """Connect and send the emergency stop. Requires cflib and a Crazyradio."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    print("Connecting to %s ..." % uri)
    with SyncCrazyflie(uri, cf=Crazyflie()) as scf:
        scf.cf.loc.send_emergency_stop()
        time.sleep(0.2)  # let the packet leave the radio before closing the link
    print("EMERGENCY STOP SENT: motors off.")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Emergency stop: kill all Crazyflie motors immediately "
                    "(second-terminal panic button).")
    parser.add_argument("--uri", default=DEFAULT_URI, help="Crazyflie URI")
    parser.add_argument("--dry-run", action="store_true",
                        help="print what would happen and exit (no cflib, no hardware)")
    args = parser.parse_args(argv)

    print("Emergency stop plan: connect %s -> send emergency stop "
          "-> motors off, vehicle falls where it is." % args.uri)
    if args.dry_run:
        print("Dry run: not connecting to a Crazyflie.")
        return 0
    try:
        send_emergency_stop(args.uri)
    except Exception as exc:
        print("Emergency stop FAILED: %s" % exc, file=sys.stderr)
        print("Fall back to the physical layer: let it crash into the net and "
              "disconnect the battery.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
