"""Fly a min-snap trajectory on a Crazyflie via the high-level commander.

Builds a ``quadsim`` MinSnapTrajectory (or loads a poly4d CSV), converts it to
the firmware's poly4d segment format with ``cf_trajectory``, then uploads it to
trajectory memory and flies it: takeoff -> go to trajectory start ->
start_trajectory -> land. Follows the cflib ``autonomous_sequence_high_level``
example pattern.

``--dry-run`` validates the rows (row width, positive durations, sane peak
velocity/accel), prints the segment table and total duration, and exits 0
without importing cflib, so it works with no radio, no Crazyflie, and even no
cflib installed. The same validation guards the real flight path.

Examples:
    python flight/fly_minsnap.py --dry-run
    python flight/fly_minsnap.py --preset square --height 0.6 --dry-run
    python flight/fly_minsnap.py --csv out/traj.csv --dry-run
    python flight/fly_minsnap.py --uri radio://0/80/2M/E7E7E7E7E7
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cf_trajectory  # noqa: E402

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"
TRAJECTORY_ID = 1

# Waypoint presets, built at the height given by --height (meters, x-y in meters).
PRESETS = {
    "square": np.array([
        [0.0, 0.0], [0.6, 0.0], [0.6, 0.6], [0.0, 0.6], [0.0, 0.0],
    ]),
    "line": np.array([
        [0.0, 0.0], [0.8, 0.0], [0.0, 0.0],
    ]),
}


def build_rows(args):
    """Return (poly4d rows, description string) from --csv or a waypoint preset."""
    if args.csv:
        rows = cf_trajectory.load_csv(args.csv)
        return rows, "CSV %s" % args.csv
    from quadsim.trajectory import MinSnapTrajectory

    xy = PRESETS[args.preset]
    waypoints = np.column_stack([xy, np.full(len(xy), args.height)])
    traj = MinSnapTrajectory(waypoints, avg_speed=args.speed,
                             yaw_mode="fixed", yaw_fixed=0.0)
    rows = cf_trajectory.to_poly4d(traj)
    return rows, "preset '%s' at height %.2f m" % (args.preset, args.height)


def print_segment_table(rows, source):
    """Print one line per segment: duration and firmware-evaluated start/end points."""
    print("Trajectory from %s: %d segments, total duration %.2f s"
          % (source, len(rows), cf_trajectory.total_duration(rows)))
    print("%3s %9s  %-28s %-28s" % ("seg", "dur [s]", "start (x, y, z) [m]", "end (x, y, z) [m]"))
    t0 = 0.0
    for i, row in enumerate(rows):
        p0, _ = cf_trajectory.evaluate_poly4d(rows, t0)
        p1, _ = cf_trajectory.evaluate_poly4d(rows, t0 + row[0])
        print("%3d %9.3f  (%7.3f, %7.3f, %7.3f)  (%7.3f, %7.3f, %7.3f)"
              % (i, row[0], p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]))
        t0 += row[0]


# --- Flight path: everything below imports cflib lazily -------------------------


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


class _Uploader:
    """Blocks until a trajectory-memory write finishes (cflib example pattern)."""

    def __init__(self):
        self._done = False
        self._success = False

    def upload(self, trajectory_mem):
        print("Uploading trajectory...")
        trajectory_mem.write_data(self._write_done, write_failed_cb=self._write_failed)
        while not self._done:
            time.sleep(0.2)
        return self._success

    def _write_done(self, mem, addr):
        self._done = True
        self._success = True

    def _write_failed(self, mem, addr):
        self._done = True
        self._success = False


def upload_trajectory(cf, trajectory_id, rows):
    """Write poly4d rows to trajectory memory and define them; return duration [s]."""
    from cflib.crazyflie.mem import MemoryElement, Poly4D

    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []
    for row in rows:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
    if not _Uploader().upload(trajectory_mem):
        raise RuntimeError("trajectory upload failed")
    cf.high_level_commander.define_trajectory(
        trajectory_id, 0, len(trajectory_mem.trajectory))
    return cf_trajectory.total_duration(rows)


def fly(args, rows):
    """Connect and fly the trajectory. Requires cflib, a Crazyradio, and a Crazyflie."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    print("Connecting to %s ..." % args.uri)
    with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
        cf = scf.cf
        cf.param.set_value("commander.enHighLevel", "1")
        reset_estimator(scf)
        duration = upload_trajectory(cf, TRAJECTORY_ID, rows)
        print("Trajectory uploaded, duration %.2f s" % duration)

        print("Arming...")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        commander = cf.high_level_commander
        start_pos, start_yaw = cf_trajectory.evaluate_poly4d(rows, 0.0)

        print("Takeoff to %.2f m" % args.height)
        commander.takeoff(args.height, 2.0)
        time.sleep(3.0)

        print("Moving to trajectory start (%.2f, %.2f, %.2f)" % tuple(start_pos))
        commander.go_to(start_pos[0], start_pos[1], start_pos[2], start_yaw, 2.0)
        time.sleep(2.5)

        print("Starting trajectory")
        commander.start_trajectory(TRAJECTORY_ID, time_scale=1.0, relative=False)
        time.sleep(duration + 0.5)

        print("Landing")
        commander.land(0.0, 2.0)
        time.sleep(2.5)
        commander.stop()
    print("Done.")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Fly a min-snap trajectory on a Crazyflie (high-level commander).")
    parser.add_argument("--uri", default=DEFAULT_URI, help="Crazyflie URI")
    parser.add_argument("--csv", default=None,
                        help="poly4d CSV to fly instead of building a preset")
    parser.add_argument("--preset", default="square", choices=sorted(PRESETS),
                        help="waypoint preset used when --csv is not given")
    parser.add_argument("--height", type=float, default=0.5,
                        help="flight height in meters for the preset waypoints")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="average speed in m/s used for segment timing")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the segment table and exit (no cflib, no hardware)")
    args = parser.parse_args(argv)

    rows, source = build_rows(args)
    try:
        peak_v, peak_a = cf_trajectory.validate_rows(rows)
    except ValueError as exc:
        print("Invalid trajectory: %s" % exc, file=sys.stderr)
        return 2
    print_segment_table(rows, source)
    print("Peak speed %.2f m/s, peak accel %.2f m/s^2 (limits %.1f, %.1f)"
          % (peak_v, peak_a, cf_trajectory.MAX_SPEED, cf_trajectory.MAX_ACCEL))
    if args.dry_run:
        print("Dry run: not connecting to a Crazyflie.")
        return 0
    fly(args, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
