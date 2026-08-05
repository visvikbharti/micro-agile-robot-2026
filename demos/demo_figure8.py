"""Figure-eight (lemniscate) demo.

Two laps of a Gerono lemniscate with ~1.2 m half-width sampled into waypoints,
flown as a minimum-snap trajectory (peaking around 2.5 m/s through the fast
sections; the dense waypoint sampling and the 0.5 s minimum segment time put
the whole-lap average near 0.9 m/s) with yaw_mode="velocity" (nose follows the
direction of travel). Reports RMS tracking error (target 0.10 m) and peak
speed. Outputs: figure8_3d.png,
figure8_tracking.png, figure8.gif.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RMS_TARGET = 0.10  # m


def lemniscate_waypoints(laps: int, points_per_lap: int = 16,
                         a: float = 1.2, z: float = 1.0) -> np.ndarray:
    """Sample a Gerono lemniscate x = a sin(th), y = a sin(th) cos(th) at height z.

    Half-width is `a` (x spans [-a, a]); half-height is a/2. Returns a
    ((laps * points_per_lap) + 1, 3) waypoint array with no consecutive
    duplicates (theta step 2*pi/points_per_lap never repeats a sample).
    """
    theta = np.linspace(0.0, 2.0 * np.pi * laps, laps * points_per_lap + 1)
    x = a * np.sin(theta)
    y = a * np.sin(theta) * np.cos(theta)
    return np.column_stack([x, y, np.full(theta.shape, z)])


def main(argv: Optional[List[str]] = None) -> int:
    """Run the figure-eight demo and return the exit code (1 only on gross failure)."""
    parser = argparse.ArgumentParser(
        description="Figure-eight lemniscate, two laps, velocity-aligned yaw.")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true", help="one lap only, skip GIF")
    parser.add_argument("--no-gif", action="store_true", help="skip GIF generation")
    parser.add_argument("--show", action="store_true",
                        help="use an interactive matplotlib backend")
    args = parser.parse_args(argv)

    # Backend must be fixed BEFORE pyplot / quadsim.viz are imported.
    if not args.show:
        os.environ["MPLBACKEND"] = "Agg"
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    from quadsim.controller import SE3Controller
    from quadsim.dynamics import QuadState
    from quadsim.params import QuadParams
    from quadsim.sim import simulate
    from quadsim.trajectory import MinSnapTrajectory
    from quadsim.viz import animate_quads, plot_tracking, plot_trajectory_3d

    os.makedirs(args.out, exist_ok=True)

    laps = 1 if args.fast else 2
    waypoints = lemniscate_waypoints(laps)
    traj = MinSnapTrajectory(waypoints, avg_speed=2.5, yaw_mode="velocity")

    params = QuadParams()
    controller = SE3Controller(params)
    state0 = QuadState.hover(waypoints[0], yaw=float(traj(0.0).yaw))

    T_sim = traj.T + 0.5  # small settle tail at the final (rest) waypoint
    hist = simulate(params, controller, traj, state0, T_sim, dt=0.002)

    rms_err = float(hist.rms_pos_error())
    max_err = float(np.max(hist.pos_error()))
    peak_speed = float(np.max(np.linalg.norm(hist.v, axis=1)))

    print(f"rms_error_m: {rms_err:.4f}")
    print(f"max_error_m: {max_err:.4f}")
    print(f"peak_speed_mps: {peak_speed:.3f}")
    print(f"total_time_s: {traj.T:.3f}")

    path_3d = os.path.join(args.out, "figure8_3d.png")
    path_tracking = os.path.join(args.out, "figure8_tracking.png")
    plot_trajectory_3d([hist], waypoints=waypoints, save=path_3d,
                       title="Figure-eight: 3D path")
    plot_tracking(hist, save=path_tracking, title="Figure-eight: tracking")
    print(f"saved: {path_3d}")
    print(f"saved: {path_tracking}")

    if not (args.no_gif or args.fast):
        path_gif = animate_quads([hist], params,
                                 save=os.path.join(args.out, "figure8.gif"),
                                 title="Figure-eight, velocity-aligned yaw")
        print(f"saved: {path_gif}")

    if args.show:
        plt.show()

    if rms_err >= RMS_TARGET:
        print(f"WARN rms_error {rms_err:.4f} m >= target {RMS_TARGET} m")
    else:
        print("PASS")
    gross = rms_err > 5.0 * RMS_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
