"""Minimum-snap obstacle course demo.

Six waypoints thread two Gates and skirt a Box inside a 4 x 3 x 2 m volume,
flown with a minimum-snap trajectory (avg_speed = 2.0 m/s) and the SE(3)
controller. Reports RMS / max tracking error (targets 0.08 / 0.20 m), total
trajectory time, and peak speed. Outputs: minsnap_3d.png, minsnap_tracking.png,
minsnap_course.gif.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RMS_TARGET = 0.08  # m
MAX_TARGET = 0.20  # m

# Course design (hand-computed; flight volume x in [0, 4], y in [-1.5, 1.5], z in [0, 2]):
#
#   Gate 1: center (1.2, 0.0, 1.0), 0.9 x 0.9, axis "x"
#           -> opening spans y in [-0.45, 0.45], z in [0.55, 1.45] in the plane x = 1.2.
#   Gate 2: center (3.2, 0.8, 1.5), 0.9 x 0.9, axis "x"
#           -> opening spans y in [0.35, 1.25], z in [1.05, 1.95] in the plane x = 3.2.
#   Box:    center (2.2, -0.5, 0.5), size (0.8, 0.6, 1.0)
#           -> occupies x in [1.8, 2.6], y in [-0.8, -0.2], z in [0.0, 1.0].
#
# The straight-line waypoint path threads both gates dead-center and clears the box:
#   * x is non-decreasing along W0..W4 (0.2, 1.2, 2.2, 3.2, 3.8) and stays at 3.8 on W4->W5,
#     so the gate plane x = 1.2 is met exactly once, at W1 = gate-1 center (0.45 m from every
#     frame edge), and the plane x = 3.2 exactly once, at W3 = gate-2 center (again 0.45 m
#     clearance all around).
#   * Box clearance: the only segments overlapping the box x-range [1.8, 2.6] are
#     W1->W2 (y = 0.4 * (x - 1.2), so y in [0.24, 0.40] for x in [1.8, 2.2]) and
#     W2->W3 (y = 0.4 + 0.4 * (x - 2.2), so y in [0.40, 0.56] for x in [2.2, 2.6]).
#     Minimum y there is +0.24 vs box y_max = -0.2 -> y-clearance 0.44 m >= 0.2 m.
#     Every other segment is >= 0.6 m from the box in x alone (W0->W1 has x <= 1.2 vs box
#     x_min = 1.8; W4->W5 sits at x = 3.8 vs box x_max = 2.6). Densely sampling the polyline
#     against the cuboid gives an overall minimum Euclidean clearance of 0.43 m (near the
#     box's front-top corner region, where x- and y-offsets combine) — still >> 0.2 m.
WAYPOINTS = np.array([
    [0.2, 0.0, 1.0],    # W0 start
    [1.2, 0.0, 1.0],    # W1 = gate 1 center
    [2.2, 0.4, 1.3],    # W2 passes the box on its +y side
    [3.2, 0.8, 1.5],    # W3 = gate 2 center
    [3.8, 0.8, 1.0],    # W4 dive after gate 2
    [3.8, -0.4, 0.8],   # W5 finish
])


def main(argv: Optional[List[str]] = None) -> int:
    """Run the min-snap course demo and return the exit code (1 only on gross failure)."""
    parser = argparse.ArgumentParser(
        description="Minimum-snap obstacle course: two gates and a box.")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true",
                        help="shorter course (first 4 waypoints), skip GIF")
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
    from quadsim.viz import Box, Gate, animate_quads, plot_tracking, plot_trajectory_3d

    os.makedirs(args.out, exist_ok=True)

    obstacles = (
        Gate(center=np.array([1.2, 0.0, 1.0]), width=0.9, height=0.9, axis="x"),
        Gate(center=np.array([3.2, 0.8, 1.5]), width=0.9, height=0.9, axis="x"),
        Box(center=np.array([2.2, -0.5, 0.5]), size=np.array([0.8, 0.6, 1.0])),
    )

    waypoints = WAYPOINTS[:4] if args.fast else WAYPOINTS  # fast course still takes both gates
    traj = MinSnapTrajectory(waypoints, avg_speed=2.0)

    params = QuadParams()
    controller = SE3Controller(params)
    state0 = QuadState.hover(waypoints[0])

    T_sim = traj.T + 1.0  # small tail so the quad settles at the finish waypoint
    hist = simulate(params, controller, traj, state0, T_sim, dt=0.002)

    err = hist.pos_error()
    rms_err = float(hist.rms_pos_error())
    max_err = float(np.max(err))
    peak_speed = float(np.max(np.linalg.norm(hist.v, axis=1)))

    print(f"rms_error_m: {rms_err:.4f}")
    print(f"max_error_m: {max_err:.4f}")
    print(f"total_time_s: {traj.T:.3f}")
    print(f"peak_speed_mps: {peak_speed:.3f}")

    path_3d = os.path.join(args.out, "minsnap_3d.png")
    path_tracking = os.path.join(args.out, "minsnap_tracking.png")
    plot_trajectory_3d([hist], obstacles=obstacles, waypoints=waypoints,
                       save=path_3d, title="Min-snap obstacle course: 3D path")
    plot_tracking(hist, save=path_tracking, title="Min-snap obstacle course: tracking")
    print(f"saved: {path_3d}")
    print(f"saved: {path_tracking}")

    if not (args.no_gif or args.fast):
        path_gif = animate_quads([hist], params, obstacles=obstacles,
                                 save=os.path.join(args.out, "minsnap_course.gif"),
                                 title="Min-snap obstacle course")
        print(f"saved: {path_gif}")

    if args.show:
        plt.show()

    if rms_err >= RMS_TARGET:
        print(f"WARN rms_error {rms_err:.4f} m >= target {RMS_TARGET} m")
    elif max_err >= MAX_TARGET:
        print(f"WARN max_error {max_err:.4f} m >= target {MAX_TARGET} m")
    else:
        print("PASS")
    gross = rms_err > 5.0 * RMS_TARGET or max_err > 5.0 * MAX_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
