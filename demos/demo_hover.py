"""Hover recovery demo.

Start 0.4 m away from the hover target (0, 0, 1.0) and let the SE(3) controller
recover; 6 s run. Reports settle time to < 2 cm position error and the
steady-state RMS error over the last 2 s (target < 0.005 m).
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SETTLE_THRESHOLD = 0.02    # m, "settled" means position error stays below this
STEADY_RMS_TARGET = 0.005  # m, steady-state RMS target over the tail window


def main(argv: Optional[List[str]] = None) -> int:
    """Run the hover demo and return the process exit code (1 only on gross failure)."""
    parser = argparse.ArgumentParser(description="Hover recovery demo (0.4 m initial offset).")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true",
                        help="shorter run, skip GIF generation")
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
    from quadsim.trajectory import hover_ref
    from quadsim.viz import plot_tracking, plot_trajectory_3d

    os.makedirs(args.out, exist_ok=True)

    params = QuadParams()
    controller = SE3Controller(params)

    target = np.array([0.0, 0.0, 1.0])
    # Initial offset with ||offset|| = 0.4 m, split over all three axes.
    offset = 0.4 * np.array([1.0, -1.0, 1.0]) / np.sqrt(3.0)
    state0 = QuadState.hover(target + offset)
    ref_fn = hover_ref(target)

    # --fast: shorter run with a proportionally shorter steady-state window.
    T = 3.0 if args.fast else 6.0
    steady_window = 1.0 if args.fast else 2.0
    hist = simulate(params, controller, ref_fn, state0, T, dt=0.002)

    err = hist.pos_error()
    above = np.nonzero(err >= SETTLE_THRESHOLD)[0]
    if above.size == 0:
        settle_time = float(hist.t[0])
    elif above[-1] + 1 < hist.t.size:
        settle_time = float(hist.t[above[-1] + 1])
    else:
        settle_time = float("nan")  # never settled
    steady_rms = float(hist.rms_pos_error(t_from=T - steady_window))

    print(f"settle_time_s: {settle_time:.3f}")
    print(f"steady_rms_m: {steady_rms:.5f}")
    print(f"final_error_m: {float(err[-1]):.5f}")

    path_3d = os.path.join(args.out, "hover_3d.png")
    path_tracking = os.path.join(args.out, "hover_tracking.png")
    plot_trajectory_3d([hist], waypoints=np.vstack([target + offset, target]),
                       save=path_3d, title="Hover recovery: 3D path")
    plot_tracking(hist, save=path_tracking, title="Hover recovery: tracking")
    print(f"saved: {path_3d}")
    print(f"saved: {path_tracking}")

    if args.show:
        plt.show()

    if np.isnan(settle_time):
        print(f"WARN position error never settled below {SETTLE_THRESHOLD} m")
    elif steady_rms >= STEADY_RMS_TARGET:
        print(f"WARN steady_rms {steady_rms:.5f} m >= target {STEADY_RMS_TARGET} m")
    else:
        print("PASS")
    gross = steady_rms > 5.0 * STEADY_RMS_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
