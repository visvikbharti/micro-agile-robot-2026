"""Cross-engine validation demo: quadsim vs MuJoCo on the same flights.

Flies the SE(3) controller through two experiments in BOTH engines — a hover
recovery from an offset start, and the two-lap figure-eight lemniscate from
demo_figure8 — using identical parameters, controller gains, references, and
control rate. Reports per-engine RMS tracking error plus the cross-engine
position divergence, and renders the MuJoCo flight of the 3D-printed frame
(out/frame_binary.stl) to MP4.

Requires the ``mujoco`` optional dependency (native arm64 Python on Apple
silicon — see docs/MUJOCO.md). Outputs: mujoco_fig8_tracking.png,
mujoco_divergence.png, mujoco_fig8.mp4, mujoco_hover.mp4.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

RMS_TARGET = 0.10        # m: same target as demo_figure8
DIVERGENCE_TARGET = 0.08  # m: RMS cross-engine position divergence


def lemniscate_waypoints(laps: int, points_per_lap: int = 16,
                         a: float = 1.2, z: float = 1.0) -> np.ndarray:
    """Sample a Gerono lemniscate (same geometry as demo_figure8)."""
    theta = np.linspace(0.0, 2.0 * np.pi * laps, laps * points_per_lap + 1)
    x = a * np.sin(theta)
    y = a * np.sin(theta) * np.cos(theta)
    return np.column_stack([x, y, np.full(theta.shape, z)])


def main(argv: Optional[List[str]] = None) -> int:
    """Run the cross-engine validation demo and return the exit code."""
    parser = argparse.ArgumentParser(
        description="quadsim vs MuJoCo: same controller, same reference, two engines.")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true", help="one lap only, skip videos")
    parser.add_argument("--no-video", action="store_true",
                        help="full run, but skip the (slow) MP4 rendering")
    parser.add_argument("--show", action="store_true",
                        help="use an interactive matplotlib backend")
    parser.add_argument("--mesh", default=os.path.join(PROJECT_ROOT, "out", "frame_binary.stl"),
                        help="binary STL for the visual frame ('' to disable)")
    args = parser.parse_args(argv)

    # Backend must be fixed BEFORE pyplot / quadsim.viz are imported.
    if not args.show:
        os.environ["MPLBACKEND"] = "Agg"
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    from quadsim.controller import SE3Controller
    from quadsim.dynamics import QuadState
    from quadsim.mujoco_bridge import render_history, simulate_mujoco
    from quadsim.params import QuadParams
    from quadsim.sim import simulate
    from quadsim.trajectory import MinSnapTrajectory, hover_ref
    from quadsim.viz import plot_tracking

    os.makedirs(args.out, exist_ok=True)
    mesh = args.mesh if args.mesh and os.path.exists(args.mesh) else None
    if args.mesh and mesh is None:
        print(f"note: mesh not found at {args.mesh}; using simple frame geometry")

    params = QuadParams()

    # --- Experiment 1: hover recovery from an offset start -------------------
    p_ref = np.array([0.0, 0.0, 1.0])
    state0 = QuadState.hover(np.array([0.15, -0.1, 0.8]))
    ref = hover_ref(p_ref)
    T_hover = 3.0
    hover_qs = simulate(params, SE3Controller(params), ref, state0, T_hover, dt=0.002)
    hover_mj = simulate_mujoco(params, SE3Controller(params), ref, state0, T_hover,
                               dt=0.002, mesh_path=mesh)
    print("== hover recovery (0.15 m offset, 3 s) ==")
    print(f"quadsim settle RMS (last 1 s): {hover_qs.rms_pos_error(t_from=2.0):.5f} m")
    print(f"mujoco  settle RMS (last 1 s): {hover_mj.rms_pos_error(t_from=2.0):.5f} m")

    # --- Experiment 2: figure-eight lemniscate -------------------------------
    laps = 1 if args.fast else 2
    waypoints = lemniscate_waypoints(laps)
    traj = MinSnapTrajectory(waypoints, avg_speed=2.5, yaw_mode="velocity")
    state0 = QuadState.hover(waypoints[0], yaw=float(traj(0.0).yaw))
    T_sim = traj.T + 0.5
    fig8_qs = simulate(params, SE3Controller(params), traj, state0, T_sim, dt=0.002)
    fig8_mj = simulate_mujoco(params, SE3Controller(params), traj, state0, T_sim,
                              dt=0.002, mesh_path=mesh)

    rms_qs = fig8_qs.rms_pos_error()
    rms_mj = fig8_mj.rms_pos_error()
    div = np.linalg.norm(fig8_mj.p - fig8_qs.p, axis=1)
    div_rms = float(np.sqrt(np.mean(div**2)))
    div_max = float(np.max(div))
    peak_mj = float(np.max(np.linalg.norm(fig8_mj.v, axis=1)))

    print(f"== figure-eight ({laps} lap(s), {traj.T:.2f} s) ==")
    print(f"quadsim rms_error_m: {rms_qs:.4f}   max: {float(np.max(fig8_qs.pos_error())):.4f}")
    print(f"mujoco  rms_error_m: {rms_mj:.4f}   max: {float(np.max(fig8_mj.pos_error())):.4f}")
    print(f"cross-engine divergence rms_m: {div_rms:.4f}   max: {div_max:.4f}")
    print(f"mujoco peak_speed_mps: {peak_mj:.3f}")

    path_tracking = os.path.join(args.out, "mujoco_fig8_tracking.png")
    plot_tracking(fig8_mj, save=path_tracking,
                  title="Figure-eight in MuJoCo: tracking")
    print(f"saved: {path_tracking}")

    fig, ax = plt.subplots(figsize=(8, 3.2), constrained_layout=True)
    ax.plot(fig8_mj.t, 1000.0 * np.linalg.norm(fig8_mj.p - fig8_mj.ref_p, axis=1),
            label="MuJoCo vs reference")
    ax.plot(fig8_qs.t, 1000.0 * np.linalg.norm(fig8_qs.p - fig8_qs.ref_p, axis=1),
            label="quadsim vs reference", alpha=0.8)
    ax.plot(fig8_mj.t, 1000.0 * div, label="MuJoCo vs quadsim", ls="--")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("position error [mm]")
    ax.set_title("Same controller, two physics engines")
    ax.legend()
    path_div = os.path.join(args.out, "mujoco_divergence.png")
    fig.savefig(path_div, dpi=130)
    plt.close(fig)
    print(f"saved: {path_div}")

    if not (args.no_video or args.fast):
        path_mp4 = render_history(fig8_mj, params, os.path.join(args.out, "mujoco_fig8.mp4"),
                                  mesh_path=mesh)
        print(f"saved: {path_mp4}")
        path_mp4 = render_history(hover_mj, params, os.path.join(args.out, "mujoco_hover.mp4"),
                                  mesh_path=mesh)
        print(f"saved: {path_mp4}")

    if args.show:
        plt.show()

    ok = rms_mj < RMS_TARGET and div_rms < DIVERGENCE_TARGET
    if ok:
        print("PASS")
    else:
        print(f"WARN rms {rms_mj:.4f} (target {RMS_TARGET}) "
              f"divergence {div_rms:.4f} (target {DIVERGENCE_TARGET})")
    gross = rms_mj > 5.0 * RMS_TARGET or div_rms > 5.0 * DIVERGENCE_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
