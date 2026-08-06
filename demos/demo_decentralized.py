"""Nine-robot decentralized formation demo (ch08): relative sensing vs the centralized swarm.

Timeline (~11 s, dt = 0.004): hold a 3x3 grid (0.55 m spacing) at z = 1.2, blend
to a 9-robot ring (r = 0.9), then the leader flies one smooth circle lap (r = 0.7,
C^2 smoothstep-ramped as in demo_swarm). The SAME scenario is flown three ways:
SwarmSim (centralized baseline), DecentralizedSwarmSim (relative sensing within
R_sense only), and DecentralizedSwarmSim with 30% per-edge sensing dropout. With
implicit coordination — every robot carries the shared plan — the decentralized
metrics should match the centralized ones almost exactly; that agreement is the
Turpin, Michael & Kumar (2011) point, not a bug. Metrics per variant: steady
formation RMS (target 0.06 m), max formation error, min pairwise distance (must
stay > 0.15 m), relative formation RMS (shape error from pairwise e_ij). Outputs:
decentralized_3d.png, decentralized.gif. --fast: 6 s, grid -> ring only, leader
hovers, skip GIF.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Callable, List, Optional

import numpy as np

from quadsim.trajectory import FlatOutput

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RMS_TARGET = 0.06      # m, steady formation RMS target
MIN_DIST_FLOOR = 0.15  # m, min pairwise distance must stay above this
N_QUADS = 9
# Sensing radius: nearest-neighbor slot spacing is 0.55 m in the grid and 0.62 m on
# the ring (chord 2*0.9*sin(pi/9)). R_SENSE = 1.2 m is ~2x that, so the sensing graph
# stays connected through the blend, yet on the ring (diameter 1.8 m) each robot only
# senses the two nearest ring mates per side (chords 0.62 and 1.16 m; the next chord,
# 1.56 m, is out of range) — sensing stays genuinely local.
R_SENSE = 1.2
P_DROP = 0.3           # per-edge, per-step sensing dropout for the lossy variant


def grid_offsets(spacing: float = 0.55) -> np.ndarray:
    """3x3 grid of slot offsets (z = 0) centered on the leader; min spacing = `spacing`."""
    pts = []
    for iy in (-1, 0, 1):
        for ix in (-1, 0, 1):
            pts.append([ix * spacing, iy * spacing, 0.0])
    return np.array(pts)


def ring_offsets(radius: float = 0.9, n: int = N_QUADS) -> np.ndarray:
    """Ring of n slots (z = 0); chord spacing 2*r*sin(pi/n) = 0.62 m for r=0.9, n=9."""
    ang = 2.0 * np.pi * np.arange(n) / n
    return np.column_stack([radius * np.cos(ang), radius * np.sin(ang), np.zeros(n)])


def make_leader(t0: float, t_circle: float,
                radius: float = 0.7) -> Callable[[float], FlatOutput]:
    """C^2 leader reference: hover at (0, 0, 1.2) until t0, then one smooth circle lap.

    The circle angle is theta(t) = 2*pi * s(u) with u = (t - t0) / t_circle clamped
    to [0, 1] and s the quintic smoothstep 6u^5 - 15u^4 + 10u^3, so velocity and
    acceleration are zero at both ends of the lap and the reference is C^2 in time.
    """
    p_hover = np.array([0.0, 0.0, 1.2])
    center = p_hover - np.array([radius, 0.0, 0.0])  # circle starts/ends at p_hover
    two_pi = 2.0 * np.pi

    def leader(t: float) -> FlatOutput:
        if t <= t0:
            return FlatOutput(pos=p_hover.copy())
        u = min((t - t0) / t_circle, 1.0)
        s = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
        ds = (30.0 * u**4 - 60.0 * u**3 + 30.0 * u**2) / t_circle
        dds = (120.0 * u**3 - 180.0 * u**2 + 60.0 * u) / t_circle**2
        th = two_pi * s
        thd = two_pi * ds
        thdd = two_pi * dds
        c, sn = np.cos(th), np.sin(th)
        pos = center + radius * np.array([c, sn, 0.0])
        vel = radius * thd * np.array([-sn, c, 0.0])
        acc = radius * (thdd * np.array([-sn, c, 0.0]) + thd**2 * np.array([-c, -sn, 0.0]))
        return FlatOutput(pos=pos, vel=vel, acc=acc)

    return leader


def main(argv: Optional[List[str]] = None) -> int:
    """Run the decentralized demo and return the exit code (1 only on gross failure)."""
    parser = argparse.ArgumentParser(
        description="Nine-quad decentralized formation: grid -> ring -> circle lap, "
                    "centralized vs relative-sensing vs lossy-sensing.")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true",
                        help="6 s run, grid -> ring only, skip GIF")
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

    from quadsim.decentralized import DecentralizedSwarmSim
    from quadsim.params import QuadParams
    from quadsim.swarm import (FormationKeyframe, SwarmSim, formation_errors,
                               min_pairwise_distance)
    from quadsim.viz import animate_quads, plot_trajectory_3d

    os.makedirs(args.out, exist_ok=True)

    params = QuadParams()

    if args.fast:
        T = 6.0
        keyframes = [
            FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
            FormationKeyframe(t_start=1.5, t_blend=3.0, offsets=ring_offsets()),
        ]
        leader = make_leader(t0=T + 5.0, t_circle=5.0)  # leader hovers for the whole run
    else:
        T = 11.0
        keyframes = [
            FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
            FormationKeyframe(t_start=2.0, t_blend=3.0, offsets=ring_offsets()),
        ]
        leader = make_leader(t0=5.5, t_circle=5.0)  # circle lap over t in [5.5, 10.5]

    dec = DecentralizedSwarmSim(params, leader, keyframes, R_sense=R_SENSE)
    variants = [
        ("centralized", SwarmSim(params, leader, keyframes)),
        ("decentralized", dec),
        ("dec_dropout", DecentralizedSwarmSim(params, leader, keyframes, R_sense=R_SENSE,
                                              p_drop=P_DROP, seed=0)),
    ]

    print(f"n_quads: {N_QUADS}")
    print(f"duration_s: {T:.1f}")
    print(f"R_sense_m: {R_SENSE}")
    print(f"p_drop: {P_DROP}")

    dec_histories = None
    dec_steady_rms = dec_min_dist = 0.0
    for name, sim in variants:
        histories = sim.run(T, dt=0.004)
        errs = formation_errors(histories)  # (N, n)
        t = histories[0].t
        # "Steady" excludes each blend plus a short settling margin after it.
        steady_mask = np.ones(t.shape, dtype=bool)
        for kf in keyframes:
            steady_mask &= ~((t >= kf.t_start) & (t < kf.t_start + kf.t_blend + 1.5))
        steady_rms = float(np.sqrt(np.mean(errs[steady_mask] ** 2)))
        max_err = float(np.max(errs))
        min_dist = float(min_pairwise_distance(histories))
        rel = dec.relative_formation_errors(histories)  # same keyframes/assignment
        rel_rms = float(np.sqrt(np.mean(rel[steady_mask] ** 2)))
        # 6 decimals: the centralized-vs-decentralized gap is sub-millimeter by design.
        print(f"{name}_steady_formation_rms_m: {steady_rms:.6f}")
        print(f"{name}_max_formation_error_m: {max_err:.6f}")
        print(f"{name}_min_pairwise_distance_m: {min_dist:.4f}")
        print(f"{name}_relative_rms_m: {rel_rms:.6f}")
        if name == "decentralized":
            dec_histories = histories
            dec_steady_rms = steady_rms
            dec_min_dist = min_dist

    path_3d = os.path.join(args.out, "decentralized_3d.png")
    plot_trajectory_3d(dec_histories, save=path_3d,
                       title="Decentralized nine-quad swarm: grid -> ring -> circle lap")
    print(f"saved: {path_3d}")

    if not (args.no_gif or args.fast):
        path_gif = animate_quads(dec_histories, params,
                                 save=os.path.join(args.out, "decentralized.gif"),
                                 title="Decentralized formation flight (relative sensing)")
        print(f"saved: {path_gif}")

    if args.show:
        plt.show()

    if dec_steady_rms >= RMS_TARGET:
        print(f"WARN decentralized steady_formation_rms {dec_steady_rms:.4f} m "
              f">= target {RMS_TARGET} m")
    elif dec_min_dist <= MIN_DIST_FLOOR:
        print(f"WARN decentralized min_pairwise_distance {dec_min_dist:.4f} m "
              f"<= floor {MIN_DIST_FLOOR} m")
    else:
        print("PASS")
    gross = dec_steady_rms > 5.0 * RMS_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
