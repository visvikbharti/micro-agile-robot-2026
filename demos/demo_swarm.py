"""Nine-robot swarm formation demo (a nod to the TED talk's nine-robot finale).

Timeline (~30 s, dt = 0.004): hold a 3x3 grid (0.55 m spacing) at z = 1.2,
blend to a 9-robot ring (r = 0.9), blend to a "V" of migrating-birds chevron
slots, then the leader flies one smooth circle lap (r = 0.7) with the V in tow.
The leader reference is C^2: hover until t0, then an analytic circle whose
angle is ramped by a quintic smoothstep, so velocity and acceleration are
continuous everywhere. Metrics: steady formation RMS (target 0.06 m), max
formation error, min pairwise distance (must stay > 0.15 m). Outputs:
swarm_3d.png, swarm.gif. --fast: 10 s, grid -> ring only.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Callable, List, Optional

import numpy as np

from quadsim.trajectory import FlatOutput

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RMS_TARGET = 0.06     # m, steady formation RMS target
MIN_DIST_FLOOR = 0.15  # m, min pairwise distance must stay above this
N_QUADS = 9


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


def v_offsets(dx: float = 0.35, dy: float = 0.35) -> np.ndarray:
    """Chevron / "V" of 9 slots like migrating birds: apex forward (+x), wings swept back.

    Adjacent slots along a wing are sqrt(dx^2 + dy^2) = 0.495 m apart (>= 0.4 m);
    the closest cross-wing pair (k = 1) is 2*dy = 0.70 m apart.
    """
    pts = [[0.7, 0.0, 0.0]]
    for k in range(1, 5):
        pts.append([0.7 - k * dx, +k * dy, 0.0])
        pts.append([0.7 - k * dx, -k * dy, 0.0])
    return np.array(pts)


def make_leader(t0: float, t_circle: float,
                radius: float = 0.7) -> Callable[[float], FlatOutput]:
    """C^2 leader reference: hover at (0, 0, 1.2) until t0, then one smooth circle lap.

    The circle angle is theta(t) = 2*pi * s(u) with u = (t - t0) / t_circle clamped
    to [0, 1] and s the quintic smoothstep 6u^5 - 15u^4 + 10u^3. Since
    s'(0) = s''(0) = s'(1) = s''(1) = 0, velocity and acceleration are zero at both
    ends of the lap, matching the hover phases: the reference is C^2 in time.
    Velocity/acceleration are analytic via the chain rule.
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
    """Run the swarm demo and return the exit code (1 only on gross failure)."""
    parser = argparse.ArgumentParser(
        description="Nine-quad swarm: grid -> ring -> V, then a leader circle lap.")
    parser.add_argument("--out", default=os.path.join(PROJECT_ROOT, "out"),
                        help="output directory (default: out/ under the project root)")
    parser.add_argument("--fast", action="store_true",
                        help="10 s run, grid -> ring only, skip GIF")
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

    from quadsim.params import QuadParams
    from quadsim.swarm import (FormationKeyframe, SwarmSim, formation_errors,
                               min_pairwise_distance)
    from quadsim.viz import animate_quads, plot_trajectory_3d

    os.makedirs(args.out, exist_ok=True)

    params = QuadParams()

    if args.fast:
        T = 10.0
        keyframes = [
            FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
            FormationKeyframe(t_start=3.0, t_blend=3.0, offsets=ring_offsets()),
        ]
        leader = make_leader(t0=T + 5.0, t_circle=10.0)  # leader hovers for the whole run
    else:
        T = 30.0
        keyframes = [
            FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
            FormationKeyframe(t_start=6.0, t_blend=4.0, offsets=ring_offsets()),
            FormationKeyframe(t_start=12.0, t_blend=4.0, offsets=v_offsets()),
        ]
        leader = make_leader(t0=18.0, t_circle=10.0)  # circle lap over t in [18, 28]

    swarm = SwarmSim(params, leader, keyframes)
    histories = swarm.run(T, dt=0.004)

    errs = formation_errors(histories)  # (N, n)
    t = histories[0].t
    # "Steady" excludes each blend plus a short settling margin after it.
    steady_mask = np.ones(t.shape, dtype=bool)
    for kf in keyframes:
        steady_mask &= ~((t >= kf.t_start) & (t < kf.t_start + kf.t_blend + 1.5))
    steady_rms = float(np.sqrt(np.mean(errs[steady_mask] ** 2)))
    max_err = float(np.max(errs))
    min_dist = float(min_pairwise_distance(histories))

    print(f"n_quads: {N_QUADS}")
    print(f"duration_s: {T:.1f}")
    print(f"steady_formation_rms_m: {steady_rms:.4f}")
    print(f"max_formation_error_m: {max_err:.4f}")
    print(f"min_pairwise_distance_m: {min_dist:.4f}")

    path_3d = os.path.join(args.out, "swarm_3d.png")
    plot_trajectory_3d(histories, save=path_3d,
                       title="Nine-quad swarm: grid -> ring -> V -> circle lap")
    print(f"saved: {path_3d}")

    if not (args.no_gif or args.fast):
        path_gif = animate_quads(histories, params,
                                 save=os.path.join(args.out, "swarm.gif"),
                                 title="Nine-quad swarm formation flight")
        print(f"saved: {path_gif}")

    if args.show:
        plt.show()

    if steady_rms >= RMS_TARGET:
        print(f"WARN steady_formation_rms {steady_rms:.4f} m >= target {RMS_TARGET} m")
    elif min_dist <= MIN_DIST_FLOOR:
        print(f"WARN min_pairwise_distance {min_dist:.4f} m <= floor {MIN_DIST_FLOOR} m")
    else:
        print("PASS")
    gross = steady_rms > 5.0 * RMS_TARGET
    return 1 if gross else 0


if __name__ == "__main__":
    sys.exit(main())
