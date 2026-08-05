"""Tests for quadsim.swarm: goal assignment, smoothstep blending, formation flight."""
from __future__ import annotations

import numpy as np

from quadsim.params import QuadParams
from quadsim.swarm import (
    FormationKeyframe,
    SwarmSim,
    min_pairwise_distance,
    formation_errors,
)
from quadsim.trajectory import hover_ref

LEADER_POS = np.array([0.0, 0.0, 1.0])


def _nearest_index(t_arr: np.ndarray, t: float) -> int:
    return int(np.argmin(np.abs(t_arr - t)))


def test_assignment_recovers_permutation():
    """Keyframe offsets that are a pure permutation must leave every slot in place."""
    params = QuadParams()
    leader = hover_ref(LEADER_POS)
    offsets0 = np.array(
        [
            [0.5, 0.5, 0.0],
            [-0.5, 0.5, 0.0],
            [-0.5, -0.5, 0.0],
            [0.5, -0.5, 0.0],
        ]
    )
    perm = np.array([2, 0, 3, 1])
    keyframes = [
        FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=offsets0.copy()),
        FormationKeyframe(t_start=0.2, t_blend=0.4, offsets=offsets0[perm].copy()),
    ]
    swarm = SwarmSim(params, leader, keyframes)
    histories = swarm.run(T=1.0, dt=0.004)
    assert len(histories) == 4
    # optimal assignment on a zero-cost permutation keeps each robot in its own slot,
    # so the per-quad reference never moves
    for i, hist in enumerate(histories):
        slot = LEADER_POS + offsets0[i]
        assert np.max(np.linalg.norm(hist.ref_p - slot, axis=1)) < 1e-8
    errs = formation_errors(histories)
    assert errs[-1].max() < 0.02


def test_smoothstep_blend_c1():
    """Blended offset velocity vanishes at both blend endpoints and matches d/dt ref_p."""
    params = QuadParams()
    leader = hover_ref(LEADER_POS)
    offsets0 = np.array([[0.6, 0.0, 0.0], [-0.6, 0.0, 0.0]])
    offsets1 = offsets0 + np.array([0.0, 0.0, 0.4])
    t_start, t_blend = 0.4, 0.8
    keyframes = [
        FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=offsets0.copy()),
        FormationKeyframe(t_start=t_start, t_blend=t_blend, offsets=offsets1.copy()),
    ]
    histories = SwarmSim(params, leader, keyframes).run(T=1.6, dt=0.004)
    for i, hist in enumerate(histories):
        t = hist.t
        # C^1 endpoints: offset velocity ~ 0 at blend start and end
        assert np.linalg.norm(hist.ref_v[_nearest_index(t, t_start)]) < 1e-3
        assert np.linalg.norm(hist.ref_v[_nearest_index(t, t_start + t_blend)]) < 1e-3
        # the blend actually moves mid-way (peak offset speed = 1.875 * 0.4 / 0.8 m/s)
        assert np.linalg.norm(hist.ref_v[_nearest_index(t, t_start + 0.5 * t_blend)]) > 0.5
        # endpoints of the reference position
        assert np.allclose(hist.ref_p[0], LEADER_POS + offsets0[i], atol=1e-6)
        assert np.allclose(hist.ref_p[-1], LEADER_POS + offsets1[i], atol=1e-6)
        # ref_v consistent with central finite differences of ref_p across the whole run
        fd = (hist.ref_p[2:] - hist.ref_p[:-2]) / (t[2:] - t[:-2])[:, None]
        assert np.max(np.linalg.norm(fd - hist.ref_v[1:-1], axis=1)) < 5e-3


def test_short_run_formation():
    """n=4, 4 s, dt=0.004: terminal formation error and pairwise separation."""
    params = QuadParams()
    leader = hover_ref(LEADER_POS)
    offsets0 = 0.5 * np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )
    c30, s30 = np.cos(np.pi / 6), np.sin(np.pi / 6)
    Rz = np.array([[c30, -s30, 0.0], [s30, c30, 0.0], [0.0, 0.0, 1.0]])
    offsets1 = offsets0 @ Rz.T
    keyframes = [
        FormationKeyframe(t_start=0.0, t_blend=0.2, offsets=offsets0.copy()),
        FormationKeyframe(t_start=0.5, t_blend=1.5, offsets=offsets1.copy()),
    ]
    histories = SwarmSim(params, leader, keyframes).run(T=4.0, dt=0.004)
    assert len(histories) == 4
    errs = formation_errors(histories)
    assert errs.shape[1] == 4
    assert errs[-1].max() < 0.08
    assert min_pairwise_distance(histories) > 0.12
