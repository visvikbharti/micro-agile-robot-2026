"""Tests for quadsim.decentralized: relative sensing, connectivity, dropout, safety."""
from __future__ import annotations

import numpy as np

from quadsim.decentralized import DecentralizedSwarmSim
from quadsim.params import QuadParams
from quadsim.swarm import FormationKeyframe, formation_errors, min_pairwise_distance
from quadsim.trajectory import hover_ref

LEADER_POS = np.array([0.0, 0.0, 1.0])

# Diamond of 4 slots; adjacent slots are 0.5 * sqrt(2) = 0.707 m apart.
OFFSETS4 = 0.5 * np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)
# Deterministic initial perturbations (m) away from the slots.
START4 = np.array(
    [
        [0.3, 0.1, 0.0],
        [-0.1, -0.2, 0.1],
        [0.0, 0.15, -0.1],
        [0.1, 0.0, 0.05],
    ]
)


def _sim4(**kwargs) -> DecentralizedSwarmSim:
    keyframes = [FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=OFFSETS4.copy())]
    return DecentralizedSwarmSim(QuadParams(), hover_ref(LEADER_POS), keyframes, **kwargs)


def test_converges_with_full_connectivity():
    """R_sense covering the whole swarm: perturbed starts converge onto the formation."""
    sim = _sim4(R_sense=5.0)
    histories = sim.run(T=4.0, dt=0.004, start_offsets=START4.copy())
    assert len(histories) == 4
    errs = formation_errors(histories)
    assert errs[-1].max() < 0.02  # empirically ~1e-5 with k_form = 4.0
    rel = sim.relative_formation_errors(histories)
    assert rel[-1] < 0.01  # empirically ~6e-6


def test_disconnected_graph_degrades_formation():
    """R_sense below the 0.707 m slot spacing: no sensing, worse transient shape error."""
    connected = _sim4(R_sense=5.0)
    disconnected = _sim4(R_sense=0.35)  # >= d_safe = 0.30, < slot spacing 0.707
    # at nominal spacing the disconnected sim senses nobody -> zero correction
    slots = LEADER_POS + OFFSETS4
    assert np.allclose(disconnected.formation_accels(slots, 0.0), 0.0, atol=1e-12)
    assert not np.allclose(connected.formation_accels(slots + START4, 0.0), 0.0)
    h_conn = connected.run(T=4.0, dt=0.004, start_offsets=START4.copy())
    h_disc = disconnected.run(T=4.0, dt=0.004, start_offsets=START4.copy())
    t = h_conn[0].t
    transient = t <= 1.5
    rel_conn = float(connected.relative_formation_errors(h_conn)[transient].mean())
    rel_disc = float(disconnected.relative_formation_errors(h_disc)[transient].mean())
    # empirically 0.088 m (connected) vs 0.114 m (disconnected), ratio 1.31
    assert rel_disc > 1.15 * rel_conn


def test_zero_correction_at_exact_formation():
    """Corrective acceleration vanishes when every e_ij is zero, including mid-blend."""
    keyframes = [
        FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=OFFSETS4.copy()),
        FormationKeyframe(t_start=0.5, t_blend=1.0, offsets=2.0 * OFFSETS4),
    ]
    sim = DecentralizedSwarmSim(QuadParams(), hover_ref(LEADER_POS), keyframes, R_sense=5.0)
    for t in (0.0, 0.3, 1.0, 2.0):  # before, during and after the blend
        off, _, _ = sim.blended_offsets(t)
        positions = LEADER_POS + off
        assert np.allclose(sim.formation_accels(positions, t), 0.0, atol=1e-12)


def test_dropout_determinism():
    """Two runs with the same dropout seed are bit-identical; another seed differs."""
    h_a = _sim4(R_sense=5.0, p_drop=0.3, seed=7).run(T=2.0, dt=0.004,
                                                     start_offsets=START4.copy())
    h_b = _sim4(R_sense=5.0, p_drop=0.3, seed=7).run(T=2.0, dt=0.004,
                                                     start_offsets=START4.copy())
    for x, y in zip(h_a, h_b):
        assert np.array_equal(x.p, y.p)
        assert np.array_equal(x.f, y.f)
        assert np.array_equal(x.tau, y.tau)
    h_c = _sim4(R_sense=5.0, p_drop=0.3, seed=8).run(T=2.0, dt=0.004,
                                                     start_offsets=START4.copy())
    assert max(np.abs(x.p - y.p).max() for x, y in zip(h_a, h_c)) > 1e-4


def test_demo_scenario_min_distance():
    """Demo grid -> ring scenario (n=9, R_sense=1.2): separation stays above the floor."""
    spacing = 0.55
    grid = np.array([[ix * spacing, iy * spacing, 0.0]
                     for iy in (-1, 0, 1) for ix in (-1, 0, 1)])
    ang = 2.0 * np.pi * np.arange(9) / 9
    ring = np.column_stack([0.9 * np.cos(ang), 0.9 * np.sin(ang), np.zeros(9)])
    keyframes = [
        FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid),
        FormationKeyframe(t_start=1.5, t_blend=3.0, offsets=ring),
    ]
    sim = DecentralizedSwarmSim(QuadParams(), hover_ref(np.array([0.0, 0.0, 1.2])),
                                keyframes, R_sense=1.2)
    histories = sim.run(T=6.0, dt=0.004)
    assert min_pairwise_distance(histories) > 0.15
    assert formation_errors(histories)[-1].max() < 0.08
