"""Decentralized formation control from relative-position sensing only.

In the style of Turpin, Michael & Kumar (2011), "Trajectory design and control for
aggressive formation flight with quadrotors": every robot knows the shared leader
flat-output plan and the pre-assigned slot offsets (implicit coordination -- everyone
carries the same plan), but senses only the RELATIVE positions ``x_i - x_j`` of
neighbors within a sensing radius ``R_sense``. No robot reads another robot's absolute
state or velocity. The pairwise formation error ``e_ij = (x_i - x_j) - (d_i - d_j)``
drives a corrective acceleration ``a_i = -k_form * mean_j e_ij`` (norm-capped at
``a_form_max``), injected into robot i's reference acceleration exactly the way
``SwarmSim`` injects collision avoidance.

Honest scope: the CONTROL is decentralized -- each robot's corrective term uses only
its own state plus relative neighbor positions. Slot offsets are pre-assigned at
construction (the anonymous goal-assignment problem is ch05 / CAPT territory), and
ground-truth relative positions from the simulator stand in for onboard relative
sensing. The pairwise repulsion reused from ``swarm`` also needs only relative
positions, since it acts within ``d_safe <= R_sense``.
"""
from __future__ import annotations

from typing import Callable, List, Optional

import numpy as np

from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadrotorDynamics, QuadState
from quadsim.params import QuadParams
from quadsim.sim import History
from quadsim.swarm import FormationKeyframe, SwarmSim
from quadsim.trajectory import FlatOutput


class DecentralizedSwarmSim(SwarmSim):
    """Lockstep swarm simulation whose corrective term uses relative sensing only.

    Inherits keyframe validation, sequential optimal slot assignment, smoothstep
    offset blending and pairwise repulsion from ``SwarmSim``; adds a consensus-style
    formation correction from the relative positions of neighbors within ``R_sense``,
    with optional per-edge sensing dropout (deterministic under ``seed``).
    """

    def __init__(self, params: QuadParams, leader: Callable[[float], FlatOutput],
                 keyframes: List[FormationKeyframe], R_sense: float = 1.2,
                 k_form: float = 4.0, a_form_max: float = 3.0, d_safe: float = 0.30,
                 k_avoid: float = 4.0, a_avoid_max: float = 3.0,
                 p_drop: float = 0.0, seed: int = 0) -> None:
        """Build the decentralized swarm.

        Args:
            params: Quadrotor parameters shared by all quads.
            leader: Flat-output reference ``t -> FlatOutput``; must be C^2 in time.
            keyframes: Formation keyframes; the first must have ``t_start == 0``.
            R_sense: Sensing radius (m); only neighbors closer than this are used.
            k_form: Formation stiffness (s^-2) on the mean pairwise error. Default
                4.0 is a quarter of the position-loop stiffness ``kp/m = 16 s^-2``,
                so the added spring (which brings no extra relative damping) stays
                well damped.
            a_form_max: Cap (m/s^2) on the norm of the formation correction; default
                3.0 matches ``SwarmSim``'s ``a_avoid_max`` so both injected terms
                stay well inside the ~1 g of thrust margin (thrust/weight ~ 2.0).
            d_safe: Pairwise distance (m) below which repulsion activates; must not
                exceed ``R_sense`` (repulsion needs the relative position too).
            k_avoid: Repulsion gain (m^2/s^2), as in ``SwarmSim``.
            a_avoid_max: Cap (m/s^2) on the total repulsion accel, as in ``SwarmSim``.
            p_drop: Per-step probability in ``[0, 1]`` that a directed sensing edge
                i->j drops out (lossy sensing model).
            seed: Seed for the dropout generator; ``run`` re-creates the generator
                from it, so repeated runs are identical.
        """
        super().__init__(params, leader, keyframes, d_safe=d_safe, k_avoid=k_avoid,
                         a_avoid_max=a_avoid_max)
        if R_sense <= 0.0:
            raise ValueError("R_sense must be positive")
        if self.d_safe > R_sense:
            raise ValueError("d_safe must not exceed R_sense: repulsion is computed "
                             "from sensed relative positions")
        if not 0.0 <= p_drop <= 1.0:
            raise ValueError("p_drop must be in [0, 1]")
        self.R_sense = float(R_sense)
        self.k_form = float(k_form)
        self.a_form_max = float(a_form_max)
        self.p_drop = float(p_drop)
        self.seed = int(seed)

    def formation_accels(self, positions: np.ndarray, t: float,
                         rng: Optional[np.random.Generator] = None) -> np.ndarray:
        """Per-quad formation-correction accelerations from relative positions.

        For robot i, over neighbors j with ``||x_i - x_j|| <= R_sense`` (each
        directed edge independently dropped with probability ``p_drop`` when ``rng``
        is given; ``run`` passes its own generator):
        ``a_i = -k_form * mean_j [(x_i - x_j) - (d_i - d_j)]``, norm-capped at
        ``a_form_max``. Robots with no sensed neighbor get zero correction. The
        offsets ``d`` are the blended (assigned) slot offsets at time ``t``.
        """
        acc = np.zeros((self.n, 3))
        if self.n < 2:
            return acc
        off, _, _ = self.blended_offsets(t)
        diff = positions[:, None, :] - positions[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        sensed = dist <= self.R_sense
        np.fill_diagonal(sensed, False)
        if rng is not None and self.p_drop > 0.0:
            sensed &= rng.random((self.n, self.n)) >= self.p_drop
        err = diff - (off[:, None, :] - off[None, :, :])
        for i in range(self.n):
            idx = np.nonzero(sensed[i])[0]
            if idx.size == 0:
                continue
            a_i = -self.k_form * err[i, idx].mean(axis=0)
            norm = float(np.linalg.norm(a_i))
            if norm > self.a_form_max:
                a_i *= self.a_form_max / norm
            acc[i] = a_i
        return acc

    def relative_formation_errors(self, histories: List[History]) -> np.ndarray:
        """``(N,)`` per-sample RMS over all pairs of ``||e_ij||``.

        ``e_ij(t) = (p_i - p_j) - (d_i - d_j)`` with ``d`` the blended assigned
        offsets: the formation SHAPE error, independent of absolute leader tracking,
        which is the quantity the decentralized corrective term actually regulates.
        """
        t_arr = histories[0].t
        ps = np.stack([h.p for h in histories], axis=0)
        offs = np.stack([self.blended_offsets(float(t))[0] for t in t_arr], axis=0)
        sq_sum = np.zeros(t_arr.shape[0])
        n_pairs = 0
        for i in range(self.n):
            for j in range(i + 1, self.n):
                e_ij = (ps[i] - ps[j]) - (offs[:, i, :] - offs[:, j, :])
                sq_sum += (e_ij ** 2).sum(axis=1)
                n_pairs += 1
        return np.sqrt(sq_sum / max(n_pairs, 1))

    def run(self, T: float, dt: float = 0.004,
            start_offsets: Optional[np.ndarray] = None) -> List[History]:
        """Simulate all quads in lockstep for ``T`` seconds; one History per quad.

        Mirrors ``SwarmSim.run`` with the formation correction added to the
        repulsion term; recorded ``ref_p``/``ref_v`` are each quad's formation
        reference WITHOUT either injected term. ``start_offsets`` (``(n, 3)``,
        optional) perturbs the initial hover positions away from the slots. The
        dropout generator is re-seeded from ``seed`` on every call, so repeated
        runs are identical.
        """
        n_steps = int(round(T / dt))
        num = n_steps + 1
        dyn = QuadrotorDynamics(self.params)
        controllers = [SE3Controller(self.params) for _ in range(self.n)]
        rng = np.random.default_rng(self.seed)
        base0 = self.leader(0.0)
        p0_extra = (np.zeros((self.n, 3)) if start_offsets is None
                    else np.asarray(start_offsets, dtype=float))
        if p0_extra.shape != (self.n, 3):
            raise ValueError("start_offsets must have shape (n, 3)")
        states = [QuadState.hover(base0.pos + self._offsets[0][i] + p0_extra[i],
                                  yaw=base0.yaw) for i in range(self.n)]
        t_arr = np.arange(num) * dt
        p_arr = np.zeros((self.n, num, 3))
        v_arr = np.zeros((self.n, num, 3))
        q_arr = np.zeros((self.n, num, 4))
        w_arr = np.zeros((self.n, num, 3))
        f_arr = np.zeros((self.n, num))
        tau_arr = np.zeros((self.n, num, 3))
        rp_arr = np.zeros((self.n, num, 3))
        rv_arr = np.zeros((self.n, num, 3))
        for k in range(num):
            t = float(t_arr[k])
            positions = np.array([st.p for st in states])
            a_corr = (self._avoidance_accels(positions)
                      + self.formation_accels(positions, t, rng))
            base = self.leader(t)
            off, doff, _ = self.blended_offsets(t)
            new_states = []
            for i in range(self.n):

                def ref_fn(s: float, i: int = i, a_i: np.ndarray = a_corr[i]) -> FlatOutput:
                    return self._formation_ref(i, s, a_i)

                f_cmd, tau_cmd = controllers[i].compute(states[i], ref_fn, t)
                f_act, tau_act = dyn.unmix(dyn.mix(f_cmd, tau_cmd))
                st = states[i]
                p_arr[i, k] = st.p
                v_arr[i, k] = st.v
                q_arr[i, k] = st.q
                w_arr[i, k] = st.w
                f_arr[i, k] = f_act
                tau_arr[i, k] = tau_act
                rp_arr[i, k] = base.pos + off[i]
                rv_arr[i, k] = base.vel + doff[i]
                if k < n_steps:
                    new_states.append(dyn.step(st, f_cmd, tau_cmd, dt))
            if k < n_steps:
                states = new_states
        return [History(t=t_arr.copy(), p=p_arr[i], v=v_arr[i], q=q_arr[i], w=w_arr[i],
                        f=f_arr[i], tau=tau_arr[i], ref_p=rp_arr[i], ref_v=rv_arr[i])
                for i in range(self.n)]
