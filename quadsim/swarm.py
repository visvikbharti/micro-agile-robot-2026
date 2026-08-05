"""Swarm formation flight with optimal goal assignment and smoothstep blending.

Formation slots are offsets relative to a leader flat-output reference. Consecutive
formations are matched with optimal concurrent goal assignment (Kushleyev et al. 2013;
Turpin, Michael & Kumar), blended with a quintic smoothstep, and flown by one SE(3)
controller per quad with a reciprocal collision-avoidance acceleration term.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadrotorDynamics, QuadState
from quadsim.params import QuadParams
from quadsim.sim import History
from quadsim.trajectory import FlatOutput


@dataclass
class FormationKeyframe:
    """A formation of slot offsets and the timing of the transition into it.

    Attributes:
        t_start: Time (s) when the transition to these offsets begins.
        t_blend: Blend duration (s) of the smoothstep transition.
        offsets: ``(n, 3)`` formation slot offsets (m) relative to the leader.
    """

    t_start: float
    t_blend: float
    offsets: np.ndarray


def _smoothstep(u: float) -> Tuple[float, float, float]:
    """Quintic smoothstep and its first two analytic derivatives at ``u``.

    ``s(u) = 6u^5 - 15u^4 + 10u^3`` with ``s(0) = 0``, ``s(1) = 1`` and both
    ``s'`` and ``s''`` vanishing at ``u = 0`` and ``u = 1``.
    """
    s = ((6.0 * u - 15.0) * u + 10.0) * u ** 3
    ds = ((30.0 * u - 60.0) * u + 30.0) * u ** 2
    dds = ((120.0 * u - 180.0) * u + 60.0) * u
    return s, ds, dds


def smoothstep(u: float) -> float:
    """Quintic smoothstep ``s(u) = 6u^5 - 15u^4 + 10u^3`` for ``u`` in ``[0, 1]``.

    ``s(0) = 0``, ``s(1) = 1``; the first and second derivatives vanish at both
    endpoints, making blends that use it C^2 in time.
    """
    return _smoothstep(u)[0]


class SwarmSim:
    """Lockstep simulation of ``n`` quadrotors holding formation about a leader.

    The slot assignment between consecutive keyframes is resolved once at
    construction, sequentially: each keyframe's offset rows are permuted by
    ``scipy.optimize.linear_sum_assignment`` on squared distances from the
    previous (already-permuted) offsets, so each robot flies to the nearest
    available slot.
    """

    def __init__(self, params: QuadParams, leader: Callable[[float], FlatOutput],
                 keyframes: List[FormationKeyframe], d_safe: float = 0.30,
                 k_avoid: float = 4.0, a_avoid_max: float = 3.0) -> None:
        """Build the swarm.

        Args:
            params: Quadrotor parameters shared by all quads.
            leader: Flat-output reference ``t -> FlatOutput``; must be C^2 in time.
            keyframes: Formation keyframes; the first must have ``t_start == 0``.
            d_safe: Pairwise distance (m) below which avoidance activates.
            k_avoid: Avoidance gain (m^2/s^2).
            a_avoid_max: Cap (m/s^2) on the norm of the total avoidance accel.
        """
        if len(keyframes) == 0:
            raise ValueError("at least one FormationKeyframe is required")
        if keyframes[0].t_start != 0.0:
            raise ValueError("first keyframe must have t_start == 0")
        first = np.asarray(keyframes[0].offsets, dtype=float)
        if first.ndim != 2 or first.shape[1] != 3:
            raise ValueError("keyframe offsets must have shape (n, 3)")
        self.params = params
        self.leader = leader
        self.keyframes = list(keyframes)
        self.d_safe = float(d_safe)
        self.k_avoid = float(k_avoid)
        self.a_avoid_max = float(a_avoid_max)
        self.n = int(first.shape[0])
        self._offsets: List[np.ndarray] = [first]
        for kf in self.keyframes[1:]:
            cand = np.asarray(kf.offsets, dtype=float)
            if cand.shape != first.shape:
                raise ValueError("all keyframes must share the offsets shape (n, 3)")
            prev = self._offsets[-1]
            cost = ((prev[:, None, :] - cand[None, :, :]) ** 2).sum(axis=2)
            _, col = linear_sum_assignment(cost)
            self._offsets.append(cand[col])

    def blended_offsets(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Formation offsets and their time derivatives at time ``t``.

        Returns ``(offset, d_offset, dd_offset)``, each ``(n, 3)``: the smoothstep
        blend between the assigned offsets of consecutive keyframes, with velocity
        and acceleration from the analytic smoothstep derivatives.
        """
        off = self._offsets[0]
        vel = np.zeros((self.n, 3))
        acc = np.zeros((self.n, 3))
        for k in range(1, len(self._offsets)):
            kf = self.keyframes[k]
            if t >= kf.t_start + kf.t_blend:
                off = self._offsets[k]
                continue
            if t > kf.t_start and kf.t_blend > 0.0:
                delta = self._offsets[k] - self._offsets[k - 1]
                s, ds, dds = _smoothstep((t - kf.t_start) / kf.t_blend)
                off = self._offsets[k - 1] + s * delta
                vel = (ds / kf.t_blend) * delta
                acc = (dds / (kf.t_blend * kf.t_blend)) * delta
            break
        return np.array(off), vel, acc

    def _formation_ref(self, i: int, t: float, a_extra: np.ndarray) -> FlatOutput:
        """Reference for quad ``i``: leader shifted by its blended offset.

        ``a_extra`` (held constant in ``t``) is added to the acceleration.
        """
        base = self.leader(t)
        off, doff, ddoff = self.blended_offsets(t)
        return FlatOutput(pos=base.pos + off[i], vel=base.vel + doff[i],
                          acc=base.acc + ddoff[i] + a_extra,
                          jerk=np.array(base.jerk), snap=np.array(base.snap),
                          yaw=base.yaw, yaw_rate=base.yaw_rate)

    def _avoidance_accels(self, positions: np.ndarray) -> np.ndarray:
        """Per-quad collision-avoidance accelerations from current positions.

        For each pair closer than ``d_safe``, ``k_avoid * (1/d_ij - 1/d_safe)``
        pushes the pair apart along the separating unit vector; each quad's total
        is capped in norm at ``a_avoid_max``.
        """
        acc = np.zeros((self.n, 3))
        for i in range(self.n):
            for j in range(i + 1, self.n):
                diff = positions[i] - positions[j]
                d_ij = float(np.linalg.norm(diff))
                if d_ij >= self.d_safe or d_ij < 1e-9:
                    continue
                push = self.k_avoid * (1.0 / d_ij - 1.0 / self.d_safe) * (diff / d_ij)
                acc[i] += push
                acc[j] -= push
        for i in range(self.n):
            norm = float(np.linalg.norm(acc[i]))
            if norm > self.a_avoid_max:
                acc[i] *= self.a_avoid_max / norm
        return acc

    def run(self, T: float, dt: float = 0.004) -> List[History]:
        """Simulate all quads in lockstep for ``T`` seconds; one History per quad.

        Recorded ``ref_p``/``ref_v`` are each quad's formation reference WITHOUT
        the avoidance term; ``f``/``tau`` are the saturated values applied.
        """
        n_steps = int(round(T / dt))
        num = n_steps + 1
        dyn = QuadrotorDynamics(self.params)
        controllers = [SE3Controller(self.params) for _ in range(self.n)]
        base0 = self.leader(0.0)
        states = [QuadState.hover(base0.pos + self._offsets[0][i], yaw=base0.yaw)
                  for i in range(self.n)]
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
            a_avoid = self._avoidance_accels(positions)
            base = self.leader(t)
            off, doff, _ = self.blended_offsets(t)
            new_states = []
            for i in range(self.n):

                def ref_fn(s: float, i: int = i, a_i: np.ndarray = a_avoid[i]) -> FlatOutput:
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


def min_pairwise_distance(histories: List[History]) -> float:
    """Minimum over time of the minimum pairwise distance between quads (m)."""
    n = len(histories)
    if n < 2:
        return float("inf")
    ps = np.stack([h.p for h in histories], axis=0)
    best = float("inf")
    for i in range(n):
        for j in range(i + 1, n):
            d_ij = float(np.linalg.norm(ps[i] - ps[j], axis=1).min())
            best = min(best, d_ij)
    return best


def formation_errors(histories: List[History]) -> np.ndarray:
    """``(N, n)`` array of ``||p_i - ref_p_i||`` per time sample and quad."""
    errs = [np.linalg.norm(h.p - h.ref_p, axis=1) for h in histories]
    return np.stack(errs, axis=1)
