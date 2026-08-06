# Chapter 8 — Decentralized swarms: nobody in charge, everybody with the plan

There is a moment in Kumar's TED talk where the whole lecture pivots. The robots, he says, compute their control a hundred times a second from what they *sense from their neighbors* — and he puts up a slide with one small equation on it, the pairwise formation error $e_{ij} = x_i - x_j - s_{des}$, citing [Turpin, Michael & Kumar 2011]. Two arguments carry the moment. **Scale**: a central computer coordinating every robot cannot keep up as the swarm grows, so each robot must fly itself from local information. **Anonymity**: the robots are interchangeable — robot $i$ does not care *which* robot is next to it, only *where* it is. Chapter 5 built a nine-robot swarm and then confessed, in its honesty section, how much of it was centralized convenience. This chapter earns part of the claim back: the same nine robots, the same grid → ring → circle-lap show, but the corrective control now uses **only relative positions of neighbors within a sensing radius**. No robot reads another robot's absolute state or velocity, ever. The punchline — and it *is* the Turpin–Michael–Kumar punchline, not a bug — is how little changes.

## After this chapter you can …

1. State the information model of decentralized formation flight — what each robot knows, what it senses, and what it never needs — and write Kumar's slide law $e_{ij} = x_i - x_j - s_{des}$ in this repo's notation.
2. Recast "mean of pairwise errors" as consensus — Laplacian feedback on slot errors — separate the differential (shape) mode from the common mode, and name which loop owns each.
3. Predict from the sensing graph's connectivity what shrinking $R_{sense}$ does (a cliff) versus what per-edge dropout does (graceful degradation), and verify both numerically.
4. Explain implicit coordination — why nine robots that never exchange a word still agree to sub-millimeter with the centralized swarm, because each carries the same plan.
5. Say honestly which parts of *this* sim remain centralized, which parts of *Kumar's own demos* were centralized (Vicon fed every state), and what real relative sensing would take.

## The ECE bridge

| New concept | What you already know |
|---|---|
| Consensus (mean-of-neighbors correction) | Distributed clock sync and gossip protocols: every node nudges toward the average of what it hears; there is no master, and the network agrees anyway |
| Pairwise error $e_{ij}$ | Differential signaling: the leader's position cancels out of $x_i - x_j$ the way common-mode noise cancels off a twisted pair — the correction regulates only the differential mode |
| Sensing-graph connectivity | Network topology: a partitioned network cannot reach consensus, no matter how good each link is; each partition converges to its own value |
| Sensing radius $R_{sense}$ | Radio link budget: you only hear nodes whose signal clears your receiver sensitivity, and that range — not your intentions — defines the graph |
| Per-edge dropout $p_{drop}$ | Packet loss: a lossy link lowers throughput but does not move the protocol's fixed point; retries average it out |
| Implicit coordination via a shared plan | GPS-disciplined oscillators: thousands of them, never exchanging a word, all holding the same 10 MHz — because each independently tracks one broadcast reference |

## Core theory

### The information model

Read the docstring at the top of `quadsim/decentralized.py`; it is a contract. Robot $i$ knows three things: its **own state** (position, velocity, attitude — ch07 told you what that costs in reality), the **shared leader plan** $\ell(t)$ with the full keyframe schedule, and its **own slot offset** $\delta_i(t)$ — plus everyone else's offsets $\delta_j(t)$, since the schedule is common knowledge. It *senses* one thing: the relative position $x_i - x_j$ of each neighbor $j$ within $R_{sense}$. It never reads a neighbor's absolute position, velocity, or intent. This is the talk's information model: the plan is broadcast once (implicit coordination — everyone carries the same score), and everything at runtime is local.

### The pairwise law, and why it is consensus

Kumar's slide says $e_{ij} = x_i - x_j - s_{des}$: the error between where your neighbor actually is, relative to you, and where the formation says they should be. In our notation $s_{des} = \delta_i - \delta_j$, so

$$e_{ij} = (x_i - x_j) - (\delta_i - \delta_j), \qquad a_i = -k_{form}\,\frac{1}{|N_i|}\sum_{j \in N_i} e_{ij},$$

with $N_i$ the sensed neighbors and the correction norm-capped at $a_{form,max} = 3\ \mathrm{m/s^2}$, injected into robot $i$'s reference acceleration exactly the way Chapter 5 injected repulsion.

Now the change of coordinates that makes the structure jump out. Let $y_i = x_i - \ell(t) - \delta_i$ — robot $i$'s error relative to its own slot. Then

$$e_{ij} = y_i - y_j,$$

and notice what *vanished*: the leader position $\ell$ cancels from every pairwise error. That cancellation is the entire reason relative sensing suffices — the common mode never appears in the measurement, like common-mode noise never appearing across a differential pair. The correction becomes

$$a_i = -k_{form}\Big(y_i - \tfrac{1}{|N_i|}\textstyle\sum_{j\in N_i} y_j\Big),$$

which is exactly $-k_{form}\,[L\,y]_i$ with $L$ the row-normalized graph Laplacian of the sensing graph. This is the consensus protocol of Olfati-Saber, Fax & Murray: each node relaxes toward the average of its neighbors, like a grid of resistors relaxing toward one voltage. On a **connected** graph, $L$'s null space is spanned by the all-ones vector: the only error the pairwise terms cannot see is *everyone off by the same amount* — a rigid translation of the whole formation. And that one mode is precisely what the shared plan's absolute position loop (Chapter 3's $k_p e_p$, running on each robot's own state) regulates. Clean division of labor: **consensus kills the shape error, the plan kills the common mode.** Differential and common-mode rejection, flying in formation.

One gain choice matters. The docstring sets $k_{form} = 4\ \mathrm{s^{-2}}$, a quarter of the position-loop stiffness $k_p/m = 16\ \mathrm{s^{-2}}$, and gives the reason: the pairwise term is a spring on *relative* position with **no relative damping** — there is no relative-velocity measurement to damp it. The only damping the relative modes get is second-hand, through each robot's own $k_v$ loop, so the added spring must stay soft. Homework 5 has you harden it and watch.

### Connectivity is the convergence condition

Everything above assumed the sensing graph is connected. Suppose it is not — $R_{sense}$ too small, or the formation too spread out. The Laplacian's null space grows: one all-ones vector *per connected component*. Within each component, robots still agree on shape; *between* components, there is no measurement at all. Each component then flies the shared plan **dead-reckoned** — open loop with respect to the other components, closed only on its own state estimate. In this simulator that failure is disguised, because every robot knows its own state perfectly, so each component independently parks on its absolute slots anyway (Experiment 3 shows the residual coupling collapse). On hardware, a disconnected component holds formation only as well as its estimator holds position — and Chapter 7 told you what flow-deck drift does over minutes. The graph is also *state-dependent*: it is actual distances, not slot distances, that decide who senses whom, so a formation can disconnect mid-transient even when its slots are fine. Connectivity is a design deliverable, exactly like network topology: the demo chose $R_{sense} = 1.2$ m as roughly twice the nearest-neighbor slot spacing (0.55 m grid, 0.62 m ring chord) so the graph stays connected through the blend — yet local, since on the ring only the 0.62 m and 1.16 m chords are in range and the 1.56 m chord is not.

### Honesty: what is still centralized — here, and in the videos

The **control** is now decentralized: robot $i$'s corrective term uses its own state plus sensed relative positions, nothing else. Still centralized in our sim: the **slot assignment** is solved once, offline, by Chapter 5's Hungarian machinery — anonymity, the second half of Kumar's argument, is exactly the assignment problem, and pre-assigning $\delta_i$ quietly de-anonymizes the robots (Homework 2). The **plan distribution** is perfect: every robot has bit-identical keyframes and a flawless clock (implicit coordination assumes the broadcast worked). And the **relative positions are ground truth** from the simulator, delivered as full 3-D vectors in a common frame — real relative sensing gives you a scalar range (UWB) or a bearing (camera), in your own body frame, with noise (Homework 3). Before you feel bad about any of this: Kumar's famous lab demos ran decentralized *control laws* while a Vicon motion-capture system measured every vehicle's state centrally and streamed it down — the same honesty, at higher production values. In Kushleyev et al. (2013), only the kilohertz attitude loops ran onboard. The demos that finally closed the loop — onboard sensing, onboard planning, no mocap, outdoors — took another decade: the 2022 Zhejiang "wild swarm" (Further reading 4).

## Guided code walkthrough

Open `quadsim/decentralized.py`. The whole chapter is one subclass:

```python
class DecentralizedSwarmSim(SwarmSim):
```

Keyframe validation, Hungarian slot assignment, smoothstep blending, and pairwise repulsion are all inherited from Chapter 5 unchanged. The constructor adds `R_sense`, `k_form`, `a_form_max`, and the dropout pair `p_drop`/`seed`, and enforces one new invariant — `d_safe <= R_sense` — because the inherited repulsion also runs on relative positions of nearby pairs, and a safety reflex against threats you cannot sense is not a safety reflex (Experiment 5 trips this guard).

The new control law is `formation_accels`. Its core is four lines:

```python
diff = positions[:, None, :] - positions[None, :, :]
dist = np.linalg.norm(diff, axis=2)
sensed = dist <= self.R_sense
np.fill_diagonal(sensed, False)
```

— the sensing graph, rebuilt from actual distances every step (state-dependent, as promised), with each directed edge then independently surviving a `rng.random(...) >= self.p_drop` draw when dropout is on. Then, per robot:

```python
err = diff - (off[:, None, :] - off[None, :, :])
...
a_i = -self.k_form * err[i, idx].mean(axis=0)
```

That is $e_{ij} = (x_i - x_j) - (\delta_i - \delta_j)$ and the mean-of-neighbors correction, norm-capped at `a_form_max`. A robot that senses nobody gets *zero* correction — it flies the plan and nothing else, which is the disconnection story of the theory section made executable. The offsets come from `blended_offsets(t)`, so the law tracks formation transitions mid-blend.

`run` mirrors `SwarmSim.run` with one changed line:

```python
a_corr = (self._avoidance_accels(positions)
          + self.formation_accels(positions, t, rng))
```

Both injected terms ride into `_formation_ref` held constant in $t$ — the Chapter 5 lesson about not detonating the finite-difference attitude feedforward applies verbatim. Two additions earn their keep in the experiments: `start_offsets` lets you launch the swarm *off* its slots (without it, $e_{ij} \approx 0$ from the first step and there is nothing to watch), and the dropout generator is rebuilt from `seed` on every `run` call, so lossy runs are bit-identical and diffable — `tests/test_decentralized.py::test_dropout_determinism` pins that down. Finally, `relative_formation_errors` is the chapter's own metric: the per-sample RMS of $\lVert e_{ij}\rVert$ over all pairs — pure formation **shape** error, blind to absolute leader tracking, which is exactly the quantity the decentralized term regulates.

`demos/demo_decentralized.py` flies the same ~11 s show three ways — `SwarmSim` (centralized baseline), `DecentralizedSwarmSim`, and `DecentralizedSwarmSim` with `p_drop = 0.3` — and prints the same four metrics for each — the three error metrics at six decimals, because the centralized-vs-decentralized gap is sub-millimeter by design. The comment block above `R_SENSE = 1.2` does the connectivity arithmetic from the theory section with the demo's actual geometry; read it.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`.

**Experiment 1 — baseline: three swarms, one dance.**

```bash
.venv/bin/python demos/demo_decentralized.py --fast
```

(~29 s.) Observe the three variants side by side. Centralized: `steady_formation_rms_m` 0.000011, `max_formation_error_m` 0.000726, `min_pairwise_distance_m` 0.4102. Decentralized: 0.000004, 0.000710, 0.4102. Decentralized with 30 % dropout: 0.000004, 0.000710, 0.4102 — indistinguishable from lossless at this precision. The swarm that senses only neighbors within 1.2 m matches — fractionally *beats* — the swarm with perfect central knowledge, and the relative-shape RMS (`relative_rms_m` 0.000006 vs the centralized 0.000016) shows why: the consensus spring actively regulates shape, which the centralized baseline never measures. This agreement is the Turpin–Michael–Kumar point: with implicit coordination, every robot executing the same plan *is already* almost exactly where its neighbors expect, so the sensed correction idles near zero. Relative sensing is a trim tab here, not the pilot — until something goes wrong, which is what the rest of the experiments are for.

**Experiment 2 — the full show.**

```bash
.venv/bin/python demos/demo_decentralized.py --no-gif
```

(~55 s; drop `--no-gif` if you want `decentralized.gif`.) The full 11 s timeline adds the leader's circle lap. All three variants land on `steady_formation_rms_m` 0.000197 — identical to six decimals — with max errors 0.000836 / 0.000815 / 0.000816 m and min pairwise distance 0.4102 m across the board. Steady relative RMS: 0.000003 m centralized, 0.000001 m for both decentralized runs. Yes, those are microns; a perfect model, perfect state, and no disturbances will do that (Chapter 7 priced what reality adds). The number that matters is not any single value but the *spread* between variants: tens of micrometers. Look at `out/decentralized_3d.png` — you cannot tell which architecture flew it. That is the point.

**Experiment 3 — break it: shrink $R_{sense}$ to the disconnection cliff.** Four robots on a diamond ($0.5\sqrt{2} = 0.7071$ m between adjacent slots, 1.0 m across), launched deliberately *off* their slots, with a hovering leader. Sweep $R_{sense}$ down, and include a `k_form = 0` run — the correction surgically removed — as the plan-only reference:

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.decentralized import DecentralizedSwarmSim
from quadsim.swarm import FormationKeyframe
from quadsim.trajectory import hover_ref

OFFSETS = 0.5 * np.array([[1.0,0,0],[0,1.0,0],[-1.0,0,0],[0,-1.0,0]])
START = np.array([[0.3,0.1,0.0],[-0.1,-0.2,0.1],[0.0,0.15,-0.1],[0.1,0.0,0.05]])
LEADER = np.array([0.0,0.0,1.0])
keyframes = [FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=OFFSETS.copy())]

slots = LEADER + OFFSETS
dist = np.linalg.norm(slots[:,None,:]-slots[None,:,:], axis=2)
print("slot spacings: adjacent %.4f m, opposite %.4f m" % (dist[0,1], dist[0,2]))

for R, kf in [(5.0,4.0),(5.0,0.0),(1.1,4.0),(0.9,4.0),(0.71,4.0),(0.70,4.0),(0.5,4.0),(0.35,4.0)]:
    sim = DecentralizedSwarmSim(QuadParams(), hover_ref(LEADER), keyframes, R_sense=R, k_form=kf)
    hist = sim.run(T=4.0, dt=0.004, start_offsets=START.copy())
    rel = sim.relative_formation_errors(hist)
    tr = hist[0].t <= 1.5
    n_edges = int(((dist <= R) & (dist > 0)).sum()) // 2
    print(f"R_sense={R:5.2f} k_form={kf:.1f} edges_at_slots={n_edges} "
          f"transient_rel_mean_m={float(rel[tr].mean()):.4f} final_rel_m={float(rel[-1]):.2e}")
EOF
```

(~23 s.) Observe three things. First, the graph: 6 edges at $R_{sense} = 5.0$ and 1.1 (complete), 4 at 0.9 (adjacent pairs only), and a hard drop to **0** between 0.71 and 0.70 — the adjacent spacing is 0.7071 m, and connectivity is a threshold, not a dial. Second, the transient shape error degrades from 0.0875 m (complete graph) through 0.0903 and 0.1015 down to 0.1143 m with no sensing — a 1.31× penalty, the same ratio the test suite pins — and the no-sensing run reproduces the `k_form = 0` plan-only run *exactly* (0.1143 m transient, 5.91e-06 m final, digit for digit): a disconnected robot and a robot with no correction term are the same robot. Third, the subtlety: $R_{sense} = 0.71$ and 0.70 print the *same* transient error (0.1015 m) despite sitting on opposite sides of the slot-graph cliff, because during the perturbed transient it is actual distances that decide the graph — but their final residuals differ by ≈21× (2.12e-07 vs 4.53e-06 m): once settled near the slots, one swarm is still consensus-coupled and the other is four strangers independently dead-reckoning the same plan. In sim, with perfect self-knowledge, dead reckoning still parks on the slots. On hardware, that final column *is* your estimator drift.

**Experiment 4 — dropout is graceful, disconnection is a cliff.** Same diamond, full connectivity, now with lossy sensing:

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.decentralized import DecentralizedSwarmSim
from quadsim.swarm import FormationKeyframe
from quadsim.trajectory import hover_ref

OFFSETS = 0.5 * np.array([[1.0,0,0],[0,1.0,0],[-1.0,0,0],[0,-1.0,0]])
START = np.array([[0.3,0.1,0.0],[-0.1,-0.2,0.1],[0.0,0.15,-0.1],[0.1,0.0,0.05]])
LEADER = np.array([0.0,0.0,1.0])
keyframes = [FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=OFFSETS.copy())]

for p in [0.0, 0.3, 0.6, 0.9, 0.99]:
    sim = DecentralizedSwarmSim(QuadParams(), hover_ref(LEADER), keyframes,
                                R_sense=5.0, p_drop=p, seed=0)
    hist = sim.run(T=4.0, dt=0.004, start_offsets=START.copy())
    rel = sim.relative_formation_errors(hist)
    tr = hist[0].t <= 1.5
    print(f"p_drop={p:4.2f} transient_rel_mean_m={float(rel[tr].mean()):.5f} "
          f"final_rel_m={float(rel[-1]):.2e}")
EOF
```

(~15 s.) Observe: 0.08750 m lossless; 0.08783 at 30 % loss — a 0.4 % penalty, essentially free, which is why the demo's dropout variant matched the lossless one to six decimals; 0.09174 at 60 %; 0.10573 at 90 %; and 0.11336 at 99 %, converging on the no-sensing value (0.1143 m) from Experiment 3. No cliff anywhere — packet loss, not partition. The reason is structural: the correction is a *mean over surviving edges*, and every surviving $e_{ij}$ still points toward the correct shape, so dropout thins the estimate without biasing it — the fixed point never moves, only the effective gain sags. Losing packets slows a gossip protocol; cutting the cable partitions it. Homework 4 puts probability on exactly where this curve bends.

**Experiment 5 — break it: promise a reflex you cannot see.** Ask for $R_{sense} = 0.2$ m with the default $d_{safe} = 0.30$ m bubble:

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.decentralized import DecentralizedSwarmSim
from quadsim.swarm import FormationKeyframe
from quadsim.trajectory import hover_ref
OFFSETS = 0.5 * np.array([[1.0,0,0],[0,1.0,0],[-1.0,0,0],[0,-1.0,0]])
keyframes = [FormationKeyframe(t_start=0.0, t_blend=0.1, offsets=OFFSETS)]
DecentralizedSwarmSim(QuadParams(), hover_ref(np.array([0.,0.,1.])), keyframes, R_sense=0.2)
EOF
```

Observe: `ValueError: d_safe must not exceed R_sense: repulsion is computed from sensed relative positions` — at construction, before anything flies. Chapter 5's repulsion needs a relative position to push along; in the decentralized information model that position must be *sensed*, so a safety bubble wider than the sensing radius would be a promise the sensors cannot keep. Same habit as Chapter 5's overlapping-blend guard: encode the physical assumption as a boundary check and fail loudly. The safety implication is worth saying out loud — in this architecture, **collision avoidance degrades with the same sensor that formation keeping does**; a blind robot is not just sloppy, it is unsafe.

## Homework

1. Derive the consensus form: starting from $a_i = -k_{form}\,\mathrm{mean}_{j \in N_i}\,e_{ij}$, substitute $y_i = x_i - \ell - \delta_i$, show the leader cancels, and write the stacked dynamics as $a = -k_{form} (I - D^{-1}A)\,y$. What is the null space of that matrix on a connected graph, which physical motion does it correspond to, and which term of Chapter 3's controller pins it?

> **Your answer:**



2. Kumar's second argument was *anonymity* — robots should be interchangeable. Our robots are not: each is constructed with a pre-assigned $\delta_i$. Explain why genuine anonymity turns the offsets into a runtime decision, why that decision is exactly Chapter 5's assignment problem, and which property of the squared-distance CAPT assignment you would refuse to give up when solving it online.

> **Your answer:**



3. Forward-looking: the sim hands each robot $x_i - x_j$ as an exact 3-D vector in a common frame. What would a real Crazyflie need to measure that? Consider UWB two-way ranging (a scalar distance) and an onboard camera (a bearing in the *body* frame): what does each alone fail to give you, what does combining them require, and what supplies the shared yaw reference without which "the neighbor is 0.6 m north of me" is not even a well-formed sentence?

> **Your answer:**



4. In the Experiment 4 diamond every robot has three potential neighbors and each directed edge drops independently with probability $p$. Derive the probability that a robot is completely blind in a given step, evaluate it at $p = 0.3$, 0.6, and 0.9, and use those three numbers to explain the shape of the measured sweep — in particular why 30 % loss cost only 0.4 %.

> **Your answer:**



5. Hands-on: the docstring keeps $k_{form} = 4\ \mathrm{s^{-2}}$, a quarter of $k_p/m = 16\ \mathrm{s^{-2}}$, because the pairwise spring adds no relative damping. Predict what $k_{form} = 64$ does to the transient, then rerun Experiment 3's $R_{sense} = 5.0$ case with it and report. Where would relative damping have to come from, and what would the robot need to sense to provide it?

> **Your answer:**



6. The demo claims $R_{sense} = 1.2$ m keeps ring sensing "genuinely local." Compute all four distinct chord lengths of the 9-slot, $r = 0.9$ m ring ($2r\sin(k\pi/9)$ for $k = 1..4$), count each robot's neighbors at $R_{sense} = 1.2$ and at 2.0, and argue both sides: what improves as the graph densifies toward complete, and what — in radio-link-budget terms and in Kumar's scaling argument — you pay for it.

> **Your answer:**



## Hints

1. $\mathrm{mean}_j (y_i - y_j) = y_i - \frac{1}{|N_i|}\sum_j y_j$; stack rows. The null vector has every component equal. The pinning term is the one that reads *absolute* position error $e_p$.
2. If nobody owns a slot in advance, someone must choose at runtime who takes which — from relative information. The property to keep is the one Chapter 5 proved with the parallelogram law: non-crossing straight-line transitions.
3. A range alone puts the neighbor on a sphere; a bearing alone on a ray. Think about what rotates a body-frame bearing into a shared frame — and which sensors from Chapter 7 (magnetometer? UWB anchors? Lighthouse?) can anchor yaw.
4. Blind means all three incoming edges drop at once: $p^3$. Compare $0.3^3$ with $0.9^3$ and look at which sweep rows bend.
5. A spring with stiffness 4× the position loop's, damped only through each robot's own $k_v$ acting on absolute velocity — expect relative-mode ringing. Watch `relative_formation_errors` oscillate rather than decay cleanly. Damping it directly would need relative *velocity*, which the information model deliberately excludes.
6. $2 \times 0.9 \times \sin(\pi/9) \approx 0.62$ m is the first chord; three more to compute. At $R_{sense} \to$ complete graph, the mean over neighbors approaches the centralized average — good for stiffness, but every added edge is a link your radio/vision must actually sustain, on every robot, at 100 Hz.

## Further reading

1. **Turpin, Michael & Kumar — "Trajectory design and control for aggressive formation flight with quadrotors" (2011).** The citation on the TED slide: the pairwise-error control law this module implements, with the stability analysis and the aggressive-flight results the talk shows off.
2. **Olfati-Saber, Fax & Murray — "Consensus and cooperation in networked multi-agent systems" (Proceedings of the IEEE, 2007).** The consensus survey: graph Laplacians, connectivity as the convergence condition, convergence rate as algebraic connectivity — the theory this chapter used with the serial numbers filed off.
3. **Kushleyev, Mellinger, Powers, Kumar — "Towards a swarm of agile micro quadrotors" (2013).** Reread it after this chapter with one question: for each subsystem, did this run onboard or on the base station? The answers are the honest version of the highlight reel.
4. **Zhou et al. — "Swarm of micro flying robots in the wild" (Science Robotics, 2022).** The Zhejiang University swarm: palm-sized quadrotors flying *through a bamboo forest* with onboard vision, onboard planning, and onboard relative localization — no mocap, no GPS, no shared plan even. The distance between Chapter 8 and that paper is a fair map of the decade of research between Kumar's talk and now, and the natural "what's next" if this chapter hooked you.

---

*← Previous: [Chapter 7 — The real flight stack](ch07-real-flight.md) · [Course index](README.md) · Next: [Chapter 9 — Your own firmware](ch09-own-firmware.md) →*
