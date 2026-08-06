# Chapter 5 — Swarms: nine robots, one dance

You have flown one quadrotor through a figure-eight and a gate course. This chapter flies nine at once, and the punchline is how little new control theory that requires. The swarm problem, as this repo frames it, decomposes into three ideas layered on top of everything you already built: a *shared reference* (the virtual leader), a *matching problem* (who takes which slot), and a *safety reflex* (short-range repulsion). Each quad still runs the same `SE3Controller` from Chapter 3, unmodified.

## After this chapter you can …

1. Explain the virtual-leader + formation-offset architecture and why it turns a nine-robot coordination problem into nine independent tracking problems.
2. Formulate slot assignment as bipartite matching, solve it with the Hungarian algorithm, and defend the choice of *squared* distances.
3. Derive the quintic smoothstep and explain — pointing at a specific line of `controller.py` — why formation transitions must be $C^2$ in time.
4. Predict when the 0.30 m avoidance bubbles activate, and describe the standoff between the position loop and the repulsion term when they do.
5. Say honestly which parts of this sim are decentralized in Kumar's sense and which are centralized conveniences.

## The ECE bridge

**Virtual leader + offsets** is a phased array. In a beamforming array you never command each element independently; you command one phase center and each element holds a fixed geometric offset from it. Here the leader reference $\ell(t)$ is the phase center, and slot $i$ is $\ell(t) + \delta_i(t)$. Steering the array = flying the leader.

**The assignment problem** is channel allocation. Nine users, nine channels, a cost for every (user, channel) pair, each channel assigned to exactly one user, minimize total cost: that is resource allocation in OFDMA schedulers and crossbar-switch scheduling, and it is exactly *minimum-cost bipartite matching*. The Hungarian algorithm (via `scipy.optimize.linear_sum_assignment`) solves it in polynomial time — no need to enumerate $9! = 362{,}880$ permutations.

**Smoothstep blending** is pulse shaping. Keying a carrier with rectangular edges splatters energy across the spectrum, so you shape symbols with raised-cosine pulses whose derivatives vanish at the edges. A formation change is a keying event on the reference signal; the controller *differentiates* that signal (twice!), and differentiation multiplies spectra by $(j\omega)^n$. Sharp edges in, torque splatter out.

**Squared distances** are least squares. Same reason as everywhere in estimation: the $L^2$ cost is smooth, and it obeys the parallelogram law, which — you will see — is precisely what makes optimal assignments non-crossing.

**Repulsion** is Coulomb's law with a gain: a $1/d$ potential-style push, applied reciprocally like a force pair, acting as a fast local loop underneath the slower global plan — the same layering as a current-limit loop inside a voltage regulator.

## Core theory

### Virtual leader and formation offsets

The central design decision in `quadsim/swarm.py` is that a formation is *not* nine trajectories. It is one trajectory — the leader flat output $\ell(t)$, a `Callable[[float], FlatOutput]` — plus a $(9,3)$ array of offsets. Quad $i$'s reference is

$$p_i^{ref}(t) = \ell_p(t) + \delta_i(t), \qquad v_i^{ref}(t) = \ell_v(t) + \dot{\delta}_i(t), \qquad a_i^{ref}(t) = \ell_a(t) + \ddot{\delta}_i(t).$$

Because references live in flat-output space (Chapter 4), they add by superposition: derivatives of a sum are sums of derivatives, so the leader's motion and the formation's shape change compose without any coupling. Each quad then tracks its own reference with its own controller and never needs to know the others exist — except through the repulsion term below. This is why the demo runs nine full `SE3Controller` + RK4 pipelines in lockstep and the code barely grows.

### The assignment problem, and why squared distances

When the formation changes from a grid to a ring, *some* robot must take *some* ring slot, but nothing says row-major order is a good pairing. A bad pairing sends robots across the formation through each other. So between consecutive keyframes we build the cost matrix

$$C_{ij} = \lVert \delta_i^{prev} - \delta_j^{next} \rVert^2$$

and solve $\min_\pi \sum_i C_{i\,\pi(i)}$ over permutations $\pi$ — minimum-cost perfect matching on a bipartite graph of robots and slots.

Why *squared* norms and not plain distances? Take any two robots at $p_1, p_2$ assigned to goals $g_1, g_2$. If that pairing is optimal under squared cost, swapping cannot help:

$$\lVert p_1 - g_1\rVert^2 + \lVert p_2 - g_2\rVert^2 \le \lVert p_1 - g_2\rVert^2 + \lVert p_2 - g_1\rVert^2.$$

Expand both sides and the quadratic terms cancel, leaving

$$(p_1 - p_2) \cdot (g_1 - g_2) \ge 0.$$

The displacement between the two robots and the displacement between their two goals never point in opposing directions — straight-line transitions don't cross head-on. This is the heart of Turpin, Michael & Kumar's CAPT result: squared-distance assignment plus synchronized straight-line motion guarantees separation, given enough initial clearance. A linear-distance cost gives no such inequality (the cross terms don't cancel), and it also happily trades one very long trip for savings spread across others; the squared cost penalizes the *longest* trips hardest, which balances the transition. Cheap insurance, chosen for a theorem, not for taste.

### Smoothstep and the $C^2$ requirement

Between keyframes, offsets blend as $\delta(t) = \delta^{prev} + s(u)\,(\delta^{next} - \delta^{prev})$ with $u = (t - t_{start})/t_{blend}$ and the quintic smoothstep

$$s(u) = 6u^5 - 15u^4 + 10u^3, \qquad s'(0)=s''(0)=s'(1)=s''(1)=0.$$

It is the *lowest-order* polynomial meeting those six boundary conditions — five derivative constraints plus $s(0)=0, s(1)=1$ need six coefficients. Why insist on two vanishing derivatives? Look at `quadsim/controller.py`. The reference acceleration enters the force command directly (`F_des = ... + m * ref.acc`), so a jump in $\ddot{\delta}$ is a step in commanded thrust direction — ugly but survivable. The real killer is a few lines down:

```python
d = fd_dt
R_ff_0 = R_ff(t)
w_ff = rot_log(R_ff_0.T @ R_ff(t + d)) / d
w_ff_m = rot_log(R_ff(t - d).T @ R_ff_0) / d
a_ff = (w_ff - w_ff_m) / d
```

The controller builds its attitude feedforward by *finite-differencing the reference* with $d = 10^{-3}$ s. A discontinuity in reference acceleration becomes a discontinuity in the commanded rotation $R_{ff}$, and the second difference turns that into a spike of order $1/d^2$ in $a_{ff}$ — a torque hammer. So the reference handed to this controller must be $C^2$: position, velocity, *and* acceleration continuous. That is why `_smoothstep` returns all three of $s, s', s''$ analytically, why the leader in the demo ramps its circle angle with the same quintic, and why `SwarmSim` refuses overlapping blend windows (two simultaneously active blends would need summing logic the single-blend loop doesn't have — the guard fails loudly instead of flying wrongly). Experiment 4 puts a number on the damage a mere $C^0$ blend does.

### Repulsion: the safety reflex

The plan above is open-loop about collisions: assignment and smooth blending make them unlikely, not impossible. So a last layer watches actual positions. For any pair closer than $d_{safe} = 0.30$ m, each robot receives an acceleration along the separating direction,

$$a_{push} = k_{avoid}\left(\frac{1}{d_{ij}} - \frac{1}{d_{safe}}\right)\hat{n}_{ij},$$

zero exactly at the bubble edge, growing without bound as $d_{ij} \to 0$, applied equal-and-opposite to both robots (reciprocal, like a force pair), and capped at $3\,\mathrm{m/s^2}$ per robot so it can never demand more than the actuators plausibly deliver. Crucially, it is injected as an *extra acceleration on the reference*, not as a raw force — it flows through the same controller, so the vehicle leans into the dodge properly. Note what it needs to compute: only the relative position of a nearby neighbor. That is a genuinely decentralizable rule.

### Honesty: how much of this is really "decentralized"?

Kumar's talks celebrate decentralization; this sim keeps its *flavor* while quietly centralizing the hard parts. Genuinely local: the formation rule (each quad needs only the broadcast leader state and its own offset — like each array element needing only the reference oscillator) and the pairwise repulsion (relative positions of near neighbors, measurable onboard in principle). Centralized conveniences: the assignment is solved *once, offline, at construction* by an all-knowing planner; every quad reads exact ground-truth state (no estimation, no mocap noise, no latency); the simulation is lockstep-synchronous, i.e., perfect clock distribution and a lossless radio. And the bubble is a *sphere* — real multi-quad work uses ellipsoids stretched along $z$, because a quadrotor's downwash hammers anyone below it; this demo dodges the issue by keeping every formation flat at one altitude. To be fair, the famous lab demos (Kushleyev et al. 2013) were also Vicon-tracked and centrally planned — only the kilohertz attitude loops ran onboard. The sim is honest company; just don't mistake either for a GPS-denied forest swarm.

## Guided code walkthrough

Open `quadsim/swarm.py`. A formation change is data, not code:

```python
@dataclass
class FormationKeyframe:
    t_start: float
    t_blend: float
    offsets: np.ndarray   # (n, 3), meters, relative to the leader
```

The assignment happens once, in `SwarmSim.__init__`, sequentially keyframe-to-keyframe — note `prev` is the *already permuted* previous formation, so identities chain correctly through the whole show:

```python
prev = self._offsets[-1]
cost = ((prev[:, None, :] - cand[None, :, :]) ** 2).sum(axis=2)
_, col = linear_sum_assignment(cost)
self._offsets.append(cand[col])
```

Three lines: broadcasted squared-distance cost matrix, Hungarian solve, permute the candidate rows. `blended_offsets(t)` then walks the keyframes and returns offsets *and* their two analytic derivatives, which `_formation_ref` adds onto the leader's flat output — including the avoidance term:

```python
return FlatOutput(pos=base.pos + off[i], vel=base.vel + doff[i],
                  acc=base.acc + ddoff[i] + a_extra, ...)
```

`a_extra` is held constant in $t$ deliberately: the controller will finite-difference this `ref_fn`, and a frozen avoidance term contributes zero to those differences — the reflex nudges the force loop without detonating the attitude feedforward. The repulsion itself is `_avoidance_accels`, a plain $O(n^2)$ pair loop implementing the formula above, with the per-robot norm cap at the end.

Now `demos/demo_swarm.py`. The three formations are small pure functions — `grid_offsets()` (3×3, 0.55 m spacing), `ring_offsets()` (radius 0.9 m, chord $2r\sin(\pi/9) = 0.62$ m), and `v_offsets()`, a migrating-birds chevron whose docstring does the safety arithmetic for you (closest slots 0.495 m apart). `make_leader` builds the $C^2$ leader: hover, then one circle lap whose *angle* is ramped by the same quintic smoothstep, so the lap starts and ends with zero velocity and acceleration, matching the hovers. The timeline is just three keyframes and a leader:

```python
keyframes = [
    FormationKeyframe(t_start=0.0,  t_blend=1.0, offsets=grid_offsets()),
    FormationKeyframe(t_start=6.0,  t_blend=4.0, offsets=ring_offsets()),
    FormationKeyframe(t_start=12.0, t_blend=4.0, offsets=v_offsets()),
]
leader = make_leader(t0=18.0, t_circle=10.0)
```

Everything you watch in `swarm.gif` is those five lines plus the machinery above.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`.

**Experiment 1 — baseline.** Establish the healthy numbers:

```bash
.venv/bin/python demos/demo_swarm.py --fast
```

(~16 s.) Observe: `steady_formation_rms_m` ≈ 0.0000, `max_formation_error_m` ≈ 0.0007, and `min_pairwise_distance_m` ≈ 0.4102 — comfortably above the 0.30 m bubble. The stock demo *never* triggers avoidance; every formation was designed with ≥ 0.4 m slot spacing on purpose. Look at `out/swarm_3d.png`.

**Experiment 2 — your initials as a keyframe.** Nine robots, nine dots: trace a letter. Template (a "V" — edit `pts` into your own letter, then try your other initial):

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.swarm import FormationKeyframe, SwarmSim, min_pairwise_distance, formation_errors
from demos.demo_swarm import grid_offsets, make_leader

def letter_V(scale=0.45):
    pts = [[0.0, -0.9, 0.0]]                                  # vertex at the bottom
    for k in range(1, 5):
        pts.append([-k * 0.5 * scale, -0.9 + k * scale, 0.0])  # left stroke
        pts.append([+k * 0.5 * scale, -0.9 + k * scale, 0.0])  # right stroke
    return np.array(pts)

off = letter_V()
d = np.linalg.norm(off[:, None, :] - off[None, :, :], axis=2)
d[d == 0] = np.inf
print("min slot spacing (m):", d.min())      # keep this >= 0.4!

keyframes = [
    FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
    FormationKeyframe(t_start=3.0, t_blend=3.0, offsets=letter_V()),
]
swarm = SwarmSim(QuadParams(), make_leader(t0=1e9, t_circle=10.0), keyframes)
hist = swarm.run(9.0, dt=0.004)
print("min_pairwise_distance_m:", min_pairwise_distance(hist))
print("max_formation_error_m:", float(formation_errors(hist).max()))
from quadsim.viz import plot_trajectory_3d
plot_trajectory_3d(hist, save="out/swarm_initials.png", title="Letter formation")
EOF
```

Observe: the spacing check first (0.45 m for this "V"), then min pairwise ≈ 0.446 m and max error under a millimeter. Print `swarm._offsets[1]` if you're curious which grid robot the Hungarian algorithm sent to which stroke.

**Experiment 3 — break it: shrink the ring into the bubbles.** A 9-slot ring has chord spacing $2r\sin(\pi/9) \approx 0.684\,r$; at $r = 0.35$ that is 0.239 m < 0.30 m, so the *requested formation itself* violates the bubbles:

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.swarm import FormationKeyframe, SwarmSim, min_pairwise_distance, formation_errors
from demos.demo_swarm import grid_offsets, ring_offsets, make_leader

keyframes = [
    FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
    FormationKeyframe(t_start=3.0, t_blend=3.0, offsets=ring_offsets(radius=0.35)),
]
swarm = SwarmSim(QuadParams(), make_leader(t0=1e9, t_circle=10.0), keyframes)
hist = swarm.run(10.0, dt=0.004)
errs = formation_errors(hist)
print("min_pairwise_distance_m:", min_pairwise_distance(hist))
print("max_formation_error_m:", float(errs.max()))
print("final error per quad (m):", np.round(errs[-1], 3))
EOF
```

Observe: min pairwise ≈ 0.213 m — the repulsion did *not* restore 0.30 m, but it kept robots off each other — and final formation errors of 3–10 cm that never settle. What it teaches: the safety layer and the position loop reach a standoff. The position loop pulls toward the slot with acceleration $\approx (k_p/m)\,e_p = 16\,e_p$; the repulsion pushes back with $k_{avoid}(1/d - 1/d_{safe})$, capped at 3 m/s². Neither wins; the swarm hovers in a strained compromise, permanently off its formation. Safety layers *degrade* tracking by design — the fix is a better plan, not a bigger gain.

**Experiment 4 — break it: replace the smoothstep with a linear ramp.** Monkeypatch the blend to $s(u) = u$ (a $C^0$ blend: reference velocity jumps at the window edges):

```bash
.venv/bin/python - <<'EOF'
import quadsim.swarm as sw
sw._smoothstep = lambda u: (u, 1.0, 0.0)   # sabotage: linear, derivatives lie
from quadsim.params import QuadParams
from quadsim.swarm import FormationKeyframe, SwarmSim, formation_errors
from demos.demo_swarm import grid_offsets, ring_offsets, make_leader
keyframes = [
    FormationKeyframe(t_start=0.0, t_blend=1.0, offsets=grid_offsets()),
    FormationKeyframe(t_start=3.0, t_blend=3.0, offsets=ring_offsets()),
]
swarm = SwarmSim(QuadParams(), make_leader(t0=1e9, t_circle=10.0), keyframes)
hist = swarm.run(10.0, dt=0.004)
print("max_formation_error_m:", float(formation_errors(hist).max()))
EOF
```

Observe: max error jumps from 0.0007 m to ≈ 0.037 m — a 50× degradation — and the peak lands at $t \approx 3.2$ s, right at the blend's leading edge, where the velocity step hits `e_v` and the finite-difference feedforward. Rectangular keying, torque splatter.

**Experiment 5 — break it: overlap two blend windows.** Start the second blend before the first finishes:

```bash
.venv/bin/python - <<'EOF'
from quadsim.params import QuadParams
from quadsim.swarm import FormationKeyframe, SwarmSim
from demos.demo_swarm import grid_offsets, ring_offsets, make_leader
keyframes = [
    FormationKeyframe(t_start=0.0, t_blend=2.0, offsets=grid_offsets()),
    FormationKeyframe(t_start=1.0, t_blend=3.0, offsets=ring_offsets()),
]
SwarmSim(QuadParams(), make_leader(t0=1e9, t_circle=10.0), keyframes)
EOF
```

Observe: `ValueError: keyframe blend windows must not overlap ...` at construction, before anything flies. What it teaches: `blended_offsets` handles exactly one active blend; overlapping windows would silently produce a reference that teleports between partial blends. Validating inputs at the boundary and failing loudly is a controls habit worth stealing.

## Homework

1. Complete the proof from the theory section: expand $\lVert p_1-g_1\rVert^2 + \lVert p_2-g_2\rVert^2 \le \lVert p_1-g_2\rVert^2 + \lVert p_2-g_1\rVert^2$ and show it reduces to $(p_1-p_2)\cdot(g_1-g_2) \ge 0$. Then give a 2-robot example where the *linear*-distance-optimal assignment makes their straight-line paths pass through the same point at the same time.

> **Your answer:**



2. In the fast demo (grid → ring), predict which grid robot takes which ring slot for at least the four corner robots, then check by printing `swarm._offsets[1]` next to `grid_offsets()`. Was the Hungarian solution the one you would have picked by eye?

> **Your answer:**



3. The blend's peak offset speed is $\max_u s'(u) \cdot \lVert\Delta\delta\rVert / t_{blend}$. Find $\max_u s'(u)$ for the quintic analytically, then compute the fastest slot's peak speed in the grid → ring transition of the fast demo ($t_{blend} = 3$ s).

> **Your answer:**



4. `_formation_ref` adds the avoidance acceleration `a_extra` *held constant in* $t$. Suppose instead `ref_fn` recomputed avoidance from live positions at each queried time. Explain, using the `w_ff`/`a_ff` lines of `controller.py`, why that seemingly "more correct" version is dangerous.

> **Your answer:**



5. The avoidance bubble is a 0.30 m *sphere*. Explain why a real multi-quadrotor system needs an ellipsoidal keep-out region instead, and identify the design choice in `demo_swarm.py`'s formations that lets the spherical bubble get away with it here.

> **Your answer:**



6. Hands-on: in Experiment 3, the equilibrium is a balance between the position loop's pull and the repulsion's push. Using $k_p/m = 16\ \mathrm{s^{-2}}$, $k_{avoid} = 4$, $d_{safe} = 0.30$, estimate the pairwise distance at which a robot with 0.05 m of formation error is in equilibrium with one neighbor, and compare with the measured final errors and distances.

> **Your answer:**



## Hints

1. The quadratic terms $\lVert p_i\rVert^2, \lVert g_j\rVert^2$ appear on both sides. For the counterexample, try two robots and two goals arranged so both pairings have *equal* total linear distance — degenerate ties are where linear cost fails.
2. Build the `SwarmSim` exactly as in the demo's `--fast` branch, in an interactive shell. Row $i$ of `_offsets[1]` is where grid robot $i$ ends up.
3. Set $s''(u) = 0$: the extremum of $s'$ is at $u = 1/2$, and $s'(1/2)$ is a nice fraction. The fastest slot is the one with the largest $\lVert\Delta\delta\rVert$ after assignment — a corner robot.
4. The finite differences evaluate `ref_fn` at $t - d$, $t$, and $t + d$ with $d = 10^{-3}$. What does dividing a jump by $d^2$ produce, and which command does `a_ff` feed?
5. Think about what the propellers do to the air *below* the vehicle, then look at the $z$ column of every offsets array in the demo.
6. Set $16 \times 0.05 = 4\,(1/d - 1/0.30)$ and solve for $d$. Remember each robot in a tight ring has *two* close neighbors whose pushes partially cancel tangentially.

## Further reading

1. **Kushleyev, Mellinger, Powers, Kumar — "Towards a swarm of agile micro quadrotors" (2013).** The paper behind this module: formation keyframes, ellipsoidal (downwash-aware) separation, and what actually ran onboard vs. on the base station.
2. **Turpin, Michael, Kumar — "CAPT: Concurrent assignment and planning of trajectories" (IJRR 2014).** The theorem that makes "why squared distances" a proof instead of a preference.
3. **Kuhn — "The Hungarian method for the assignment problem" (1955).** Short, classic, and the reason `linear_sum_assignment` exists; skim to see the duality idea, which will feel like network-flow LP.
4. **Kumar's TED talk, "Robots that fly … and cooperate" (2012).** The nine-robot finale this demo is a nod to — watch it once before, and once after, this chapter.
