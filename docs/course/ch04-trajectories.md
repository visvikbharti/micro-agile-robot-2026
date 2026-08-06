# Chapter 4 — Minimum snap: why the motion looks graceful

Chapter 3 gave you a controller that can chase a reference. This chapter is about making
references *worth chasing*. The difference between a quadrotor that lurches between waypoints
and one that flows through them like a swallow is not the controller — it is the trajectory. The
key ideas are differential flatness (a structural gift of the quadrotor's dynamics) and
minimum-snap polynomial optimization (Mellinger & Kumar, ICRA 2011), and both live in
`quadsim/trajectory.py`, our walkthrough file.

## After this chapter you can …

1. State what differential flatness means for a quadrotor and trace the chain from position
   derivatives to attitude, body rates, and motor commands — and therefore explain *why* the
   fourth derivative (snap) is the thing to minimize.
2. Count the degree-of-freedom budget of a piecewise 7th-order spline — unknowns vs
   constraints per segment — and say where the optimizer's leftover freedom lives.
3. Read the KKT linear system in `_solve_coefficients` as equality-constrained least squares
   and explain the role of the time scaling $\tau = (t - t_i)/T_i$.
4. Retell the project's real integration incident — the figure-8 whose yaw-rate demand exceeded
   vehicle authority — and replay its mechanism in simulation on today's code.
5. Design your own waypoint course, find the vehicle's empirical speed limit, and verify a
   trajectory's derivatives numerically.

## The ECE bridge

| New thing here | What you already know |
|---|---|
| Differential flatness: 4 outputs $(x, y, z, \psi)$ determine all 13 states | Observability: for an observable system you can reconstruct the full internal state from the output and enough of its derivatives. Flatness is the constructive version — an *algebraic* formula, no observer needed |
| Minimizing $\int \lVert \text{snap} \rVert^2 dt$ | Differentiation multiplies a spectrum by $j\omega$; four derivatives means $\omega^8$ in the cost. Minimizing snap is a smoothness regularizer that crushes high-frequency content, like penalizing stopband energy in filter design |
| Equality-constrained QP via the KKT system | The MVDR beamformer: minimize $w^H R w$ subject to $w^H a = 1$, solved with Lagrange multipliers. Identical structure — quadratic cost, linear equality constraints, one bordered linear solve. Also: normal equations with constraints |
| Piecewise polynomials with derivative continuity at knots | Continuous-phase FSK: phase is kept continuous at symbol boundaries so the spectrum stays compact. Here derivatives 1–4 are kept continuous at waypoints so the *actuator* spectrum stays compact |
| Time scaling $\tau = (t - t_i)/T_i \in [0, 1]$ | Normalized frequency in filter design: design the prototype on $[0, 1]$, then frequency-scale. Same trick, same reason — numerical conditioning |
| Velocity-aligned yaw via `atan2`, `np.unwrap`, hold below 0.15 m/s | Instantaneous phase of an analytic signal, with unwrapping; the low-speed hold is a PLL holding its last phase estimate when the carrier drops below usable SNR |

## Core theory

### Differential flatness: four numbers rule them all

The quadrotor has 13 states and 4 inputs, and yet the planner in this project only ever thinks
about four scalar functions of time: $x(t), y(t), z(t), \psi(t)$ — position and yaw. That is
legal because the quadrotor is **differentially flat**: every state and every input can be
written as an algebraic function of these four *flat outputs* and finitely many of their time
derivatives. No differential equation needs to be integrated to plan; planning becomes curve
design.

The construction is worth internalizing because it *is* the controller's feedforward path.
Newton's law for the translational dynamics reads $m\ddot{p} = -mg e_3 + f R e_3$, so the
required world-frame force is

$$F = m\,(\ddot{p} + g e_3),$$

and since the thrust can only point along the body z-axis, $F$ hands us both the thrust
magnitude $f = \lVert F \rVert$ and the direction $b_3 = F / \lVert F \rVert$. The yaw angle
$\psi$ then pins down the remaining rotation about $b_3$, giving the full attitude $R$. So:
**acceleration determines attitude**. Differentiate once more: the *jerk* $\dddot{p}$ tells you
how fast $b_3$ is turning, i.e. the body angular rates $\omega$. Differentiate again: the
**snap** $\ddddot{p}$ determines the angular acceleration $\dot{\omega}$, which through
$\tau = J\dot{\omega} + \omega \times J\omega$ determines the torques, which through the mixer
matrix of Chapter 2 determine the four individual motor thrusts.

That chain — position → velocity → acceleration → attitude → (jerk) body rates → (snap) motor
commands — is the whole argument for minimum snap. Snap is the derivative that lands directly
on the motors. Minimize $\int \lVert \ddddot{p} \rVert^2 dt$ and you are, quite literally,
minimizing actuator violence. This is why the flight looks graceful: the plan never asks the
motors for anything abrupt.

### Why 7th-order polynomials

Apply calculus of variations to $\min \int (\ddddot{p})^2 dt$ and the Euler–Lagrange equation
is $p^{(8)}(t) = 0$: the optimal curve between boundary conditions is a polynomial of degree 7.
(The general pattern: minimizing the $k$-th derivative squared gives degree $2k - 1$; minimum
jerk would give quintics.) So each spline segment is a 7th-order polynomial per axis — 8
coefficients, `_N_COEFF = 8` in the code.

### The constraint budget

Take $n$ segments (from $M = n + 1$ waypoints). Per axis you have $8n$ unknown coefficients.
The constraints imposed in `quadsim/trajectory.py` are:

- each segment starts and ends at its waypoints: $2n$ constraints;
- rest at both trajectory endpoints — velocity, acceleration, jerk all zero: $6$ constraints;
- continuity of physical-time derivatives 1 through 4 at each interior knot: $4(n-1)$.

Total: $6n + 2$. The slack is $8n - (6n + 2) = 2(n - 1)$ free parameters per axis — two per
interior waypoint. Those are what the optimizer spends: it gets to *choose* the velocity,
acceleration, jerk, and snap values at interior knots (subject to continuity) so as to minimize
total snap energy. For $n = 1$ the budget is exact ($8 = 8$): a single segment is fully
determined by its boundary conditions and there is nothing to optimize.

### Constrained least squares and the KKT system

The cost is quadratic in the coefficients, $c^T H c$, and all constraints are linear,
$Ac = b$. You have solved this shape of problem before — MVDR beamforming, equality-constrained
least squares — and the solution is the same here: introduce multipliers $\lambda$, set the
gradient of the Lagrangian to zero, and solve one bordered linear system,

$$\begin{bmatrix} 2H & A^T \\ A & 0 \end{bmatrix} \begin{bmatrix} c \\ \lambda \end{bmatrix} = \begin{bmatrix} 0 \\ b \end{bmatrix}.$$

This is the KKT system, and it is solved once per axis (the three axes conveniently share the
same matrix, so the code solves for a 3-column right-hand side).

### Time scaling: the conditioning trick

If you wrote each segment's polynomial in raw time $t$, a 16-second trajectory would need
monomials up to $t^7 \approx 2.7 \times 10^8$ next to constants of order one — a numerically
disastrous Vandermonde-style spread. Instead each segment uses scaled time
$\tau = (t - t_i)/T_i \in [0, 1]$, so every basis value stays $O(1)$. The bookkeeping cost is
the chain rule: a $k$-th physical-time derivative picks up $T_i^{-k}$, and the snap cost picks
up $T_i^{-7}$ per segment. This is exactly your filter-design habit of working in normalized
frequency and scaling afterwards.

That $T_i^{-7}$ is worth staring at: **halving a segment's duration multiplies its snap cost by
128**. Segment times matter enormously, which brings us to the incident.

### Segment times, the 0.5 s floor, and the figure-8 incident

The constructor allocates default segment times as
$T_i = \max(d_i / \bar{v},\ 0.5)$ — segment length over requested average speed, floored at
half a second. The floor exists because this vehicle (33 g, four motors of 0.16 N each, about
2:1 thrust-to-weight) simply cannot execute an arbitrarily short segment; $T_i^{-7}$ says the
demanded aggression explodes as $T_i$ shrinks.

Here is the real story from this project's integration, recorded in the git log (commit
`3a06026`: *"retuned the figure-8 waypoint density (8 → 16 per lap) to keep yaw-rate demand
within vehicle authority"*). The original figure-8 demo sampled the lemniscate at **8 waypoints
per lap**. Every segment hit the 0.5 s floor, so two laps compressed into 8 s, with peak speeds
near 3.9 m/s — and `yaw_mode="velocity"` obliged the nose to sweep the whole heading profile in
half the time. The integration run clocked the demanded yaw rate around **22 rad/s** — more than
three revolutions per second, far beyond what the attitude loop can deliver while
simultaneously banking hard. (Re-measuring on today's code with the 500 Hz yaw grid you will
see reference peaks in the mid-teens of rad/s; either number is well beyond authority.) The sim
did not politely WARN; it diverged, RMS error ≈ 69 m. (That 69 m is the historical record from
the integration run; the code has been tuned since, and replaying the 8-point case today —
Experiment 4 — gives a degraded-but-stable RMS ≈ 0.25 m rather than outright divergence. The
mechanism, though, is unchanged.) The fix was counterintuitive and
instructive: **densify the waypoints**. At 16 points per lap there are twice as many
floor-limited segments, so the same geometry takes 16 s instead of 8 s, peak speed drops to
about 2.5 m/s, and the demo passes at RMS 46 mm. The docstring of `demos/demo_figure8.py` now
records the outcome: dense sampling plus the 0.5 s floor put the whole-lap average near
0.9 m/s. Moral: with floor-dominated timing, waypoint density *is* the speed knob.

### Yaw modes

Two modes exist. `"fixed"` holds a constant heading — right for formation flying. `"velocity"`
points the nose along the direction of travel, $\psi = \operatorname{atan2}(v_y, v_x)$ — the
instantaneous phase of the horizontal velocity, treated exactly like a communications phase:
computed on a dense 500 Hz grid, unwrapped with `np.unwrap`, differentiated with `np.gradient`
to get the yaw rate, and *held* wherever horizontal speed drops below 0.15 m/s, because the
phase of a near-zero vector is noise — a PLL coasting through a signal dropout.

## Guided code walkthrough: `quadsim/trajectory.py`

The planner's public face is the `FlatOutput` dataclass — one flat-output sample:

```python
pos: np.ndarray   # (3,)
vel: np.ndarray = field(default_factory=lambda: np.zeros(3))   # (3,)
acc: np.ndarray = field(default_factory=lambda: np.zeros(3))   # (3,)
jerk: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)
snap: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)
yaw: float = 0.0
yaw_rate: float = 0.0
```

Everything a controller needs, nothing it doesn't. The polynomial machinery rests on one small
function, `_basis_row`, which builds the row vector whose dot product with the 8 coefficients
gives the $k$-th $\tau$-derivative:

```python
def _basis_row(k: int, tau: float) -> np.ndarray:
    """Row ``r`` with ``r @ c`` = k-th tau-derivative of ``sum_j c_j tau**j`` (8 coeffs)."""
    row = np.zeros(_N_COEFF)
    for j in range(k, _N_COEFF):
        row[j] = _falling(j, k) * tau ** (j - k)
    return row
```

Inside `_solve_coefficients`, the snap cost for one segment is a closed-form $8 \times 8$ Gram
matrix — the integral $\int_0^1 \ddddot{\tau^j}\,\ddddot{\tau^k}\, d\tau$ done analytically —
and the $T_i^{-7}$ scaling appears exactly where the theory said it would:

```python
for j in range(4, _N_COEFF):
    for k in range(4, _N_COEFF):
        snap_block[j, k] = _falling(j, 4) * _falling(k, 4) / (j + k - 7)
...
    hess[sl, sl] = snap_block * t_i ** (-7)
```

The constraint rows follow the budget we counted. Note how the interior continuity rows equate
*physical-time* derivatives by scaling each side with its own segment's $T^{-k}$ — segment $i$'s
end must match segment $i{+}1$'s start:

```python
for i in range(n_seg - 1):
    for k in (1, 2, 3, 4):
        row = np.zeros(n_c)
        row[i * _N_COEFF:(i + 1) * _N_COEFF] = (
            _basis_row(k, 1.0) * self._times[i] ** (-k))
        row[(i + 1) * _N_COEFF:(i + 2) * _N_COEFF] = (
            -_basis_row(k, 0.0) * self._times[i + 1] ** (-k))
```

The KKT assembly is textbook — compare it symbol-for-symbol with the block matrix above:

```python
kkt[:n_c, :n_c] = 2.0 * hess
kkt[:n_c, n_c:] = a_mat.T
kkt[n_c:, :n_c] = a_mat
```

Evaluation applies the chain rule one last time — every derivative order gets its $T_i^{-k}$:

```python
return [(_basis_row(k, tau) @ coeffs) * t_i ** (-k) for k in range(5)]
```

Finally, see how the flatness chain is *consumed* in `quadsim/controller.py`: the reference
acceleration becomes force, and force plus yaw becomes the commanded attitude —

```python
F_des = -self.kp * e_p - self.kv * e_v + m * g * e3 + m * ref.acc
f = max(0.0, float(F_des @ (R @ e3)))
R_cmd = flat_to_rotation(F_des, ref.yaw)
```

— which is $F = m(\ddot{p} + ge_3)$ wearing feedback goggles. The class is callable
(`__call__ = eval`), so a `MinSnapTrajectory` instance drops straight into `simulate` as a
`ref_fn`.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`.

**Experiment 1 — design your own gate course.** First fly the stock course:

```bash
.venv/bin/python demos/demo_minsnap.py --no-gif
```

Note `rms_error_m` (~0.001), `total_time_s`, `peak_speed_mps` (~4.1), and look at
`out/minsnap_3d.png`. Now open `demos/demo_minsnap.py` and edit the `WAYPOINTS` array: add a
waypoint, move the finish, make the dive after gate 2 deeper — but keep W1 and W3 pinned to the
gate centers `[1.2, 0.0, 1.0]` and `[3.2, 0.8, 1.5]` so you still thread the openings. Re-run
the same command and check the 3D plot: does your path still clear the box? Notice the smooth
curve never touches your straight polyline except at the waypoints — the optimizer bows the
path to spread out curvature. When done, restore with `git checkout -- demos/demo_minsnap.py`.

**Experiment 2 — raise `avg_speed` until it breaks.** Find this vehicle's limit empirically
without editing files:

```bash
.venv/bin/python - <<'EOF'
import numpy as np, sys
sys.path.insert(0, '.')
from demos.demo_minsnap import WAYPOINTS
from quadsim.trajectory import MinSnapTrajectory
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate

for spd in (2.0, 2.2, 2.4, 2.6, 3.0, 6.0):
    traj = MinSnapTrajectory(WAYPOINTS, avg_speed=spd)
    params = QuadParams()
    hist = simulate(params, SE3Controller(params), traj,
                    QuadState.hover(WAYPOINTS[0]), traj.T + 1.0, dt=0.002)
    rms = float(hist.rms_pos_error())
    peak = float(np.max(np.linalg.norm(hist.v, axis=1)))
    print(f"avg_speed={spd:4.1f}  T={traj.T:5.2f}s  peak={peak:5.2f} m/s  rms={rms:.4f} m  "
          + ("WARN" if rms >= 0.08 else "ok"))
EOF
```

Observe: 2.2 m/s still tracks at 49 mm RMS, but 2.4 m/s is a cliff — RMS jumps to about 1.5 m
(the vehicle saturates its motors and gives up, not gradually but catastrophically). Also
observe that from 2.6 upward *nothing changes*: every segment has hit the 0.5 s floor
($T = 2.50$ s), so `avg_speed` has become a dead knob. This experiment *is* the sabotage: you
are deliberately commanding beyond vehicle authority and watching the failure mode — the same
failure the figure-8 incident hit, found the same way.

**Experiment 3 — trust, then verify, the derivatives.** Evaluate a trajectory in a shell and
finite-difference its position against its claimed velocity:

```bash
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.trajectory import MinSnapTrajectory

wp = np.array([[0., 0., 1.], [1., 0., 1.], [1., 1., 1.5], [0., 1., 1.]])
traj = MinSnapTrajectory(wp, avg_speed=1.5)
h = 1e-5
worst = 0.0
for t in np.linspace(0.1, traj.T - 0.1, 200):
    fd_vel = (traj(t + h).pos - traj(t - h).pos) / (2 * h)
    worst = max(worst, float(np.linalg.norm(fd_vel - traj(t).vel)))
print("T =", traj.T, " max |fd_vel - vel| =", worst)
EOF
```

Expect agreement around $10^{-9}$. **Break it:** in `quadsim/trajectory.py`, in
`_derivatives`, change `* t_i ** (-k)` to `* 1.0` and re-run. The mismatch jumps to order one —
you have deleted the chain rule, so the code returns $\tau$-derivatives while physics needs
$t$-derivatives. Also run `.venv/bin/python demos/demo_minsnap.py --fast --no-gif` in the broken
state: the constructor's fit check does *not* catch the bug (it only verifies position, the
$k = 0$ case, and $T_i^0 = 1$), but the controller now receives feedforward velocity and
acceleration in the wrong units of time and tracking collapses to a WARN at RMS ≈ 0.36 m — a
silent-units bug, the classic kind. Restore with `git checkout -- quadsim/trajectory.py`.

**Experiment 4 — replay the integration incident.**

```bash
.venv/bin/python - <<'EOF'
import numpy as np, sys
sys.path.insert(0, '.')
from demos.demo_figure8 import lemniscate_waypoints
from quadsim.trajectory import MinSnapTrajectory
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate

for ppl in (8, 16):
    wp = lemniscate_waypoints(2, points_per_lap=ppl)
    traj = MinSnapTrajectory(wp, avg_speed=2.5, yaw_mode="velocity")
    params = QuadParams()
    hist = simulate(params, SE3Controller(params), traj,
                    QuadState.hover(wp[0], yaw=float(traj(0.0).yaw)),
                    traj.T + 0.5, dt=0.002)
    print(f"points_per_lap={ppl}: T={traj.T:.1f}s "
          f"ref_yaw_rate_peak={np.max(np.abs(traj._yaw_rates)):.1f} rad/s "
          f"rms={hist.rms_pos_error():.4f} m")
EOF
```

Observe: 8 points/lap gives $T = 8$ s and RMS ≈ 0.25 m — more than double the demo's 0.10 m
WARN threshold; 16 points/lap gives $T = 16$ s and RMS ≈ 0.046 m, centimeter-level tracking.
Same lemniscate, same `avg_speed` argument; the only change is waypoint density interacting
with the 0.5 s floor, which halves every segment time and pushes the peak speed from about
2.5 to 3.9 m/s. On today's retuned code this replay shows a five-fold tracking degradation,
not the full 69 m divergence of the historical incident. One caution when reading the
printout: the single `ref_yaw_rate_peak` number is nearly identical in the two cases
(≈ 16 vs ≈ 15 rad/s), because in both runs the raw peak comes from one brief low-speed moment
(horizontal speed ≈ 0.4 m/s, above the 0.15 m/s hold threshold, so the yaw reference is not
held) where the velocity heading swings fast. The *sustained*
yaw-rate demand really is about double in the 8-point case (95th percentile ≈ 8.6 vs
≈ 4.1 rad/s), so judge the two runs by RMS and trajectory time, not by the peak alone.

## Homework

1. For the six-waypoint course in `demos/demo_minsnap.py` ($n = 5$ segments), compute the
   per-axis constraint budget: unknowns, constraints of each kind, and the optimizer's leftover
   freedom. Where, physically, does that freedom live?

> **Your answer:**
>
>
>

2. Minimum snap uses 7th-order polynomials. Derive the polynomial order that a
   minimum-*jerk* planner would use, and explain via the Euler–Lagrange equation why minimizing
   the $k$-th derivative yields degree $2k - 1$.

> **Your answer:**
>
>
>

3. Why would minimum-*velocity* paths — straight lines between waypoints at constant speed — be
   terrible for a quadrotor? Answer using the flatness chain: what happens to attitude and body
   rates at each waypoint corner?

> **Your answer:**
>
>
>

4. The figure-8 docstring says dense sampling plus the 0.5 s floor put the whole-lap average
   near 0.9 m/s, even though `avg_speed=2.5` is requested. Using the lemniscate geometry
   (16 chords/lap, $a = 1.2$ m), estimate the lap length, confirm every segment hits the floor,
   and compute the actual average speed. Why is `avg_speed` a dead knob here?

> **Your answer:**
>
>
>

5. Hands-on: extend Experiment 3 to check `acc` against a finite difference of `vel`, sampling
   *away* from the interior knots and then *straddling* a knot. Derivatives 1–4 are continuous
   at knots by construction — which derivative is the first one allowed to jump, and can you
   detect the jump numerically?

> **Your answer:**
>
>
>

6. In `_build_yaw_grid`, yaw is held wherever horizontal speed is below 0.15 m/s. What goes
   wrong at the lemniscate's center crossing if you remove the hold? Relate this to phase
   estimation of a weak carrier.

> **Your answer:**
>
>
>

## Hints

1. The counts are in the "constraint budget" section; the freedom is $2(n-1)$ — think about
   which quantities at interior knots the optimizer chooses rather than being told.
2. $\min \int (p^{(k)})^2 dt$ gives the Euler–Lagrange condition $p^{(2k)} = 0$; count the
   polynomial's degrees of freedom from there.
3. A corner means velocity changes direction instantaneously — what does the flatness chain say
   acceleration, hence attitude, must do, and what body rate does an instantaneous attitude
   change require?
4. Chord length of one lap ≈ sum of 16 segment distances (roughly 7.2 m); each
   $d_i / 2.5 < 0.5$ s, so $T_{\text{lap}} = 16 \times 0.5 = 8$ s.
5. Continuity is enforced only through the 4th derivative, so the first quantity allowed to
   jump is the *5th*; estimate it by finite-differencing `snap` across a knot.
6. `atan2` of a near-zero vector is noise, and at the crossing the velocity direction reverses;
   `np.gradient` of a step is a spike — of exactly the kind that broke the original figure-8.

## Further reading

1. **Mellinger & Kumar, *Minimum snap trajectory generation and control for quadrotors*, ICRA
   2011** — the source paper; short and genuinely readable now that you've seen the code.
2. **Richter, Bry & Roy, *Polynomial trajectory planning for aggressive quadrotor flight in
   dense indoor environments*, ISRR 2013** — reformulates the QP in unconstrained form for
   better conditioning and optimizes segment times too; the natural "what's next" after this
   chapter's fixed-time solver.
3. **Boyd & Vandenberghe, *Convex Optimization*, §10.1 (equality-constrained minimization)** —
   the KKT system in this chapter, stated with full generality in four pages.
