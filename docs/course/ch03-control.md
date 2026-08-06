# Chapter 3 — Control: from the PID you know to geometry you'll love

You already know feedback control; you learned it with op-amps and transfer functions. This
chapter shows you that `quadsim/controller.py` is the same discipline — pole placement,
cascaded loops, feedforward — with one genuinely new idea on top: doing PD control *directly on
the rotation group* so the math survives aggressive tilt. Read with the file open beside you.

## After this chapter you can …

1. Convert the controller's `kp, kv, kR, kw` gains into natural frequencies and damping ratios,
   and predict step-response overshoot before running the sim.
2. Explain the cascade — position loop commands attitude, attitude loop commands torque — and
   compute the ~8× time-scale separation from the actual default gains.
3. Walk through every term of the Lee-2010 SE(3) control law as implemented in
   `quadsim/controller.py`, including the thrust projection and both feedforward terms.
4. Say precisely why Euler-angle control fails at large tilt and what the matrix error $e_R$
   fixes.
5. Predict what actuator saturation does to a feedback loop, and demonstrate it.

## The ECE bridge

| New concept | What you already know |
|---|---|
| $k_p, k_v$ position gains | Coefficients of a second-order characteristic polynomial $s^2 + 2\zeta\omega_n s + \omega_n^2$ — a series RLC step response with $\omega_n = 1/\sqrt{LC}$, $\zeta = \frac{R}{2}\sqrt{C/L}$ |
| Cascaded position/attitude loops | Inner current loop + outer voltage loop in an SMPS, or a PLL inside a data link: the inner loop must be fast enough to look like a unity-gain block to the outer one |
| Attitude error $e_R = \frac{1}{2}\mathrm{vee}(R_c^\top R - R^\top R_c)$ | A phase detector: output $\propto \sin(\Delta\phi)$, linear for small error, well defined for any error |
| Feedforward $\omega_{ff}, \alpha_{ff}$ | Pre-emphasis / feedforward compensation: inject the known part of the command so feedback only handles the *unknown* part |
| Finite-difference feedforward (`rot_log(...)/d`) | A discrete differentiator — $|H(e^{j\omega})|$ grows with frequency, so it amplifies any kink or noise in the reference |
| Motor saturation in `dynamics.mix` | Amplifier clipping inside a feedback loop: loop gain collapses to zero the instant you clip, and the controller's polite linear analysis is suspended |

## Core theory

### PD control is pole placement on a double integrator

Suppose the attitude loop were perfect, so the vehicle could produce any world-frame force
instantly. Newton then gives $m\ddot{p} = F - mg e_3$, and if the controller chooses
$F = -k_p e_p - k_v e_v + mg e_3 + m\ddot{p}_{ref}$ with $e_p = p - p_{ref}$, the error obeys

$$m\ddot{e}_p + k_v \dot{e}_p + k_p e_p = 0 \quad\Longleftrightarrow\quad s^2 + \frac{k_v}{m}s + \frac{k_p}{m} = 0.$$

Matching $s^2 + 2\zeta\omega_n s + \omega_n^2$: $\omega_n = \sqrt{k_p/m}$ and
$\zeta = k_v/(2\sqrt{k_p m})$. The defaults in `controller.py` are $k_p = 16m$, $k_v = 8m$, so

$$\omega_n = \sqrt{16} = 4\ \text{rad/s}, \qquad \zeta = \frac{8}{2\cdot 4} = 1.0.$$

Critically damped, bandwidth about 0.64 Hz. The gains are *scaled by mass* on purpose: the
error dynamics — and every plot you make — stay identical if you change `m` in `params.py`.
This is the same trick as normalizing a filter design to $\omega_n = 1$.

### The cascade and the 8× rule

The vehicle cannot produce an arbitrary force: all four rotors point along body $z$, so it can
only produce $f\,Re_3$ — a magnitude along whatever direction it is currently tilted. Torque,
however, it *can* produce in any body direction, and the body is light: torque changes tilt
very fast. So we split the problem. The **position loop** computes the force it wishes it had,
$F_{des}$, and converts it into a commanded attitude $R_{cmd}$ (tilt so body $z$ points along
$F_{des}$). The **attitude loop** computes the torque that drives $R \to R_{cmd}$.

The outer loop's design pretended the inner loop was instantaneous. That lie is acceptable only
if the inner loop is much faster. In `compute`, torque is
$\tau = J(-k_R e_R - k_\omega e_\omega) + \ldots$, and since $J\dot{\omega} \approx \tau$, the
$J$ cancels: the rotational error obeys $\ddot{e} + k_\omega \dot{e} + k_R e \approx 0$ with
the gains acting directly as $\omega_n^2$ and $2\zeta\omega_n$ (that is why `kR`, `kw` are not
mass-scaled — the inertia is already factored out). With the defaults `kR = [1000, 1000, 100]`,
`kw = [63, 63, 20]`:

- roll/pitch: $\omega_n = \sqrt{1000} \approx 31.6$ rad/s, $\zeta = 63/(2\sqrt{1000}) \approx 1.0$;
- yaw: $\omega_n = \sqrt{100} = 10$ rad/s, $\zeta = 20/(2\cdot 10) = 1.0$.

Roll/pitch is what the position loop leans on, and $31.6 / 4 \approx 7.9$: the inner loop is
**8× faster** than the outer one. Within the outer loop's 4 rad/s band, the attitude loop
contributes almost no phase lag — exactly the assumption an op-amp circuit makes about the
op-amp inside its gain-bandwidth. Erode that ratio (Experiment 1) and the outer loop starts
eating its own phase margin. Yaw can afford to be slower because position tracking never
depends on it.

### Why Euler angles break, and what SE(3) control does instead

The classic approach parameterizes attitude with roll-pitch-yaw and runs three little PID loops.
The catch: the map from Euler-angle rates to body rates has a Jacobian containing
$1/\cos(\text{pitch})$ terms — at 90° pitch it is singular (gimbal lock, Module 1), and well
before 90° it is badly conditioned, so the "gains" the vehicle actually feels vary wildly with
tilt. Euler PID is a linearization around hover; a quadrotor pulling through a fast figure-8
banks far outside that neighborhood.

The Lee–Leok–McClamroch controller never parameterizes. It keeps the attitude as a rotation
matrix $R \in SO(3)$ and defines the error *on the group*:

$$e_R = \tfrac{1}{2}\,\mathrm{vee}\!\left(R_{cmd}^\top R - R^\top R_{cmd}\right).$$

$R_{cmd}^\top R$ is the relative rotation "actual as seen from commanded". Its antisymmetric
part, extracted by `vee`, is a 3-vector of length $\sin\theta$ per axis — your phase detector:
linear near zero, smooth and well defined at any tilt, no singular Jacobian anywhere. PD on
$e_R$ therefore behaves the same whether the vehicle is level or 60° banked. That is the whole
pitch of geometric control: same PD instincts, error defined where the state actually lives.

### Saturation

The controller returns an idealized wrench $(f, \tau)$. Reality intrudes in
`quadsim/dynamics.py`: `mix` solves for the four per-motor thrusts and clips each to
$[0, 0.16]$ N, and `unmix` recomputes the wrench actually applied — which is what `sim.py`
records in `History.f` and `History.tau`. Two consequences. First, thrust and torque share the
same four motors, so a large roll-torque demand steals headroom from thrust and vice versa —
the clipped wrench is not just "smaller", it is *distorted*. Second, while any motor is
clipped, that channel's loop gain is effectively zero: the controller keeps shouting and
nothing more happens, exactly like an op-amp stuck at its rail. Every divergence you will
create in the experiments below passes through saturation on its way down.

## Guided code walkthrough — `quadsim/controller.py`

The spec for this file is `SPEC.md` § `quadsim/controller.py`; the code follows it line for
line. Top to bottom:

**`flat_to_rotation(F, yaw)`** builds $R_{cmd}$ from a desired world force and a heading:

```python
b3 = F / norm_F
b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
c = np.cross(b3, b1c)
...
b2 = c / np.linalg.norm(c)
b1 = np.cross(b2, b3)
return np.column_stack([b1, b2, b3])
```

Body $z$ goes along the force; body $x$ gets as close to the heading as the tilt allows. The
two guards handle $F \approx 0$ (free fall — point up) and $b_3$ parallel to the heading
(pick a perpendicular heading instead), the two degenerate cross products.

**Default gains** in `SE3Controller.__init__` are the numbers we analyzed:

```python
self.kp = m * np.array([16.0, 16.0, 16.0]) if kp is None else ...
self.kv = m * np.array([8.0, 8.0, 8.0]) if kv is None else ...
self.kR = np.array([1000.0, 1000.0, 100.0]) if kR is None else ...
self.kw = np.array([63.0, 63.0, 20.0]) if kw is None else ...
```

**`compute` — position loop.** PD plus gravity plus acceleration feedforward, then the thrust
projection:

```python
F_des = -self.kp * e_p - self.kv * e_v + m * g * e3 + m * ref.acc
f = max(0.0, float(F_des @ (R @ e3)))
R_cmd = flat_to_rotation(F_des, ref.yaw)
```

The projection $f = F_{des} \cdot Re_3$ is a subtle, important choice. The vehicle can only
push along its *current* body $z$, which is $Re_3$. Commanding $\|F_{des}\|$ would apply the
full magnitude in the wrong direction while the attitude is still catching up; projecting
commands exactly the useful component and lets the attitude loop rotate the thrust axis into
place. The `max(0, ...)` is because rotors cannot pull.

**Attitude feedforward.** The commanded rotation is a pure function of time (feedback terms
deliberately excluded so the reference stays deterministic), and its rate and acceleration are
obtained by finite differences on the rotation group:

```python
def R_ff(s: float) -> np.ndarray:
    r = ref_fn(s)
    return flat_to_rotation(m * r.acc + m * g * e3, r.yaw)

w_ff = rot_log(R_ff_0.T @ R_ff(t + d)) / d
w_ff_m = rot_log(R_ff(t - d).T @ R_ff_0) / d
a_ff = (w_ff - w_ff_m) / d
```

`rot_log` (Module 1) turns a small relative rotation into an axis-angle vector; dividing by
$d$ gives an angular rate, and differencing two one-sided rates gives an angular acceleration.
This is the SO(3) version of $\dot{x} \approx (x(t{+}d) - x(t))/d$ — with the same weakness:
a differentiator amplifies reference roughness, which Experiment 3 will make visible.

**Attitude loop.** The two errors and the torque:

```python
e_R = 0.5 * vee(R_cmd.T @ R - R.T @ R_cmd)
e_w = w - R.T @ R_cmd @ w_ff
tau = (
    J @ (-self.kR * e_R - self.kw * e_w)
    + np.cross(w, J @ w)
    - J @ (hat(w) @ R.T @ R_cmd @ w_ff - R.T @ R_cmd @ a_ff)
)
```

$e_\omega$ compares body rates *in the same frame*: $\omega_{ff}$ lives in the commanded body
frame, so it is transported into the actual body frame by $R^\top R_{cmd}$ before subtracting —
you cannot subtract vectors expressed in different frames, any more than you can subtract
phasors referenced to different carriers. In $\tau$: the first term is the PD law (with $J$
factored in so `kR`, `kw` are pure $s^{-2}$, $s^{-1}$ numbers); `np.cross(w, J @ w)` cancels
the gyroscopic term $\omega \times J\omega$ from Euler's equation (Module 2) so the loop sees a
clean double integrator; the last line is the feedforward torque — the transported-rate term
$\hat{\omega}R^\top R_{cmd}\omega_{ff}$ accounts for the transport frame itself rotating
(differentiating $R^\top R_{cmd}\omega_{ff}$ by the product rule), and $R^\top R_{cmd}\alpha_{ff}$
injects the torque the maneuver is *known* to require, so feedback only cleans up the residual.
The docstring's last line is the saturation contract: "Actuator saturation is applied by the
dynamics, not here."

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`.

### Experiment 1 — starve the inner loop (break it: kR/16)

Scale `kR` down and watch the time-scale separation erode on a one-lap figure-8:

```bash
MPLBACKEND=Agg .venv/bin/python - <<'EOF'
import numpy as np
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate
from quadsim.trajectory import MinSnapTrajectory

th = np.linspace(0.0, 2.0 * np.pi, 17)          # one figure-8 lap, 16 waypoints
wp = np.column_stack([1.2 * np.sin(th), 1.2 * np.sin(th) * np.cos(th), np.full(17, 1.0)])
traj = MinSnapTrajectory(wp, avg_speed=2.5, yaw_mode="velocity")
params = QuadParams()
for name, scale in [("stock", 1.0), ("kR/2", 0.5), ("kR/4", 0.25), ("kR/16", 1 / 16)]:
    ctrl = SE3Controller(params, kR=np.array([1000.0, 1000.0, 100.0]) * scale)
    state0 = QuadState.hover(wp[0], yaw=float(traj(0.0).yaw))
    hist = simulate(params, ctrl, traj, state0, traj.T + 0.5)
    print(f"{name:6s} rms {hist.rms_pos_error():8.4f} m   max {hist.pos_error().max():8.3f} m")
EOF
```

**Observe:** RMS error goes 0.064 → 0.083 → 0.170 → 61 m. Halving `kR` drops the inner
$\omega_n$ to $\sqrt{500} \approx 22$ rad/s (separation 5.6×): the vehicle leans late into
every turn and tracking degrades 30%. At `kR/4` the ratio is ~4× and the path visibly weaves.
**The sabotage is `kR/16`:** inner $\omega_n = 7.9$ rad/s, barely 2× the outer loop — the
cascade assumption collapses, the loops fight, saturation takes over, and the vehicle departs
by 150 m. The 8× rule is not a convention; it is a stability margin.

### Experiment 2 — double $k_p$ with $k_v$ fixed (underdamped ringing)

Predict first: doubling `kp` gives $\omega_n = \sqrt{32} = 5.66$ rad/s and
$\zeta = 8/(2\cdot 5.66) = 0.707$ — the Butterworth point, ideal overshoot
$e^{-\pi\zeta/\sqrt{1-\zeta^2}} \approx 4\%$. Quadrupling gives $\zeta = 0.5$, ideal 16%. Now
measure a 0.4 m step:

```bash
MPLBACKEND=Agg .venv/bin/python - <<'EOF'
import numpy as np
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate
from quadsim.trajectory import hover_ref
from quadsim.viz import plot_tracking

params = QuadParams()
for name, scale in [("stock", 1.0), ("kp_x2", 2.0), ("kp_x4", 4.0)]:
    ctrl = SE3Controller(params, kp=params.m * 16.0 * scale * np.ones(3))
    state0 = QuadState.hover(np.array([0.4, 0.0, 1.0]))     # 0.4 m step in x
    hist = simulate(params, ctrl, hover_ref(np.array([0.0, 0.0, 1.0])), state0, 4.0)
    overshoot = -hist.p[:, 0].min() / 0.4 * 100.0
    print(f"{name}: overshoot {overshoot:5.1f} %   final err {hist.pos_error()[-1]*1e3:.2f} mm")
    plot_tracking(hist, save=f"out/step_{name}.png", title=f"0.4 m step, {name}")
EOF
```

**Observe:** stock overshoots 0% (critically damped, as designed); `kp_x2` overshoots 7.5%;
`kp_x4` overshoots 38% and rings — open the three `out/step_*.png` and compare the $x$ traces
to an RLC step response. **What the deviation teaches:** measured overshoot exceeds the ideal
second-order prediction (7.5% vs 4%, 38% vs 16%) because the model assumed a perfect inner
loop. Raising the outer $\omega_n$ toward the attitude loop's band makes its phase lag felt —
you are watching time-scale separation erode from the outer side this time.

### Experiment 3 — zero the feedforward (break it: fly it fast)

Every use of `rot_log` inside `compute` builds feedforward, so monkeypatching it to zero kills
`w_ff` and `a_ff` and nothing else (equivalently: edit `controller.py`, set them to zero, and
revert after):

```bash
MPLBACKEND=Agg .venv/bin/python - <<'EOF'
import numpy as np
import quadsim.controller as C
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.sim import simulate
from quadsim.trajectory import MinSnapTrajectory

def lemniscate(laps, n):
    th = np.linspace(0.0, 2.0 * np.pi * laps, laps * n + 1)
    return np.column_stack([1.2 * np.sin(th), 1.2 * np.sin(th) * np.cos(th),
                            np.full(th.shape, 1.0)])

params = QuadParams()
real_rot_log = C.rot_log                          # keep a handle on the real one

def run(traj, wp, use_ff):
    C.rot_log = real_rot_log if use_ff else (lambda R: np.zeros(3))  # kill w_ff, a_ff
    ctrl = C.SE3Controller(params)
    state0 = QuadState.hover(wp[0], yaw=float(traj(0.0).yaw))
    hist = simulate(params, ctrl, traj, state0, traj.T + 0.5)
    pk = float(np.max(np.linalg.norm(hist.v, axis=1)))
    print(f"  {'with ff':8s}" if use_ff else f"  {'no ff':8s}",
          f"rms {hist.rms_pos_error():8.4f} m   peak speed {pk:6.2f} m/s")

print("gentle (demo-style: 16 wp/lap, velocity yaw)")
wp = lemniscate(1, 16)
traj = MinSnapTrajectory(wp, avg_speed=2.5, yaw_mode="velocity")
run(traj, wp, True); run(traj, wp, False)

print("aggressive (8 wp/lap, 0.5 s segments, fixed yaw)")
wp = lemniscate(2, 8)
traj = MinSnapTrajectory(wp, segment_times=np.full(wp.shape[0] - 1, 0.5))
run(traj, wp, True); run(traj, wp, False)
C.rot_log = real_rot_log
EOF
```

**Observe — and be honest about it.** On the gentle demo-style lap, removing feedforward
*improves* RMS (0.064 → 0.031 m). Surprised? Good. At 2.5 m/s the tilts are small enough that
feedback alone tracks fine, while the finite-difference $\alpha_{ff}$ is a double
differentiator chewing on a reference whose velocity-mode yaw is only piecewise-smooth — it
injects torque spikes (peaks over 250 rad/s² if you probe it). Feedforward is only as good as
the smoothness of what you differentiate. **The sabotage is the aggressive run:** at 3.8 m/s
peak, with feedforward the tracking is 0.043 m RMS; without it the vehicle *diverges*
(RMS 87 m — a tumble through saturation). Feedback corrects errors after they appear;
at aggressive speeds the errors appear faster than a 4 rad/s loop can answer, and the
lean-into-the-turn-early torque must be supplied in advance or not at all.

## Homework

1. Derive $\omega_n$ and $\zeta$ for the *vertical* (z) position channel from the default
   gains. Why do the same numbers come out for x, y, and z even though gravity only acts on z?

> **Your answer:**



2. The attitude gains `kR = [1000, 1000, 100]` are not multiplied by mass or inertia in
   `__init__` — yet the torque line starts with `J @ (...)`. Explain why this makes the
   attitude error dynamics independent of `J`, and what units `kR` and `kw` therefore carry.

> **Your answer:**



3. Why does `compute` project thrust as `F_des @ (R @ e3)` instead of using
   `np.linalg.norm(F_des)`? Describe a concrete situation (vehicle level, $F_{des}$ 45° off
   vertical) and compare what each choice would command.

> **Your answer:**



4. In `e_w = w - R.T @ R_cmd @ w_ff`, what goes wrong physically if you write
   `w - w_ff` instead? Try it in a copy of the code on the aggressive figure-8 from
   Experiment 3 and report the RMS error.

> **Your answer:**



5. Experiment 2's overshoot exceeded the ideal second-order prediction, and the gap grew with
   `kp`. Using the cascade story, explain the mechanism, and estimate at what outer
   $\omega_n$ (in rad/s) you would expect the design to become unusable if the inner loop sits
   at 31.6 rad/s.

> **Your answer:**



6. In Experiment 3 the *gentle* run got better without feedforward. List the two independent
   reasons discussed in this chapter, and propose one change to the *trajectory* (not the
   controller) that would shrink the harmful one.

> **Your answer:**



## Hints

1. Write the closed-loop ODE for $e_z$; note $mg e_3$ in `F_des` meets $-mg e_3$ in the
   dynamics before the error equation forms.
2. Substitute $\tau$ into $J\dot{\omega} = \tau - \omega \times J\omega$ and watch what
   cancels; then unit-check $\omega_n^2$.
3. Compute $\|F_{des}\|\cos 45°$ vs $\|F_{des}\|$ applied along world z while $R = I$; which
   one overshoots in altitude?
4. Frames. $\omega$ is expressed in the actual body frame; which frame is $\omega_{ff}$
   expressed in?
5. The inner loop contributes roughly $\arctan(\omega/\omega_{n,inner})$-ish phase lag inside
   the outer crossover; think phase margin, and recall the 5–10× rule of thumb.
6. One reason is about the differentiator, one about how hard feedback has to work; the
   trajectory knob is in how yaw is generated (see `yaw_mode` in Experiment 3's two runs).

## Further reading

1. Lee, Leok, McClamroch, *Geometric Tracking Control of a Quadrotor UAV on SE(3)*, CDC 2010 —
   the six equations of `compute` with stability proofs; short and readable after this chapter.
2. Mellinger & Kumar, *Minimum Snap Trajectory Generation and Control for Quadrotors*, ICRA
   2011 — where $F_{des}$-from-flat-outputs comes from; also Chapter 4's main text.
3. Åström & Murray, *Feedback Systems*, ch. 11 (PID) — the cleanest treatment of why derivative
   action is damping and how cascaded loops are tuned; free online.
4. Crazyflie firmware `controller_mellinger.c` — this chapter's controller in C on real
   hardware; diff it against `controller.py` as a preview of Module 7.
