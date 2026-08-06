# Chapter 2 — Rigid-body dynamics: thirteen numbers and four knobs

## After this chapter you can …

1. Write down the quadrotor's 13-state model $[p, v, q, \omega]$ from memory and say what each block does physically.
2. Rederive every row of the mixer matrix from $\tau = r \times f$ for the X layout, and predict which way the vehicle tips when you push on any motor.
3. Explain why $\omega \times J\omega$ appears in Euler's equation and when it actually matters for this vehicle.
4. Compute the hover thrust per motor, the peak angular acceleration, and the thrust-to-weight ratio straight from `params.py`.
5. Make Kumar's "small is agile" claim quantitative: state how inertia, torque, and angular acceleration scale with vehicle size.

## The ECE bridge

| New concept | What you already know |
|---|---|
| State $[p, v, q, \omega]$, dynamics $\dot{x} = f(x, u)$ | A nonlinear state-space system, exactly like capacitor voltages and inductor currents in a circuit — 13 states, 4 inputs |
| Quaternion kinematics $\dot{q} = \tfrac12\, q \otimes [0, \omega]$ | The phasor derivative $\frac{d}{dt}e^{j\theta} = j\omega\, e^{j\theta}$: the derivative of a rotation is the rotation times the rate |
| Mixer matrix $A$ and its inverse | A change of basis between channels — like a differential/common-mode decomposition, or beam-space vs element-space in an antenna array. Four motor thrusts on one side, thrust + three torques on the other, connected by an invertible $4\times4$ linear map |
| Gyroscopic term $\omega \times J\omega$ | The cross-coupling terms $\omega L i_q$, $-\omega L i_d$ in field-oriented motor control: differentiating in a rotating frame always adds an $\omega \times$ term. Same product rule, one dimension up |
| Actuator saturation `np.clip` | Op-amp rail clipping / PA compression: the command is linear until it isn't, and everything interesting (and dangerous) happens at the rails |
| RK4 vs Euler integration | Discretizing a continuous filter: forward Euler is cheap and drifts; higher-order methods buy accuracy per sample, like bilinear vs forward-difference transforms |
| Small-is-agile scaling | Dennard scaling. Shrink a transistor and capacitance falls faster than drive current, so small transistors switch faster. Shrink a quadrotor and inertia falls faster than torque, so small quadrotors roll faster |

## Core theory

### Thirteen numbers

A rigid body free in space needs its position and orientation, plus their rates. The state here is $x = [p, v, q, \omega]$: position $p \in \mathbb{R}^3$ and velocity $v \in \mathbb{R}^3$ in the world frame, attitude quaternion $q \in \mathbb{R}^4$ (body→world, Hamilton convention $[w,x,y,z]$ from Chapter 1), and angular velocity $\omega \in \mathbb{R}^3$ in the *body* frame. That is $3+3+4+3 = 13$ numbers for a system with only 12 degrees of freedom — the quaternion carries one redundant number, paid for by the constraint $\|q\| = 1$. The simulator re-imposes that constraint by renormalizing after every step; Chapter 1's "break it" showed what happens when you don't.

The four knobs are not the motors directly but the *wrench* they jointly produce: total thrust $f$ along the body z-axis and body torque $\tau = [\tau_x, \tau_y, \tau_z]$. Four knobs, six degrees of freedom: the vehicle is underactuated. It cannot accelerate sideways without first tilting its thrust axis — which is why attitude dynamics sit inside every position maneuver, and why Chapter 3's controller is cascaded.

### Newton and Euler

Newton's part is translation, done in the world frame where momentum is simplest:

$$\dot{p} = v, \qquad \dot{v} = -g\,e_3 + \frac{f}{m}\, R\, e_3.$$

Gravity pulls straight down; thrust pushes along the body z-axis $b_3 = R e_3$, wherever the attitude currently points it. Nothing else — this model has no aerodynamic drag, which is a deliberate simplification you should remember when we go to hardware.

Euler's part is rotation. Angular momentum in the body frame is $H = J\omega$ with inertia matrix $J$. The torque law $\dot{H} = \tau$ holds in the *inertial* frame, but $J$ is only constant in the *body* frame, so we apply the rotating-frame product rule — the transport theorem: $\left(\dot H\right)_{\text{inertial}} = \left(\dot H\right)_{\text{body}} + \omega \times H$. Rearranged:

$$J\dot{\omega} = \tau - \omega \times J\omega.$$

That $\omega \times J\omega$ is the gyroscopic term, and it is exactly the $j\omega$ your phasors pick up, or the $\omega L i$ cross-coupling in dq-frame motor control: any vector described in a spinning frame appears to rotate even when nothing physical is torquing it. The classic picture is a spinning bicycle wheel held by its axle: it carries a large angular momentum vector along the axle, and when you *yaw* the axle, that momentum vector must swing — which requires a torque, so the wheel fights back by *pitching*. Rotation about one axis, while momentum exists about another, produces apparent torque about the third. One subtlety worth a mental note: this term is about the momentum of the whole rigid body. The spinning propellers carry their own angular momentum too, and this model neglects it — a real, if small, sim-to-real gap.

For our vehicle $J = \mathrm{diag}(1.43\times10^{-5},\ 1.43\times10^{-5},\ 2.89\times10^{-5})\ \mathrm{kg\,m^2}$: a flat X shape, so the z inertia is roughly the sum of the other two. For a diagonal $J$ the gyroscopic term is $\big(\omega_y\omega_z(J_{zz}{-}J_{yy}),\ \omega_x\omega_z(J_{xx}{-}J_{zz}),\ \omega_x\omega_y(J_{yy}{-}J_{xx})\big)$ — with $J_{xx}=J_{yy}$ its z-component is identically zero, and the whole thing vanishes at hover where $\omega \approx 0$. It matters when you yaw *while* rolling or pitching: aggressive, coupled maneuvers.

### The mixer, row by row

The motors produce four individual thrusts $f_1..f_4$; the physics wants $[f, \tau]$. The mixer matrix $A$ is the bookkeeping between them. In the X configuration, each arm of length $L = 46$ mm points along a diagonal, $45°$ from the body axes, so each rotor sits at $x,y$ offsets of $\pm d$ with $d = L\cos 45° = L/\sqrt{2} \approx 32.5$ mm. Numbering runs counterclockwise seen from above: rotor 1 front-left $(+d,+d,0)$, rotor 2 rear-left $(-d,+d,0)$, rotor 3 rear-right $(-d,-d,0)$, rotor 4 front-right $(+d,-d,0)$.

Each rotor pushes $f_i$ along body z, so its torque about the center of mass is $\tau_i = r_i \times f_i e_3$. Work the cross product for a rotor at $(x_i, y_i, 0)$:

$$(x_i, y_i, 0) \times (0, 0, f_i) = f_i\,(y_i,\ -x_i,\ 0).$$

Read the rows straight off this:

- **Row 1 (thrust):** total thrust is just the sum, $[1, 1, 1, 1]$.
- **Row 2 ($\tau_x$, roll):** coefficient $y_i$ per motor: rotors 1, 2 sit at $y = +d$, rotors 3, 4 at $y = -d$, giving $[d, d, -d, -d]$. Sanity check: push harder on the two left rotors and $\tau_x > 0$, which by the right-hand rule rotates $+y$ toward $+z$ — the left side lifts. Correct.
- **Row 3 ($\tau_y$, pitch):** coefficient $-x_i$: rotors 1, 4 at $x = +d$ contribute $-d$, rotors 2, 3 at $x = -d$ contribute $+d$, giving $[-d, d, d, -d]$. Push the two rear rotors, the nose dips forward — pitch.
- **Row 4 ($\tau_z$, yaw):** this one is *not* a lever-arm effect. Each spinning rotor drags air around and receives an equal-and-opposite reaction torque about its own axis, proportional to its thrust: $\tau_{\text{drag}} = c_\tau f_i$, with $c_\tau = 6$ mm playing the role of an effective moment arm. Rotors 1 and 3 spin CCW (reaction on the body: $-z$), rotors 2 and 4 spin CW ($+z$), giving $[-c, c, -c, c]$. Diagonal pairs share a spin direction so that at hover the four reactions cancel; yawing means deliberately unbalancing them.

Because $A$ is invertible, the controller can think entirely in $[f, \tau]$ and let `mix` solve $f_{1..4} = A^{-1}[f, \tau]$ — the same move as designing in beam-space and converting to element feeds at the last moment.

### Saturation, once per step

Real motors cannot pull ($f_i \geq 0$) and cannot exceed $f_{\max} = 0.16$ N each. The simulator clips each motor thrust to $[0, 0.16]$ and then maps the *clipped* motors back through $A$ to get the wrench the vehicle actually receives. Crucially, this happens **once** per 2 ms step, before integration, and the result is held constant across the step — a zero-order hold, exactly like a DAC output between samples. That models reality (the motor command is updated at the control rate, not continuously) and it keeps the ODE right-hand side smooth over the step, which RK4's accuracy guarantees quietly assume. Clipping inside the derivative would pretend the electronics re-decide mid-step and would feed RK4 a kinked function.

Note what clipping does to the *torque*: `np.clip` acts per motor, so when a demanded wrench hits the rails, the achieved $[f, \tau]$ is not a scaled-down version of the demand — it is distorted. Saturation doesn't just weaken your authority, it bends it. You will watch this happen in Experiment 2.

### RK4 versus Euler

Forward Euler advances $x_{k+1} = x_k + dt\, f(x_k)$: one derivative evaluation, local error $O(dt^2)$. Classic Runge–Kutta 4 blends four evaluations — at the start, two midpoints, and the end — for local error $O(dt^5)$. With $dt = 2$ ms and angular accelerations that reach hundreds of $\mathrm{rad/s^2}$, Euler's drift would masquerade as vehicle behavior; RK4 makes integration error negligible so that everything you observe is physics or control, not numerics. Four derivative calls per step is the price, and for 13 states it is nothing.

### Small is agile, quantitatively

Now Kumar's scaling argument, with our numbers. Scale every length of the vehicle by $s$, same materials. Mass scales with volume: $m \propto s^3$. Inertia is mass times length squared: $J \propto s^5$ — check against `params.py`: $mL^2 = 0.033 \times 0.046^2 = 7.0\times10^{-5}$, and $J_{xx} = 1.43\times10^{-5} \approx 0.2\, mL^2$, a sensible geometric factor. Rotor thrust scales with disk area times tip speed squared; at constant tip speed (Mach scaling) $F \propto s^2$, so torque, thrust times lever arm, scales as $\tau \propto s^3$. Therefore peak angular acceleration scales as

$$\alpha_{\max} = \frac{\tau_{\max}}{J} \propto \frac{s^3}{s^5} = \frac{1}{s^2}.$$

(Under Froude scaling, $v_{\text{tip}} \propto \sqrt{s}$, you get $1/s$ — either way, shrinking wins.) For our vehicle: max roll torque is two motors at full thrust against two at zero, $\tau_x^{\max} = 2 d f_{\text{motor max}} = 2 \times 0.0325 \times 0.16 \approx 0.0104$ N·m, so $\alpha_{\max} = 0.0104 / 1.43\times10^{-5} \approx 730\ \mathrm{rad/s^2}$ — about $42{,}000°/\mathrm{s^2}$, a 90° bank from rest in roughly 66 ms. Scale this design up $10\times$ (0.46 m arms, ~33 kg) and $\alpha_{\max}$ collapses by $100\times$ to ~7 $\mathrm{rad/s^2}$. This is why the agile-flight literature is full of palm-sized vehicles, and why this project simulates a 33-gram one.

## Guided code walkthrough

Open `quadsim/dynamics.py`. The state is a plain dataclass with vector round-tripping (`as_vector` / `from_vector`) so the integrator can treat it as one flat $\mathbb{R}^{13}$ array. The equations of motion are almost a transliteration of the theory:

```python
# quadsim/dynamics.py — derivative()
v_dot = -self.params.g * _E3 + (f / self.params.m) * (R @ _E3)
q_dot = quat_derivative(q, w)
w_dot = self._J_inv @ (tau - np.cross(w, self._J @ w))
```

Line 1 is Newton, line 3 is Euler with the gyroscopic term written exactly as $J^{-1}(\tau - \omega \times J\omega)$; `quat_derivative` is Chapter 1's $\tfrac12 q \otimes [0,\omega]$. The mixer is precomputed in the constructor:

```python
# quadsim/dynamics.py — __init__()
d = params.L / np.sqrt(2.0)
c = params.c_tau
self.A = np.array([
    [1.0, 1.0, 1.0, 1.0],
    [d, d, -d, -d],
    [-d, d, d, -d],
    [-c, c, -c, c],
])
```

Compare each row against the derivation above; they must match sign for sign. `mix` applies $A^{-1}$ then clips per motor; `unmix` maps motors back to the wrench. The step glues it together:

```python
# quadsim/dynamics.py — step()
f_act, tau_act = self.unmix(self.mix(f, tau))
```

That single line *is* the actuator model: demand in, achievable wrench out, held constant while the four RK4 stages run, then `quat_normalize` repairs the unit constraint.

`quadsim/params.py` holds the Crazyflie-2-like numbers: `m = 0.033`, `L = 0.046`, `c_tau = 0.006`, `f_motor_max = 0.16`, plus two conveniences, `f_max` ($= 4 f_{\text{motor max}} = 0.64$ N) and `weight` ($= mg \approx 0.324$ N). Divide those: thrust-to-weight $\approx 1.98$. Hold that number; Experiment 2 spends it.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`. The `sed -i ''` syntax is macOS; always run the revert line, and verify with the final `grep`.

**Experiment 1 — hover arithmetic.** Ask the mixer for pure hover thrust and no torque:

```sh
.venv/bin/python -c "
import numpy as np
from quadsim.params import QuadParams
from quadsim.dynamics import QuadrotorDynamics
p = QuadParams()
dyn = QuadrotorDynamics(p)
motors = dyn.mix(p.m * p.g, np.zeros(3))
print('per-motor thrust [N]:', motors)
print('m*g/4 =', p.m * p.g / 4)
print('fraction of f_motor_max:', motors[0] / p.f_motor_max)
"
```

Observe: all four motors at exactly $mg/4 = 0.0809$ N — the mixer's first row summed against equal motors — sitting at ~51% of the 0.16 N rail. Half the throttle range is head-room for maneuvering. That margin is the next experiment's victim.

**Experiment 2 — the mass ladder (break it: doubled mass).** The demo's controller reads the same `QuadParams`, so it always *knows* the mass; what changes is what the motors can deliver.

```sh
.venv/bin/python demos/demo_minsnap.py --fast --no-gif    # baseline m=0.033: rms ~0.0006 m PASS
sed -i '' 's/m: float = 0.033/m: float = 0.0165/' quadsim/params.py
.venv/bin/python demos/demo_minsnap.py --fast --no-gif    # half mass: rms ~0.003 m PASS
sed -i '' 's/m: float = 0.0165/m: float = 0.040/' quadsim/params.py
.venv/bin/python demos/demo_minsnap.py --fast --no-gif    # +21%: rms ~0.10 m WARN
sed -i '' 's/m: float = 0.040/m: float = 0.066/' quadsim/params.py
.venv/bin/python demos/demo_minsnap.py --fast --no-gif    # doubled: rms ~8.8 m, gone
sed -i '' 's/m: float = 0.066/m: float = 0.033/' quadsim/params.py   # ALWAYS restore
grep 'm: float' quadsim/params.py                          # must print 0.033
```

Observe the cliff: halving the mass barely changes anything, +21% already breaches the 0.08 m target, and +50% or more is a crash. Why so nonlinear? At $m = 0.066$, hover alone needs $0.162$ N per motor against a $0.16$ N rail — thrust-to-weight $0.99$, the vehicle cannot even hold altitude. At $m = 0.040$ hover is fine but the course's peak accelerations push motors into the rails, and per-motor clipping distorts the commanded torque. This is the sabotage: it teaches that saturation is a cliff, not a slope, and that Kumar's "small is agile" has a hard flip side — grams are a budget.

**Experiment 3 — the mixer sign flip (break it: maiden-flight crash, in sim).** Flip motor 1's sign in the roll row, exactly the miswiring or mis-numbered-motor error that destroys real vehicles on their first takeoff:

```sh
sed -i '' 's/\[d, d, -d, -d\]/[-d, d, -d, -d]/' quadsim/dynamics.py
.venv/bin/python demos/demo_hover.py --fast --no-gif
sed -i '' 's/\[-d, d, -d, -d\]/[d, d, -d, -d]/' quadsim/dynamics.py
grep -n 'd, d, -d, -d' quadsim/dynamics.py                 # must show the restored row
```

Observe: baseline hover settles in ~1.2 s; sabotaged, `settle_time_s: nan` and a final error near 28 m — the vehicle flips and departs. The controller asks for a small corrective roll, the corrupted mixer delivers partly the *opposite* roll, the error grows, the "correction" grows: positive feedback, exactly an inverted feedback polarity in an op-amp loop. One sign, total loss. This is why real builds get a hand-on-the-vehicle prop-direction check before the first flight.

## Homework

1. The state has 13 numbers but the vehicle has 12 degrees of freedom. Where is the redundancy, what constraint removes it, and which exact line of `quadsim/dynamics.py` enforces that constraint?

> **Your answer:**
<br><br><br>

2. Rederive the pitch row $[-d, d, d, -d]$ of $A$ from $\tau_i = r_i \times f_i e_3$, showing the cross product for rotor 1 explicitly. Then explain in one sentence, without math, why pushing harder on the *front* rotors gives *negative* $\tau_y$.

> **Your answer:**
<br><br><br>

3. For our $J$ with $J_{xx} = J_{yy}$, show that the z-component of $\omega \times J\omega$ is always zero, and describe one flight maneuver where the remaining components are large. What would change if the propellers' own spin momentum were modeled too?

> **Your answer:**
<br><br><br>

4. Compute this vehicle's maximum yaw angular acceleration from `params.py` the way the chapter computed the roll value (~730 rad/s²). Roughly how many times weaker is yaw, and what is the physical reason?

> **Your answer:**
<br><br><br>

5. Suppose `step` clipped motor thrusts inside `derivative` at every RK4 stage instead of once before integrating. Name two distinct things that would be wrong with that, one about physics and one about numerics.

> **Your answer:**
<br><br><br>

6. Hands-on: predict, then verify. If you doubled *only* `f_motor_max` to 0.32 N (mass unchanged), what happens to hover thrust per motor, thrust-to-weight, and the Experiment 2 crash at $m = 0.066$? Make the edit, run `demo_minsnap.py --fast --no-gif` at $m = 0.066$, record the result, and restore both values.

> **Your answer:**
<br><br><br>

## Hints

1. Count constraints, not just numbers — and look at the last two lines of `step`.
2. The pitch coefficient is $-x_i$; rotor 1 sits at $x = +d$.
3. Write out $\omega \times J\omega$ for diagonal $J$ and watch $(J_{yy} - J_{xx})$ appear; think "yaw while rolling" for the maneuver.
4. Max yaw torque is $2\, c_\tau f_{\text{motor max}}$; compare $c_\tau = 6$ mm with $d = 32.5$ mm, and remember yaw uses $J_{zz}$.
5. ZOH is a claim about the *electronics*; RK4's error bound is a claim about *smoothness* of the right-hand side.
6. Hover per motor doesn't depend on the rail at all; the crash was a thrust-budget problem, so predict which symptom disappears.

## Further reading

1. **Kumar & Michael, "Opportunities and challenges with autonomous micro aerial vehicles," IJRR 2012** — the small-is-agile scaling laws from the source, including the Froude/Mach distinction this chapter compressed.
2. **Mahony, Kumar & Corke, "Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor," IEEE Robotics & Automation Magazine 2012** — the standard tutorial for exactly the model `dynamics.py` implements, plus the aerodynamic effects it leaves out.
3. **Mellinger & Kumar, "Minimum snap trajectory generation and control for quadrotors," ICRA 2011, §II** — the same dynamics written in the notation Chapters 3–4 will use; read the model section now, the rest later.
4. **Press et al., *Numerical Recipes*, ch. 17 (ODE integration)** — a practical, engineer-friendly account of why RK4 earns its four function evaluations.
