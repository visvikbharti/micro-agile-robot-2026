# Chapter 0 — Why robots fly: the project and your toolkit

Welcome. Before we touch a single equation of motion, this chapter gets three things settled:
*why* this project exists (the story it recreates), *how* we will learn (a specific method, not
vibes), and *where* everything lives in the repository. It ends with the notation contract that
every later chapter — and every line of code — obeys.

## After this chapter you can …

1. Retell the "small is agile" scaling argument from Vijay Kumar's TED talk and say which three
   ideas from the talk this repo implements from scratch.
2. Navigate the repository confidently: say what each file in `quadsim/` does, why `SPEC.md`
   exists, and where the images and GIFs in `out/` come from.
3. Run the test suite and any demo from the command line using the project's virtual
   environment, and interpret a demo's `PASS`/`WARN` output.
4. Read and use the project's notation — frames, $e_3$, $R$, $q$, $\omega$, $f$, $\tau$ —
   without flipping back to look things up.
5. Diagnose the single most common tooling failure (wrong Python interpreter) in under a minute.

## The ECE bridge

Nothing in this chapter is truly foreign to you; here is the dictionary.

| New thing here | What you already know |
|---|---|
| The quadrotor as 13 states, 4 inputs | State-space: $\dot{x} = f(x, u)$, just nonlinear and with a quaternion inside the state vector |
| World frame vs body frame | I/Q demodulation: the body frame is a *rotating reference frame*, exactly like viewing a passband signal in the carrier's frame at baseband |
| Quaternion $q$ | The phasor $e^{j\theta}$ one dimension up: a unit-norm object whose *multiplication* composes rotations, the way multiplying phasors adds phases |
| Control loop at $dt = 0.002$ s (500 Hz) | Discrete-time DSP: sample rates, and the rule of thumb that the loop rate must comfortably exceed the plant's bandwidth |
| `SPEC.md` | A datasheet / interface control document: register maps, pin conventions, signal polarities. You never argue with the datasheet mid-project |
| `pytest` suite | A testbench: automated stimulus + checked responses, run after every change, like regression tests on an RTL design |
| The venv `.venv/` | A pinned toolchain: the same reason you lock one compiler/SDK version for an embedded target instead of "whatever is on the machine" |
| Thrust $f$ and torque $\tau$ | The actuator drive signals — the analog outputs of your controller. On real hardware they become four PWM duty cycles (Chapter 7 territory, your home turf) |

## Core theory: the story, the argument, the method

### The talk and the three ideas

In 2012 Vijay Kumar showed a TED audience palm-sized quadrotors doing things that looked
impossible: snapping through thrown hoops, flying aggressive figure-eights, and — the finale —
nine robots morphing between formations in tight airspace. The magic decomposed into three
concrete pieces of engineering, and this repository implements all three from scratch on plain
numpy/scipy/matplotlib:

1. **Minimum-snap trajectories** (Mellinger & Kumar, 2011) — planning motion so smooth that the
   motors are never asked for violent commands. This is why the flight looks *graceful*.
2. **Geometric SE(3) control** (Lee, Leok & McClamroch, 2010) — feedback done directly on the
   rotation group, with no small-angle approximation, so large tilts don't break the math. This
   is why the flight can be *agile*.
3. **Optimal goal assignment for swarms** (Kushleyev et al., 2013) — deciding which robot takes
   which formation slot by minimizing total squared travel. This is the *cooperation*.

### Why small robots are agile

The talk's central physics argument deserves to be internalized now, because it explains a
design choice baked into `quadsim/params.py`: the simulated vehicle is a 33-gram
Crazyflie-class machine, not a camera drone. Scale a quadrotor's linear dimension by $r$. Mass
goes as volume, $m \sim r^3$. Moment of inertia is mass times length squared, $J \sim r^5$.
Rotor thrust scales roughly with the vehicle's cube, and torque is thrust times arm length, so
$\tau \sim r^4$. Angular acceleration is therefore

$$\dot{\omega} = \frac{\tau}{J} \sim \frac{r^4}{r^5} = \frac{1}{r}.$$

Halve the robot and it can reorient roughly *twice* as fast. Agility is not a software feature;
it is a consequence of being small. Every gram matters — which is why Module 6 will care
intensely about a 6-gram printed frame.

### How this course works

The method, agreed in `docs/LEARNING.md`, is **read, run, break, explain**. Each chapter you
read a little theory, run real code, deliberately sabotage something to see the failure mode,
and then explain what you saw in your own words. The "break" step is not a gimmick: seeing a
simulation flip over because of one sign error teaches more, and costs less, than the same
mistake on a real maiden flight. Homework goes into your Word copy of each chapter — every
question has an answer box. Never move to the next chapter while the current one feels like
magic.

### Why SPEC.md exists

One file deserves special explanation. `SPEC.md` is the project's *binding interface contract*:
it fixes names, function signatures, units, frames, and sign conventions before any module was
written, so that independently authored modules plug together. Its own words:

> Every module MUST follow this spec exactly: same names, signatures, units, frames, conventions.
> Multiple authors write modules concurrently against this contract.

You have met this idea before: it is an ICD, a bus specification, a datasheet. In robotics the
stakes are concrete — if `controller.py` thinks the quaternion is body→world and `dynamics.py`
thinks it is world→body, nothing crashes, no exception is raised, the vehicle simply flies into
the floor. Conventions are load-bearing. That is also why this course takes its notation table
(below) *from* `SPEC.md` rather than from any textbook: the course must match the code.

## Guided code walkthrough: a tour of the repository

Open `README.md` first; its layout section is the map:

```
quadsim/            core library
  maths.py          SO(3)/quaternion utilities (hat, vee, quat ops, rotation log)
  params.py         Crazyflie-2-like physical parameters
  dynamics.py       rigid-body dynamics, motor mixing/saturation, RK4 step
  trajectory.py     minimum-snap piecewise polynomials, flat outputs, yaw modes
  controller.py     geometric SE(3) tracking controller
  sim.py            simulation loop and History logging/metrics
  swarm.py          formation keyframes, optimal assignment, collision avoidance
  viz.py            3D trajectory plots, tracking panels, GIF animation
demos/              four runnable demos (write into out/)
tests/              pytest suite (maths, dynamics, trajectory, controller, swarm)
docs/HARDWARE.md    sim-to-real roadmap (Crazyflie levels 0-3)
SPEC.md             the binding interface contract the modules are written against
```

Notice the library has a strict layering (SPEC.md, "Global conventions"): `maths` and `params`
sit at the bottom; `dynamics` builds on them; `controller` builds on maths/params/trajectory;
`sim` orchestrates dynamics + controller + trajectory; `swarm` sits on everything; `viz` alone
may touch matplotlib. Each chapter of this course climbs one layer of that stack — the course
outline *is* the import graph.

Now trace how a picture gets made, because you will run demos constantly. Every demo has the
same skeleton; here is the heart of `demos/demo_hover.py`:

```python
params = QuadParams()
controller = SE3Controller(params)

target = np.array([0.0, 0.0, 1.0])
# Initial offset with ||offset|| = 0.4 m, split over all three axes.
offset = 0.4 * np.array([1.0, -1.0, 1.0]) / np.sqrt(3.0)
state0 = QuadState.hover(target + offset)
ref_fn = hover_ref(target)

T = 3.0 if args.fast else 6.0
hist = simulate(params, controller, ref_fn, state0, T, dt=0.002)
```

Physical parameters, a controller, a reference to follow, an initial state — then one call to
`simulate`. That function, in `quadsim/sim.py`, is the closed loop you drew in every controls
course, discretized at 500 Hz:

```python
for k in range(n):
    t = k * dt
    f_cmd, tau_cmd = controller.compute(state, ref_fn, t)
    f_act, tau_act = dyn.unmix(dyn.mix(f_cmd, tau_cmd))
    ...
    state = dyn.step(state, f_cmd, tau_cmd, dt)
```

Controller reads the state and the reference, outputs a wrench $(f, \tau)$; the dynamics
saturate it through the motor mixer (`unmix(mix(...))` — Chapter 2), integrate one step, and
loop. Everything is recorded into a `History` object — arrays of $t, p, v, q, \omega, f, \tau$
plus the reference — and the demo then hands that history to `quadsim/viz.py` functions like
`plot_tracking(hist, ...)` and `animate_quads([hist], ...)`, which write the PNGs and GIFs you
see in `out/`. That is the whole answer to "where do the GIFs come from": **demo → `simulate`
→ `History` → `viz` → `out/`**. Nothing in `out/` is hand-made; delete the folder and any demo
will regenerate its part.

Demos end by printing metrics as `name: value` lines and a verdict — `PASS`, or `WARN` with a
reason if a target from `SPEC.md` was missed. The targets (hover steady-state RMS < 0.005 m,
figure-eight tracking RMS < 0.10 m, swarm separation > 0.15 m…) are listed in the README's
demo table, so a demo is simultaneously a movie and a regression test.

## The notation contract (memorize this page)

All symbols below are lifted directly from `SPEC.md` and used by every later chapter. SI units
throughout: m, s, kg, N, rad.

| Symbol | Meaning | Convention (binding) |
|---|---|---|
| world frame | fixed inertial frame | $z$ **up**; gravity is $-g e_3$ with $g = 9.81$ |
| $e_3$ | third basis vector | `np.array([0.0, 0.0, 1.0])` |
| $p, v$ | position, velocity | expressed in the **world** frame, shape `(3,)` |
| $R \in SO(3)$ | attitude rotation matrix | maps **body → world**: $v_{world} = R\, v_{body}$ |
| $q = [w, x, y, z]$ | attitude quaternion | Hamilton convention, unit norm, scalar **first**, body→world |
| $b_3 = R e_3$ | body z-axis | the **thrust axis** of the vehicle |
| $f \ge 0$ | total thrust (scalar, N) | produces world-frame force $f \cdot b_3$ |
| $\tau$ | body torque, `(3,)`, N·m | expressed in the **body** frame |
| $\omega$ (code: `w`) | angular velocity, `(3,)`, rad/s | expressed in the **body** frame |
| $m, J, L$ | mass, inertia matrix, arm length | `QuadParams`: 0.033 kg, diag inertia, 0.046 m |

Three of these rows cause essentially all real-world quadrotor bugs, so stare at them now:
$R$ is body→world (many texts use the transpose); $q$ stores the scalar part *first* (scipy
stores it last); and $\omega$ lives in the *body* frame (the gyro's frame — a real IMU is
bolted to the body, which is exactly why the convention exists).

## Experiments

Run everything from the project root, `/Users/vishalbharti/Downloads/micro-agile-robot-2026`,
with the project interpreter `.venv/bin/python`.

**Experiment 1 — prove the toolchain works.**

```bash
cd /Users/vishalbharti/Downloads/micro-agile-robot-2026
.venv/bin/python -m pytest tests/ -q
```

Observe: `27 passed` in about 10–25 s depending on your machine. This is your testbench; run it after *any* change you make
in this course. If it passes, the physics, math, controller, trajectories, and swarm all still
meet spec.

**Experiment 2 — first flight.**

```bash
.venv/bin/python demos/demo_hover.py --fast
```

Observe the three metric lines (`settle_time_s`, `steady_rms_m`, `final_error_m`), the two
`saved:` paths, and `PASS`. Open `out/hover_tracking.png`: the top panel shows $x, y, z$
converging to the dashed references — a textbook second-order step response, which is no
coincidence (Chapter 3). Then run the other three demos with `--fast` to see the whole show:
`demo_minsnap.py`, `demo_figure8.py`, `demo_swarm.py`.

**Experiment 3 — sanity-check a number by hand.**

```bash
.venv/bin/python -c "
from quadsim.params import QuadParams
p = QuadParams()
print('weight_N:', p.weight)
print('f_max_N:', p.f_max)
print('per_motor_hover_N:', p.weight / 4)
"
```

Observe: weight $mg = 0.033 \times 9.81 \approx 0.324$ N, maximum total thrust 0.64 N. Their
ratio, $0.64/0.324 \approx 1.98$, is the thrust-to-weight the README rounds to "~2", and each
motor idles near 0.081 N at hover — about 51% of its 0.16 N limit. That headroom is the
agility budget.

**Experiment 4 — break it: the wrong interpreter.** Deliberately run a demo with your *system*
Python instead of the venv:

```bash
python3 demos/demo_hover.py --fast
```

Observe: on this machine it dies with an `ImportError` from inside matplotlib (an
Anaconda/numpy version clash); on other machines you may get `ModuleNotFoundError: quadsim`.
Either way, nothing about the *project* is broken — only the toolchain choice. This is the
number-one failure you will ever hit here, and the fix is always the same: use
`.venv/bin/python`, where `pip install -e ".[dev]"` installed the package editable against
pinned-compatible libraries. File a mental note of what the error looks like so you recognize
it instantly.

**Experiment 5 — break it: move the hover target.** Edit `demos/demo_hover.py` line 55 and
change the target height from `1.0` to `2.5`:

```python
target = np.array([0.0, 0.0, 2.5])
```

Re-run `.venv/bin/python demos/demo_hover.py --fast` and look at `out/hover_3d.png`. The robot
still starts 0.4 m from the target (the offset is *relative*) and settles in essentially the
same time — the controller does not care where in space the setpoint is, only about the error.
Now change it back and re-run to confirm you restored it. Getting comfortable with
edit → run → inspect → revert *is* the skill of this module.

## Homework

Write your answers in the Word copy of this chapter.

**Q1.** In your own words (no peeking), reproduce the scaling argument: why does halving a
quadrotor's size roughly double its angular acceleration? State which quantity scales as
$r^3$, $r^4$, and $r^5$.

> **Your answer:**



**Q2.** From Experiment 3: at hover, each motor produces about 0.081 N. What happens to the
vehicle's ability to *translate* aggressively as battery weight is added, given
`f_motor_max = 0.16` N is fixed? Express your answer using thrust-to-weight ratio.

> **Your answer:**



**Q3.** `SPEC.md` says $R$ maps body→world. Suppose one module silently used the transpose
(world→body) when computing the thrust direction $b_3 = R e_3$ for a tilted vehicle. Would any
Python error occur? What would you observe in `out/hover_3d.png` instead? Why is this class of
bug worse than a crash?

> **Your answer:**



**Q4.** Trace the artifact chain for `demos/demo_figure8.py` by reading the file: which
function call produces `out/figure8.gif`, which produces `out/figure8_tracking.png`, and what
single object (name the class) carries the data from the simulation into both?

> **Your answer:**



**Q5.** Scipy's `scipy.spatial.transform.Rotation` stores quaternions as $[x, y, z, w]$
(scalar **last**), while this project uses $[w, x, y, z]$ (scalar first). Write the one-line
index shuffle that converts a quadsim quaternion into scipy's order, and explain what a
"successfully wrong" conversion would do to a rotation of, say, 90° about $z$.

> **Your answer:**



**Q6.** The control loop in `quadsim/sim.py` runs at $dt = 0.002$ s. Using your DSP instincts:
name one thing that could go wrong if we raised $dt$ to 0.05 s (20 Hz) while the vehicle flies
the aggressive figure-eight. (You will test this for real in Chapter 3.)

> **Your answer:**



## Hints

1. Start from $\dot\omega = \tau / J$ and scale numerator and denominator separately.
2. Thrust-to-weight is `p.f_max / p.weight`; recompute it with $m' = m + \Delta m$.
3. At hover, $R \approx I$, and $I^\top = I$ — so when would the two conventions first disagree?
4. `grep -n "animate\|plot_\|simulate" demos/demo_figure8.py` shows every relevant call; the
   carrier class is defined in `quadsim/sim.py`.
5. Numpy fancy indexing: `q[[1, 2, 3, 0]]`. For the 90° case, ask which component the wrong
   order treats as the scalar $\cos(\theta/2)$.
6. Think sampling a fast-changing reference, and phase lag around a feedback loop — the
   figure-eight demands attitude changes with significant energy above a few hertz.

## Further reading

- **V. Kumar, "Robots that fly … and cooperate," TED 2012**
  (https://www.youtube.com/watch?v=4ErEBkj_3PY) — 16 minutes; watch it *tonight*. It is the
  design brief for this entire repository, and you now know where each demo appears in it.
- **`README.md` "Theory primer" section** — a four-paragraph preview of Chapters 2–5; reading
  it now plants the vocabulary before the math arrives.
- **`docs/HARDWARE.md`** — skim Level 0 and Level 1 only, to see where this course is headed
  once the simulator holds no more mysteries: these very trajectories on a real 33 g Crazyflie.
- **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors," ICRA
  2011** — do not *read* it yet; just look at the figures. By Chapter 4 you will implement-read
  the whole thing.
