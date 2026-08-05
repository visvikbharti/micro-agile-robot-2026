# Learning Path — from ECE to agile aerial robotics

A curriculum for an Electronics & Communication engineer building this project. Every module is
anchored to code in this repository: you read a little theory, run something, deliberately break
it, and explain what you saw. That loop — *read, run, break, explain* — is the whole method.

## What ECE already gives you

You are not starting from zero. Most of this field is ECE wearing a flight suit:

| You already know (ECE) | Where it shows up here |
|---|---|
| Complex exponentials, phasors | Quaternions are the same idea one dimension up: a rotation algebra where multiplication composes rotations (`quadsim/maths.py`) |
| State-space models, `dx/dt = Ax + Bu` | The quadrotor is a nonlinear state-space system with 13 states and 4 inputs (`quadsim/dynamics.py`) |
| Transfer functions, poles, damping | Controller gains `kp, kv` set exactly the natural frequency and damping you know from second-order systems (`quadsim/controller.py`) |
| Least squares, filter design | Minimum-snap trajectory generation is a constrained least-squares problem (`quadsim/trajectory.py`) |
| Sampling, discrete time | The sim runs control at 500 Hz over 2 ms steps — aliasing and rate limits are real concerns |
| Kalman filters, sensor fusion | How a real Crazyflie knows its attitude from a noisy IMU (Level 1 hardware) |
| PWM, H-bridges, motor drivers | Brushed motor control on the real vehicle — home turf |
| RF, 2.4 GHz radios | The Crazyflie radio link and swarm communication — home turf |

The genuinely new material: rigid-body mechanics, rotation math beyond 2D, control on curved
spaces, and mechanical design/3D printing. That is what the modules below build up.

## How we work

Each module is one or two guided sessions. Bring the questions; I bring the explanations and the
experiments. A session looks like: ~20 minutes of concept, ~40 minutes of hands-on in the code,
and one "explain it back" question at the end. Never move on while a module feels like magic.

---

## Module 0 — Tooling (half a session)
**Goal:** comfortable with the venv, pytest, and the demo scripts.
- **Run:** `.venv/bin/python -m pytest tests/ -q`, then every demo in `demos/`.
- **Do:** change the hover target height in `demos/demo_hover.py` and re-run.
- **Check:** where do the GIFs come from? Trace the call chain demo → `simulate` → `viz`.

## Module 1 — Rotations: quaternions and SO(3)
**Goal:** stop fearing quaternions. They are unit complex numbers' big sibling.
- **Read:** `quadsim/maths.py` (150 lines, the whole rotation toolkit).
- **Concept:** why Euler angles fail (gimbal lock), why `q = [w, x, y, z]` composes rotations by
  multiplication exactly like `e^(jθ)` composes phase, what `hat`/`vee` do.
- **Do:** in a Python shell, rotate a vector two ways — `quat_to_rot(q) @ v` and by composing two
  half-rotations — and confirm they agree.
- **Break:** skip the `quat_normalize` in `dynamics.step` and watch the sim slowly explode.
- **Check:** why does the attitude error `e_R` live in 3 numbers when a rotation matrix has 9?

## Module 2 — Rigid-body dynamics
**Goal:** own the equations of motion; know what each term does physically.
- **Read:** `quadsim/dynamics.py`; the four equations in `SPEC.md` §dynamics.
- **Concept:** Newton–Euler; why torque is force × lever arm (the mixer matrix rows); why
  `ω × Jω` exists (gyroscopic effect); RK4 vs Euler integration.
- **Do:** double the mass in `params.py`, re-run `demo_minsnap.py`, watch tracking degrade.
  Then halve it. This is Kumar's "small is agile" argument made tangible.
- **Break:** flip one sign in the mixer matrix `A` and watch the vehicle instantly flip over —
  the #1 real-world maiden-flight failure, previewed safely in sim.
- **Check:** at hover, what is each motor's thrust in newtons? Verify against `m·g/4`.

## Module 3 — Control: from PID intuition to SE(3)
**Goal:** read `controller.py` line by line and know why every term is there.
- **Concept path:** P/PD control (you know this) → gains as `ω_n²` and `2ζω_n` of a second-order
  system → cascaded loops (position loop commands attitude, attitude loop commands torque) →
  why the attitude loop must be ~8× faster → geometric control: doing PD directly on the
  rotation group so aggressive tilts don't break the math.
- **Do:** halve `kR` — watch wobble. Double `kp` with `kv` fixed — watch overshoot and ringing,
  exactly like an under-damped RLC step response. Plot with `plot_tracking`.
- **Break:** set the feedforward terms (`w_ff`, `a_ff`) to zero and re-run `demo_figure8.py`;
  measure how much tracking error grows. Feedforward is why the robot leans *into* turns early.
- **Check:** why does thrust get projected onto the current body z-axis (`F_des · Re3`) instead
  of just using `‖F_des‖`?

## Module 4 — Trajectories: minimum snap and differential flatness
**Goal:** understand why the motion looks graceful and how the optimization works.
- **Read:** `quadsim/trajectory.py`; Mellinger & Kumar 2011 (the paper is readable!).
- **Concept:** differential flatness — 4 outputs determine all 13 states; snap is the 4th
  derivative and maps to motor commands, so minimizing it minimizes actuator violence;
  polynomial splines + equality constraints solved as one linear (KKT) system — this is
  constrained least squares, a filter-design cousin.
- **Do:** design your own waypoint course through the gates in `demo_minsnap.py`. Raise
  `avg_speed` until the demo prints WARN — find this vehicle's limit empirically.
- **Break:** lower the polynomial continuity requirement mentally: why would minimum-*velocity*
  paths (straight lines between waypoints) be terrible for a quadrotor?
- **Check:** why 7th-order polynomials specifically? Count the constraints per segment.

## Module 5 — Swarms
**Goal:** decentralized-flavored coordination: formations, assignment, avoidance.
- **Read:** `quadsim/swarm.py`; skim Kushleyev et al. 2013.
- **Concept:** virtual leader + offsets; the Hungarian algorithm for "which robot takes which
  slot" (an allocation/matching problem — think channel assignment); smoothstep blending;
  short-range repulsion as a safety layer.
- **Do:** design your own formation (your initials?) as a keyframe in `demo_swarm.py`.
- **Break:** shrink the ring radius until robots enter each other's 0.30 m avoidance bubble and
  watch the repulsion term activate (the current demo never triggers it — margin by design).
- **Check:** why is assignment computed on *squared* distances?

## Module 6 — Mechanical design and CAD (new territory)
**Goal:** read and modify the frame CAD; understand printed-part design.
- **Read:** `hardware/frame.scad` (parameters at top), `docs/DESIGN.md`, `out/frame_drawing.png`.
- **Concept:** parametric CAD; design-for-printing (overhangs, layer adhesion, press-fits);
  stiffness vs mass; why the frame is ~6 g of a 33 g budget.
- **Do:** install OpenSCAD (free), open `frame.scad`, change a parameter, export an STL.
- **Later (agreed):** rebuild the frame in Fusion 360 (traditional CAD) using `frame.scad` as
  the master dimension source — worth doing when we want STEP files or nicer renders.
- **Check:** trace one dimension (arm length 46 mm) from `params.py` → SCAD → drawing.

## Module 7 — The real flight stack (ECE home turf)
**Goal:** map every sim module to its real-world counterpart before spending money.
- **Read:** `docs/HARDWARE.md`, Crazyflie firmware docs (`controller_mellinger.c` is our
  `controller.py` in C).
- **Concept:** IMU sensor fusion (complementary/Kalman — your Kalman knowledge pays off here),
  brushed motor PWM drive, 1S battery management, the 2.4 GHz link, and why state estimation
  is the hard part reality adds (the sim gets truth for free).
- **Do:** decide the Level 1 purchase (see `docs/HARDWARE.md`), fly manually, then fly one of
  *our* minimum-snap trajectories via cflib.
- **Check:** list the three biggest sim-to-real gaps and how each will show up in flight.

---

## References (read in this order)
1. Mellinger & Kumar, *Minimum snap trajectory generation and control for quadrotors*, ICRA 2011.
2. Lee, Leok, McClamroch, *Geometric tracking control of a quadrotor UAV on SE(3)*, CDC 2010.
3. Kushleyev, Mellinger, Powers, Kumar, *Towards a swarm of agile micro quadrotors*, 2013.
4. Turpin, Michael, Kumar, *Trajectory design and control for aggressive formation flight*, 2012.
