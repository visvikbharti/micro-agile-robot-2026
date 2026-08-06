# ESTIMATION.md — State estimation and sensing design

**Platform (binding decision, 2026-08-06):** Crazyflie 2.1+ with Flow deck v2 and
Crazyradio — the Level-1 package of `docs/HARDWARE.md`. This document is the design
that `docs/DESIGN.md` §9 admits the DIY track cannot have: with only an IMU,
"position/velocity are not observable at all." The DIY printed-frame whoop stays a
manual, mechanical-learning track; every autonomous flight in the FT-x.y test plan
flies on the Crazyflie, and this document says how that vehicle knows where it is.

Companion reading: `docs/course/ch07-real-flight.md` (the theory and the three
experiments this document leans on), `flight/README.md` Part C (the operational
procedures), `docs/HARDWARE.md` Levels 1–2 (what the money buys).

---

## 1. The problem — the simulator's biggest lie

Open `quadsim/sim.py`, `simulate()`:

```python
f_cmd, tau_cmd = controller.compute(state, ref_fn, t)
```

`state` is the integrator's own output — exact position, exact velocity, exact
attitude, delivered instantly, every 2 ms. Ch07 calls this line "the biggest lie in
the simulator," and it is: every other sim-to-real delta in `docs/DESIGN.md` §9
(drag, motor lag, battery sag) is a *disturbance*, and feedback exists to reject
disturbances. But the controller can only fight what it can see. State estimation is
not a disturbance to reject — it is the eyes of the loop, and the sim skips it
entirely.

What exactly must be estimated? `quadsim/controller.py`, `compute()`, reads four
quantities and nothing else:

| State | Symbol | Used for (controller line) | Sensitivity to estimation error |
| --- | --- | --- | --- |
| Position | `state.p` | `e_p = state.p - ref.pos` → `F_des` | Tolerant of noise; drift moves the whole hover |
| Velocity | `state.v` | `e_v = state.v - ref.vel` → `F_des` | **Bias is poison**: `e_p = -(k_v/k_p)·b` at equilibrium |
| Attitude | `state.R` | `e_R = ½ vee(R_cmdᵀR − RᵀR_cmd)` | Sets the thrust direction; errors tilt gravity itself |
| Body rates | `state.w` | `e_w`, gyroscopic and feedforward terms | Least tolerant of *latency* (fastest loop) |

Rates and latency budget, with numbers from our own gains
(`SE3Controller.__init__`: `kR = [1000, 1000, 100]`):

- The attitude loop crosses over near ω_n = √1000 ≈ **32 rad/s**. Ch07 Experiment 2
  degrades only the controller's *view* of the state (physics stays perfect) and
  finds: unmeasurable through 16 ms of estimate delay, residual ringing at 20 ms, a
  394 mm limit cycle at 30 ms, total departure (~100 m) at 40 ms. The fine-to-gone
  cliff spans ~20 ms.
- Consequence: estimation and attitude control must run **onboard at the stabilizer
  rate, 500 Hz** — the same rate as our sim's `dt = 0.002` (copied from the
  Crazyflie firmware deliberately). The radio link (tens of ms, non-deterministic)
  carries trajectories and logs, never the inner loop (`flight/README.md` §C.6).
- The position loop is far slower (`kp = m·16` → ω_n ≈ 4 rad/s from
  √(k_p/m) = √16), so position/velocity measurements may arrive slower and noisier
  than attitude ones. This asymmetry is exactly what the sensor suite below
  exploits.

And ch07 Experiment 1 ranks *which* errors hurt, on our exact controller and gains:

| Estimate corruption (8 s hover) | Result | Lesson |
| --- | --- | --- |
| White noise, σ_p = 1 cm, σ_v = 5 cm/s | ~2 mm RMS | Vehicle inertia low-passes zero-mean noise — noise floor barely matters |
| Constant 0.1 m/s velocity bias | parked −50.0 mm off target | `e_p = −(k_v/k_p)·b = −0.5 × 0.1` m; bias → offset, exactly |
| 1 mm/step position random walk | wanders, never settles | drift is the failure mode that never averages out |

**Design requirement, in one sentence:** the estimator must deliver `R` and `w` with
millisecond-scale latency at 500 Hz, `v` with low *bias* (noise is cheap), and `p`
with bounded *drift* — and we must know the bound.

## 2. Sensor suite — Crazyflie 2.1+ with Flow deck v2

| Sensor | Where | What it measures | Frame | Rate | Noise character |
| --- | --- | --- | --- | --- | --- |
| BMI088 gyroscope | Crazyflie 2.1+ mainboard IMU | Angular velocity ω (3 axes) — precisely our `state.w` | Body | ≥ stabilizer rate (500 Hz); exact ODR: consult the Bitcraze/Bosch datasheet | Low white noise + slowly varying **bias** (~0.01–0.02 rad/s class after calibration); integrating bias gives a b·t attitude ramp (ch07 Exp 3) |
| BMI088 accelerometer | Same package | Specific force (3 axes): gravity-in-body when unaccelerated → drift-free roll/pitch (never yaw); thrust+drag otherwise | Body | Same as gyro; consult datasheet | White-noisy; also picks up **prop vibration in-band** — a bent prop is estimator damage (`docs/HARDWARE.md` safety) |
| PMW3901 optical flow | Flow deck v2, camera facing the **floor** | Optical flow of ground texture (pixel displacement per frame) → after scaling by height and de-rotating by gyro, **body-frame horizontal velocity** | Body (x–y) | ~100 Hz class; consult the Bitcraze deck datasheet | Zero-mean when texture is good; degrades to outages/garbage on poor texture — errors integrate into position drift |
| VL53L1x ToF ranger | Flow deck v2, laser pointing down | Range to floor along the deck axis → **height above ground**, usable to roughly 4 m (deck spec — verify against the Bitcraze datasheet) | Down-axis | Tens of Hz; consult datasheet | mm-to-cm class per reading; IR — direct sunlight is a documented interferer; measures *slant* range under tilt (firmware compensates via attitude) |

Two geometric facts that make this suite a *system* rather than four parts —
both pure geometry, stated here as derivations:

1. **Flow needs height to mean anything.** A camera sees angular motion of the
   floor: ω_optical ≈ v_horizontal / h (rad/s), minus the vehicle's own rotation
   (which the gyro provides for de-rotation). Pixel counts become m/s only after
   multiplying by the ToF height. The flow camera and the ranger are one sensor in
   two packages — lose either and horizontal velocity is gone.
2. **Tilt corrupts the ranger predictably.** At tilt θ the beam reads slant range
   r = h/cos θ; the raw overestimate is 1/cos θ − 1, i.e. **+15.5 % at 30°**. The
   firmware corrects this with the estimated attitude, but at aggressive tilt the
   beam (and the flow camera's view) leaves the floor patch entirely — see §4.

What is **not** on the vehicle: no magnetometer usable for heading in practice
(indoor fields are junk), no GPS (indoors, and a nano flies under India Drone Rules
2021 anyway), no external cameras at Level 1. Absolute yaw and absolute position are
therefore **unobservable** — the estimator's world origin is wherever the vehicle
powered on, with axes set by its boot heading (`flight/README.md` §C.5: start every
trajectory at (0, 0, takeoff height) for exactly this reason).

## 3. The estimator — Bitcraze's onboard EKF

The pedagogical bridge is ch07's complementary filter: gyro integration is clean at
high frequency but drifts at DC; the accelerometer is drift-free at DC but noisy and
lying under acceleration; a crossover with fusion constant τ splits the spectrum,
and the two paths sum to exactly 1. Experiment 3 builds it and finds the U-shaped
optimum (0.28° RMS at τ = 0.2 s for the synthetic IMU) — and its "break it" case
shows a 10× boot-calibration bias collapsing the sweet spot. Then the magic words:
*a Kalman filter is a complementary filter that computes τ for you, optimally, every
step, from the noise covariances — and it can put the bias in the state vector and
estimate it away instead of trading against it.*

The Crazyflie firmware runs exactly that, extended (E) because the process model is
our nonlinear Chapter-2 dynamics: the EKF in `kalman_core` of
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware), in the lineage
of Mueller, Hamer & D'Andrea (ICRA 2015 — ch07 further reading #2).

**Structure** (verify details in `kalman_core.c` before quoting further):

- **State vector**: world-frame position (3), **body-frame** velocity (3), and a
  3-parameter attitude-error state alongside the reference quaternion — 9 error
  states. Gyro bias handling and exact conventions: read the source, don't guess.
- **Prediction** at IMU rate: gyro and accelerometer enter as *inputs* to the
  process model (Chapter 2's kinematics/dynamics), not as measurements. This is why
  attitude and rates arrive at 500 Hz with essentially zero latency — they live in
  the prediction path, inside the delay budget of §1.
- **Updates** as measurements arrive: the flow measurement model couples pixel rate
  to (body velocity)/(height) with gyro de-rotation — one scalar equation per axis
  per frame; the ToF measurement updates height through the tilt-compensated range
  model. Each update corrects velocity and attitude *and*, through correlations,
  position.

**Why the errors behave the way they do:**

| Quantity | Directly measured? | Error behavior |
| --- | --- | --- |
| Attitude (roll/pitch) | Yes — accelerometer gives drift-free "down" | Good indefinitely; gyro bias estimated away |
| Body rates | Yes — gyro, at full rate | Best signal on the vehicle |
| Height | Yes — ToF | Good (cm-class) inside the ranger envelope |
| Horizontal velocity | Yes — flow (when texture cooperates) | Zero-mean noise mostly; bias only from calibration/scale errors |
| Horizontal position | **No.** Obtained by integrating velocity | **Random walk**: every flow-velocity error integrates once; drift grows with time and with texture poverty |
| Yaw | **No** absolute reference | Slow gyro-integrated drift; harmless for hover, rotates the whole world frame over long flights |

That table *is* the flight behavior predicted in `flight/README.md` §C.6: "wander,
not jitter." Ch07 Experiment 1 is the same table in simulation — white noise ≈ free,
velocity bias = fixed offset, position random walk = the slow cm-level wander you
will actually watch during FT-1.x hovers.

## 4. Operating envelope — where estimation works, and how it fails

The comfort zone (from `docs/HARDWARE.md` Level 1 and `flight/README.md` §C.3):
**textured, matte floor; decent even lighting; below ~2 m altitude; gentle speeds
and modest tilt.** A rug is ideal; glossy tile is not. Deck mounted underneath,
camera and ranger unobstructed, boot flat and still (gyro calibration — ch07 Exp 3's
break-it case is what moving it costs).

Failure modes, DESIGN.md-§9 style — each with its in-flight fingerprint and the
mitigation, because a failure you can name from the log is a failure you can fix:

| Failure mode | Physics | In-flight symptom (the fingerprint) | Mitigation |
| --- | --- | --- | --- |
| Glossy / featureless floor | PMW3901 tracks texture; no texture (or specular shine) → no correlation → flow outage; EKF coasts on integrated accel | Position hold degrades to accelerating drift; vehicle "ice-skates" sideways, slow at first then diverging | Fly over a rug/textured mat; tape newspaper or a printed pattern to the floor; abort early — drift compounds |
| Low light | Flow SNR collapses; fewer valid frames | Same ice-skating drift, often intermittent (good patches, bad patches) | Bright, even room lighting before every FT session; log flow quality if available |
| Direct sunlight on the floor | Sunlight's IR floods the VL53L1x ToF return | Height estimate glitches → altitude twitching or a sudden climb/drop; flow scaling (v = ω·h) corrupts too | Fly indoors away from sun patches; curtains; the ToF is the reason "indoor platform" is literal |
| Altitude too high | ToF ranger runs out (~4 m class — consult datasheet) and flow angular resolution drops (ω_optical = v/h shrinks) | Height hold goes soft above ~2 m; horizontal hold gets sluggish/noisy | Keep FT flights ≤ 1.5 m; the envelope in §5 assumes it |
| Too fast, too low | Optical flow rate ω ≈ v/h exceeds the tracker's range: at h = 0.5 m, 1 m/s is already 2 rad/s of ground motion | Velocity underestimated during fast segments → tracking cuts corners, then position drift jumps after the segment | Respect §5 speed caps; fly faster segments a bit higher (v/h is the invariant, not v) |
| Aggressive tilt | Camera and ToF beam leave the floor patch; slant-range error (1/cos θ − 1) grows nonlinearly | During hard maneuvers: height spike in the log, position estimate jumps when level flight resumes | Cap tilt in FT-2.x; FT-3.x flies as reduced-speed characterization at Level 1 — maneuvers beyond the flow envelope wait for Lighthouse (§6) and carry no FT ID |
| Vibration (bent prop) | In-band accelerometer noise corrupts the prediction step | Fat attitude noise, degraded everything; worse after any crash | Replace bent props before blaming the tune (`docs/HARDWARE.md` safety) — estimator damage precedes frame damage |
| Moved during boot calibration | Gyro bias mis-zeroed; ch07 Exp 3 break-it: optimum collapses, error ×8 at the old tuning | Immediate steady drift/toppling tendency from takeoff | Power on flat and still, hands off for the first seconds; re-power if bumped |
| Long flights | Position random walk and yaw drift accumulate (minutes scale) | Return-to-start misses by tens of cm; "home" isn't home | Keep sorties short; land and re-origin between runs; FT-4.x measures the actual drift rate |

## 5. What this means for our trajectories — and FT pass criteria

**Hard caps** (already enforced in code): `flight/cf_trajectory.py` `validate_rows`
rejects any uploaded trajectory whose analytic peaks exceed **MAX_SPEED = 3.0 m/s**
(a generic sanity cap) or **MAX_ACCEL = 8.0 m/s²** (the Crazyflie thrust-margin cap,
gravity excluded) — matching the code's own comments; checked in every
`fly_minsnap.py` dry run.

**Estimator-driven derating below the caps.** The flow envelope, not the thrust
budget, is the binding constraint at Level 1:

- Fly minimum-snap courses at `avg_speed` ≈ 0.5–0.7 m/s (`fly_minsnap.py` default
  0.5; `flight/README.md` example 0.7). Min-snap peaks run ≈ 2× the average
  (`cf_trajectory.py` comment), so 0.7 m/s average ≈ 1.4 m/s peak — half the hard
  cap, and at 0.5–0.8 m height an optical rate of ~1.8–2.8 rad/s, inside the
  comfortable zone. The sim demos' 2 m/s through the 4×3×2 m volume is explicitly
  Lighthouse territory (`flight/README.md` §C.5).
- Keep courses at 0.5–1.0 m height over the textured mat; remember v/h is the flow
  invariant — if a segment must be faster, fly it higher, not lower.
- `yaw_mode="fixed"` for exported trajectories (poly4d constraint, `flight/README.md`
  §C.5) — which also sidesteps yaw-drift coupling during the course.
- Start every course at (0, 0, takeoff height): the EKF origin is the power-on point.

**Expected estimator behavior per phase.** Authority note, first:
[`docs/FLIGHT_TEST_PLAN.md`](FLIGHT_TEST_PLAN.md) owns every binding pass/fail number
— the bands below are *engineering expectations and rationale* derived from the
sources above (the "centimeter-level wander" hover expectation of `flight/README.md`
§C.4, Exp-1's bias arithmetic, and the drift model of §3), not gates; where a band is
tighter than the plan's bar it is labeled a stretch expectation. First flights
calibrate them; tighten or loosen with data, and record the revision in the flight
log.

| Phase | Test intent | Estimator-related expectation (binding bars live in the plan) |
| --- | --- | --- |
| FT-0.x | Bench & ground checks | Deck detected; boot flat/still; on the ground with props off, logged estimate reads z ≈ deck height, v ≈ 0, attitude level (FT-0.5) and *stationary* (no ramp over 60 s — a ramp is a failed bias calibration); emergency stop drilled on the ground (FT-0.6, `flight/estop.py`) |
| FT-1.x | First hovers | Binding bars per the plan: FT-1.1 excursion ≤ 0.10 m and altitude ±0.10 m; FT-1.2 total drift ≤ 0.20 m over 30 s. *Stretch expectation* for a healthy deck on a good floor: altitude within ±5 cm, wander inside a ~10 cm-radius circle. **No monotonic runaway** (runaway = flow outage, land and fix the floor, not the gains); a *constant* offset from the target is bias, not tuning — re-place and re-boot before touching gains |
| FT-2.x | Trajectory following | Square preset at avg 0.5–1.0 m/s: position-tracking error RMS ≲ 10–15 cm — matching the plan's FT-2 bars (RMS ≤ 0.10–0.15 m) — no corner-cutting jumps; return-to-start within ~20 cm after a ≤ 60 s course |
| FT-3.x | Aggressive maneuvers | Flown at Level 1 as *characterization* (FT-3.1 reduced-speed figure-8; FT-3.2 speed sweep to 2.0 m/s peak): missing the bars with a stable flight plus an anomaly report still satisfies the phase gate. Full-speed sim-envelope flight waits for Lighthouse (§6) and carries no FT ID — an estimator gate, not a controller gate |
| FT-4.x | Endurance & robustness | Measure, don't assume: repeated 2–3 min hovers to log actual drift rate (cm/min) and the battery-sag altitude fingerprint; drift rate becomes the revised FT-1.x/FT-2.x budget |

One diagnostic discipline, worth stating twice: log setpoint-vs-estimate for
position/velocity/attitude (the same signals as `quadsim.sim.History` — `t, p, v, q,
w, f, tau, ref_p, ref_v`) so every plotting habit from `plot_tracking` transfers.
When an FT fails, first classify the error with the §1 table — noise, bias, or drift
— because each points at a different subsystem.

## 6. Upgrade path — Lighthouse, and making the sim stop lying

**Level 2 — Lighthouse** (`docs/HARDWARE.md`: ~+$450 approximate; deck + two SteamVR
Base Station 2.0 + mounts; ~5×5 m coverage). Two base stations sweep IR laser planes;
the deck computes position *onboard* to sub-centimeter accuracy — lab-grade,
Vicon-analog state estimation at hobby budget.

What changes: horizontal **position becomes directly measured** — the random-walk row
of §3's table disappears, absolute yaw becomes observable, and the floor-texture,
lighting, height, and tilt constraints of §4 stop binding (the receivers look
*sideways/up* at the base stations, not down at the floor). That unlocks the sim's
actual demo envelope: the 2 m/s obstacle course and figure-8 at full speed, tilt
beyond the Level-1 FT-3 characterization (deferred flights that carry no FT ID until
Lighthouse exists), repeatable tracking-error measurement against `plot_tracking`.

What stays: the same EKF (Lighthouse is one more measurement model into the same
filter), the same 500 Hz onboard loop and latency budget, the same
`controller_mellinger.c`, the same poly4d upload path, and the same `validate_rows`
caps (the 3.0 m/s generic sanity cap and the 8.0 m/s² thrust-margin cap are vehicle
limits, not flow limits). The upgrade
swaps measurement models under an unchanged stack — which is precisely why the
Level-1→2 gate in ch07 is "move only when the flow estimator *measurably* limits
you." FT-2.x logs are the evidence for that gate.

**The roadmap experiment — teach `quadsim` to lie less.** Ch07's experiments already
contain the seed: `Estimator` and `DelayedEstimate` wrappers that degrade what the
controller sees while physics stays exact. The next step is promoting them from
throwaway experiment code into a reusable `quadsim` estimator-model layer:

1. **Noise/bias/drift injection** (Exp 1's `Estimator`, parameterized): white σ_p/σ_v,
   constant velocity bias, position random walk — with defaults *fitted from FT-1.x
   and FT-4.x logs* rather than guessed. The FT-4.x measured drift rate (cm/min) is
   the first real number to feed back.
2. **Latency injection** (Exp 2's `DelayedEstimate`): validate that our flight-plan
   speeds keep margins even with realistic estimate delays.
3. **A flow-envelope model**: zero out (or inflate the noise of) the simulated flow
   velocity when commanded v/h or tilt exceeds §4's envelope — so a trajectory that
   would blind the real flow deck *fails in simulation first*, the same discipline
   `swarm.py` uses for collision checks before any formation flight.

That closes the twin loop the same way `docs/DESIGN.md` §9 closes it for mass and
thrust: measure on hardware, push the numbers back into the sim, and keep the
simulator honest about the one thing it currently gets for free — knowing where it is.
