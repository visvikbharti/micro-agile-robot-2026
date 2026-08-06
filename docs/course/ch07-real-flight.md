# Chapter 7 — The real flight stack: where ECE comes home

## After this chapter you can …

1. Map every `quadsim` module to its Crazyflie counterpart, and name the one job reality adds that the sim never had to do.
2. Explain IMU attitude estimation: why neither gyro nor accelerometer works alone, how a complementary filter splits the spectrum between them, and what a Kalman filter adds.
3. Predict — with numbers from our gains — what estimator bias and control latency do to a hover, and verify both predictions in simulation.
4. Describe the Crazyflie architecture (STM32 firmware, radio SoC, cflib, Crazyradio) and locate our `controller.py` and `trajectory.py` inside it.
5. Make a defensible Level 0→3 purchase decision and recite the safety rules you will actually follow.

## The ECE bridge

This is the chapter where the bridge carries the most traffic, most of it in your direction.

| New concept | What you already know |
|---|---|
| Complementary filter | A two-way audio crossover: low-pass one source, high-pass the other, transfer functions summing to 1. The woofer is the accelerometer, the tweeter the gyro |
| Gyro bias drift | DC offset in an amplifier chain: integrate it and it becomes a ramp. Boot-time calibration is auto-zeroing |
| Kalman filter for attitude/position | You know Kalman. Only the application is new: process model = Chapter 2's dynamics, measurements = IMU/flow/Lighthouse |
| Estimator bias → position offset | Op-amp input offset voltage: a constant input error appears at the output scaled by a gain ratio |
| Control latency instability | Phase margin: a pure delay $e^{-s T_d}$ adds no gain but $\omega T_d$ of phase lag — the loop rings, then oscillates |
| Motor spin-up dynamics | An RC time constant: commanded thrust is a step, delivered thrust a first-order lag (~20 ms) |
| Battery sag | Supply droop under load: available thrust falls as the pack drains |
| PWM brushed-motor drive | H-bridges and duty cycles — home turf |
| 2.4 GHz Crazyradio link | ISM band, GFSK, packet protocols, link budgets — home turf |
| Firmware on an STM32F405 | Embedded C on a Cortex-M4 under an RTOS: a hard-real-time 500 Hz task, not a Python `for` loop |

## Core theory

### What the simulator gets for free

Open `quadsim/sim.py`; one line of the loop:

```python
# quadsim/sim.py — simulate()
f_cmd, tau_cmd = controller.compute(state, ref_fn, t)
```

The controller is handed `state` — the *actual* state the integrator produces: exact position, velocity, attitude, instantly, at 500 Hz. No real vehicle has ever known its own state. This line is the biggest lie in the simulator, and the honest name for the missing machinery is **state estimation**. Everything else reality adds — motor lag, battery sag, wind — is a disturbance, and good feedback loops shrug off disturbances. But a controller can only fight what it can *see*, and reality makes seeing hard.

### Attitude from an IMU: two bad sensors make one good one

A MEMS IMU gives two measurements. The **gyroscope** reads angular velocity $\omega$ — precisely `state.w` — and integrating it gives attitude via Chapter 1's $\dot{q} = \tfrac12 q \otimes [0, \omega]$. But every gyro adds a small bias $b$, and integrating a constant gives a ramp: attitude error grows as $b \cdot t$, so even a good bias of 0.02 rad/s ruins the estimate within a minute. The **accelerometer** measures specific force: when the vehicle isn't accelerating, that is gravity in the body frame — a drift-free reading of *down*, hence of roll and pitch (never yaw: rotating about gravity doesn't move gravity). But it is noisy, and it lies whenever the vehicle accelerates.

So one sensor is clean at high frequency and drifts at DC; the other is trustworthy at DC and garbage at high frequency. You solved this with audio drivers: build a crossover with fusion time constant $\tau$,

$$\hat\theta = \underbrace{\frac{\tau s}{\tau s + 1}}_{\text{high-pass}} \cdot \frac{\omega_{\text{gyro}}}{s} \; + \; \underbrace{\frac{1}{\tau s + 1}}_{\text{low-pass}} \cdot \theta_{\text{accel}}.$$

The two filters sum to exactly 1 — no frequency is lost, hence *complementary*. Discretized at step $dt$ with $\alpha = \tau/(\tau + dt)$, it collapses to one line:

$$\hat\theta_k = \alpha\,(\hat\theta_{k-1} + \omega_k\, dt) + (1-\alpha)\,\theta_{\text{accel},k}.$$

Choosing $\tau$ is a genuine trade: large $\tau$ trusts the gyro longer, filtering accel noise hard but letting bias leak through as a steady error of $b\tau$; small $\tau$ kills the bias but passes jitter. There is a U-shaped optimum; Experiment 3 finds it.

Now say the magic words: *a Kalman filter is a complementary filter that computes $\tau$ for you, optimally, every step, from the noise covariances* — and, better, it can append $b$ to the state vector and estimate the bias itself, removing the trade instead of balancing it. The Crazyflie firmware runs exactly this — an extended Kalman filter (nonlinear process, hence the E) fusing gyro, accelerometer, and whichever deck is installed into the full $[p, v, q]$ estimate. When `docs/HARDWARE.md` promises Flow-deck "position hold with no external positioning system," the enabling technology is that EKF.

### What else reality adds

**Motor dynamics.** The sim delivers any within-limits thrust instantly. A real coreless brushed motor is an R-L circuit spinning a mass: command a thrust step and it arrives as a first-order lag, time constant 15–25 ms — pure phase lag inside our fastest loop, the attitude loop.

**Latency, quantitatively.** Sensor read-out, estimator, control, PWM update: each real pipeline stage adds delay $T_d$, contributing phase lag $\omega T_d$ with no gain change — the classic phase-margin thief. Our attitude loop has $k_R = 1000$ per axis on inertia-normalized error, so $\omega_n = \sqrt{1000} \approx 32$ rad/s; a 30 ms delay costs $0.03 \times 32 \approx 0.95$ rad $\approx 54°$ at crossover — most of a healthy margin. Prediction: ringing past 20 ms, departure around 30–40 ms. Experiment 2 checks this.

**Battery sag and the missing integrator.** A 1S LiPo starts at 4.2 V and sags toward 3.0 V under load; thrust per unit PWM falls with it. Our `SE3Controller` constructor has `kp, kv, kR, kw` — no `ki` anywhere. In sim that is fine: the model is exact, so feedforward plus PD leaves no steady error. In reality, any constant unmodeled force — a draining pack, an off-center battery, a bent prop — leaves a *steady offset* that only integral action removes. `controller_mellinger.c` is our controller plus exactly that repair: gains `ki_xy`, `ki_z`, and integral attitude terms accumulated in an `i_error` state — the difference to hunt for in Homework 3.

### The Crazyflie architecture: your sim, compiled

The Crazyflie 2.1+ is two processors and a protocol. An **STM32F405** (Cortex-M4, 168 MHz) runs the flight firmware under FreeRTOS; the stabilizer loop ticks at 500 Hz — the *same rate* as our `dt = 0.002` sim loop, because we copied it. An **nRF51** handles the 2.4 GHz radio and power management; on your desk a **Crazyradio 2.0** dongle speaks the same GFSK physical layer, carrying CRTP packets — a packetized ISM-band link you could characterize in your sleep.

The firmware is our repo in C, module for module — `docs/HARDWARE.md` tabulates the correspondence:

| Sim module | Real-world counterpart |
| --- | --- |
| `quadsim/params.py` | Vehicle mass/inertia/thrust limits in the Crazyflie firmware; refine via system ID |
| `quadsim/dynamics.py` | The physical vehicle (reality is the integrator) |
| `quadsim/maths.py` | Firmware math libraries (quaternion/rotation utilities) |
| `quadsim/trajectory.py` | `cflib` high-level commander; uploaded piecewise-polynomial trajectories (7th-order, same form) |
| `quadsim/controller.py` | Firmware `controller_mellinger.c` (select via parameter) |
| `quadsim/sim.py` | Flight tests; onboard logging via `cflib`/cfclient log blocks |
| `quadsim/swarm.py` | Crazyswarm2 (ROS 2): multi-vehicle upload, broadcast start, synchronized execution |
| `quadsim/viz.py` | cfclient plotter, log-file analysis, rosbag + your own matplotlib scripts |

Read the middle rows. `quadsim/dynamics.py` maps to *the physical vehicle*: reality is the integrator, and it has no numerical error. Chapter 3's controller is selected in firmware via the `stabilizer.controller` parameter. The trajectory row should make you grin — the firmware's uploadable format is piecewise 7th-order polynomials in $(x, y, z, \text{yaw})$, and in `quadsim/trajectory.py`:

```python
# quadsim/trajectory.py
_N_COEFF = 8            # 7th-order polynomial -> 8 coefficients per segment per axis
```

Same representation, coefficient for coefficient. Flying your Chapter 4 minimum-snap course on hardware is a formatting exercise via `cflib`'s high-level commander (`takeoff`, `go_to`, `upload_trajectory`, `start_trajectory`) — the role `traj.eval` plays for `simulate`.

### Spending money: the Levels, and when

`docs/HARDWARE.md` lays out four levels: **Level 0**, this repo (≈ \$0); **Level 1**, one Crazyflie + Flow deck + Crazyradio (~\$360–400), scripted indoor flight with no infrastructure; **Level 2**, add Lighthouse positioning (~+\$450), lab-grade state estimation for aggressive trajectories; **Level 3**, the Crazyswarm2 multi-vehicle path (~+\$350 per vehicle). All prices approximate — check the store first.

The decision framework is a gate, not a shopping list. Move 0→1 only when Level 0 is *exhausted*: you can read a tracking plot and say which gain to touch, you have broken every demo on purpose, and you can state what the Flow deck cannot give you (drifts over minutes, wants a textured lit floor, gentle speeds, altitude below ~2 m). Move 1→2 only when the flow estimator measurably limits you; 2→3 only with netting up and the formation verified collision-free in sim. At every gate ask: *is hardware the bottleneck, or am I buying motivation?* The alternatives (~\$40–120 tiny whoop for stick skills, an ESP-drone toe-in) exist for honest answers of "motivation."

### Safety culture

The TED-talk lab flies inside netting, and that lab flies better than you do. From `docs/HARDWARE.md`: props off for bench work; safety glasses when debugging close-up; replace bent props — vibration wrecks state estimation before it wrecks the frame, and now you know why: vibration is in-band accelerometer noise. LiPos: charge attended on a non-flammable surface, retire puffed or crashed packs, store at ~50%, never fly below ~3.0 V/cell. Net the flight volume before any aggressive or multi-vehicle flight; indoors you set the rules, outdoors aviation law applies. Wire up `cflib`'s emergency stop before your first scripted flight and *test it on the ground*. First flights: low, slow, one vehicle, cushion underneath.

## Guided code walkthrough

This walkthrough is short because its point is an absence. In `quadsim/controller.py`, `compute` opens with:

```python
# quadsim/controller.py — compute()
R = state.R
w = state.w

ref = ref_fn(t)
e_p = state.p - ref.pos
e_v = state.v - ref.vel
```

Those reads — `p`, `v`, `R`, `w` — are the estimator's job description on hardware. Nothing else changes when you go real: `controller_mellinger.c` computes the same `F_des`, $e_R$, and torque; it just receives its state from the EKF instead of the integrator, plus the `i_error` terms our exact-model sim never needed. And `quadsim/sim.py`'s `History` — `t, p, v, q, w, f, tau, ref_p, ref_v` — is precisely the signal set you will configure as `cflib` log blocks, so your Chapter 3 plotting habits transfer unchanged. The sim-to-real jump is not a new stack; it is the same stack with one hard new module (estimation) and three soft ones (lag, sag, delay).

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`. Each experiment wraps the real `SE3Controller` in a fake estimator — degrading what the controller *sees* while the physics stays perfect: reality's problems, added one at a time.

**Experiment 1 — what kind of error actually hurts?** Corrupt the state estimate during an 8 s hover three ways: white noise (a jittery estimator), a constant velocity bias (a miscalibrated flow sensor), and a position random walk (flow drift).

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.sim import simulate
from quadsim.trajectory import hover_ref

p0 = np.array([0.0, 0.0, 1.0])

class Estimator:
    """The controller sees reality through this."""
    def __init__(self, inner, sp=0.0, sv=0.0, vbias=None, walk=0.0, seed=0):
        self.inner, self.sp, self.sv, self.walk = inner, sp, sv, walk
        self.vbias = np.zeros(3) if vbias is None else np.asarray(vbias)
        self.drift = np.zeros(3)
        self.rng = np.random.default_rng(seed)
    def compute(self, state, ref_fn, t):
        self.drift += self.rng.normal(0, self.walk, 3)
        est = QuadState(p=state.p + self.drift + self.rng.normal(0, self.sp, 3),
                        v=state.v + self.vbias + self.rng.normal(0, self.sv, 3),
                        q=state.q, w=state.w)
        return self.inner.compute(est, ref_fn, t)

for name, kw in [("perfect estimate   ", dict()),
                 ("white noise only   ", dict(sp=0.01, sv=0.05)),
                 ("+0.1 m/s vel bias  ", dict(vbias=[0.1, 0, 0])),
                 ("1 mm/step pos drift", dict(walk=0.001))]:
    est = Estimator(SE3Controller(QuadParams()), **kw)
    hist = simulate(QuadParams(), est, hover_ref(p0), QuadState.hover(p0), 8.0)
    final = hist.p[-1] - p0
    print(f"{name} rms={hist.rms_pos_error(2.0)*1000:6.1f} mm   final x offset {final[0]*1000:+7.1f} mm")
EOF
```

Observe: 1 cm / 5 cm/s *white* noise costs only ~2 mm RMS — the vehicle's inertia low-passes zero-mean noise, the same reason a sigma-delta ADC tolerates huge quantization noise. But the 0.1 m/s velocity *bias* parks the vehicle exactly $-50.0$ mm off target — predictable: at equilibrium $k_p e_p = -k_v b$, so $e_p = -(k_v/k_p)\, b = -0.5 \times 0.1$ m. Input offset voltage, quadrotor edition. The drift case wanders without settling. Moral: estimators are judged by bias and drift, not noise floor — why bias-estimating Kalman filters earn their complexity.

**Experiment 2 — latency, and (break it) the phase-margin cliff.** Delay the controller's view of the state by $N$ steps and fly a 30 cm step:

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.sim import simulate
from quadsim.trajectory import hover_ref

p0 = np.array([0.0, 0.0, 1.0])
s0 = QuadState.hover(np.array([0.3, 0.0, 1.0]))   # 30 cm step response

class DelayedEstimate:
    def __init__(self, inner, n_steps):
        self.inner, self.buf, self.n = inner, [], n_steps
    def compute(self, state, ref_fn, t):
        self.buf.append(state)
        old = self.buf[0] if len(self.buf) <= self.n else self.buf[-1 - self.n]
        return self.inner.compute(old, ref_fn, t)

for n in [0, 5, 8, 10, 12, 15, 20]:
    ctrl = DelayedEstimate(SE3Controller(QuadParams()), n)
    hist = simulate(QuadParams(), ctrl, hover_ref(p0), s0, 6.0)
    e = hist.pos_error()
    print(f"delay {n*2:3d} ms   peak err {min(np.nanmax(e), 999):7.3f} m   final err {min(e[-1], 999)*1000:9.1f} mm")
EOF
```

Observe: nothing measurable through 16 ms, a 12 mm residual ring at 20 ms, 71 mm at 24 ms, a 394 mm limit-cycle at 30 ms — and the **break-it** case, 40 ms, departs to ~100 m: total loss, right where the 54°-of-phase estimate said it would be. The cliff from "fine" to "gone" spans barely 20 ms — which is why flight firmware is hard-real-time C on an RTOS, not Python over USB.

**Experiment 3 — build the complementary filter, then (break it) skip gyro calibration.** Sixty seconds of synthetic 500 Hz IMU, one axis: true roll is a slow wobble; the gyro has bias 0.02 rad/s plus noise; the accel-derived angle is unbiased but noisy ($\sigma = 0.05$ rad).

```sh
.venv/bin/python - <<'EOF'
import numpy as np
dt, T = 0.002, 60.0
t = np.arange(0.0, T, dt)
true_angle = 0.3 * np.sin(2 * np.pi * 0.5 * t)
true_rate = 0.3 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t)
rng = np.random.default_rng(1)
bias = 0.02                                   # rad/s — break it: set to 0.2
gyro = true_rate + bias + rng.normal(0, 0.02, t.size)
accel_angle = true_angle + rng.normal(0, 0.05, t.size)

gyro_est = np.cumsum(gyro) * dt
print(f"gyro only   rms {np.sqrt(np.mean((gyro_est - true_angle)**2))*57.3:6.2f} deg (drifts forever)")
print(f"accel only  rms {np.sqrt(np.mean((accel_angle - true_angle)**2))*57.3:6.2f} deg (white jitter)")
for alpha in [0.9, 0.99, 0.999, 0.9999]:
    comp = np.zeros_like(t)
    for k in range(1, t.size):
        comp[k] = alpha * (comp[k-1] + gyro[k] * dt) + (1 - alpha) * accel_angle[k]
    tau = alpha * dt / (1 - alpha)
    err = comp - true_angle
    print(f"alpha={alpha:6.4f} (tau {tau:6.2f} s)  rms {np.sqrt(np.mean(err**2))*57.3:5.2f} deg  "
          f"bias leak ~ {bias*tau*57.3:5.2f} deg")
EOF
```

Observe: gyro alone drifts to ~39° RMS; accel alone jitters at ~2.9°; the fusion bottoms out at **0.28°** at $\alpha = 0.99$ ($\tau = 0.2$ s), the U-shape visible on both sides — noise leaks in at small $\tau$, bias ($\approx b\tau$, check the printed column) at large $\tau$. **Break it**: set `bias = 0.2`, as if the vehicle was moved during boot calibration. The sweet spot collapses toward small $\tau$, the best error more than doubles, and the formerly optimal $\alpha = 0.99$ becomes 8× worse. This is why a Crazyflie must sit still for its first seconds after power-on — and why the EKF, which estimates the bias instead of splitting the difference, earns its arithmetic.

## Homework

1. The essay (Module 7's "check" in `docs/LEARNING.md`): name the three biggest sim-to-real gaps for flying our minimum-snap course on a Level-1 Crazyflie, and for each, say *what symptom you would see in the flight logs* — the fingerprint, not just the category.

> **Your answer:**
<br><br><br>

2. Shopping decision: you have \$400 and Level 0 is genuinely exhausted. Using `docs/HARDWARE.md`, choose between (a) the full Level-1 kit, (b) a tiny whoop + radio now, Level 1 later, and (c) an ESP-drone. State your choice, the strongest argument *against* it, and the observable milestone triggering your next purchase.

> **Your answer:**
<br><br><br>

3. Read one function of real firmware: open `controller_mellinger.c` in the crazyflie-firmware repository (`src/modules/src/controller/`), find the main controller function, and identify (i) our `F_des` line — the position/velocity/feedforward force sum — and (ii) two terms it has that ours lacks, naming which reality each one fights.

> **Your answer:**
<br><br><br>

4. Hands-on, predict then verify: Experiment 1's 0.1 m/s velocity bias produced a −50 mm offset because $e_p = -(k_v/k_p)b$. Predict the offset with `SE3Controller(QuadParams(), kp=0.033*np.array([32.,32.,32.]))` (doubled $k_p$, same $k_v$), then modify the experiment to check. What op-amp fact about loop gain and offsets does this illustrate?

> **Your answer:**
<br><br><br>

5. Show that the complementary filter's two paths sum to unity — $\frac{\tau s}{\tau s + 1}\cdot\frac{1}{s}\cdot s + \frac{1}{\tau s+1} = 1$ when both sensors read the truth — and say in one sentence why this "all-pass in disguise" property is the whole point.

> **Your answer:**
<br><br><br>

6. Experiment 2 delayed the *full* state. On real hardware the gyro path (IMU→controller, onboard) has far lower latency than the position path (external system over radio). Which of the four estimated quantities tolerates the least delay, and what does that imply about where the attitude loop must physically run?

> **Your answer:**
<br><br><br>

## Hints

1. One gap is this chapter's star; the other two are in "What else reality adds." A fingerprint reads like "altitude sags late in the flight."
2. No letter is wrong, only justifications; answer "what is the current bottleneck?" first.
3. Search the file for `i_error` and gain names starting with `ki`; our `F_des` is `quadsim/controller.py` line 106 — find its C twin.
4. The formula answers in one line; the op-amp fact is about what more loop gain does to input-referred offsets.
5. The gyro path is (angle × s) → integrator (1/s) → high-pass; multiply it out and add the low-pass.
6. Which loop's crossover (32 rad/s vs 4 rad/s) sets the delay budget, and what does "onboard" mean with two processors?

## Further reading

1. **Mahony, Hamel & Pflimlin, "Nonlinear complementary filters on the special orthogonal group," IEEE TAC 2008** — Experiment 3's filter done properly on SO(3) with Chapter 1's math; the algorithm inside many hobby flight controllers.
2. **Mueller, Hamer & D'Andrea, "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation," ICRA 2015** — the lineage of the EKF the Crazyflie flies; Kalman applied, with Chapter 2's dynamics as the process model.
3. **The crazyflie-firmware source** (github.com/bitcraze/crazyflie-firmware) — `controller_mellinger.c`, the EKF in `kalman_core`, the trajectory player: this repo in C, free to read before you spend a rupee.
4. **Bitcraze's "Getting started" and cflib guides** (bitcraze.io) — the exact Level-1 bring-up path, including the motor-test and radio-setup rituals the safety section assumes.

---

*[Course index](README.md) · Next: [Chapter 8 — Decentralized swarms](ch08-decentralized-swarms.md) →*
