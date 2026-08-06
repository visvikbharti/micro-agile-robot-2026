# Chapter 9 — Your own firmware: our controller on the real robot

## After this chapter you can …

1. Explain what buying a Crazyflie actually buys — motors, IMU, radio, and a solved hard-real-time substrate — and what this chapter replaces: the controller in the 500 Hz loop.
2. Name the crazyflie-firmware extension points (pluggable controllers, the app layer, out-of-tree builds) and say why you write *inside* the stabilizer loop rather than around it.
3. Map every function of `quadsim/controller.py` onto its counterpart concept in `controller_mellinger.c`, and state the two structural differences you must verify before trusting the map.
4. Write the core SE(3) math in firmware-discipline C — float-only, no malloc — and prove it against golden vectors exported from our Python controller, on your laptop, today.
5. Recite the four validation gates between "it compiles" and "it flies", and state exactly where the flight-test campaign re-enters: **FT-1.1**, for *any* custom firmware.

A word on expectations before we start. Everything in this chapter up to and including compiling the C port and passing its unit gate runs **today, with no hardware** — that is Experiments 1–3. Flashing and flying wait for the vehicle. And `flight/README.md` Part A already told you when this project becomes a good idea: *"Writing firmware becomes a great project the moment you can already fly and read a tracking log, because then you can tell your bug from physics."* This chapter is the roadmap for that moment — walked as far as a laptop allows, so the day the parcel arrives you flash code that has already survived three gates.

## The ECE bridge

| New concept | What you already know |
|---|---|
| Pluggable firmware controller | Implementing a fixed callback against a vendor HAL: the RTOS owns the schedule and the drivers, you own the math inside one function |
| 500 Hz stabilizer tick | Worst-case-execution-time budgeting for an ISR: 2 ms per tick, shared with the estimator — blow the budget and the loop slips, and ch07 Experiment 2 priced what slip costs |
| Float-only math | Single-precision DSP discipline: the STM32F405's FPU (Cortex-M4F) does `float` in hardware; a stray `double` literal drops you into software emulation |
| No malloc, static allocation | Embedded C you already practice: all state in `static` structs, stack bounded, nothing allocated after boot |
| Golden test vectors | HDL testbench vectors: same stimulus into two implementations, diff the outputs bit-for-bit-ish |
| Log replay validation | Hardware-in-the-loop with recorded stimulus: play a captured trace through the new block before it touches the plant |
| Props-off bench test | Bringing up a new board behind a current-limited supply: full power path exercised, no energy to do damage |
| Re-entry at FT-1.1 | Re-qualification after an engineering change: new implementation ⇒ the acceptance tests run again, from the bottom |
| Flashing over the radio | A bootloader/DFU cycle, with the stock image as your known-good ROM: one reflash from safety |

## Core theory

### Why buy a robot and then replace its brain

Chapter 7 made the case for *not* writing firmware on day one, and nothing here retracts it: the Bitcraze stack's drivers, EKF, radio protocol, and failsafe stay exactly as they are. What changes is one plug-in. Buying the Crazyflie was buying motors, an IMU, and a radio in a convenient, crash-tolerant arrangement — plus years of solved hard-real-time problems. The stock `controller_mellinger.c` is a fine brain, and per `docs/ARCHITECTURE.md` §2 it is what Level-1 flights use. But the whole point of this project was to *own* the algorithms, and ownership has a final exam: the day `quadsim/controller.py` — our controller, our gains, our attitude-error convention — commands the real motors.

### The firmware landscape

The flight firmware is open source: **github.com/bitcraze/crazyflie-firmware**, C on **FreeRTOS**, with the stabilizer loop ticking at **500 Hz** on the STM32F405 — the same rate as our `dt = 0.002` sim loop (`flight/README.md` Part A). Controllers are *pluggable*: the stock tree ships several under `src/modules/src/controller/` — `controller_pid.c` (the default), `controller_mellinger.c`, `controller_indi.c`, and in recent trees more — selected at runtime through the `stabilizer.controller` parameter (Mellinger is value 2 in current firmware, `flight/README.md` §C.5). Each one implements the same small interface: an init, a self-test, and a per-tick function that receives the setpoint, the sensor data, and the estimated state, and fills in the motor-facing control output. The exact signatures live in the firmware's controller interface header (`src/modules/interface/controller.h`) — read that header rather than trusting any book, this one included: the API has evolved and will again.

Two supported extension points matter to us, both documented on the Bitcraze firmware pages (see the docs' "Out-of-tree build" and "App layer" chapters in the crazyflie-firmware documentation):

- **Out-of-tree controllers**: your controller compiles in its own directory against the firmware tree and registers as the selectable controller, without patching Bitcraze's files. This is the intended home for our port — upstream merges stay trivial.
- **The app layer**: a sandboxed FreeRTOS task for higher-level logic (sequencing, logging, supervisors). Useful later; it is *not* where a 500 Hz controller belongs.

The discipline the substrate imposes is the ECE bridge's right-hand column: your per-tick function must finish comfortably inside 2 ms alongside the EKF, use single-precision floats (the M4F FPU does not accelerate `double` — write `1.0f`, use `sinf`/`sqrtf`), and allocate nothing: every accumulator and scratch matrix is a `static` or a stack array, sized at compile time. Our controller is a few hundred floating-point operations per tick (Homework 4 makes you count them), so the budget is generous — the discipline is about *guarantees*, not speed.

### The mapping table: `controller.py` ↔ `controller_mellinger.c`

Chapter 7's Homework 3 sent you into `controller_mellinger.c` hunting for `i_error` and `ki_*`; this table is that homework grown up. Left column: our code, with line numbers you can check because the file is in this repo. Right columns: the *concept* to find in the firmware — deliberately not line numbers, because that file changes; Experiment 4 has you fill the Verify column against today's source.

| Ours (`quadsim/controller.py`) | Concept | Find in `controller_mellinger.c` | Verify |
| --- | --- | --- | --- |
| `F_des = -kp*e_p - kv*e_v + m*g*e3 + m*ref.acc` (line 106) | Outer loop: desired world force | Position + velocity error terms, gravity, acceleration feedforward — **plus integral terms ours lacks** (`ki_*` gains, `i_error` accumulators; ch07 §"Battery sag") | What does each `i_error` integrate, and where is it clamped (anti-windup)? |
| `f = max(0, F_des @ (R @ e3))` (line 107) | Thrust = projection of desired force on the actual body z | The scalar thrust output. **Units differ**: ours is newtons; the firmware's output is scaled for the power-distribution/PWM stage | Find the scale factor and where saturation happens |
| `flat_to_rotation(F_des, ref.yaw)` (lines 21–47) | Desired attitude from a force vector + yaw | Construction of desired body axes from desired acceleration and yaw | How does it handle the degenerate heading case our lines 42–44 guard? |
| `e_R = 0.5*vee(R_cmd.T @ R - R.T @ R_cmd)` (line 120) | SO(3) attitude error | The attitude-error computation. **The convention may differ** — the Mellinger paper uses this skew-symmetric form, but the firmware implementation may build roll/pitch/yaw errors from body-axis cross products with its own sign conventions | Hand-check one tilt case (Experiment 3) against the firmware's formula before trusting signs |
| `e_w`, `w_ff`, `a_ff` by finite-differencing `ref_fn` (lines 110–121) | Rate error + attitude feedforward | Rate error against setpoint-supplied rates. **No finite differencing**: the firmware receives one setpoint sample per tick, not a callable `ref_fn(t)` | Which setpoint fields (rates, acceleration) does it actually consume? |
| `tau = J @ (-kR*e_R - kw*e_w) + w×Jw - J@(…)` (lines 122–126) | Torque, gains premultiplied by `J` so `kR`, `kw` read as ωn², 2ζωn (`docs/ARCHITECTURE.md` §2) | Attitude gains applied **without** our `J` premultiplication, tuned as raw numbers; gyroscopic/feedforward terms may be absent or folded elsewhere | Compare term by term; note which of our terms have no twin |
| `kp, kv, kR, kw` defaults (lines 72–75) | Pole placement, ζ = 1 everywhere | A different gain set with different names and units | **Do not copy numbers.** `docs/ARCHITECTURE.md` §2: the twin is structural; gains never transfer numerically |

The two differences worth reading twice: the firmware has **integral action** (real batteries sag, real frames trim off-center — ch07 explained why the exact-model sim never needed it), and its **attitude-error convention is not guaranteed to be ours** — a sign or transpose difference there is not a small bug, it is positive feedback on attitude. Experiment 3 shows what that looks like in numbers.

### The C port, sketched

Here is the heart of the port in firmware-discipline C — the same math as `controller.py` lines 104–127, `float`-only, no allocation. (`R` is row-major `float[9]`; the full compilable file, including the vector helpers and a test harness, is written by Experiment 2.)

```c
/* e_R = 0.5 * vee(R_cmd^T R - R^T R_cmd).  The argument is exactly
 * skew-symmetric, so vee reads the off-diagonal entries directly. */
static void attitude_error(const float r_cmd[9], const float r[9], float e_r[3]) {
    float m[9];
    mat3_at_b(r_cmd, r, m);              /* m = R_cmd^T R; R^T R_cmd = m^T */
    e_r[0] = 0.5f * (m[3 * 2 + 1] - m[3 * 1 + 2]);
    e_r[1] = 0.5f * (m[3 * 0 + 2] - m[3 * 2 + 0]);
    e_r[2] = 0.5f * (m[3 * 1 + 0] - m[3 * 0 + 1]);
}

/* Outer loop force, thrust projection, torque — controller.py's compute(),
 * hover/setpoint case (w_ff = a_ff = 0; see the walkthrough). */
for (int i = 0; i < 3; ++i)
    f_des[i] = -KP[i] * (p[i] - ref_pos[i]) - KV[i] * (v[i] - ref_vel[i])
             + MASS_KG * ref_acc[i] + (i == 2 ? MASS_KG * GRAV : 0.0f);
re3[0] = r[2]; re3[1] = r[5]; re3[2] = r[8];       /* R e3 = third column  */
f = dot3(f_des, re3);
if (f < 0.0f) f = 0.0f;                            /* thrust cannot pull   */
flat_to_rotation(f_des, ref_yaw, r_cmd);
attitude_error(r_cmd, r, e_r);
cross3(w, jw, wxjw);                               /* gyroscopic term w×Jw */
for (int i = 0; i < 3; ++i)                        /* e_w = w when w_ff = 0 */
    tau[i] = J_DIAG[i] * (-KR[i] * e_r[i] - KW[i] * w[i]) + wxjw[i];
```

Three notes, one per discipline. **Precision**: every constant carries an `f` suffix and every libm call is the `f` variant (`sinf`, `sqrtf`, `cosf`) — on the M4F, `sqrtf` is one hardware instruction while `sqrt` is a software subroutine. **Time**: this is on the order of a hundred floating-point operations plus a handful of square roots and two trig calls — microseconds against a 2 ms tick (Homework 4 puts numbers on it); the budget threat is never this math, it is accidentally blocking (a print, a log, a semaphore) inside the tick. **Memory**: gains and inertia are `static const` tables, scratch lives on the stack with compile-time sizes; the only mutable state a full port adds is integral accumulators — `static float`, clamped.

### The validation ladder — four gates between "compiles" and "flies"

This is the chapter's crown, and it is a ladder you already believe in from the flight-test plan: *a phase may not begin until every test of the previous phase has passed* (`docs/FLIGHT_TEST_PLAN.md` §1.1). We simply extend the ladder downward, below the bench, all the way to your laptop. Each rung is a gate: do not climb past a rung you have not passed.

**Gate 1 — unit level, on the laptop (today).** Compile the C math with your desktop compiler and run it against **golden vectors** exported from `quadsim/controller.py`: a file of (state, reference) → (f, τ) cases the Python controller has already answered. Pass bar: worst |C − Python| disagreement at float precision (~1e-7 on our magnitudes), across cases that include asymmetric attitudes — Experiment 3 shows why symmetric cases can hide a fatal sign bug. Experiments 1–2 are this gate, executed.

**Gate 2 — closed-loop replay, on the laptop (after first stock-firmware flights).** Feed *logged real-flight states* through both implementations and diff the commands. The `--log` CSVs from `flight/fly_hover.py` carry `t, x, y, z, vbat` (`flight/README.md` §C.4) — enough to start; a fuller replay wants velocity, attitude, and rates from cfclient log blocks (Homework 5 designs the block). This gate catches what unit vectors cannot: real signal magnitudes, real noise, real trajectories — the C port must produce the same commands as `controller.py` on data no one hand-picked.

**Gate 3 — props-off bench, on the vehicle (hardware day).** Flash the port; **props stay off**. Re-run the bench ladder under the new brain: `flight/preflight.py` (FT-0.1–FT-0.5) and the FT-0.6 emergency-stop drill — the firmware you just flashed is precisely the component the stop path runs through, so the drill is not a formality. Then hold the vehicle level in your hand (props off), select your controller via `stabilizer.controller`, and watch motor commands respond sanely to tilts: tilt left ⇒ left-side motors speed up. Keep the stock firmware image on disk — one radio-bootloader reflash from safety at all times.

**Gate 4 — re-enter the flight-test campaign at FT-1.1.** Flying **any** custom firmware — a full controller port or a one-line patch — re-enters the campaign at **FT-1.1** (first hover, 0.5 m / 5 s) and climbs the phases in order, per `docs/FLIGHT_TEST_PLAN.md`. A new brain is a new configuration, and the hard-gate rule exists exactly for configuration changes; your FT-2 and FT-3 passes under stock firmware say nothing about the code you flashed yesterday. G-DRY, G-PREFLIGHT, and G-BATTERY apply unchanged. No new FT IDs exist for custom firmware — it is the same campaign, re-flown, and the comparison of *your* FT-2.2 tracking numbers against the stock Mellinger's is the project's graduation plot.

## Guided code walkthrough

Read `compute()` in `quadsim/controller.py` one more time — this time as a **port checklist**, sorting each line into "ports directly" versus "must be restructured".

**Ports directly.** The state reads (`p, v, R, w` — in firmware these arrive from the EKF via the state argument, ch07's whole story), the `F_des` sum (line 106), the thrust projection and its `max(0, ·)` clamp (line 107), `flat_to_rotation` including its degenerate-heading guard (lines 36–44), `e_R` (line 120), and the torque assembly (lines 122–126). This is the code Experiment 2 compiles, and it is most of the file.

**Must be restructured.** Everything touching `ref_fn`. Our Python controller receives the reference as a *callable* and evaluates it at three times — `R_ff(t)`, `R_ff(t ± fd_dt)` (lines 110–118) — to finite-difference the attitude feedforward `w_ff`, `a_ff`. Firmware receives one setpoint *sample* per tick; there is no function to probe at `t ± 1e-3`. The honest options: (a) set `w_ff = a_ff = 0` — exactly right for hover and gentle setpoints, since for `hover_ref` the commanded rotation is constant and the feedforward is identically zero (that is why Gate 1's golden vectors are hover-referenced, and why our C `se3_feedback` is complete for that case); (b) for trajectories, consume the rates and accelerations the onboard trajectory player provides in the setpoint — check which fields the setpoint struct actually carries in the firmware headers before designing this; (c) keep a one-tick history onboard and finite-difference across ticks — workable, but you are differentiating a 500 Hz sampled signal and ch07 taught you what that does to noise. Start with (a); flying our full min-snap feedforward is a Gate-2-then-Gate-4 project of its own.

One absence to notice, again: no integral terms — line 106 has four terms and none is an accumulator. The sim's exact model never needed one; the real vehicle does (ch07 Experiment 1's bias offset, battery sag). A flyable port grows `ki` terms exactly where `controller_mellinger.c` has them, with clamps. Homework 2 makes you write those three lines.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`. No hardware, no radio, no cflib — a Python interpreter and any C compiler (`cc` on macOS).

**Experiment 1 — export golden vectors from the Python controller.** Six hand-chosen cases: perfect hover, a position offset, a descent, a pure tilt, tilt plus body rates, and everything at once with a yaw setpoint. Each row of the CSV is the full controller input (state, reference) and the Python controller's answer (f, τ) at full precision:

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.params import QuadParams
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.maths import quat_normalize
from quadsim.trajectory import hover_ref

ctrl = SE3Controller(QuadParams())
p_ref = np.array([0.0, 0.0, 1.0])

def tilt(deg, axis):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    h = np.deg2rad(deg) / 2.0
    return quat_normalize(np.r_[np.cos(h), np.sin(h) * axis])

qI = np.array([1.0, 0.0, 0.0, 0.0])
z3 = np.zeros(3)
cases = [
    ("hover, zero error ", [0, 0, 1.0],       z3,             qI,                 z3,              0.0),
    ("10 cm x offset    ", [0.1, 0, 1.0],     z3,             qI,                 z3,              0.0),
    ("sinking 0.2 m/s   ", [0, 0, 1.0],       [0, 0, -0.2],   qI,                 z3,              0.0),
    ("10 deg roll tilt  ", [0, 0, 1.0],       z3,             tilt(10, [1, 0, 0]), z3,             0.0),
    ("tilt + body rate  ", [0, 0, 1.0],       z3,             tilt(10, [1, 0, 0]), [1.0, -0.5, 0.2], 0.0),
    ("everything at once", [0.1, -0.05, 0.9], [0.2, 0, 0.1],  tilt(15, [1, 1, 0]), [1.0, -2.0, 0.5], 0.3),
]

rows = []
for name, p, v, q, w, yaw in cases:
    st = QuadState(p=np.asarray(p, float), v=np.asarray(v, float), q=q, w=np.asarray(w, float))
    ref_fn = hover_ref(p_ref, yaw=yaw)
    f, tau = ctrl.compute(st, ref_fn, t=0.0)
    ref = ref_fn(0.0)
    rows.append(np.r_[st.p, st.v, st.R.ravel(), st.w, ref.pos, ref.vel, ref.acc, ref.yaw, f, tau])
    print(f"{name}  f = {f:8.6f} N   tau = [{tau[0]:+12.5e} {tau[1]:+12.5e} {tau[2]:+12.5e}] N m")

header = "p(3), v(3), R(9 row-major), w(3), ref_pos(3), ref_vel(3), ref_acc(3), ref_yaw, f, tau(3)"
np.savetxt("out/golden_se3.csv", np.array(rows), fmt="%.17g", delimiter=",", header=header)
print(f"\nwrote out/golden_se3.csv  ({len(rows)} rows x {rows[0].size} columns)")
EOF
```

Observe, and check against theory before moving on: hover thrust is `f = mg = 0.033 × 9.81 = 0.323730` N with zero torque; the sinking case adds exactly `kv·0.2 = 0.033·8·0.2 = 0.0528` N (f = 0.376530); the 10° roll tilt gives `f = mg·cos 10° = 0.318812` N and `tau_x = J_x·(−k_R·sin 10°) = 1.43e-5 × (−1000 × 0.173648) = −2.48317e-3` N·m. Every printed number should be explainable this way — golden vectors you cannot explain are not golden, they are merely recorded.

**Experiment 2 — compile the C port and pass Gate 1.** This writes the complete C file — the chapter's core functions plus vector helpers and a harness `main` that replays the CSV — compiles it with your desktop compiler, and diffs C against Python:

```sh
mkdir -p out && cat > out/golden_check.c <<'EOF'
/* golden_check.c — the SE(3) core math in firmware-style C, checked on the
 * laptop against golden vectors exported from quadsim/controller.py.
 * Discipline: float only (the STM32F405 FPU is single-precision), no malloc,
 * no libc beyond math.h (stdio/stdlib are harness-only, not for firmware). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- vehicle constants: quadsim/params.py + controller.py defaults ---- */
#define MASS_KG   0.033f
#define GRAV      9.81f
static const float J_DIAG[3] = {1.43e-5f, 1.43e-5f, 2.89e-5f};
static const float KP[3] = {0.033f * 16.0f, 0.033f * 16.0f, 0.033f * 16.0f};
static const float KV[3] = {0.033f * 8.0f, 0.033f * 8.0f, 0.033f * 8.0f};
static const float KR[3] = {1000.0f, 1000.0f, 100.0f};
static const float KW[3] = {63.0f, 63.0f, 20.0f};

/* ---- tiny static-allocation linear algebra (R is row-major float[9]) ---- */
static void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}
static float dot3(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static float norm3(const float a[3]) { return sqrtf(dot3(a, a)); }

/* M = A^T * B, all 3x3 row-major */
static void mat3_at_b(const float a[9], const float b[9], float m[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m[3 * i + j] = a[i] * b[j] + a[3 + i] * b[3 + j] + a[6 + i] * b[6 + j];
}

/* ---- the three functions that ARE the controller ---- */

/* R_cmd = flat_to_rotation(F_des, yaw): b3 along F, b1 near the yaw heading */
static void flat_to_rotation(const float f_des[3], float yaw, float r_cmd[9]) {
    float b1c[3] = {cosf(yaw), sinf(yaw), 0.0f};
    float b3[3] = {0.0f, 0.0f, 1.0f};
    float b1[3], b2[3], c[3];
    float n = norm3(f_des);
    if (n >= 1e-6f) {
        b3[0] = f_des[0] / n; b3[1] = f_des[1] / n; b3[2] = f_des[2] / n;
    }
    cross3(b3, b1c, c);
    if (norm3(c) < 1e-6f) {              /* singular heading: rotate b1c 90 deg */
        b1c[0] = cosf(yaw + (float)M_PI / 2.0f);
        b1c[1] = sinf(yaw + (float)M_PI / 2.0f);
        cross3(b3, b1c, c);
    }
    n = norm3(c);
    b2[0] = c[0] / n; b2[1] = c[1] / n; b2[2] = c[2] / n;
    cross3(b2, b3, b1);
    for (int i = 0; i < 3; ++i) {        /* columns of R_cmd are b1, b2, b3 */
        r_cmd[3 * i + 0] = b1[i];
        r_cmd[3 * i + 1] = b2[i];
        r_cmd[3 * i + 2] = b3[i];
    }
}

/* e_R = 0.5 * vee(R_cmd^T R - R^T R_cmd); the argument is exactly skew,
 * so vee reads the three off-diagonal entries directly */
static void attitude_error(const float r_cmd[9], const float r[9], float e_r[3]) {
    float m[9];
    mat3_at_b(r_cmd, r, m);              /* m = R_cmd^T R; R^T R_cmd = m^T   */
    e_r[0] = 0.5f * (m[3 * 2 + 1] - m[3 * 1 + 2]);
    e_r[1] = 0.5f * (m[3 * 0 + 2] - m[3 * 2 + 0]);
    e_r[2] = 0.5f * (m[3 * 1 + 0] - m[3 * 0 + 1]);
}

/* One controller step, hover/setpoint case (w_ff = a_ff = 0 — see chapter):
 * inputs are the estimator's state and one reference sample; outputs f (N,
 * clamped >= 0) and body torque tau (N m). */
static void se3_feedback(const float p[3], const float v[3], const float r[9],
                         const float w[3], const float ref_pos[3],
                         const float ref_vel[3], const float ref_acc[3],
                         float ref_yaw, float *f_out, float tau_out[3]) {
    float f_des[3], r_cmd[9], e_r[3], jw[3], wxjw[3], re3[3];
    for (int i = 0; i < 3; ++i)
        f_des[i] = -KP[i] * (p[i] - ref_pos[i]) - KV[i] * (v[i] - ref_vel[i])
                 + MASS_KG * ref_acc[i] + (i == 2 ? MASS_KG * GRAV : 0.0f);
    re3[0] = r[2]; re3[1] = r[5]; re3[2] = r[8];   /* R e3 = third column */
    *f_out = dot3(f_des, re3);
    if (*f_out < 0.0f) *f_out = 0.0f;
    flat_to_rotation(f_des, ref_yaw, r_cmd);
    attitude_error(r_cmd, r, e_r);
    for (int i = 0; i < 3; ++i) jw[i] = J_DIAG[i] * w[i];
    cross3(w, jw, wxjw);                           /* gyroscopic term w x Jw */
    for (int i = 0; i < 3; ++i)                    /* e_w = w when w_ff = 0  */
        tau_out[i] = J_DIAG[i] * (-KR[i] * e_r[i] - KW[i] * w[i]) + wxjw[i];
}

/* ---- harness: replay out/golden_se3.csv, report worst disagreement ---- */
int main(void) {
    FILE *fp = fopen("out/golden_se3.csv", "r");
    if (!fp) { fprintf(stderr, "run the export snippet first\n"); return 1; }
    char line[4096];
    float worst = 0.0f;
    int row = 0;
    while (fgets(line, sizeof line, fp)) {
        if (line[0] == '#') continue;
        float x[32];
        char *tok = strtok(line, ",");
        for (int i = 0; i < 32 && tok; ++i, tok = strtok(NULL, ","))
            x[i] = strtof(tok, NULL);
        float f, tau[3];
        se3_feedback(&x[0], &x[3], &x[6], &x[15], &x[18], &x[21], &x[24],
                     x[27], &f, tau);
        float err = fabsf(f - x[28]);
        for (int i = 0; i < 3; ++i) {
            float e = fabsf(tau[i] - x[29 + i]);
            if (e > err) err = e;
        }
        printf("row %d   f = %10.6f N (python %10.6f)   max |C - python| = %.3g\n",
               ++row, f, x[28], err);
        if (err > worst) worst = err;
    }
    fclose(fp);
    printf("worst disagreement over all rows: %.3g  (%s)\n", worst,
           worst < 1e-5f ? "PASS < 1e-5" : "FAIL");
    return worst < 1e-5f ? 0 : 1;
}
EOF
cc -std=c99 -Wall -Wextra -O2 -o out/golden_check out/golden_check.c -lm && ./out/golden_check
```

Observe: all six rows agree to **2.98e-08** or better — that is one unit-in-the-last-place of a `float` near 0.32, i.e. the C port in single precision reproduces the double-precision Python controller to the limit `float` allows. That number is Gate 1's signature: not "close", but *explainably* at machine precision. (Note what the harness is: stimulus from one implementation, response diffed against another — an HDL testbench, wearing C.)

**Experiment 3 — hand-derive one golden vector, then (break it) flip the attitude-error transpose.** First the derivation, because a gate you cannot derive is a gate you cannot debug. Take row 4: `R` is a pure roll by θ = 10°, `R_cmd = I` up to the tilt-induced correction — but at zero position/velocity error `F_des = mge3`, so `R_cmd` is exactly `I` and `e_R = 0.5·vee(R − Rᵀ)`. For a single-axis rotation, `R − Rᵀ` has exactly one off-diagonal pair, of magnitude `2 sin θ`, and vee's averaging halves it twice: `e_R = [sin θ, 0, 0] = [0.173648, 0, 0]`. Torque: `tau_x = J_x(−k_R·sin θ − k_w·0) = 1.43e-5 × (−1000 × 0.173648) = −2.48317e-3` N·m — the golden CSV's row 4 to the last digit. Also read the *sign*: positive roll produces negative roll torque. That sign is the entire stability story.

Now break it. In `out/golden_check.c`, swap the transpose order — change `mat3_at_b(r_cmd, r, m)` to `mat3_at_b(r, r_cmd, m)` (computing `RᵀR_cmd` first, i.e. `e_R` negated) — recompile, rerun:

```
row 1   f =   0.323730 N (python   0.323730)   max |C - python| = 2.98e-08
row 2   f =   0.323730 N (python   0.323730)   max |C - python| = 0.0046
row 3   f =   0.376530 N (python   0.376530)   max |C - python| = 2.98e-08
row 4   f =   0.318812 N (python   0.318812)   max |C - python| = 0.00497
row 5   f =   0.318812 N (python   0.318812)   max |C - python| = 0.00497
row 6   f =   0.314042 N (python   0.314042)   max |C - python| = 0.0115
worst disagreement over all rows: 0.0115  (FAIL)
```

Two lessons, both load-bearing. First: rows 1 and 3 **still pass** — zero attitude error hides the bug completely, which is why a golden set with only symmetric, level cases would wave this catastrophe through to the props. Second: read the failing magnitudes — each is exactly *twice* the `J·kR·e_R` contribution to that row's torque (row 4: 2 × 2.48317e-3 = 4.97e-3), because flipping the transpose order negates `e_R` and only that term: the attitude-feedback torque points the *wrong way*. On the bench (Gate 3) this reads as "tilt left, left motors slow *down*"; in the air it is positive feedback and instant departure. This one-character bug is precisely the class of error the mapping table's "attitude-error convention" row warns about when comparing against `controller_mellinger.c` — now you have seen its fingerprint at every gate below flight.

**Experiment 4 — fieldwork: fill the mapping table's Verify column.** Open `controller_mellinger.c` on GitHub (`src/modules/src/controller/` in bitcraze/crazyflie-firmware) and, for each row of the mapping table, write one sentence in the Verify column of your workbook copy: what you found, at which commit hash (record it — the file changes). Search the file for `i_error` and `ki` (the integral machinery, ch07 Homework 3), for the thrust computation and its scaling, and for how the attitude error is built — cross products of body axes, or the skew-symmetric form, and with which sign convention relative to Experiment 3's derivation. Budget an hour; this table, verified against a pinned commit, *is* the design document for the real port.

## Homework

1. The gates essay: Gate 1 passed at 2.98e-08, yet Gate 3 (props-off bench) can still fail. Name two distinct classes of bug that survive perfect golden vectors and are caught on the bench, and for each say what you would *observe* — the fingerprint, not the category.

> **Your answer:**
<br><br><br>

2. Integral action: write the C for adding a `ki` term to our `F_des` — the `static` accumulator, its per-tick update, and its clamp — and explain in two sentences why the clamp (anti-windup) is non-optional on hardware when it was not even worth writing in the sim. (Where does the accumulator go during the seconds the vehicle sits on the pad with the controller running?)

> **Your answer:**
<br><br><br>

3. Conventions: Experiment 3 showed `e_R = [sin θ, 0, 0]` for a single-axis tilt. Some implementations use the axis-angle error `rot_log(RᵀR_cmd)` instead (our `quadsim/maths.py` has `rot_log`), which gives `[−θ, 0, 0]` for the same case (mind the sign convention). Show the two agree to first order for small θ, compute both at θ = 90° and θ = 179°, and say which behavior you would rather have commanding torque during an aggressive recovery — then say why *matching the golden vectors* still beats "better" during a port.

> **Your answer:**
<br><br><br>

4. The time budget: count the floating-point operations in Experiment 2's `se3_feedback` path (including `flat_to_rotation` and `attitude_error` — count a multiply-add as two), plus the `sqrtf` and trig calls. Assuming order-of-magnitude 1 FLOP/cycle on the M4F FPU at 168 MHz and tens of cycles per libm call, estimate the tick cost in microseconds and compare to the 2 ms budget. What fraction did the math cost, and what does that tell you about where 500 Hz firmware actually spends its time?

> **Your answer:**
<br><br><br>

5. Design Gate 2: the `--log` CSV from `flight/fly_hover.py` carries `t, x, y, z, vbat` — list every additional signal a full closed-loop replay of `controller.py` needs (read `compute()`'s state and reference usage), and name the cfclient log variables or log-block layout you would configure to capture them, within the constraint that log blocks have limited size and rate (`flight/README.md` §C.4 and §C.6).

> **Your answer:**
<br><br><br>

6. Failure drill: you pass Gates 1–3, fly FT-1.1 with your port, and the vehicle oscillates fast in roll immediately after takeoff and is landed by Ctrl-C. Using ch07's latency and gain arguments plus this chapter's gates, list your first three hypotheses *in the order you would test them*, each with a test that needs no flight (bench, log replay, or golden-vector variant).

> **Your answer:**
<br><br><br>

## Hints

1. Think about what golden vectors never exercise: everything between your function and the motors, and everything about *time*.
2. On the pad, `e_p`'s z-component is a constant ~0.5 m for as long as the script waits — integrate that.
3. Both errors are odd functions with slope ±1 at zero (mind each convention's sign); they part company as θ → π. `sin θ` folds back after 90°; `θ` keeps growing. Which keeps pushing hardest when the vehicle is nearly inverted?
4. You should land in single-digit microseconds — a fraction under 1% of the tick. The rest of the answer is in ch07's list of what the firmware does 500 times a second that is *not* control math.
5. `compute()` reads `p, v, R, w` and the reference's `pos, vel, acc, yaw` — the state side maps to `stateEstimate.*` variables; the setpoint side you already have, because you commanded it.
6. One hypothesis is Experiment 3 wearing flight clothes; one is a units/scaling row of the mapping table; one is ch07 Experiment 2's cliff. The order follows "cheapest test first."

## Further reading

1. **The crazyflie-firmware repository and documentation** (github.com/bitcraze/crazyflie-firmware; docs on bitcraze.io) — the controller sources under `src/modules/src/controller/`, the controller interface header under `src/modules/interface/`, and the documentation chapters on the out-of-tree build and the app layer: the authoritative, current answers to every API question this chapter deliberately hedged.
2. **Lee, Leok, McClamroch, "Geometric Tracking Control of a Quadrotor UAV on SE(3)," CDC 2010** — the equations you just ported, with the stability proofs that say why Experiment 3's sign is the difference between a controller and an anti-controller.
3. **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors," ICRA 2011** — the paper `controller_mellinger.c` is named for; read its controller section against your verified mapping table.
4. **`docs/ARCHITECTURE.md` §2 and `docs/FLIGHT_TEST_PLAN.md`** — the control architecture of record (what the port must preserve) and the campaign your firmware re-enters at FT-1.1 (what the port must pass).

---

*← Previous: [Chapter 8 — Decentralized swarms](ch08-decentralized-swarms.md) · [Course index](README.md) — this is the final chapter of the course.*
