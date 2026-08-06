# flight/ — Microcontroller & flight-code guide

Two flyable brains exist for this project, and they play different roles:

- **The DIY printed-frame build** (`hardware/frame.scad` + BetaFPV brushed AIO board):
  you *configure* Betaflight and fly it **manually** with a radio. No position autonomy —
  see `docs/DESIGN.md` §9: with only an IMU, position/velocity are not observable.
- **The Crazyflie 2.1+**: the research platform whose firmware ships a Mellinger
  geometric controller, the close cousin of `quadsim/controller.py`. This is where the
  `quadsim` minimum-snap trajectories **actually fly**, via the scripts in this directory.

No hardware yet? Everything below degrades gracefully: every flight script has a
`--dry-run` mode that touches no radio and needs no drone. Read, dry-run, and be ready
for the day the parcel arrives (`docs/SOURCING_INDIA.md` for where to order).

---

## Part A — Which brain, which path

| | DIY build (Path B) | Crazyflie 2.1+ (Path A) |
| --- | --- | --- |
| MCU | STM32F411 (Cortex-M4) on a BetaFPV F4 1S brushed AIO FC | STM32F405 (Cortex-M4F, 168 MHz) + **nRF51822** radio/power SoC |
| Firmware | **Betaflight** — you configure via a GUI, you do not write code | Bitcraze flight stack on **FreeRTOS**: 500 Hz stabilizer loop (same rate as our sim `dt = 0.002`), EKF state estimator, `controller_mellinger.c` — the closest C relative of our SE(3) `controller.py` |
| Radio link | ELRS/Frsky SPI RX on the FC, hobby transmitter | 2.4 GHz CRTP via Crazyradio USB dongle; scripted from your laptop with `cflib` |
| What flies | Your thumbs (angle mode, manual) | Our `MinSnapTrajectory` output, uploaded as poly4d segments |
| Role in this project | Stick skills, hardware intuition, the ₹6–12k on-ramp | The sim-to-real twin: trajectories, logging, controller comparison |
| Rotor convention worry? | **Yes** — Part B below; our R1–R4 map is not Betaflight's default | **No** — stock vehicle, stock mixer; we command flat outputs, the firmware owns the motors |

### Why we don't write bare-metal firmware on day one

It's tempting — the ECE itch is real — but flight firmware is a *hard-real-time* system,
and ch07's numbers say why that's unforgiving: our attitude loop crosses over near
32 rad/s, so 20 ms of added latency makes it ring and ~40 ms makes it depart
(`docs/course/ch07-real-flight.md`, Experiment 2). Betaflight and the Bitcraze stack
embody years of solved problems we'd otherwise re-debug at 500 Hz with props spinning:
SPI/DMA IMU drivers, gyro filtering, estimator tuning, radio protocol, failsafe,
brownout handling. Day one, our leverage is at the *top* of the stack — trajectories,
controllers, logs — which both firmwares expose cleanly. Writing firmware becomes a
*great* project the moment you can already fly and read a tracking log, because then you
can tell your bug from physics. Two natural entry points: (1) **Silverware/NFE** on a
brushed board (the BetaFPV Lite 1S FC alternate in `docs/DESIGN.md` §5 ships it) — a
complete brushed-quad firmware small enough to actually read; (2) **patching
crazyflie-firmware** — e.g. modify one term in `controller_mellinger.c`
(ch07, Homework 3), rebuild, flash over the radio, and the stock firmware is one
reflash away if you break it.

---

## Part B — Betaflight setup checklist (DIY printed-frame build)

Bench rules first: **props off** for everything in this section until step 10 says
otherwise. Safety glasses when powered. Battery unplugged while soldering/plugging motors.

### 1. Install Betaflight Configurator
- [ ] Download Betaflight Configurator (app.betaflight.com or the GitHub release) on the
      laptop. Betaflight 4.x assumed throughout.
- [ ] Plug the FC in over USB (no battery). If it doesn't enumerate, install the STM32
      VCP/DFU drivers per the Configurator's help.

### 2. Board target and firmware
- [ ] Connect. Betaflight 4.x reads the target name from the board — note it down.
- [ ] The board ships flying; you do **not** need to flash on day one. If you do update,
      use the target named in the board's own manual (BetaFPV F411-class boards use an
      F411 target family — check the manual, don't guess from a forum post).
- [ ] Before changing anything: CLI tab → type `diff all` → save the output to a text
      file. That's your undo button.

### 3. Basic configuration
- [ ] Configuration tab: board orientation — FC arrow must point at our **+x** (the arm
      between rotors R1 and R4, `docs/DESIGN.md` §8). If the board is mounted rotated,
      set "board alignment" yaw accordingly and verify in Setup tab: the 3D model must
      mirror the real vehicle when you tilt it.
- [ ] Motor protocol: **BRUSHED** (this is a brushed-FET board — DShot/oneshot are for
      brushless and will not spin these motors). Brushed PWM frequency 16–32 kHz
      (32 kHz = quieter).
- [ ] Battery: cells = 1. For LiHV set max cell voltage 4.40 V, warning ~3.50 V,
      min ~3.30 V. Never fly below ~3.0 V under load (`docs/HARDWARE.md` safety).

### 4. Receiver
- [ ] Bind the RX per the FC manual (SPI ELRS or Frsky variant).
- [ ] Receiver tab: channel order AETR; sticks move the right bars, full stick travel
      reads ~1000–2000, centers ~1500.

### 5. Modes — angle mode is not the default!
- [ ] Modes tab: **ARM** on a switch (AUX1).
- [ ] **ANGLE** assigned to a switch *and active in the position you'll fly in*. If no
      flight-mode box is active, Betaflight flies **acro** (rate mode) — not what a
      beginner wants on flight one.

### 6. Motor mapping — check, then remap if needed

Betaflight numbers motors by *position*: **M1 = rear-right, M2 = front-right,
M3 = rear-left, M4 = front-left**. Our rotor numbers come from the sim mixer in
`quadsim/dynamics.py` and are different. If you wired exactly per `docs/DESIGN.md` §5
(R1→M4 pad, R2→M3, R3→M1, R4→M2), the mapping is already correct — but verify, never
assume:

> **WARNING — our rotors vs Betaflight motor numbers.** Both the numbering *and* every
> spin direction differ from the Betaflight default diagram. Trust this table.

| Our rotor (sim mixer) | Position | Our spin (top view) | Betaflight motor # | Betaflight *default* spin at this position | Ours vs default |
| --- | --- | --- | --- | --- | --- |
| **R1** | front-left (+d,+d) | **CCW** | **M4** | CW | opposite |
| **R2** | rear-left (−d,+d) | **CW** | **M3** | CCW | opposite |
| **R3** | rear-right (−d,−d) | **CCW** | **M1** | CW | opposite |
| **R4** | front-right (+d,−d) | **CW** | **M2** | CCW | opposite |

All four directions being "opposite" is not a wiring mistake — it's **props out**
(our convention, from the sim's allocation matrix) vs Betaflight's props-in default.
Step 7 tells Betaflight about it.

Checking the position map, props OFF:
- [ ] Motors tab → read the warning → enable the "I understand" toggle → battery in.
- [ ] Raise **one** motor slider slightly. Confirm the motor at the *Betaflight position*
      for that number spins (slider M1 → rear-right motor, etc.). Repeat for all four.
- [ ] **If a slider spins the wrong position:** use the Motors tab **motor reorder
      wizard** ("Reorder motors" button, Betaflight 4.2+): it spins one output at a
      time and you click the position that moved; it writes the remap for you. (Under
      the hood this is the CLI `resource MOTOR n pin` assignment — the wizard is the
      exact procedure, the CLI is the mechanism.) Save, then re-verify all four.

### 7. Spin direction — props out, hardware-fixed
- [ ] Same Motors-tab session: watch each shaft (a sliver of tape helps). Viewed from
      above: **R1, R3 must spin CCW; R2, R4 CW** (the §3 diagram in `docs/DESIGN.md`).
- [ ] Brushed FETs drive one polarity: direction **cannot be reversed in software**. A
      wrong direction means the wrong motor variant on that arm (CCW = white/black
      wires, CW = blue/red) or reversed connector polarity — fix the hardware.
- [ ] Tell Betaflight the scheme: Motors tab → **"Motor direction is reversed" = ON**
      (CLI: `set yaw_motors_reversed = ON`). Skip this and yaw control fights itself.
- [ ] Save. Battery out.

### 8. Rates for a beginner (angle mode)
- [ ] PID Tuning → keep stock PIDs for flight one; the board's defaults are whoop-tuned.
- [ ] "Level angle limit": **30°** (default is ~55 — too much for flight one).
- [ ] Rates: RC rate 0.80, Super/rate 0.50, expo 0.20 on roll/pitch; yaw capped near
      ~240 °/s. Sluggish is the goal; raise later, 10% at a time.
- [ ] Airmode OFF for the first flights (it keeps PIDs active at zero throttle — great
      later, confusing while learning to land).

### 9. Failsafe — verify, props OFF
- [ ] Failsafe tab: Stage 2 = **Drop**.
- [ ] Test: battery in, props off, arm, motors idling → switch the transmitter OFF →
      motors must stop within ~1 s. Receiver tab must show link loss. Do not fly until
      this test passes.

### 10. Pre-arm checklist (every flight)
- [ ] Frame: motors seated, screws tight, no cracked arms.
- [ ] Props: correct hand per arm ("A"/CCW on R1, R3; "B"/CW on R2, R4), hub boss up,
      every prop blows **down** (`docs/DESIGN.md` §8 gotcha #2), none bent.
- [ ] Battery charged (LiHV ≥ ~4.3 V resting), strapped, CG centered (fingertip balance).
- [ ] All-up weight ≤ 36 g on the scale.
- [ ] TX on first, correct model, throttle zero, arm switch off. Then battery.
- [ ] Area clear of people/pets, over carpet, glasses on.
- [ ] Arm at ≤ 0.5 m altitude target; hover throttle should sit near mid-stick
      (≈43% thrust — much higher means tired motors, sagged pack, or excess mass).

---

## Part C — Crazyflie quickstart: flying `quadsim` trajectories

This is Path A (`docs/SOURCING_INDIA.md`): Crazyflie 2.1+ kit, **Flow deck v2**,
Crazyradio. Level-1 expectations in `docs/HARDWARE.md`.

### 1. Install the client and library
- [ ] `pip install cfclient` (pulls in `cflib`). The project venv is pinned to
      Python 3.9; if recent cfclient/cflib refuse to install there, make a separate
      venv on Python ≥ 3.10 for the radio tools — the flight scripts' `--dry-run` mode
      never imports the radio stack, so it runs fine in the plain project venv.
- [ ] Radio plumbing: Linux needs Bitcraze's udev rules; macOS needs libusb
      (`brew install libusb`). Plug in the Crazyradio, launch `cfclient`, scan — the
      default address is `radio://0/80/2M/E7E7E7E7E7`.

### 2. Firmware update
- [ ] In cfclient: Connect menu → Bootloader → flash the latest release. Keep firmware
      and cflib versions current *together* — protocol drift between old firmware and
      new library is a classic time sink.

### 3. Flow deck v2
- [ ] Power off. Mount the deck **underneath** (its flow camera and laser ranger must
      see the floor), deck-front matching the Crazyflie's forward arrow, pins fully
      seated.
- [ ] Fly over a **textured, non-shiny floor in decent light** (a rug, not glossy
      tile), below ~2 m, at gentle speeds — that's the deck's comfort zone
      (`docs/HARDWARE.md` Level 1).
- [ ] Power on with the vehicle **flat and still** and leave it still for the first
      seconds: gyro bias calibration. Ch07 Experiment 3 shows exactly what moving it
      during calibration does to the estimator.

### 4. First hover — `fly_hover.py`
Always dry-run first (no radio, no drone, no cflib needed — it validates and prints the
flight plan):

```sh
.venv/bin/python flight/fly_hover.py --dry-run
```

Then, hardware on the floor, area clear, hand near Ctrl-C:

```sh
.venv/bin/python flight/fly_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --height 0.5
```

(Both scripts take `--uri`, `--height`, `--dry-run`; `fly_hover.py` adds `--hold` and
`fly_minsnap.py` adds `--csv`, `--preset`, `--speed`. Run `--help` on each script for
the authoritative list and defaults.)

Expect: takeoff, a hover holding ~0.5 m with centimeter-level wander, landing. Know
your kill: Ctrl-C the script and be ready to catch/cushion; cflib exposes an emergency
stop — learn what interruption does *on the ground* before trusting it in the air.

### 5. Fly a minimum-snap course — `fly_minsnap.py`

The pipeline, end to end:

```
waypoints ──> quadsim MinSnapTrajectory ──> poly4d CSV ──> upload_trajectory ──> start_trajectory
              (scaled-tau coefficients)     (unscaled,       (high-level          (it flies)
                                             per-segment)     commander)
```

**The formatting exercise, precisely.** `MinSnapTrajectory` stores, per segment *i*,
8 coefficients per axis in ascending powers of **scaled** time
`tau = (t − t_i)/T_i ∈ [0, 1]`. The Crazyflie high-level commander wants each segment
as a duration plus 8 coefficients in ascending powers of **unscaled** segment-local
seconds. One rule converts them:

```
c_unscaled[k] = c_scaled[k] / T_i**k        (k = 0..7, per segment i, per axis)
```

The CSV is the cflib/`uav_trajectory` **poly4d** format — one row per segment,
33 numbers:

```
duration, x0..x7, y0..y7, z0..z7, yaw0..yaw7
```

Build the trajectory with modest speed for a flow-deck room (the sim demos' 2 m/s
through a 4×3×2 m volume is Lighthouse/Level-2 territory — scale it down):

```python
import numpy as np
from quadsim.trajectory import MinSnapTrajectory

wps = np.array([[0.0, 0.0, 0.5],
                [0.8, 0.0, 0.8],
                [0.8, 0.8, 0.5],
                [0.0, 0.0, 0.5]])
traj = MinSnapTrajectory(wps, avg_speed=0.7, yaw_mode="fixed")
```

Two hardware-facing notes: start the course at (0, 0, takeoff height) — the flow-deck
EKF's origin is wherever it powered on; and use `yaw_mode="fixed"` for export
(velocity-following yaw is a precomputed grid in `quadsim`, not a polynomial — it has
no poly4d twin, so the yaw columns are a constant).

Then the usual two-step:

```sh
.venv/bin/python flight/fly_minsnap.py --csv out/course.csv --dry-run
.venv/bin/python flight/fly_minsnap.py --csv out/course.csv --uri radio://0/80/2M/E7E7E7E7E7
```

The dry run validates the CSV (row width, positive durations, sane peak
velocity/accel) and prints the plan without touching a radio. The real run takes off,
uploads the segments, starts the trajectory, and lands.

To compare against the sim's controller rather than the stock PID: set the firmware
parameter `stabilizer.controller` to the **Mellinger** controller (value 2 in current
firmware) via cfclient's parameter tab — that puts `controller_mellinger.c`, the
firmware's near-twin of our `controller.py`, in the loop. Stock PID is fine (and
forgiving) for first flights.

### 6. What to expect vs the sim — and why

The sim hands the controller the exact state; the Crazyflie estimates it from an IMU +
flow deck. Everything below is quantified in
[`docs/course/ch07-real-flight.md`](../docs/course/ch07-real-flight.md); the
fingerprints to watch for:

- **Wander, not jitter.** White estimator noise barely matters (the vehicle's inertia
  low-passes it — ch07 Exp 1: 1 cm/5 cm/s noise costs ~2 mm RMS). What you'll see
  instead is slow cm-level wander: flow drift is a random walk, and it grows over
  minutes and on featureless floors.
- **Constant offsets = bias, not bad tuning.** A miscalibrated velocity estimate parks
  the vehicle a fixed distance off target (`e_p = −(k_v/k_p)·b`; 0.1 m/s → 50 mm in
  our gains). Recalibrate/re-place before touching gains.
- **Altitude sag late in the flight** is battery sag, not the controller — the
  firmware's integral terms (`ki_*` in `controller_mellinger.c` — the terms our
  exact-model sim never needed) absorb most of it.
- **Tracking "inside" the simulated path** on fast segments: aerodynamic drag and
  motor lag, neither of which `quadsim` models (`docs/DESIGN.md` §9).
- **Why your laptop isn't the controller:** the 500 Hz attitude loop lives onboard
  because ~40 ms of loop latency is departure (ch07 Exp 2). The radio carries
  trajectories and logs, never the inner loop.

Log it like the sim: configure cfclient log blocks for position/velocity/attitude
setpoint-vs-estimate — the same signal set as `quadsim.sim.History` — and reuse your
`plot_tracking` reading habits on real data. When hover throttle, mass, or tracking
drift from the sim's predictions, push measured values back into `quadsim/params.py`:
that's the twin staying a twin.

### 7. Safety, always
Props off for bench work; glasses close-up; replace bent props (vibration is in-band
accelerometer noise — it wrecks the estimator before the frame). LiPo: charge attended,
retire crashed/puffed packs, store ~50%. First flights low, slow, one vehicle, cushion
underneath, and test the emergency stop on the ground (`docs/HARDWARE.md` safety
section is binding). Indoors you set the rules; outdoors, Drone Rules 2021 apply —
nano class helps, red zones don't care (`docs/SOURCING_INDIA.md`).
