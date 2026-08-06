# FLIGHT_TEST_PLAN.md — Crazyflie autonomous flight-test campaign

A complete, gated flight-test campaign for the **Crazyflie 2.1+ / Flow deck v2 / Crazyradio**
autonomous path (the Level-1 package in [`docs/HARDWARE.md`](HARDWARE.md)), designed for a
**solo operator flying indoors in Delhi**. Every test is a numbered card a single person can
execute alone, with quantitative pass criteria traced to the simulation targets in
[`README.md`](../README.md) and the sim-to-real deltas in [`docs/DESIGN.md`](DESIGN.md) §9.

Binding context:

- **Platform**: Crazyflie 2.1+ with Flow deck v2, scripted over Crazyradio with `cflib` —
  the flight scripts in [`flight/`](../flight/README.md) Part C. The DIY printed whoop
  (Betaflight, manual-only) is a separate mechanical-learning track and is **not** covered
  here: with only an IMU its position is unobservable (`docs/DESIGN.md` §9), so it cannot
  fly this campaign.
- **Sim twin**: `quadsim` at 500 Hz (`dt = 0.002 s` in `quadsim/sim.py`) — the same rate as
  the Crazyflie stabilizer loop (`docs/course/ch07-real-flight.md`). Trajectories are
  `MinSnapTrajectory` output exported through
  [`flight/cf_trajectory.py`](../flight/cf_trajectory.py) (poly4d, 33 floats/row,
  validated to peak speed ≤ 3.0 m/s and peak accel ≤ 8.0 m/s²).
- **Tooling**: [`flight/preflight.py`](../flight/preflight.py) automates FT-0.1 through
  FT-0.5 — five bench checks: radio, deck, battery, estimator convergence, attitude
  level; the flight scripts write analyze-ready flight logs via `--log` (shared helper
  [`flight/flightlog.py`](../flight/flightlog.py), §4.1);
  [`flight/analyze_log.py`](../flight/analyze_log.py) takes that flight-log CSV plus
  (via `--ref`) the poly4d reference CSV and reports RMS/max tracking error — the
  hardware sibling of the sim's `History.rms_pos_error`;
  [`flight/estop.py`](../flight/estop.py) is the emergency-stop helper (§2.6).
- **Regulatory**: India Drone Rules 2021, nano class (≤ 250 g) — see §2.7.

A note on numbers: every threshold below either comes from a repo file (cited) or is
derived inline. Where a real-world bar is looser than its sim twin, the relaxation and its
reason are stated — that gap *is* the measurement this campaign exists to make.

---

## 1. Campaign overview

### 1.1 Phase structure and hard gates

Five phases, strictly sequential:

| Phase | Name | Question it answers |
| --- | --- | --- |
| **FT-0** | Bench & ground checks | Does every subsystem work with props off / vehicle on the pad? |
| **FT-1** | First hovers | Can the vehicle hold a point in this room, with this floor and light? |
| **FT-2** | Trajectory following | Do exported `quadsim` min-snap trajectories fly, at conservative speed? |
| **FT-3** | Aggressive maneuvers | Where does tracking degrade as speed rises toward 2 m/s peak? |
| **FT-4** | Endurance & robustness | Is performance repeatable, and what does the battery really give? |

**Hard gate rule: a phase may not begin until every test of the previous phase has passed
and been logged.** A failed test is re-flown after the cause is understood (not just after
a reboot); three consecutive failures of the same card end the session — write an anomaly
report (§4.3) and stop. There is no skipping ahead "just to see": the solo operator has no
second pair of eyes to catch what a skipped check would have caught.

Additional standing gates:

- **G-DRY**: every scripted flight is preceded, same session, by the identical command with
  `--dry-run` appended, and the dry run must exit 0 (`fly_hover.py` / `fly_minsnap.py`
  both support it; the min-snap dry run validates row width, positive durations, and the
  3.0 m/s / 8.0 m/s² peaks via `cf_trajectory.validate_rows`).
- **G-PREFLIGHT**: `flight/preflight.py` passes at the start of every session (it wraps
  FT-0.1–FT-0.5 — all five bench checks, attitude level included — as an automated
  regression once each has passed manually).
- **G-BATTERY**: no FT-session takeoff below **3.9 V resting** (the `preflight.py`
  default); land by 3.2 V under load (§2.4). 3.7 V resting is the documented absolute
  floor, usable only for non-FT practice via `preflight.py --min-voltage 3.7` — never
  for an FT card.

### 1.2 Campaign at a glance

| ID | Title | Props | Key pass criterion | Sim twin |
| --- | --- | --- | --- | --- |
| FT-0.1 | Radio link + firmware | off | Stable connect, firmware version recorded | — |
| FT-0.2 | Deck detection + sensor sanity | off | Flow deck detected; ranger/flow respond correctly | — |
| FT-0.3 | Battery health | off | Full-charge resting ≥ 4.15 V; no physical damage | — |
| FT-0.4 | Estimator convergence on the pad | off | Kalman variances settle (< 0.001 spread) within 15 s | ch07 Exp 3 |
| FT-0.5 | Attitude level on the pad | off | \|roll\|, \|pitch\| ≤ 2° on a flat pad | — |
| FT-0.6 | Emergency-stop drill | **off** | Ctrl-C lands, E-stop kills motors < 1 s, 3/3 trials | — |
| FT-1.1 | First hover, 0.5 m / 5 s | on | Horiz. excursion ≤ 0.10 m; alt 0.5 ± 0.10 m | `demo_hover.py` |
| FT-1.2 | Extended hover, 30 s | on | Total drift ≤ 0.20 m, no runaway | `demo_hover.py` |
| FT-1.3 | Hover height ladder | on | Each rung ≤ 0.15 m wander; drift-vs-height curve | `demo_hover.py` |
| FT-2.1 | Straight line (0.8 m) | on | RMS ≤ 0.10 m, max ≤ 0.20 m | `demo_minsnap.py` |
| FT-2.2 | Square circuit, 0.5 m/s | on | RMS ≤ 0.10 m, max ≤ 0.20 m | `demo_minsnap.py` |
| FT-2.3 | Speed ramp on the square | on | RMS ≤ 0.15 m at 1.0 m/s avg | `demo_minsnap.py` |
| FT-3.1 | Figure-8 | on | RMS ≤ 0.15 m, max ≤ 0.30 m | `demo_figure8.py` |
| FT-3.2 | Speed sweep toward 2 m/s peak | on | RMS ≤ 0.20 m at 2.0 m/s peak; degradation curve | `demo_figure8.py` |
| FT-4.1 | Battery-sag endurance | on | Planned landing, sag curve captured | DESIGN §9 (sag) |
| FT-4.2 | Repeatability ×3 | on | 3/3 pass FT-2.2 bars; RMS spread ≤ 0.05 m | `demo_minsnap.py` |
| FT-4.3 | Parameter feedback into `params.py` | off | Measured m pushed to sim; tests + demos re-pass | DESIGN §9 (closing) |

---

## 2. Environment & safety specification

### 2.1 Flight volume

| Requirement | Value | Derivation |
| --- | --- | --- |
| Minimum clear volume, FT-0/1 | 2.5 × 2.5 × 2.2 m | Hover wander bar is 0.20 m (FT-1.2); vehicle + 0.7 m margin to every obstacle on all sides. |
| Minimum clear volume, FT-2/3 | 3.0 × 3.0 × 2.2 m | Largest course footprint: figure-8 of 1.6 × 0.8 m (§FT-3.1) plus max-error allowance 0.30 m plus ≥ 0.7 m to the net on each side. |
| Maximum test altitude | 1.5 m | Flow deck v2 is "happiest below roughly 2 m" (`docs/HARDWARE.md` Level 1); 1.5 m keeps 25 % margin under that ceiling. |
| Clearance below ceiling | ≥ 0.7 m | Prop wash recirculation near the ceiling destabilizes height hold (same family as the ground-effect caveat in `docs/DESIGN.md` §9). |
| Floor covering | rug/carpet or foam mat under the whole volume | Crash energy: ~0.03 kg from 1.5 m is < 0.5 J — soft floor makes most crashes consumable-free. |

The room is cleared of people and pets for every powered test. Solo rule: **nobody else in
the volume means nobody unbriefed can wander in** — close the door.

### 2.2 Floor texture and lighting (Flow deck v2)

The Flow deck's PMW3901 optical-flow camera measures floor texture motion; the paired laser
ranger measures height. Both `docs/HARDWARE.md` (Level 1) and `flight/README.md` Part C
require a **textured, non-shiny floor in decent light**. Operationally:

- **Required**: matte, visually textured surface — a patterned rug, textured carpet, or a
  printed-pattern mat. If the room's floor is glossy or uniform, lay a rug that covers the
  whole flight footprint plus the max-error allowance.
- **Forbidden**: glossy tile, polished stone, uniform single-color floors, mirrors/glass,
  deep-pile carpet that sways in prop wash.
- **Lighting**: bright, even, indirect room light. **No direct sunlight** on the floor —
  it saturates the flow camera, adds IR that disturbs the laser ranger, and moving
  sun/shadow boundaries look like floor motion to the flow sensor. No strongly flickering
  sources; curtains closed for repeatability (also removes time-of-day as a variable in
  FT-4.2).
- **Verification, not assumption**: FT-1.2's drift measurement is the quantitative floor
  qualification. If drift exceeds its bar, fix texture/light before touching anything else
  — ch07 Exp 1's moral is that estimator *drift and bias*, not noise, are what hurt.

### 2.3 Netting and perimeter for a solo operator

`docs/HARDWARE.md`: "the TED talk's flying arena is wrapped in netting for a reason, and
that lab flies better than you do." Solo, the net is not optional decoration — it is the
second crew member:

- Mesh curtain / garden net enclosing the flight volume before **any** FT-2 flight, and
  strongly recommended from FT-1.1. Net stands ≥ 0.7 m outside the planned trajectory
  extremes (§2.1). Weight or tape the bottom edge; a vehicle sliding under the net at
  ankle height defeats it.
- **Operator station outside the net**, at the volume edge, with an unobstructed sightline
  to the whole volume and the laptop on a stable surface. One hand stays within reach of
  the keyboard (Ctrl-C) for the entire flight — the solo operator is pilot, safety
  officer, and note-taker; the station layout must let one person do all three without
  moving.
- Windows and shelves on the trajectory side get the net between them and the vehicle.
- Never reach into the volume while the script is running. Approach only after the script
  prints `Done.` (both flight scripts print it after `commander.stop()`) **and** the props
  are visibly stopped.
- No hand-catching, ever. A landing that drifts is a data point; a hand in props is a
  hospital trip with nobody to drive you — that asymmetry is the whole solo-ops safety
  argument.

### 2.4 LiPo handling

Per `docs/HARDWARE.md` (binding, per `flight/README.md` §7):

- Charge attended, on a non-flammable surface, ideally in a LiPo-safe bag. Never leave the
  building with a pack charging.
- Never charge or fly a puffed, punctured, or crash-damaged pack — retire it (salt-water
  discharge, then local disposal rules).
- Store at ~50 % charge, not full.
- **Voltage discipline** (1S pack: 4.2 V full, hard floor ~3.0 V/cell under load, from
  `docs/HARDWARE.md` and ch07 "battery sag"): take off ≥ **3.9 V resting** for every FT
  session (the `preflight.py` default and gate G-BATTERY); **land by 3.2 V under load**
  — the 0.2 V margin above the 3.0 V floor covers the sag transient plus the ≈ 4.5 s
  landing sequence (2.0 s `commander.land(0.0, 2.0)` descent + 2.5 s settle in both
  flight scripts). 3.7 V resting is the absolute floor — non-FT practice only, invoked
  explicitly with `--min-voltage 3.7`. `pm.vbat` is in every flight log (§3) precisely
  to enforce the landing line.
- **Fire response**: a sand bucket or dry-powder extinguisher stays within reach of the
  charging spot, and the charger is never placed between the operator and the exit — a
  LiPo that lights is abandoned, not carried.
- After any crash: disconnect, inspect the pack (swelling, dents, hot spots) before it goes
  anywhere near the charger — then run the full §2.9 re-qualification checklist before
  the next powered test.

### 2.5 Props-off discipline

- **All of FT-0 is flown with props removed.** No exceptions, including "just a quick
  re-check". Bench work with props on and a battery connected is the classic self-inflicted
  injury (`docs/HARDWARE.md` safety; `flight/README.md` §7).
- Props go on only at the start of a flight-phase session, after the §2.8 checklist, and
  come off again for any hands-on debugging.
- Safety glasses whenever powered with props on and you are within 2 m, and for all
  close-up debugging.
- Replace bent props immediately: "vibration is in-band accelerometer noise — it wrecks
  the estimator before the frame" (`flight/README.md` §7, ch07).

### 2.6 Emergency stop — the cflib mechanism

Three layers, all exercised in FT-0.6 before any flight:

1. **Ctrl-C on the script = commanded landing.** Both flight scripts catch
   `KeyboardInterrupt` during flight and execute the normal land sequence
   (2.0 s `commander.land(0.0, 2.0)` descent + settle + stop, §2.4) before closing the
   link — Ctrl-C *is* the commanded-landing mechanism, not an abandonment. If the
   script is hung, or a second Ctrl-C interrupts the landing itself, the link closes
   and the firmware's commander watchdog cuts motors after its timeout — that teardown
   path is the backstop, and FT-0.6 measures the actual latencies props-off.
2. **cflib emergency stop** via [`flight/estop.py`](../flight/estop.py)
   (it sends `cf.loc.send_emergency_stop()` — the mechanism `docs/HARDWARE.md` tells
   you to "wire up before your first scripted flight and test on the ground"):
   immediate motor kill, vehicle falls. FT-0.6 keeps `flight/estop.py` open in a
   second terminal for this. **Single-Crazyradio constraint:** one dongle serves one
   process — while a flight script holds the radio, `estop.py` cannot open it. With
   one dongle the real layer-2 sequence is: kill the script (second Ctrl-C or close
   its terminal) → the watchdog cuts motors within its timeout → fire `estop.py`
   once the dongle frees, as confirmation and to leave the vehicle safed. A **second
   Crazyradio** (see `docs/SOURCING_INDIA.md` first-order list) removes the
   constraint: `estop.py --uri` on dongle 2 kills motors instantly, mid-flight,
   independent of the script.
3. **Physical**: the vehicle is 30-ish grams (`docs/HARDWARE.md`) over a soft floor inside
   a net — worst case, let it fall. Falling is the designed failure mode; chasing it is not.

Know the trade before you need it: layer 1 is graceful — a controlled ≈ 4.5 s descent —
but needs a vehicle stable enough to fly the landing; layer 2 is instant but guarantees
a fall. FT-0.6 exists so this choice is reflexive, not researched mid-incident.

### 2.7 India Drone Rules 2021 — nano class, indoor

From [`docs/SOURCING_INDIA.md`](SOURCING_INDIA.md) ("Rules — the good news for us"):

- The Crazyflie 2.1+ at ~30 g sits deep inside the **nano category (≤ 250 g)**: for
  recreational use, no registration, no UIN, no pilot license.
- **Indoor flying is unregulated** — this entire campaign is indoor by design, which also
  matches the Flow deck's needs (§2.2) and Kumar-lab practice (netted indoor arena).
- Delhi outdoors is heavily red-zoned (airport, VIP areas). **No test in this plan flies
  outdoors.** If that ever changes, check the Digital Sky map
  (digitalsky.dgca.gov.in) first and re-plan — outdoor flight is out of scope here.

### 2.8 Pre-session checklist (run once per session, before props go on)

- [ ] Room: door closed, people/pets out, volume clear per §2.1, net rigged and bottom
      edge secured (§2.3).
- [ ] Floor: textured mat covers footprint + error allowance; no direct sunlight on it;
      lights on (§2.2).
- [ ] Operator station: laptop stable, outside net, full sightline, charger and spare pack
      staged on the non-flammable surface.
- [ ] `flight/preflight.py` passes (radio, deck, battery, estimator, attitude level —
      the automated FT-0.1–0.5 regression). If it fails, the session is a bench session.
- [ ] Battery: resting ≥ 3.9 V (≥ 4.15 V if the card needs a full pack), no puffing.
- [ ] Airframe: props correct and unbent, deck seated, no cracked arms; weigh if anything
      changed since FT-4.3.
- [ ] Dry-run of every command you intend to fly today exits 0 (gate G-DRY).
- [ ] Log book row opened (§4.2); log file names pre-decided (§4.1).
- [ ] `flight/estop.py` open and ready in a second terminal (§2.6 layer 2); you can
      state today's abort criteria from memory.
- [ ] Vehicle placed **flat and still** at the pad origin; power-on with hands off for the
      first seconds — gyro bias calibration (`flight/README.md` Part C §3; ch07 Exp 3
      shows what moving it during calibration costs).

Fatigue rule (solo-specific): hard stop after 90 minutes or two anomalies, whichever comes
first. Nobody is watching your judgment degrade except you.

### 2.9 After any crash — re-qualification checklist

Run after **every** crash, however minor, before the next powered test (referenced from
the §3 standard aborts and the §4.3 anomaly rule):

- [ ] Battery inspected per §2.4 (swelling, dents, hot spots) before it goes anywhere
      near the charger.
- [ ] Props inspected close-up; any bent prop replaced (vibration is estimator damage —
      §2.5).
- [ ] Flow deck reseated on its pins; flow-camera and ranger lenses wiped clean.
- [ ] Frame flex check: gentle twist of each arm; a cracked or soft arm grounds the
      vehicle.
- [ ] `flight/preflight.py` re-passes (the FT-0.1–0.5 regression) before props go back
      on.

---

## 3. Test cards

Card conventions:

- **Pad origin**: the Flow-deck EKF's origin is wherever the vehicle powered on
  (`flight/README.md` Part C §5). Mark a fixed pad spot with tape; every flight starts
  there, so `stateEstimate.x/y` are pad-relative and comparable across flights.
- **Standard log** (every scripted flight): pass `--log out/logs/<name>.csv` to
  `fly_hover.py` / `fly_minsnap.py` — the shared helper
  [`flight/flightlog.py`](../flight/flightlog.py) records `t,x,y,z,vbat` (`t` in
  seconds) during the flight, in exactly the format `analyze_log.py` consumes. For
  trajectory flights add `--save-ref out/logs/<name>_ref.csv` to archive the flown
  poly4d rows. This is the hardware `History` — the same signal habit the sim logs
  (`quadsim/sim.py`), as ch07's walkthrough promises.
- **Fallback log path — non-scripted sessions only**: cfclient GUI log blocks
  (Block A, 10 ms period: `stateEstimate.x/y/z/vx/vy/vz`; Block B, 100 ms period:
  `pm.vbat`, `kalman.varPX/PY/PZ`, `range.zrange`). cfclient **cannot share the radio
  link with a running flight script**, so this path exists only for manual/bench
  sessions (e.g. the FT-0.x hand checks); its export needs the §4.1 rename-and-rescale
  step before `analyze_log.py` will read it.
- **Standard aborts** (apply to every airborne card, in addition to per-card aborts):
  `vbat` < 3.2 V under load; vehicle within 0.5 m of the net or any obstacle;
  visible oscillation/ringing; any behavior you cannot explain; loss of radio telemetry
  > 2 s. Abort = Ctrl-C first — it commands the normal landing sequence (§2.6 layer 1);
  `flight/estop.py` if the vehicle is accelerating toward a boundary. After any crash,
  run the §2.9 re-qualification checklist before the next powered test.
- **Post-flight, every airborne card**: save the `--log` CSV per §4.1, fill the
  log-book row, and for trajectory flights run
  `.venv/bin/python flight/analyze_log.py <log.csv> --ref <ref.csv> --t0 <seconds>`
  (hover cards: `--hover X Y Z` in place of `--ref`) for the RMS/max error verdict.

---

### Phase FT-0 — bench & ground checks (props OFF throughout)

#### FT-0.1 — Bench radio link + firmware baseline

- **Objective**: establish a reliable Crazyradio↔Crazyflie link and pin the firmware
  version the whole campaign flies.
- **Prerequisites**: `cfclient`/`cflib` installed per `flight/README.md` Part C §1
  (separate ≥ 3.10 venv if the pinned 3.9 project venv refuses); Crazyradio plugged in;
  udev rules (Linux) / libusb (macOS) done. Props OFF, vehicle on pad.
- **Setup**: fully charged pack; vehicle 2–3 m from the radio (realistic range, not
  touching).
- **Procedure**:
  1. Launch `cfclient`, scan; expect the default URI `radio://0/80/2M/E7E7E7E7E7`.
  2. Connect; record firmware version from the console/about panel in the log book.
  3. Update firmware via Connect → Bootloader to the latest release **now**, per
     `flight/README.md` Part C §2 — firmware and cflib move together; protocol drift is
     "a classic time sink". Re-record the version. Do not update again mid-campaign
     unless an anomaly demands it (version is a controlled variable for FT-4.2).
  4. Stay connected 5 minutes with the console open; watch for disconnects/errors.
  5. Run `flight/fly_hover.py --dry-run` and confirm exit 0 (validates the toolchain
     without touching the radio).
- **Pass criteria**: scan finds the vehicle on the first or second attempt; 5-minute
  connection with zero link drops; firmware version recorded; dry run exits 0.
  *Rationale*: a link that drops on the bench will drop with the vehicle at 1.5 m — and
  a solo operator troubleshooting a link mid-flight has no one watching the aircraft.
- **Abort**: repeated enumeration failures → stop, fix drivers; this card costs nothing
  to re-run.
- **Log**: firmware version, cflib version, URI, link-quality impression → log book.
- **Post-analysis**: none beyond records. `flight/preflight.py` automates this check
  from now on.

#### FT-0.2 — Flow deck detection + sensor sanity

- **Objective**: confirm the Flow deck v2 is detected and both its sensors (PMW3901 flow
  camera, laser ranger) produce physically sensible data.
- **Prerequisites**: FT-0.1. Props OFF.
- **Setup**: deck mounted **underneath**, deck-front matching the Crazyflie forward arrow,
  pins fully seated (`flight/README.md` Part C §3). Textured mat under the pad (§2.2).
- **Procedure**:
  1. Power on flat and still on the pad; connect with `cfclient`.
  2. Deck detection: parameter `deck.bcFlow2` must read 1; console shows no deck errors.
  3. Ranger sanity: log `range.zrange` while holding the vehicle level by hand (fingers
     clear of motor shafts) at measured heights of ~0.2 m, 0.5 m, 1.0 m against a tape
     measure. Readings should track within ~a few cm and respond immediately.
  4. Flow sanity: with the plotter on `motion.deltaX` / `motion.deltaY`, translate the
     vehicle slowly fore/aft and left/right ~0.5 m above the mat. Deltas must be strong,
     signed consistently with motion direction, and near-zero when still.
  5. Repeat step 4 over the *worst* floor patch you might fly over; note the difference —
     this is your first floor-texture measurement.
- **Pass criteria**: `deck.bcFlow2 = 1`; zrange tracks hand-held height within ±0.05 m at
  0.5 m; flow deltas respond with correct sign on both axes and are quiet at rest.
  *Rationale*: ±0.05 m at 0.5 m is 10 % — coarse enough for a hand-held test, tight
  enough to catch a mis-seated deck or obstructed lens. The flow sign check catches a
  deck mounted 180° off, which would otherwise become a violent position runaway on FT-1.1.
- **Abort**: n/a (props off).
- **Log**: zrange-vs-tape table (3 points), flow response notes → log book.
- **Post-analysis**: none. Folded into `flight/preflight.py` thereafter.

#### FT-0.3 — Battery health baseline

- **Objective**: qualify every pack in the fleet and record a per-pack baseline for the
  FT-4.1 sag comparison.
- **Prerequisites**: FT-0.1. Props OFF.
- **Setup**: all packs charged full, attended, per §2.4; pack IDs written on each
  (P1, P2, …).
- **Procedure**, per pack:
  1. Visual/mechanical: no puffing, dents, connector damage.
  2. Resting voltage after full charge, off the vehicle if you have a meter, else via
     `pm.vbat` a minute after connect.
  3. On-vehicle: log `pm.vbat` for 2 minutes idle (radio on, motors off); note droop.
  4. Record pack ID, purchase date, resting V, idle droop in the log book's battery table.
- **Pass criteria** (per pack): resting ≥ 4.15 V after a full charge (a healthy 1S LiPo
  charges to 4.2 V — ch07 "battery sag"; 0.05 V allowance for meter/settle); no physical
  defects; idle droop < 0.05 V over 2 min.
  *Rationale*: a pack that cannot hold 4.15 V resting has lost capacity and will hit the
  3.2 V abort line early — better to find out on the bench than at 1.5 m altitude.
- **Abort/fail action**: failing packs are retired per §2.4, not "kept for short flights".
- **Log**: battery table (§4.2) — this table is the FT-4.1 input.
- **Post-analysis**: none. `flight/preflight.py` re-checks resting voltage each session.

#### FT-0.4 — Estimator convergence on the pad

- **Objective**: verify the EKF converges reliably on your actual floor, and measure how
  long it takes — this wait is inside every flight script.
- **Prerequisites**: FT-0.2 (deck sane). Props OFF.
- **Setup**: vehicle flat and still on the pad, textured mat, session lighting (§2.2).
- **Procedure**:
  1. Power on hands-off (gyro bias calibration — ch07 Exp 3's "break it" case is exactly
     a vehicle moved during these seconds).
  2. In `cfclient`, log `kalman.varPX/varPY/varPZ` at 500 ms.
  3. Trigger the reset the scripts use: set `kalman.resetEstimation` = 1 then 0 (this is
     `reset_estimator()` in both `fly_hover.py` and `fly_minsnap.py`).
  4. Time how long until the script's own convergence criterion holds: max−min of each
     variance < 0.001 over the last 10 samples at 500 ms — i.e., a 5 s settled window
     (`_wait_for_position_estimator` in the flight scripts).
  5. Repeat 3 times; then once more with the room lights dimmed to your worst plausible
     session lighting, to see the margin.
- **Pass criteria**: converges within **15 s** of reset (the `preflight.py` default
  timeout), 3/3 trials at session lighting.
  *Rationale*: the scripts block on this exact criterion before arming; a bench
  convergence time beyond 15 s predicts a hang mid-procedure. The 10 × 500 ms settled
  window makes ~6 s the physical minimum, so 15 s still leaves healthy margin — the bar
  exists to catch pathologies (bad texture, sun on the floor), not to race. Cold starts
  may be *investigated* with `preflight.py --timeout 30`, but the pass bar stays 15 s.
- **Abort**: n/a (props off). Non-convergence → fix floor/light per §2.2, re-run.
- **Log**: convergence times (4 trials) → log book.
- **Post-analysis**: none. Folded into `flight/preflight.py`.

#### FT-0.5 — Attitude level on the pad

- **Objective**: confirm the attitude estimate reads level on a flat pad — an estimate
  tilted by θ becomes a lateral acceleration command of g·sin θ at takeoff (2° →
  0.34 m/s², noticeable drift; 5° → 0.86 m/s², abort territory).
- **Prerequisites**: FT-0.4. Props OFF.
- **Setup**: vehicle flat and still at the pad origin, powered on hands-off (gyro bias
  calibrates at boot).
- **Procedure**: read `stabilizer.roll` / `stabilizer.pitch` (cfclient plotter, or just
  run `flight/preflight.py` — this is its fifth check); power-cycle and repeat once.
- **Pass criteria**: |roll| and |pitch| ≤ **2°** on the flat pad, both trials
  (`preflight.py` WARNs from 2° to 5° and FAILs beyond).
- **Abort**: n/a (props off). Above the bar → re-place flat and power-cycle before
  suspecting the pad itself.
- **Log**: roll/pitch readings → log book.
- **Post-analysis**: none. Automated by `flight/preflight.py` thereafter.

#### FT-0.6 — Emergency-stop drill (props OFF)

- **Objective**: make both software stop layers (§2.6) reflexive, and measure their
  latency, before anything flies.
- **Prerequisites**: FT-0.1–FT-0.5. **Props OFF — verify twice; this card spins motors.**
- **Setup**: vehicle on the pad, strapped or held down is unnecessary (no props = no
  thrust); safety glasses anyway; `flight/estop.py` open in terminal 2.
- **Procedure**:
  1. Start `flight/fly_hover.py --height 0.5` for real. The vehicle will "take off"
     — motors spin, nothing lifts (props off).
  2. Trial A ×3: during the hold phase, Ctrl-C. The script catches it and flies the
     normal land sequence (§2.6 layer 1): motors ramp down through the 2.0 s land +
     settle, then stop. Time keypress → motor stop; note what the script prints and
     what state the vehicle/link is left in.
  3. Trial B ×3: restart; during the hold phase, rehearse the **hard-kill sequence
     for your dongle count** (§2.6 layer 2). One Crazyradio: second Ctrl-C (or close
     the script's terminal) → time keypress → watchdog motor cut; then fire
     `flight/estop.py` from terminal 2 once the dongle frees and confirm it connects
     and reports the stop. Two Crazyradios: fire `flight/estop.py` live from
     terminal 2 mid-"flight" — motors must cut immediately while the script still
     holds dongle 1.
  4. After each trial, practice the full recovery: power-cycle, re-place at pad origin,
     reconnect — the exact sequence you'll need after a real abort.
- **Pass criteria**: motors stop < 1 s after E-stop, 3/3; Ctrl-C triggers the scripted
  landing sequence and motor stop 3/3, with keypress-to-motor-stop time recorded
  (expect ≈ 4.5 s — the §2.4 land + settle; that duration is your layer-1 budget);
  recovery sequence executed without notes by the third trial.
  *Rationale*: `docs/HARDWARE.md` is explicit — "wire up an emergency stop before your
  first scripted flight… test it on the ground." Solo, the drill matters double: there
  is no one to shout instructions during an abort.
- **Abort**: n/a.
- **Log**: latencies for both layers, recovery notes → log book.
- **Post-analysis**: none.

**GATE: all of FT-0 passed and logged → props may go on. This is the FT-1 entry gate.**

---

### Phase FT-1 — first hovers

Sim twin for the whole phase: `demos/demo_hover.py` — sim target "settle to < 2 cm;
steady-state RMS < 0.005 m" (`README.md` demo table). Reality will not match it, by a
factor you are about to measure: the sim controller sees exact state; the Crazyflie sees
an EKF fed by optical flow whose error is a slow random walk plus bias, not white noise
(ch07 Exp 1: 1 cm/5 cm/s *white* noise costs only ~2 mm RMS; a 0.1 m/s velocity *bias*
parks the vehicle 50 mm off target via e_p = −(k_v/k_p)·b = −0.5 × 0.1 m).

#### FT-1.1 — First takeoff–hover–land, 0.5 m, 5 s

- **Objective**: first autonomous flight; verify takeoff, hold, and landing behave as
  scripted.
- **Prerequisites**: all FT-0 passed. §2.8 checklist done. Cushion at pad optional but
  encouraged (`docs/HARDWARE.md`: "first flights low, slow, one vehicle, cushion
  underneath").
- **Setup**: pad origin, full pack (≥ 4.15 V — first flights get full margin), net up,
  `--log` file name pre-decided per §4.1, glasses on, hand at keyboard.
- **Procedure**:
  1. `flight/fly_hover.py --dry-run` — read the printed plan, confirm it says 0.5 m / 5 s.
  2. `flight/fly_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --height 0.5 --log
     out/logs/<date>_FT-1.1_f01.csv`
  3. The script will: reset estimator (wait for convergence), arm, take off over 2.0 s
     (0.25 m/s climb — `commander.takeoff(args.height, 2.0)`), hold 5 s, land over 2 s,
     stop. Watch, hand on Ctrl-C; do not approach until `Done.`
  4. Repeat once more, same pack, to confirm the first result wasn't luck.
- **Pass criteria** (both flights, from the `--log` CSV):
  - Horizontal excursion from takeoff point ≤ **0.10 m** at all times during the hold.
    *Rationale*: sim bar (0.005 m RMS) is unreachable on flow — the dominant error is
    bias-class, and ch07 quantifies 0.1 m/s of velocity bias at a 50 mm parked offset.
    A healthy deck on a good floor should sit well under that bias; 2× (0.10 m) passes
    normal cm-level wander (the `flight/README.md` Part C expectation: "hover holding
    ~0.5 m with centimeter-level wander") while failing a mis-calibrated or badly-lit
    setup.
  - Altitude 0.5 ± **0.10 m** during the hold. *Rationale*: zrange is the strong sensor
    here; ±0.10 m (20 %) catches ground-effect wobble or a bad ranger while tolerating
    the takeoff/land transients at the window edges.
  - Landing within **0.20 m** of takeoff point, upright.
- **Abort**: standard aborts; plus any lateral drift that looks like a runaway (steady
  acceleration in one direction — the FT-0.2 deck-orientation failure signature) →
  Ctrl-C immediately.
- **Data to log**: standard `--log` CSV; note pack ID and resting V.
- **Post-flight analysis**: plot x/y/z vs t; compute hold-window excursion and altitude
  band; compare wander character to ch07 Exp 1 (wander = drift/random walk is normal;
  a *constant parked offset* = bias — re-place the vehicle and recalibrate before
  blaming anything else; jitter = something mechanical, check props).

#### FT-1.2 — Extended hover, 30 s, drift measurement

- **Objective**: measure flow drift over a duration long enough for the random walk to
  show, and quantitatively qualify the floor/lighting combination (§2.2).
- **Prerequisites**: FT-1.1 ×2 passed.
- **Setup**: as FT-1.1.
- **Procedure**:
  1. `flight/fly_hover.py --dry-run` then
     `flight/fly_hover.py --height 0.5 --hold 30`.
  2. Fly once over the best floor patch; once over the worst patch you intend to use in
     FT-2 courses. Log both.
- **Pass criteria** (each flight):
  - Total horizontal drift over 30 s ≤ **0.20 m**, and not monotone (a straight-line
    march is bias/mis-mount, not drift). *Rationale*: flow drift is a random walk (ch07
    Exp 1), so expected excursion grows ~√t: scaling FT-1.1's 0.10 m/5 s allowance by
    √(30/5) ≈ 2.4 gives 0.24 m; the bar is rounded *down* to 0.20 m because a good
    textured floor should beat the allowance comfortably, and this card is the floor
    qualification.
  - Altitude 0.5 ± 0.10 m throughout the hold.
  - `vbat` at landing ≥ 3.5 V (a 30 s hover should barely dent a full pack; more
    droop flags a weak pack for the FT-0.3 table).
- **Abort**: standard; plus drift crossing 0.3 m from origin (halfway to the net margin).
- **Data to log**: standard `--log` CSV; floor patch identity in the log book.
- **Post-flight analysis**: drift trace plot (x–y plan view); drift distance vs √t
  sanity check; record the per-floor-patch drift numbers — the worse patch's number is
  an input to FT-2 course placement.

#### FT-1.3 — Hover height ladder

- **Objective**: map hold quality vs altitude, and fix the campaign's working ceiling.
- **Prerequisites**: FT-1.2 passed.
- **Setup**: as FT-1.1.
- **Procedure**: one 10 s hover per rung at 0.3, 0.6, 0.9, 1.2, 1.5 m
  (`--height H --hold 10`, dry-run each first). Land, let the pack rest 1 min, next
  rung. Stop climbing the ladder early if any rung fails.
- **Pass criteria**:
  - Every flown rung: horizontal wander ≤ **0.15 m**, altitude ±0.10 m.
    *Rationale*: flow position resolution degrades roughly linearly with height (the
    same floor texture subtends fewer pixels), so rungs above 0.5 m get FT-1.1's bar
    relaxed 1.5×; the ladder's product is the measured curve, the bar just catches
    breakage.
  - 0.3 m rung: expect the worst *altitude* behavior — ground effect adds ~10–15 %
    effective thrust and wobble near the floor (`docs/DESIGN.md` §9); pass needs only
    the same ±0.10 m band, but note the character.
  - Ladder completes to ≥ 1.2 m. 1.5 m is desirable, not required — the ceiling you
    *measure* becomes the campaign ceiling (never above 1.5 m per §2.1 regardless).
- **Abort**: standard; estimator health suspect (wander growing rung-over-rung,
  altitude glitches) → land, re-run `preflight.py` before deciding whether to fly the
  next rung.
- **Data to log**: standard `--log` CSV per rung.
- **Post-flight analysis**: wander-vs-height and variance-vs-height table → log book.
  This sets `--height` for all FT-2/3 cards (use the best-performing rung, typically
  0.5–0.8 m).

**GATE: FT-1 complete → trajectory flights may begin.**

---

### Phase FT-2 — trajectory following (conservative speed)

Sim twin: `demos/demo_minsnap.py` — sim target "RMS tracking < 0.08 m, max < 0.20 m" at
2 m/s average (`README.md` demo table). FT-2 flies the same *machinery* (min-snap →
poly4d → high-level commander, the `flight/README.md` Part C §5 pipeline) at one quarter
of the sim demo's speed, because "the sim demos' 2 m/s through a 4×3×2 m volume is
Lighthouse/Level-2 territory — scale it down" (`flight/README.md`). At 0.5 m/s the
dynamics are easy; what these cards actually measure is the estimator riding along a
moving trajectory. All flights use `yaw_mode="fixed"` (the `fly_minsnap.py` presets
already do).

#### FT-2.1 — Straight line, 0.8 m out and back

- **Objective**: first uploaded-trajectory flight; smallest possible step beyond hover.
- **Prerequisites**: FT-1 complete. Height from FT-1.3 (default 0.5 m assumed below).
- **Setup**: §2.8 checklist; course centered in the volume with ≥ 0.7 m net margin
  beyond the 0.8 m line plus 0.20 m error allowance; pad origin at the line's start.
- **Procedure**:
  1. `flight/fly_minsnap.py --preset line --height 0.5 --speed 0.5 --dry-run` — read the
     segment table; confirm total duration and that peak speed ≈ 2× the 0.5 m/s average
     (min-snap peaks ~2× average — comment at `cf_trajectory.MAX_SPEED`), well under the
     3.0 m/s validator limit.
  2. Fly: same command without `--dry-run`, with `--uri`, plus `--log
     out/logs/<...>.csv` and `--save-ref out/logs/<...>_ref.csv` (the `--save-ref`
     flag archives the flown poly4d rows — works for presets and `--csv` alike, so
     `analyze_log.py` always scores against exactly what flew). Script sequence:
     estimator reset → upload → arm → takeoff → `go_to` trajectory start →
     `start_trajectory` → land (`fly_minsnap.py`).
  3. Note the trajectory-start time from the script's console output — that is the
     `--t0` value `analyze_log.py` needs.
- **Pass criteria** (from `.venv/bin/python flight/analyze_log.py <log.csv> --ref
  <ref.csv> --t0 <seconds>`):
  - RMS position error ≤ **0.10 m**; max ≤ **0.20 m**.
    *Rationale*: the sim bar is 0.08 m RMS at 2 m/s with perfect state. At 0.5 m/s,
    dynamic tracking error is negligible; the budget is estimation error — the ch07
    bias analysis puts a plausible flow bias at the 50 mm class, so the RMS bar is the
    sim bar + a 25 % estimation allowance (0.10 m). The max bar stays at the sim's
    0.20 m because corners at 0.5 m/s are benign; exceeding it means something
    non-dynamic (drift burst, texture hole in the floor).
  - Both turnaround points visited within 0.15 m (plan-view check).
- **Abort**: standard; deviation from the line > 0.3 m.
- **Data to log**: standard `--log` CSV; reference CSV archived next to the log (§4.1).
- **Post-flight analysis**: `analyze_log.py` RMS/max; overlay plan-view plot of flown
  vs reference path; look for the DESIGN §9 fingerprints — steady lag along the motion
  direction is drag+motor lag (expected, small at this speed).

#### FT-2.2 — Square circuit at conservative speed

- **Objective**: multi-segment trajectory with four corners — the first real test of
  segment-boundary continuity on hardware.
- **Prerequisites**: FT-2.1 passed.
- **Setup**: `square` preset (0.6 m side — `PRESETS` in `fly_minsnap.py`); footprint
  0.6 × 0.6 m + 0.20 m allowance centered with ≥ 0.7 m net margin. Path length 2.4 m at
  0.5 m/s average ≈ 5 s + corner time.
- **Procedure**: dry-run then fly
  `flight/fly_minsnap.py --preset square --height 0.5 --speed 0.5` with `--log` and
  `--save-ref` per §4.1; repeat ×2 total.
- **Pass criteria** (each flight, `analyze_log.py`): RMS ≤ **0.10 m**, max ≤ **0.20 m**
  — same bars and rationale as FT-2.1; corners at 0.5 m/s add curvature but peak accel
  stays far below the 8 m/s² validator limit, so no relaxation is warranted. All four
  corners reached within 0.15 m of their waypoints.
- **Abort**: standard; corner overshoot heading toward the net.
- **Data to log**: standard `--log` CSV + reference CSV.
- **Post-flight analysis**: RMS/max via `analyze_log.py`; plan-view overlay — expect
  slight *inside* rounding of corners (drag/lag, DESIGN §9: fast segments "track inside
  the simulated path"); note its magnitude now to compare with FT-2.3.

#### FT-2.3 — Speed ramp on the square

- **Objective**: first controlled speed increase; measure how tracking degrades with
  speed on a fixed geometry.
- **Prerequisites**: FT-2.2 ×2 passed.
- **Setup**: as FT-2.2. One flight per speed step, pack ≥ 3.9 V at each takeoff so sag
  doesn't confound the speed trend.
- **Procedure**: fly the square at `--speed` 0.5, 0.75, 1.0 m/s in that order — dry-run
  each (the printed "Peak speed" line is your record of the actual peak; at avg 1.0 m/s
  expect ~2 m/s peak, still under the 3.0 m/s validator). Stop the ramp at the first
  failed step.
- **Pass criteria**:
  - 0.5 m/s step re-passes FT-2.2 bars (regression check).
  - 1.0 m/s step: RMS ≤ **0.15 m**, max ≤ **0.30 m**.
    *Rationale*: drag and motor lag are speed-dependent and unmodeled (DESIGN §9); the
    sim, with neither, holds 0.08 m at 2 m/s. Doubling speed from 0.5 → 1.0 m/s
    doubles the drag-induced lag, so the RMS bar gets +50 % over FT-2.2 (0.15 m) and
    max scales likewise. Larger degradation than that flags a tune/estimator problem
    rather than physics.
  - The RMS-vs-speed trend is monotone and documented (3 points).
- **Abort**: standard; any oscillation entering corners (motor-lag ringing signature —
  ch07 latency cliff) → land, do not fly the next step.
- **Data to log**: standard `--log` CSV per step + reference CSVs.
- **Post-flight analysis**: RMS/max per step; RMS-vs-speed table → log book. This curve
  is the phase's product and the FT-3.2 baseline.

**GATE: FT-2 complete → aggressive maneuvers may begin. Net mandatory from here
(it already was — re-verify it).**

---

### Phase FT-3 — aggressive maneuvers

Sim twin: `demos/demo_figure8.py` — sim target "RMS tracking < 0.10 m" with peaks near
2.5 m/s (`README.md`). FT-3 pushes toward the Flow deck's edge deliberately: `docs/
HARDWARE.md` Level 1 is explicit that aggressive trajectories are Level-2 (Lighthouse)
territory, so the *goal here is a measured degradation curve, not sim-matching numbers*.
Honest limitation, stated up front: if tracking at 2 m/s peak is unacceptable, that is
the Flow deck being itself — the fix is Level 2 hardware, not gain-chasing.

Binding stance: FT-3 **is flyable at Level 1, as characterization** — missing the bars
with a stable flight plus an anomaly report still satisfies the phase gate. The
full-speed sim-envelope flight (the 2 m/s demos through the 4×3×2 m volume) is deferred
to Level 2 (Lighthouse) and carries **no FT ID**.

#### FT-3.1 — Figure-8

- **Objective**: fly the project's signature maneuver, exported from `quadsim`, on real
  hardware.
- **Prerequisites**: FT-2 complete.
- **Setup**: build the trajectory in a scratch script per the `flight/README.md` Part C
  §5 pattern: figure-8 waypoints spanning 1.6 × 0.8 m (two-thirds of the sim demo's
  ~1.2 m half-width, to fit the §2.1 volume with margins), constant height from FT-1.3,
  `MinSnapTrajectory(..., avg_speed=1.0, yaw_mode="fixed")`, exported with
  `cf_trajectory.save_csv` — starting at (0, 0, height) because the EKF origin is the
  power-on point. Two laps (close the loop through the start waypoint twice).
- **Procedure**:
  1. `flight/fly_minsnap.py --csv out/fig8.csv --dry-run` — validator must pass; record
     printed peak speed/accel.
  2. Fly. Repeat ×2.
- **Pass criteria** (`analyze_log.py`, each flight): RMS ≤ **0.15 m**, max ≤ **0.30 m**.
  *Rationale*: matches the FT-2.3 1.0 m/s bars — same average speed, but continuous
  curvature replaces straight segments, which the min-snap/SE(3) machinery handles well
  in sim (0.10 m RMS at 2.5 m/s peak); the flow estimator under sustained lateral
  velocity is the unknown, and the FT-2.3-equivalent bar detects whether curvature per
  se costs anything extra. Lap 2 must not be systematically worse than lap 1 (drift
  accumulation check).
- **Abort**: standard; crossing-point miss > 0.4 m.
- **Data to log**: standard `--log` CSV + reference CSV.
- **Post-flight analysis**: RMS/max; plan-view overlay (expect the DESIGN §9 "inside"
  tracking on the lobes); lap-1 vs lap-2 error comparison. Optional after both flights
  pass: set `stabilizer.controller = 2` (Mellinger — the firmware twin of
  `quadsim/controller.py`, per `flight/README.md` Part C §5) and re-fly once; log the
  controller value with the flight and compare RMS. Revert to stock PID if worse.
- **Note on yaw**: stay `yaw_mode="fixed"`. Velocity-aligned yaw export exists
  (`cf_trajectory._yaw_segment_coeffs` least-squares fit, ≤ 0.05 rad error per its
  smoke test), but yaw is the weakest axis — `c_tau = 0.006 m` makes yaw torque ~5×
  weaker than roll/pitch per unit differential thrust (DESIGN §9) — so it is a
  deliberate extension *after* FT-3.2, not part of the gate.

#### FT-3.2 — Speed sweep toward 2 m/s peak

- **Objective**: find, on purpose and inside the net, where flow-based tracking
  degrades; produce the campaign's headline speed-vs-error curve.
- **Prerequisites**: FT-3.1 ×2 passed.
- **Setup**: FT-3.1 figure-8 geometry, rebuilt at increasing `avg_speed` so the
  dry-run-reported **peak** speed steps ≈ 1.0 → 1.5 → 2.0 m/s (peak ≈ 2× average, so
  avg ≈ 0.5/0.75/1.0 m/s; the validator's 3.0 m/s ceiling stays untouched). Fresh-ish
  pack (≥ 3.9 V) per step.
- **Procedure**: one flight per step, ascending; `analyze_log.py` verdict *between*
  steps — the next step flies only if the current one passed its bar and showed no
  oscillation. Stop the sweep at the first failure; the curve up to there is the result.
- **Pass criteria**:
  - 2.0 m/s peak step: RMS ≤ **0.20 m**, max ≤ **0.40 m**.
    *Rationale*: extrapolating the FT-2.3 relaxation pattern (+50 % RMS per doubling of
    speed from the 0.10 m base: 0.10 → 0.15 at 1.0 m/s avg → 0.20 at 2.0 m/s peak
    class), and consistent with expecting drag/lag — not instability — to dominate.
    Max at 2× RMS. These are *characterization* bars: missing them with a smooth,
    stable flight is a finding (Flow deck limit, → Level 2), not a campaign failure —
    but the phase gate requires either the bars met or the anomaly report written and
    the limit curve documented.
  - No sign of estimator divergence at any step (no position-estimate jumps in the
    `--log` trace; `preflight.py` re-passes between steps if anything looks off).
  - RMS-vs-peak-speed curve with ≥ 3 points delivered to the log book.
- **Abort**: standard; sustained oscillation (motor-lag/latency signature — the ch07
  Exp 2 cliff is sharp: ringing is the last warning before departure); any step whose
  dry-run peak exceeds 2.2 m/s (re-generate, don't "round up").
- **Data to log**: standard `--log` CSV + reference CSV per step; controller parameter value
  (stock PID vs Mellinger) held constant across the sweep.
- **Post-flight analysis**: RMS/max per step; final speed-vs-error curve plotted; annotate
  which DESIGN §9 mechanism each degradation feature matches (drag lag = smooth inside
  offset; motor lag = corner overshoot/ring; sag = late-flight altitude droop).

**GATE: FT-3 complete (bars met, or limit documented via anomaly report) →
endurance & robustness.**

---

### Phase FT-4 — endurance & robustness

#### FT-4.1 — Battery-sag endurance characterization

- **Objective**: measure real usable flight time and the sag signature, so every future
  session has a per-pack budget instead of a guess.
- **Prerequisites**: FT-3 complete (or FT-2 complete + FT-3 limit documented). FT-0.3
  battery table current.
- **Setup**: best pack from the FT-0.3 table, charged full (≥ 4.15 V resting); hover at
  the FT-1.3 best height; timer visible at the operator station.
- **Procedure**:
  1. Dry-run then fly `flight/fly_hover.py --height 0.5 --hold 600 --log
     out/logs/<...>.csv` — the 600 s hold is an upper bound; **the operator ends the
     flight with Ctrl-C**, which the script catches and turns into the normal land
     sequence (§2.6 layer 1 — the exact behavior drilled in FT-0.6). Watch `vbat` live
     by tailing the `--log` CSV in a second terminal (`tail -f`, last column) and press
     Ctrl-C when it reaches **3.2 V under load** (§2.4). Do not ride it to the
     firmware's low-battery behavior.
  2. Repeat with the second-best pack.
- **Pass criteria**:
  - Flight ends by the operator's Ctrl-C at the 3.2 V line, and the script's landing
    sequence completes normally — not by brownout, estimator failure, or floor contact.
    *Rationale*: the abort line exists to protect the pack (3.0 V hard floor + landing
    margin, §2.4); an uncommanded end means the procedure, not the pack, failed.
  - Altitude held within ±0.15 m until the final 30 s (relaxed from FT-1's ±0.10 m
    because late-flight sag droop is the *expected*, integral-term-absorbed behavior —
    ch07: "altitude sag late in the flight is battery sag, not the controller").
  - Deliverables recorded: hover endurance (min:s) per pack, `pm.vbat`-vs-time curve,
    time-of-onset of visible altitude droop. The measured endurance × 0.8 becomes the
    standing per-flight budget in the log book. (No pre-set minutes target — the repo
    gives no Crazyflie endurance number, and inventing one would violate house rules;
    this card *creates* the number.)
- **Abort**: standard (the 3.2 V rule *is* the plan here); altitude below 0.3 m
  uncommanded.
- **Data to log**: standard `--log` CSV for the whole flight (`vbat` rides along in
  every row); pack ID; ambient temperature note.
- **Post-flight analysis**: vbat curve annotated with droop onset; per-pack endurance →
  battery table; sanity-check the sag fraction against DESIGN §9's "max thrust falls
  ≈ 25 % from fresh to sagged" expectation (measured on the DIY propulsion, directionally
  applicable here).

#### FT-4.2 — Repeatability: three consecutive identical flights

- **Objective**: show the FT-2.2 result is a property of the system, not of one good
  flight — the precondition for ever trusting a comparison (gain change, controller
  swap, firmware update).
- **Prerequisites**: FT-4.1 (so the flight budget is known); same firmware, controller
  parameter, floor, and lighting as the FT-2.2 passes.
- **Setup**: FT-2.2 exactly — same preset, speed, height, pad origin, same pack charged
  to the same resting voltage class (≥ 4.0 V) for each flight.
- **Procedure**: three flights of
  `fly_minsnap.py --preset square --height 0.5 --speed 0.5` (each with its own `--log`
  file; one shared `--save-ref` reference), back-to-back in one
  session, 2-minute pack rest between (or rotate two qualified packs); no parameter,
  floor, or lighting changes between flights — that's the point.
- **Pass criteria**:
  - **3/3** flights individually meet the FT-2.2 bars (RMS ≤ 0.10 m, max ≤ 0.20 m).
  - RMS spread (max − min across the three) ≤ **0.05 m**. *Rationale*: half the RMS
    bar — if flight-to-flight scatter eats half the budget, no A/B comparison at this
    speed can resolve anything smaller, and the campaign's later conclusions would be
    noise. (Sim twin scatter is ~zero: `demo_minsnap.py` is deterministic; the spread
    you measure *is* the estimator + environment noise floor.)
  - Landing points all within 0.30 m of takeoff.
- **Abort**: standard. One aborted flight → the triplet restarts.
- **Data to log**: standard `--log` CSV ×3 + the single shared reference CSV.
- **Post-flight analysis**: `analyze_log.py` ×3; a three-row table (RMS, max, landing
  offset, vbat at start/end) → log book; overlay all three plan-view traces.

#### FT-4.3 — Parameter feedback: close the twin loop (props OFF)

- **Objective**: push measured reality back into `quadsim/params.py`, per the DESIGN §9
  closing instruction: "push the measured `m`, `f_motor_max`, and (via system ID) `J`
  back into `params.py` so the sim tracks the aging aircraft."
- **Prerequisites**: FT-4.1 and FT-4.2 complete (so the numbers being fed back come
  from a characterized, repeatable vehicle). Props off; bench card.
- **Setup**: 0.01 g pocket scale (the DESIGN §7 tools list: "weigh everything; the
  budget is a contract"); the campaign's log archive.
- **Procedure**:
  1. **Weigh** the flight configuration: Crazyflie 2.1+ + Flow deck + flight battery,
     exactly as flown. Record to 0.01 g.
  2. **Collect** the measured numbers: all-up mass; FT-4.1 endurance and droop onset per
     pack; FT-2.3/FT-3.2 RMS-vs-speed curves; hover altitude-hold character.
  3. **Update** `quadsim/params.py`: set `m` to the measured all-up mass (the sim's
     0.033 kg is the *DIY frame's* target — the Crazyflie's measured mass is what the
     twin of *this* campaign must carry). Leave `J`, `f_motor_max`, `c_tau` untouched
     unless you have a measurement (system ID for `J` is future work — do not adjust
     parameters you didn't measure).
  4. **Re-verify the sim**: `.venv/bin/python -m pytest` and
     `demos/demo_hover.py`, `demos/demo_minsnap.py`, `demos/demo_figure8.py` — all must
     still end `PASS` with the updated mass.
  5. **Document** a before/after table (parameter, old, new, source flight IDs) in the
     log book, and commit the `params.py` change with the table in the message.
- **Pass criteria**: measured mass recorded to 0.01 g; `params.m` updated; full pytest
  suite green; all three sim demos re-run `PASS`; before/after table written with a
  source flight ID for every changed number. *Rationale*: the twin discipline — every
  number in `params.py` traceable to a measurement or a derivation — is the project's
  house rule; this card is where the hardware starts holding up its end.
- **Abort**: n/a.
- **Data to log**: mass, table, commit hash → log book.
- **Post-analysis**: if the mass change moved any sim demo's metrics noticeably, note
  the deltas — that sensitivity is itself a finding for the next campaign iteration.

**Campaign complete.** The exit state: a qualified vehicle, a measured
speed-vs-error envelope, a battery budget, a demonstrated repeatability floor, and a
`params.py` that tells the truth. The natural next decision is the `docs/HARDWARE.md`
Level-2 gate question: is the Flow deck now the measured bottleneck?

---

## 4. Data management

### 4.1 Log file naming convention

All flight artifacts live under `out/logs/` (gitignored data; the log book and anomaly
reports are committed):

```
out/logs/<YYYYMMDD>_<FT-id>_f<nn>.csv          # flight-script --log output (t,x,y,z,vbat)
out/logs/<YYYYMMDD>_<FT-id>_f<nn>_ref.csv      # poly4d reference actually flown (--save-ref)
out/logs/<YYYYMMDD>_<FT-id>_f<nn>_notes.md     # free-form notes if the log-book row overflows
```

**Primary pipeline — every scripted flight.** Pass `--log <path>` to `fly_hover.py` /
`fly_minsnap.py`: the shared [`flight/flightlog.py`](../flight/flightlog.py) helper
streams `t,x,y,z,vbat` (`t` in seconds) to the CSV during the flight — already in
`analyze_log.py`'s input format, no reformatting. Trajectory flights also pass
`--save-ref <path>` so the *flown* poly4d rows are archived, even for presets:
regenerate-and-diff is not a provenance strategy. Scoring always runs on the archived
pair, never on a re-generated reference:

```
.venv/bin/python flight/analyze_log.py out/logs/<...>.csv --ref out/logs/<...>_ref.csv --t0 <seconds>
```

(hover cards use `--hover X Y Z` in place of `--ref`; `--t0` is the log time at which
trajectory time zero occurs, taken from the flight script's console output).

**Fallback pipeline — non-scripted sessions only.** cfclient GUI logging **cannot share
the radio link with a running flight script**, so it is only for manual/bench sessions
(FT-0.x hand checks, free exploration): export the cfclient CSV, rename the columns to
`t,x,y,z(,vbat)`, and convert the millisecond `Timestamp` to seconds before
`analyze_log.py` will accept it.

Example: `out/logs/20260812_FT-2.2_f02.csv` — second flight of FT-2.2 on 12 Aug 2026.
`f<nn>` counts flights of that card on that date, including aborts (aborts keep their
logs — aborted-flight data is often the most valuable).

### 4.2 Flight-log-book table (one row per flight — `docs/flight_logbook.md`, or a
spreadsheet with these exact columns)

| Date | FT id | Flight # | Log file | Pack ID | V start (rest) | V end (load) | Cmd (script + args) | Firmware | Controller (PID/Mellinger) | Height [m] | Speed avg/peak [m/s] | RMS [m] | Max [m] | Result (PASS/FAIL/ABORT) | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |

Plus two auxiliary tables in the same file:

- **Battery table** (from FT-0.3, updated by FT-4.1): pack ID, purchase date, resting V
  full, idle droop, measured endurance, status (active/retired).
- **Configuration table**: date, firmware version, cflib version, `stabilizer.controller`
  value, floor/lighting setup — one row per change, so any flight's full configuration is
  reconstructible from its date.

### 4.3 Anomaly / incident report template (one file per event —
`docs/anomalies/<YYYYMMDD>_<FT-id>_<slug>.md`)

```markdown
# Anomaly <YYYYMMDD> — <FT-id> — <one-line title>

- **Flight**: log file(s), log-book row(s)
- **Severity**: note / abort / crash / hardware damage / safety near-miss
- **What was commanded**: script + args, reference CSV
- **What happened**: factual timeline, times from the log (not from memory)
- **What the data shows**: the log evidence — variables, values, plots
- **Candidate causes**: each mapped, where possible, to a DESIGN §9 delta or ch07
  mechanism (drag, motor lag, sag, estimator bias/drift/latency, ground effect, yaw
  authority) — or "unknown", honestly
- **Ruled out**: what you checked that it wasn't
- **Action**: fix / procedure change / threshold change (with re-derivation) / accept
- **Re-test**: which card re-flies, and its result
```

Rule: a crash or safety near-miss always gets a report before the next powered test, even
props-off — and after any crash the §2.9 re-qualification checklist runs before that
test. Three same-card failures also trigger a report (§1.1). Solo corollary: the report
is written the same day — there is no debrief partner to keep the memory honest overnight.

---

## 5. Sim-to-real correlation

For each phase: which sim demo is the twin, and which `docs/DESIGN.md` §9 deltas (plus
ch07 mechanisms) are *expected* to show up — so a matching discrepancy is confirmation,
not alarm.

| Phase | Sim twin (target, `README.md`) | Expected discrepancy | Mechanism (source) |
| --- | --- | --- | --- |
| FT-1 hovers | `demo_hover.py` — settle < 2 cm, steady RMS < 0.005 m | cm-level slow wander and occasional parked offsets instead of mm-level hold; bars 0.10–0.20 m | Flow drift is a random walk; velocity bias parks the vehicle at e_p = −(k_v/k_p)b (0.1 m/s → 50 mm) — ch07 Exp 1; "sensor noise and state estimation", DESIGN §9. Ground-effect wobble on the 0.3 m rung — DESIGN §9. |
| FT-2 trajectories | `demo_minsnap.py` — RMS < 0.08 m, max < 0.20 m at 2 m/s avg | Flown at 0.5–1.0 m/s (flow comfort zone, `flight/README.md`); slight inside corner-rounding; bars 0.10–0.15 m RMS | Aerodynamic drag (unmodeled) + motor first-order lag → "track inside the simulated path" — DESIGN §9; speed scaled down per Level-1 guidance, `docs/HARDWARE.md`. |
| FT-3 aggressive | `demo_figure8.py` — RMS < 0.10 m, peak ~2.5 m/s | Degradation curve toward 2 m/s peak; possible flow-limit ceiling below sim speeds; lobes tracked inside; yaw kept fixed | Drag + motor lag (DESIGN §9); latency cliff sharpness (ch07 Exp 2: fine at 16 ms, gone by 40 ms) explains why degradation, when it comes, comes fast; yaw ~5× weaker per c_tau = 0.006 m (DESIGN §9). |
| FT-4.1 endurance | none directly — sim `f_motor_max` is a constant 0.16 N | Late-flight altitude droop, sluggish response near end of pack | Battery sag: max thrust falls ≈ 25 % fresh→sagged, absorbed by firmware integral terms the exact-model sim never needed — DESIGN §9; ch07 "battery sag and the missing integrator". |
| FT-4.2 repeatability | any sim demo — deterministic, scatter ≈ 0 | Nonzero flight-to-flight RMS spread (bar: ≤ 0.05 m) | The spread *is* the estimator + environment noise floor; sim has neither — DESIGN §9 "sensor noise and state estimation". |
| FT-4.3 feedback | `params.py` itself | Measured mass ≠ 0.033 kg (that number is the DIY frame's target) | The twin-maintenance loop: measure, update `params.py`, re-verify — DESIGN §9 closing paragraph. |

The reading discipline, campaign-wide (from `flight/README.md` Part C §6): **wander, not
jitter** is normal; **constant offsets are bias**, so recalibrate before retuning;
**late-flight altitude sag is the pack**, not the tune; **inside-tracking at speed is
drag/lag**, not a controller bug. When a discrepancy fits none of these fingerprints —
that is what the anomaly report (§4.3) is for.
