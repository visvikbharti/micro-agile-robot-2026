# DESIGN.md — Physical design of the micro agile quadrotor

A buildable, 33-gram physical twin of the vehicle simulated in
[`quadsim/params.py`](../quadsim/params.py). Every load-bearing number in this document is
derived from, and checked against, the simulation parameters. All prices are **approximate
2026 USD** from memory — verify against live listings before ordering.

---

## 1. Design intent

This vehicle is the physical twin of the simulated quadrotor: every dimension and limit
traces to `quadsim/params.py`. The sim's `m = 0.033` kg is the all-up-weight target (36 g
hard maximum), `L = 0.046` m is the arm length from frame center to motor axis, and
`f_motor_max = 0.16` N (≈ 16.3 gf) is the per-motor thrust the hardware must meet or exceed
— we spec ≥ 16.5 gf per motor so the real vehicle can always deliver what the sim's mixer
commands. The rotor layout, spin directions, and numbering are copied verbatim from the
mixer in `quadsim/dynamics.py` (rotors 1,3 on the +x+y / −x−y diagonal spin CCW; rotors 2,4
spin CW), so a controller tuned in simulation maps onto the airframe without remapping. The
result is a whoop-class brushed 1S quadrotor — Crazyflie-sized, 3D-printed frame, off-the-shelf
motors, props, flight controller, and battery — that hits thrust-to-weight ≈ 2.0 at the
33 g simulated mass.

## 2. Master dimensions

| Dimension | Value | Derivation |
| --- | --- | --- |
| Arm length (frame center → motor axis) | **46.0 mm** | `params.L = 0.046 m`, directly. |
| Motor-to-motor diagonal | **92.0 mm** | 2 × L (opposite rotors are 2 arm lengths apart). |
| Rotor x/y offset from center | **32.53 mm** | d = L/√2 = 46/1.41421, the `d` used in the sim's allocation matrix. |
| Adjacent motor spacing | **65.05 mm** | 2d = 92/√2; distance between neighboring rotor axes. |
| Propeller diameter | **55 mm** | Largest standard hobby size satisfying the tip-clearance rule below. |
| Adjacent prop tip clearance | **10.05 mm** | 65.05 − 55 = 10.05 mm ≥ 10 mm required (no ducts, so tips must never meet). |
| Overall footprint (prop tips, square) | **120.1 × 120.1 mm** | 2 × (d + prop radius) = 2 × (32.53 + 27.5). |
| Tip-to-tip across the diagonal | **147 mm** | 92 mm diagonal + one prop diameter (2 × 27.5 mm). |
| Overall height (battery bottom → prop plane) | **≈ 29 mm** | 7 mm underslung battery (GNB 300 LiHV) + 3 mm arm deck + 16 mm motor + ≈ 3 mm shaft/prop hub. |
| All-up mass | **33 g target / 36 g max** | `params.m = 0.033 kg`; hard max gives the controller ≥ 1.83 T/W even in the worst case. |
| Per-motor max thrust | **≥ 16.5 gf (0.162 N)** | Must exceed `params.f_motor_max = 0.16 N` (16.3 gf) so the mixer's clip is the true limit. |

Consistency check against the sim's inertia: four 3.2 g motors at d = 32.5 mm contribute
≈ 4·(0.0032)·(0.0325)² ≈ 1.35 × 10⁻⁵ kg·m² to Ixx/Iyy and ≈ 2.7 × 10⁻⁵ kg·m² (at r = 46 mm)
to Izz — right on top of the sim's `J = diag(1.43e-5, 1.43e-5, 2.89e-5)`. The motors *are*
the inertia at this scale; keep everything else near the center.

## 3. Propulsion

### Motor — 7×16 mm brushed coreless, 19,000 rpm/V class

| Property | Value |
| --- | --- |
| Part | BetaFPV 7×16 mm 19000KV brushed motor set (2 CW + 2 CCW); equivalents: Happymodel / NewBeeDrone 0716 19000KV class |
| Size class | 7 mm can diameter × 16 mm can length, 1.0 mm shaft |
| Kv-equivalent | ≈ 19,000 rpm/V (no-load ≈ 75–80 krpm at 4.2 V; ≈ 35–45 krpm loaded with the 55 mm prop) |
| Mass | ≈ 3.2 g each including 55 mm leads and connector |
| Connector | 2-pin Molex PicoBlade 1.25 mm ("micro JST-1.25"), standard on whoop-class AIO boards |
| Wire convention | CW motors: blue/red wires. CCW motors: white/black wires. Verify with a props-off spin test — conventions occasionally vary by vendor. |

The 6×15 mm class (~2.1 g) cannot swing a 55 mm prop to 16.5 gf; the 8.5×20 mm class
(~5.5 g each) blows the weight budget. 7×16 mm is the sweet spot: it meets the thrust spec
with ≈ 15 % margin at ≈ 3.2 g per motor.

### Propeller — 55 mm 2-blade, press-fit

| Property | Value |
| --- | --- |
| Part | Hubsan X4 (H107) 55 mm 2-blade rotor set — sold as 4 × "A" + 4 × "B" (CW/CCW pairs); Ladybird-style 55 mm equivalents also fit |
| Diameter / pitch | 55 mm; shallow ≈ 25 mm-equivalent pitch (toy-class 2-blade) |
| Bore | 1.0 mm press-fit on the motor shaft |
| Mass | ≈ 0.4 g each |
| Pairing | 2 × CCW ("A") on rotors 1 and 3; 2 × CW ("B") on rotors 2 and 4 — see diagram below |

### Rotor numbering and spin direction (must match `quadsim/dynamics.py`)

Body frame: x forward, y left, z up. `d = L/√2 = 32.53 mm`. Top view (x up the page,
y to the left, props toward you):

```
                 +x (front)
                     ^
        R1 CCW       |       R4 CW
      (+d, +d)       |     (+d, -d)
      front-left     |     front-right
                     |
  +y <---------------+---------------> -y
  (left)             |             (right)
        R2 CW        |       R3 CCW
      (-d, +d)       |     (-d, -d)
      rear-left      |     rear-right
```

Rotors 1 and 3 (the +x+y / −x−y diagonal) spin **CCW** viewed from above (reaction torque
−z per unit thrust, the `−c` entries in the sim's allocation matrix with
`c = c_tau = 0.006 m`); rotors 2 and 4 spin **CW** (+z). Note this is the *opposite* of the
Betaflight factory default ("props out" rather than "props in") — handled entirely by which
motor variant goes on which arm, see §5 wiring.

### Thrust budget

Static thrust, 55 mm Hubsan-class prop, measured-class figures for a fresh 1S LiHV pack:

| Condition | Per motor | Total (×4) | T/W at 33 g |
| --- | --- | --- | --- |
| Spec floor (must meet) | 16.5 gf (0.162 N) | 66 gf (0.65 N) | **2.00** |
| Sim limit `f_motor_max` | 16.3 gf (0.160 N) | 65.2 gf (0.64 N) | 1.98 |
| Expected, fresh LiHV (4.35 V) | ≈ 19 gf | ≈ 76 gf | ≈ 2.3 |
| Expected, sagged pack (≈ 3.6 V under load) | ≈ 14–15 gf | ≈ 56–60 gf | ≈ 1.7–1.8 |
| Hover demand at 33 g | 8.25 gf | 33 gf | — |

Hover sits near 43 % of expected max thrust; full-throttle current is ≈ 2.5–3 A per motor
(≈ 10–12 A total, within a 30C 300 mAh pack's rating), hover ≈ 3–3.5 A total, giving
≈ 4–5 min gentle flight per pack.

### Why brushed coreless at this scale

Kumar's "small is agile" scaling: torque available to rotate the body scales with thrust ×
arm length (∝ L³·L = L⁴ at constant disc loading) while rotational inertia scales ∝ L⁵, so
achievable angular acceleration grows as ≈ 1/L as the vehicle shrinks — a palm-sized quad
snaps to attitude faster than any larger machine, which is exactly why this project is
micro-scale. At sub-50 g all-up mass, brushed coreless motors win the *system* trade:
no separate ESCs (four FETs on the AIO board replace ~4 g of brushless ESC hardware), no
bell/magnet mass, $4/motor, and adequate power density for 4-minute flights. The price is
finite brush life (thrust fades over ~hours of runtime — motors are consumables) and a
slower thrust response than brushless, which is listed honestly in §9.

## 4. Airframe

A single-piece 3D-printed X frame, defined parametrically in `hardware/frame.scad`. This
document matches that file's **defaults** — the load-bearing parameters and their default
values are:

| `frame.scad` parameter | Default | Meaning |
| --- | --- | --- |
| `arm_length` | 46 | Center to motor axis, mm — equals `params.L × 1000`. Do not change without changing the sim. |
| `motor_diam` | 7.0 | Motor can OD, mm (7×16 motor). |
| `motor_fit_tol` | 0.1 | Diametral press-fit clearance added to the ring bore, mm (bore = 7.1 mm). |
| `motor_ring_height` | 8 | Grip length of the motor ring on the 16 mm can, mm (asserted ≥ 8). |
| `motor_ring_wall` | 1.2 | Ring wall thickness, mm. |
| `arm_width` / `arm_thickness` | 7.5 / 2.8 | Arm cross-section, mm (plus a 2.0 × 1.2 mm stiffening rib on top). |
| `fc_hole_pitch` | 25.5 | FC mount hole square, mm (whoop 25.5 × 25.5 M2 standard). |
| `fc_hole_diam` | 2.1 | M2 clearance holes, mm (set 1.7–1.8 for self-tapping screws). |
| `strap_slot_length` / `strap_slot_width` / `strap_slot_spacing` | 12 / 3.2 / 16 | Battery strap slots through the plate, mm — the pack rides under the plate along x, held by a strap threaded down one slot and up the other (fits 300 mAh 1S). |
| `check_prop_diam` | 55 | Spec prop diameter, mm — used in the geometry assertion `adjacent_spacing - check_prop_diam >= 10`. |
| `guard_prop_diam` | 55 | Largest prop the optional guard envelope is sized for, mm (`prop_guards = false` by default). |

**Material: standard PLA** (first choice). At this size stiffness matters more than
toughness — flexy arms couple motor vibration into the IMU and wreck attitude estimation.
PLA is the stiffest common filament, prints thin walls accurately, and a 6 g frame is a
$0.50 consumable: crash it, reprint it. PETG is the alternate for crash resilience (bump
`arm_thickness` to 3.2, +≈0.6 g, to recover stiffness). Lightweight (foaming) LW-PLA is
*not* recommended: at 1.2 mm walls and 7.5 mm arms there is no section to foam, and it
loses too much strength. Print settings: 0.20 mm layers, 3 perimeters, 30 % gyroid infill (the
part is nearly all perimeter), arms flat on the bed.

**Target printed mass: 5.5–7 g; the defaults above print at ≈ 6.0 g in PLA.**

## 5. Electronics

### Flight controller — brushed whoop AIO

- **Primary: BetaFPV F4 1S Brushed Flight Controller** (Betaflight, integrated 4 × brushed
  FET ESCs, OSD, SPI 2.4 GHz receiver — ELRS or Frsky variant), 25.5 × 25.5 mm M2 mount
  pattern, ≈ 3.0 g. Runs Betaflight, so the sim-to-real story (rate/angle control, blackbox
  logs) is standard.
- **Alternate: BetaFPV Lite 1S Brushed FC** (Silverware/NFE firmware, ≈ 4.6 g, ~$25) —
  cheaper, heavier, simpler.

### Battery

- **GNB (Gaoneng) 300 mAh 1S LiHV (3.8 V), 30C/60C, PH2.0 connector**, ≈ 7.8 g,
  ≈ 60 × 12 × 7 mm — rides under the center plate along the x axis, held by a strap
  through the frame's two strap slots. The 260 mAh (≈ 6.9 g) and
  350 mAh (≈ 8.8 g) siblings bracket the allowed class; 300 mAh is the mass/endurance
  sweet spot. PH2.0 is the whoop-standard connector and matches the FC pigtail; avoid
  mixing with BT2.0 gear.

### Radio

- **ELRS 2.4 GHz** (SPI RX on the FC) with any ELRS transmitter — e.g., Radiomaster Pocket
  (~$65). Frsky D8/D16 FC variant if you already own a Frsky radio. Bind procedure per the
  FC manual; failsafe must be verified props-off before first flight.

### Wiring — FC pads to OUR rotor numbers

Betaflight numbers motors M1=rear-right, M2=front-right, M3=rear-left, M4=front-left. Our
convention (§3) is position-based from the sim mixer. The mapping, with required spin
directions and the matching motor variant:

| Our rotor (sim mixer) | Position | Spin (top view) | Motor variant / wires | FC pad (Betaflight) |
| --- | --- | --- | --- | --- |
| R1 | front-left (+d, +d) | CCW | CCW motor, white/black | **M4** |
| R2 | rear-left (−d, +d) | CW | CW motor, blue/red | **M3** |
| R3 | rear-right (−d, −d) | CCW | CCW motor, white/black | **M1** |
| R4 | front-right (+d, −d) | CW | CW motor, blue/red | **M2** |

Brushed FETs drive one polarity only, so spin direction is fixed by *which motor variant you
plug in*, not by software: on a CW (blue/red) motor, blue goes to the pad's + and red to −;
on a CCW (white/black) motor, white to + and black to −. If a motor spins backwards you
either plugged the wrong variant into that arm or the connector polarity is reversed —
fix the hardware, don't try to fix it in the mixer. Note the resulting rotation scheme is
Betaflight "props out" (front props' tops sweep away from the body), the opposite of the
Betaflight default diagram — trust the table above, not the default diagram.

## 6. Weight budget

| Component | Qty | Unit mass | Subtotal |
| --- | --- | --- | --- |
| 7×16 mm brushed motor (with lead + connector) | 4 | 3.2 g | 12.8 g |
| 55 mm prop | 4 | 0.4 g | 1.6 g |
| BetaFPV F4 1S brushed AIO FC | 1 | 3.0 g | 3.0 g |
| GNB 300 mAh 1S LiHV, PH2.0 | 1 | 7.8 g | 7.8 g |
| Printed frame (PLA, `frame.scad` defaults) | 1 | 6.0 g | 6.0 g |
| M2×6 nylon screws (4), battery strap, foam tape | — | — | 0.9 g |
| **All-up total** | | | **32.1 g** |

Margin: **+0.9 g under the 33 g sim target**, **+3.9 g under the 36 g hard max**. If the
build lands under 33 g, either add center ballast to match the sim exactly or (better)
weigh the finished vehicle and set `params.m` to the measured value; if it creeps over,
the first grams to reclaim are the frame (−0.5 g at 30 % infill) and direct-soldering the
motor leads (−0.2 g).

## 7. Bill of materials (approximate 2026 USD)

### Must-have

| Item | Qty | Approx. price |
| --- | --- | --- |
| BetaFPV 7×16 mm 19000KV brushed motors (4-pack, 2 CW + 2 CCW) | 1 | ~$16 |
| Hubsan X4 55 mm props (8-pack, 4A + 4B) | 1 | ~$4 |
| BetaFPV F4 1S Brushed FC, ELRS RX variant | 1 | ~$30 |
| GNB 300 mAh 1S LiHV, PH2.0 | 2 | ~$14 |
| ViFly WhoopStor V3 1S charger (PH2.0) | 1 | ~$28 |
| M2 nylon hardware kit + battery strap | 1 | ~$6 |
| PLA filament (frame + spares, share of spool) | — | ~$1 |
| Radiomaster Pocket ELRS transmitter (skip if you own an ELRS radio) | 1 | ~$65 |
| **Must-have subtotal** | | **~$164** (~$99 without radio) |

### Tools (if not owned)

| Item | Approx. price |
| --- | --- |
| Soldering iron (basic temperature-controlled) | ~$25 |
| Flush cutters + tweezers | ~$10 |
| 0.01 g pocket scale (weigh everything; the budget in §6 is a contract) | ~$15 |
| Safety glasses | ~$5 |

### Spares

| Item | Approx. price |
| --- | --- |
| Second motor 4-pack (brushed motors are consumables) | ~$16 |
| Second prop 8-pack | ~$4 |
| Third battery | ~$7 |

**Approximate grand total: ~$250 from zero; ~$130 if you already own a radio and tools.**
All prices approximate — check current listings.

## 8. Assembly sequence

1. Print the frame from `hardware/frame.scad` defaults; deburr the motor cups; confirm
   printed mass is 5.5–7 g on the scale.
2. Mount the FC to the center plate with 4 × M2×6 nylon screws, arrow/forward marking
   toward +x (the arm between rotors R1 and R4). Add a thin foam pad under the IMU corner
   if the board has one specified.
3. **Gotcha #1 — motor CW/CCW placement.** Press motors into the cups per the §5 table:
   white/black (CCW) motors into the front-left (R1) and rear-right (R3) cups; blue/red
   (CW) motors into the front-right (R4) and rear-left (R2) cups. Getting one pair swapped
   is the classic "flips instantly on takeoff" failure.
4. Route each lead along its arm to the mapped FC pad (R1→M4, R2→M3, R3→M1, R4→M2) and
   plug in (or cut and solder, saving ≈ 0.2 g). Observe polarity: blue/white to +.
5. Props off: bind the RX, verify failsafe, then use the configurator's motor tab to spin
   each motor at low throttle and confirm (a) the pad mapping matches the table and (b)
   spin direction viewed from above matches the §3 diagram (R1, R3 CCW; R2, R4 CW).
6. **Gotcha #2 — prop orientation.** These are all *puller* props mounted above the motors:
   every prop must blow air **downward**. Press "A" (CCW) props onto R1 and R3, "B" (CW)
   props onto R2 and R4, lettering/hub boss up. A prop of the wrong hand — or the right
   hand flipped upside-down — produces near-zero or negative thrust on that corner, which
   looks confusingly like a dead motor.
7. Strap the battery under the center plate (strap threaded down one slot and up the
   other), connect PH2.0, and check the center of gravity sits at the frame center
   (balance on a fingertip at the FC screws); slide the pack under its strap to trim.
8. Weigh the complete vehicle: it must be ≤ 36 g, target ≈ 32–33 g. Record the number.
9. First flight: props on, glasses on, over carpet, gentle hover at ≤ 0.5 m. Confirm hover
   throttle sits near mid-stick (≈ 43 % thrust per §3) — much higher means tired motors,
   a sagging pack, or excess mass.

## 9. Sim-to-real deltas (what `quadsim` ignores)

Honest list, with how each omission shows up in flight:

- **Aerodynamic drag.** The sim's only forces are gravity and rotor thrust. The real
  vehicle has body and rotor drag roughly linear in speed at this scale — expect steady
  position lag on fast trajectory segments and a natural terminal speed the sim doesn't
  have. Fast figure-eights will track "inside" the simulated path.
- **Motor/rotor lag.** Sim thrust is instantaneous; a brushed coreless motor plus prop
  spool-up behaves like a first-order lag with a time constant of roughly 30–60 ms. Sharp
  torque commands arrive late, which reads as attitude overshoot and forces lower rate
  gains than the sim tolerates.
- **Battery sag.** `f_motor_max` is a constant 0.16 N in the sim; in reality max thrust
  falls ≈ 25 % from a fresh LiHV to a sagged pack (§3 table). Late-flight behavior —
  sluggish climbs, mushy punch-outs — is the sag, not the tune. T/W ≈ 2.0 holds only on
  the spec floor with a healthy pack.
- **Brush and prop wear.** Brushed motors lose thrust over hours of runtime, and bent
  props add vibration that corrupts the IMU. Both drift the vehicle away from `params.py`
  over its life; refresh consumables before blaming the controller.
- **Sensor noise and state estimation.** The sim's controller consumes perfect position,
  velocity, attitude, and rates. The real FC has only an IMU: attitude and rates are
  estimated (noisily), and **position/velocity are not observable at all** — so the sim's
  position-tracking demos correspond to *pilot-in-the-loop* or externally-tracked flight,
  not something this board does autonomously. Closing the full loop needs optical flow or
  external tracking (see `docs/HARDWARE.md`, the Crazyflie path).
- **Yaw authority.** `c_tau = 0.006 m` means yaw torque is ~5× weaker than roll/pitch
  torque per unit differential thrust, and drag-based yaw is the slowest axis to respond;
  aggressive simulated yaw profiles will lag noticeably in reality.
- **Ground effect and prop wash.** Near-floor hover gains ~10–15 % effective thrust and
  gets wobbly; descending through your own wake causes the classic drop-and-tilt. The sim
  models neither — take off and land decisively.
- **Geometry and CG imperfection.** The sim's rotors are perfectly placed and the CG is
  exactly central; a millimeter of battery offset shows up as a constant trim the
  integrator (or your thumb) must absorb.

None of these break the twin — they define the gap the controller's robustness must cover,
and each is measurable: log hover throttle, step responses, and pack voltage, then push the
measured `m`, `f_motor_max`, and (via system ID) `J` back into `params.py` so the sim
tracks the aging aircraft.
