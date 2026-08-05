# Hardware: 3D-printable frame for the micro quadrotor

`frame.scad` is a fully parametric OpenSCAD model of an X-configuration micro
quad frame whose geometry and mass budget are **binding to the simulation** in
`quadsim/params.py`:

| Sim parameter (`params.py`) | Value | Hardware counterpart |
|---|---|---|
| `L = 0.046 m` | 46 mm | `arm_length = 46` (center to motor axis) |
| motor diagonal `2L` | 92 mm | adjacent motor spacing 92/√2 = **65.05 mm** |
| `m = 0.033 kg` | 33 g | all-up mass target (36 g hard max) |
| `f_motor_max = 0.16 N` | 16.3 gf | motors specified for **≥ 16.5 gf** each |
| mixer rotor order | 1..4 | arms at 45°/135°/225°/315°, see spin table |

The `.scad` file contains `assert()` checks that fail the render if the
geometry drifts: adjacent motor spacing must equal 65.05 ± 0.1 mm, and the
prop-tip gap with the spec 55 mm prop must be ≥ 10 mm (65.05 − 55 = 10.05 mm).

## Bill of materials (all-up mass budget)

| Part | Example (real, widely available) | Mass |
|---|---|---|
| Frame (this repo, printed PLA) | `frame.scad`, guards off | 6.0 g |
| 4 × brushed coreless motor, 7×16 mm | BetaFPV 7×16 19000KV (2 CW + 2 CCW wired sets); equivalents: Happymodel / NewBeeDrone 0716 19000KV class | 12.8 g |
| 4 × 55 mm 2-blade prop, 1.0 mm bore | Hubsan X4 (H107) 55 mm rotor set, 4 × "A" (CCW) + 4 × "B" (CW) | 1.6 g |
| Whoop AIO brushed FC (FC + 4 ESC + RX), 25.5 × 25.5 mm M2 pattern | BetaFPV F4 1S Brushed FC (SPI ELRS or Frsky); alternate: BetaFPV Lite 1S Brushed (Silverware, ~4.6 g) | 3.0 g |
| 1S 300 mAh LiHV battery | GNB 300 mAh HV (PH2.0) | 7.8 g |
| 4 × M2×6 nylon screws + nuts, battery strap, foam pad | generic | 0.9 g |
| **All-up mass** | | **32.1 g** |

32.1 g sits 0.9 g under the 33 g sim target with 3.9 g margin to the 36 g
hard max (matching the §6 weight budget in `docs/DESIGN.md`). If the finished
build lands under 33 g, either add center ballast or weigh it and set
`params.m` to the measured value; a 260 mAh pack (GNB 260, −0.9 g) is the
lighter in-class option.

**Thrust check:** 7×16 19000KV-class motors with the 55 mm Hubsan-class prop
on a fresh 1S HV cell bench at roughly 17–19 gf per motor — above the
required 16.5 gf (= sim `f_motor_max` 0.16 N plus margin), sagging toward
14–15 gf on a depleted pack. Total ≥ 66 gf at the spec floor gives
thrust-to-weight ≈ 2.0 at 33 g. Props larger than 55 mm are **not allowed**:
55 mm is the largest size that keeps adjacent tip clearance ≥ 10 mm
(65.05 − 55 = 10.05 mm).

## Rotor layout and spin directions (matches the sim mixer)

| Rotor | Arm angle | Position | Spin | Prop type | Frame marking |
|---|---|---|---|---|---|
| 1 | 45°  | +x +y | CCW | CCW prop | 1 engraved dot |
| 2 | 135° | −x +y | CW  | CW prop  | 2 engraved dots |
| 3 | 225° | −x −y | CCW | CCW prop | 1 engraved dot |
| 4 | 315° | +x −y | CW  | CW prop  | 2 engraved dots |

+x is "forward". In Betaflight, remap motor outputs so this table holds.
The resulting rotation scheme is Betaflight **"props out"** (front props'
tops sweep away from the body) — the opposite of the Betaflight factory
default "props in" — and the motor numbering also differs from the
Betaflight default order, so verify both in the Motors tab before first
flight (see the wiring table in `docs/DESIGN.md` §5).

## Rendering the STL

Requires OpenSCAD **2019.05 or newer** (uses `rotate_extrude(angle=...)` and
`assert()`). Download from <https://openscad.org/downloads.html>.

**GUI:** open `hardware/frame.scad`, press F6 (full render), then F7 or
*File → Export → Export as STL*. Toggle parameters (e.g. `prop_guards`) in
*Window → Customizer*.

**CLI one-liners (macOS app bundle path shown; plain `openscad` on Linux or
if it is on your PATH):**

```sh
/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD -o frame.stl hardware/frame.scad

# with prop guards (bench/indoor tuning only — adds ~5 g):
/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD -D prop_guards=true -o frame_guards.stl hardware/frame.scad
```

A successful render prints the derived values (`ECHO:` lines: 65.0538 mm
spacing, 10.05 mm tip gap with the 55 mm prop, 7.1 mm bore) and proves all
`assert()`s passed.

## Print settings

| Setting | Value |
|---|---|
| Material | PLA (stiffest per gram at this scale; PETG only if you need crash toughness and accept ~10% more flex) |
| Nozzle | 0.4 mm |
| Layer height | 0.16–0.20 mm |
| Perimeters / walls | 3 (arms and rings become effectively solid) |
| Top/bottom layers | 4 |
| Infill | 30% gyroid (only the center plate has any) |
| Supports | **None needed** — flat base, every feature rises from the bed, all chamfers cut from above |
| Brim | Not required; 3 mm brim optional on textured sheets |
| Orientation | As modeled: flat side down, motor rings pointing up |
| Bed footprint | ~75 × 75 mm (guards off), ~129 × 129 mm (guards on) |
| Expected mass | ~6 g (guards off), ~11 g (guards on) |
| Expected time | 35–55 min (guards off) |

Print one perimeter-calibration test first if your printer over-extrudes:
the press-fit bores are the critical dimension.

## Post-processing and assembly

1. **Motor rings:** the bore is parametric, `motor_diam + motor_fit_tol` =
   7.1 mm. Printed holes typically come out 0.1–0.2 mm undersized; if the
   motor will not press in by hand, run a **7.0 mm drill bit** (or step up
   from 6.8 mm) through the ring by hand — do not power-drill, PLA melts.
   The top chamfer is the insertion lead-in; push motors in from the top
   until flush. If a ring ends up loose after a repair, a drop of CA glue or
   a strip of tape around the can restores the fit. Motor wires exit through
   the vertical side slot.
2. **FC mounting (M2 tapping vs self-tapping):** as printed, the four holes
   on the 25.5 × 25.5 mm pattern are **2.1 mm clearance holes** — use M2×6
   nylon screws with nylon nuts under the arms (lightest, recommended, and
   the soft mount whoop FCs expect). If you prefer screws that bite the
   plastic, set `fc_hole_diam = 1.8` in the parameter block, re-render, and
   let the M2 screws self-tap; a proper M2 machine tap wants a 1.6 mm pilot
   and is overkill in a 2.8 mm plate. Deburr hole exits with a knife twist.
3. **Battery:** the pack (~60 × 12 × 7 mm) rides **under** the center plate
   along the x axis, held by a 10 mm hook-and-loop strap (or a wide rubber
   band) threaded down one slot and up the other. A 1 mm foam pad between
   pack and plate stops sliding.
4. **Check before first flight:** props clear each other by ≥ 10 mm (55 mm
   props), spin directions match the table above (dots: 1 = CCW, 2 = CW),
   and all-up mass on a scale is ≤ 36 g.
