# Chapter 6 — Mechanical design and CAD: the physical twin

Everything so far lived in numpy. This chapter is where the simulation grows a body: a
3D-printed frame, four brushed motors, and documents that promise the physical vehicle will
behave like `quadsim/params.py` says it does. Mechanical design is genuinely new territory
for you — so we lean hard on the fact that it runs exactly like an electronics project: one
datasheet, parameterized source files, design-rule checks, and budgets.

## After this chapter you can …

1. Trace any master dimension (33 g, $L = 46$ mm, 92 mm diagonal, 65.05 mm adjacent
   spacing, T/W ≈ 2.0) from `quadsim/params.py` through `hardware/frame.scad` to
   `out/frame_drawing.png`, and explain why `params.py` is the single source of truth.
2. Read and modify a parametric OpenSCAD model, re-render it, and export an STL — and say
   what its `assert()` lines protect you from.
3. Explain the four big design-for-printing rules (support-free geometry, layer-adhesion
   anisotropy, press-fit tolerances, stiffness-per-gram) and point at the exact feature in
   `frame.scad` that implements each one.
4. Read a dimensioned two-view engineering drawing: views, phantom lines, dimension chains,
   leaders, and the title block.
5. Audit the weight and thrust budgets in `docs/DESIGN.md`, recompute the margins yourself,
   and catch the places where the hardware documents disagree with each other.

## The ECE bridge

| New thing here | What you already know |
|---|---|
| `params.py` as the master for all hardware dims | The datasheet / interface control document. Board layout never contradicts the ICD; here, CAD never contradicts `params.py` |
| Parametric CAD (OpenSCAD: geometry as code) | A parameterized Verilog module: change one `parameter`, every derived dimension recomputes. The `frame.scad` parameter block *is* a parameter block |
| Derived values + `assert()` in the SCAD file | `localparam` plus DRC/ERC: computed constants and design-rule checks that fail the build, not the flight |
| STL export vs the `.scad` source | Gerbers vs the schematic: STL is a dumb triangle mesh you send to the fab (printer); the design intent lives only in the source |
| Press-fit bore = 7.0 + 0.1 mm tolerance | PCB drill sizing: hole diameter = lead diameter + plating/fit allowance. Print shrinkage plays the role of plating |
| FDM layer anisotropy (weak between layers) | A laminate: strong in-plane, delaminates between plies — same reason PCB prepreg fails between layers, not through copper |
| Weight budget table (§6 of DESIGN.md) | A power budget or RF link budget: every entry measured, subtotaled, margin declared, and treated as a contract |
| Thrust-to-weight = 2.0 | Amplifier headroom / PA back-off: hover is the average signal, T/W is how far below saturation you operate |
| CW/CCW motor + prop pairing matched to the mixer | Polarity conventions: swapping one differential pair inverts the signal; swapping one prop hand inverts a torque column |

## Core theory

### One number to rule them: why every dimension traces to params.py

The controller of Chapters 3–5 was tuned against a specific plant: $m = 0.033$ kg,
$L = 0.046$ m, $J = \mathrm{diag}(1.43, 1.43, 2.89)\times 10^{-5}$ kg·m², per-motor thrust
capped at $f_{\max} = 0.16$ N. If the physical vehicle drifts from these numbers, the sim's
predictions — hover throttle, tracking error, the speed at which `demo_minsnap` starts to
WARN — quietly stop applying. So the project adopts datasheet discipline: one file owns
each number; everything else *derives* from it and is *checked* against it.

The derivations are short enough to own. The X frame puts the four rotors at angles 45°,
135°, 225°, 315°, each at radius $L$ from the center. Opposite rotors are therefore
$2L = 92$ mm apart (the diagonal), and each rotor sits at body coordinates $(\pm d, \pm d)$
with $d = L/\sqrt{2} = 32.53$ mm — the very $d$ in the allocation matrix of
`quadsim/dynamics.py`, which uses `d = params.L / np.sqrt(2.0)` for its roll and pitch rows.
Adjacent rotors are $2d = 2L/\sqrt{2} = 65.05$ mm apart. That last number decides the
biggest prop you may fit: with no ducts, adjacent tips must never come closer than 10 mm, so
prop diameter is capped at $65.05 - 10 \approx 55$ mm. And the thrust-to-weight ratio is
pure `params.py`:

$$\frac{T}{W} = \frac{4 f_{\text{motor max}}}{mg} = \frac{4 \times 0.16}{0.033 \times 9.81} = 1.98 \approx 2.0$$

which is why every hardware document specs motors at ≥ 16.5 gf each — slightly above the
sim's 16.3 gf — so the mixer's software clip, not the motor, is always the binding limit.

### Parametric CAD versus direct modeling

There are two ways to make a 3D model. **Direct (history-based) modeling** — Fusion 360,
SolidWorks — is sculptural: you sketch, extrude, fillet, and the tool records a feature
history. It is interactive and great for organic shapes, but the design *intent* ("this
bore is motor diameter plus fit clearance") lives in your head unless you carefully attach
named parameters. **Parametric code-CAD** — OpenSCAD — is the opposite: the model is a
program. Geometry is built from primitives (`cube`, `cylinder`) combined with `union` and
`difference`, and every dimension is a named variable. Change `arm_length`, press render,
and the entire frame — arms, ribs, motor rings, assert checks — recomputes. For someone who
already thinks in parameterized HDL, code-CAD is the gentler entry, and it diffs cleanly in
git, which is why `frame.scad` is the master geometry. The agreed follow-up (Module 6,
LEARNING.md) is to later rebuild this frame in Fusion 360 *with frame.scad as the dimension
source* — worth it when we want STEP files (the exchange format that preserves real
surfaces, unlike STL's triangle soup) or nicer renders. Same relationship as schematic →
Gerber: you regenerate the export, you never edit it.

### Design for 3D printing

An FDM printer builds the part as a stack of ~0.2 mm molten plastic layers. Four
consequences drive the whole frame design.

**Overhangs.** Each layer needs something under it. Walls leaning past roughly 45° from
vertical droop and need sacrificial support material — ugly, weak, extra work. The frame
sidesteps this completely: it has a flat base at $z = 0$, every feature (arms, ribs, motor
rings) rises straight up from the bed, and every chamfer or slot is *cut from above*. It
prints support-free, flat side down.

**Layer adhesion.** The part is a laminate: strong along layers, weak between them (the
inter-layer bond is maybe half the bulk strength). So you orient the part so that flight
loads put layers in tension along their plane, never peeling them apart. Arms printed flat
mean bending loads from thrust are carried by continuous in-plane filament roads.

**Press fits.** Printed holes come out 0.1–0.2 mm undersized (the perimeter squishes
inward). The motor bore is therefore parametric — nominal can diameter plus an explicit
tolerance — and there is a chamfered lead-in so the motor starts straight. If your printer
runs tight, you change one number, not the geometry.

**Stiffness per gram.** The frame is ≈ 6 g of the 33 g budget — about 18 % — and it must be
*stiff*, not strong: flexy arms couple motor vibration into the IMU and wreck attitude
estimation. Bending stiffness of a thin section scales with thickness cubed, so the
cheapest stiffness is a tall thin rib on top of each arm (a poor man's I-beam), plus
lightening holes where material carries no load. This is why the model spends parameters on
`rib_height` and `lightening_hole_diam` rather than just making the arms fatter.

### The propulsion picture

The vehicle uses **7×16 mm brushed coreless motors** (7 mm can, 16 mm long, ≈ 3.2 g each)
driving ~55 mm press-fit props. At this scale brushed wins the *system* trade: the four
drive FETs live on the flight controller board, so there are no separate ESCs; the motors
cost ~$4 and are treated as consumables (brushes wear, thrust fades over hours). The price
is a slower thrust response — a first-order lag of 30–60 ms the sim doesn't model.

The part that must match the software exactly is *handedness*. In the mixer of
`quadsim/dynamics.py`, rotors 1 and 3 (the $+x{+}y$ / $-x{-}y$ diagonal) spin CCW seen from
above and contribute $-c$ to the yaw row; rotors 2 and 4 spin CW and contribute $+c$. So
the hardware needs 2 CCW + 2 CW motors *and* 2 CCW + 2 CW props, placed on the correct
arms. Brushed FETs drive one polarity only, so spin direction is fixed by which motor
variant you press into which ring — you cannot fix a swap in software. One prop of the
wrong hand flips the sign of one column of the allocation matrix: yaw torque and thrust
fight each other and the vehicle flips on takeoff. This is the mechanical version of
swapping one differential pair.

### Reading the drawing

Open `out/frame_drawing.png` (generated by `hardware/drawing.py`). It is a standard
two-view drawing. **Top view**: solid lines are real edges; dashed circles are *phantom
lines* — the swept prop discs, things that exist only in motion; the dotted square is the
FC footprint. Thin blue lines with arrowheads are *dimensions* (46, 65.05, 92, 147
tip-to-tip, 10.05 clearance); a *leader* with the Ø symbol calls out the 55 mm prop
diameter. Spin arrows and circled rotor numbers restate the mixer. **Side view**: a
dimension *chain* stacks the heights — 7 battery + 3 plate + 16 motor = 26 mm overall. The
**title block** at the bottom is the drawing's passport: title, date, units, sheet, rev,
and — crucially here — the statement `matches quadsim/params.py: m = 33 g, L = 46 mm`.

## Guided code walkthrough

Open `hardware/frame.scad`. The header declares its allegiance — "Geometry is BINDING to
the simulation model in quadsim/params.py" — and then comes the parameter block:

```scad
/* [Core X geometry] */
arm_length = 46;                    // frame center to motor axis (sim L = 0.046 m)
arm_angles = [45, 135, 225, 315];   // deg; index i = sim rotor i+1

/* [Motor press-fit rings — 7x16 mm brushed coreless] */
motor_diam        = 7.0;   // nominal motor can diameter
motor_fit_tol     = 0.1;   // diametral press-fit clearance added to the bore
motor_ring_height = 8;     // grip length on the 16 mm can (must be >= 8)
```

Every design decision from the theory section is a named number here (the `/* [...] */`
comments make them sliders in OpenSCAD's Customizer). Below the parameters come the
derived values — never hardcoded, exactly like `localparam`:

```scad
ring_id  = motor_diam + motor_fit_tol;          // press-fit bore
motor_pos = [for (a = arm_angles) arm_length * [cos(a), sin(a)]];
adjacent_spacing = norm(motor_pos[0] - motor_pos[1]);   // = arm_length*sqrt(2)
diagonal_spacing = norm(motor_pos[0] - motor_pos[2]);   // = 2*arm_length
```

and then the design-rule checks that fail the *render* if the geometry drifts:

```scad
assert(abs(adjacent_spacing - 65.05) <= 0.1, ...);
assert(abs(diagonal_spacing - 92) <= 0.1, ...);
assert(adjacent_spacing - check_prop_diam >= 10, ...);
```

The geometry itself is three small modules. `arm_solid()` is one arm laid along local $+x$
— a `cube` for the arm, a taller thin `cube` for the stiffening rib, a `cylinder` for the
motor ring — and `frame()` instantiates it four times with `rotate([0, 0, a])`, then
`difference()`s away the bores, chamfers, wire slots, FC holes, lightening holes, and the
engraved CW/CCW marker dots (1 dot = CCW rotor, 2 dots = CW). Note in `arm_cuts()` how the
lead-in chamfer is cut *downward from the ring top* — that is the support-free rule showing
up in code.

Now `docs/DESIGN.md`. Its §2 master-dimension table derives everything from the sim
("Arm length … **46.0 mm** … `params.L = 0.046 m`, directly"), and §6 is the weight budget:

| Component | Qty | Unit mass | Subtotal |
| --- | --- | --- | --- |
| 7×16 mm brushed motor | 4 | 3.2 g | 12.8 g |
| 55 mm prop | 4 | 0.4 g | 1.6 g |
| BetaFPV F4 1S brushed AIO FC | 1 | 3.0 g | 3.0 g |
| GNB 300 mAh 1S LiHV | 1 | 7.8 g | 7.8 g |
| Printed frame (PLA) | 1 | 6.0 g | 6.0 g |
| Screws, strap, tape | — | — | 0.9 g |
| **All-up total** | | | **32.1 g** |

Its §3 thrust budget puts the spec floor at 16.5 gf/motor → 66 gf total → **T/W 2.00** at
33 g, expected ≈ 19 gf fresh (T/W ≈ 2.3) sagging to ≈ 14–15 gf on a tired pack, with hover
demanding only 8.25 gf per motor (≈ 43 % throttle).

**A cautionary tale — real drift, since reconciled.** These documents were written at
different times and genuinely drifted before being brought back into agreement; the
audit trail is worth studying. Three examples of what the drift looked like: (a)
DESIGN.md §4's parameter table once listed `motor_cup_height = 10.0` and
`arm_width/arm_thickness = 5.5/3.0` while `frame.scad` actually said
`motor_ring_height = 8` and `7.5/2.8`, under different names; (b) the prop diameter was
55 mm in DESIGN.md, Ø51 in the drawing, and 40 mm in `hardware/README.md`'s BOM — three
documents, three props; (c) the all-up mass totaled 32.1 g (DESIGN.md), 33.0 g (drawing
notes), and 34.3 g (hardware/README.md). All four artifacts now agree — 55 mm Hubsan-class
prop, 32.1 g budget, `frame.scad`'s real parameter names — but this is exactly what
happens on real teams, and it is why the `assert()`s live in the geometry file rather
than in prose: only the executable checks never drifted.

## Experiments

Run everything from `/Users/vishalbharti/Downloads/micro-agile-robot-2026`.

**Experiment 1 — trace 46 mm end to end.** Find the number in all three layers, then verify
the derived dimensions:

```sh
grep -n "L: float" quadsim/params.py
grep -n "^arm_length" hardware/frame.scad
grep -n "^ARM_L" hardware/drawing.py
.venv/bin/python -c "
from quadsim.params import QuadParams
import math
p = QuadParams()
print('diagonal 2L      =', 2*p.L*1000, 'mm')
print('adjacent 2L/sqrt2=', round(2*p.L*1000/math.sqrt(2), 2), 'mm')
print('d = L/sqrt2      =', round(p.L*1000/math.sqrt(2), 2), 'mm')
print('T/W at 33 g      =', round(p.f_max/p.weight, 3))"
```

Observe: 46 appears once per layer, each annotated with a comment pointing back at
`params.py`; the derived numbers print 92.0, 65.05, 32.53, 1.977 — matching DESIGN.md §2.

**Experiment 2 — regenerate the drawing.** `.venv/bin/python hardware/drawing.py` rewrites
`out/frame_drawing.png` deterministically. Open it and find all five dimensions from
Experiment 1 on the sheet. **Break it:** edit `hardware/drawing.py`, change `PROP_D = 55.0`
to `PROP_D = 66.0`, and rerun. The drawing renders happily — adjacent prop discs now
overlap and the CLR callout reads a physically impossible −0.95 — with no error at all.
*Revert the edit.* Lesson: a drawing is a rendering, not a model; it will illustrate a
broken design as cheerfully as a good one. Only `frame.scad` carries `assert()`s.

**Experiment 3 — install OpenSCAD and export an STL.** Install (macOS):
`brew install --cask openscad`, or download from openscad.org (need ≥ 2019.05). Then:

```sh
/Applications/OpenSCAD-2021.01.app/Contents/MacOS/OpenSCAD -o out/frame.stl hardware/frame.scad
```

Observe the `ECHO:` lines: adjacent spacing 65.0538 mm, tip gap 10.05 mm, bore 7.1 mm —
the asserts all passed, and `out/frame.stl` is printable. In the GUI, open
`hardware/frame.scad`, use *Window → Customizer* to toggle `prop_guards = true`, press F6,
and watch ~5 g of guard rings appear, correctly spaced, without you drawing anything.

**Experiment 4 — break the geometry and let the DRC catch you.**

```sh
/Applications/OpenSCAD-2021.01.app/Contents/MacOS/OpenSCAD -D arm_length=50 -o out/frame_bad.stl hardware/frame.scad
```

The render *fails*: adjacent spacing becomes $50\sqrt{2} = 70.71$ mm, violating the
65.05 ± 0.1 assert, and OpenSCAD prints the message and refuses to emit an STL. Now try
`-D motor_fit_tol=0.3` — this one *succeeds* (bore 7.3 mm) because no assert covers it,
and you would only discover the sloppy fit with a motor in hand. Lesson: design rules only
protect what someone thought to encode — same as DRC on a board.

## Homework

1. Starting from $L = 46$ mm and arms at 45°/135°/225°/315°, derive both the 92 mm diagonal
   and the 65.05 mm adjacent spacing. Why does the allocation matrix in
   `quadsim/dynamics.py` use $d = L/\sqrt{2}$ in its roll/pitch rows instead of $L$?

> **Your answer:**
<br><br><br>

2. Diff DESIGN.md §4's parameter table against the actual parameter block in
   `hardware/frame.scad` and verify every name and value matches. Before the
   reconciliation it listed `motor_cup_height = 10.0` and `arm_width/arm_thickness =
   5.5/3.0` — which file should have won each dispute, and what one-sentence policy
   would prevent the drift from recurring?

> **Your answer:**
<br><br><br>

3. The spec prop is 55 mm; before the reconciliation the drawing said 51 mm and
   `hardware/README.md`'s BOM said 40 mm. Check all three against the ≥ 10 mm
   tip-clearance rule with 65.05 mm spacing. Which choices are legal? What physically
   happens with a 60 mm prop, and which line of `frame.scad` would catch it — under
   what condition?

> **Your answer:**
<br><br><br>

4. DESIGN.md claims T/W = 2.00 at the spec floor, 1.98 at the sim limit, and "≥ 1.83 even
   in the worst case" at 36 g. Recompute all three. Which thrust number does each claim
   use, and what do you get at 36 g using `f_motor_max` instead of the spec floor?

> **Your answer:**
<br><br><br>

5. The frame prints with zero support material. Identify three specific features in
   `frame.scad` (quote the module or parameter) that make this true, and explain why the
   arms-flat orientation is also the *strongest* orientation for flight loads.

> **Your answer:**
<br><br><br>

6. DESIGN.md §2 claims four 3.2 g motors at $d = 32.5$ mm contribute
   ≈ $1.35\times10^{-5}$ kg·m² to $I_{xx}$ and ≈ $2.7\times10^{-5}$ kg·m² to $I_{zz}$
   (at $r = 46$ mm). Verify both with the point-mass formula and compare against
   `params.J`. Why does the motor sit at radius $d$ for roll but $L$ for yaw?

> **Your answer:**
<br><br><br>

## Hints

1. Place the four rotors at $(\pm d, \pm d)$ and compute two distances; the torque about
   the $x$-axis depends on each rotor's *perpendicular* offset from that axis.
2. Compare parameter *names* first — several concepts were renamed — then values; think
   about which file is executable and which is prose.
3. Clearance = spacing − diameter. For the last part, look at `check_prop_diam` and ask
   what value it currently holds.
4. 16.5 gf and 0.16 N are not the same thrust; $1\,\mathrm{gf} = 9.81\,\mathrm{mN}$.
5. Search the SCAD comments for "support-free" and "cut from above"; for strength, recall
   which direction a laminate is weak.
6. A point mass $m$ at perpendicular distance $r$ from an axis contributes $mr^2$; project
   each rotor position onto the axis in question.

## Further reading

1. **OpenSCAD User Manual + Cheat Sheet** (openscad.org/documentation) — the whole
   language fits on one page; skim it before Experiment 3 and `frame.scad` reads like
   plain code.
2. **Prusa Knowledge Base, "Design rules for 3D printing"** — the practical numbers
   (overhang angles, hole shrinkage, minimum walls) behind every choice in §Design for
   3D printing, from people who print for a living.
3. **Kushleyev, Mellinger, Powers, Kumar, *Towards a swarm of agile micro quadrotors*
   (2013), §II** — the vehicle-design section of the reference stack: watch the same
   mass/inertia/agility budget logic play out on a real 73 g machine.
