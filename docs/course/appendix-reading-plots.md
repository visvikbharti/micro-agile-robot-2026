# Appendix A — Reading the plots: five checks for every flight

Every demo in this repo ends in the same four-panel figure (`quadsim/viz.py`'s
`plot_tracking`), and every *real* flight in the test campaign will end in the same
panels drawn from a log (`flight/analyze_log.py --plot`). That is deliberate: reading
this figure is a skill you use from your first simulation to your last flight test.
This page is the checklist. Run `.venv/bin/python demos/demo_hover.py --fast` and open
`out/hover_tracking.png` to follow along with real numbers.

The four panels, top-left to bottom-right: **position** (solid = actual, dashed =
reference), **position error norm** (one scalar: distance to the reference),
**total thrust** (the effort, with the `f_max` limit dashed red), and **body rates**
(the attitude loop's activity).

## The five checks

**1. Find reference vs actual — their gap is the whole story.**
Dashed is where the vehicle should be; solid is where it is. Everything else on the
page explains the gap between them. Hover demo: the gap starts at 0.40 m
(= √(0.23² + 0.23² + 0.23²), the three initial offsets combined) and ends below a
millimeter. In a real-flight plot the "actual" line is itself an *estimate* — keep
ch07's grain of salt.

**2. Overshoot and ringing — read the damping.**
Does the solid line cross the dashed one and come back? That is overshoot:
underdamping. Does the error oscillate as it decays? Ringing. Our default gains place
both loops at critical damping (ζ = 1, ch03), so healthy plots show **monotone**
convergence — no crossing, no ripple. A brief, sharp spike in *body rates* at the
start is normal (that's the tilt that moves the vehicle, ~±5 rad/s in the hover
demo); **sustained oscillation** in body rates means the attitude loop is fighting
itself — ch03's starve-kR experiment shows exactly that signature.

**3. Settle time vs prediction — the design audit.**
Before looking, predict: a critically damped loop settles in roughly 4–6/ωₙ. Position
loop: ωₙ = √(kp/m) = √16 = 4 rad/s → predicted 1.0–1.5 s. Measured by the demo:
`settle_time_s: 1.182`. If the measured value is far slower than predicted, something
is stealing authority — usually saturation (check 4) or a missing feedforward (ch03
Experiment 3).

**4. Actuator vs limits — hunt for saturation.**
The thrust panel's dashed red line is `f_max` = 4 × 0.16 = 0.64 N. The command
touching that line means the controller asked for more than the motors have; tracking
then degrades *silently* — no error message, just a growing gap in panel 1. The hover
demo cruises near half throttle (T/W ≈ 2, ch02); the aggressive figure-8 demo is
where this check earns its keep. On real logs the same check applies to battery sag:
a "sagged" pack lowers the true ceiling ~25 % (`docs/DESIGN.md` §9).

**5. Steady-state physics audit — do the numbers land where physics says?**
After settling: thrust must equal weight — m·g = 0.033 × 9.81 = **0.324 N** — and the
hover demo's thrust trace flattens exactly there. Body rates must go to zero. Error
must flatten at ~0 in simulation (`steady_rms_m: 0.00074`); in real flight a
**constant** error offset is the estimator-bias fingerprint e_p = −(k_v/k_p)·b from
ch07 Experiment 1 — a 0.1 m/s velocity bias parks the vehicle 50 mm off target,
forever. Flat-but-not-zero is a *diagnosis*, not a mystery.

## The same panels, elsewhere

| Where | What changes |
|---|---|
| `demo_minsnap.py`, `demo_figure8.py` | Reference is a moving curve, not a point; error stays nonzero *during* the maneuver (RMS targets: 0.08 m / 0.10 m), and check 4 matters most at the corners. |
| `demo_swarm.py`, `demo_decentralized.py` | One vehicle's panels per quad plus formation metrics; add ch05/ch08's min-pairwise-distance check to the list. |
| `flight/analyze_log.py --plot` | Same layout from a real log. Expect centimeters where sim shows millimeters (Flow-deck reality, `docs/ESTIMATION.md`), and run the five checks *in order* — they are the post-flight analysis the flight-test cards ask for. |

Five checks, one habit: reference vs actual, damping, settle time vs prediction,
saturation, physics audit. If all five pass, the flight was healthy; whichever one
fails names the chapter that explains it.

---

*[Course index](README.md)*
