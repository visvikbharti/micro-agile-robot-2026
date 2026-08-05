# From simulation to flight — a hardware roadmap

This repo simulates a Crazyflie-class micro-quadrotor on purpose: the
[Crazyflie 2.1+](https://www.bitcraze.io/) is an open-source, 30-ish-gram quadrotor whose
firmware ships the *same* Mellinger-style controller and polynomial-trajectory machinery
this simulation implements. That makes the jump from `quadsim` to a real robot unusually
short. This document lays out that jump as four levels, each one buildable on the last,
plus cheaper alternatives, a module-by-module mapping, prices, and safety notes.

**A note on prices**: all prices in this document are **approximate 2026 USD**, from memory
rather than live quotes. Treat every number as a ballpark and check the
[Bitcraze store](https://store.bitcraze.io/) or a regional distributor before budgeting.

---

## Level 0 — this repo (≈ $0)

You are here. Everything in the talk's first two acts — agile single-vehicle flight and
formation swarms — runs on your laptop:

- Run the demos, read the code, and break things. Change gains in `SE3Controller` and watch
  the tracking plots degrade or improve. Move the gates in `demo_minsnap.py`. Add a tenth
  robot to the swarm.
- The habits you build here (waypoint design, understanding tracking error plots, tuning
  intuition) transfer directly to the hardware levels below, where crashes cost real props.

Cost: your time. This level is worth exhausting before spending money.

## Level 1 — one Crazyflie, no external infrastructure (≈ $360)

**Hardware**: Crazyflie 2.1+ kit, Flow deck v2, Crazyradio 2.0 USB dongle, spare props and
batteries.

The Flow deck v2 points a small optical-flow camera and a laser ranger at the floor, giving
the Crazyflie onboard velocity and height estimates — position hold in a normal room with
**no external positioning system**. With the Python library
[`cflib`](https://github.com/bitcraze/crazyflie-lib-python) you script flights from the
same laptop that runs this sim:

- Use the **high-level commander** (`takeoff`, `land`, `go_to`, and
  `upload_trajectory`/`start_trajectory`) to fly waypoint sequences and uploaded polynomial
  trajectories.
- The Crazyflie firmware's trajectory format is piecewise 7th-order polynomials in
  `(x, y, z, yaw)` — the same representation `MinSnapTrajectory` produces. Exporting your
  simulated trajectories to the real robot is a coefficient-formatting exercise, not a
  research project.
- Select the firmware's **Mellinger controller** (a real-world sibling of
  `quadsim/controller.py`) via a parameter and compare real tracking against your simulated
  `History` plots.

Expectations: flow-based positioning drifts over long flights, wants a textured,
non-shiny floor and decent light, and is happiest below roughly 2 m altitude and at gentle
speeds. Fly conservative versions of your sim trajectories here; save the aggressive ones
for Level 2.

Approximate cost: ~$250 (Crazyflie 2.1+) + ~$65 (Flow deck v2) + ~$45 (Crazyradio 2.0),
plus ~$25 for spare props/batteries — call it **~$360–400, approximate**.

## Level 2 — Lighthouse positioning and aggressive trajectories (≈ +$450)

**Hardware added**: Lighthouse positioning deck, two SteamVR Base Station 2.0 units, mounts
(tripods or wall brackets).

In the TED talk, the robots fly inside a motion-capture (Vicon) net that tells each vehicle
where it is at millimeter precision. The **Lighthouse system** is the hobby-budget analog:
two SteamVR base stations sweep the room with infrared laser planes, and a deck on the
Crazyflie computes its own position onboard to sub-centimeter accuracy — lab-grade state
estimation for a few hundred dollars instead of tens of thousands.

With Lighthouse you can fly the things this sim actually demonstrates:

- The minimum-snap **obstacle course** and **figure-eight** demos at full speed, with
  repeatable tracking-error measurements to compare against `plot_tracking` output.
- Real gates: build hoops from foam pipe insulation and place them where your simulated
  `Gate` obstacles are.
- Aggressive maneuvers where the SE(3)/Mellinger controller earns its keep — large attitude
  angles, fast direction reversals.

Approximate cost: ~$100 (Lighthouse deck) + 2 × ~$160 (base stations) + ~$30 (mounts) —
**~$450 on top of Level 1, approximate**. Coverage is roughly a 5 × 5 m volume, more than
enough for the 4 × 3 × 2 m course volume used in `demo_minsnap.py`.

## Level 3 — the swarm: Crazyswarm2, 4–9 Crazyflies (≈ +$1,100–3,000)

**Hardware added**: 3–8 more Crazyflies, each with a Lighthouse deck; ideally a second
Crazyradio; a Linux machine (Ubuntu) running ROS 2.

[Crazyswarm2](https://imrclab.github.io/crazyswarm2/) is the ROS 2 stack the research
community uses to fly many Crazyflies at once — it handles multi-vehicle connections over
shared radios, broadcast commands, and synchronized trajectory execution. Your
`quadsim/swarm.py` logic ports over directly: compute formation offsets and assignments
offline (reuse `linear_sum_assignment` exactly as the sim does), generate one trajectory per
vehicle, upload, and trigger them in lockstep.

- Start with 4 vehicles (the same n=4 case as `tests/test_swarm.py`) before scaling to the
  talk's nine-robot finale replicated in `demo_swarm.py`.
- Keep the sim's discipline: check minimum pairwise distance *in simulation first* for every
  formation transition you plan to fly. `d_safe = 0.30 m` is a sensible floor indoors.
- Radio bandwidth is the practical ceiling: plan on one Crazyradio per ~4–5 vehicles.

Approximate cost: each additional swarm-ready Crazyflie is ~$250 + ~$100 (Lighthouse deck)
≈ **~$350 per vehicle, approximate**. Four vehicles total lands near ~$1,600 all-in;
nine near ~$3,600 — approximate, and very much a "grow gradually" purchase.

## Alternatives and detours

- **65 mm "tiny whoop" (~$40–120, approximate, plus ~$50–150 for a radio)** — a ducted-prop
  micro FPV quad. No position control and no scripting, but the best value per dollar for
  building *stick skills* and crash-proof intuition about how quadrotors behave. Flying one
  manually for a week will make every plot in this repo more meaningful. Safe indoors, very
  hard to hurt yourself or the furniture.
- **ESP-drone (~$40–70, approximate)** — an ESP32-based, Crazyflie-inspired open design
  flyable from a phone over Wi-Fi. A budget way to get a programmable micro-quad in the air,
  but with a smaller community, less capable positioning, and no Crazyswarm2 path. Fine as
  a toe in the water; expect to outgrow it quickly if the levels above appeal to you.

## Mapping this repo to the real stack

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

## Approximate price summary (2026 USD — all approximate)

| Item | Approx. price |
| --- | --- |
| Crazyflie 2.1+ kit | ~$250 |
| Crazyradio 2.0 | ~$45 |
| Flow deck v2 | ~$65 |
| Lighthouse positioning deck | ~$100 |
| SteamVR Base Station 2.0 (two needed) | ~$160 each |
| Spare props (set) / battery (each) | ~$5 / ~$10 |
| 65 mm whoop, bind-and-fly | ~$40–120 |
| Hobby radio transmitter | ~$50–150 |
| ESP-drone kit | ~$40–70 |
| Safety net / mesh enclosure materials | ~$50–150 |

Again: **approximate**, from memory, and subject to regional pricing, stock, and revisions —
verify against current store listings before ordering.

## Safety

Micro-quads are among the safest robots you can fly, but "safe" is earned, not assumed:

- **Props**: even tiny micro-quad props at full throttle can cut skin and are a genuine eye
  hazard.
  Wear safety glasses when flying toward yourself or debugging close-up. Never handle a
  powered vehicle; remove props (or use the firmware's motor-test with props off) for bench
  work. Replace bent props — vibration wrecks state estimation before it wrecks the frame.
- **LiPo batteries**: charge attended, on a non-flammable surface, ideally in a LiPo-safe
  bag. Never charge a puffed, punctured, or crash-damaged pack — retire it (discharge in
  salt water, then follow local disposal rules). Store at ~50% charge, not full. Don't fly
  packs below ~3.0 V/cell.
- **Netting, like in the video**: the TED talk's flying arena is wrapped in netting for a
  reason, and that lab flies better than you do. A cheap mesh curtain or garden net hung
  around your flight space protects spectators, windows, and the robot. Absolutely do this
  before Level 3 swarm flights, and before any aggressive Level 2 trajectory.
- **Indoors vs outdoors**: indoors, you set the rules — control the space, clear people and
  pets from the flight volume, and keep flights over a soft-ish floor. Outdoors, aviation
  law applies: rules vary by country and change over time (registration thresholds,
  altitude limits, distance from people and airfields, remote-ID requirements). A sub-250 g
  vehicle is treated leniently in many jurisdictions, but *you* are responsible for
  checking your local regulator's current rules (in the US, the FAA; elsewhere, your civil
  aviation authority) before any outdoor flight. When in doubt, stay inside the net.
- **Software kill switch**: wire up an emergency stop before your first scripted flight —
  `cflib` exposes one, and Crazyswarm2 has a swarm-wide emergency stop. Test it on the
  ground. First scripted flights: low, slow, one vehicle, cushion underneath.

Fly small, fly smart, and send the nine-robot video to a friend when you get there.
