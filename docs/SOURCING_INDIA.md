# Sourcing guide — Delhi / India

Where to actually buy everything for this project when you live in Delhi. Prices are
approximate (August 2026) and INR unless noted; always check current listings.

## The two paths, in India terms

### Path A — Crazyflie 2.1+ (recommended for the autonomy goals of this project)

The Crazyflie is the research-grade platform whose firmware runs the same Mellinger
controller as our `quadsim/controller.py`. Options, best first:

| Source | Notes |
|---|---|
| [Fab.to.Lab](https://www.fabtolab.com/bitcraze-crazyflie-2-1-plus) | **Indian distributor** of Bitcraze — no customs hassle, INR pricing. Check stock; email them if listed out of stock. |
| [Bitcraze store](https://store.bitcraze.io/products/crazyflie-2-1) (Sweden) | Direct, ~USD 250 for the kit; add shipping + Indian customs duty (~30-40% landed markup). Reliable but slower. |
| [RobotShop](https://www.robotshop.com/products/bitcraze-crazyflie-21) | International retailer, ships to India; compare landed cost with Bitcraze direct. |

Flow deck v2 (position hold) and Crazyradio come via the same channels — and per the
2026-08-06 autonomy decision (`docs/ARCHITECTURE.md` ADR-005) they are day-one items, not
add-laters: without the Flow deck there is no position estimate, and without the Crazyradio
there is no scripted flight.

### Path B — DIY whoop-class build (secondary, manual-flight track — the 3D-printed frame in `hardware/`)

Total ballpark for a first build: **₹6,000–12,000** (excluding a radio transmitter), per
[Zbotic's India whoop guide](https://zbotic.in/whoop-drone-for-indoor-fpv-best-tiny-whoops-in-india-2026/).

| Part (our spec) | Where in India | Approx price |
|---|---|---|
| 4× 7×16 mm coreless motors (2 CW + 2 CCW) | [Robu.in drone motors](https://robu.in/product-category/drone-parts/drone-motor/), [RoboSap](https://robosap.in/product/7-x-16mm-coreless-motor-with-propellercwccw/), [Indian Hobby Center](https://www.indianhobbycenter.com/products/3-7v-716-7x16mm-micro-core-less-motor-with-propeller-high-speed-mini-drones), [Amazon.in](https://www.amazon.in/India-Coreless-Helicopter-Propeller-Airplane/dp/B0D3LXVLCN) | ₹150–400 per motor; often bundled with props |
| Props ≤ 55 mm (CW/CCW pairs) | Usually bundled with motors; spares via [Robu.in propellers](https://robu.in/product-tag/propeller-for-coreless-small-motors/) | ₹50–150 per set |
| Brushed 1S AIO flight controller (F4, integrated ESCs + RX) | [Zbotic](https://zbotic.in/whoop-drone-for-indoor-fpv-best-tiny-whoops-in-india-2026/), QuadKart, Robu.in — look for "BetaFPV F4 1S brushed" / "Happymodel Beecore"-class boards | ₹1,500–3,000 |
| 1S LiPo/LiHV 260–350 mAh (PH2.0 connector) ×3 | Robu.in, Zbotic, QuadKart | ₹250–500 each |
| 1S USB charger (multi-port) | Same stores | ₹500–1,000 |
| Frame | **You print it**: `hardware/frame.stl` (~6 g of PLA, ≈₹10 of filament). No printer? Delhi has many 3D-printing services — search "3D printing service Delhi", or use a college makerspace/fab lab. | ₹100–300 printed |
| M2 screws, strap, spares | [ElectronicsComp](https://www.electronicscomp.com) (Delhi-based), Robu.in | ₹200–400 |

A note on brushed AIO boards: the hobby market has largely moved to brushless whoops, so
brushed FCs come and go from stock. Two good fallbacks: (a) buy a cheap brushed BNF whoop
(e.g., a Happymodel/Eachine 65 mm) and transplant its FC + motors onto our frame; (b) go
brushless later — that becomes a redesign exercise we can do together (new motor mounts in
`frame.scad`, heavier battery, updated `params.py` — the sim absorbs it in minutes).

**Delhi on-foot option:** Lajpat Rai Market (Old Delhi, opp. Red Fort) is the classic
electronics bazaar — fine for wires, chargers, batteries, tools; drone-specific parts are
more reliable online from the stores above.

## Radio and FPV (optional, Path B)

For autonomous work (our path) you do not need FPV goggles. A small radio transmitter
(BetaFPV LiteRadio / Jumper T-Lite class, ₹3,000–6,000) is worth having for manual practice
and as a safety override.

## Rules — the good news for us

Under India's [Drone Rules 2021](https://static.pib.gov.in/writereaddata/specificdocs/documents/2022/jan/doc202212810701.pdf),
our build sits in the **nano category (≤ 250 g)** — for recreational use it needs
[no registration, no UIN, and no pilot license](https://thinkrobotics.com/blogs/learn/drone-regulations-in-india-2025-what-you-need-to-know).
No-fly-zone restrictions still apply everywhere, and Delhi is heavily red-zoned (airport,
VIP areas) — but **indoor flying is unregulated**, and indoors-with-netting is exactly how
Kumar's lab flies in the video. Plan: fly indoors; check the
[Digital Sky map](https://digitalsky.dgca.gov.in) before any outdoor flight.

## Suggested first order (Path A — the chosen autonomy platform)

Per the binding 2026-08-06 decision (`docs/ARCHITECTURE.md` ADR-005), this is the order
that starts the `docs/FLIGHT_TEST_PLAN.md` campaign:

1. **Crazyflie 2.1+ kit** — mandatory day one
2. **Flow deck v2** — mandatory day one (the position estimate; nothing autonomous flies
   without it)
3. **Crazyradio** — mandatory day one (the scripted-flight link; `cflib` needs it)
4. 2–3 spare 250 mAh packs + a multi-port 1S charger (the flight-test plan's battery
   discipline — take off ≥ 3.9 V resting for test sessions, land by 3.2 V under load —
   chews through packs fast in a session)
5. ≥ 2 sets of spare props (bent props get replaced immediately, per the plan's §2.5)
6. Spare motors — optional on the first order; add them once FT-4.1 tells you your real
   flight-hour burn rate
7. **Second Crazyradio — recommended for a solo operator**: one dongle serves one
   process, so with a single radio the `flight/estop.py` panic button can only fire
   after the flight script is killed and releases it; a second dongle makes the
   emergency stop instant and independent (`docs/FLIGHT_TEST_PLAN.md` §2.6 layer 2)

**Landed cost, Fab.to.Lab vs Bitcraze direct** (estimates — all prices approximate, verify
against live listings):

- *Bitcraze direct (Sweden)*: the `docs/HARDWARE.md` Level-1 package is ~USD 250
  (Crazyflie 2.1+) + ~$65 (Flow deck v2) + ~$45 (Crazyradio) + ~$25 spares ≈ **$360–400**
  before shipping; with the ~30–40 % landed markup for shipping + Indian customs duty
  documented above, estimate **≈ $470–560 landed** — convert at the day's USD→INR rate
  when you order (neither this doc nor `docs/HARDWARE.md` commits an exchange rate, so no
  INR figure here would be honest).
- *Fab.to.Lab (Indian distributor)*: INR pricing with no customs or import hassle, which
  is exactly the 30–40 % markup you avoid — likely the cheaper *and* faster route when in
  stock. Their INR list prices are not recorded in this repo; check the current listing.

Lead times: confirm with the vendor before ordering — Fab.to.Lab stock comes and goes
(email them if listed out of stock, as noted above), and Bitcraze-direct adds
international shipping plus customs clearance time.

## Suggested first order (Path B — secondary/manual track, minimal)

1. 8× 7×16 motors (4 + 4 spares — coreless motors are consumables), with props
2. 1× brushed 1S AIO FC (or a donor BNF whoop)
3. 3× 1S 300 mAh + multi-charger
4. Print `hardware/frame.stl`
5. M2 screws + battery strap

≈ ₹5,000–8,000 to a flying airframe.
