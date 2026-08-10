# Cross-engine validation with MuJoCo

`quadsim` integrates its own 13-state rigid-body model with RK4. That code was written by
us — so how do we know the physics (and not just the controller) is right? The strongest
software-only evidence is the industry-standard **sim-to-sim check**: rebuild the *same
vehicle* inside an independently developed physics engine, fly the *same controller* on the
*same trajectory*, and compare. Agreement means any error would have to exist identically
in two engines written by different people with different methods — vanishingly unlikely.

[MuJoCo](https://mujoco.org) (maintained by Google DeepMind, the workhorse of modern
robotics research) is that second engine here. `quadsim/mujoco_bridge.py` builds a MuJoCo
model **from `QuadParams` itself** — same mass, inertia, rotor positions (`d = L/√2`),
drag-torque coefficient `c_tau`, per-motor thrust limits, gravity, RK4, zero drag — and
`demos/demo_mujoco.py` flies the unmodified `SE3Controller` in both worlds. The visual body
is the actual 3D-printed frame (`out/frame_binary.stl`), so the videos show *our* robot.

## Results (2026-08-10, MuJoCo 3.11.0)

Same gains, same reference, same 500 Hz control rate, two engines:

| Experiment | quadsim RMS | MuJoCo RMS | engine-vs-engine divergence |
| --- | --- | --- | --- |
| Hover recovery (0.15 m offset, settle RMS) | 0.47 mm | 0.47 mm | — |
| Figure-eight, 2 laps, 2.5 m/s peaks | 46.2 mm | 45.9 mm | **0.4 mm RMS / 2.5 mm max** |

The two engines land within a millimeter of each other over an aggressive 16-second
flight. The residual tracking error (~46 mm) is the *controller's* (it is the same in both
worlds — that is the point); the engines themselves agree ~100× more tightly.

## What the bridge deliberately does NOT hide

- **The catalog inertia is non-physical.** Any real rigid body satisfies
  `Jx + Jy ≥ Jz` (equals `2∫z²dm ≥ 0`). Our Crazyflie catalog values
  (`1.43+1.43 < 2.89 ×10⁻⁵`) violate it — they are identified, rounded numbers. MuJoCo
  refuses them outright; quadsim's integrator never noticed. The bridge applies the minimal
  projection `Jz ← Jx + Jy` (−1%, yaw only) to the MuJoCo plant while quadsim keeps the
  catalog values, making the comparison a small honest model-mismatch experiment too.
  Finding this was itself a validation payoff.
- **No aerodynamic drag in either engine** (matching assumptions, honestly matched).
  MuJoCo can add air density/viscosity with one XML attribute — a future realism knob.
- The MuJoCo world has a real floor with contact: a failing controller crashes, not
  falls through.

## Running it

MuJoCo needs a **native arm64 Python** on Apple-silicon Macs; the project's main `.venv`
is x86_64 (Rosetta), so a second venv lives beside it:

```bash
/opt/homebrew/bin/python3.12 -m venv .venv-mj
.venv-mj/bin/pip install -e ".[mujoco]" pytest
.venv-mj/bin/python demos/demo_mujoco.py            # full: 2 laps + MP4s
.venv-mj/bin/python demos/demo_mujoco.py --fast     # 1 lap, numbers only
.venv-mj/bin/python -m pytest tests/test_mujoco_bridge.py -v
```

Outputs land in `out/`: `mujoco_fig8.mp4` and `mujoco_hover.mp4` (the printed frame flying,
tracking camera), `mujoco_fig8_tracking.png`, `mujoco_divergence.png`.
The base venv's test suite simply skips the MuJoCo tests.

To fly it **live and interactively** (drag the camera while it flies), MuJoCo on macOS
requires its `mjpython` launcher:

```bash
.venv-mj/bin/mjpython -c "
import mujoco, mujoco.viewer, numpy as np
from quadsim.mujoco_bridge import MujocoQuadSim
from quadsim.controller import SE3Controller
from quadsim.dynamics import QuadState
from quadsim.params import QuadParams
from quadsim.trajectory import hover_ref
import time
params = QuadParams()
world = MujocoQuadSim(params, mesh_path='out/frame_binary.stl')
world.reset(QuadState.hover(np.array([0.2, -0.2, 0.4])))
ctl, ref, t = SE3Controller(params), hover_ref(np.array([0, 0, 1.0])), 0.0
with mujoco.viewer.launch_passive(world.model, world.data) as v:
    while v.is_running():
        f, tau = ctl.compute(world.state(), ref, t)
        world.apply(f, tau); world.step(0.002); t += 0.002
        v.sync(); time.sleep(0.002)
"
```

## Where this goes next

- **Realism knobs**: air density/viscosity, motor lag (first-order rotor dynamics),
  sensor noise + an estimator — each one narrows the sim-to-real gap the
  `docs/FLIGHT_TEST_PLAN.md` campaign will measure on the actual Crazyflie.
- **Gates**: drop `demo_minsnap.py`'s course into the MuJoCo world as visual geoms and
  render the obstacle run.
- **RL playground**: a MuJoCo model of our vehicle is exactly what `gymnasium`-style
  training pipelines consume, if that chapter ever calls.
