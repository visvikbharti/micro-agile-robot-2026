# quadsim SPEC — binding interface contract

Micro agile quadrotor simulation in the style of Vijay Kumar's GRASP Lab (UPenn), after the TED
talk "Robots that fly ... and cooperate": agile micro-quadrotor dynamics, minimum-snap
trajectories (Mellinger & Kumar, ICRA 2011), geometric SE(3) tracking control (Lee, Leok,
McClamroch, CDC 2010), and swarm formation flight with optimal goal assignment (Kushleyev et
al. 2013; Turpin, Michael & Kumar).

Every module MUST follow this spec exactly: same names, signatures, units, frames, conventions.
Multiple authors write modules concurrently against this contract.

## Global conventions

- Python 3.9 compatible. Use `from __future__ import annotations` in every file; use
  `typing.Optional/List/Tuple/Callable` where needed. No `match` statements.
- Interpreter: `/Users/vishalbharti/Downloads/micro-agile-robot-2026/.venv/bin/python`.
- numpy float64 everywhere; vectors are shape `(3,)` ndarrays.
- World frame: z UP. `e3 = np.array([0.0, 0.0, 1.0])`. Gravity acceleration is `-g * e3`.
- Rotation `R ∈ SO(3)` maps BODY → WORLD: `v_world = R @ v_body`.
- Quaternion `q = [w, x, y, z]` (Hamilton convention, unit norm, body→world).
- Body z-axis `b3 = R @ e3` is the thrust axis; total thrust `f >= 0` produces force `f * b3`
  in the world frame.
- Angular velocity `w` (omega) is expressed in the BODY frame.
- SI units throughout: m, s, kg, N, rad.
- Library modules (`quadsim/*.py`) never print and hold no global mutable state. Any randomness
  uses `np.random.default_rng(0)`.
- Library import graph (strict): `maths` and `params` at the bottom; `dynamics` imports
  maths/params; `trajectory` imports maths; `controller` imports maths/params/trajectory;
  `sim` imports dynamics/controller/trajectory; `swarm` imports all of the above;
  `viz` may import matplotlib. Nothing in `quadsim/` imports from `demos/` or `tests/`.
- Allowed third-party deps: numpy, scipy, matplotlib, pillow (viz only). Nothing else.
- Type hints on all public functions; docstrings on all public API; line width <= 100.
- No dead code, no TODO comments.

## quadsim/maths.py

Module-level functions (exact names):

- `hat(v)` -> `(3,3)` skew-symmetric matrix with `hat(v) @ u == np.cross(v, u)`.
- `vee(M)` -> `(3,)` inverse of `hat` (use averaged off-diagonal entries).
- `quat_normalize(q)` -> `(4,)` unit quaternion (returns q/||q||; if w<0 keep sign as-is).
- `quat_mul(q1, q2)` -> `(4,)` Hamilton product.
- `quat_to_rot(q)` -> `(3,3)` rotation matrix (body→world) of a unit quaternion.
- `rot_to_quat(R)` -> `(4,)` with `w >= 0`; robust (Shepperd's method or equivalent).
- `quat_derivative(q, w)` -> `(4,)` = `0.5 * quat_mul(q, [0, wx, wy, wz])` (w in body frame).
- `rot_log(R)` -> `(3,)` axis-angle vector `theta * axis` (vee of the matrix log), numerically
  robust near angle 0 and near pi.

## quadsim/params.py

```python
@dataclass
class QuadParams:
    m: float = 0.033                 # kg (Crazyflie-2-like)
    J: np.ndarray = field(default_factory=lambda: np.diag([1.43e-5, 1.43e-5, 2.89e-5]))
    L: float = 0.046                 # m, arm length, center -> rotor axis
    c_tau: float = 0.006             # m, rotor drag torque per unit thrust
    g: float = 9.81
    f_motor_min: float = 0.0         # N per rotor
    f_motor_max: float = 0.16        # N per rotor (total 0.64 N, thrust/weight ~ 2.0)
```

Properties: `f_max` (= 4*f_motor_max), `weight` (= m*g).

## quadsim/dynamics.py

```python
@dataclass
class QuadState:
    p: np.ndarray   # (3,) position, world
    v: np.ndarray   # (3,) velocity, world
    q: np.ndarray   # (4,) quaternion [w,x,y,z], body->world
    w: np.ndarray   # (3,) angular velocity, body frame
```
- property `R` -> `(3,3)` via `quat_to_rot`.
- `classmethod hover(cls, p, yaw=0.0)` -> state at rest at `p` rotated by `yaw` about z.
- `as_vector()` -> `(13,)` in order [p, v, q, w]; `classmethod from_vector(x)`.

```python
class QuadrotorDynamics:
    def __init__(self, params: QuadParams): ...
```

Mixer geometry (X configuration), with `d = L / sqrt(2)` and `c = c_tau`:
rotor positions in body frame: `r1=(+d,+d,0), r2=(-d,+d,0), r3=(-d,-d,0), r4=(+d,-d,0)`;
rotors 1,3 spin CCW (reaction torque on body is -z per unit thrust), rotors 2,4 spin CW (+z).
Allocation matrix, `u = A @ motor_thrusts` with `u = [f, tau_x, tau_y, tau_z]`:

```
A = [[ 1,  1,  1,  1],
     [ d,  d, -d, -d],
     [-d,  d,  d, -d],
     [-c,  c, -c,  c]]
```

(derived from `tau = sum_i r_i x (f_i e3)` plus yaw drag — do NOT change any sign.)

- `mix(f, tau)` -> `(4,)` motor thrusts = `A^-1 @ u` then `np.clip` to
  `[f_motor_min, f_motor_max]`.
- `unmix(motors)` -> `(f, tau)` = `A @ motors` (returns float, `(3,)`).
- `derivative(x, f, tau)` -> `(13,)` state derivative of the 13-vector:
  `p' = v`; `v' = -g e3 + (f/m) R e3`; `q' = quat_derivative(q, w)`;
  `w' = J^-1 (tau - w x (J w))`.
- `step(state, f, tau, dt)` -> `QuadState`: apply actuator saturation ONCE via
  `f_act, tau_act = unmix(mix(f, tau))`, then integrate with classic RK4 holding
  `(f_act, tau_act)` constant, then `quat_normalize`.

## quadsim/trajectory.py

```python
@dataclass
class FlatOutput:
    pos: np.ndarray   # (3,)
    vel: np.ndarray   # (3,) default zeros
    acc: np.ndarray   # (3,) default zeros
    jerk: np.ndarray  # (3,) default zeros
    snap: np.ndarray  # (3,) default zeros
    yaw: float = 0.0
    yaw_rate: float = 0.0
```
(use `field(default_factory=...)` for array defaults)

- `hover_ref(p, yaw=0.0)` -> `Callable[[float], FlatOutput]` constant reference.

```python
class MinSnapTrajectory:
    def __init__(self, waypoints, avg_speed=2.0, segment_times=None,
                 yaw_mode="fixed", yaw_fixed=0.0): ...
```

- `waypoints`: `(M,3)` array, M >= 2. Consecutive duplicate waypoints are invalid.
- Each segment i is a 7th-order polynomial per axis in SCALED time
  `tau = (t - t_i) / T_i in [0,1]` (mandatory, for numerical conditioning).
- Objective: minimize `sum_i T_i^(-7) * integral_0^1 ||d4p/dtau4||^2 dtau` (minimum snap).
- Equality constraints: positions at all waypoints; vel = acc = jerk = 0 at the two trajectory
  endpoints; continuity of derivatives 1..4 at interior knots.
- Solve per axis via the KKT system `[[2H, A^T], [A, 0]] [c; lam] = [0; b]` with
  `np.linalg.solve` (fall back to `np.linalg.lstsq` if singular).
- Default segment times: `T_i = max(dist_i / avg_speed, 0.5)`.
- `T` property: total duration. `eval(t)` -> `FlatOutput`, clamping t to `[0, T]`, with correct
  physical-time derivative scaling (`d^k/dt^k = T_i^(-k) d^k/dtau^k`). `__call__ = eval`, so a
  trajectory instance IS a valid `ref_fn`.
- Yaw: `"fixed"` -> `yaw_fixed` always. `"velocity"` -> yaw follows `atan2(vy, vx)`:
  precompute a dense grid (>= 500 Hz) at init, valid where speed >= 0.15 m/s, hold the nearest
  valid value across low-speed spans, `np.unwrap` the angles, then `eval` interpolates
  (`np.interp`) and gets `yaw_rate` by finite difference on the grid.
- Cheap self-check at construction: max waypoint fit error < 1e-6 (raise ValueError otherwise).

## quadsim/controller.py

Module-level helper:

- `flat_to_rotation(F, yaw)` -> `(3,3)` desired rotation from a desired force vector `F` (world)
  and yaw: `b3 = F/||F||`; `b1c = [cos(yaw), sin(yaw), 0]`; `b2 = normalize(cross(b3, b1c))`;
  `b1 = cross(b2, b3)`; `R = column_stack([b1, b2, b3])`. Guard: if `||F|| < 1e-6` use
  `b3 = e3`; if `||cross(b3, b1c)|| < 1e-6` use `b1c = [cos(yaw+pi/2), sin(yaw+pi/2), 0]`.

```python
class SE3Controller:
    def __init__(self, params, kp=None, kv=None, kR=None, kw=None): ...
    def compute(self, state, ref_fn, t, fd_dt=1e-3): ...  # -> (f, tau)
```

Default gains, all `(3,)` arrays (integration may retune ONLY these defaults, nothing else):
`kp = m * [16, 16, 16]`, `kv = m * [8, 8, 8]`, `kR = [1000, 1000, 100]`, `kw = [63, 63, 20]`.

`compute` algorithm (Lee et al. 2010 geometric controller + finite-difference feedforward):

1. `ref = ref_fn(t)`; `e_p = p - ref.pos`; `e_v = v - ref.vel`.
2. `F_des = -kp*e_p - kv*e_v + m*g*e3 + m*ref.acc` (elementwise gains).
3. Thrust `f = max(0, F_des @ (R @ e3))`.
4. Commanded attitude `R_cmd = flat_to_rotation(F_des, ref.yaw)`.
5. Feedforward rotation as pure function of time:
   `R_ff(s) = flat_to_rotation(m*ref_fn(s).acc + m*g*e3, ref_fn(s).yaw)`.
   With `d = fd_dt`:
   `w_ff = rot_log(R_ff(t).T @ R_ff(t+d)) / d`
   `w_ff_m = rot_log(R_ff(t-d).T @ R_ff(t)) / d`
   `a_ff = (w_ff - w_ff_m) / d`   (angular acceleration feedforward, body frame)
6. `e_R = 0.5 * vee(R_cmd.T @ R - R.T @ R_cmd)`;
   `e_w = w - R.T @ R_cmd @ w_ff`.
7. `tau = J @ (-kR*e_R - kw*e_w) + cross(w, J@w)
        - J @ (hat(w) @ R.T @ R_cmd @ w_ff - R.T @ R_cmd @ a_ff)`.
8. Return `(f, tau)` — saturation is the dynamics' job, not the controller's.

## quadsim/sim.py

```python
@dataclass
class History:
    t: np.ndarray      # (N,)
    p: np.ndarray      # (N,3)
    v: np.ndarray      # (N,3)
    q: np.ndarray      # (N,4)
    w: np.ndarray      # (N,3)
    f: np.ndarray      # (N,)   actual (post-saturation) thrust
    tau: np.ndarray    # (N,3)  actual torque
    ref_p: np.ndarray  # (N,3)
    ref_v: np.ndarray  # (N,3)
```
- `pos_error()` -> `(N,)` of `||p - ref_p||`; `rms_pos_error(t_from=0.0)` -> float.

- `simulate(params, controller, ref_fn, state0, T, dt=0.002)` -> `History`. Loop: compute
  `(f, tau) = controller.compute(state, ref_fn, t)`, record the SATURATED values actually
  applied (recompute via dynamics `unmix(mix(...))`), step dynamics, advance.

## quadsim/swarm.py

```python
@dataclass
class FormationKeyframe:
    t_start: float        # when the transition to these offsets begins
    t_blend: float        # blend duration (smoothstep)
    offsets: np.ndarray   # (n,3) formation slot offsets relative to the leader
```

Blending uses smoothstep `s(u) = 6u^5 - 15u^4 + 10u^3` (with analytic `s'` and `s''` for
vel/acc of the offset).

```python
class SwarmSim:
    def __init__(self, params, leader, keyframes, d_safe=0.30, k_avoid=4.0,
                 a_avoid_max=3.0): ...
    def run(self, T, dt=0.004): ...   # -> List[History], one per quad
```

- `leader`: a `ref_fn` (`Callable[[float], FlatOutput]`); must be C^2 in time (it gets
  finite-differenced by the controller).
- `n` inferred from `keyframes[0].offsets`; first keyframe must have `t_start == 0`.
- Optimal slot assignment, resolved once at init, sequentially: for each subsequent keyframe,
  permute its offset rows via `scipy.optimize.linear_sum_assignment` on squared distances from
  the previous (already-permuted) offsets, so each robot flies to the nearest available slot —
  Kumar-lab "concurrent goal assignment".
- Per-quad reference at time t: leader flat output shifted by the blended offset (offset
  velocity/acceleration from smoothstep derivatives), PLUS collision-avoidance acceleration
  added to `.acc`, computed from CURRENT simulated states each step: for each pair with
  distance `d_ij < d_safe`, add `k_avoid * (1/d_ij - 1/d_safe)` along the separating unit
  vector (norm of total avoidance accel capped at `a_avoid_max`).
- Initial states: hover at `leader(0).pos + offsets_0[i]`.
- `run` steps all quads in lockstep with a per-quad `SE3Controller` and records a `History`
  per quad (`ref_p/ref_v` = that quad's formation reference WITHOUT the avoidance term).

Module helpers:
- `min_pairwise_distance(histories)` -> float, minimum over time of min pairwise distance.
- `formation_errors(histories)` -> `(N, n)` array of `||p_i - ref_p_i||` per time and quad.

## quadsim/viz.py

Obstacles (visual props only — trajectories avoid them by waypoint choice, no collision engine):

```python
@dataclass
class Box:      # axis-aligned cuboid
    center: np.ndarray  # (3,)
    size: np.ndarray    # (3,)

@dataclass
class Gate:     # rectangular frame to fly through; opening is perpendicular to `axis`
    center: np.ndarray  # (3,)
    width: float
    height: float
    axis: str = "x"     # "x" or "y": direction of flight through the gate
```

- `plot_trajectory_3d(histories, obstacles=(), waypoints=None, save=None, title="")` — 3D
  flight path(s), waypoint markers, obstacles; near-equal aspect; saves PNG if `save`.
- `plot_tracking(history, save=None, title="")` — 4 panels: xyz vs t (refs dashed); position
  error norm; thrust f with the `f_max` line; body rates.
- `animate_quads(histories, params, obstacles=(), save="out/anim.gif", fps=25,
  max_frames=400, trail=3.0, elev=25, azim=-60, title="")` — 3D animation: each quad drawn as
  two arm segments (rotated by R) + rotor dots + a fading trail of the last `trail` seconds;
  uniformly subsample so total frames <= max_frames; save GIF with `PillowWriter`; return the
  saved path.
- All functions must work headless (Agg backend); never call `plt.show()`.

## demos/ (four scripts)

Common structure for every demo:
- `sys.path` does NOT need patching (package is pip-installed editable).
- `matplotlib.use("Agg")` unless `--show` is passed (set backend BEFORE importing pyplot).
- `main(argv=None)` + argparse flags: `--out` (default `out/` under the project root),
  `--fast` (shortened run, roughly one-half to one-third length, skip GIF), `--no-gif`, `--show`.
- Print metrics as `name: value` lines; print `PASS` or `WARN <reason>` at the end; exit with
  code 1 only on gross failure (error > 5x target), else 0.
- Keep full-run wall time under ~90 s each; GIFs <= 400 frames.

1. `demos/demo_hover.py` — start 0.4 m offset from hover target (0,0,1.0); 6 s. Metrics:
   settle time to < 2 cm, steady-state RMS over the last 2 s (target < 0.005 m).
2. `demos/demo_minsnap.py` — obstacle course: ~6 waypoints threading two `Gate`s and skirting
   a `Box` (author designs a sensible course within a 4x3x2 m volume), `avg_speed=2.0`.
   Metrics: RMS and max tracking error (targets 0.08 / 0.20 m), total time, peak speed.
   Outputs: `minsnap_3d.png`, `minsnap_tracking.png`, `minsnap_course.gif`.
3. `demos/demo_figure8.py` — figure-eight (lemniscate, ~1.2 m half-width) waypoints, two laps,
   `avg_speed=2.5`, `yaw_mode="velocity"`. Metrics: RMS tracking (target 0.10 m), peak speed.
   Outputs: `figure8_3d.png`, `figure8_tracking.png`, `figure8.gif`.
4. `demos/demo_swarm.py` — n=9 (a nod to the talk's nine-robot finale), `dt=0.004`, ~30 s:
   3x3 grid (0.55 m spacing) at z=1.2 → transition to a 9-robot ring (r=0.9) → transition to a
   "V" formation → leader flies one smooth circle lap (r=0.7) with the V. Leader `ref_fn` must
   be C^2 (smoothstep-ramped analytic circle; hover before that). Metrics: steady formation RMS
   (target 0.06 m), max formation error, min pairwise distance (must stay > 0.15 m).
   Outputs: `swarm_3d.png`, `swarm.gif`. `--fast`: 10 s, grid → ring only.

## tests/ (pytest, five files, total wall time < 60 s)

- `tests/test_maths.py` — hat/vee roundtrip; `quat_mul` matches rotation composition;
  `quat_to_rot` orthonormal, det=1; `rot_to_quat` roundtrip on random rotations;
  `rot_log(scipy.linalg.expm(hat(v))) == v` for random small and near-pi v.
- `tests/test_dynamics.py` — hover equilibrium (`f = m*g`, `tau=0`): position drift < 1e-9
  over 1000 steps; free fall (f=0): `dz = -0.5*g*t^2` to rel err 1e-6; `unmix(mix(f, tau))`
  identity for feasible commands; hard saturation: requesting `f = 10*f_max` yields all motors
  at `f_motor_max`.
- `tests/test_trajectory.py` — waypoint interpolation <= 1e-5; endpoint vel/acc/jerk ~ 0;
  central finite-difference consistency of vel/acc/jerk vs pos (rel tol 1e-3, h=1e-4) at
  random interior times; pos/vel/acc continuity across knots; works with M=2; `T > 0`.
- `tests/test_controller.py` — `flat_to_rotation` returns valid rotations with b3 parallel to
  F; hover convergence: 0.5 m initial offset -> error < 1 cm within 4 s; circle tracking:
  analytic circle ref (r=1.0, omega=1.5 rad/s, z=1), state initialized on the circle with
  matching velocity, 8 s: RMS over last 4 s < 0.08 m.
- `tests/test_swarm.py` — assignment recovers a permutation optimally on a toy case;
  smoothstep blend is C^1 at endpoints; short run (n=4, 4 s, dt=0.004): terminal formation
  error < 0.08 m and `min_pairwise_distance > 0.12`.

## Docs

- `README.md` (root): what this is + link to the talk
  (https://www.youtube.com/watch?v=4ErEBkj_3PY); feature list; quickstart
  (`python3 -m venv .venv`, `.venv/bin/pip install -e ".[dev]"`, `pytest`, run demos); table of
  demos and their outputs in `out/`; a short theory primer (why small = agile: the r-scaling
  argument from the talk; differential flatness + minimum snap; SE(3) control; swarm/assignment);
  repo layout; references (Mellinger & Kumar 2011; Lee, Leok, McClamroch 2010; Kushleyev et al.
  2013; Turpin, Michael & Kumar 2012); pointer to `docs/HARDWARE.md`.
- `docs/HARDWARE.md`: sim-to-real roadmap. Level 0: this repo. Level 1: one Crazyflie 2.1+
  with Flow deck v2, flying these very trajectories via cflib's high-level commander (the CF
  firmware ships a Mellinger controller). Level 2: Lighthouse positioning (the lab-grade analog
  of the Vicon net in the video) + aggressive trajectories. Level 3: swarm via Crazyswarm2
  (ROS 2), 4–9 CFs. Alternatives: 65 mm whoop for stick skills; ESP-drone as a budget option.
  A mapping table sim module → real stack (trajectory.py → cflib trajectories, controller.py →
  firmware `controller_mellinger.c`, swarm.py → Crazyswarm2). Approximate 2026 USD prices,
  clearly marked approximate. Safety notes (props, LiPo handling, netting like in the video,
  indoor vs outdoor rules).
