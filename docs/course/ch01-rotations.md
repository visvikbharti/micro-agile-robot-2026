# Chapter 1 — Rotations: quaternions are phasors, one dimension up

## After this chapter you can …

1. Build the unit quaternion for any axis-angle rotation and explain where the half-angle comes from.
2. Compose rotations by quaternion multiplication, apply them to vectors via `quat_to_rot`, and predict when order matters.
3. Demonstrate gimbal lock numerically — and explain why the simulator's state carries a quaternion instead of three Euler angles.
4. Read $\dot q = \tfrac{1}{2}\, q \otimes [0, \boldsymbol\omega]$ as the 3D version of the phasor derivative rule, and integrate it yourself.
5. Use `hat`/`vee` to move between vectors and skew-symmetric matrices, and linearize a small rotation as $R \approx I + \hat{\boldsymbol\theta}$.

## The ECE bridge

Every idea in this chapter is one you already own, promoted one dimension.

| You know (ECE) | This chapter |
|---|---|
| A phasor $e^{j\theta}$ is a *unit* complex number; multiplying phasors **adds phases** | A unit quaternion $q$ represents a 3D rotation; multiplying quaternions **composes rotations** (`quat_mul`) |
| $\frac{d}{dt}e^{j\theta} = (j\omega)\,e^{j\theta}$ — multiply by an imaginary number to rotate | $\dot q = \tfrac{1}{2}\, q \otimes [0,\boldsymbol\omega]$ — multiply by a *pure* quaternion to rotate (`quat_derivative`) |
| Scalar transfer functions commute: $H_1 H_2 = H_2 H_1$; MIMO (matrix) systems do **not** | 2D rotations commute (they're scalars in $\mathbb{C}$); 3D rotations do **not** — they're matrices |
| An oscillator needs amplitude control (AGC) or numerical drift grows | An integrated quaternion needs `quat_normalize` or $\lVert q\rVert$ drifts off 1 |
| $\operatorname{atan2}(0,0)$: phase of a zero-amplitude signal is undefined — a *coordinate* failure, not a physical one | Gimbal lock: at 90° pitch the Euler chart degenerates — the attitude is fine, its coordinates aren't |
| Writing convolution as a Toeplitz matrix: recasting an operation as a linear operator | `hat(v)` recasts the cross product $v\times{}$ as a matrix, so calculus and linear algebra apply |
| Small-signal linearization around a bias point | $R \approx I + \hat{\boldsymbol\theta}$ for small $\boldsymbol\theta$ — the small-angle model every attitude controller lives on |

## The theory

**Start in 2D, where you are at home.** A rotation of the plane by $\theta$ is multiplication by $z = e^{j\theta} = \cos\theta + j\sin\theta$. Two facts make phasors so pleasant: composing rotations is just multiplying, $e^{j\theta_1} e^{j\theta_2} = e^{j(\theta_1+\theta_2)}$, and the constraint "this is a pure rotation, no scaling" is simply $\lvert z \rvert = 1$. The set of unit complex numbers is a circle; rotations live *on* that circle, and calculus on it is easy: $\frac{d}{dt}e^{j\theta} = j\omega\,e^{j\theta}$, a velocity tangent to the circle, always perpendicular to the current phasor.

**3D rotations want the same treatment, but they are bigger.** A rotation in 3D has three degrees of freedom (an axis, two numbers, plus an angle). The honest representation is a $3\times 3$ rotation matrix $R$ with $R^\top R = I$ and $\det R = 1$ — nine numbers tied down by six constraints. Matrices compose correctly but are clumsy to integrate: numerically drift off the constraint and you get a matrix that *shears* your vehicle instead of rotating it.

**Quaternions are the 3D unit complex numbers.** A quaternion $q = [w, x, y, z]$ has one real part and three imaginary parts $(i, j, k)$ with $i^2 = j^2 = k^2 = ijk = -1$. Restrict to unit norm and you get exactly the phasor deal: a compact object where *multiplication composes rotations*. The rotation by angle $\theta$ about unit axis $\mathbf{n}$ is

$$q = \big[\cos\tfrac{\theta}{2},\; \sin\tfrac{\theta}{2}\,\mathbf{n}\big].$$

Why the half angle? Because a quaternion rotates a vector by the *sandwich* product $[0,\mathbf v'] = q \otimes [0,\mathbf v] \otimes q^{-1}$ — $q$ acts **twice**, once from each side, so each copy carries half the angle. One curious consequence: $q$ and $-q$ produce the same physical rotation (the "double cover"); you will verify this in the homework.

**Order matters.** In 2D, rotations commute because complex multiplication does. In 3D they don't: roll 20° then pitch 50° is a genuinely different attitude than pitch 50° then roll 20°. You have met this before — scalar transfer functions commute in cascade, but matrix (MIMO) systems don't. 3D rotation is inherently a matrix phenomenon, and quaternion multiplication faithfully inherits the non-commutativity.

**Why not Euler angles?** Three angles (yaw, pitch, roll) are the minimal parametrization, and minimal parametrizations of curved spaces always have a singular point — just as $\operatorname{atan2}(0,0)$ is undefined even though the signal is perfectly fine. For the aerospace `zyx` convention the bad point is pitch $= \pm 90°$: the yaw axis and the roll axis line up, one degree of freedom vanishes from the *coordinates* (not from the physics), and infinitely many angle triples describe one attitude. A quadrotor doing a flip passes through that point at speed. Quaternions have no singular point anywhere, which is why the simulator's 13-dimensional state vector carries $[p, v, q, \omega]$, not Euler angles.

**The derivative rule.** If the body spins with angular velocity $\boldsymbol\omega$ (expressed in the body frame), the attitude quaternion evolves as

$$\dot q = \tfrac{1}{2}\, q \otimes [0, \boldsymbol\omega].$$

Read it next to $\frac{d}{dt}e^{j\theta} = e^{j\theta} \cdot (j\omega)$: multiply the current orientation by a purely imaginary object built from the angular rate. The $\tfrac12$ is the half-angle again, now in differential form. And just as $j\omega e^{j\theta}$ is tangent to the unit circle, $\dot q$ is orthogonal to $q$ — the motion stays on the unit sphere in 4D *exactly*, in continuous time. Discrete integration steps off the sphere by a tiny amount each step, which is why `dynamics.step` renormalizes — the AGC of attitude integration.

**hat, vee, and small angles.** The map $\operatorname{hat}: \mathbb{R}^3 \to 3\times3$ skew-symmetric matrices, $\hat{\mathbf v}\,\mathbf u = \mathbf v \times \mathbf u$, turns the cross product into a linear operator. Its payoff is the small-angle model: for a small rotation vector $\boldsymbol\theta$,

$$R = e^{\hat{\boldsymbol\theta}} \approx I + \hat{\boldsymbol\theta},$$

the rotational sibling of $e^x \approx 1 + x$, and the small-signal linearization on which attitude control (Module 3) rests. `vee` is the inverse — it extracts the 3 independent numbers hiding in a skew-symmetric matrix. Keep that phrase in mind for the homework question about $e_R$.

## Guided walkthrough: `quadsim/maths.py`

Open `quadsim/maths.py` — 130 lines, the whole rotation toolkit. The module docstring fixes the conventions; misreading any of these is the classic source of sign bugs:

```python
"""Conventions (see SPEC): quaternions are ``[w, x, y, z]`` (Hamilton, unit norm, body->world);
rotation matrices map body to world; angular velocity is expressed in the body frame."""
```

`hat` is the cross product in a matrix costume — compare each row against the determinant formula for $\mathbf v \times \mathbf u$ you know:

```python
def hat(v: np.ndarray) -> np.ndarray:
    """Return the (3,3) skew-symmetric matrix such that ``hat(v) @ u == np.cross(v, u)``."""
    v = np.asarray(v, dtype=float)
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
```

`quat_mul` is the Hamilton product written out. The first line is a dot-product-like combination for the real part; the remaining three mix a scalar-times-vector part with a cross product — this cross product is precisely where non-commutativity enters:

```python
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        ...
```

`quat_to_rot` converts $q$ to a matrix, and hides a detail you will need for Homework 1:

```python
    w, x, y, z = np.asarray(q, dtype=float)
    s = 2.0 / (w * w + x * x + y * y + z * z)
```

The textbook formula uses $s = 2$ and assumes $\lVert q\rVert = 1$. Dividing by $\lVert q\rVert^2$ instead makes the conversion *self-normalizing*: feed it a slightly off-unit quaternion and it still returns a proper rotation matrix. Remember this.

`quat_derivative` is the phasor rule verbatim:

```python
def quat_derivative(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Return the (4,) quaternion time derivative ``0.5 * q * [0, wx, wy, wz]``."""
    w = np.asarray(w, dtype=float)
    return 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
```

Finally, `rot_to_quat` (Shepperd's method: branch on the largest of trace and diagonal so the square root and division are always well-conditioned — a numerical-robustness idiom worth stealing) and `rot_log`, which returns the axis-angle vector $\theta\,\mathbf n$ of a rotation matrix. `rot_log` is how the controller will later measure "how far apart are these two attitudes" as a single 3-vector.

Where does it plug in? `quadsim/dynamics.py` line 107 computes `q_dot = quat_derivative(q, w)` inside the state derivative, and `step` closes the loop after RK4 integration:

```python
        x_new = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        x_new[6:10] = quat_normalize(x_new[6:10])
```

## Experiments

These are the six live-session experiments (A–F). Run everything from the repo root, `/Users/vishalbharti/Downloads/micro-agile-robot-2026`. The outputs shown are real, captured on this machine.

**Experiment 1 (A + B) — the half-angle.** Warm up with phasors, then build quaternions:

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.maths import quat_mul
np.set_printoptions(precision=4, suppress=True)
z30, z45 = np.exp(1j*np.deg2rad(30)), np.exp(1j*np.deg2rad(45))
print("e^(j30) * e^(j45) angle:", round(np.rad2deg(np.angle(z30*z45)), 4), "deg")
def qrot(axis, deg):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
    th = np.deg2rad(deg)
    return np.concatenate([[np.cos(th/2)], np.sin(th/2)*axis])
print("q(z,30)           =", qrot([0,0,1], 30))
print("q(z,30) * q(z,45) =", quat_mul(qrot([0,0,1],30), qrot([0,0,1],45)))
print("q(z,75) directly  =", qrot([0,0,1], 75))
EOF
```

Observed output:

```
e^(j30) * e^(j45) angle: 75.0 deg
q(z,30)           = [0.9659 0.     0.     0.2588]
q(z,30) * q(z,45) = [0.7934 0.     0.     0.6088]
q(z,75) directly  = [0.7934 0.     0.     0.6088]
```

Multiplication added the angles — the phasor property survived the trip to 3D. **Break it:** in `qrot`, replace `th/2` with `th` (full angle) and add `from quadsim.maths import quat_to_rot`; then `quat_to_rot(qrot([0,0,1],45)) @ [1,0,0]` returns `[0. 1. 0.]` — a **90°** rotation from a "45°" quaternion. The sandwich product applied your angle twice. This is the single most common quaternion bug in the wild.

**Experiment 2 (C) — composition works; order doesn't commute.**

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.maths import quat_mul, quat_to_rot, rot_log
np.set_printoptions(precision=4, suppress=True)
def qrot(axis, deg):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
    th = np.deg2rad(deg)
    return np.concatenate([[np.cos(th/2)], np.sin(th/2)*axis])
q75 = quat_mul(qrot([0,0,1],30), qrot([0,0,1],45))
print("R(q75) @ [1,0,0] =", quat_to_rot(q75) @ np.array([1.0,0,0]))
qa, qb = qrot([1,0,0],20), qrot([0,1,0],50)
Rab, Rba = quat_to_rot(quat_mul(qa,qb)), quat_to_rot(quat_mul(qb,qa))
print("|R(qa qb) - R(qa)R(qb)|_max =", f"{np.abs(Rab - quat_to_rot(qa)@quat_to_rot(qb)).max():.2e}")
print("|R(qa qb) - R(qb qa)|_max   =", f"{np.abs(Rab - Rba).max():.3f}")
print("actual angle between the two orders:", round(np.rad2deg(np.linalg.norm(rot_log(Rab.T @ Rba))), 3), "deg")
EOF
```

Observed: `R(q75) @ [1,0,0] = [0.2588 0.9659 0. ]` (that is $[\cos 75°, \sin 75°, 0]$), the composition check is `1.11e-16` (machine epsilon — quaternion multiplication and matrix multiplication are the *same* operation in different clothes), and the swapped order differs by `0.262` in matrix entries — `16.834 deg` of genuine physical disagreement. Try it with a book on your desk: roll then pitch vs pitch then roll.

**Experiment 3 (D) — gimbal lock, and a lesson in trusting numbers.**

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from scipy.spatial.transform import Rotation as Rot
r1 = Rot.from_euler("zyx", [10.0, 89.9, 0.0], degrees=True)
r2 = Rot.from_euler("zyx", [-80.0, 89.9, 90.0], degrees=True)
print("difference:", round(np.rad2deg((r1.inv() * r2).magnitude()), 4), "deg")
EOF
```

Observed: `difference: 0.1414 deg`. Two *wildly* different angle triples — yaw 10° vs −80°, roll 0° vs 90° — describe essentially the same physical attitude. At 90° pitch, only the combination $\text{yaw} + \text{roll}$ survives (here $10 + 0 = -80 + 90$); the individual angles are meaningless. A confession from the live session: our first guess was that the invariant is $\text{yaw} - \text{roll}$, and the check with `[-80, 89.9, -90]` returned `179.9999 deg` — not "roughly equal," but *exactly upside down*. The number didn't negotiate; the narrative had the sign wrong. **Break it** yourself: rerun with `roll = -90` and watch the 180° appear. When a clean number contradicts your story, the number is doing you a favor.

**Experiment 4 (E) — integrate the derivative rule.**

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.maths import quat_derivative, quat_normalize, quat_to_rot
np.set_printoptions(precision=4, suppress=True)
q, w, dt = np.array([1.0,0,0,0]), np.array([0,0,np.deg2rad(90)]), 0.001
for _ in range(1000):
    q = quat_normalize(q + dt * quat_derivative(q, w))
print("after 1 s at 90 deg/s: R @ [1,0,0] =", quat_to_rot(q) @ np.array([1.0,0,0]))
EOF
```

Observed: `[0. 1. 0.]` — spin about z at 90°/s for one second and the body x-axis lands exactly on world y. **Break it:** replace `quat_derivative(q, w)` with `quat_mul(q, np.array([0.0, *w]))` (dropping the 0.5). The result becomes `[-1. 0. 0.]` — 180° instead of 90°. The half-angle factor is not a convention you can shrug off; it is load-bearing.

**Experiment 5 (F) — hat, cross, and the small-angle model.**

```sh
.venv/bin/python - <<'EOF'
import numpy as np
from quadsim.maths import hat, quat_to_rot
w, u = np.array([1.0,2,3]), np.array([0.5,-1,2.0])
print("hat(w) @ u =", hat(w) @ u, "  np.cross(w,u) =", np.cross(w,u))
th = np.deg2rad(0.1)
q = np.array([np.cos(th/2), 0, 0, np.sin(th/2)])
print("small-angle error:", f"{np.abs(quat_to_rot(q) - (np.eye(3)+hat([0,0,th]))).max():.2e}")
EOF
```

Observed: both products give `[ 7. -0.5 -2. ]`, and $R \approx I + \hat{\boldsymbol\theta}$ holds to `1.52e-06` at 0.1°. Raise the angle to 10° and then 90° to watch the linearization degrade — that decay curve is exactly why large-tilt maneuvers need the geometric controller of Module 3 rather than a small-angle PID.

**Experiment 6 — the whole session in one go.** The original live script is at `/private/tmp/claude-501/-Users-vishalbharti-Downloads-micro-agile-robot-2026/7406182a-5a7e-4e6b-bf26-9dff77fdca73/scratchpad/module1_rotations.py` (scratchpad copies are session-temporary, so the snippets above are the durable record). If it still exists, run it and diff its output against the numbers embedded here.

## Homework

**1.** In `quadsim/dynamics.py`, comment out the renormalization line in `step` (`x_new[6:10] = quat_normalize(x_new[6:10])`, line 124), run `.venv/bin/python demos/demo_figure8.py --fast --no-gif`, and record the RMS error. `docs/LEARNING.md` predicts the sim will "slowly explode." Explain what you actually observe, reconcile it with that prediction, and state under what conditions the missing normalization *would* bite. Restore the line afterwards.

> **Your answer:**



**2.** The attitude error in `quadsim/controller.py` line 120 is `e_R = 0.5 * vee(R_cmd.T @ R - R.T @ R_cmd)` — exactly 3 numbers, although a rotation matrix has 9 entries. Why is 3 the right count, and what role does `vee` play in getting there?

> **Your answer:**



**3.** Show that $\dot q = \tfrac12 q \otimes [0,\boldsymbol\omega]$ is always orthogonal to $q$ (numerically is fine: sample random unit $q$ and $\boldsymbol\omega$, check `np.dot(q, quat_derivative(q, w))`). What is the 2D phasor analogue of this fact, and what does it imply about $\frac{d}{dt}\lVert q \rVert$ in continuous time?

> **Your answer:**



**4.** Verify numerically that `quat_to_rot(q)` and `quat_to_rot(-q)` return the same matrix for several random unit quaternions. Why do $q$ and $-q$ encode the same rotation, and why does `rot_to_quat` in `quadsim/maths.py` force $w \ge 0$ before returning?

> **Your answer:**



**5.** Predict, *then* test: repeat Experiment 3 at pitch $-89.9°$ instead of $+89.9°$. Is the gimbal-lock invariant $\text{yaw}+\text{roll}$ or $\text{yaw}-\text{roll}$ there? Write your prediction down before running anything, and report whether the numbers agreed with you.

> **Your answer:**



## Hints

1. Re-read the `s = 2.0 / (w*w + x*x + y*y + z*z)` line in `quat_to_rot`, then ask: how fast does RK4 at $dt = 2$ ms actually drift the norm, and in what arithmetic (float32? seconds vs hours?) would that answer change?
2. Count degrees of freedom, not matrix entries; then check what kind of matrix $R_{cmd}^\top R - R^\top R_{cmd}$ always is.
3. $[0,\boldsymbol\omega]$ has zero real part; compare with $j\omega$ being purely imaginary, and recall what $\frac{d}{dt}\lVert q\rVert^2 = 2\,q^\top \dot q$ tells you.
4. Look at the sandwich $q \otimes [0,\mathbf v] \otimes q^{-1}$ and count how many sign flips cancel; the $w \ge 0$ choice is about returning *one* consistent representative of the pair.
5. Flipping the pitch sign flips which way the yaw and roll axes align at the singularity — expect the invariant's sign to flip too.

## Further reading

1. **Kuipers, *Quaternions and Rotation Sequences*** — written for engineers and builds from complex numbers upward, the same road this chapter took.
2. **Solà, *Quaternion kinematics for the error-state Kalman filter*** (arXiv:1711.02508) — the free reference PDF everyone keeps open; it will pay off again when Module 7 fuses IMU data.
3. **Ben Eater & 3Blue1Brown, *Visualizing quaternions*** (eater.net/quaternions) — interactive; the fastest way to *see* the double cover from Homework 4.
4. **Lee, Leok, McClamroch, *Geometric tracking control on SE(3)*** (CDC 2010), §2 only for now — the origin of `e_R`; you will read the rest in Module 3.
