# Agile Micro-Quadrotor: A Course for the ECE Engineer

An eight-chapter course that takes you from "why are small robots agile?" to a working
simulated swarm and a plan for real flight — built around the code in this repository and
written for an electrical/computer engineer: every robotics idea is introduced through a
concept you already own (phasors, feedback loops, filters, linear systems).

## How to use this course

Each chapter follows the same method: **read** the theory with the ECE bridge that connects
it to what you already know, **run** the referenced code and demos from the repository root
using the project's virtual environment, **break** something on purpose — change a gain, zero
a term, flip a sign — and watch what the simulator does, then **explain** the result in your
own words as if teaching it to a classmate. Don't skip the breaking step; it is where the
understanding actually forms. Work the chapters in order (each builds on the last), and write
your homework solutions and derivations in the combined Word document,
[`Quadrotor_Course.docx`](Quadrotor_Course.docx), which contains all eight chapters with a
table of contents and is meant to travel with you when you're away from the repo.

## Chapters

| # | Chapter | What you'll learn | Link |
|---|---------|-------------------|------|
| 0 | Why robots fly: the project and your toolkit | The "small is agile" scaling argument, the repository layout, and how to run every test and demo. | [ch00-introduction.md](ch00-introduction.md) |
| 1 | Rotations: quaternions are phasors, one dimension up | Build, compose, and integrate unit quaternions; see gimbal lock numerically and read q-dot as a 3D phasor rule. | [ch01-rotations.md](ch01-rotations.md) |
| 2 | Rigid-body dynamics: thirteen numbers and four knobs | The 13-state model, the mixer matrix from first principles, and why inertia scaling makes small vehicles agile. | [ch02-dynamics.md](ch02-dynamics.md) |
| 3 | Control: from the PID you know to geometry you'll love | Turn gains into natural frequencies and damping, then walk the full Lee SE(3) cascade term by term. | [ch03-control.md](ch03-control.md) |
| 4 | Minimum snap: why the motion looks graceful | Differential flatness, the snap-minimizing 7th-order spline, and the KKT system that solves it. | [ch04-trajectories.md](ch04-trajectories.md) |
| 5 | Swarms: nine robots, one dance | Virtual-leader formations, Hungarian slot assignment, C² transitions, and collision-avoidance bubbles. | [ch05-swarms.md](ch05-swarms.md) |
| 6 | Mechanical design and CAD: the physical twin | Parametric OpenSCAD from `params.py` as single source of truth, design-for-printing rules, and STL export. | [ch06-mechanical.md](ch06-mechanical.md) |
| 7 | The real flight stack: where ECE comes home | IMU estimation with complementary/Kalman filters, latency and bias effects, and the Crazyflie path to real flight. | [ch07-real-flight.md](ch07-real-flight.md) |

## The Word document

The complete course — all eight chapters, one file, with a table of contents — lives at
[`Quadrotor_Course.docx`](Quadrotor_Course.docx) in this directory. It is generated from the
markdown chapters with pandoc; the markdown files are the source of truth, so edit those and
regenerate rather than editing the .docx directly. Use the .docx for offline reading, for
printing, and as the place your homework answers go.
