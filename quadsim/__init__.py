"""quadsim — agile micro-quadrotor simulation in the style of Vijay Kumar's GRASP Lab.

Minimum-snap trajectories (Mellinger & Kumar 2011), geometric SE(3) control (Lee, Leok,
McClamroch 2010), and swarm formation flight with optimal goal assignment.
"""
from __future__ import annotations

import importlib

__version__ = "0.1.0"

_EXPORTS = {
    "QuadParams": "quadsim.params",
    "QuadState": "quadsim.dynamics",
    "QuadrotorDynamics": "quadsim.dynamics",
    "FlatOutput": "quadsim.trajectory",
    "MinSnapTrajectory": "quadsim.trajectory",
    "hover_ref": "quadsim.trajectory",
    "SE3Controller": "quadsim.controller",
    "flat_to_rotation": "quadsim.controller",
    "History": "quadsim.sim",
    "simulate": "quadsim.sim",
    "FormationKeyframe": "quadsim.swarm",
    "SwarmSim": "quadsim.swarm",
}

__all__ = sorted(_EXPORTS) + ["__version__"]


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(importlib.import_module(_EXPORTS[name]), name)
    raise AttributeError(f"module 'quadsim' has no attribute {name!r}")
