"""Physical parameters of the simulated micro-quadrotor (Crazyflie-2-like)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class QuadParams:
    """Mass, inertia, geometry, and actuator limits of the quadrotor.

    Attributes:
        m: Mass, kg.
        J: (3,3) body-frame inertia matrix, kg m^2.
        L: Arm length from the body center to a rotor axis, m.
        c_tau: Rotor drag torque per unit thrust, m.
        g: Gravitational acceleration, m/s^2.
        f_motor_min: Minimum thrust per rotor, N.
        f_motor_max: Maximum thrust per rotor, N.
    """

    m: float = 0.033
    J: np.ndarray = field(default_factory=lambda: np.diag([1.43e-5, 1.43e-5, 2.89e-5]))
    L: float = 0.046
    c_tau: float = 0.006
    g: float = 9.81
    f_motor_min: float = 0.0
    f_motor_max: float = 0.16

    @property
    def f_max(self) -> float:
        """Maximum total thrust of all four rotors (= 4 * f_motor_max), N."""
        return 4.0 * self.f_motor_max

    @property
    def weight(self) -> float:
        """Weight force of the vehicle (= m * g), N."""
        return self.m * self.g
