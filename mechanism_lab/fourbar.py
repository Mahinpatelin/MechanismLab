"""Planar four-bar linkage kinematics using loop-closure constraints."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, isclose, pi, sin, sqrt

import numpy as np


class PositionError(ValueError):
    """Raised when the requested input angle cannot assemble the linkage."""


class KinematicSingularity(ArithmeticError):
    """Raised when velocity/acceleration equations are singular at a toggle."""


@dataclass(frozen=True)
class State:
    input_angle: float
    coupler_angle: float
    output_angle: float
    point_a: tuple[float, float]
    point_b: tuple[float, float]
    output_pivot: tuple[float, float]
    omega_input: float
    omega_coupler: float
    omega_output: float
    alpha_input: float
    alpha_coupler: float
    alpha_output: float
    transmission_angle: float

    @property
    def point_b_velocity(self) -> tuple[float, float]:
        x = self.point_b[0] - self.output_pivot[0]
        y = self.point_b[1] - self.output_pivot[1]
        return (-self.omega_output * y, self.omega_output * x)


@dataclass(frozen=True)
class FourBar:
    """Four-bar with fixed pivots O2=(0,0) and O4=(ground,0).

    Lengths use any consistent unit. Angles are radians, angular velocity is
    radians/time, and angular acceleration is radians/time².
    """

    ground: float
    crank: float
    coupler: float
    rocker: float
    assembly: str = "open"

    def __post_init__(self) -> None:
        if min(self.ground, self.crank, self.coupler, self.rocker) <= 0:
            raise ValueError("all link lengths must be positive")
        if self.assembly not in {"open", "crossed"}:
            raise ValueError("assembly must be 'open' or 'crossed'")

    @property
    def grashof(self) -> bool:
        lengths = sorted((self.ground, self.crank, self.coupler, self.rocker))
        return lengths[0] + lengths[-1] <= lengths[1] + lengths[2]

    @property
    def mobility(self) -> int:
        """Planar Grübler mobility for four links and four revolute joints."""
        return 3 * (4 - 1) - 2 * 4

    def solve(self, theta: float, omega: float = 0.0, alpha: float = 0.0) -> State:
        a = np.array((self.crank * cos(theta), self.crank * sin(theta)), dtype=float)
        o4 = np.array((self.ground, 0.0), dtype=float)
        delta = o4 - a
        distance = float(np.linalg.norm(delta))

        if distance > self.coupler + self.rocker + 1e-12 or distance < abs(self.coupler - self.rocker) - 1e-12:
            raise PositionError(f"linkage cannot assemble at input angle {theta:.6g} rad")
        if isclose(distance, 0.0, abs_tol=1e-12):
            raise PositionError("moving pivot coincides with the output ground pivot")

        along = (self.coupler**2 - self.rocker**2 + distance**2) / (2 * distance)
        height_sq = max(0.0, self.coupler**2 - along**2)
        height = sqrt(height_sq)
        unit = delta / distance
        normal = np.array((-unit[1], unit[0]))
        sign = 1.0 if self.assembly == "open" else -1.0
        b = a + along * unit + sign * height * normal

        r2 = a
        r3 = b - a
        r4 = b - o4
        kxr3 = np.array((-r3[1], r3[0]))
        kxr4 = np.array((-r4[1], r4[0]))
        matrix = np.column_stack((kxr3, -kxr4))
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) < 1e-10:
            raise KinematicSingularity("velocity solution is singular at this toggle position")

        velocity_a = omega * np.array((-r2[1], r2[0]))
        omega3, omega4 = np.linalg.solve(matrix, -velocity_a)

        acceleration_a = alpha * np.array((-r2[1], r2[0])) - omega**2 * r2
        rhs = -acceleration_a + omega3**2 * r3 - omega4**2 * r4
        alpha3, alpha4 = np.linalg.solve(matrix, rhs)

        raw_mu = acos(float(np.clip(np.dot(r3, r4) / (self.coupler * self.rocker), -1.0, 1.0)))
        transmission = min(raw_mu, pi - raw_mu)
        return State(
            input_angle=theta,
            coupler_angle=atan2(r3[1], r3[0]),
            output_angle=atan2(r4[1], r4[0]),
            point_a=(float(a[0]), float(a[1])),
            point_b=(float(b[0]), float(b[1])),
            output_pivot=(self.ground, 0.0),
            omega_input=omega,
            omega_coupler=float(omega3),
            omega_output=float(omega4),
            alpha_input=alpha,
            alpha_coupler=float(alpha3),
            alpha_output=float(alpha4),
            transmission_angle=transmission,
        )

    def coupler_point(self, state: State, fraction: float = 0.5, offset: float = 0.0) -> tuple[float, float]:
        """Return a point fixed to the coupler.

        ``fraction`` measures from A toward B. ``offset`` is perpendicular to AB.
        """
        a = np.asarray(state.point_a)
        b = np.asarray(state.point_b)
        direction = (b - a) / self.coupler
        normal = np.array((-direction[1], direction[0]))
        point = a + fraction * (b - a) + offset * normal
        return float(point[0]), float(point[1])

    def sweep(self, samples: int = 361, omega: float = 1.0) -> list[State | None]:
        if samples < 2:
            raise ValueError("samples must be at least two")
        states: list[State | None] = []
        for theta in np.linspace(0.0, 2 * pi, samples):
            try:
                states.append(self.solve(float(theta), omega=omega))
            except (PositionError, KinematicSingularity):
                states.append(None)
        return states
