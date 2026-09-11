"""Shared dynamic limits and Minimum-Jerk peak calculations."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MotionLimits:
    """Shared translational velocity, acceleration, and jerk limits."""

    velocity: float
    acceleration: float
    jerk: float

    def validate(self) -> None:
        values = (self.velocity, self.acceleration, self.jerk)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("motion limits must be finite and positive")


@dataclass(frozen=True)
class MinimumJerkPeaks:
    """Analytic peak magnitudes for a zero-end-derivative quintic segment."""

    velocity: float
    acceleration: float
    jerk: float


def minimum_jerk_peaks(distance: float, duration: float) -> MinimumJerkPeaks:
    """Return analytic peak speed, acceleration, and jerk magnitudes."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("distance must be finite and non-negative")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    return MinimumJerkPeaks(
        velocity=1.875 * distance / duration,
        acceleration=(10.0 / math.sqrt(3.0)) * distance / duration**2,
        jerk=60.0 * distance / duration**3,
    )
