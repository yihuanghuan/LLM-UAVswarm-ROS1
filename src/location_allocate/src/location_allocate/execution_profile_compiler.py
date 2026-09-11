"""Central compilation of per-UAV execution profiles."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple

from .lfs_types import ExecutableLFS, ExecutionProfile, Vector3
from .motion_limits import MotionLimits, MinimumJerkPeaks, minimum_jerk_peaks


class ProfileCompileError(ValueError):
    """Raised when a complete bounded execution profile cannot be produced."""


@dataclass(frozen=True)
class ExecutionProfilePolicy:
    """Explicit policy values; no semantic-control defaults live in code."""

    base_omega_c: Vector3
    base_omega_o: Vector3
    style_gains: Mapping[str, float]
    task_adaptation_type: str
    task_reference_speed: Optional[float]
    task_gain_intercept: Optional[float]
    task_gain_slope: Optional[float]
    task_gain_range: Optional[Tuple[float, float]]
    total_gain_range: Tuple[float, float]
    motion_limits: MotionLimits
    configuration_id: str

    def validate(self) -> None:
        numeric = (
            *self.base_omega_c,
            *self.base_omega_o,
            *self.style_gains.values(),
            *self.total_gain_range,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ProfileCompileError("profile policy values must be finite and positive")
        try:
            self.motion_limits.validate()
        except ValueError as exc:
            raise ProfileCompileError(str(exc)) from exc
        if self.task_adaptation_type == "identity":
            pass
        elif self.task_adaptation_type == "linear_speed":
            linear = (
                self.task_reference_speed,
                self.task_gain_intercept,
                self.task_gain_slope,
            )
            if any(value is None or not math.isfinite(value) for value in linear):
                raise ProfileCompileError("linear task adaptation is incomplete")
            if self.task_reference_speed <= 0.0 or self.task_gain_intercept <= 0.0:
                raise ProfileCompileError("linear task adaptation is invalid")
            if self.task_gain_slope < 0.0 or self.task_gain_range is None:
                raise ProfileCompileError("linear task gain bounds are invalid")
            if self.task_gain_range[0] > self.task_gain_range[1]:
                raise ProfileCompileError("invalid task_gain_range")
        else:
            raise ProfileCompileError("unsupported task_adaptation_type")
        if self.total_gain_range[0] > self.total_gain_range[1]:
            raise ProfileCompileError("invalid total_gain_range")


@dataclass(frozen=True)
class SoftSafetyParameters:
    """Already-resolved IAPF soft values; d_hard is deliberately absent."""

    enter_distance: float
    exit_distance: float
    repulsion_scale: float

    def validate(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.enter_distance, self.exit_distance)
        ):
            raise ProfileCompileError("IAPF distances must be finite and positive")
        if not math.isfinite(self.repulsion_scale) or self.repulsion_scale < 0.0:
            raise ProfileCompileError(
                "IAPF repulsion scale must be finite and non-negative"
            )
        if self.exit_distance <= self.enter_distance:
            raise ProfileCompileError("IAPF exit distance must exceed enter distance")


def _clamp(value: float, limits: Tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def compile_execution_profiles(
    executable: ExecutableLFS,
    initial: Sequence[Sequence[float]],
    assigned_targets: Sequence[Sequence[float]],
    policy: ExecutionProfilePolicy,
    soft_safety: SoftSafetyParameters,
) -> Tuple[ExecutionProfile, ...]:
    """Compile profiles whose only duration source is executable.duration."""
    policy.validate()
    soft_safety.validate()
    if len(initial) != len(assigned_targets) or len(initial) != len(executable.uav_ids):
        raise ProfileCompileError("profile inputs must align with executable U")
    if executable.motion_style not in policy.style_gains:
        raise ProfileCompileError(
            f"missing style profile: {executable.motion_style}"
        )
    if not math.isfinite(executable.duration) or executable.duration <= 0.0:
        raise ProfileCompileError("T_exec must be finite and positive")

    style_gain = float(policy.style_gains[executable.motion_style])
    profiles = []
    for start, target in zip(initial, assigned_targets):
        distance = math.dist(start, target)
        average_speed = distance / executable.duration
        try:
            peaks = minimum_jerk_peaks(distance, executable.duration)
        except ValueError as exc:
            raise ProfileCompileError(str(exc)) from exc
        limits = policy.motion_limits
        tolerance = 1e-12
        if (
            peaks.velocity > limits.velocity + tolerance
            or peaks.acceleration > limits.acceleration + tolerance
            or peaks.jerk > limits.jerk + tolerance
        ):
            raise ProfileCompileError(
                "final Minimum-Jerk profile violates shared motion limits"
            )
        if policy.task_adaptation_type == "identity":
            task_gain = 1.0
        else:
            raw_task_gain = policy.task_gain_intercept + policy.task_gain_slope * (
                average_speed / policy.task_reference_speed
            )
            task_gain = _clamp(raw_task_gain, policy.task_gain_range)
        total_gain = _clamp(style_gain * task_gain, policy.total_gain_range)
        profiles.append(
            ExecutionProfile(
                duration=executable.duration,
                style=executable.motion_style,
                omega_c=tuple(value * total_gain for value in policy.base_omega_c),
                omega_o=tuple(value * total_gain for value in policy.base_omega_o),
                velocity_limit=limits.velocity,
                acceleration_limit=limits.acceleration,
                jerk_limit=limits.jerk,
                iapf_enter_distance=soft_safety.enter_distance,
                iapf_exit_distance=soft_safety.exit_distance,
                iapf_repulsion_scale=soft_safety.repulsion_scale,
                configuration_id=policy.configuration_id,
                style_gain=style_gain,
                task_gain=task_gain,
            )
        )
    return tuple(profiles)




def predict_profile_peaks(
    initial: Sequence[Sequence[float]],
    assigned_targets: Sequence[Sequence[float]],
    duration: float,
) -> Tuple[MinimumJerkPeaks, ...]:
    """Expose the compiler's analytic post-condition inputs for audit traces."""
    if len(initial) != len(assigned_targets):
        raise ProfileCompileError("peak prediction inputs must align")
    try:
        return tuple(
            minimum_jerk_peaks(math.dist(start, target), duration)
            for start, target in zip(initial, assigned_targets)
        )
    except ValueError as exc:
        raise ProfileCompileError(str(exc)) from exc
