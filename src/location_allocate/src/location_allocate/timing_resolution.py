"""Two-stage planning and final timing with explicitly injected policy."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Mapping, Protocol, Sequence

from .lfs_types import ExecutableLFS, ResolutionTrace, ResolvedTaskIntent
from .motion_limits import MotionLimits


class TimingError(ValueError):
    """Raised when a duration request cannot be resolved safely."""


class TimingPolicy(Protocol):
    """Policy boundary for provisional timing algorithms."""

    configuration_id: str

    def feasible_duration(self, distance: float) -> float:
        ...

    def auto_duration(self, distance: float, motion_style: str) -> float:
        ...


@dataclass(frozen=True)
class ConfiguredMinimumJerkTimingPolicy:
    """Selectable candidate policy; it is never constructed as a default."""

    motion_limits: MotionLimits
    minimum_duration: float
    auto_style_factors: Mapping[str, float]
    configuration_id: str

    def __post_init__(self) -> None:
        values = (
            self.minimum_duration,
        )
        try:
            self.motion_limits.validate()
        except ValueError as exc:
            raise TimingError(str(exc)) from exc
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise TimingError("timing limits must be finite and positive")
        if not self.auto_style_factors:
            raise TimingError("auto_style_factors must be explicit")
        if any(
            not math.isfinite(value) or value < 1.0
            for value in self.auto_style_factors.values()
        ):
            raise TimingError("auto style factors must be finite and >= 1")

    def feasible_duration(self, distance: float) -> float:
        if not math.isfinite(distance) or distance < 0.0:
            raise TimingError("distance must be finite and non-negative")
        velocity_time = 1.875 * distance / self.motion_limits.velocity
        acceleration_time = math.sqrt(
            (10.0 / math.sqrt(3.0)) * distance
            / self.motion_limits.acceleration
        )
        jerk_time = (60.0 * distance / self.motion_limits.jerk) ** (1.0 / 3.0)
        return max(
            self.minimum_duration,
            velocity_time,
            acceleration_time,
            jerk_time,
        )

    def auto_duration(self, distance: float, motion_style: str) -> float:
        if motion_style not in self.auto_style_factors:
            raise TimingError(f"missing auto timing style: {motion_style}")
        return (
            self.feasible_duration(distance)
            * self.auto_style_factors[motion_style]
        )


def max_pairwise_distance_bound(
    initial: Sequence[Sequence[float]], targets: Sequence[Sequence[float]]
) -> float:
    """One conservative candidate bound, selected only when explicitly passed."""
    if not initial or not targets:
        raise TimingError("planning distance bound needs starts and targets")
    return max(math.dist(start, target) for start in initial for target in targets)


def _resolve_request(
    request: Mapping[str, object],
    distance: float,
    motion_style: str,
    policy: TimingPolicy,
) -> tuple[float, bool]:
    feasible = policy.feasible_duration(distance)
    if request["mode"] == "explicit":
        requested = float(request["value"])
        if not math.isfinite(requested) or requested <= 0.0:
            raise TimingError("explicit duration must be finite and positive")
        return max(requested, feasible), requested < feasible
    if request["mode"] == "auto":
        result = policy.auto_duration(distance, motion_style)
        if result + 1e-12 < feasible:
            raise TimingError("auto timing policy violated dynamic feasibility")
        return result, False
    raise TimingError(f"unsupported T request mode: {request['mode']}")


def estimate_planning_duration(
    intent: ResolvedTaskIntent,
    initial: Sequence[Sequence[float]],
    targets: Sequence[Sequence[float]],
    policy: TimingPolicy,
    distance_bound: Callable[
        [Sequence[Sequence[float]], Sequence[Sequence[float]]], float
    ],
    trace: ResolutionTrace,
) -> float:
    """Produce T_plan without allocating targets."""
    bound = float(distance_bound(initial, targets))
    duration, corrected = _resolve_request(
        intent.time_request, bound, intent.motion_style, policy
    )
    trace.t_plan = duration
    trace.configuration_id = policy.configuration_id
    if corrected:
        trace.corrections.append(
            "planning duration raised above explicit request for feasibility"
        )
    return duration


def resolve_final_duration(
    intent: ResolvedTaskIntent,
    initial: Sequence[Sequence[float]],
    assigned_targets: Sequence[Sequence[float]],
    policy: TimingPolicy,
    trace: ResolutionTrace,
) -> float:
    """Produce T_exec from actual assignment distances."""
    if len(initial) != len(assigned_targets) or not initial:
        raise TimingError("final timing needs equal non-empty starts and targets")
    max_distance = max(
        math.dist(start, target)
        for start, target in zip(initial, assigned_targets)
    )
    duration, corrected = _resolve_request(
        intent.time_request, max_distance, intent.motion_style, policy
    )
    trace.t_exec = duration
    if corrected:
        trace.corrections.append(
            "explicit duration raised to final dynamic feasibility bound"
        )
    return duration


def timing_requires_recheck(
    t_plan: float, t_exec: float, tolerance: float
) -> bool:
    if tolerance < 0.0 or not math.isfinite(tolerance):
        raise TimingError("timing recheck tolerance must be finite and non-negative")
    return abs(t_exec - t_plan) > tolerance


def build_executable_lfs(
    intent: ResolvedTaskIntent, radius: float, t_exec: float
) -> ExecutableLFS:
    """Build the formal tuple only after final timing is known."""
    return ExecutableLFS(
        uav_ids=intent.uav_ids,
        formation=dict(intent.formation),
        center=intent.center,
        radius=float(radius),
        duration=float(t_exec),
        motion_style=intent.motion_style,
        safety_factor=intent.safety_factor,
        trigger_semantics=dict(intent.trigger_semantics),
    )
