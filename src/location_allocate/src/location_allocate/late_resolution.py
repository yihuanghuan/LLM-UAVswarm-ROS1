"""Composition root for one late-bound Candidate LFS task."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence, Tuple

from .execution_profile_compiler import (
    ExecutionProfilePolicy,
    SoftSafetyParameters,
    compile_execution_profiles,
    predict_profile_peaks,
)
from .formation_geometry import (
    ScalePolicy,
    build_final_geometry,
    build_unit_geometry,
    resolve_scale,
)
from .lfs_resolver import resolve_candidate_task
from .paper_lfs_validator import runtime_validate_candidate_task
from .lfs_types import (
    ExecutableLFS,
    ExecutionProfile,
    ResolutionTrace,
    ResolvedTaskIntent,
    StateSnapshot,
    Vector3,
)
from .safety_aware_allocator import AssignmentMetrics, SafetyAwareTopologyAllocator
from .timing_resolution import (
    TimingPolicy,
    build_executable_lfs,
    estimate_planning_duration,
    resolve_final_duration,
    timing_requires_recheck,
)


class LateResolutionError(RuntimeError):
    """Raised when a task fails a deterministic late-resolution gate."""

    def __init__(self, message, *, code="late_resolution_error", diagnostics=None):
        super().__init__(message)
        self.code = str(code)
        self.diagnostics = dict(diagnostics or {})


@dataclass(frozen=True)
class SafetyResolution:
    """Hard and soft safety values resolved outside the LFS tuple."""

    d_hard: float
    d_plan: float
    soft_iapf: SoftSafetyParameters

    def validate(self) -> None:
        values = (self.d_hard, self.d_plan)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("resolved safety values must be finite")
        if self.d_hard <= 0.0 or self.d_plan < self.d_hard:
            raise ValueError("resolved safety violates d_plan >= d_hard > 0")
        try:
            self.soft_iapf.validate()
        except ValueError as exc:
            raise ValueError(str(exc)) from exc
        if self.soft_iapf.enter_distance <= self.d_hard:
            raise ValueError("resolved IAPF enter distance must exceed d_hard")


@dataclass(frozen=True)
class LateResolutionPolicy:
    """All provisional choices are explicit dependencies."""

    scale: ScalePolicy
    timing: TimingPolicy
    profile: ExecutionProfilePolicy
    resolve_safety: Callable[[float], SafetyResolution]
    planning_distance_bound: Callable[
        [Sequence[Sequence[float]], Sequence[Sequence[float]]], float
    ]
    timing_recheck_tolerance: float
    allocator_factory: Callable[[float, float], SafetyAwareTopologyAllocator]
    policy_hash: str = "unknown"
    code_git_sha: str = "unknown"
    schema_version: str = "paper-candidate-schema-v2"
    schema_hash: str = "unknown"
    allocator_mode: str = "lexicographic-safety-aware-v2"


@dataclass(frozen=True)
class ResolvedExecutionTask:
    executable_lfs: ExecutableLFS
    assigned_targets: Tuple[Vector3, ...]
    profiles: Tuple[ExecutionProfile, ...]
    planning_metrics: AssignmentMetrics
    final_metrics: AssignmentMetrics
    trace: ResolutionTrace


@dataclass(frozen=True)
class ResolvedParallelGroup:
    """Parallel tasks resolved from one snapshot and one joint allocation."""

    tasks: Tuple[ResolvedExecutionTask, ...]
    completion_mode: str
    planning_metrics: AssignmentMetrics
    final_metrics: AssignmentMetrics


@dataclass(frozen=True)
class _PreparedTask:
    intent: ResolvedTaskIntent
    trace: ResolutionTrace
    safety: SafetyResolution
    radius: float
    initial: Tuple[Vector3, ...]
    targets: Tuple[Vector3, ...]
    t_plan: float


def _metrics_payload(
    metrics: AssignmentMetrics,
    d_hard: float | None = None,
    d_plan: float | None = None,
) -> dict:
    payload = {
        "N_hard": metrics.hard_violations,
        "J_margin": metrics.margin_cost,
        "J_distance": metrics.distance,
        "min_3d_distance": metrics.min_distance,
        "xy_crossings": metrics.xy_crossings,
    }
    if d_hard is not None and d_plan is not None:
        hard_feasible = metrics.min_distance + 1e-9 >= d_hard
        planning_margin_met = metrics.min_distance + 1e-9 >= d_plan
        payload.update({
            "d_hard": d_hard,
            "d_plan": d_plan,
            "hard_feasible": hard_feasible,
            "planning_margin_met": planning_margin_met,
            "residual_planning_risk": (
                hard_feasible and not planning_margin_met
            ),
            "margin_intrusion_m": max(
                0.0, d_plan - metrics.min_distance
            ),
        })
    return payload


def _record_allocation_trace(
    trace: ResolutionTrace,
    allocator: SafetyAwareTopologyAllocator,
    planning_metrics: AssignmentMetrics,
    final_metrics: AssignmentMetrics,
) -> None:
    trace.allocator_version = allocator.VERSION
    trace.assignment_mode = "safety_aware"
    trace.hungarian_initial_assignment = list(allocator.last_initial_assignment)
    trace.final_assignment = list(allocator.last_assignment)
    trace.planning_assignment_metrics = _metrics_payload(
        planning_metrics, allocator.d_hard, allocator.d_plan
    )
    trace.final_assignment_metrics = _metrics_payload(
        final_metrics, allocator.d_hard, allocator.d_plan
    )


def _record_profile_trace(
    trace: ResolutionTrace,
    uav_ids: Sequence[int],
    initial: Sequence[Sequence[float]],
    assigned: Sequence[Sequence[float]],
    duration: float,
) -> None:
    peaks = predict_profile_peaks(initial, assigned, duration)
    trace.per_uav_dynamics = [
        {
            "uav_id": int(uav_id),
            "distance": float(math.dist(start, target)),
            "predicted_v_peak": peak.velocity,
            "predicted_a_peak": peak.acceleration,
            "predicted_j_peak": peak.jerk,
        }
        for uav_id, start, target, peak in zip(
            uav_ids, initial, assigned, peaks
        )
    ]


def _record_residual_planning_risk(
    traces: Sequence[ResolutionTrace],
    metrics: AssignmentMetrics,
    d_hard: float,
    d_plan: float,
) -> None:
    if metrics.min_distance + 1e-9 >= d_plan:
        return
    intrusion = d_plan - metrics.min_distance
    warning = (
        "residual planning risk accepted: nominal minimum distance "
        f"{metrics.min_distance:.6f}m is {intrusion:.6f}m inside "
        f"d_plan={d_plan:.6f}m but remains above "
        f"d_hard={d_hard:.6f}m"
    )
    for trace in traces:
        trace.warnings.append(warning)


def _prepare_task(
    candidate_task: dict,
    snapshot: StateSnapshot,
    policy: LateResolutionPolicy,
    minimum_d_plan: float | None = None,
) -> _PreparedTask:
    """Resolve through T_plan, stopping before any assignment is chosen."""
    runtime_validate_candidate_task(candidate_task, snapshot)
    intent, trace = resolve_candidate_task(candidate_task, snapshot)
    trace.policy_hash = policy.policy_hash
    trace.code_git_sha = policy.code_git_sha
    trace.schema_version = policy.schema_version
    trace.schema_hash = policy.schema_hash
    trace.allocator_mode = policy.allocator_mode
    safety = policy.resolve_safety(intent.safety_factor)
    if minimum_d_plan is not None and minimum_d_plan > safety.d_plan:
        safety = SafetyResolution(
            d_hard=safety.d_hard,
            d_plan=float(minimum_d_plan),
            soft_iapf=safety.soft_iapf,
        )
        trace.corrections.append(
            "d_plan raised to ParallelGroup maximum safety margin"
        )
    try:
        safety.validate()
    except ValueError as exc:
        raise LateResolutionError(f"invalid compiled Safety Profile: {exc}") from exc
    trace.safety_factor = intent.safety_factor
    trace.d_hard = safety.d_hard
    trace.d_plan = safety.d_plan
    trace.iapf_enter_distance = safety.soft_iapf.enter_distance
    trace.iapf_exit_distance = safety.soft_iapf.exit_distance
    trace.iapf_repulsion_scale = safety.soft_iapf.repulsion_scale

    unit = build_unit_geometry(intent.formation, len(intent.uav_ids))
    radius = resolve_scale(intent, unit, safety.d_plan, policy.scale, trace)
    targets = build_final_geometry(
        intent.center,
        unit,
        radius,
        policy.scale.workspace_bounds,
        safety.d_plan,
    )
    initial = tuple(snapshot.positions(intent.uav_ids))
    t_plan = estimate_planning_duration(
        intent,
        initial,
        targets,
        policy.timing,
        policy.planning_distance_bound,
        trace,
    )
    return _PreparedTask(
        intent=intent,
        trace=trace,
        safety=safety,
        radius=radius,
        initial=initial,
        targets=targets,
        t_plan=t_plan,
    )


def resolve_execution_task(
    candidate_task: dict,
    snapshot: StateSnapshot,
    policy: LateResolutionPolicy,
) -> ResolvedExecutionTask:
    """Resolve one task exactly once through each independent stage."""
    prepared = _prepare_task(candidate_task, snapshot, policy)
    intent = prepared.intent
    trace = prepared.trace
    safety = prepared.safety
    initial = prepared.initial
    targets = prepared.targets
    t_plan = prepared.t_plan

    allocator = policy.allocator_factory(safety.d_hard, safety.d_plan)
    assigned, planning_metrics = allocator.allocate_with_metrics(
        initial, targets, duration=t_plan
    )
    t_exec = resolve_final_duration(
        intent, initial, assigned, policy.timing, trace
    )
    final_metrics = planning_metrics
    if timing_requires_recheck(
        t_plan, t_exec, policy.timing_recheck_tolerance
    ):
        final_metrics = allocator.evaluate(
            initial,
            assigned,
            list(range(len(assigned))),
            duration=t_exec,
        )
        trace.corrections.append("final assignment safety re-evaluated once")
    if final_metrics.min_distance + 1e-9 < safety.d_hard:
        error_code = "single_nominal_trajectory_d_hard_violation"
        trace.rejection_reason = error_code
        raise LateResolutionError(
            "single-task nominal trajectory violates d_hard",
            code=error_code,
            diagnostics={
                "error_code": error_code,
                "classification": "hard_safety_boundary_violation",
                "task_id": intent.task_id,
                "uav_ids": list(intent.uav_ids),
                "planning_metrics": _metrics_payload(
                    planning_metrics, safety.d_hard, safety.d_plan
                ),
                "final_metrics": _metrics_payload(
                    final_metrics, safety.d_hard, safety.d_plan
                ),
            },
        )

    _record_residual_planning_risk(
        (trace,), final_metrics, safety.d_hard, safety.d_plan
    )

    executable = build_executable_lfs(intent, prepared.radius, t_exec)
    profiles = compile_execution_profiles(
        executable, initial, assigned, policy.profile, safety.soft_iapf
    )
    _record_allocation_trace(trace, allocator, planning_metrics, final_metrics)
    _record_profile_trace(trace, intent.uav_ids, initial, assigned, t_exec)
    return ResolvedExecutionTask(
        executable_lfs=executable,
        assigned_targets=tuple(tuple(value for value in target) for target in assigned),
        profiles=profiles,
        planning_metrics=planning_metrics,
        final_metrics=final_metrics,
        trace=trace,
    )


def resolve_execution_parallel(
    candidate_tasks: Sequence[dict],
    snapshot: StateSnapshot,
    policy: LateResolutionPolicy,
    completion_mode: str,
    group_d_plan: float | None = None,
) -> ResolvedParallelGroup:
    """
    Resolve one parallel group with a shared snapshot and joint safety horizon.

    The paper implementation freezes group planning margin aggregation to max.
    The optional argument remains only as an assertion-compatible API.
    """
    if completion_mode not in ("independent", "synchronized"):
        raise LateResolutionError("invalid parallel completion_mode")
    if not candidate_tasks:
        raise LateResolutionError("parallel group must not be empty")
    requested_d_plan = tuple(
        policy.resolve_safety(float(task["s"])).d_plan
        for task in candidate_tasks
    )
    frozen_group_d_plan = max(requested_d_plan)
    if group_d_plan is not None and abs(group_d_plan - frozen_group_d_plan) > 1e-12:
        raise LateResolutionError(
            "group_d_plan must equal the frozen max aggregation"
        )
    group_d_plan = frozen_group_d_plan
    prepared = tuple(
        _prepare_task(task, snapshot, policy, group_d_plan)
        for task in candidate_tasks
    )
    hard_values = {item.safety.d_hard for item in prepared}
    if len(hard_values) != 1:
        raise LateResolutionError("d_hard must be constant across a mission")

    groups = [
        {
            "uav_ids": list(item.intent.uav_ids),
            "initial": list(item.initial),
            "targets": list(item.targets),
        }
        for item in prepared
    ]
    group_d_hard = hard_values.pop()
    allocator = policy.allocator_factory(group_d_hard, group_d_plan)
    assigned_groups, planning_metrics = allocator.allocate_grouped(
        groups, durations=[item.t_plan for item in prepared]
    )
    t_exec_values = [
        resolve_final_duration(
            item.intent,
            item.initial,
            assigned,
            policy.timing,
            item.trace,
        )
        for item, assigned in zip(prepared, assigned_groups)
    ]
    if completion_mode == "synchronized":
        synchronized = max(t_exec_values)
        for item, original in zip(prepared, t_exec_values):
            item.trace.t_exec = synchronized
            if synchronized > original:
                item.trace.corrections.append(
                    "duration synchronized by explicit parallel relation"
                )
        t_exec_values = [synchronized] * len(prepared)

    final_metrics = planning_metrics
    needs_recheck = any(
        timing_requires_recheck(
            item.t_plan, t_exec, policy.timing_recheck_tolerance
        )
        for item, t_exec in zip(prepared, t_exec_values)
    )
    if needs_recheck:
        flat_initial = [point for item in prepared for point in item.initial]
        flat_assigned = [point for group in assigned_groups for point in group]
        uav_durations = [
            duration
            for item, duration in zip(prepared, t_exec_values)
            for _ in item.intent.uav_ids
        ]
        if max(uav_durations) - min(uav_durations) <= 1e-12:
            final_metrics = allocator.evaluate(
                flat_initial,
                flat_assigned,
                list(range(len(flat_assigned))),
                uav_durations[0],
            )
        else:
            final_metrics = allocator.evaluate_variable(
                flat_initial,
                flat_assigned,
                list(range(len(flat_assigned))),
                uav_durations,
            )
        for item in prepared:
            item.trace.corrections.append(
                "parallel assignment safety re-evaluated once"
            )
    if final_metrics.min_distance + 1e-9 < group_d_hard:
        diagnostics = dict(allocator.last_diagnostics)
        diagnostics.update({
            "error_code": "parallel_nominal_trajectory_d_hard_violation",
            "classification": "hard_safety_boundary_violation",
            "completion_mode": completion_mode,
            "planning_metrics": _metrics_payload(
                planning_metrics, group_d_hard, group_d_plan
            ),
            "final_metrics": _metrics_payload(
                final_metrics, group_d_hard, group_d_plan
            ),
            "groups": [
                {
                    "group_index": index,
                    "uav_ids": list(item.intent.uav_ids),
                    "initial_positions": [list(value) for value in item.initial],
                    "resolved_center": list(item.intent.center),
                    "formation": item.intent.formation,
                    "resolved_radius": item.radius,
                    "generated_targets": [list(value) for value in item.targets],
                    "individual_d_plan": item.safety.d_plan,
                    "t_plan": item.t_plan,
                    "t_exec": t_exec,
                }
                for index, (item, t_exec) in enumerate(
                    zip(prepared, t_exec_values)
                )
            ],
        })
        for item in prepared:
            item.trace.rejection_reason = diagnostics["error_code"]
        raise LateResolutionError(
            "parallel nominal trajectory violates d_hard",
            code=diagnostics["error_code"],
            diagnostics=diagnostics,
        )

    _record_residual_planning_risk(
        tuple(item.trace for item in prepared),
        final_metrics,
        group_d_hard,
        group_d_plan,
    )

    resolved_tasks = []
    for item, assigned, t_exec in zip(
        prepared, assigned_groups, t_exec_values
    ):
        executable = build_executable_lfs(
            item.intent, item.radius, t_exec
        )
        profiles = compile_execution_profiles(
            executable,
            item.initial,
            assigned,
            policy.profile,
            item.safety.soft_iapf,
        )
        _record_allocation_trace(
            item.trace, allocator, planning_metrics, final_metrics
        )
        _record_profile_trace(
            item.trace,
            item.intent.uav_ids,
            item.initial,
            assigned,
            t_exec,
        )
        resolved_tasks.append(
            ResolvedExecutionTask(
                executable_lfs=executable,
                assigned_targets=tuple(
                    tuple(value for value in target) for target in assigned
                ),
                profiles=profiles,
                planning_metrics=planning_metrics,
                final_metrics=final_metrics,
                trace=item.trace,
            )
        )
    return ResolvedParallelGroup(
        tasks=tuple(resolved_tasks),
        completion_mode=completion_mode,
        planning_metrics=planning_metrics,
        final_metrics=final_metrics,
    )
