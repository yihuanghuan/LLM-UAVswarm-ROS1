from dataclasses import asdict
import json
from types import SimpleNamespace

import pytest

from location_allocate.execution_profile_compiler import (
    ExecutionProfilePolicy,
    SoftSafetyParameters,
)
from location_allocate.execution_command_builder import build_execution_command
from location_allocate.formation_geometry import ScalePolicy
from location_allocate.late_resolution import (
    LateResolutionError,
    LateResolutionPolicy,
    SafetyResolution,
    resolve_execution_task,
    resolve_execution_parallel,
)
from location_allocate.safety_aware_allocator import SafetyAwareTopologyAllocator
from location_allocate.motion_limits import MotionLimits
from location_allocate.state_snapshot import FreshStateSnapshotManager
from location_allocate.timing_resolution import (
    ConfiguredMinimumJerkTimingPolicy,
    max_pairwise_distance_bound,
)


def candidate_task():
    return {
        "task_id": 1,
        "U": [1, 2, 3],
        "F": {"type": "Triangle"},
        "c": {"mode": "maintain_current_centroid"},
        "r": {"mode": "qualitative", "value": "normal"},
        "T": {"mode": "auto"},
        "m": "normal",
        "s": 1.0,
        "q": {"mode": "direct"},
    }


def snapshot():
    manager = FreshStateSnapshotManager(1.0, 0.1)
    manager.update(1, [-2.0, 0.0, 2.0], 10.0)
    manager.update(2, [1.0, -1.5, 2.0], 10.0)
    manager.update(3, [1.0, 1.5, 2.0], 10.0)
    return manager.snapshot([1, 2, 3], 10.0)


def fake_command_type():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=None, frame_id=""),
        target_pos=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        profile=SimpleNamespace(),
    )


def policy(safety_resolver=None, tolerance=0.0):
    if safety_resolver is None:
        def safety_resolver(s):
            return SafetyResolution(
                d_hard=0.5,
                d_plan=0.8 * s,
                soft_iapf=SoftSafetyParameters(1.0 * s, 1.2 * s, 1.0),
            )
    return LateResolutionPolicy(
        scale=ScalePolicy(
            nominal_spacing=2.0,
            qualitative_multipliers={"normal": 1.0},
            workspace_bounds=((-20.0, -20.0, 0.0), (20.0, 20.0, 10.0)),
            configuration_id="test-only",
        ),
        timing=ConfiguredMinimumJerkTimingPolicy(
            motion_limits=MotionLimits(2.0, 1.5, 3.0),
            minimum_duration=0.5,
            auto_style_factors={"normal": 1.1},
            configuration_id="test-only",
        ),
        profile=ExecutionProfilePolicy(
            base_omega_c=(3.0, 3.0, 3.5),
            base_omega_o=(10.0, 10.0, 15.0),
            style_gains={"normal": 1.0},
            task_adaptation_type="linear_speed",
            task_reference_speed=2.0,
            task_gain_intercept=0.8,
            task_gain_slope=0.2,
            task_gain_range=(0.7, 1.3),
            total_gain_range=(0.5, 1.8),
            motion_limits=MotionLimits(2.0, 1.5, 3.0),
            configuration_id="test-only",
        ),
        resolve_safety=safety_resolver,
        planning_distance_bound=max_pairwise_distance_bound,
        timing_recheck_tolerance=tolerance,
        allocator_factory=lambda d_hard, d_plan: SafetyAwareTopologyAllocator(
            sample_hz=20.0, d_hard=d_hard, d_plan=d_plan
        ),
    )


def test_candidate_to_executable_pipeline_has_one_duration_source():
    result = resolve_execution_task(candidate_task(), snapshot(), policy())

    assert result.executable_lfs.duration == result.trace.t_exec
    assert all(
        profile.duration == result.executable_lfs.duration
        for profile in result.profiles
    )
    assert result.trace.t_request == {"mode": "auto"}
    assert result.trace.d_hard == 0.5
    assert result.trace.safety_factor == 1.0
    assert result.trace.iapf_enter_distance == 1.0
    assert result.trace.iapf_exit_distance == 1.2
    assert result.trace.iapf_repulsion_scale == 1.0
    assert result.trace.r_safe == pytest.approx(
        result.trace.d_plan / result.trace.delta_min
    )
    assert result.trace.allocator_version == "lexicographic-safety-aware-v2"
    assert result.trace.hungarian_initial_assignment
    assert result.trace.final_assignment
    assert result.trace.planning_assignment_metrics.keys() == {
        "N_hard", "J_margin", "J_distance", "min_3d_distance",
        "xy_crossings", "d_hard", "d_plan", "hard_feasible",
        "planning_margin_met", "residual_planning_risk",
        "margin_intrusion_m",
    }
    assert len(result.trace.per_uav_dynamics) == 3
    json.dumps(asdict(result.trace), allow_nan=False)


def test_timing_difference_causes_only_one_final_recheck():
    result = resolve_execution_task(candidate_task(), snapshot(), policy(tolerance=0.0))

    assert result.trace.corrections.count(
        "final assignment safety re-evaluated once"
    ) == 1


def test_invalid_safety_mapping_is_rejected_before_geometry():
    def unsafe(s):
        return SafetyResolution(
            d_hard=1.0,
            d_plan=0.5,
            soft_iapf=SoftSafetyParameters(1.2, 1.4, 1.0),
        )

    with pytest.raises(LateResolutionError, match="d_plan >= d_hard"):
        resolve_execution_task(candidate_task(), snapshot(), policy(unsafe))


def test_composite_command_has_no_second_duration_field():
    result = resolve_execution_task(candidate_task(), snapshot(), policy())
    command = build_execution_command(
        result, 0, mission_id=5, task_id=1, command_type=fake_command_type
    )

    assert not hasattr(command, "duration")
    assert command.profile.duration == result.executable_lfs.duration
    assert command.target_pos.x == result.assigned_targets[0][0]


def _parallel_task(task_id, uav_ids, center, duration):
    task = candidate_task()
    task.update(
        {
            "task_id": task_id,
            "U": uav_ids,
            "F": {"type": "Line"},
            "c": {"mode": "absolute", "value": center},
            "r": {"mode": "explicit", "value": 1.0},
            "T": {"mode": "explicit", "value": duration},
        }
    )
    return task


def _parallel_snapshot():
    manager = FreshStateSnapshotManager(1.0, 0.1)
    manager.update(1, [-4.0, -2.0, 2.0], 10.0)
    manager.update(2, [-2.0, -2.0, 2.0], 10.0)
    manager.update(3, [2.0, 2.0, 2.0], 10.0)
    manager.update(4, [4.0, 2.0, 2.0], 10.0)
    return manager.snapshot([1, 2, 3, 4], 10.0)


def _parallel_threshold_case(separation):
    centers = ([0.0, 0.0, 2.0], [0.0, separation, 2.0])
    tasks = (
        _parallel_task(1, [1, 2], centers[0], 2.0),
        _parallel_task(2, [3, 4], centers[1], 2.0),
    )
    manager = FreshStateSnapshotManager(1.0, 0.1)
    for uav_id, position in enumerate((
        [-1.0, 0.0, 2.0], [1.0, 0.0, 2.0],
        [-1.0, separation, 2.0], [1.0, separation, 2.0],
    ), start=1):
        manager.update(uav_id, position, 10.0)
    threshold_policy = policy(lambda _s: SafetyResolution(
        d_hard=1.0,
        d_plan=2.0,
        soft_iapf=SoftSafetyParameters(1.5, 1.7, 1.0),
    ), tolerance=100.0)
    return tasks, manager.snapshot([1, 2, 3, 4], 10.0), threshold_policy


def _single_threshold_case(separation):
    task = _parallel_task(1, [1, 2], [0.0, 0.0, 2.0], 2.0)
    manager = FreshStateSnapshotManager(1.0, 0.1)
    manager.update(1, [-separation / 2.0, 0.0, 2.0], 10.0)
    manager.update(2, [separation / 2.0, 0.0, 2.0], 10.0)
    threshold_policy = policy(lambda _s: SafetyResolution(
        d_hard=1.0,
        d_plan=2.0,
        soft_iapf=SoftSafetyParameters(1.5, 1.7, 1.0),
    ), tolerance=100.0)
    return task, manager.snapshot([1, 2], 10.0), threshold_policy


def test_single_final_gate_rejects_below_d_hard():
    task, state, threshold_policy = _single_threshold_case(0.5)

    with pytest.raises(LateResolutionError) as caught:
        resolve_execution_task(task, state, threshold_policy)

    assert caught.value.code == "single_nominal_trajectory_d_hard_violation"
    assert caught.value.diagnostics["final_metrics"]["N_hard"] > 0
    assert caught.value.diagnostics["final_metrics"]["hard_feasible"] is False


def test_single_final_gate_accepts_residual_planning_risk():
    task, state, threshold_policy = _single_threshold_case(1.5)

    result = resolve_execution_task(task, state, threshold_policy)

    assert result.final_metrics.min_distance == pytest.approx(1.5)
    assert result.final_metrics.hard_violations == 0
    assert result.final_metrics.margin_cost > 0.0
    metrics = result.trace.final_assignment_metrics
    assert metrics["hard_feasible"] is True
    assert metrics["planning_margin_met"] is False
    assert metrics["residual_planning_risk"] is True
    assert metrics["margin_intrusion_m"] == pytest.approx(0.5)
    assert any(
        warning.startswith("residual planning risk accepted")
        for warning in result.trace.warnings
    )


def test_single_final_gate_accepts_full_planning_margin():
    task, state, threshold_policy = _single_threshold_case(2.5)

    result = resolve_execution_task(task, state, threshold_policy)

    assert result.final_metrics.min_distance == pytest.approx(2.0)
    assert result.final_metrics.hard_violations == 0
    assert result.final_metrics.margin_cost == pytest.approx(0.0)
    metrics = result.trace.final_assignment_metrics
    assert metrics["hard_feasible"] is True
    assert metrics["planning_margin_met"] is True
    assert metrics["residual_planning_risk"] is False
    assert metrics["margin_intrusion_m"] == pytest.approx(0.0)
    assert not any(
        warning.startswith("residual planning risk accepted")
        for warning in result.trace.warnings
    )


def test_parallel_final_gate_rejects_below_d_hard():
    tasks, state, threshold_policy = _parallel_threshold_case(0.5)

    with pytest.raises(LateResolutionError) as caught:
        resolve_execution_parallel(
            tasks, state, threshold_policy, "synchronized",
            group_d_plan=2.0,
        )

    assert caught.value.code == "parallel_nominal_trajectory_d_hard_violation"
    assert caught.value.diagnostics["final_metrics"]["N_hard"] > 0
    assert caught.value.diagnostics["final_metrics"]["hard_feasible"] is False


def test_parallel_final_gate_accepts_residual_planning_risk():
    tasks, state, threshold_policy = _parallel_threshold_case(1.5)

    result = resolve_execution_parallel(
        tasks, state, threshold_policy, "synchronized",
        group_d_plan=2.0,
    )

    assert result.final_metrics.min_distance == pytest.approx(1.5)
    assert result.final_metrics.hard_violations == 0
    assert result.final_metrics.margin_cost > 0.0
    metrics = result.tasks[0].trace.final_assignment_metrics
    assert metrics["hard_feasible"] is True
    assert metrics["planning_margin_met"] is False
    assert metrics["residual_planning_risk"] is True
    assert metrics["margin_intrusion_m"] == pytest.approx(0.5)
    assert result.tasks[0].trace.warnings


def test_parallel_final_gate_accepts_full_planning_margin():
    tasks, state, threshold_policy = _parallel_threshold_case(2.5)

    result = resolve_execution_parallel(
        tasks, state, threshold_policy, "synchronized",
        group_d_plan=2.0,
    )

    assert result.final_metrics.min_distance == pytest.approx(2.0)
    assert result.final_metrics.hard_violations == 0
    assert result.final_metrics.margin_cost == pytest.approx(0.0)
    metrics = result.tasks[0].trace.final_assignment_metrics
    assert metrics["hard_feasible"] is True
    assert metrics["planning_margin_met"] is True
    assert metrics["residual_planning_risk"] is False
    assert metrics["margin_intrusion_m"] == pytest.approx(0.0)
    assert not any(
        warning.startswith("residual planning risk accepted")
        for warning in result.tasks[0].trace.warnings
    )


def test_parallel_independent_preserves_distinct_exec_durations():
    tasks = (
        _parallel_task(1, [1, 2], [-3.0, -1.0, 2.0], 2.0),
        _parallel_task(2, [3, 4], [3.0, 1.0, 2.0], 4.0),
    )

    result = resolve_execution_parallel(
        tasks, _parallel_snapshot(), policy(tolerance=100.0),
        "independent", group_d_plan=0.8
    )

    durations = [task.executable_lfs.duration for task in result.tasks]
    assert durations[0] != durations[1]
    assert all(
        profile.duration == task.executable_lfs.duration
        for task in result.tasks for profile in task.profiles
    )


def test_parallel_synchronized_uses_max_feasible_exec_duration():
    tasks = (
        _parallel_task(1, [1, 2], [-3.0, -1.0, 2.0], 2.0),
        _parallel_task(2, [3, 4], [3.0, 1.0, 2.0], 4.0),
    )

    result = resolve_execution_parallel(
        tasks, _parallel_snapshot(), policy(tolerance=100.0),
        "synchronized", group_d_plan=0.8
    )

    durations = [task.executable_lfs.duration for task in result.tasks]
    assert durations[0] == durations[1]
    assert durations[0] >= 4.0


def test_parallel_group_rejects_non_max_margin_override():
    tasks = (
        _parallel_task(1, [1, 2], [-3.0, -1.0, 2.0], 2.0),
        _parallel_task(2, [3, 4], [3.0, 1.0, 2.0], 4.0),
    )

    with pytest.raises(LateResolutionError, match="frozen max aggregation"):
        resolve_execution_parallel(
            tasks, _parallel_snapshot(), policy(), "independent",
            group_d_plan=99.0,
        )
