from pathlib import Path

import pytest

from location_allocate.late_resolution import resolve_execution_task
from location_allocate.motion_limits import minimum_jerk_peaks
from location_allocate.policy_adapter import load_runtime_policy
from location_allocate.state_snapshot import FreshStateSnapshotManager


PAPER_CURRENT = (
    Path(__file__).parents[2]
    / "lfs_policy"
    / "config"
    / "lfs_policy.paper_current.yaml"
)
STYLES = ("smooth", "normal", "aggressive")


def snapshot():
    manager = FreshStateSnapshotManager(
        0.5,
        0.15,
        require_velocity=True,
        allow_receive_time_fallback=False,
    )
    manager.update(1, [-1.0, 0.0, 1.5], 10.0, [0.0, 0.0, 0.0], 10.0)
    manager.update(2, [1.0, 0.0, 1.5], 10.0, [0.0, 0.0, 0.0], 10.0)
    return manager.snapshot([1, 2], 10.0)


def candidate(style, time_request, safety_factor=1.0):
    return {
        "task_id": 1,
        "U": [1, 2],
        "F": {"type": "Line"},
        "c": {"mode": "absolute", "value": [4.0, 0.0, 1.5]},
        "r": {"mode": "explicit", "value": 1.0},
        "T": time_request,
        "m": style,
        "s": safety_factor,
        "q": {"mode": "direct"},
    }


def resolve_all(time_request):
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    return policy, {
        style: resolve_execution_task(
            candidate(style, time_request), snapshot(), policy
        )
        for style in STYLES
    }


def test_explicit_t_keeps_one_nominal_mj_reference_and_orders_bandwidths():
    _policy, results = resolve_all({"mode": "explicit", "value": 8.0})

    assert {item.executable_lfs.duration for item in results.values()} == {8.0}
    assert {
        item.assigned_targets for item in results.values()
    } == {results["normal"].assigned_targets}
    assert {
        tuple(
            (peak["distance"], peak["predicted_v_peak"],
             peak["predicted_a_peak"], peak["predicted_j_peak"])
            for peak in item.trace.per_uav_dynamics
        )
        for item in results.values()
    } == {
        tuple(
            (peak["distance"], peak["predicted_v_peak"],
             peak["predicted_a_peak"], peak["predicted_j_peak"])
            for peak in results["normal"].trace.per_uav_dynamics
        )
    }

    for axis in range(3):
        assert (
            results["smooth"].profiles[0].omega_c[axis]
            < results["normal"].profiles[0].omega_c[axis]
            < results["aggressive"].profiles[0].omega_c[axis]
        )
        assert (
            results["smooth"].profiles[0].omega_o[axis]
            < results["normal"].profiles[0].omega_o[axis]
            < results["aggressive"].profiles[0].omega_o[axis]
        )
    assert {
        profile.task_gain
        for item in results.values()
        for profile in item.profiles
    } == {1.0}


def test_auto_t_orders_duration_and_keeps_all_predicted_peaks_feasible():
    policy, results = resolve_all({"mode": "auto"})
    durations = {
        style: item.executable_lfs.duration
        for style, item in results.items()
    }
    assert durations["smooth"] > durations["normal"] > durations["aggressive"]

    limits = policy.timing.motion_limits
    for item in results.values():
        max_distance = max(
            peak["distance"] for peak in item.trace.per_uav_dynamics
        )
        minimum_duration = policy.timing.feasible_duration(max_distance)
        assert item.executable_lfs.duration >= minimum_duration
        for peak in item.trace.per_uav_dynamics:
            assert peak["predicted_v_peak"] <= limits.velocity + 1e-12
            assert peak["predicted_a_peak"] <= limits.acceleration + 1e-12
            assert peak["predicted_j_peak"] <= limits.jerk + 1e-12


def test_dynamic_feasibility_overrides_infeasible_explicit_t_and_style():
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    result = resolve_execution_task(
        candidate("aggressive", {"mode": "explicit", "value": 0.1}),
        snapshot(),
        policy,
    )
    max_distance = max(
        peak["distance"] for peak in result.trace.per_uav_dynamics
    )
    minimum_duration = policy.timing.feasible_duration(max_distance)

    assert result.executable_lfs.duration == pytest.approx(minimum_duration)
    assert result.executable_lfs.duration > 0.1
    assert any(
        "dynamic feasibility" in correction
        for correction in result.trace.corrections
    )
    predicted = minimum_jerk_peaks(max_distance, result.executable_lfs.duration)
    assert predicted.velocity <= policy.timing.motion_limits.velocity + 1e-12
    assert (
        predicted.acceleration
        <= policy.timing.motion_limits.acceleration + 1e-12
    )
    assert predicted.jerk <= policy.timing.motion_limits.jerk + 1e-12
    assert result.final_metrics.min_distance + 1e-9 >= result.trace.d_hard


def test_task_safety_factor_reaches_allocator_trace_and_execution_profile():
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    result = resolve_execution_task(
        candidate("normal", {"mode": "explicit", "value": 8.0}, 1.5),
        snapshot(),
        policy,
    )

    assert result.executable_lfs.safety_factor == 1.5
    assert result.trace.safety_factor == 1.5
    assert result.trace.d_hard == 1.5
    assert result.trace.d_plan == pytest.approx(1.95)
    assert result.trace.iapf_enter_distance == pytest.approx(1.65)
    assert result.trace.iapf_exit_distance == pytest.approx(1.8)
    assert result.trace.iapf_repulsion_scale == 1.125
    for profile in result.profiles:
        assert profile.iapf_enter_distance == pytest.approx(1.65)
        assert profile.iapf_exit_distance == pytest.approx(1.8)
        assert profile.iapf_repulsion_scale == 1.125
