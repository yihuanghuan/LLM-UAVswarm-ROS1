import pytest

from location_allocate.lfs_resolver import resolve_candidate_task
from location_allocate.state_snapshot import FreshStateSnapshotManager
from location_allocate.timing_resolution import (
    ConfiguredMinimumJerkTimingPolicy,
    build_executable_lfs,
    estimate_planning_duration,
    max_pairwise_distance_bound,
    resolve_final_duration,
    timing_requires_recheck,
)
from location_allocate.motion_limits import MotionLimits


STYLES = ("smooth", "normal", "aggressive")


def policy():
    return ConfiguredMinimumJerkTimingPolicy(
        motion_limits=MotionLimits(2.0, 1.5, 3.0),
        minimum_duration=0.5,
        auto_style_factors={
            "smooth": 1.3,
            "normal": 1.1,
            "aggressive": 1.0,
        },
        configuration_id="test-only",
    )


def resolved_intent(time_request, style="normal"):
    manager = FreshStateSnapshotManager(state_timeout=1.0, snapshot_skew=0.1)
    manager.update(1, [0.0, 0.0, 1.0], receive_timestamp=10.0)
    manager.update(2, [1.0, 0.0, 1.0], receive_timestamp=10.0)
    snapshot = manager.snapshot([1, 2], now=10.0)
    task = {
        "task_id": 1,
        "U": [1, 2],
        "F": {"type": "Line"},
        "c": {"mode": "maintain_current_centroid"},
        "r": {"mode": "explicit", "value": 1.0},
        "T": time_request,
        "m": style,
        "s": 1.0,
        "q": {"mode": "direct"},
    }
    return resolve_candidate_task(task, snapshot)


@pytest.mark.parametrize("style", STYLES)
def test_explicit_feasible_t_is_not_modified_by_motion_style(style):
    intent, trace = resolved_intent({"mode": "explicit", "value": 20.0}, style)
    initial = [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]]
    targets = [[3.0, 0.0, 1.0], [4.0, 0.0, 1.0]]

    t_plan = estimate_planning_duration(
        intent, initial, targets, policy(), max_pairwise_distance_bound, trace
    )
    t_exec = resolve_final_duration(intent, initial, targets, policy(), trace)

    assert t_plan == 20.0
    assert t_exec == 20.0


@pytest.mark.parametrize("style", STYLES)
def test_auto_t_uses_style_after_dynamic_feasibility(style):
    intent, trace = resolved_intent({"mode": "auto"}, style)
    initial = [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]]
    targets = [[3.0, 0.0, 1.0], [4.0, 0.0, 1.0]]

    t_exec = resolve_final_duration(intent, initial, targets, policy(), trace)

    feasible = policy().feasible_duration(3.0)
    assert t_exec == pytest.approx(
        feasible * policy().auto_style_factors[style]
    )
    assert t_exec >= feasible


def test_infeasible_explicit_t_is_raised_and_traced():
    intent, trace = resolved_intent({"mode": "explicit", "value": 0.1})
    initial = [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]]
    targets = [[3.0, 0.0, 1.0], [4.0, 0.0, 1.0]]

    t_exec = resolve_final_duration(intent, initial, targets, policy(), trace)

    assert t_exec == pytest.approx(policy().feasible_duration(3.0))
    assert "final dynamic feasibility" in trace.corrections[-1]


def test_executable_lfs_uses_only_final_t_exec():
    intent, trace = resolved_intent({"mode": "auto"})
    trace.t_plan = 99.0
    executable = build_executable_lfs(intent, radius=2.0, t_exec=7.5)

    assert executable.duration == 7.5
    assert executable.as_dict()["T"] == 7.5
    assert "t_plan" not in executable.as_dict()


def test_recheck_tolerance_is_explicit_and_deterministic():
    assert timing_requires_recheck(5.0, 6.0, tolerance=0.5)
    assert not timing_requires_recheck(5.0, 5.4, tolerance=0.5)
    assert not timing_requires_recheck(5.0, 5.0, tolerance=0.0)
    assert timing_requires_recheck(5.0, 5.000001, tolerance=0.0)
