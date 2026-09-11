import math

import numpy as np
import pytest

from location_allocate.safety_aware_allocator import (
    AssignmentMetrics,
    SafetyAwareTopologyAllocator,
)


def allocator(**kwargs):
    return SafetyAwareTopologyAllocator(
        d_hard=kwargs.pop("d_hard", 0.5),
        d_plan=kwargs.pop("d_plan", 1.0),
        **kwargs,
    )


def metrics(hard, margin, distance, crossings=0):
    return AssignmentMetrics(hard, margin, distance, 0.25, crossings)


def test_allocator_requires_ordered_positive_safety_thresholds():
    with pytest.raises(ValueError, match="d_plan >= d_hard"):
        SafetyAwareTopologyAllocator(d_hard=2.0, d_plan=1.0)
    with pytest.raises(ValueError, match="finite and positive"):
        SafetyAwareTopologyAllocator(d_hard=0.0, d_plan=1.0)


def test_lexicographic_hard_priority_dominates_margin_and_distance():
    subject = allocator()
    assert subject.lexicographically_better(
        metrics(0, 0.9, 100.0), metrics(1, 0.0, 1.0)
    )


def test_lexicographic_margin_priority_dominates_distance():
    subject = allocator()
    assert subject.lexicographically_better(
        metrics(0, 0.1, 100.0), metrics(0, 0.2, 1.0)
    )


def test_lexicographic_distance_breaks_equivalent_margin_tie():
    subject = allocator(comparison_tolerance=1e-5)
    assert subject.lexicographically_better(
        metrics(0, 0.100004, 1.0), metrics(0, 0.1, 2.0)
    )


def test_xy_crossing_is_diagnostic_only_when_z_separation_is_safe():
    subject = allocator(d_hard=1.0, d_plan=2.0)
    initial = [[-1.0, 0.0, 0.0], [1.0, 0.0, 10.0]]
    targets = [[1.0, 2.0, 0.0], [-1.0, 2.0, 10.0]]
    result = subject.evaluate(initial, targets, [0, 1], duration=3.0)

    assert result.xy_crossings == 1
    assert result.hard_violations == 0
    assert result.margin_cost == 0.0
    assert result.score == (0, 0.0, result.distance)


@pytest.mark.parametrize(
    "distance, hard, positive_margin",
    [(0.4, 1, True), (0.75, 0, True), (1.0, 0, False), (1.2, 0, False)],
)
def test_hard_and_planning_threshold_semantics(
    distance, hard, positive_margin
):
    subject = allocator(d_hard=0.5, d_plan=1.0)
    result = subject.evaluate(
        [[0, 0, 0], [distance, 0, 0]],
        [[1, 0, 0], [1 + distance, 0, 0]],
        [0, 1],
        duration=2.0,
    )
    assert result.hard_violations == hard
    assert (result.margin_cost > 0.0) is positive_margin


def test_equal_progress_analytic_closest_approach_matches_dense_ground_truth():
    subject = allocator()
    starts = np.array([[-2.0, -1.0, 0.0], [2.0, 1.0, 3.0]])
    goals = np.array([[3.0, 2.0, 1.0], [-1.0, -2.0, 2.0]])
    analytic = subject.equal_progress_closest_approach(
        starts[0], goals[0], starts[1], goals[1]
    )
    progress = np.linspace(0.0, 1.0, 500001)
    first = starts[0] + progress[:, None] * (goals[0] - starts[0])
    second = starts[1] + progress[:, None] * (goals[1] - starts[1])
    numerical = np.linalg.norm(first - second, axis=1).min()
    assert analytic == pytest.approx(numerical, abs=2e-5)


def test_equal_progress_degenerate_relative_motion_is_constant():
    result = allocator().equal_progress_closest_approach(
        [0, 0, 0], [2, 1, 0], [0, 3, 4], [2, 4, 4]
    )
    assert result == pytest.approx(5.0)


def test_variable_duration_sampling_uses_synchronized_clock_and_holds_goal():
    subject = allocator(sample_hz=10.0)
    trajectories = subject.sample_nominal_trajectories_variable(
        [[0, 0, 0], [0, 2, 0]],
        [[1, 0, 0], [4, 2, 0]],
        [1.0, 3.0],
    )
    assert trajectories.shape == (2, 31, 3)
    assert trajectories[0, 10:].tolist() == [[1.0, 0.0, 0.0]] * 21
    result = subject.evaluate_variable(
        [[0, 0, 0], [0, 2, 0]],
        [[1, 0, 0], [4, 2, 0]],
        [0, 1],
        [1.0, 3.0],
    )
    assert math.isfinite(result.min_distance)


def test_hungarian_initialization_and_pair_swap_terminate_lexicographically():
    subject = allocator(d_hard=1.0, d_plan=1.5)
    initial = [[-2, -2, 0], [-2, 0, 0], [-2, 2, 0]]
    targets = [[-2, -2, 0], [-2, 0, 0], [0, -2, 0]]
    expected_initial = subject._hungarian_assignment(initial, targets)
    _allocated, result = subject.allocate_with_metrics(initial, targets, 4.0)

    assert subject.last_initial_assignment == expected_initial
    assert subject.last_iterations >= 0
    for i in range(len(initial)):
        for j in range(i + 1, len(initial)):
            candidate = subject._swap(subject.last_assignment, i, j)
            candidate_metrics = subject.evaluate(
                initial, targets, candidate, 4.0
            )
            assert not subject.lexicographically_better(
                candidate_metrics, result
            )


def test_grouped_allocator_preserves_target_ownership_and_scores_all_pairs():
    subject = allocator()
    groups = [
        {"uav_ids": [1, 2], "initial": [[0, 0, 0], [0, 2, 0]],
         "targets": [[2, 0, 0], [2, 2, 0]]},
        {"uav_ids": [3, 4], "initial": [[0, 4, 0], [0, 6, 0]],
         "targets": [[4, 4, 0], [4, 6, 0]]},
    ]
    allocated, result = subject.allocate_grouped(
        groups, durations=[1.0, 3.0], mode="safety_aware"
    )
    assert {tuple(point) for point in allocated[0]} == {
        tuple(point) for point in groups[0]["targets"]
    }
    assert {tuple(point) for point in allocated[1]} == {
        tuple(point) for point in groups[1]["targets"]
    }
    assert math.isfinite(result.min_distance)


def test_grouped_allocator_rejects_duplicate_uav_ids():
    groups = [
        {"uav_ids": [1], "initial": [[0, 0, 0]], "targets": [[1, 0, 0]]},
        {"uav_ids": [1], "initial": [[0, 1, 0]], "targets": [[1, 1, 0]]},
    ]
    with pytest.raises(ValueError, match="more than one group"):
        allocator().allocate_grouped(groups, duration=3.0)


def test_fixed_and_distance_modes_remain_available_for_experiments():
    subject = allocator()
    initial = [[0, 0, 0], [4, 0, 0]]
    targets = [[4, 0, 0], [0, 0, 0]]
    fixed, _ = subject.allocate_mode_with_metrics(
        initial, targets, mode="fixed"
    )
    distance, _ = subject.allocate_mode_with_metrics(
        initial, targets, mode="distance_hungarian"
    )
    assert fixed == targets
    assert distance == initial
