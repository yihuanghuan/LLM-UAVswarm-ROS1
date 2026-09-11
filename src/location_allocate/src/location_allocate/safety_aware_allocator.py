"""Lexicographic safety-aware target allocation for the Paper pipeline."""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class AssignmentMetrics:
    """Paper objective and diagnostics for one assignment candidate."""

    hard_violations: int
    margin_cost: float
    distance: float
    min_distance: float
    xy_crossings: int

    @property
    def score(self) -> tuple[int, float, float]:
        return (self.hard_violations, self.margin_cost, self.distance)


class SafetyAwareTopologyAllocator:
    """Hungarian initialization plus lexicographic pairwise refinement."""

    VERSION = "lexicographic-safety-aware-v2"

    def __init__(
        self,
        d_hard: float,
        d_plan: float,
        sample_hz: float = 20.0,
        comparison_tolerance: float = 1e-6,
    ):
        values = (d_hard, d_plan, sample_hz, comparison_tolerance)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError(
                "allocator thresholds and sampling values must be finite "
                "and positive"
            )
        if d_plan < d_hard:
            raise ValueError("allocator requires d_plan >= d_hard")
        self.d_hard = float(d_hard)
        self.d_plan = float(d_plan)
        self.sample_hz = float(sample_hz)
        self.comparison_tolerance = float(comparison_tolerance)
        self.last_metrics: AssignmentMetrics | None = None
        self.last_iterations = 0
        self.last_initial_assignment: List[int] = []
        self.last_assignment: List[int] = []
        self.last_diagnostics: Dict[str, Any] = {}

    @staticmethod
    def _positions(
        initial: Sequence[Sequence[float]],
        targets: Sequence[Sequence[float]],
    ) -> tuple[np.ndarray, np.ndarray]:
        initial_np = np.asarray(initial, dtype=float)
        targets_np = np.asarray(targets, dtype=float)
        if initial_np.shape != targets_np.shape:
            raise ValueError("initial and targets must have the same shape")
        if initial_np.ndim != 2 or initial_np.shape[1] != 3:
            raise ValueError("positions must be shaped as N x 3")
        if (
            not np.all(np.isfinite(initial_np))
            or not np.all(np.isfinite(targets_np))
        ):
            raise ValueError("positions must be finite")
        return initial_np, targets_np

    @staticmethod
    def _minimum_jerk_progress(normalized_time: np.ndarray) -> np.ndarray:
        return (
            10.0 * normalized_time**3
            - 15.0 * normalized_time**4
            + 6.0 * normalized_time**5
        )

    def sample_nominal_trajectories_variable(
        self,
        initial: Sequence[Sequence[float]],
        assigned_targets: Sequence[Sequence[float]],
        durations: Sequence[float],
    ) -> np.ndarray:
        """Sample independent motions on one synchronized clock."""
        initial_np, targets_np = self._positions(initial, assigned_targets)
        duration_np = np.asarray(durations, dtype=float)
        if duration_np.shape != (len(initial_np),):
            raise ValueError("durations must contain one value per UAV")
        if not np.all(np.isfinite(duration_np)) or np.any(duration_np <= 0.0):
            raise ValueError("durations must be finite and positive")
        horizon = float(duration_np.max())
        sample_count = max(2, int(np.ceil(horizon * self.sample_hz)) + 1)
        times = np.linspace(0.0, horizon, sample_count)
        normalized = np.minimum(times[None, :] / duration_np[:, None], 1.0)
        progress = self._minimum_jerk_progress(normalized)
        return initial_np[:, None, :] + progress[:, :, None] * (
            targets_np - initial_np
        )[:, None, :]

    @staticmethod
    def equal_progress_closest_approach(
        first_start: Sequence[float],
        first_target: Sequence[float],
        second_start: Sequence[float],
        second_target: Sequence[float],
    ) -> float:
        """Return analytic synchronized 3-D closest approach."""
        first_start_np = np.asarray(first_start, dtype=float)
        first_target_np = np.asarray(first_target, dtype=float)
        second_start_np = np.asarray(second_start, dtype=float)
        second_target_np = np.asarray(second_target, dtype=float)
        a = first_start_np - second_start_np
        b = ((first_target_np - first_start_np)
             - (second_target_np - second_start_np))
        denominator = float(np.dot(b, b))
        if denominator <= np.finfo(float).eps:
            progress = 0.0
        else:
            progress = float(
                np.clip(-np.dot(a, b) / denominator, 0.0, 1.0)
            )
        return float(np.linalg.norm(a + progress * b))

    @staticmethod
    def _equal_progress_closest_approach_details(
        first_start, first_target, second_start, second_target
    ) -> tuple[float, float]:
        first_start_np = np.asarray(first_start, dtype=float)
        first_target_np = np.asarray(first_target, dtype=float)
        second_start_np = np.asarray(second_start, dtype=float)
        second_target_np = np.asarray(second_target, dtype=float)
        a = first_start_np - second_start_np
        b = ((first_target_np - first_start_np)
             - (second_target_np - second_start_np))
        denominator = float(np.dot(b, b))
        progress = 0.0 if denominator <= np.finfo(float).eps else float(
            np.clip(-np.dot(a, b) / denominator, 0.0, 1.0)
        )
        return float(np.linalg.norm(a + progress * b)), progress

    @classmethod
    def _minimum_jerk_inverse(cls, progress: float) -> float:
        low, high = 0.0, 1.0
        for _ in range(60):
            middle = (low + high) / 2.0
            value = float(cls._minimum_jerk_progress(np.asarray(middle)))
            if value < progress:
                low = middle
            else:
                high = middle
        return (low + high) / 2.0

    def closest_approach_diagnostic(
        self, initial, assigned_targets, durations, uav_ids, group_indices
    ) -> Dict[str, Any]:
        initial_np, targets_np = self._positions(initial, assigned_targets)
        durations_np = np.asarray(durations, dtype=float)
        best = None
        synchronized = float(durations_np.max() - durations_np.min()) <= 1e-12
        if synchronized:
            for i, j in itertools.combinations(range(len(initial_np)), 2):
                distance, progress = self._equal_progress_closest_approach_details(
                    initial_np[i], targets_np[i], initial_np[j], targets_np[j]
                )
                normalized_time = self._minimum_jerk_inverse(progress)
                item = (distance, i, j, progress,
                        normalized_time * float(durations_np[0]))
                if best is None or item < best:
                    best = item
        else:
            trajectories = self.sample_nominal_trajectories_variable(
                initial_np, targets_np, durations_np
            )
            horizon = float(durations_np.max())
            times = np.linspace(0.0, horizon, trajectories.shape[1])
            for i, j in itertools.combinations(range(len(initial_np)), 2):
                distances = np.linalg.norm(
                    trajectories[i] - trajectories[j], axis=1
                )
                sample = int(np.argmin(distances))
                elapsed = float(times[sample])
                progress = float(self._minimum_jerk_progress(np.asarray(
                    min(elapsed / durations_np[i], 1.0)
                )))
                item = (float(distances[sample]), i, j, progress, elapsed)
                if best is None or item < best:
                    best = item
        target_min = float("inf")
        target_pair = None
        for i, j in itertools.combinations(range(len(targets_np)), 2):
            distance = float(np.linalg.norm(targets_np[i] - targets_np[j]))
            if distance < target_min:
                target_min, target_pair = distance, (i, j)
        if best is None:
            return {}
        distance, first, second, progress, elapsed = best
        return {
            "min_predicted_distance": distance,
            "offending_pair_indices": [first, second],
            "offending_uav_pair": [int(uav_ids[first]), int(uav_ids[second])],
            "offending_pair_groups": [
                int(group_indices[first]), int(group_indices[second])
            ],
            "closest_approach_progress": progress,
            "closest_approach_time_s": elapsed,
            "target_min_distance": target_min,
            "target_min_pair": (
                [] if target_pair is None else
                [int(uav_ids[target_pair[0]]), int(uav_ids[target_pair[1]])]
            ),
        }

    @staticmethod
    def _orientation(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        return (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )

    @classmethod
    def _xy_segments_cross(cls, p1, p2, p3, p4) -> bool:
        if any(np.allclose(left[:2], right[:2]) for left, right in (
            (p1, p3), (p1, p4), (p2, p3), (p2, p4)
        )):
            return False
        a, b, c, d = p1[:2], p2[:2], p3[:2], p4[:2]
        if (max(a[0], b[0]) < min(c[0], d[0])
                or max(c[0], d[0]) < min(a[0], b[0])
                or max(a[1], b[1]) < min(c[1], d[1])
                or max(c[1], d[1]) < min(a[1], b[1])):
            return False
        return (
            cls._orientation(a, b, c) * cls._orientation(a, b, d) < 0.0
            and cls._orientation(c, d, a) * cls._orientation(c, d, b) < 0.0
        )

    def _metrics_from_pair_distances(
        self,
        initial_np: np.ndarray,
        assigned_targets: np.ndarray,
        pair_distances: Sequence[tuple[int, int, float]],
    ) -> AssignmentMetrics:
        distance = float(
            np.linalg.norm(assigned_targets - initial_np, axis=1).sum()
        )
        hard_violations = sum(
            pair_min < self.d_hard for _, _, pair_min in pair_distances
        )
        margin_cost = sum(
            max(0.0, (self.d_plan - pair_min) / self.d_plan) ** 2
            for _, _, pair_min in pair_distances
        )
        min_distance = min(
            (pair_min for _, _, pair_min in pair_distances),
            default=float("inf"),
        )
        xy_crossings = sum(
            self._xy_segments_cross(
                initial_np[i],
                assigned_targets[i],
                initial_np[j],
                assigned_targets[j],
            )
            for i, j, _ in pair_distances
        )
        return AssignmentMetrics(
            hard_violations=int(hard_violations),
            margin_cost=float(margin_cost),
            distance=distance,
            min_distance=float(min_distance),
            xy_crossings=int(xy_crossings),
        )

    def evaluate(
        self, initial, targets, assignment, duration=3.0
    ) -> AssignmentMetrics:
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration must be finite and positive")
        initial_np, target_np = self._positions(initial, targets)
        assignment_np = np.asarray(assignment, dtype=int)
        valid_assignment = (
            assignment_np.shape == (len(initial_np),)
            and sorted(assignment_np.tolist()) == list(range(len(initial_np)))
        )
        if not valid_assignment:
            raise ValueError(
                "assignment must be a permutation of target indices"
            )
        assigned = target_np[assignment_np]
        pairs = [
            (i, j, self.equal_progress_closest_approach(
                initial_np[i], assigned[i], initial_np[j], assigned[j]
            ))
            for i, j in itertools.combinations(range(len(initial_np)), 2)
        ]
        return self._metrics_from_pair_distances(initial_np, assigned, pairs)

    def evaluate_variable(
        self, initial, targets, assignment, durations
    ) -> AssignmentMetrics:
        initial_np, target_np = self._positions(initial, targets)
        assignment_np = np.asarray(assignment, dtype=int)
        valid_assignment = (
            assignment_np.shape == (len(initial_np),)
            and sorted(assignment_np.tolist()) == list(range(len(initial_np)))
        )
        if not valid_assignment:
            raise ValueError(
                "assignment must be a permutation of target indices"
            )
        assigned = target_np[assignment_np]
        trajectories = self.sample_nominal_trajectories_variable(
            initial_np, assigned, durations
        )
        pairs = [
            (i, j, float(np.linalg.norm(
                trajectories[i] - trajectories[j], axis=1
            ).min()))
            for i, j in itertools.combinations(range(len(initial_np)), 2)
        ]
        return self._metrics_from_pair_distances(initial_np, assigned, pairs)

    def lexicographically_better(
        self, candidate: AssignmentMetrics, current: AssignmentMetrics
    ) -> bool:
        if candidate.hard_violations != current.hard_violations:
            return candidate.hard_violations < current.hard_violations
        tolerance = self.comparison_tolerance
        if candidate.margin_cost < current.margin_cost - tolerance:
            return True
        if abs(candidate.margin_cost - current.margin_cost) <= tolerance:
            return candidate.distance < current.distance - tolerance
        return False

    @staticmethod
    def _hungarian_assignment(initial, targets) -> List[int]:
        initial_np, target_np = SafetyAwareTopologyAllocator._positions(
            initial, targets
        )
        cost = np.linalg.norm(
            initial_np[:, None, :] - target_np[None, :, :], axis=2
        )
        rows, columns = linear_sum_assignment(cost)
        assignment = [0] * len(initial_np)
        for row, column in zip(rows, columns):
            assignment[int(row)] = int(column)
        return assignment

    @staticmethod
    def _swap(assignment, i, j) -> List[int]:
        result = list(assignment)
        result[i], result[j] = result[j], result[i]
        return result

    def _refine(self, assignment, evaluator, swappable_ranges):
        initial_assignment = list(assignment)
        best = evaluator(assignment)
        iterations = 0
        while True:
            improved = False
            for group_range in swappable_ranges:
                for i, j in itertools.combinations(group_range, 2):
                    candidate = self._swap(assignment, i, j)
                    metrics = evaluator(candidate)
                    if self.lexicographically_better(metrics, best):
                        assignment, best, improved = candidate, metrics, True
            if not improved:
                break
            iterations += 1
        self.last_initial_assignment = initial_assignment
        self.last_assignment = list(assignment)
        self.last_metrics = best
        self.last_iterations = iterations
        return assignment, best

    @staticmethod
    def _empty_metrics() -> AssignmentMetrics:
        return AssignmentMetrics(0, 0.0, 0.0, float("inf"), 0)

    def allocate_with_metrics(self, initial, targets, duration=3.0):
        if len(initial) != len(targets):
            raise ValueError(
                "initial and targets must contain the same number of positions"
            )
        if not initial:
            empty = self._empty_metrics()
            self.last_initial_assignment = self.last_assignment = []
            self.last_metrics, self.last_iterations = empty, 0
            return [], empty
        assignment = self._hungarian_assignment(initial, targets)
        assignment, metrics = self._refine(
            assignment,
            lambda value: self.evaluate(initial, targets, value, duration),
            [range(len(initial))],
        )
        return np.asarray(targets, dtype=float)[assignment].tolist(), metrics

    def allocate_mode_with_metrics(
        self, initial, targets, duration=3.0, mode="safety_aware"
    ):
        if mode == "safety_aware":
            return self.allocate_with_metrics(initial, targets, duration)
        if len(initial) != len(targets):
            raise ValueError(
                "initial and targets must contain the same number of positions"
            )
        if mode == "fixed":
            assignment = list(range(len(initial)))
        elif mode == "distance_hungarian":
            assignment = self._hungarian_assignment(initial, targets)
        else:
            raise ValueError(
                "assignment_mode must be fixed, distance_hungarian, "
                "or safety_aware"
            )
        metrics = self.evaluate(initial, targets, assignment, duration)
        self.last_initial_assignment = self.last_assignment = list(assignment)
        self.last_metrics, self.last_iterations = metrics, 0
        return np.asarray(targets, dtype=float)[assignment].tolist(), metrics

    def allocate_grouped(
        self,
        groups: Sequence[Dict[str, Any]],
        duration: Optional[float] = None,
        mode: str = "safety_aware",
        durations: Optional[Sequence[float]] = None,
    ):
        if not groups:
            empty = self._empty_metrics()
            self.last_initial_assignment = self.last_assignment = []
            self.last_metrics, self.last_iterations = empty, 0
            return [], empty
        group_durations = (
            [float(duration)] * len(groups) if durations is None
            else [float(value) for value in durations]
        )
        if len(group_durations) != len(groups) or any(
            not math.isfinite(value) or value <= 0.0
            for value in group_durations
        ):
            raise ValueError(
                "durations must contain one finite positive value per group"
            )
        initial, targets, uav_durations, ranges = [], [], [], []
        flat_uav_ids, flat_group_indices = [], []
        seen, cursor = set(), 0
        for group_index, (group, group_duration) in enumerate(
            zip(groups, group_durations)
        ):
            ids = [int(value) for value in group.get("uav_ids", [])]
            starts, goals = group.get("initial", []), group.get("targets", [])
            if len(ids) != len(starts) or len(ids) != len(goals):
                raise ValueError(
                    "each group needs equal uav_ids, initial, and targets "
                    "lengths"
                )
            if seen.intersection(ids):
                raise ValueError(
                    "uav_ids must not appear in more than one group"
                )
            seen.update(ids)
            initial.extend(starts)
            targets.extend(goals)
            uav_durations.extend([group_duration] * len(ids))
            flat_uav_ids.extend(ids)
            flat_group_indices.extend([group_index] * len(ids))
            ranges.append(range(cursor, cursor + len(ids)))
            cursor += len(ids)
        assignment = list(range(len(initial)))
        if mode != "fixed":
            for group_range in ranges:
                indices = list(group_range)
                local = self._hungarian_assignment(
                    [initial[index] for index in indices],
                    [targets[index] for index in indices],
                )
                for row, target in enumerate(local):
                    assignment[indices[row]] = indices[target]
        if max(uav_durations) - min(uav_durations) <= 1e-12:
            def evaluator(value):
                return self.evaluate(
                    initial, targets, value, uav_durations[0]
                )
        else:
            def evaluator(value):
                return self.evaluate_variable(
                    initial, targets, value, uav_durations
                )
        if mode == "safety_aware":
            assignment, metrics = self._refine(assignment, evaluator, ranges)
            local_assignment = list(assignment)
            candidate_count = math.prod(
                math.factorial(len(item)) for item in ranges
            )
            exhaustive_checked = 0
            feasible_assignment = None
            feasible_metrics = None
            exhaustive_best_assignment = list(assignment)
            exhaustive_best_metrics = metrics
            best_min_distance = metrics.min_distance
            best_min_assignment = list(assignment)
            if metrics.min_distance + 1e-9 < self.d_plan and candidate_count <= 100000:
                permutations = [
                    list(itertools.permutations(list(item))) for item in ranges
                ]
                for choices in itertools.product(*permutations):
                    candidate = list(range(len(initial)))
                    for group_range, targets_choice in zip(ranges, choices):
                        for row, target in zip(group_range, targets_choice):
                            candidate[row] = target
                    candidate_metrics = evaluator(candidate)
                    exhaustive_checked += 1
                    if self.lexicographically_better(
                        candidate_metrics, exhaustive_best_metrics
                    ):
                        exhaustive_best_assignment = list(candidate)
                        exhaustive_best_metrics = candidate_metrics
                    if candidate_metrics.min_distance > best_min_distance:
                        best_min_distance = candidate_metrics.min_distance
                        best_min_assignment = list(candidate)
                    if candidate_metrics.min_distance + 1e-9 >= self.d_plan and (
                        feasible_metrics is None or self.lexicographically_better(
                            candidate_metrics, feasible_metrics
                        )
                    ):
                        feasible_assignment = list(candidate)
                        feasible_metrics = candidate_metrics
                assignment = exhaustive_best_assignment
                metrics = exhaustive_best_metrics
                self.last_assignment = list(assignment)
                self.last_metrics = metrics
            assigned_for_diagnostic = np.asarray(
                targets, dtype=float
            )[assignment].tolist()
            self.last_diagnostics = self.closest_approach_diagnostic(
                initial, assigned_for_diagnostic, uav_durations,
                flat_uav_ids, flat_group_indices,
            )
            self.last_diagnostics.update({
                "group_d_plan": self.d_plan,
                "d_hard": self.d_hard,
                "hard_feasible": (
                    metrics.min_distance + 1e-9 >= self.d_hard
                ),
                "planning_margin_met": (
                    metrics.min_distance + 1e-9 >= self.d_plan
                ),
                "residual_planning_risk": (
                    metrics.min_distance + 1e-9 >= self.d_hard
                    and metrics.min_distance + 1e-9 < self.d_plan
                ),
                "margin_intrusion_m": max(
                    0.0, self.d_plan - metrics.min_distance
                ),
                "local_search_assignment": local_assignment,
                "final_assignment": list(assignment),
                "exhaustive_candidate_count": candidate_count,
                "exhaustive_checked": exhaustive_checked,
                "exhaustive_best_assignment": exhaustive_best_assignment,
                "exhaustive_best_metrics": {
                    "N_hard": exhaustive_best_metrics.hard_violations,
                    "J_margin": exhaustive_best_metrics.margin_cost,
                    "J_distance": exhaustive_best_metrics.distance,
                    "min_3d_distance": exhaustive_best_metrics.min_distance,
                },
                "feasible_group_local_assignment_found": (
                    feasible_assignment is not None
                ),
                "planning_margin_satisfying_assignment_found": (
                    feasible_assignment is not None
                ),
                "best_group_local_min_distance": best_min_distance,
                "best_group_local_assignment": best_min_assignment,
            })
        elif mode in ("fixed", "distance_hungarian"):
            metrics = evaluator(assignment)
            self.last_initial_assignment = self.last_assignment = list(
                assignment
            )
            self.last_metrics, self.last_iterations = metrics, 0
            assigned_for_diagnostic = np.asarray(
                targets, dtype=float
            )[assignment].tolist()
            self.last_diagnostics = self.closest_approach_diagnostic(
                initial, assigned_for_diagnostic, uav_durations,
                flat_uav_ids, flat_group_indices,
            )
        else:
            raise ValueError(
                "assignment_mode must be fixed, distance_hungarian, "
                "or safety_aware"
            )
        flat = np.asarray(targets, dtype=float)[assignment].tolist()
        return [flat[item.start:item.stop] for item in ranges], metrics

    def allocate(self, initial, targets, duration=3.0):
        return self.allocate_with_metrics(initial, targets, duration)[0]

    def metrics_dict(self) -> Dict[str, Any]:
        metrics = self.last_metrics
        hard_feasible = (
            None if metrics is None else
            metrics.min_distance + 1e-9 >= self.d_hard
        )
        planning_margin_met = (
            None if metrics is None else
            metrics.min_distance + 1e-9 >= self.d_plan
        )
        return {
            "allocator_version": self.VERSION,
            "hard_violations": (
                None if metrics is None else metrics.hard_violations
            ),
            "margin_cost": None if metrics is None else metrics.margin_cost,
            "distance": None if metrics is None else metrics.distance,
            "min_distance": None if metrics is None else metrics.min_distance,
            "xy_crossings": None if metrics is None else metrics.xy_crossings,
            "d_hard": self.d_hard,
            "d_plan": self.d_plan,
            "hard_feasible": hard_feasible,
            "planning_margin_met": planning_margin_met,
            "residual_planning_risk": (
                None if metrics is None else
                hard_feasible and not planning_margin_met
            ),
            "margin_intrusion_m": (
                None if metrics is None else
                max(0.0, self.d_plan - metrics.min_distance)
            ),
            "hungarian_initial_assignment": list(self.last_initial_assignment),
            "final_assignment": list(self.last_assignment),
            "iterations": self.last_iterations,
        }
