"""Paper Candidate schema, static semantics, and runtime-state validation."""
from __future__ import annotations

import copy
import math
from typing import Any, Dict, Optional, Sequence

from .lfs_types import StateSnapshot
from .validation_common import LFSValidationError, is_candidate_mission

try:
    from jsonschema import Draft202012Validator
except ModuleNotFoundError:
    Draft202012Validator = None


def _assert_finite(value: Any, location: str) -> None:
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise LFSValidationError(f"{location} must be finite")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _assert_finite(item, f"{location}[{index}]")
    elif isinstance(value, dict):
        for key, item in value.items():
            _assert_finite(item, f"{location}.{key}")


def early_validate_candidate_mission(
    payload: Dict[str, Any],
    schema: Optional[Dict[str, Any]] = None,
    available_uav_ids: Optional[Sequence[int]] = None,
) -> Dict[str, Any]:
    validate_candidate_schema(payload, schema)
    early_validate_candidate_semantics(payload, available_uav_ids)
    return copy.deepcopy(payload)


def validate_candidate_schema(
    payload: Dict[str, Any], schema: Optional[Dict[str, Any]] = None
) -> None:
    if Draft202012Validator is None:
        raise LFSValidationError(
            "jsonschema is required for Paper Candidate validation"
        )
    if schema is None:
        from .prompt_loader import load_paper_schema

        schema = load_paper_schema()
    validator = Draft202012Validator(schema)
    errors = sorted(validator.iter_errors(payload), key=lambda err: list(err.path))
    if errors:
        first = errors[0]
        location = ".".join(str(part) for part in first.path) or "<root>"
        raise LFSValidationError(
            f"Candidate JSON schema validation failed: {location}: "
            f"{first.message}"
        )
    if not is_candidate_mission(payload):
        raise LFSValidationError("payload is not a Candidate Mission")
    _assert_finite(payload, "<root>")


def early_validate_candidate_semantics(
    payload: Dict[str, Any],
    available_uav_ids: Optional[Sequence[int]] = None,
) -> None:
    if not is_candidate_mission(payload):
        raise LFSValidationError("payload is not a Candidate Mission")
    if available_uav_ids is None:
        raise LFSValidationError(
            "static semantic validation requires available UAV IDs"
        )
    available = {int(uid) for uid in available_uav_ids}
    if not available:
        raise LFSValidationError("available UAV IDs must not be empty")
    nodes = payload["mission"]["nodes"]
    seen_task_ids = set()
    for node_index, node in enumerate(nodes):
        tasks = [node["task"]] if node["type"] == "task" else node["tasks"]
        if node["type"] == "parallel":
            seen_uavs = set()
            for task in tasks:
                overlap = seen_uavs.intersection(task["U"])
                if overlap:
                    raise LFSValidationError(
                        f"parallel node {node_index} has overlapping UAV IDs: "
                        f"{sorted(overlap)}"
                    )
                seen_uavs.update(task["U"])
        for task in tasks:
            task_id = int(task["task_id"])
            if task_id in seen_task_ids:
                raise LFSValidationError(f"duplicate task_id: {task_id}")
            seen_task_ids.add(task_id)
            unknown = sorted(set(task["U"]) - available)
            if unknown:
                raise LFSValidationError(
                    f"task {task_id} references unavailable UAV IDs: {unknown}"
                )
            count = len(task["U"])
            descriptor = task["F"]
            formation = descriptor["type"]
            cardinality_valid = {
                "Line": count >= 2,
                "Circle": count >= 4,
                "Sphere": count >= 2,
                "Triangle": count == 3,
                "Polygon": count >= int(descriptor.get("sides", count + 1)),
            }[formation]
            if not cardinality_valid:
                raise LFSValidationError(
                    f"task {task_id} has invalid {formation} cardinality: {count}"
                )
            if float(task["s"]) < 1.0:
                raise LFSValidationError(
                    f"task {task_id} safety factor s must be >= 1"
                )
            if task["q"]["mode"] == "continuous":
                if node["type"] == "parallel":
                    raise LFSValidationError(
                        f"task {task_id} continuous transition is not allowed "
                        "inside a ParallelGroup"
                    )
                if node_index == len(nodes) - 1:
                    raise LFSValidationError(
                        f"task {task_id} continuous transition requires a successor"
                    )


def runtime_validate_candidate_task(
    task: Dict[str, Any], snapshot: StateSnapshot
) -> None:
    if not isinstance(snapshot, StateSnapshot):
        raise LFSValidationError("runtime validation requires StateSnapshot")
    requested = tuple(int(uid) for uid in task["U"])
    if not requested or len(requested) != len(set(requested)):
        raise LFSValidationError("runtime task U must contain unique UAV IDs")
    missing = sorted(uid for uid in requested if uid not in snapshot.states)
    if missing:
        raise LFSValidationError(f"runtime snapshot is missing UAV IDs: {missing}")
    if not math.isfinite(snapshot.epoch):
        raise LFSValidationError("runtime snapshot epoch must be finite")
    for uid in requested:
        if snapshot.states[uid].effective_timestamp > snapshot.epoch:
            raise LFSValidationError(
                f"runtime snapshot contains future state for UAV {uid}"
            )
