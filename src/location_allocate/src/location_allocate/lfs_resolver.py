"""Deterministic, snapshot-bound resolution of Candidate LFS semantics."""
from __future__ import annotations

import math
from typing import Any, Dict, Sequence

from .lfs_types import ResolutionTrace, ResolvedTaskIntent, StateSnapshot, Vector3


class ResolutionError(ValueError):
    """Raised when a candidate task cannot be resolved safely."""


def _centroid(snapshot: StateSnapshot, uav_ids: Sequence[int]) -> Vector3:
    try:
        positions = snapshot.positions(uav_ids)
    except KeyError as exc:
        raise ResolutionError(f"snapshot is missing UAV {exc.args[0]}") from exc
    count = float(len(positions))
    return tuple(
        sum(position[axis] for position in positions) / count for axis in range(3)
    )  # type: ignore[return-value]


def resolve_candidate_task(
    task: Dict[str, Any], snapshot: StateSnapshot
) -> tuple[ResolvedTaskIntent, ResolutionTrace]:
    """Resolve only runtime semantics; geometry/timing remain separate."""
    uav_ids = tuple(int(uid) for uid in task["U"])
    if not uav_ids or len(uav_ids) != len(set(uav_ids)):
        raise ResolutionError("U must contain unique participants")
    for uid in uav_ids:
        if uid not in snapshot.states:
            raise ResolutionError(f"snapshot is missing UAV {uid}")

    center_spec = task["c"]
    mode = center_spec["mode"]
    if mode == "absolute":
        center = tuple(float(value) for value in center_spec["value"])
        center_source = "candidate.absolute"
    elif mode == "maintain_current_centroid":
        center = _centroid(snapshot, uav_ids)
        center_source = "candidate.maintain_current_centroid"
    elif mode == "auto":
        center = _centroid(snapshot, uav_ids)
        center_source = "default.current_task_centroid"
    elif mode == "relative":
        if (
            center_spec.get("reference") != "current_swarm_centroid"
            or center_spec.get("frame") != "world"
        ):
            raise ResolutionError("unsupported relative center reference or frame")
        base = _centroid(snapshot, uav_ids)
        offset = tuple(float(value) for value in center_spec["offset"])
        center = tuple(base[index] + offset[index] for index in range(3))
        center_source = "snapshot.participant_centroid+world_offset"
    else:
        raise ResolutionError(f"unsupported center mode: {mode}")
    if len(center) != 3 or not all(math.isfinite(value) for value in center):
        raise ResolutionError("resolved center must be a finite Vector3")

    safety_factor = float(task["s"])
    if not math.isfinite(safety_factor) or safety_factor < 1.0:
        raise ResolutionError("s must be finite and >= 1")
    intent = ResolvedTaskIntent(
        task_id=int(task["task_id"]),
        uav_ids=uav_ids,
        formation=dict(task["F"]),
        center=center,  # type: ignore[arg-type]
        radius_request=dict(task["r"]),
        time_request=dict(task["T"]),
        motion_style=task["m"],
        safety_factor=safety_factor,
        trigger_semantics=dict(task["q"]),
    )
    trace = ResolutionTrace(
        task_id=intent.task_id,
        candidate_lfs=dict(task),
        snapshot_epoch=snapshot.epoch,
        state_timestamps={
            uid: {
                "source": snapshot.states[uid].source_timestamp,
                "receive": snapshot.states[uid].receive_timestamp,
                "effective_source": snapshot.states[uid].timestamp_source,
            }
            for uid in uav_ids
        },
        center_source=center_source,
        resolved_center=intent.center,
        t_request=dict(intent.time_request),
        warnings=list(snapshot.warnings),
    )
    return intent, trace
