"""Candidate-level mission graph compilation without numeric resolution."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

from .lfs_types import (
    CompiledMission,
    CompiledParallelGroup,
    CompiledTaskNode,
    WaitSpec,
)


class MissionCompileError(ValueError):
    """Raised when candidate relation semantics cannot form one graph."""


@dataclass(frozen=True)
class QRelationPolicy:
    """Explicit, injected q-to-graph mapping; no production defaults are hidden."""

    completion_event_by_q: Mapping[str, str]
    wait_condition_by_q: Mapping[str, Optional[str]]


def _compile_task(
    task: Dict[str, Any], policy: QRelationPolicy
) -> CompiledTaskNode:
    q_value = task["q"]["mode"]
    if q_value not in policy.completion_event_by_q:
        raise MissionCompileError(f"q={q_value!r} has no configured graph mapping")
    wait_condition = policy.wait_condition_by_q.get(q_value)
    wait = None
    wait_time = task["q"].get("duration")
    if wait_condition is not None:
        wait = WaitSpec(
            condition=wait_condition,
            duration=None if wait_time is None else float(wait_time),
        )
    elif wait_time is not None:
        raise MissionCompileError(
            f"q.mode={q_value!r} does not permit q.duration"
        )
    return CompiledTaskNode(
        task=dict(task),
        completion_event=policy.completion_event_by_q[q_value],
        wait=wait,
    )


def compile_candidate_mission(
    candidate_mission: Dict[str, Any], policy: QRelationPolicy
) -> CompiledMission:
    """Compile only task relations; all c/r/T values remain unresolved."""
    nodes = []
    for node in candidate_mission["mission"]["nodes"]:
        node_type = node["type"]
        if node_type == "task":
            nodes.append(_compile_task(node["task"], policy))
            continue
        if node_type == "parallel":
            tasks = tuple(_compile_task(task, policy) for task in node["tasks"])
            nodes.append(
                CompiledParallelGroup(
                    tasks=tasks,
                    completion_mode=node.get("completion_mode", "independent"),
                )
            )
            continue
        raise MissionCompileError(f"unsupported mission node type: {node_type}")
    return CompiledMission(nodes=tuple(nodes))
