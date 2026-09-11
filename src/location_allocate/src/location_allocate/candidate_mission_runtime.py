"""Pure Candidate validation → graph compilation → FSM composition root."""
from __future__ import annotations

from .paper_lfs_validator import early_validate_candidate_mission
from .mission_compiler import QRelationPolicy, compile_candidate_mission
from .mission_executor import MissionRuntimeCallbacks, execute_compiled_mission


def candidate_relation_policy() -> QRelationPolicy:
    return QRelationPolicy(
        completion_event_by_q={
            "direct": "stable",
            "hover-and-wait": "stable",
            "continuous": "trajectory_complete",
        },
        wait_condition_by_q={
            "direct": None,
            "hover-and-wait": "elapsed",
            "continuous": None,
        },
    )


def execute_candidate_payload(
    payload, callbacks: MissionRuntimeCallbacks, available_uav_ids
):
    validated = early_validate_candidate_mission(
        payload, available_uav_ids=available_uav_ids
    )
    compiled = compile_candidate_mission(validated, candidate_relation_policy())
    execute_compiled_mission(compiled, callbacks)
    return compiled
