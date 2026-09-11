import pytest

from location_allocate.paper_lfs_validator import (
    LFSValidationError,
    early_validate_candidate_mission,
)
from location_allocate.mission_compiler import (
    MissionCompileError,
    QRelationPolicy,
    compile_candidate_mission,
)


AVAILABLE_UAV_IDS = [1, 2, 3, 4, 5]






def candidate_task(**overrides):
    task = {
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
    task.update(overrides)
    return task


def relation_policy():
    return QRelationPolicy(
        completion_event_by_q={
            "direct": "stable",
            "hover-and-wait": "stable",
            "continuous": "trajectory_complete",
        },
        wait_condition_by_q={
            "direct": None,
            "hover-and-wait": "hover_stable",
            "continuous": None,
        },
    )
















def test_candidate_mission_early_validation_and_graph_compilation():
    payload = {
        "lfs_version": "2.1",
        "mission": {
            "nodes": [
                {"type": "task", "task": candidate_task()},
                {
                    "type": "parallel",
                    "completion_mode": "independent",
                    "tasks": [
                        candidate_task(
                            task_id=2, U=[1, 2], F={"type": "Line"}
                        ),
                        candidate_task(task_id=3, U=[3, 4, 5]),
                    ],
                },
            ]
        }
    }

    validated = early_validate_candidate_mission(payload, available_uav_ids=AVAILABLE_UAV_IDS)
    compiled = compile_candidate_mission(validated, relation_policy())

    assert len(compiled.nodes) == 2
    assert compiled.nodes[1].completion_mode == "independent"
    assert compiled.nodes[1].tasks[0].task["T"] == {"mode": "auto"}


def test_candidate_auto_center_and_scale_are_schema_valid():
    payload = {
        "lfs_version": "2.1",
        "mission": {"nodes": [{
            "type": "task",
            "task": candidate_task(c={"mode": "auto"}, r={"mode": "auto"}),
        }]}
    }

    assert early_validate_candidate_mission(
        payload, available_uav_ids=AVAILABLE_UAV_IDS
    ) == payload


def test_paper_wait_is_only_canonical_q_and_wait_node_is_rejected():
    canonical = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "task",
        "task": candidate_task(q={"mode": "hover-and-wait", "duration": 2.0}),
    }]}}
    wait_node = {"lfs_version": "2.1", "mission": {"nodes": [
        {"type": "wait", "condition": "elapsed", "duration": 2.0}
    ]}}

    assert early_validate_candidate_mission(
        canonical, available_uav_ids=AVAILABLE_UAV_IDS
    ) == canonical
    compiled = compile_candidate_mission(canonical, relation_policy())
    assert compiled.nodes[0].wait.condition == "hover_stable"
    assert compiled.nodes[0].wait.duration == 2.0
    with pytest.raises(LFSValidationError, match="schema"):
        early_validate_candidate_mission(
            wait_node, available_uav_ids=AVAILABLE_UAV_IDS
        )


def test_candidate_parallel_overlap_is_rejected_early():
    payload = {
        "lfs_version": "2.1",
        "mission": {
            "nodes": [{
                "type": "parallel",
                "completion_mode": "independent",
                "tasks": [
                    candidate_task(task_id=1, U=[1, 2]),
                    candidate_task(task_id=2, U=[2, 3]),
                ],
            }]
        }
    }

    with pytest.raises(LFSValidationError, match="overlapping UAV"):
        early_validate_candidate_mission(payload, available_uav_ids=AVAILABLE_UAV_IDS)


def test_candidate_non_finite_and_weak_safety_are_rejected():
    infinite = {
        "lfs_version": "2.1", "mission": {"nodes": [{
            "type": "task",
            "task": candidate_task(
                c={"mode": "absolute", "value": [0.0, float("inf"), 1.0]}
            ),
        }]}
    }
    weak_safety = {
        "lfs_version": "2.1", "mission": {"nodes": [{
            "type": "task", "task": candidate_task(s=0.5)
        }]}
    }

    with pytest.raises(LFSValidationError):
        early_validate_candidate_mission(infinite, available_uav_ids=AVAILABLE_UAV_IDS)
    with pytest.raises(LFSValidationError):
        early_validate_candidate_mission(weak_safety, available_uav_ids=AVAILABLE_UAV_IDS)


def test_candidate_requires_explicit_q_graph_policy():
    payload = {
        "lfs_version": "2.1",
        "mission": {"nodes": [{
            "type": "task", "task": candidate_task(q={"mode": "direct"})
        }]}
    }
    validated = early_validate_candidate_mission(payload, available_uav_ids=AVAILABLE_UAV_IDS)
    missing_policy = QRelationPolicy({}, {})

    with pytest.raises(MissionCompileError, match="no configured graph mapping"):
        compile_candidate_mission(validated, missing_policy)




@pytest.mark.parametrize(
    "formation,uav_ids",
    [
        ({"type": "Line"}, [1]),
        ({"type": "Circle"}, [1, 2, 3]),
        ({"type": "Sphere"}, [1]),
        ({"type": "Triangle"}, [1, 2, 3, 4]),
        ({"type": "Polygon", "sides": 5}, [1, 2, 3, 4]),
    ],
)
def test_static_validator_rejects_invalid_formation_cardinality(
        formation, uav_ids):
    payload = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "task",
        "task": candidate_task(F=formation, U=uav_ids),
    }]}}

    with pytest.raises(LFSValidationError, match="cardinality"):
        early_validate_candidate_mission(
            payload, available_uav_ids=AVAILABLE_UAV_IDS
        )


def test_static_validator_rejects_unavailable_uav_and_duplicate_task_id():
    unavailable = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "task", "task": candidate_task(U=[1, 2, 9]),
    }]}}
    duplicate = {"lfs_version": "2.1", "mission": {"nodes": [
        {"type": "task", "task": candidate_task()},
        {"type": "task", "task": candidate_task()},
    ]}}

    with pytest.raises(LFSValidationError, match="unavailable UAV"):
        early_validate_candidate_mission(
            unavailable, available_uav_ids=AVAILABLE_UAV_IDS
        )
    with pytest.raises(LFSValidationError, match="duplicate task_id"):
        early_validate_candidate_mission(
            duplicate, available_uav_ids=AVAILABLE_UAV_IDS
        )


def test_static_validator_requires_availability_and_continuous_successor():
    direct = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "task", "task": candidate_task(),
    }]}}
    continuous = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "task",
        "task": candidate_task(q={"mode": "continuous"}),
    }]}}

    with pytest.raises(LFSValidationError, match="requires available UAV"):
        early_validate_candidate_mission(direct)
    with pytest.raises(LFSValidationError, match="requires a successor"):
        early_validate_candidate_mission(
            continuous, available_uav_ids=AVAILABLE_UAV_IDS
        )


def test_static_validator_accepts_nonterminal_top_level_continuous():
    payload = {"lfs_version": "2.1", "mission": {"nodes": [
        {
            "type": "task",
            "task": candidate_task(q={"mode": "continuous"}),
        },
        {
            "type": "task",
            "task": candidate_task(task_id=2, q={"mode": "direct"}),
        },
    ]}}

    assert early_validate_candidate_mission(
        payload, available_uav_ids=AVAILABLE_UAV_IDS
    ) == payload


def test_static_validator_rejects_continuous_inside_parallel_group():
    payload = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "parallel",
        "completion_mode": "independent",
        "tasks": [
            candidate_task(
                task_id=1, U=[1, 2], F={"type": "Line"},
                q={"mode": "continuous"},
            ),
            candidate_task(
                task_id=2, U=[3, 4], F={"type": "Line"},
            ),
        ],
    }, {
        "type": "task",
        "task": candidate_task(task_id=3),
    }]}}

    with pytest.raises(LFSValidationError, match="ParallelGroup"):
        early_validate_candidate_mission(
            payload, available_uav_ids=AVAILABLE_UAV_IDS
        )
