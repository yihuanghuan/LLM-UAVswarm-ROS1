from pathlib import Path
from types import SimpleNamespace

import pytest

from location_allocate.candidate_mission_runtime import execute_candidate_payload
from location_allocate.execution_command_builder import (
    build_parallel_command_batch,
    build_task_command_batch,
)
from location_allocate.late_resolution import (
    resolve_execution_parallel,
    resolve_execution_task,
)
from location_allocate.mission_executor import MissionRuntimeCallbacks
from location_allocate.policy_adapter import load_runtime_policy
from location_allocate.state_snapshot import FreshStateSnapshotManager


PAPER_CURRENT = (
    Path(__file__).parents[2]
    / "lfs_policy"
    / "config"
    / "lfs_policy.paper_current.yaml"
)


def command_type():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=None, frame_id=""),
        target_pos=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        profile=SimpleNamespace(),
    )


def task(task_id, center_mode=None):
    return {
        "task_id": task_id,
        "U": [1, 2, 3],
        "F": {"type": "Triangle"},
        "c": center_mode or {"mode": "auto"},
        "r": {"mode": "auto"},
        "T": {"mode": "auto"},
        "m": "normal",
        "s": 1.0,
        "q": {"mode": "direct"},
    }


def snapshot(epoch, x_offset=0.0, ids=(1, 2, 3)):
    radius = 2.0 / (3.0 ** 0.5)
    positions = {
        1: [radius + x_offset, 0.0, 2.0],
        2: [-radius / 2.0 + x_offset, 1.0, 2.0],
        3: [-radius / 2.0 + x_offset, -1.0, 2.0],
        4: [5.0 + x_offset, 0.0, 2.0],
        5: [8.0 + x_offset, 0.0, 2.0],
    }
    manager = FreshStateSnapshotManager(
        0.5, 0.15, require_velocity=True,
        allow_receive_time_fallback=False,
    )
    for uid in ids:
        manager.update(uid, positions[uid], epoch, [0, 0, 0], epoch)
    return manager.snapshot(ids, epoch)


def parallel_snapshot(epoch=10.0):
    manager = FreshStateSnapshotManager(
        0.5, 0.15, require_velocity=True,
        allow_receive_time_fallback=False,
    )
    for uid, position in {
        1: [-1.5, 0.0, 2.0],
        2: [1.5, 0.0, 2.0],
        4: [5.5, 0.0, 2.0],
        5: [8.5, 0.0, 2.0],
    }.items():
        manager.update(uid, position, epoch, [0, 0, 0], epoch)
    return manager.snapshot([1, 2, 4, 5], epoch)


def test_natural_language_fixture_reaches_execution_commands_with_late_snapshot():
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    payload = {"lfs_version": "2.1", "mission": {"nodes": [
        {"type": "task", "task": task(1)},
        {"type": "task", "task": task(2)},
    ]}}
    snapshots = iter([snapshot(10.0), snapshot(20.0, x_offset=4.0)])
    resolved = []
    commands = []

    def execute_task(machine):
        result = resolve_execution_task(machine.task, next(snapshots), policy)
        resolved.append(result)
        commands.extend(build_task_command_batch(
            result, 7, machine.task["task_id"], command_type=command_type
        ))

    execute_candidate_payload(
        payload,
        MissionRuntimeCallbacks(execute_task, lambda *_args: None, lambda _wait: None),
        available_uav_ids=[1, 2, 3],
    )

    assert len(commands) == 6
    assert resolved[0].executable_lfs.center == pytest.approx((0.0, 0.0, 2.0))
    assert resolved[1].executable_lfs.center == pytest.approx((4.0, 0.0, 2.0))
    assert resolved[0].trace.state_timestamps[1] == {
        "source": 10.0,
        "receive": 10.0,
        "effective_source": "source_timestamp",
    }
    assert all(
        command.profile.duration == result.executable_lfs.duration
        for result, task_commands in zip(resolved, (commands[:3], commands[3:]))
        for command in task_commands
    )
    assert {command.profile.configuration_id for command in commands} == {
        "paper-current-v11-c0-f-frozen"
    }
    assert len(resolved[0].trace.policy_hash) == 64
    assert resolved[0].trace.schema_version == "paper-candidate-schema-v2"
    assert resolved[0].trace.geometry_version == "paper-unit-geometry-v3"
    assert resolved[0].trace.allocator_mode == "lexicographic-safety-aware-v2"
    assert resolved[0].trace.code_git_sha != ""


def test_parallel_group_uses_one_snapshot_and_builds_one_atomic_batch():
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    first = {
        **task(1, {"mode": "absolute", "value": [0, 0, 2]}),
        "U": [1, 2],
        "F": {"type": "Line"},
    }
    second = {
        **task(2, {"mode": "absolute", "value": [7, 0, 2]}),
        "U": [4, 5],
        "F": {"type": "Line"},
        "s": 2.0,
    }
    shared = parallel_snapshot()
    calls = []

    def execute_parallel(machines, completion_mode):
        calls.append(shared)
        result = resolve_execution_parallel(
            [machine.task for machine in machines],
            shared,
            policy,
            completion_mode,
            group_d_plan=max(policy.resolve_safety(1).d_plan,
                             policy.resolve_safety(2).d_plan),
        )
        batch = build_parallel_command_batch(
            result, 9, 1, command_type=command_type
        )
        assert len(batch) == 4

    payload = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "parallel",
        "completion_mode": "independent",
        "tasks": [first, second],
    }]}}
    execute_candidate_payload(
        payload,
        MissionRuntimeCallbacks(lambda _machine: None, execute_parallel,
                                lambda _wait: None),
        available_uav_ids=[1, 2, 3, 4, 5],
    )

    assert calls == [shared]


def test_parallel_resolution_failure_produces_zero_publishable_commands():
    _config, policy = load_runtime_policy(PAPER_CURRENT)
    good = {
        **task(1, {"mode": "absolute", "value": [0, 0, 2]}),
        "U": [1, 2],
        "F": {"type": "Line"},
    }
    bad = {
        **task(2, {"mode": "absolute", "value": [100, 0, 2]}),
        "U": [4, 5],
        "F": {"type": "Line"},
    }
    published = []

    with pytest.raises(Exception):
        result = resolve_execution_parallel(
            [good, bad], parallel_snapshot(), policy,
            "independent", group_d_plan=2.0,
        )
        published.extend(build_parallel_command_batch(
            result, 1, 1, command_type=command_type
        ))

    assert published == []
