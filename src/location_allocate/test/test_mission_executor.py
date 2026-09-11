from location_allocate.lfs_types import (
    CompiledMission,
    CompiledParallelGroup,
    CompiledTaskNode,
    WaitSpec,
)
from location_allocate.mission_executor import (
    MissionRuntimeCallbacks,
    compile_task_state_machine,
    execute_compiled_mission,
)


def test_task_fsm_uses_compiled_completion_event_not_candidate_q():
    node = CompiledTaskNode(
        task={"task_id": 1, "q": "candidate-value-must-not-be-read"},
        completion_event="hover_stable",
        wait=WaitSpec("elapsed", 2.0),
    )

    machine = compile_task_state_machine(node)

    assert machine.completion_event == "hover_stable"
    assert machine.post_completion_wait == WaitSpec("elapsed", 2.0)


def test_graph_runtime_preserves_sequential_parallel_and_wait_nodes():
    first = CompiledTaskNode({"task_id": 1}, "complete", None)
    parallel = CompiledParallelGroup(
        tasks=(
            CompiledTaskNode({"task_id": 2}, "hover", None),
            CompiledTaskNode({"task_id": 3}, "hover", None),
        ),
        completion_mode="independent",
    )
    wait = WaitSpec("elapsed", 1.0)
    events = []
    callbacks = MissionRuntimeCallbacks(
        execute_task=lambda machine: events.append(
            ("task", machine.task["task_id"])
        ),
        execute_parallel=lambda machines, mode: events.append(
            ("parallel", tuple(m.task["task_id"] for m in machines), mode)
        ),
        execute_wait=lambda spec: events.append(
            ("wait", spec.condition, spec.duration)
        ),
    )

    execute_compiled_mission(CompiledMission((first, parallel, wait)), callbacks)

    assert events == [
        ("task", 1),
        ("parallel", (2, 3), "independent"),
        ("wait", "elapsed", 1.0),
    ]
