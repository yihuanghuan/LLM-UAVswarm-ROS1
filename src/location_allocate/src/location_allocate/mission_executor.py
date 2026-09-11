"""Mission graph runtime and the distinct per-task state-machine compiler."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Tuple

from .lfs_types import (
    CompiledMission,
    CompiledParallelGroup,
    CompiledTaskNode,
    TaskStateMachine,
    WaitSpec,
)


TASK_EXECUTION_STATES = (
    "waiting_for_fresh_snapshot",
    "runtime_validation",
    "deterministic_resolution",
    "unit_geometry",
    "scale_resolution",
    "final_geometry",
    "planning_timing",
    "allocation",
    "final_timing",
    "execution_profile_compilation",
    "executing",
    "waiting_for_completion_event",
    "complete",
)


def compile_task_state_machine(node: CompiledTaskNode) -> TaskStateMachine:
    """Compile execution states without re-reading q from the Candidate task."""
    return TaskStateMachine(
        task=dict(node.task),
        states=TASK_EXECUTION_STATES,
        completion_event=node.completion_event,
        post_completion_wait=node.wait,
    )


@dataclass(frozen=True)
class MissionRuntimeCallbacks:
    """Side-effect boundary supplied by ROS or a deterministic test harness."""

    execute_task: Callable[[TaskStateMachine], None]
    execute_parallel: Callable[
        [Tuple[TaskStateMachine, ...], str], None
    ]
    execute_wait: Callable[[WaitSpec], None]


def execute_compiled_mission(
    mission: CompiledMission, callbacks: MissionRuntimeCallbacks
) -> None:
    """Execute graph order; q is already consumed by Mission Compiler."""
    for node in mission.nodes:
        if isinstance(node, CompiledTaskNode):
            machine = compile_task_state_machine(node)
            callbacks.execute_task(machine)
            if machine.post_completion_wait is not None:
                callbacks.execute_wait(machine.post_completion_wait)
        elif isinstance(node, CompiledParallelGroup):
            machines = tuple(
                compile_task_state_machine(task) for task in node.tasks
            )
            # The parallel runtime owns each machine's completion/wait timing;
            # replaying those waits here would serialize independent tasks.
            callbacks.execute_parallel(machines, node.completion_mode)
        elif isinstance(node, WaitSpec):
            callbacks.execute_wait(node)
        else:
            raise TypeError(f"unsupported compiled mission node: {type(node)!r}")
