"""ROS message construction from resolved execution tasks."""
from __future__ import annotations

from typing import Any

from .late_resolution import ResolvedExecutionTask


def build_execution_command(
    resolved: ResolvedExecutionTask,
    index: int,
    mission_id: int,
    task_id: int,
    group_id: int = 0,
    stamp: Any = None,
    command_type=None,
):
    """Build a composite command without introducing another duration field."""
    if command_type is None:
        from uav_swarm_interfaces.msg import UAVExecutionCommand

        command_type = UAVExecutionCommand

    command = command_type()
    if stamp is not None:
        command.header.stamp = stamp
    command.header.frame_id = "world"
    command.mission_id = int(mission_id)
    command.task_id = int(task_id)
    command.group_id = int(group_id)
    command.uav_id = int(resolved.executable_lfs.uav_ids[index])
    target = resolved.assigned_targets[index]
    command.target_pos.x = target[0]
    command.target_pos.y = target[1]
    command.target_pos.z = target[2]
    source = resolved.profiles[index]
    command.profile.duration = source.duration
    command.profile.style = source.style
    command.profile.omega_c = list(source.omega_c)
    command.profile.omega_o = list(source.omega_o)
    command.profile.velocity_limit = source.velocity_limit
    command.profile.acceleration_limit = source.acceleration_limit
    command.profile.jerk_limit = source.jerk_limit
    command.profile.iapf_enter_distance = source.iapf_enter_distance
    command.profile.iapf_exit_distance = source.iapf_exit_distance
    command.profile.iapf_repulsion_scale = source.iapf_repulsion_scale
    command.profile.style_gain = source.style_gain
    command.profile.task_gain = source.task_gain
    command.profile.configuration_id = source.configuration_id
    return command


def build_task_command_batch(
    resolved,
    mission_id,
    task_id,
    group_id=0,
    stamp=None,
    command_type=None,
):
    """Build a complete task batch before the caller performs any publish."""
    return tuple(
        build_execution_command(
            resolved, index, mission_id, task_id, group_id, stamp, command_type
        )
        for index in range(len(resolved.executable_lfs.uav_ids))
    )


def build_parallel_command_batch(
    resolved_group,
    mission_id,
    group_id,
    stamp=None,
    command_type=None,
):
    """All-or-nothing construction boundary for a resolved parallel group."""
    commands = []
    for resolved in resolved_group.tasks:
        commands.extend(build_task_command_batch(
            resolved,
            mission_id,
            resolved.trace.task_id,
            group_id,
            stamp,
            command_type,
        ))
    return tuple(commands)
