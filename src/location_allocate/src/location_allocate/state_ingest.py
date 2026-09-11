"""Validation boundary for standardized nav_msgs/Odometry planning state."""
from __future__ import annotations


def ingest_standardized_odometry(manager, message, uav_id: int, receive_time: float):
    expected_child = f"uav{int(uav_id)}/base_link_enu"
    if message.header.frame_id != "world":
        raise ValueError("swarm_state header.frame_id must be world")
    if message.child_frame_id != expected_child:
        raise ValueError(f"swarm_state child_frame_id must be {expected_child}")
    source_time = (
        float(message.header.stamp.secs)
        + float(message.header.stamp.nsecs) / 1e9
    )
    manager.update(
        int(uav_id),
        [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ],
        receive_time,
        [
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        ],
        source_time,
    )
