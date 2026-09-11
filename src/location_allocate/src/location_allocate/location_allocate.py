"""ROS1 entry point for the paper Candidate mission runtime."""
from __future__ import annotations

import json
from pathlib import Path

from . import ros1_runtime as ros_api
from nav_msgs.msg import Odometry
from .ros1_runtime import Node
from .ros1_runtime import qos_profile_sensor_data
from uav_swarm_interfaces.msg import UAVExecutionCommand, UAVStatus

from .completion_tracker import CompletionGenerationTracker
from .paper_candidate_parser import CandidateParseError, parse_candidate_mission
from .policy_adapter import load_runtime_policy
from .state_ingest import ingest_standardized_odometry
from .state_snapshot import FreshStateSnapshotManager


ALL_UAV_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


def _default_paper_policy_path():
    try:
        from rospkg import RosPack
        get_package_share_directory = RosPack().get_path

        return str(
            Path(get_package_share_directory("lfs_policy"))
            / "config"
            / "lfs_policy.paper_current.yaml"
        )
    except Exception:
        return str(
            Path(__file__).resolve().parents[2]
            / "lfs_policy"
            / "config"
            / "lfs_policy.paper_current.yaml"
        )


class UAVFormationNode(Node):
    """Own ROS resources and delegate to exactly one selected runtime."""

    def __init__(self):
        super().__init__("location_allocate")
        self.declare_parameter("assignment_mode", "safety_aware")
        self.declare_parameter("lfs_runtime_mode", "candidate_v2")
        self.declare_parameter("lfs_policy_file", "")
        self.declare_parameter("uav_ids", ALL_UAV_IDS)
        self.declare_parameter("candidate_completion_timeout", 120.0)
        # Orchestration readiness only. Accepted snapshots still use the
        # frozen C0-B state_timeout/skew predicate without alteration.
        self.declare_parameter("candidate_dispatch_readiness_timeout", 15.0)
        self.declare_parameter("candidate_state_timeout", -1.0)
        self.declare_parameter("candidate_snapshot_skew", -1.0)

        assignment_mode = self.get_parameter(
            "assignment_mode"
        ).get_parameter_value().string_value
        if assignment_mode not in ("fixed", "distance_hungarian", "safety_aware"):
            raise ValueError(
                "assignment_mode must be fixed, distance_hungarian, or safety_aware"
            )
        self.runtime_mode = self.get_parameter(
            "lfs_runtime_mode"
        ).get_parameter_value().string_value
        if self.runtime_mode != "candidate_v2":
            raise ValueError("Only candidate_v2 is supported")
        self.available_uav_ids = [
            int(value) for value in self.get_parameter("uav_ids").value
        ]
        if not self.available_uav_ids or len(self.available_uav_ids) != len(
            set(self.available_uav_ids)
        ):
            raise ValueError("uav_ids must be a non-empty unique list")

        self.uav_hover_status = {
            uid: False for uid in self.available_uav_ids
        }
        self.completion_tracker = CompletionGenerationTracker()
        self.status_sub = {}
        for uid in self.available_uav_ids:
            self.status_sub[uid] = self.create_subscription(
                UAVStatus,
                f"/uav{uid}/status",
                lambda msg, uid=uid: self._status_callback(msg, uid),
                10,
            )

        self._configure_paper_runtime()

    def _configure_paper_runtime(self):
        from .paper_runtime import PaperMissionRuntime

        configured = self.get_parameter(
            "lfs_policy_file"
        ).get_parameter_value().string_value
        policy_path = configured or _default_paper_policy_path()
        policy_config, candidate_policy = load_runtime_policy(policy_path)
        state = policy_config.state
        self.snapshot_manager = FreshStateSnapshotManager(
            state.state_timeout,
            state.snapshot_skew,
            require_velocity=state.require_velocity,
            allow_receive_time_fallback=state.allow_receive_time_fallback,
        )
        self.execution_publisher = {}
        self.swarm_state_sub = {}
        for uid in self.available_uav_ids:
            self.execution_publisher[uid] = self.create_publisher(
                UAVExecutionCommand, f"/uav{uid}/execution_command", 10
            )
            self.swarm_state_sub[uid] = self.create_subscription(
                Odometry,
                f"/uav{uid}/swarm_state",
                lambda msg, uid=uid: self._swarm_state_callback(msg, uid),
                qos_profile_sensor_data,
            )
        self.paper_runtime = PaperMissionRuntime(
            self,
            policy_config,
            candidate_policy,
            self.snapshot_manager,
            self.execution_publisher,
            self.completion_tracker,
            self.available_uav_ids,
        )
        self.get_logger().info(
            f"Paper Candidate runtime enabled with policy "
            f"{policy_config.configuration_id}"
        )
        for warning in policy_config.warnings:
            self.get_logger().warn(f"Paper policy audit: {warning}")


    def _status_callback(self, msg, uid):
        self.uav_hover_status[uid] = msg.is_hover_stable
        self.completion_tracker.update(uid, msg.is_hover_stable)


    def _swarm_state_callback(self, msg, uid):
        receive_timestamp = self.get_clock().now().nanoseconds / 1e9
        try:
            ingest_standardized_odometry(
                self.snapshot_manager, msg, uid, receive_timestamp
            )
        except ValueError as exc:
            self.get_logger().error(f"拒绝 UAV{uid} swarm_state: {exc}")

    def run_candidate_mission(self, payload):
        if self.paper_runtime is None:
            raise RuntimeError("Candidate runtime is not enabled")
        return self.paper_runtime.run(payload)



def execute_runtime_payload(node, payload):
    """Dispatch only validated Candidate missions."""
    if node.runtime_mode != "candidate_v2":
        raise ValueError(f"unsupported runtime mode: {node.runtime_mode}")
    return node.run_candidate_mission(payload)


def main():
    ros_api.init()
    node = UAVFormationNode()
    paper_ros = (
        f"Available UAV IDs: {node.available_uav_ids}\n"
        f"Total available UAVs: {len(node.available_uav_ids)}"
    )
    try:
        while True:
            user_command = input("\n请输入无人机编队指令: ")
            if user_command.strip().lower() in ("exit", "quit", "q"):
                break
            if not user_command.strip():
                continue
            try:
                result = parse_candidate_mission(user_command, paper_ros)
            except CandidateParseError as exc:
                node.get_logger().error(str(exc))
                continue
            print(json.dumps(result, indent=2, ensure_ascii=False))
            try:
                execute_runtime_payload(node, result)
            except Exception as exc:
                node.get_logger().error(
                    f"Candidate 任务执行失败: {exc}"
                )
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，停止任务")
    finally:
        node.destroy_node()
        ros_api.shutdown()


if __name__ == "__main__":
    main()
