"""ROS1 boundary regressions absent from the upstream ROS2 suite."""
from types import SimpleNamespace
import rospy
from location_allocate.ros1_runtime import Node
from location_allocate import ros1_runtime
from location_allocate.state_ingest import ingest_standardized_odometry
from location_allocate.state_snapshot import FreshStateSnapshotManager
from nav_msgs.msg import Odometry


def test_recorder_connection_does_not_impersonate_controller(monkeypatch):
    class Publisher:
        resolved_name = '/uav1/execution_command'
        impl = SimpleNamespace(connections=[SimpleNamespace(endpoint_id='/recorder')])
    monkeypatch.setattr(ros1_runtime.rospy, 'Publisher', Publisher)
    node = Node('unused')
    node.resources = [Publisher()]
    assert all(p.node_namespace != '/uav1' for p in node.get_subscriptions_info_by_topic('/uav1/execution_command'))
    Publisher.impl.connections.append(SimpleNamespace(endpoint_id='/uav1/ladrc_position_controller'))
    assert any(p.node_namespace == '/uav1' for p in node.get_subscriptions_info_by_topic('/uav1/execution_command'))
    assert node.get_subscriptions_info_by_topic('/uav2/execution_command') == []


def test_native_ros1_stamp_preserves_source_time_and_freshness():
    msg = Odometry()
    msg.header.stamp = rospy.Time(100, 5000000)
    msg.header.frame_id = 'world'
    msg.child_frame_id = 'uav1/base_link_enu'
    msg.pose.pose.position.z = 3.0
    manager = FreshStateSnapshotManager(.02208, .022043, require_velocity=True, allow_receive_time_fallback=False)
    ingest_standardized_odometry(manager, msg, 1, 100.006)
    state = manager.snapshot([1], 100.010).states[1]
    assert state.source_timestamp == 100.005
    assert state.receive_timestamp == 100.006
    assert state.position == (0.0, 0.0, 3.0)
