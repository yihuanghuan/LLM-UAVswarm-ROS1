#!/usr/bin/env python3
"""
vrpn_vision_relay.py — VRPN 动捕 → MAVROS 视觉位姿转发节点

将 vrpn_client_ros 输出的刚体 pose 转发到 MAVROS vision_pose/pose,
使 PX4 EKF2 可以融合外部视觉数据 (Nokov 动捕)。

话题流向:
  /vrpn_client_node/<rigid_body>/pose  (geometry_msgs/PoseStamped)
         ↓  本节点转发
  /uav{N}/mavros/vision_pose/pose      (geometry_msgs/PoseStamped)
         ↓  MAVROS vision_pose_estimate 插件
  MAVLink VISION_POSITION_ESTIMATE     →  ESP32 →  NxtPX4v2 飞控

用法:
  rosrun ladrc_controller vrpn_vision_relay.py \
    _vrpn_topic:=/vrpn_client_node/UAV1/pose \
    _vision_topic:=mavros/vision_pose/pose
"""

import rospy
from geometry_msgs.msg import PoseStamped


class VRPNVisionRelay:
    def __init__(self):
        # 参数: VRPN 输入话题 (完整路径)
        vrpn_topic = rospy.get_param("~vrpn_topic",
                                     "/vrpn_client_node/UAV1/pose")
        # 参数: MAVROS vision 输出话题 (相对路径, 在 UAV 命名空间内)
        vision_topic = rospy.get_param("~vision_topic",
                                       "mavros/vision_pose/pose")

        self.pub = rospy.Publisher(vision_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(vrpn_topic, PoseStamped, self._callback)

        rospy.loginfo("VRPN Vision 转发已启动: {} → {}".format(
            vrpn_topic, vision_topic))

    def _callback(self, msg):
        # VRPN 输出的 frame_id 通常为 "world" 或动捕坐标系
        # MAVROS vision_pose 期望 frame_id 为 "map" (ENU)
        # 如果坐标系不一致, 需要额外的 tf 变换
        rospy.logdebug("转发 vision pose: [{:.3f}, {:.3f}, {:.3f}]".format(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z))
        self.pub.publish(msg)


def main():
    rospy.init_node("vrpn_vision_relay", log_level=rospy.INFO)
    relay = VRPNVisionRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
