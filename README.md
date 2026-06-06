# LLM-Driven Multi-UAV Swarm Control — ROS1 + MAVROS 移植版

从 [ROS2 原版](https://github.com/yihuanghuan/LLM-UAVswarm-performance) 移植到 ROS1 Noetic + MAVROS。

## 主要变化

| 原版 (ROS2) | 移植版 (ROS1) |
|------------|------------|
| rclcpp + px4_msgs + MicroXRCEAgent | roscpp + mavros_msgs + MAVROS |
| NED 坐标 (手动转换) | ENU 坐标 (MAVROS 原生) |
| 手动 OffboardControlMode 心跳 | MAVROS 自动维持 |
| VehicleCommand 发布 | mavros_msgs 服务调用 |
| 纯定时延迟状态机 | /mavros/state 反馈状态机 |
| rclpy + Node 继承 | rospy + 普通类 |

## 核心算法（保持不变）

- LADRC 自抗扰控制器 (LESO + LSEF)
- Minimum Jerk 5 次多项式轨迹
- IAPF 分布式避障
- FormationGenerator 编队坐标生成
- TopologyAllocator 匈牙利防交叉分配
- LLM 自然语言指令解析

## 编译

```bash
cd ~/ros1_ws
catkin_make
source devel/setup.bash
```

## 单机测试

详见 [VERIFICATION.md](VERIFICATION.md)
