# ROS1 迁移阶段 2 计划

日期：2026-06-21

## 1. 目标与策略

项目 B 已经包含一版 ROS1 + MAVROS 移植代码，因此后续不从零重建 package，也不覆盖已有单无人机测试代码。迁移策略是：

- 保留项目 B 现有 `uav_swarm_interfaces`、`ladrc_controller`、`location_allocate` 三个 ROS1 package。
- 以项目 A 为行为参考，补齐项目 B 中仍缺失或风险较高的多无人机仿真能力。
- 优先修复会直接影响 ROS1 运行的配置和 launch 问题。
- 所有修改按原子 commit 推送到 `ros1-multi-uav-migration`。

## 2. 文件处理计划

### 不复制的内容

- 不复制项目 A 的 `build/`、`install/`、`log/`、`__pycache__/`。
- 不复制项目 A 的 `px4_msgs` 到项目 B；项目 B 使用 MAVROS。
- 不复制项目 A 中硬编码的 MiniMax API Key。

### 需要保留/复用的项目 B 内容

- `src/uav_swarm_interfaces/msg/UAVSwarmCommand.msg`
- `src/uav_swarm_interfaces/msg/UAVStatus.msg`
- `src/ladrc_controller/src/ladrc_position_controller_node.cpp`
- `src/ladrc_controller/include/ladrc_controller/*`
- `src/location_allocate/src/location_allocate/*`
- `src/ladrc_controller/launch/single_uav.launch`
- `test_single_uav.sh`
- `Dockerfile`

### 需要改写或补齐的内容

- 将 ROS1 参数文件改成 ROS1 `<rosparam>` 可直接加载的扁平结构。
- 扩展多机 `swarm.launch`，覆盖 UAV1-UAV10，保持 `/uav{N}` namespace。
- 修正单机测试脚本的工作区路径，避免硬编码旧目录。
- 添加多机运行脚本和 topic 检查脚本。
- 添加最终交付文档 `MIGRATION_REPORT.md`、`RUN_ROS1_SIMULATION.md`。

## 3. ROS2 到 ROS1 映射

| 项目 A ROS2 | 项目 B ROS1 |
| --- | --- |
| `ament_cmake` | `catkin` |
| `ament_python` | `catkin_python_setup()` + `catkin_install_python()` |
| `rclcpp::Node` | `ros::NodeHandle` / `roscpp` |
| `rclpy.Node` | 普通 Python 类 + `rospy` |
| `create_publisher` | `ros::Publisher` / `rospy.Publisher` |
| `create_subscription` | `ros::Subscriber` / `rospy.Subscriber` |
| ROS2 parameter | ROS1 private parameter + `<rosparam>` |
| ROS2 launch Python | ROS1 XML `.launch` |
| `rosidl_default_generators` | `message_generation` |

## 4. Topic / Service / Parameter 映射

### 自定义 topic 保持不变

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/uav{N}/swarm_command` | `uav_swarm_interfaces/UAVSwarmCommand` | 调度层向控制器发送目标 |
| `/uav{N}/status` | `uav_swarm_interfaces/UAVStatus` | 控制器反馈悬停状态 |
| `/uav{N}/odom` | `geometry_msgs/Point` | 控制器发布 ENU 全局位置 |

### PX4 / MAVROS 映射

| ROS2 / px4_msgs | ROS1 / MAVROS | 迁移说明 |
| --- | --- | --- |
| `VehicleOdometry` | `/uav{N}/mavros/local_position/odom` | MAVROS 已输出 ENU，不再手动 NED/ENU 转换 |
| `TrajectorySetpoint` | `/uav{N}/mavros/setpoint_position/local` | 发布 `geometry_msgs/PoseStamped` |
| `VehicleCommand` 解锁 | `/uav{N}/mavros/cmd/arming` | 调用 `mavros_msgs/CommandBool` |
| `VehicleCommand` 切模式 | `/uav{N}/mavros/set_mode` | 调用 `mavros_msgs/SetMode` |
| `OffboardControlMode` | 无独立 topic | 通过持续 setpoint 流维持 Offboard |

### 参数映射

ROS2 YAML 中的 `/**: ros__parameters:` 需要改为 ROS1 `<rosparam>` 可加载的普通键值：

- `control_frequency`
- `omega_o_x/y/z`
- `omega_c_x/y/z`
- `b0_x/y/z`
- `max_velocity`
- `max_acceleration_x/y/z`
- `iapf_safe_distance`
- `iapf_repulsion_gain`
- `neighbor_offset_multiplier`
- `enu_offset_x/y/z` 由 launch 覆盖

## 5. 多无人机 namespace 与 launch 设计

- namespace 规则保持 `/uav{N}`，N 从 1 到 10。
- 每架无人机启动：
  - `/uav{N}/mavros`
  - `/uav{N}/ladrc_position_controller`
- SITL 默认 MAVROS 端口：
  - 本地端口：`14540 + 10 * (N - 1)`
  - PX4 远端端口：`14579 + N`
- `target_system_id = N`。
- Gazebo spawn 偏移沿 Y 轴补偿：`enu_offset_y = 3.0 * N`。
- 实机模式保留 `use_sim:=false`，但单个 `fcu_url_real` 不能同时代表多台实机；多机实机接入需后续按硬件 IP/串口单独配置。

## 6. 验证方案

### 编译验证

优先使用 Docker：

```bash
sudo docker run --rm --network host -v $(pwd):/ros1_ws ros1-mavros:latest \
  bash -c "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"
```

检查：

- `catkin_make` 零 error。
- 生成 `devel/lib/ladrc_controller/ladrc_position_controller_node`。
- `rosmsg show uav_swarm_interfaces/UAVSwarmCommand` 可显示字段。

### 单机仿真验证

- 宿主机运行 `make px4_sitl gazebo-classic`。
- 项目 B 运行 `test_single_uav.sh` 或 `roslaunch ladrc_controller single_uav.launch`。
- 检查 `/uav1/mavros/state`、`/uav1/odom`、`/uav1/status`。

### 多机仿真验证

- 宿主机使用 PX4 多机 SITL，如 `sitl_multiple_run.sh -n 3`。
- ROS1 侧运行多机 launch 或脚本。
- 使用检查脚本确认：
  - 每架无人机有独立 `/uav{N}/mavros/state`。
  - 每架无人机有独立 `/uav{N}/mavros/setpoint_position/local`。
  - 每架无人机有独立 `/uav{N}/status` 和 `/uav{N}/odom`。
  - 不存在明显 topic 冲突。

## 7. 可能失败的点

- 宿主机没有 PX4-Autopilot 或多机 SITL 脚本路径不同。
- Docker 镜像未构建或 Docker 权限不足。
- Gazebo GUI、GPU、DISPLAY 权限导致 GUI 无法打开。
- MAVROS 端口与 PX4 实例端口不匹配。
- ROS1 参数文件如果仍为 ROS2 YAML 结构，控制器会使用默认参数而不是配置参数。
- 多机实机模式需要每架无人机不同 `fcu_url`，不能依赖当前单一 `fcu_url_real`。
- LLM 调度层依赖 `MINIMAX_API_KEY` 和网络访问，无法在无 key 环境自动端到端验证。

## 8. 后续交付顺序

1. 修正 ROS1 参数文件和路径硬编码。
2. 补齐 UAV1-UAV10 多机 launch。
3. 添加多机启动脚本与 topic 检查脚本。
4. 添加最终交付文档。
5. 运行 Docker/catkin 编译验证。
6. 在无法启动完整 Gazebo 的情况下，至少提供 headless 与 topic 检查命令。
