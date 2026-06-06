# ROS1 + MAVROS 单机仿真与实机测试 — 统一验证方案

## ⚡ 快速开始（两种方式）

| 方式 | 适用场景 | 前置条件 |
|------|---------|---------|
| **Docker 测试**（推荐本机验证） | Ubuntu 22.04 宿主机，ROS1 在容器中，PX4+Gazebo 在宿主机 | 已安装 Docker、PX4-Autopilot |
| **原生 ROS1 测试** | Ubuntu 20.04 原生安装 ROS1 Noetic | 已安装 ROS1、MAVROS、PX4-Autopilot |

---

## 方式 A: Docker 容器测试（本机 Ubuntu 22.04）

本机已通过全链路验证（2026-06-06），架构如下：

```
宿主机 (Ubuntu 22.04)                  Docker 容器 (ros1-mavros)
┌──────────────────────┐    UDP     ┌──────────────────────────┐
│ PX4 SITL + Gazebo    │◄───14580──►│ MAVROS + Controller      │
│ (原生运行)            │  MAVLink   │ (ros:noetic-ros-base)    │
└──────────────────────┘            └──────────────────────────┘
```

### A.1 构建镜像（仅首次）

```bash
cd ~/ros1_ws
sudo docker build -t ros1-mavros:latest .
```

### A.2 启动测试

```bash
# 终端 1: 启动 PX4 SITL + Gazebo（带图形界面看无人机飞行）
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
# 等待出现: "INFO  [commander] Ready for takeoff!"

# 终端 2: 启动 ROS1 环境
cd ~/ros1_ws
bash test_single_uav.sh
# 等待出现: "Offboard 模式已激活。LADRC 控制器接管。"

# 终端 3: 进入容器发送指令
sudo docker exec -it ros1_test bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash

# 发送飞行指令（注意: YAML 内层用单引号，外层双引号）
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: 'world'}, uav_id: 1, \
    target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, \
    motion_style: 'normal', safety_factor: 0.0}"
```

### A.3 监控命令

```bash
# 在容器内执行:
# 查看控制器日志
tail -f /tmp/controller2.log

# 查看 MAVROS 状态
rostopic echo /uav1/mavros/state -n 1

# 查看悬停状态
rostopic echo /uav1/status

# 查看 ENU 位置
rostopic echo /uav1/odom
```

### A.4 停止测试

```bash
sudo docker rm -f ros1_test
# PX4 SITL 在终端 1 按 Ctrl+C 停止
```

---

## 方式 B: 原生 ROS1 测试（实验室 Ubuntu 20.04）

### B.1 编译

```bash
git clone git@github.com:yihuanghuan/LLM-UAVswarm-ROS1.git ~/ros1_ws
cd ~/ros1_ws
catkin_make
source devel/setup.bash
```

编译通过标准：
- [ ] `catkin_make` 零 error
- [ ] `devel/include/uav_swarm_interfaces/UAVSwarmCommand.h` 生成
- [ ] `devel/lib/ladrc_controller/ladrc_position_controller_node` 生成

### B.2 消息格式验证

```bash
source devel/setup.bash
rosmsg show uav_swarm_interfaces/UAVSwarmCommand
# 预期: std_msgs/Header header, uint8 uav_id, geometry_msgs/Point target_pos, ...
rosmsg show uav_swarm_interfaces/UAVStatus
# 预期: uint8 uav_id, bool is_hover_stable
```

### B.3 启动测试（4 终端）

**终端 1: PX4 SITL**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

**终端 2: MAVROS + 控制器**
```bash
cd ~/ros1_ws && source devel/setup.bash
roslaunch ladrc_controller single_uav.launch uav_id:=1 enu_offset_y:=0.0
```

**终端 3: 发送飞行指令**
```bash
source ~/ros1_ws/devel/setup.bash
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: 'world'}, uav_id: 1, \
    target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, \
    motion_style: 'normal', safety_factor: 0.0}"
```

**终端 4: 监控**
```bash
source ~/ros1_ws/devel/setup.bash
rostopic echo /uav1/status          # 悬停状态
rostopic echo /uav1/odom            # ENU 位置
rostopic echo /uav1/mavros/state    # PX4 状态
```

---

## 测试通过标准 (T1-T8)

| 测试项 | 通过标准 | 检查方法 |
|--------|---------|---------|
| T1: MAVROS 连接 | 日志出现 "已接收到 MAVROS local_position/odom" | `rostopic echo /uav1/mavros/state` → `connected: True` |
| T2: 自动起飞 | 日志出现 "Offboard 模式已激活。LADRC 控制器接管。" | Gazebo 中无人机升空 |
| T3: 悬停保持 | 日志出现 "悬停保持锁定" | 无人机在空中稳定，无明显漂移 |
| T4: 接收指令 | 日志出现 "swarm_cmd 回调触发" | 发送指令后立即触发 |
| T5: 轨迹跟踪 | 日志持续输出 `Ref[x,y,z] Pos[x,y,z] Cmd[x,y,z]` | Gazebo 中无人机向目标移动 |
| T6: 到达目标 | 日志出现 "悬停稳定! pos_err=...m" | 无人机停在目标附近 |
| T7: 状态反馈 | `/uav1/status` 中 `is_hover_stable: True` | `rostopic echo /uav1/status` 验证 |
| T8: 位置反馈 | `/uav1/odom` 输出 ENU 坐标接近目标 | `rostopic echo /uav1/odom` → x≈3, y≈0, z≈3 |

---

## 调度器端到端测试

在控制器运行时（T2 之后），启动调度器：

```bash
source ~/ros1_ws/devel/setup.bash
rosrun location_allocate location_allocate_node
```

输入：
```
UAV1 以[3,0,3]为中心，变换成圆形编队，半径为0米，限时5秒
```

| 测试项 | 通过标准 |
|--------|---------|
| S1: LLM 解析 | 输出 JSON 蓝图，`task_sequences[0].uav_id=[1]` |
| S2: 命令发送 | 输出 "UAV1 -> [3.0, 0.0, 3.0]" |
| S3: 悬停等待 | 输出 "UAV1 到达目标并悬停稳定!" |
| S4: 任务完成 | 输出 "所有任务序列执行完毕" |

---

## 运动风格验证

相同目标，不同风格，观察飞行时间：

```bash
# Smooth (最慢)
rostopic pub -1 /uav1/swarm_command ... "{..., motion_style: 'smooth', ...}"
# Aggressive (最快)
rostopic pub -1 /uav1/swarm_command ... "{..., motion_style: 'aggressive', ...}"
```

通过标准: Smooth 到达时间 > Normal 到达时间 > Aggressive 到达时间

---

## 代码移植对照 (ROS2 → ROS1)

| 关注点 | ROS2 原版 | ROS1 移植版 |
|--------|----------|------------|
| 坐标转换 | NED↔ENU 手动转换 | 无需转换 (MAVROS ENU) |
| Offboard 心跳 | 手动 publishOffboardControlMode() | 删除 (MAVROS 自动) |
| 解锁 | publishVehicleCommand(ARM_DISARM) | arming_client_.call(CommandBool) |
| 切模式 | publishVehicleCommand(DO_SET_MODE) | set_mode_client_.call(SetMode) |
| 状态机反馈 | 纯定时延迟 | 基于 /mavros/state 真实反馈 |
| 位置设定点 | TrajectorySetpoint (NED) | PoseStamped (ENU) |
| Python spin | rclpy.spin_once(self, ...) | rospy.sleep(...) |
| 订阅回调参数 | lambda 闭包捕获 uid | callback_args=uid |

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 控制器收不到 odom | MAVROS 未连接 PX4 | `rostopic list \| grep mavros` 确认话题存在 |
| 解锁失败 | 安全开关/预检 | PX4 控制台 `commander prearm` 检查 |
| Offboard 切换失败 | setpoint 流 < 2Hz | 确认控制节点 50Hz 定时器正常 |
| 调度器 import 失败 | 未 source | `source devel/setup.bash` |
| rostopic pub YAML 报错 | 引号嵌套冲突 | **外层双引号，内层单引号**: `"{key: 'value'}"` |
| Docker 容器日志中文乱码 | 终端编码 | 不影响功能，控制器正常运行 |
| /uav1/odom 坐标不对 | spawn 偏移 | 单机 SITL 时 `enu_offset_y=0.0` |

---

## 实机飞行前检查清单

> ⚠️ **通过全部 T1-T8 SITL 测试后**，才能进行实机飞行。

### 硬件检查
- [ ] PX4 固件 MAVLink 串口已配置 (`MAV_1_CONFIG = TELEM2`)
- [ ] 机载计算机与 Pixhawk 串口/USB 连接正常
- [ ] 安全开关已按下，遥控器就绪
- [ ] 电池电压充足，低电量保护已配置
- [ ] 螺旋桨安装牢固，电机转向正确

### 软件配置
- [ ] MAVROS `fcu_url` 改为串口地址 (如 `/dev/ttyACM0:921600`)
- [ ] `enu_offset_y=0.0`（实机无 Gazebo spawn 偏移）
- [ ] `enu_offset_x=0.0`, `enu_offset_z=0.0`
- [ ] LADRC 参数已针对实机调低（先用保守值: ωo=8, ωc=2）

### 动捕系统
- [ ] Nokov 动捕系统已标定，VRPN 数据正常广播
- [ ] `vrpn_client_ros` 在机载计算机上正常运行
- [ ] PX4 `EKF2_EV_CTRL` 使能外部视觉融合
- [ ] PX4 `EKF2_HGT_REF` 设为 vision（若用动捕作高度参考）

### 安全措施
- [ ] 第一次飞行: 位置阶跃 ±1m，限时 5s
- [ ] 紧急降落: 遥控器切 Stabilized 模式
- [ ] 至少 2 人在场（操作员 + 安全观察员）
- [ ] 飞行区域清场，防护网就绪
