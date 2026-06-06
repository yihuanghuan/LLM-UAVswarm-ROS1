# ROS1 + MAVROS 单机实机测试 — 验证方案与测试标准

## 0. 环境要求

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 20.04 | ROS1 Noetic 官方支持版本 |
| ROS1 | Noetic | `ros-noetic-desktop-full` |
| MAVROS | 1.x | `ros-noetic-mavros` + `ros-noetic-mavros-extras` |
| PX4 | v1.14+ | SITL 仿真测试用 `make px4_sitl gazebo-classic` |
| Gazebo | Classic 11 | SITL 仿真 |
| Python | 3.8 | `numpy`, `scipy`, `openai`, `httpx` |

---

## 1. 编译验证

### 1.1 创建工作空间并编译

```bash
# 克隆 ROS1 移植代码
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
git clone <this-repo-url> .

# 安装 MAVROS (若未安装)
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
# 安装 GeographicLib (MAVROS 依赖)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Python 依赖
pip install numpy scipy openai httpx

# 编译
cd ~/ros1_ws
catkin_make

# 预期输出:
#   [100%] Built target ladrc_position_controller_node
#   无编译错误 (允许 warning)
source devel/setup.bash
```

### 1.2 编译通过标准

- [ ] `catkin_make` 零 error 退出
- [ ] 生成消息头文件：`devel/include/uav_swarm_interfaces/UAVSwarmCommand.h`
- [ ] 生成可执行文件：`devel/lib/ladrc_controller/ladrc_position_controller_node`
- [ ] 生成 Python 包：`devel/lib/python3/dist-packages/location_allocate/`

---

## 2. 消息格式验证

```bash
# 验证消息定义正确
rosmsg show uav_swarm_interfaces/UAVSwarmCommand
# 预期输出:
#   std_msgs/Header header
#   uint8 uav_id
#   geometry_msgs/Point target_pos
#   float32 duration
#   string motion_style
#   float32 safety_factor

rosmsg show uav_swarm_interfaces/UAVStatus
# 预期输出:
#   uint8 uav_id
#   bool is_hover_stable
```

---

## 3. SITL 单机仿真测试

### 3.1 启动顺序（4 个终端）

**终端 1: 启动 PX4 SITL**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
# 等待出现: "INFO  [commander] Ready for takeoff!"
# PX4 SITL 默认 MAVLink UDP 端口: 14580
```

**终端 2: 启动 ROS1 + MAVROS + 控制节点**
```bash
cd ~/ros1_ws
source devel/setup.bash
roslaunch ladrc_controller single_uav.launch uav_id:=1 enu_offset_y:=0.0

# 预期日志输出:
#   [INFO] LADRC 集群执行节点已初始化 (命名空间: /uav1)
#   [INFO] 等待 MAVROS 连接和 swarm_command 消息...
#   [INFO] 已接收到 MAVROS local_position/odom 消息
#   [INFO] 系统稳定，发送解锁命令...
#   [INFO] 解锁命令已发送
#   [INFO] 解锁成功。切换到 Offboard 模式...
#   [INFO] Offboard 模式已激活。LADRC 控制器接管。
#   [INFO] UAV1 悬停保持锁定: [...]
```

**终端 3: 发送测试指令 (手动 topic pub)**
```bash
source ~/ros1_ws/devel/setup.bash

# 发送单点飞行指令: 飞到 (3, 0, 3)，用时 5 秒，normal 模式
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: 'world'}, uav_id: 1, \
    target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, \
    motion_style: 'normal', safety_factor: 0.0}"
```

**终端 4: 监控状态**
```bash
# 监控 UAV 状态
rostopic echo /uav1/status

# 监控 ENU 位置
rostopic echo /uav1/odom

# 监控 MAVROS 里程计
rostopic echo /uav1/mavros/local_position/odom/pose/pose/position
```

### 3.2 单机 SITL 测试通过标准

| 测试项 | 通过标准 | 检查方法 |
|--------|---------|---------|
| T1: MAVROS 连接 | 终端 2 出现 "已接收到 MAVROS local_position/odom" | 确认 MAVROS 节点正常启动 |
| T2: 自动起飞 | 终端 2 出现 "Offboard 模式已激活。LADRC 控制器接管。" | 无人机在 Gazebo 中起飞 |
| T3: 悬停保持 | 终端 2 出现 "悬停保持锁定" | 无人机稳定悬停（不漂移） |
| T4: 接收指令 | 终端 2 出现 "swarm_cmd 回调触发 (目标=...)" | 发送指令后被正确接收 |
| T5: 轨迹跟踪 | 终端 2 出现轨迹日志 (Ref/Pos/Cmd) | 无人机向目标移动 |
| T6: 到达目标 | 终端 2 出现 "悬停稳定! pos_err=..., vel=..." | 无人机到达 (3,0,3) 附近 |
| T7: 状态反馈 | 终端 4 中 `/uav1/status` 的 `is_hover_stable=true` | 调度层可以收到到达反馈 |
| T8: 位置反馈 | 终端 4 中 `/uav1/odom` 输出合理 ENU 坐标 | x≈3, y≈0, z≈3 |

---

## 4. 调度器端到端测试

### 4.1 测试步骤

在终端 3 不再手动 pub，改用调度器：

```bash
source ~/ros1_ws/devel/setup.bash
pip install openai numpy scipy httpx  # 确保 Python 依赖
rosrun location_allocate location_allocate_node
```

输入测试指令:
```
UAV1 以[3,0,3]为中心，变换成圆形编队，半径为0米，限时5秒
```

> 注：单机时 radius=0 表示飞往中心点本身

### 4.2 调度器测试通过标准

| 测试项 | 通过标准 |
|--------|---------|
| S1: LLM 解析 | 终端输出 JSON 蓝图，`task_sequences[0].uav_id=[1]` |
| S2: 命令发送 | 终端输出 "UAV1 -> [3.0, 0.0, 3.0]" |
| S3: 悬停等待 | 终端输出 "UAV1 到达目标并悬停稳定!" |
| S4: 任务完成 | 终端输出 "所有任务序列执行完毕" |

---

## 5. 坐标系统验证

### 5.1 ENU 坐标检查

MAVROS 输出 ENU 坐标 (ROS 标准):
- `odom.pose.pose.position.x` = 东 (East)  → 对应 Gazebo 世界 X 轴
- `odom.pose.pose.position.y` = 北 (North) → 对应 Gazebo 世界 Y 轴
- `odom.pose.pose.position.z` = 上 (Up)    → 对应 Gazebo 世界 Z 轴

### 5.2 验证方法

```bash
# 在 Gazebo 中观察无人机位置，与 rostopic echo 输出对比
rostopic echo /uav1/odom
# 当 Gazebo 中无人机在 (x=3, y=0, z=3) 时，
# odom 输出应为 x≈3, y≈0, z≈3 (ENU 坐标)
```

---

## 6. 运动风格验证

发送相同目标位置但不同 motion_style，观察飞行速度差异：

```bash
# Smooth (最慢)
rostopic pub -1 /uav1/swarm_command ... "{..., motion_style: 'smooth', ...}"
# Normal (中等)
rostopic pub -1 /uav1/swarm_command ... "{..., motion_style: 'normal', ...}"
# Aggressive (最快)
rostopic pub -1 /uav1/swarm_command ... "{..., motion_style: 'aggressive', ...}"
```

通过标准: Smooth 到达时间 > Normal 到达时间 > Aggressive 到达时间

---

## 7. 代码修改点速查 (ROS1 vs ROS2)

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
| 参数系统 | declare_parameter + get_parameter | ros::NodeHandle::param() |

---

## 8. 常见问题排查

| 问题 | 原因 | 解决 |
|------|------|------|
| catkin_make 报 mavros_msgs 找不到 | MAVROS 未安装 | `sudo apt-get install ros-noetic-mavros ros-noetic-mavros-msgs` |
| 控制节点收不到 odom | MAVROS 未连接 PX4 | 检查 `rostopic list \| grep mavros`，确认 MAVROS 正常运行 |
| 解锁失败 | 安全开关/预检未通过 | 检查 PX4 控制台，确认 `commander prearm` 无报错 |
| Offboard 模式切换失败 | setpoint 流 < 2Hz | 确认控制节点 50Hz 定时器正常运行 |
| 调度器 import 失败 | Python 路径未包含 devel | `source devel/setup.bash` |
| /uav1/odom 坐标不对 | spawn 偏移设置有误 | 单机 SITL 时 `enu_offset_y=0.0` |

---

## 9. 实机飞行前检查清单

在 SITL 测试全部通过后，进行实机测试前：

- [ ] PX4 固件确认 MAVLink 串口配置正确 (`MAV_1_CONFIG = TELEM2`)
- [ ] MAVROS fcu_url 改为串口地址 (`/dev/ttyACM0:921600` 或实际连接)
- [ ] `enu_offset_y` 参数设为 `0.0` (实机无 Gazebo spawn 偏移)
- [ ] 安全开关已按下，遥控器就绪
- [ ] 第一次飞行使用小幅度指令 (如 ±1m 位置阶跃)
- [ ] 确认紧急降落流程 (遥控器切 Stabilized 模式或 ROS 发送 disarm)
- [ ] 低电量保护 (`BAT_LOW_THR` 等 PX4 参数已配置)
- [ ] 动捕系统已标定、VRPN 数据正常
- [ ] 若使用 Nokov 动捕: 确认 `vrpn_client_ros` 正常运行，PX4 `EKF2_EV_CTRL` 使能外部视觉
