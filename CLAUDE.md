# 全局指令
**语言要求**: 必须严格使用简体中文输出所有的思考过程、终端分析和代码注释

# LLM-Driven Multi-UAV Swarm Control — ROS1 + MAVROS 移植版
**Target Agent**: Claude Code
**Project Role**: ROS1 C++ Backend / Python 调度层 开发与真机部署
**状态**: 单机 SITL 仿真全链路通过 (2026-06-06)，等待实机飞行测试

## 1. 项目概览 (Project Overview)

本项目是 [LLM-UAVswarm-performance](https://github.com/yihuanghuan/LLM-UAVswarm-performance) (ROS2 版) 的 ROS1 + MAVROS 移植版。
目标：在实验室 Nokov 动捕平台上进行真机多无人机编队飞行实验。

### 原版 vs 移植版

| 维度 | ROS2 原版 | ROS1 移植版（本项目） |
|------|----------|-------------------|
| ROS 版本 | Humble (Ubuntu 22.04) | Noetic (Ubuntu 20.04) |
| PX4 桥接 | px4_msgs + MicroXRCEAgent | mavros_msgs + MAVROS |
| 坐标系统 | NED (手动转换) | ENU (MAVROS 原生) |
| 位置设定点 | TrajectorySetpoint (NED) | PoseStamped (ENU) |
| Offboard 心跳 | 手动发布 OffboardControlMode | MAVROS 自动维持 |
| 解锁/切模式 | 发布 VehicleCommand | ROS 服务调用 |
| 状态机反馈 | 纯定时延迟 | `/mavros/state` 真实状态 |
| C++ API | rclcpp | roscpp |
| Python API | rclpy | rospy |

### 核心算法（与 ROS2 原版完全一致，未修改）

- **LADRC 自抗扰控制器**: LESO (线性扩张状态观测器) + LSEF (线性状态误差反馈)
- **Minimum Jerk 轨迹**: 5 次多项式点到点轨迹生成
- **IAPF 分布式避障**: 改进人工势场法，位置+加速度双通道斥力
- **FormationGenerator**: 直线/圆形/球形编队坐标生成
- **TopologyAllocator**: 匈牙利算法全局最优防交叉分配
- **LLM 指令解析**: MiniMax API 自然语言 → JSON 蓝图

## 2. 目录结构

```
ros1_ws/
├── CLAUDE.md                       # 本文件
├── README.md                       # 项目说明
├── VERIFICATION.md                 # 验证方案与测试标准
├── Dockerfile                      # ROS1 + MAVROS Docker 镜像
├── test_single_uav.sh              # Docker 单机测试一键脚本
├── .gitignore
└── src/
    ├── uav_swarm_interfaces/       # 自定义消息 (catkin message_generation)
    │   ├── msg/
    │   │   ├── UAVSwarmCommand.msg # 调度层→执行层指令
    │   │   └── UAVStatus.msg       # 执行层→调度层反馈
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── ladrc_controller/           # C++ 执行层 (roscpp + MAVROS)
    │   ├── src/
    │   │   ├── ladrc_position_controller_node.cpp  # 主控制节点 (400+ 行)
    │   │   ├── ladrc_core.cpp     # LADRC 控制器封装 (不变)
    │   │   ├── leso.cpp           # 线性扩张状态观测器 (不变)
    │   │   └── lsef.cpp           # 线性状态误差反馈 (不变)
    │   ├── include/ladrc_controller/
    │   │   ├── ladrc_core.hpp
    │   │   ├── leso.hpp
    │   │   ├── lsef.hpp
    │   │   └── minimum_jerk_trajectory.hpp  # 5次多项式轨迹 (不变)
    │   ├── config/ladrc_params.yaml         # LADRC + IAPF 参数配置
    │   ├── launch/
    │   │   ├── single_uav.launch   # 单机启动
    │   │   └── swarm.launch        # 多机集群 (最多5机硬编码)
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── location_allocate/          # Python 调度层 (rospy)
        ├── src/location_allocate/
        │   ├── location_allocate.py  # 匈牙利分配 + ROS 节点
        │   ├── no_location.py        # LLM API 解析 (不变)
        │   └── visualize_goals.py    # 3D 可视化
        ├── scripts/location_allocate_node  # 入口脚本
        ├── setup.py
        ├── CMakeLists.txt
        └── package.xml
```

## 3. 系统架构与数据流

```
自然语言指令 → LLM API (no_location.py) → JSON 蓝图
                                              ↓
                    FormationGenerator (坐标生成) → TopologyAllocator (匈牙利分配)
                                              ↓
                    UAVSwarmCommand (/uav{N}/swarm_command)
                                              ↓
                    C++ 控制节点: MinimumJerkTrajectory → ENU 设定点
                                              ↓
                    MAVROS → MAVLink → PX4 Offboard 控制 → 实机/Gazebo
                                              ↓
                    UAVStatus + ENU 位置反馈 → 调度层闭环
```

### MAVROS 关键话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/uav{N}/mavros/local_position/odom` | nav_msgs/Odometry (ENU) | PX4 → 控制器 |
| `/uav{N}/mavros/state` | mavros_msgs/State | PX4 → 控制器 |
| `/uav{N}/mavros/setpoint_position/local` | geometry_msgs/PoseStamped (ENU) | 控制器 → PX4 |
| `/uav{N}/mavros/cmd/arming` | mavros_msgs/CommandBool (服务) | 控制器 → PX4 |
| `/uav{N}/mavros/set_mode` | mavros_msgs/SetMode (服务) | 控制器 → PX4 |
| `/uav{N}/swarm_command` | UAVSwarmCommand | 调度层 → 控制器 |
| `/uav{N}/status` | UAVStatus | 控制器 → 调度层 |
| `/uav{N}/odom` | geometry_msgs/Point (ENU) | 控制器 → 调度层 |

## 4. 开发环境与测试方法

### 本机开发 (Ubuntu 22.04 + Docker)

本机只有 ROS2 Humble，无 ROS1 Noetic 原生环境。ROS1 代码的编译和测试通过 Docker 容器完成。

**Docker 权限注意**: 当前用户已在 `docker` 组中，但 shell 会话可能未生效。
- 临时解决: 每次 `docker` 命令前加 `sudo`
- 永久解决: 执行 `newgrp docker` 或重新登录
- **本机所有 Docker 命令均需 `sudo` 前缀**

```bash
# 构建 ROS1 + MAVROS 镜像（仅首次）
cd ~/ros1_ws && sudo docker build -t ros1-mavros:latest .

# 编译
sudo docker run --rm -v $(pwd):/ros1_ws ros1-mavros:latest \
  bash -c "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"

# SITL 全链路测试（需先在另一终端启动 PX4 SITL）
bash test_single_uav.sh
sudo docker exec -it ros1_test bash  # 进入容器发送指令

# 验证镜像是否存在
sudo docker images | grep ros1-mavros
```

### 实验室真机 (Ubuntu 20.04 + ROS1 Noetic 原生)

```bash
cd ~/ros1_ws && catkin_make && source devel/setup.bash
roslaunch ladrc_controller single_uav.launch uav_id:=1 enu_offset_y:=0.0
```

详细验证步骤见 `VERIFICATION.md`。

## 5. 真机测试要点

### 当前状态
- ✅ ROS1 代码已完成
- ✅ Docker 编译通过
- ✅ SITL 单机仿真全链路通过 (PX4 ↔ MAVROS ↔ 控制器 ↔ 轨迹跟踪)
- ⏳ 等待实机飞行测试

### 实机 vs SITL 关键差异

| 配置项 | SITL 仿真 | 实机测试 |
|--------|----------|---------|
| MAVROS fcu_url | `udp://:14540@127.0.0.1:14580` | `/dev/ttyACM0:921600` (Pixhawk USB) |
| enu_offset_y | `0.0` (单机) / `3.0*uid` (多机) | `0.0` (无 Gazebo spawn 偏移) |
| 位置来源 | Gazebo 物理引擎 | Nokov 动捕 → VRPN → PX4 EKF |
| 安全开关 | 无 | 物理安全开关 + 遥控器 |
| LADRC 参数 | ωo=10, ωc=4 (仿真调优) | ωo=8, ωc=2 (实机保守值，需逐次调高) |
| 首次指令 | 随意 | ±1m 小幅度位置阶跃，限时 5s |

### 实机飞行前必须完成
1. Nokov 动捕数据 → VRPN → PX4 EKF 链路确认
2. PX4 参数: `EKF2_EV_CTRL` 使能外部视觉，`EKF2_HGT_REF` 设为 vision
3. 紧急处置: 遥控器切 Stabilized 模式、ROS disarm
4. 首次飞行只用 ±1m 指令，观察跟踪精度后再逐步增大

### 关键参数 (ladrc_params.yaml)

```yaml
# SITL 仿真值 (已验证)
omega_o_x: 10.0    # 观测器带宽
omega_c_x: 3.0     # 控制器带宽
b0_x: 1.0

# 实机保守值 (首次飞行建议)
omega_o_x: 8.0
omega_c_x: 2.0
b0_x: 1.0

# 实机上电后可用 rostopic 动态调参:
# rosparam set /uav1/ladrc_position_controller/omega_o_x 8.0
```

## 6. 开发原则

1. **不修改核心算法**: `leso.cpp`、`lsef.cpp`、`ladrc_core.cpp` 中的数学逻辑保持与 ROS2 原版一致
2. **原 ROS2 项目不动**: 所有 ROS1 改动在本仓库 (`ros1_ws`) 中，ROS2 项目 (`LLM_swarm_ws`) 不受影响
3. **原子化 Git 提交**: 一个 commit 只做一件事，message 写清楚 "做了什么" 和 "为什么"
4. **小步验证**: 每次改动后确保编译通过，SITL 测试不退化
5. **真机优先安全**: 任何不确定的改动先在 SITL 中验证，再上真机

## 7. 相关仓库

- ROS2 原版: https://github.com/yihuanghuan/LLM-UAVswarm-performance
- ROS1 移植版（本仓库）: https://github.com/yihuanghuan/LLM-UAVswarm-ROS1
- PX4 固件: ~/PX4-Autopilot (本机)
