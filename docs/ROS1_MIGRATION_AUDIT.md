# ROS1 迁移阶段 1 项目审计

审计日期：2026-06-21

## 1. 审计范围

- 项目 A（只读）：`/home/yihuang/learning/LLM_swarm_ws`
- 项目 A 实际源码：`/home/yihuang/learning/LLM_swarm_ws/src/LLM-UAVswarm-performance`
- 项目 B（可修改）：`/home/yihuang/learning/ros1_ws（复件）`
- 本阶段只做项目审计，不修改项目 A，不执行代码迁移。

## 2. 项目 A 结构与 package

项目 A 是 ROS2 多无人机项目，源码主要位于 `src/LLM-UAVswarm-performance`，外层 ROS2 工作区还包含 `src/px4_msgs`。

项目 A 主要 package：

| Package | 类型 | 作用 |
| --- | --- | --- |
| `ladrc_controller` | `ament_cmake` / C++ | LADRC 位置控制、Minimum Jerk 轨迹、IAPF 避障、PX4 offboard 通信 |
| `location_allocate` | `ament_python` / Python | LLM 自然语言解析、编队坐标生成、匈牙利分配、任务调度 |
| `uav_swarm_interfaces` | `ament_cmake` / message package | 自定义消息 `UAVSwarmCommand`、`UAVStatus` |
| `px4_msgs` | ROS2 message package | PX4 XRCE-DDS 桥接消息，位于外层工作区 |

项目 A 源码目录中存在 `__pycache__` 等 Python 缓存文件，迁移时不应复制。

## 3. 项目 A 节点列表

| 节点 | Package | 入口 | 语言 | 说明 |
| --- | --- | --- | --- | --- |
| `ladrc_position_controller` | `ladrc_controller` | `ladrc_position_controller_node.cpp` | C++ / `rclcpp` | 每架无人机一个执行节点，运行 LADRC、轨迹、IAPF、offboard 状态机 |
| `location_allocate` | `location_allocate` | `location_allocate.location_allocate:main` | Python / `rclpy` | 调度层，读取自然语言指令并发布 `/uav{N}/swarm_command` |
| 可视化脚本 | `location_allocate` | `visualize_goals.py` | Python / `rclpy` | 监听 `/uav1~10/goal_pose` 并做 3D 可视化，未参与主控制链路 |

项目 A 控制节点使用命名空间推导自身 UAV ID，例如 `/uav3` 推导为 `3`。

## 4. 项目 A launch 链路

### 单控制节点 launch

`minisnap_LADRC/ladrc_controller/launch/ladrc_controller.launch.py`

- 声明 `params_file`，默认加载 `ladrc_params.yaml`。
- 声明 `namespace`，默认空，示例为 `/uav1`。
- 启动 `ladrc_controller/ladrc_position_controller_node`。

### 多机 launch

`minisnap_LADRC/ladrc_controller/launch/swarm_launch.py`

- 参数 `uav_ids` 默认 `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]`。
- 对每个 `uid` 启动一个 `/uav{uid}/ladrc_position_controller`。
- 多机 PX4 映射规则：
  - UAV1 -> PX4 instance 1 -> `/px4_1`
  - UAV2 -> PX4 instance 2 -> `/px4_2`
  - 通用规则：UAV{N} -> PX4 instance {N} -> `/px4_{N}`
- 使用 remap 将 `/uav{uid}/fmu/...` 映射到 `/px4_{uid}/fmu/...`。
- 为邻居 odom 添加 remap，供 IAPF 监听其他无人机位置。
- SITL spawn 偏移按 `enu_offset_y = 3.0 * uid` 注入参数。

项目 A 的 PX4 / Gazebo Classic 启动脚本不在项目 A 源码中直接提供；launch 注释假设使用 PX4 的 `sitl_multiple_run.sh`，并说明多机实例从 `-i 1` 开始。

## 5. 项目 A topic / service / parameter

### 自定义 topic

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav{N}/swarm_command` | `uav_swarm_interfaces/msg/UAVSwarmCommand` | 调度层 -> 控制节点 | 目标点、时长、运动风格、安全系数 |
| `/uav{N}/status` | `uav_swarm_interfaces/msg/UAVStatus` | 控制节点 -> 调度层 | 是否到达目标并稳定悬停 |
| `/uav{N}/odom` | `geometry_msgs/msg/Point` | 控制节点 -> 调度层 | 低频 ENU 全局位置 |

### PX4 XRCE-DDS topic

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav{N}/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | PX4 -> 控制节点 | 原始 PX4 odom，代码中做 NED/ENU 转换 |
| `/uav{N}/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | 控制节点 -> PX4 | offboard 心跳 |
| `/uav{N}/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | 控制节点 -> PX4 | 位置设定点 |
| `/uav{N}/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 控制节点 -> PX4 | arming、切 mode |

项目 A 未发现 ROS service/action 作为主通信接口；PX4 命令通过 `VehicleCommand` topic 发布。

### 主要参数

| 参数 | 默认/来源 | 说明 |
| --- | --- | --- |
| `control_frequency` | 50.0 | 控制频率 |
| `omega_o_x/y/z` | YAML / declare_parameter | LESO 观测器带宽 |
| `omega_c_x/y/z` | YAML / declare_parameter | LSEF 控制带宽 |
| `b0_x/y/z` | YAML / declare_parameter | 控制增益估计 |
| `max_velocity` | YAML / declare_parameter | 最大速度 |
| `max_acceleration_x/y/z` | YAML / declare_parameter | 加速度限幅 |
| `enu_offset_x/y/z` | launch 注入 | Gazebo 多机 spawn 偏移 |
| `iapf_safe_distance` | YAML / declare_parameter | IAPF 安全距离 |
| `iapf_repulsion_gain` | YAML / declare_parameter | IAPF 斥力增益 |
| `neighbor_uav_ids` | launch/YAML | 邻居 UAV ID 列表 |

## 6. 项目 A 核心算法与 ROS2 接口边界

核心算法应尽量保持不变：

- `ladrc_core.cpp/hpp`：LADRC 控制器封装。
- `leso.cpp/hpp`：线性扩张状态观测器。
- `lsef.cpp/hpp`：线性状态误差反馈。
- `minimum_jerk_trajectory.hpp`：五次多项式点到点轨迹。
- `location_allocate.py` 中的 `FormationGenerator`：Line/Circle/Sphere/Free 目标点生成。
- `TopologyAllocator`：基于匈牙利算法的分配逻辑。
- `computeIAPF`：基于邻居位置的 IAPF 斥力计算。
- `no_location.py`：自然语言到 JSON 任务蓝图解析逻辑。

ROS2 接口层需要迁移：

- `rclcpp::Node`、publisher/subscription/timer/parameter。
- `rclpy.Node`、`create_publisher`、`create_subscription`、`spin_once`。
- `ament_cmake`、`ament_python`、`rosidl_default_generators`。
- `px4_msgs` + XRCE-DDS topic。
- ROS2 launch Python 文件。

安全风险：项目 A 的 `location_allocate/no_location.py` 中硬编码了 MiniMax API Key。项目 A 不能修改；迁移到项目 B 时必须使用环境变量或其他安全配置，不能复制密钥。

## 7. 项目 B 结构与 ROS1 环境

项目 B 是 ROS1 Noetic + MAVROS catkin workspace。当前根目录包含：

- `Dockerfile`
- `README.md`
- `VERIFICATION.md`
- `REALTEST.md`
- `test_single_uav.sh`
- `setup_real_hardware.sh`
- `src/ladrc_controller`
- `src/location_allocate`
- `src/uav_swarm_interfaces`

项目 B 当前已有 `build/`、`devel/` 构建产物，不应纳入迁移提交。

当前工作区位置：

```bash
/home/yihuang/learning/ros1_ws（复件）
```

任务文档和部分脚本中仍使用 `/home/yihuang/learning/ros1_ws` 或 `~/ros1_ws`，后续脚本应避免新增硬编码绝对路径。

## 8. 项目 B Docker、编译与单机测试

### Docker

`Dockerfile` 基于 `ros:noetic-ros-base`，安装：

- `ros-noetic-mavros`
- `ros-noetic-mavros-extras`
- `ros-noetic-mavros-msgs`
- `libeigen3-dev`
- MAVROS GeographicLib 数据集

镜像名在文档和脚本中约定为 `ros1-mavros:latest`。

### 编译

文档推荐：

```bash
catkin_make
source devel/setup.bash
```

Docker 编译示例：

```bash
sudo docker run --rm --network host -v $(pwd):/ros1_ws ros1-mavros:latest \
  bash -c "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"
```

### 单机 SITL 测试

`VERIFICATION.md` 和 `test_single_uav.sh` 定义单机测试流程：

1. 宿主机启动 PX4 SITL + Gazebo Classic：
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic
   ```
2. 项目 B 启动 Docker ROS1 环境：
   ```bash
   bash test_single_uav.sh
   ```
3. 容器内发送：
   ```bash
   rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand ...
   ```

`test_single_uav.sh` 当前硬编码 `WS=/home/yihuang/learning/ros1_ws`，与本次工作区 `/home/yihuang/learning/ros1_ws（复件）` 不一致，后续迁移计划中应处理。

## 9. 项目 B 已有 ROS1 package 与可复用映射

### `uav_swarm_interfaces`

- ROS1 catkin message package。
- 使用 `message_generation` / `message_runtime`。
- 消息字段与项目 A 保持一致：
  - `UAVSwarmCommand`: `Header header`、`uint8 uav_id`、`geometry_msgs/Point target_pos`、`float32 duration`、`string motion_style`、`float32 safety_factor`
  - `UAVStatus`: `uint8 uav_id`、`bool is_hover_stable`

### `ladrc_controller`

已存在 ROS1 `roscpp` 控制节点：

- 订阅 `mavros/local_position/odom` (`nav_msgs/Odometry`)。
- 订阅 `mavros/state` (`mavros_msgs/State`)。
- 订阅 `swarm_command`。
- 发布 `mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)。
- 发布 `status`。
- 发布 `odom`。
- 调用 `mavros/cmd/arming` (`mavros_msgs/CommandBool`)。
- 调用 `mavros/set_mode` (`mavros_msgs/SetMode`)。

项目 B 控制节点已将 PX4 ROS2 topic 映射为 MAVROS：

| ROS2 / PX4 | ROS1 / MAVROS |
| --- | --- |
| `VehicleOdometry` | `/uav{N}/mavros/local_position/odom` |
| `TrajectorySetpoint` | `/uav{N}/mavros/setpoint_position/local` |
| `VehicleCommand` arming | `/uav{N}/mavros/cmd/arming` |
| `VehicleCommand` mode | `/uav{N}/mavros/set_mode` |
| `OffboardControlMode` | 删除显式 topic，依靠 MAVROS setpoint 流 |

### `location_allocate`

已存在 ROS1 `rospy` 调度层：

- `rclpy` 已替换为 `rospy`。
- 发布 `/uav{N}/swarm_command`。
- 订阅 `/uav{N}/status`、`/uav{N}/odom`。
- `no_location.py` 已改为从 `MINIMAX_API_KEY`、`MINIMAX_BASE_URL`、`MINIMAX_MODEL_NAME` 读取配置，未继续硬编码项目 A 的 API Key。

## 10. 项目 B launch 链路

### `single_uav.launch`

- 启动 `/uav{uav_id}/mavros`。
- 启动 `/uav{uav_id}/ladrc_position_controller`。
- 默认 SITL MAVROS URL：`udp://:14540@127.0.0.1:14580`。
- 实机 URL 默认：`/dev/ttyACM0:921600`。
- 支持 `enu_offset_y`。

### `swarm.launch`

- 当前显式写出 UAV1-UAV5。
- 每个 UAV 启动一个 namespaced MAVROS 和控制节点。
- SITL 端口映射：
  - UAV1: `udp://:14540@127.0.0.1:14580`
  - UAV2: `udp://:14550@127.0.0.1:14581`
  - UAV3: `udp://:14560@127.0.0.1:14582`
  - UAV4: `udp://:14570@127.0.0.1:14583`
  - UAV5: `udp://:14580@127.0.0.1:14584`
- 每个 UAV 设置 `target_system_id` 为对应 UAV ID。
- 每个控制节点设置 `enu_offset_y` 为 `3.0 * uid`。

风险：`uav_ids` 参数当前没有真正驱动动态生成 group；launch 只静态包含 UAV1-UAV5，不能覆盖项目 A 默认的 10 架无人机。

### `real_hardware.launch`

- 单机/实机为主。
- 启动 `vrpn_client_ros`。
- 使用 `vrpn_vision_relay.py` 转发 `/vrpn_client_node/uav1/pose` 到 `/uav1/mavros/vision_pose/pose`。
- 启动 MAVROS 和 LADRC 控制节点。

## 11. 主要迁移风险

1. 项目 B 已有部分迁移内容，后续不能盲目覆盖；需要基于差异补齐，保护单机 SITL 测试代码。
2. 项目 B 的 `src/ladrc_controller/config/ladrc_params.yaml` 仍保留 ROS2 风格 `/**: ros__parameters:` 结构，ROS1 `<rosparam>` 加载后可能形成嵌套参数，导致私有参数读取不到预期值。后续需验证或单独修正。
3. 项目 B `swarm.launch` 当前只静态覆盖 5 架无人机，项目 A 默认多机规则覆盖 10 架；后续需要明确 ROS1 多机 launch 生成策略。
4. 项目 B `test_single_uav.sh` 硬编码 `/home/yihuang/learning/ros1_ws`，与当前工作区路径不一致；后续运行可能挂载错误目录。
5. 项目 A 使用 `px4_msgs` + XRCE-DDS，项目 B 使用 MAVROS；坐标系、offboard 心跳、arming/mode、setpoint 语义均需逐项验证。
6. 项目 A 邻居 odom 逻辑中使用 Gazebo spawn 偏移；项目 B 邻居位置补偿方向与项目 A 不完全一致，IAPF 多机效果需仿真验证。
7. 多机 PX4/Gazebo Classic 启动方式未在项目 B 中形成统一脚本；后续需明确如何启动 `sitl_multiple_run.sh` 或等价多机 SITL。
8. Docker/Gazebo GUI/GPU/显示权限可能阻止完整 GUI 仿真；应优先准备 headless 与 `rosnode`/`rostopic` 验证路径。
9. 项目 A 的 LLM 解析依赖外部网络和 API Key；项目 B 已改为环境变量，但端到端调度测试仍受网络/API 可用性影响。
10. 当前 Git 工作区已有未跟踪文件 `.catkin_workspace`、`src/CMakeLists.txt`、`task_for_codex.md`，本阶段不应误提交。

## 12. 阶段 1 结论

项目 B 已经包含一版 ROS1 + MAVROS 移植成果，阶段 2 不应从零迁移，而应以“差异审计 + 补齐验证 + 风险修复”为主：

- 继续复用项目 B 已有 `uav_swarm_interfaces`、`ladrc_controller`、`location_allocate`。
- 优先验证并修正参数加载、工作区路径、多机 launch 覆盖数量、MAVROS 端口/namespace 映射。
- 后续迁移计划应明确哪些项目 A 文件只作为算法参考，哪些项目 B 文件需要改写或补齐。
