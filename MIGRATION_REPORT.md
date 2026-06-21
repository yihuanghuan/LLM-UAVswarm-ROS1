# ROS1 多无人机迁移报告

日期：2026-06-21

## 1. 迁移目标

将项目 A（ROS2 + PX4 + Gazebo Classic 多无人机项目）的多无人机控制、调度、编队、避障和轨迹逻辑迁移到项目 B（ROS1 Noetic + MAVROS catkin workspace），并优先复用项目 B 已有单无人机测试代码。

本轮工作没有修改项目 A，所有修改均发生在项目 B。

## 2. 项目 A 结构分析

项目 A 源码位于 `/home/yihuang/learning/LLM_swarm_ws/src/LLM-UAVswarm-performance`，主要 ROS2 package：

- `ladrc_controller`：C++ `rclcpp` 控制节点，包含 LADRC、Minimum Jerk、IAPF、PX4 offboard 状态机。
- `location_allocate`：Python `rclpy` 调度节点，包含 LLM 指令解析、编队坐标生成、匈牙利分配。
- `uav_swarm_interfaces`：自定义消息 `UAVSwarmCommand`、`UAVStatus`。
- 外层工作区包含 `px4_msgs`，供 XRCE-DDS 与 PX4 通信。

项目 A 多机规则：

- namespace：`/uav{N}`。
- 默认无人机编号：1-10。
- PX4 多机 instance 从 1 开始。
- Gazebo spawn 偏移主要为 Y 轴 `3.0 * N`。

## 3. 项目 B ROS1 环境分析

项目 B 是 ROS1 Noetic + MAVROS catkin workspace，当前路径：

```bash
/home/yihuang/learning/ros1_ws（复件）
```

项目 B 已有 package：

- `uav_swarm_interfaces`
- `ladrc_controller`
- `location_allocate`

Docker 环境：

- 镜像：`ros1-mavros:latest`
- 基础镜像：`ros:noetic-ros-base`
- 已补齐 MAVROS、Eigen、NumPy、SciPy、OpenAI SDK、httpx 等依赖。

## 4. ROS2 到 ROS1 映射

| ROS2 / 项目 A | ROS1 / 项目 B |
| --- | --- |
| `rclcpp` | `roscpp` |
| `rclpy` | `rospy` |
| `ament_cmake` | `catkin` |
| `ament_python` | `catkin_python_setup()` |
| `rosidl_default_generators` | `message_generation` |
| ROS2 launch Python | ROS1 XML launch |
| ROS2 parameter YAML | ROS1 `<rosparam>` 扁平 YAML |

## 5. PX4 / MAVROS 映射

| 项目 A `px4_msgs` | 项目 B MAVROS |
| --- | --- |
| `VehicleOdometry` | `/uav{N}/mavros/local_position/odom` |
| `TrajectorySetpoint` | `/uav{N}/mavros/setpoint_position/local` |
| `VehicleCommand` 解锁 | `/uav{N}/mavros/cmd/arming` |
| `VehicleCommand` 切模式 | `/uav{N}/mavros/set_mode` |
| `OffboardControlMode` | 通过持续 setpoint 流维持，不单独发布 |

多机 SITL 端口按 PX4 `sitl_multiple_run.sh` 日志修正：

- UAV1：`udp://:14541@127.0.0.1:14581`
- UAV2：`udp://:14542@127.0.0.1:14582`
- 通用：本地 companion 端口 `14540 + N`，PX4 onboard 端口 `14580 + N`。
- PX4 多机脚本中 UAV1 对应 `MAV_SYS_ID=2`，因此 `swarm.launch` 在 `use_sim=true` 时使用 `target_system_id=N+1`；实机模式仍保持 `target_system_id=N`。

## 6. 修改文件清单

新增：

- `docs/ROS1_MIGRATION_AUDIT.md`
- `docs/ROS1_MIGRATION_PLAN.md`
- `docs/ROS1_GAZEBO_MULTI_UAV_SIMULATION.md`
- `scripts/build_ros1_ws.sh`
- `scripts/check_multi_uav_command_flight.sh`
- `scripts/check_multi_uav_topics.sh`
- `scripts/check_multi_uav_runtime.sh`
- `scripts/run_multi_uav_sim.sh`
- `scripts/run_llm_scheduler.sh`
- `MIGRATION_REPORT.md`
- `RUN_ROS1_SIMULATION.md`

修改：

- `.gitignore`
- `Dockerfile`
- `test_single_uav.sh`
- `src/ladrc_controller/config/ladrc_params.yaml`
- `src/ladrc_controller/src/ladrc_position_controller_node.cpp`
- `src/ladrc_controller/launch/single_uav.launch`
- `src/ladrc_controller/launch/swarm.launch`
- `src/location_allocate/src/location_allocate/location_allocate.py`

删除：

- 无。

## 7. 核心节点迁移说明

`ladrc_position_controller_node.cpp` 保持 LADRC、Minimum Jerk、IAPF 核心逻辑，补齐了 ROS1 参数加载和邻居 odom 订阅：

- `neighbor_uav_ids` 从 ROS1 私有参数读取。
- 邻居位置订阅 `/uav{N}/mavros/local_position/odom`。
- SITL 多机邻居偏移修正为 Y 轴。
- ROS1 参数文件改为扁平结构，避免 `<rosparam>` 加载到错误层级。

`location_allocate.py` 按项目 A `Claude.md` 的“认知层 + 调度层”架构迁移到 ROS1：

- `raw_input()` 改为 `input()`。
- 保持 FormationGenerator、TopologyAllocator、串行/并行任务编排和悬停闭环反馈逻辑。
- 从 ROS 私有参数 `~uav_ids` 或 `~uav_count` 读取当前启用 UAV，默认 8 机，不再固定把 LLM 情报告知为 10 机。
- 启动后等待启用 UAV 的 `/uavN/odom` 首帧数据，再进入自然语言输入循环。
- 对 LLM 输出的 `uav_id` 做启用集合校验，并按 `uav_id` 修正 `uav_count`。
- `no_location.py` 从 `MINIMAX_API_KEY`、`MINIMAX_BASE_URL`、`MINIMAX_MODEL_NAME` 读取真实 LLM API 配置，不复制项目 A 的硬编码 Key。
- `scripts/run_llm_scheduler.sh` 将宿主机 LLM 环境变量传入 Docker 容器并启动 `location_allocate_node`。
- Dockerfile 补齐 `scipy`、`openai`、`httpx` 运行依赖。

## 8. 多无人机 namespace 设计

- 控制 namespace 固定为 `/uav{N}`。
- 每架无人机启动：
  - `/uav{N}/mavros`
  - `/uav{N}/ladrc_position_controller`
- 调度层 topic：
  - `/uav{N}/swarm_command`
  - `/uav{N}/status`
  - `/uav{N}/odom`
- `swarm.launch` 显式覆盖 UAV1-UAV10，并提供 `enable_uavN` 开关。

## 9. 编译验证结果

已通过：

```bash
./scripts/build_ros1_ws.sh
```

结果：

- Docker 镜像 `ros1-mavros:latest` 构建成功。
- `catkin_make` 成功。
- `devel/lib/ladrc_controller/ladrc_position_controller_node` 生成成功。
- `devel/lib/location_allocate/location_allocate_node` 生成成功。
- `rosmsg show uav_swarm_interfaces/UAVSwarmCommand` 和 `UAVStatus` 成功。
- Python 导入 `scipy`、`openai`、`httpx`、`location_allocate` 成功。

## 10. Gazebo 仿真验证结果

已执行 8 机 Gazebo Classic + PX4 SITL + ROS1/MAVROS 验证：

```bash
/home/yihuang/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
./scripts/run_multi_uav_sim.sh 8
docker exec ros1_multi_uav bash -lc "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8 && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10"
```

已验证：

- Gazebo Classic `gzserver` 启动。
- `iris_1` 到 `iris_8` spawn 成功，Y 轴位置分别约为 3m、6m、9m、12m、15m、18m、21m、24m。
- `/uav1/mavros` 到 `/uav8/mavros` 节点出现。
- `/uav1/ladrc_position_controller` 到 `/uav8/ladrc_position_controller` 节点出现。
- `/uav1` 到 `/uav8` 的 MAVROS、setpoint、status、odom、swarm_command topic 全部出现。
- `/uav1/odom` 到 `/uav8/odom` 均有数据，Y 坐标符合 Gazebo spawn 偏移规则。
- `/uav1/mavros/state` 到 `/uav8/mavros/state` 均为 `connected: True`、`armed: True`、`mode: "OFFBOARD"`。
- 启动后完成一次 topic + runtime 检查，并在额外等待 60 秒后再次执行 runtime 检查，两次均通过。
- 通过 `/uav1/swarm_command` 到 `/uav8/swarm_command` 发布 8 机目标点 `[0.0, 3.0*N, 1.5]`，8 架无人机均反馈 `is_hover_stable: True`。
- 指令飞行后的最终采样位置接近 `x≈0`、`y≈3.0*N`、`z≈1.65-1.73`。
- LLM 调度节点已支持如下验收入口：

```bash
export MINIMAX_API_KEY="your-api-key"
./scripts/run_llm_scheduler.sh 8
```

在提示符输入自然语言指令后，节点会调用真实 LLM API、打印 JSON 蓝图、执行匈牙利分配并下发 `/uavN/swarm_command`。

未完全确认：

- 当前终端环境未设置 `MINIMAX_API_KEY`，因此未在本机完成真实外部 API 调用验收。
- 未验证 10 机全量仿真性能。
- 未验证 Gazebo GUI。

## 11. 已知问题

- 多机实机模式不能复用单个 `fcu_url_real` 同时连接多台无人机，需要按硬件 IP/串口拆分配置。
- LLM 调度层需要宿主机设置 `MINIMAX_API_KEY`，并由 `scripts/run_llm_scheduler.sh` 传入容器。
- PX4/Gazebo Classic 环境中加载了 ROS2 Humble 的 Gazebo ROS 插件路径，headless spawn 可用，但 GUI 或插件冲突仍需人工确认。
- `task_for_codex.md` 仍是本地未跟踪文件，未提交。

## 12. 后续人工检查建议

1. 扩展到 10 机，检查 CPU、Gazebo 实时率和 topic 冲突。
2. 设置 `MINIMAX_API_KEY` 后运行 `./scripts/run_llm_scheduler.sh 8` 做真实 LLM 自然语言调度端到端测试。
3. 实机多机前，按每架飞机实际 MAVLink 地址拆分 launch。

## 13. Git 提交记录

| Commit Hash | 提交说明 | 修改范围 | 验证结果 |
| --- | --- | --- | --- |
| `242736d` | `chore: 添加 ROS1 迁移审计记录` | 阶段 1 审计文档 | 已推送 |
| `f2670d3` | `chore: 更新 ROS1/Gazebo 工作区 gitignore` | `.gitignore` | 已推送 |
| `bc24b3f` | `chore: 添加 ROS1 迁移计划` | 阶段 2 计划文档 | 已推送 |
| `ce639e1` | `fix: 修复 ROS1 参数加载和邻居避障偏移` | 参数 YAML、控制节点 | Docker 编译通过 |
| `056b7ce` | `fix: 移除单机测试脚本工作区硬编码` | `test_single_uav.sh` | 脚本语法通过 |
| `d1c230f` | `feat: 扩展 ROS1 多无人机 MAVROS launch` | `swarm.launch` | launch 节点枚举通过 |
| `42a521a` | `test: 添加多无人机仿真检查脚本` | `scripts/` | 脚本语法通过 |
| `2dd554f` | `fix: 补齐调度层 Python3 运行依赖` | Dockerfile、调度节点 | 后续 pin 修复后导入通过 |
| `0f2bfa5` | `fix: 固定调度层 Python 依赖版本` | Dockerfile | Docker build 通过 |
| `02d1bf9` | `fix: 修复 launch 仿真模式参数判断` | 单机/多机 launch | dump-params 验证 UDP 分支 |
| `02a4436` | `fix: 修正多机 SITL MAVROS 端口映射` | 多机 launch、计划文档 | 2 机 topic/odom 验证通过 |
| `d7e0d03` | `docs: 添加 ROS1 迁移报告和运行说明` | 迁移报告、运行说明 | 已推送 |
| `994ba29` | `fix: 稳定多机 MAVROS 连接和运行时检查` | 多机 launch、控制状态机、runtime 检查脚本 | 8 机 topic/runtime 验证通过 |
| `b2ffd6a` | `docs: 更新 8 机 Gazebo 验证结果` | 迁移报告、运行说明 | 8 机延迟 runtime 复查通过 |
| `b9172c7` | `test: 添加 8 机指令飞行验证脚本` | 指令飞行检查脚本 | 8 机 `swarm_command` 验证通过 |
| `74b6b02` | `docs: 添加 ROS1 多机 Gazebo 仿真指南` | 详细仿真说明文档、迁移报告 | 终端三层验证通过 |
| `b9b470a` | `feat: 补齐 ROS1 LLM 调度入口` | 调度节点、LLM 启动脚本 | 静态检查、Docker 编译、缺 Key/缺容器检查通过 |
| 本提交 | `docs: 更新 LLM 调度层运行说明` | 迁移报告、仿真运行文档 | 记录真实 LLM API 验收步骤 |
