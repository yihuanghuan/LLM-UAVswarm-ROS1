# LATE-Swarm：ROS1 多无人机编队仿真

本项目通过自然语言或 Candidate Mission JSON 控制 PX4 无人机集群，在 Gazebo Classic 中执行编队、顺序任务和同步并行任务。

采用论文最终基线 `paper-final-sim-v3`，冻结配置为 `paper-current-v11-c0-f-frozen`。默认运行 `candidate_v2`、安全感知分配、`ladrc_acceleration`、`iapf_dual` 和 `id_order`。

```text
自然语言 / Candidate JSON
          ↓
Candidate 校验 → Mission Graph / FSM → 新鲜状态快照
          ↓
延迟解析 → 几何与安全感知分配 → T_exec → Execution Profile
          ↓
UAVExecutionCommand → Minimum Jerk + LADRC + IAPF
          ↓
MAVROS → PX4 Offboard → Gazebo
          ↑
       状态与完成反馈
```

## 1. 运行环境

| 运行位置 | 软件与用途 |
|---|---|
| 宿主机 | Linux 图形桌面、Docker、已编译的 PX4 SITL、Gazebo Classic 11 |
| Docker 容器 | ROS1 Noetic、MAVROS、Python 3.8、调度器、控制器 |
| 可选外部服务 | OpenAI 兼容 LLM API；使用 JSON 任务不需要 API Key |

实际验证环境为 Ubuntu 22.04 宿主机、Gazebo Classic 11.10.2 和 ROS1 Noetic 容器。宿主机不需要启动 ROS2、MicroXRCEAgent 或额外的 roscore。

将项目放在 `~/learning/ros1_ws`。默认 PX4 路径为 `~/PX4-Autopilot`，需已具备以下文件：

```bash
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
```

若缺少编译结果，可在已配置好 PX4 构建依赖的环境中执行：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default sitl_gazebo-classic
```

## 2. 构建 ROS1 项目

以下命令在**宿主机**执行：

```bash
cd ~/learning/ros1_ws

# 首次运行时构建镜像；本机已有该镜像时可跳过。
docker build -t ros1-paper:latest .

# 在 Docker 中检查冻结参数并编译 catkin 工作区。
./scripts/build_ros1_ws.sh
```

Dockerfile 安装 ROS1/MAVROS、GeographicLib 数据和 Python 依赖，首次构建需要网络。修改项目源码后重新运行构建脚本。

## 3. 启动五机仿真

### 终端 A：Gazebo 与 PX4

在**宿主机**执行，并保持此终端运行：

```bash
cd ~/learning/ros1_ws
./scripts/run_gazebo_headless.sh 5
```

等待 `iris_1` 至 `iris_5` 生成，各 PX4 实例启动。脚本运行真实的 Gazebo 物理仿真，只是不自动打开图形界面。

如果 PX4 位于其他路径：

```bash
PX4_DIR=/你的/PX4-Autopilot ./scripts/run_gazebo_headless.sh 5
```

### 可选终端：Gazebo 图形窗口

先启动终端 A，再在**宿主机图形桌面**执行：

```bash
cd ~/learning/ros1_ws
./scripts/open_gazebo_gui.sh
```

使用自定义 PX4 路径时，也给此脚本设置相同的 `PX4_DIR`。`gzclient` 是图形客户端；没有先启动 `gzserver` 时，它不能独立运行本项目的仿真。

### 终端 B：ROS1 与控制器

在**宿主机**执行：

```bash
cd ~/learning/ros1_ws
./scripts/run_multi_uav_sim.sh 5
```

该命令在后台创建 `ros1_multi_uav` 容器，启动 ROS Master、五个 MAVROS、五个控制器及位置流配置节点。默认情况下，飞机会自动解锁、进入 OFFBOARD，并起飞到约 **1.5 米**。

查看启动日志：

```bash
docker logs -f --tail 100 ros1_multi_uav
```

这里按 Ctrl+C 只退出日志查看。不要在执行任务时重复运行启动脚本，它会重建同名容器。

### 终端 C：进入容器并等待就绪

在宿主机执行：

```bash
docker exec -it ros1_multi_uav bash
```

此后在**容器内**执行：

```bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
cd /ros1_ws

python3 scripts/wait_for_swarm_ready.py --ids 1,2,3,4,5
```

看到 `READY: [1, 2, 3, 4, 5]` 后执行任务。也可手动检查：

```bash
./scripts/check_multi_uav_topics.sh 5
./scripts/check_multi_uav_runtime.sh 5
rostopic echo -n 1 /uav1/status
```

就绪状态应包含 `system_ready: True`、`armed: True`、`offboard: True`、`failsafe: False`。尚未收到任务时，`is_hover_stable` 可以为 False。

## 4. 运行示例任务：不需要 LLM

在已设置环境的**容器终端 C**执行：

```bash
rosrun location_allocate candidate_dispatch \
  --mission-json /ros1_ws/examples/five_uav_mission.json \
  --uav-ids 1,2,3,4,5 \
  --policy /ros1_ws/src/lfs_policy/config/lfs_policy.paper_current.yaml
```

示例依次执行：

1. 五机组成圆形，稳定后等待一秒。
2. 1～3 号机组成三角形（smooth），4～5 号机组成直线（aggressive），两个分组同步执行。
3. 五机汇合为一条直线。

成功时最后输出：

```json
{"candidate_completed": true, "failure": null}
```

可复制 `examples/five_uav_mission.json` 后编辑任务。坐标为**全局 ENU**：X/Y 为水平位置，Z 为高度，单位米；任务时长单位秒。`T: {"mode":"auto"}` 由当前算法确定执行时长。

JSON 入口经过完整的生产 Candidate 调度与反馈链路。无效任务会报错，不会下发另一套算法的命令。

## 5. 自然语言操作

在**宿主机**配置本地凭据：

```bash
cd ~/learning/ros1_ws

# 已有 .env.minimax 时保留现有文件。
[ -f .env.minimax ] || cp .env.minimax.example .env.minimax
nano .env.minimax
```

配置格式：

```dotenv
LLM_API_KEY=填写你的真实Key
LLM_BASE_URL=https://api.minimax.chat/v1
LLM_MODEL_NAME=MiniMax-M2.7-highspeed
```

`.env.minimax` 已被 Git 和 Docker 构建上下文忽略。也支持通过环境变量提供 `LLM_API_KEY` 或 `MINIMAX_API_KEY`。

启动交互调度器：

```bash
./scripts/run_llm_scheduler.sh 5
```

在 `请输入无人机编队指令:` 后输入：

```text
Have UAVs 1 through 5 form a circle centered at [0,9,3] with radius 4 meters, automatic duration, normal motion, and safety factor 1.0.
```

这条英文指令已通过真实 LLM 服务及五机飞行验证。等待任务完成后再输入下一条；输入 `q` 退出调度器。退出调度器不会停止控制器或仿真。

同一时间使用一个调度入口，避免交互调度器和 JSON 调度器同时控制同一机群。只运行 JSON 示例和自动回归时，不需要配置凭据。

## 6. 自动验证与结果

在已设置 ROS 环境的**五机容器终端**执行：

```bash
cd /ros1_ws
./scripts/validate_paper_runtime.sh
```

脚本等待五机就绪，运行示例任务，录制约 45 秒数据，并检查 15 条命令、三个 motion style、并行执行时长、加速度掩码、最终稳定状态、控制器故障及 1.50 m 最小采样机间距。

默认创建 `validation/run_时间戳/`，不会覆盖已有结果。也可指定输出目录：

```bash
./scripts/validate_paper_runtime.sh /ros1_ws/validation/my_run
```

| 输出文件 | 内容 |
|---|---|
| `readiness.log` | 五机启动就绪 |
| `mission.log` | 任务执行结果 |
| `candidate_resolution_trace.jsonl` | 状态快照、解析、分配和参数审计 |
| `telemetry.csv` | 录制的位置、速度及任务状态 |
| `flight_report.json` | 间距、最终误差、控制器状态等指标 |
| `assertions.log` | 自动通过判定 |

容器 `/ros1_ws` 映射到宿主机的项目目录。查看报告示例：

```bash
python3 -m json.tool /ros1_ws/validation/my_run/flight_report.json
```

运行单元与算法回归测试：

```bash
cd /ros1_ws
python3 -m pytest src/location_allocate/test src/lfs_policy/test -q
catkin_make run_tests_ladrc_controller
catkin_test_results
```

清理后的测试记录见 [当前验证报告](docs/VALIDATION.md)。

## 7. 观察与排查

以下命令在已设置 ROS 环境的**容器内**执行：

```bash
# 位置与速度：全局 ENU
rostopic echo /uav1/swarm_state

# 状态流频率：MAVROS 申请 100 Hz，实测受机器负载影响
rostopic hz /uav1/swarm_state

# 就绪与任务完成
rostopic echo /uav1/status

# 执行命令：发送任务前开始监听
rostopic echo /uav1/execution_command

# IAPF 与控制器诊断
rostopic echo /uav1/iapf_debug
rostopic echo /uav1/control_tracking_debug

# 默认应为 ladrc_acceleration，PositionTarget.type_mask 应为 2111
rosparam get /uav1/ladrc_position_controller/control_mode
rostopic echo -n 1 /uav1/mavros/setpoint_raw/local
```

| 问题 | 处理方式 |
|---|---|
| `rostopic` 找不到 | 进入容器，执行 Noetic 和工作区的两个 source 命令 |
| Gazebo 窗口不出现 | 先保持终端 A 运行，再在宿主机桌面执行 `open_gazebo_gui.sh` |
| 启动脚本报告已有 PX4/Gazebo | 先结束原仿真，避免实例与端口冲突 |
| MAVROS `connected: False` | 检查相应 PX4 实例是否已启动 |
| `system_ready: False` | 查看 `/uavN/status` 与容器日志，确认启动状态是否推进 |
| `stale UAV states` | 检查位置流频率和负载；不要放宽冻结的新鲜度阈值 |
| Candidate 校验拒绝 | 检查机号、编队所需机数、坐标和规模是否合法 |

状态新鲜度窗口为 22.08 ms，控制器频率为 50 Hz；参数由冻结策略统一提供。

## 8. 停止与重新启动

等待当前任务结束，在自然语言调度器中输入 `q`。在**宿主机**停止 ROS1 容器：

```bash
docker stop ros1_multi_uav
```

回到终端 A 按 Ctrl+C，结束 Gazebo/PX4；关闭另外打开的图形窗口。可检查：

```bash
docker ps --format '{{.Names}}'
pgrep -a 'px4|gzserver|gzclient'
```

重新启动时重复第 3 节。

## 9. 机数、目录与适用范围

多机启动脚本支持 1～10 机。Gazebo、ROS1 控制器、调度器的机数必须一致。三机使用以下入口：

```bash
# 分别在对应宿主机终端执行
./scripts/run_gazebo_headless.sh 3
./scripts/run_multi_uav_sim.sh 3
./scripts/run_llm_scheduler.sh 3
```

三机可使用 Triangle；三机 Circle 会被论文的 cardinality 校验拒绝。内置示例和自动验证脚本是五机专用。UAV N 对应 PX4 instance N、MAV_SYS_ID N+1，默认出生偏移 Y=3N。

```text
src/location_allocate/     Candidate 解析、调度与安全感知分配
src/lfs_policy/            冻结策略及 typed loader
src/ladrc_controller/      LADRC、Minimum Jerk、IAPF、MAVROS 接口
src/uav_swarm_interfaces/  当前执行与状态消息
src/schemas/               Candidate JSON schema
examples/                  可运行任务
scripts/                   构建、启动、诊断与验证
validation/                仿真证据与回归结果
```

当前发布内容用于 Gazebo 仿真，不包含实机操作入口。三机和五机已实测；其他机数及完整论文正式实验不属于本次工程回归。仿真使用同一宿主机 wall time。

策略 SHA-256 为 `6b47d27f4253d7311e79ea51f6dd1cf0d0182e6df24374a94abae0aa6a135858`。来源与核心算法哈希见 [migration_source.json](migration_source.json)。
