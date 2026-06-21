# ROS1 + Gazebo Classic 多无人机仿真操作说明

日期：2026-06-21

本文档说明如何在项目 B 中启动 ROS1 + MAVROS + PX4 SITL + Gazebo Classic 多无人机仿真，并给出本次已实际验证通过的 8 机终端命令。

项目路径：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
```

PX4 路径：

```bash
cd /home/yihuang/PX4-Autopilot
```

## 0. 推荐执行流程

如果只是想直接跑完整 LLM 编队仿真，最短必要流程是：步骤 2 → 步骤 3 → 步骤 6，结束后执行步骤 7。

步骤 1、4、5 是可选步骤，主要用于首次运行、代码变更后验证、或排障。建议至少打开 3 个终端：终端 A 跑 PX4 + Gazebo，终端 B 启动 ROS1/MAVROS/控制节点，终端 C 启动 LLM 调度终端。

### 步骤 1（可选）：构建 ROS1 Docker 镜像和工作区

首次运行、代码变更后、或 Docker 镜像/工作区不存在时执行。已经构建过且代码没有变化时可以跳过。

终端 A：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/build_ros1_ws.sh
```

看到 `==> 编译完成` 后进入下一步。

### 步骤 2（必要）：启动 8 机 PX4 + Gazebo Classic

终端 A：

```bash
cd /home/yihuang/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
```

等待终端出现以下输出：

```text
Spawning iris_1 at 0.0 3
Spawning iris_2 at 0.0 6
...
Spawning iris_8 at 0.0 24
```

该终端必须保持运行。

### 步骤 3（必要）：启动 ROS1、MAVROS 和 LADRC 控制节点

终端 B：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/run_multi_uav_sim.sh 8
```

该命令会在 Docker 容器 `ros1_multi_uav` 中自动启动 `roscore`、8 个 MAVROS 节点和 8 个 LADRC 控制节点。该终端命令返回后，容器会在后台继续运行。

### 步骤 4（可选）：检查 8 机 ROS topic 和 MAVROS 状态

非必须。首次运行、换机器、改过网络/端口配置、或排障时建议执行。

终端 D：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8 && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10"
```

成功标志：

```text
==> 检查通过
==> 运行时检查通过
```

并且每架无人机应显示：

```text
connected: True
armed: True
mode: "OFFBOARD"
```

### 步骤 5（可选）：先用脚本做一次不经过 LLM 的 8 机飞行检查

非必须。这个步骤用于单独验证底层 ROS1/MAVROS/Gazebo 控制链路，确认不用 LLM 时无人机也能稳定接收目标点并飞行。

终端 D：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150"
```

成功标志：

```text
稳定悬停进度: 8/8
==> 指令飞行检查通过
```

### 步骤 6（必要）：启动 LLM 自然语言调度终端

终端 C：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
cp .env.minimax.example .env.minimax
nano .env.minimax
./scripts/run_llm_scheduler.sh 8
```

把 `.env.minimax` 中的 `MINIMAX_API_KEY` 改为真实 Key；后续再次运行时不需要重复配置。`.env.minimax` 已被 Git 忽略，不会提交。也可以临时用 `export MINIMAX_API_KEY="your-api-key"` 覆盖。

如果 `.env.minimax` 已经存在，可以跳过 `cp` 和 `nano`，直接执行：

```bash
./scripts/run_llm_scheduler.sh 8
```

如果使用默认 MiniMax 配置，不需要修改其他变量。如需指定接口或模型，可编辑 `.env.minimax`：

```bash
MINIMAX_BASE_URL=https://api.minimax.chat/v1
MINIMAX_MODEL_NAME=MiniMax-M2.7-highspeed
```

看到提示后输入自然语言：

```text
1到5号机在10秒内以[0,12,2]为中心组成圆形编队，半径为3米，使用smooth模式
```

成功标志：

- 终端打印 LLM 返回的 JSON 蓝图。
- 调度层打印匈牙利分配结果。
- Gazebo 中 UAV1-UAV5 自动变成圆形编队。
- 终端最终打印全部参与无人机悬停稳定。

继续输入下一条自然语言指令即可连续变阵；输入 `q` 退出调度终端。

### 步骤 7（必要）：结束仿真并清理进程

完成测试后执行，避免 Docker 容器、PX4 或 Gazebo 进程残留：

```bash
docker rm -f ros1_multi_uav
pkill -x gzclient || true
pkill -x gzserver || true
pkill -x px4 || true
pkill -f 'sitl_multiple_run.sh -n 8 -m iris' || true
```

确认已清理：

```bash
docker ps --format '{{.Names}} {{.Status}}'
ps -ef | rg 'px4|gzserver|gzclient|gazebo'
```

## 1. 已验证结论

本轮已通过终端自主完成 8 机仿真验证：

- Gazebo Classic 中 `iris_1` 到 `iris_8` 均 spawn 成功。
- ROS1 中 `/uav1` 到 `/uav8` 的 MAVROS 节点和 LADRC 控制节点均启动成功。
- `/uav1` 到 `/uav8` 的 topic 检查通过。
- `/uav1` 到 `/uav8` 的 MAVROS runtime 检查通过。
- 8 架无人机均进入 `connected: True`、`armed: True`、`mode: "OFFBOARD"`。
- 通过 `/uavN/swarm_command` 向 8 架无人机发布目标点后，8 架均反馈 `is_hover_stable: True`。
- 最终采样位置接近目标队形：`x≈0`、`y≈3*N`、`z≈1.65-1.73`。

本次验证命令链路：

```bash
/home/yihuang/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
./scripts/run_multi_uav_sim.sh 8
docker exec ros1_multi_uav bash -lc 'source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8 && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10 && /ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150'
```

## 2. 前置条件

确认 Docker 可用：

```bash
docker --version
docker ps
```

确认 PX4 目录存在：

```bash
test -d /home/yihuang/PX4-Autopilot && echo "PX4 路径存在"
```

确认当前位于 ROS1 工作区：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
pwd
```

## 3. 构建 Docker 镜像和 ROS1 工作区

首次运行或依赖变化后执行：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/build_ros1_ws.sh
```

也可以手动执行：

```bash
docker build -t ros1-mavros:latest .
docker run --rm --network host \
  -v "$(pwd):/ros1_ws" \
  ros1-mavros:latest \
  bash -lc "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"
```

成功标志：

- `catkin_make` 返回 0。
- 生成 `devel/lib/ladrc_controller/ladrc_position_controller_node`。
- 生成 `devel/lib/location_allocate/location_allocate_node`。

## 4. 终端 1：启动 PX4 多机 Gazebo 仿真

打开第一个终端：

```bash
cd /home/yihuang/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
```

预期输出包含：

```text
Spawning iris_1 at 0.0 3
Spawning iris_2 at 0.0 6
Spawning iris_3 at 0.0 9
Spawning iris_4 at 0.0 12
Spawning iris_5 at 0.0 15
Spawning iris_6 at 0.0 18
Spawning iris_7 at 0.0 21
Spawning iris_8 at 0.0 24
```

不要关闭该终端。它负责维持 Gazebo 和 PX4 SITL 实例。

## 5. 终端 2：启动 ROS1、MAVROS 和控制节点

打开第二个终端：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/run_multi_uav_sim.sh 8
```

该脚本会：

- 删除旧的 `ros1_multi_uav` 容器。
- 以 `--network host` 启动 ROS1 Docker 容器。
- 在容器内执行 `catkin_make`。
- 启动 `roscore`。
- 启动 `roslaunch ladrc_controller swarm.launch`。
- 自动关闭 UAV9、UAV10。

查看 ROS1 launch 日志：

```bash
docker logs -f ros1_multi_uav
```

进入容器调试：

```bash
docker exec -it ros1_multi_uav bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
```

## 6. 终端 3：执行基础 topic 检查

打开第三个终端：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8"
```

预期每架无人机都有以下 topic：

- `/uavN/mavros/state`
- `/uavN/mavros/local_position/odom`
- `/uavN/mavros/setpoint_position/local`
- `/uavN/swarm_command`
- `/uavN/status`
- `/uavN/odom`

成功标志：

```text
==> 检查通过
```

## 7. 执行 MAVROS runtime 检查

继续在第三个终端执行：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10"
```

该脚本检查：

- `/uavN/mavros/local_position/odom` 有实时数据。
- `/uavN/odom` 有控制器公开的全局 ENU 位置。
- `/uavN/mavros/state` 有数据。
- `connected: True`。

本轮验证中，8 架均达到：

```text
connected: True
armed: True
mode: "OFFBOARD"
```

成功标志：

```text
==> 运行时检查通过
```

## 8. 执行 8 机 swarm_command 指令飞行

继续在第三个终端执行：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150"
```

参数含义：

- `8`：无人机数量。
- `1.5`：目标全局 ENU 高度 Z。
- `8.0`：Minimum Jerk 轨迹持续时间，单位秒。
- `150`：等待全部稳定悬停的超时时间，单位秒。

脚本会向每架无人机发布：

```text
UAV1 -> [0.0, 3.0, 1.5]
UAV2 -> [0.0, 6.0, 1.5]
UAV3 -> [0.0, 9.0, 1.5]
UAV4 -> [0.0, 12.0, 1.5]
UAV5 -> [0.0, 15.0, 1.5]
UAV6 -> [0.0, 18.0, 1.5]
UAV7 -> [0.0, 21.0, 1.5]
UAV8 -> [0.0, 24.0, 1.5]
```

这些是全局 ENU 坐标。控制器会自动减去每架无人机的 `enu_offset_y=3.0*N`，因此每架无人机主要执行原地爬升，避免横向交叉。

成功标志：

```text
稳定悬停进度: 8/8
==> 指令飞行检查通过
```

## 9. 手动查看最终位置和状态

查看 8 架无人机最终位置和悬停状态：

```bash
docker exec ros1_multi_uav bash -lc '
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
for uid in $(seq 1 8); do
  echo "---- UAV${uid} ----"
  rostopic echo /uav${uid}/odom -n 1 | grep -E "^[xyz]:"
  rostopic echo /uav${uid}/status -n 1
done'
```

本轮验证采样结果：

```text
UAV1: x=-0.0068, y=3.0145,  z=1.7347, is_hover_stable=True
UAV2: x=-0.0078, y=6.0175,  z=1.7101, is_hover_stable=True
UAV3: x=-0.0121, y=8.9954,  z=1.6810, is_hover_stable=True
UAV4: x=-0.0278, y=11.9957, z=1.6712, is_hover_stable=True
UAV5: x=-0.0477, y=14.9950, z=1.6684, is_hover_stable=True
UAV6: x=-0.0537, y=18.0246, z=1.6612, is_hover_stable=True
UAV7: x=-0.0431, y=21.0201, z=1.6609, is_hover_stable=True
UAV8: x=-0.0296, y=24.0056, z=1.6537, is_hover_stable=True
```

## 10. 手动发布单架无人机指令

如需单独测试 UAV1：

```bash
docker exec ros1_multi_uav bash -lc '
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: '\''world'\''}, uav_id: 1, target_pos: {x: 0.0, y: 3.0, z: 1.5}, duration: 8.0, motion_style: '\''smooth'\'', safety_factor: 0.0}"
'
```

查看 UAV1 状态：

```bash
docker exec ros1_multi_uav bash -lc '
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
rostopic echo /uav1/status -n 1
rostopic echo /uav1/odom -n 1
'
```

## 11. 启动 LLM 调度节点

LLM 调度节点对应项目 A `Claude.md` 中的“认知层 + 调度层”：自然语言指令先由 MiniMax/OpenAI 兼容 API 解析为 JSON 蓝图，再由 ROS1 Python 调度层生成目标点、执行匈牙利分配并发布 `swarm_command`。

首次使用时，在项目根目录创建本地密钥文件：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
cp .env.minimax.example .env.minimax
nano .env.minimax
```

把 `.env.minimax` 中的 `MINIMAX_API_KEY` 改为真实 Key。该文件已被 `.gitignore` 忽略，不会提交。

可选覆盖默认模型配置，直接写在 `.env.minimax` 中：

```bash
MINIMAX_BASE_URL=https://api.minimax.chat/v1
MINIMAX_MODEL_NAME=MiniMax-M2.7-highspeed
```

启动 LLM 调度终端：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/run_llm_scheduler.sh 8
```

看到提示后输入自然语言指令：

```text
1到5号机在10秒内以[0,12,2]为中心组成圆形编队，半径为3米，使用smooth模式
```

预期现象：

- 终端打印 LLM 返回的 JSON 蓝图。
- 调度层打印当前 UAV 位置、目标编队点和匈牙利分配结果。
- `/uav1` 到 `/uav5` 收到 `swarm_command`。
- Gazebo 中 UAV1-UAV5 自动变为圆形编队。
- `/uav1/status` 到 `/uav5/status` 最终为 `is_hover_stable: True`。

也可以用管道输入做自动化验收：

```bash
printf '1到5号机在10秒内以[0,12,2]为中心组成圆形编队，半径为3米，使用smooth模式\nq\n' | ./scripts/run_llm_scheduler.sh 8
```

## 12. 关闭和清理

停止 ROS1 Docker 容器：

```bash
docker rm -f ros1_multi_uav
```

停止 PX4/Gazebo：

```bash
pkill -x gzclient || true
pkill -x gzserver || true
pkill -x px4 || true
pkill -f 'sitl_multiple_run.sh -n 8 -m iris' || true
```

确认没有残留进程：

```bash
docker ps --format '{{.Names}} {{.Status}}'
ps -ef | rg 'px4|gzserver|gzclient|gazebo'
```

## 13. 常见问题

### topic 存在但 `connected: False`

确认 PX4 多机脚本仍在运行：

```bash
ps -ef | rg 'px4|gzserver|sitl_multiple_run'
```

确认端口：

```bash
tail -80 /home/yihuang/PX4-Autopilot/build/px4_sitl_default/rootfs/0/out.log
```

UAV1 对应 PX4 instance 1，SITL 模式下 `target_system_id=2`。

### `rostopic pub` 后无人机不稳定

优先使用全局 ENU 坐标 `[0.0, 3.0*N, Z]`，不要把所有无人机都发布到同一个 Y 坐标，否则会出现横向交叉。

推荐命令：

```bash
/ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150
```

### Gazebo GUI 无法使用

本项目的主要验证路径是终端 headless/CLI 检查。GUI 不可用时，使用：

```bash
rosnode list
rostopic list
rostopic echo /uav1/odom -n 1
rostopic echo /uav1/mavros/state -n 1
```

## 14. 一键验证命令摘要

终端 1：

```bash
cd /home/yihuang/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
```

终端 2：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/run_multi_uav_sim.sh 8
```

终端 3：

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8 && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10 && /ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150"
```

终端 4：真实 LLM 自然语言调度验收

```bash
cd "/home/yihuang/learning/ros1_ws（复件）"
./scripts/run_llm_scheduler.sh 8
```

在提示符输入：

```text
1到5号机在10秒内以[0,12,2]为中心组成圆形编队，半径为3米，使用smooth模式
```
