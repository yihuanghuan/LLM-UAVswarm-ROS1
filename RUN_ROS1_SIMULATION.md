# ROS1 仿真运行说明

## 1. 进入 Docker / 构建镜像

在项目 B 根目录执行：

```bash
docker build -t ros1-mavros:latest .
```

如果当前用户没有 Docker 权限，可在命令前加 `sudo`，或使用：

```bash
DOCKER_CMD="sudo docker" ./scripts/build_ros1_ws.sh
```

## 2. 编译 ROS1 工作区

推荐：

```bash
./scripts/build_ros1_ws.sh
```

等价 Docker 命令：

```bash
docker run --rm --network host \
  -v "$(pwd):/ros1_ws" \
  ros1-mavros:latest \
  bash -lc "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"
```

## 3. 启动 roscore

单独调试时可在容器内启动：

```bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
roscore
```

`scripts/run_multi_uav_sim.sh` 会自动在容器内启动 `roscore`。

## 4. 启动 PX4 / Gazebo Classic

单机：

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

多机 headless：

```bash
cd ~/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
```

如只做有限时间验证：

```bash
timeout 240s Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 8 -m iris
```

## 5. 启动 MAVROS 和控制节点

### 单机

```bash
roslaunch ladrc_controller single_uav.launch uav_id:=1 enu_offset_y:=0.0
```

或使用已有脚本：

```bash
./test_single_uav.sh
```

### 多机

先启动 PX4 多机 SITL，再执行：

```bash
./scripts/run_multi_uav_sim.sh 8
```

默认最多支持 UAV1-UAV10。只启用部分无人机时，脚本会自动向 `swarm.launch` 传入 `enable_uavN:=false`。

手动 launch 示例：

```bash
roslaunch ladrc_controller swarm.launch \
  enable_uav3:=false enable_uav4:=false enable_uav5:=false \
  enable_uav6:=false enable_uav7:=false enable_uav8:=false \
  enable_uav9:=false enable_uav10:=false
```

## 6. 启动 LLM 调度节点

LLM 调度节点运行在 `ros1_multi_uav` 容器内。首次使用时，在项目根目录创建本地密钥文件：

```bash
cp .env.minimax.example .env.minimax
nano .env.minimax
```

把 `.env.minimax` 中的 `MINIMAX_API_KEY` 改为真实 Key。该文件已被 `.gitignore` 忽略，不会提交。

如需临时覆盖，也可以直接在当前终端设置：

```bash
export MINIMAX_API_KEY="your-api-key"
```

启动 8 机 LLM 调度终端：

```bash
./scripts/run_llm_scheduler.sh 8
```

看到 `请输入无人机编队指令:` 后输入自然语言，例如：

```text
1到5号机在10秒内以[0,12,2]为中心组成圆形编队，半径为3米，使用smooth模式
```

调度节点会调用真实 LLM API，打印 JSON 蓝图，执行匈牙利分配，并向 `/uavN/swarm_command` 下发目标点。

## 7. 检查 node / topic / param

检查 8 机 topic：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_topics.sh 8"
```

检查 8 机 MAVROS 运行时连接、odom 和 offboard 状态：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_runtime.sh 8 10"
```

执行 8 机 `swarm_command` 指令飞行并等待稳定悬停：

```bash
docker exec ros1_multi_uav bash -lc \
  "source /opt/ros/noetic/setup.bash && source /ros1_ws/devel/setup.bash && /ros1_ws/scripts/check_multi_uav_command_flight.sh 8 1.5 8.0 150"
```

更完整的多机仿真步骤见：

```text
docs/ROS1_GAZEBO_MULTI_UAV_SIMULATION.md
```

手动检查：

```bash
rosnode list
rostopic list | grep uav1
rostopic echo /uav1/mavros/state -n 1
rostopic echo /uav1/mavros/local_position/odom -n 1
rostopic echo /uav1/odom -n 1
rosparam list | grep uav1
```

发送测试指令：

```bash
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: 'world'}, uav_id: 1, \
    target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, \
    motion_style: 'normal', safety_factor: 0.0}"
```

## 8. 常见报错

### Docker 没有权限

现象：

```text
permission denied while trying to connect to the Docker daemon socket
```

处理：

```bash
DOCKER_CMD="sudo docker" ./scripts/build_ros1_ws.sh
```

### MAVROS 走到 `/dev/ttyACM0`

说明 launch 进入了实机分支。确认：

```bash
roslaunch --dump-params ladrc_controller swarm.launch | grep fcu_url
```

SITL 应显示：

```text
/uav1/mavros/fcu_url: udp://:14541@127.0.0.1:14581
```

### topic 存在但 `mavros/state` 未连接

检查 PX4 多机脚本是否正在运行，并确认端口和 target system id 匹配：

```bash
tail -80 ~/PX4-Autopilot/build/px4_sitl_default/rootfs/0/out.log
tail -80 ~/PX4-Autopilot/build/px4_sitl_default/rootfs/1/out.log
```

PX4 instance 1 预期包含：

```text
udp port 14581 remote port 14541
```

ROS1 SITL 模式下 `swarm.launch` 使用 `target_system_id=N+1`，即 UAV1 对应 PX4 system id 2；实机模式仍使用 `target_system_id=N`。

### LLM 调度节点 import 失败

重新构建 Docker 镜像：

```bash
docker build -t ros1-mavros:latest .
./scripts/build_ros1_ws.sh
```

### Gazebo GUI 无法启动

优先使用 headless `gzserver` 路径：

```bash
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2 -m iris
```

如果仍失败，记录终端错误并改用：

```bash
rosnode list
rostopic list
rostopic echo /uav1/odom -n 1
```
