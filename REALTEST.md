# 实机飞行测试 — 完整操作步骤

> ⚠️ **安全第一**: 首次飞行前务必完成步骤 1-5 的全部预检。任何不确定的情况立即停止。

## 配置总览

| 配置项 | 值 | 说明 |
|--------|-----|------|
| 无人机 | uav0 (system_id=1) | 单机测试 |
| 飞控 | NxtPX4v2 (H7, PX4) | |
| ESP32 IP | `tcp://10.1.1.81:5760` | TCP MAVLink 桥接 |
| Nokov 动捕主机 | `10.1.1.198:3883` | VRPN 服务器 |
| VRPN 刚体名 | `uav1` | Nokov 中的命名 |
| LADRC 参数文件 | `real_hardware_params.yaml` | 保守值 ωo=8, ωc=2 |
| 笔记本 | Ubuntu 22.04 + Docker | ROS1 在容器中运行 |

---

## 步骤 1 — 环境准备

### 1.1 确认网络连通

在笔记本终端执行：

```bash
# 测试 ESP32 连通性
ping -c 3 10.1.1.81

# 测试 Nokov 动捕主机连通性
ping -c 3 10.1.1.198
```

如果 ping 不通，检查笔记本是否已连接到实验室局域网（Wi-Fi 或有线）。

### 1.2 确认 Docker 镜像

```bash
sudo docker images | grep ros1-mavros
```

如果镜像不存在，构建它：
```bash
cd ~/ros1_ws
sudo docker build -t ros1-mavros:latest .
```

### 1.3 编译 ROS1 工作空间

```bash
cd ~/ros1_ws

# 编译（在 Docker 容器内）
sudo docker run --rm --network host \
    -v $(pwd):/ros1_ws \
    ros1-mavros:latest \
    bash -c "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make"
```

预期输出：`[100%] Built target ladrc_position_controller_node`，零 error。

---

## 步骤 2 — 启动 Docker 容器

```bash
# 清理旧容器
sudo docker rm -f ros1_test 2>/dev/null

# 启动新容器（后台运行）
sudo docker run -d --name ros1_test \
    --network host \
    -v /home/yihuang/learning/ros1_ws:/ros1_ws \
    ros1-mavros:latest \
    bash -c "
        source /opt/ros/noetic/setup.bash
        source /ros1_ws/devel/setup.bash
        roscore &
        sleep 3
        tail -f /dev/null
    "

# 进入容器
sudo docker exec -it ros1_test bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
```

> 📝 后续所有命令都在容器内执行。

---

## 步骤 3 — 安装 VRPN 环境（仅首次）

```bash
# 容器内执行
bash /ros1_ws/setup_real_hardware.sh
```

这个脚本会自动安装：
- `python3-pip`, `git`
- VRPN C 库 (`libvrpn-dev`)
- `vrpn_client_ros`（从源码编译）

安装完成后验证：
```bash
roslaunch vrpn_client_ros sample.launch --help
# 如果能看到帮助信息，说明安装成功
```

---

## 步骤 4 — 启动系统

```bash
# 容器内执行
roslaunch ladrc_controller real_hardware.launch uav_id:=1
```

这将依次启动以下节点：

| 启动顺序 | 节点 | 功能 |
|---------|------|------|
| 1 | `vrpn_client_ros` | 连接 Nokov(`10.1.1.198:3883`)，接收刚体 `uav1` 的 pose |
| 2 | `vrpn_vision_relay` | 转发 `/vrpn_client_node/uav1/pose` → `/uav1/mavros/vision_pose/pose` |
| 3 | `mavros` | 连接 ESP32(`tcp://10.1.1.81:5760`)，MAVLink 双向通信 |
| 4 | `ladrc_position_controller` | LADRC 控制器 + 自动起飞状态机 |

等待约 30 秒，观察日志输出。正常情况下你会看到：
```
LADRC 集群执行节点已初始化 (命名空间: /uav1)
已接收到 MAVROS local_position/odom 消息
系统稳定，发送解锁命令...
解锁成功。切换到 Offboard 模式...
Offboard 模式已激活。LADRC 控制器接管。
```

---

## 步骤 5 — 预检（不装桨叶！）

打开一个新的终端窗口，进入容器：

```bash
# 宿主机上新终端
sudo docker exec -it ros1_test bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
```

### 5.1 检查 MAVROS 连接

```bash
rostopic echo /uav1/mavros/state -n 1
```

预期输出：
```
connected: True     ← 必须为 True
armed: True         ← 状态机自动解锁
mode: "OFFBOARD"    ← 自动切换
```

### 5.2 检查 Nokov 动捕数据

```bash
# 查看 odom 位置数据
rostopic echo /uav1/mavros/local_position/odom -n 1
```

手持无人机在动捕区域中缓慢移动，再次运行上述命令，**确认 x/y/z 数值跟随你的实际移动方向和幅度变化**。

坐标系确认：
- **正 X**：无人机向前移动时 x 增大
- **正 Y**：无人机向左移动时 y 增大
- **正 Z**：无人机向上移动时 z 增大

### 5.3 检查 VRPN 原始数据

```bash
rostopic echo /vrpn_client_node/uav1/pose -n 1
```

确认有数据输出，且与 odom 中的位置大致对应。

### 5.4 检查 topic 列表完整性

```bash
rostopic list | grep uav1
```

预期至少包含：
```
/uav1/mavros/local_position/odom
/uav1/mavros/state
/uav1/mavros/setpoint_position/local
/uav1/mavros/vision_pose/pose
/uav1/swarm_command
/uav1/status
/uav1/odom
```

### 5.5 安全功能验证

```bash
# 确认 disarm 服务可用（不执行，只确认存在）
rosservice info /uav1/mavros/cmd/arming
```

**遥控器检查**：
- [ ] 遥控器切 Stabilized 模式，`rostopic echo /uav1/mavros/state` 中 mode 应变
- [ ] Kill Switch 功能可用

---

## 步骤 6 — 装桨，首次飞行

> ⚠️ **到达此步骤前，必须完成步骤 5 的全部预检！**

### 6.1 装桨 + 上电

1. 安装螺旋桨，确认方向正确
2. 飞控上电
3. 确认遥控器就绪，初始位置在 Stabilized 模式
4. 人员撤出飞行区域
5. 在容器内重新执行步骤 4 启动系统（如果之前已关闭）

### 6.2 观察自动起飞

系统启动后，状态机会自动完成：**解锁 → 切 Offboard → 起飞悬停**

观察日志确认：
```
悬停保持锁定: [x.xx, x.xx, x.xx]
```

此时无人机应已经离地并在空中稳定悬停。

### 6.3 发送首次 ±1m 测试指令

```bash
# X 轴正向移动 1m，限时 5 秒，smooth 模式
rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \
  "{header: {stamp: now, frame_id: 'world'}, uav_id: 1, \
    target_pos: {x: 1.0, y: 0.0, z: 1.5}, duration: 5.0, \
    motion_style: 'smooth', safety_factor: 0.0}"
```

### 6.4 观察跟踪效果

```bash
# 实时监控位置
rostopic echo /uav1/odom
```

观察无人机是否平滑移动到目标点 (1.0, 0.0, 1.5)，注意：
- 有无明显超调/震荡
- 到达后是否稳定（`/uav1/status` 中 `is_hover_stable` 变 True）
- 实际位置与目标偏差

### 6.5 逐步增大测试

首次 ±1m 成功后，逐步增大指令范围：

```bash
# 第二次：±2m
rostopic pub -1 /uav1/swarm_command ... \
  "{... target_pos: {x: 2.0, y: 0.0, z: 2.0}, duration: 5.0, ...}"

# 第三次：±3m
rostopic pub -1 /uav1/swarm_command ... \
  "{... target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, ...}"
```

---

## 步骤 7 — 在线调参

首次飞行后根据跟踪表现微调 LADRC 参数：

```bash
# 如果跟踪滞后（响应慢），逐步调高 ωc (0.2 步进)：
rosparam set /uav1/ladrc_position_controller/omega_c_x 2.2
rosparam set /uav1/ladrc_position_controller/omega_c_y 2.2

# 如果震荡（过冲），调低 ωc：
rosparam set /uav1/ladrc_position_controller/omega_c_x 1.8

# 如果垂直方向跟踪差，调整 Z 轴：
rosparam set /uav1/ladrc_position_controller/omega_c_z 2.8

# 查看当前参数
rosparam get /uav1/ladrc_position_controller/omega_c_x
```

**调参目标**：响应快 + 无震荡 + 稳态误差 < 0.2m

确认最终参数后，写回配置文件 `real_hardware_params.yaml` 以备下次使用。

---

## 步骤 8 — 正常降落

测试完成后：

```bash
# ROS disarm（降落）
rosservice call /uav1/mavros/cmd/arming "{value: false}"
```

或者通过遥控器切 Stabilized 模式后手动降落。

---

## 紧急处置

| 情况 | 操作 | 效果 |
|------|------|------|
| 异常飞行/震荡 | 遥控器切 **Stabilized** | 无人机立即退出 Offboard，稳定悬停 |
| 严重失控 | 遥控器 **Kill Switch** | 立即停桨（无人机会坠落！） |
| ROS 侧急停 | `rosservice call /uav1/mavros/cmd/arming "{value: false}"` | ROS disarm |
| 程序崩溃 | 保持冷静，遥控器接管 | Stabilized 模式不依赖 ROS |

**紧急处置优先级**：遥控器 > ROS 指令

---

## 测试通过标准

| 编号 | 测试项 | 通过标准 |
|------|--------|---------|
| H1 | 网络连通 | ping ESP32 + Nokov 主机均通 |
| H2 | MAVROS 连接 | `connected: True` |
| H3 | VRPN 动捕数据 | odom 跟随无人机实际移动 |
| H4 | 坐标系方向 | ENU: 正X前、正Y左、正Z上 |
| H5 | 自动起飞 | 状态机完成 arming + offboard，无人机离地悬停 |
| H6 | 悬停稳定 | 无人机无明显漂移 |
| H7 | ±1m 指令响应 | 无人机移动到目标 1m 附近，偏差 < 0.3m |
| H8 | 悬停到达信号 | `is_hover_stable: True` |
| H9 | ±3m 指令响应 | 逐步增大后仍能稳定跟踪 |
| H10 | 正常降落 | 遥控器或 ROS 指令可安全降落 |

---

## 常见问题

| 问题 | 可能原因 | 解决 |
|------|---------|------|
| MAVROS 连不上 | ESP32 未上电/WiFi 未连接 | 确认无人机上电，`ping 10.1.1.81` |
| VRPN 无数据 | Nokov 未标定/刚体名不匹配 | 确认 Nokov 软件中刚体名为 `uav1` |
| 解锁失败 | 安全开关未按下/预检未过 | 遥控器检查，QGC 查看 prearm 报告 |
| Offboard 切换失败 | setpoint 流 < 2Hz | 确认控制节点 50Hz 定时器正常 |
| 无人机不响应指令 | 状态机未到 RUNNING_TRAJECTORY | 查看日志确认状态 |
| 位置偏差大 | 动捕标定不准/LADRC 参数不合适 | 重新标定/在线调参 |
| 震荡发散 | LADRC 带宽过高 | 降低 ωc，增加 ωo |
