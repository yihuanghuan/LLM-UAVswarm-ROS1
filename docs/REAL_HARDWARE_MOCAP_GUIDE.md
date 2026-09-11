# ROS1 动捕实验室实机迁移与配置说明

适用项目：`~/learning/ros1_ws`。编写日期：2026-09-11。

本文依据当前工作区源码编写，供后续实施人员完成硬件选型、接口开发、参数配置和验收。当前项目仍是 PX4 SITL + Gazebo 仿真版本；本文是迁移说明，**不代表实机接口已实现或参数已经飞行验证**。所有“待填写”项由实验室实施人员确认，不需要原需求提出者预先提供。

迁移的主要工作是：接入 VRPN 并向飞控提供外部定位；统一所有飞机的坐标与时间；替换仿真通信和启动入口；补齐人工启动、定位失效和降落流程；按实机能力同步标定规划、LADRC 与 IAPF 参数。上层 Candidate 任务和调度接口可尽量保留。

## 1. 实施前填写的配置表

### 1.1 实验室与软件

| 项目 | 待填写内容 | 用于决定 |
|---|---|---|
| 飞机型号、旋翼包络尺寸、含电池质量 | 待填写 | 动力学、间距、制动距离 |
| 飞控型号、固件名称、版本及 commit | 待填写 | 是否适用 PX4/MAVROS 路线、具体参数名 |
| MAVROS、ROS1、VRPN 客户端版本 | 待填写 | 插件、消息、时间戳行为 |
| 动捕品牌、服务器 IP、VRPN 端口 | 待填写；端口以服务端设置为准 | VRPN 连接 |
| 动捕位置单位、轴方向、右/左手系 | 待填写；附坐标示意图 | 世界坐标变换 |
| 姿态定义、四元数顺序、刚体原点 | 待填写 | 姿态与杆臂校准 |
| 动捕是否提供完整姿态/速度/质量标志 | 待填写 | Pose 或 Odometry 输入、有效性判断 |
| 采样频率、实际输出频率、时间戳来源 | 待填写 | 新鲜度、延迟与滤波 |
| 原点位置、可用飞行区域、障碍物 | 待填写；测量地面、天花板、墙面和立柱 | 工作空间、运行时边界 |
| 飞机数量、最大速度、最小中心间距 | 待填写 | 机群配置、运动约束 |
| 计算部署 | 集中电脑 / 各机机载电脑：待填写 | ROS 网络、链路和故障归属 |
| 飞控通信 | Wi-Fi/以太网、数传或串口：待填写 | FCU URL、带宽、设备映射 |
| 操作流程 | 人工起飞后接管 / 人工许可后程序起飞：待填写 | 启动状态机分支 |
| 遥控接管、降落区、失联处理责任人 | 待填写 | 故障动作与验收 |

### 1.2 每架飞机的身份表（按实际数量扩展）

| ROS 命名空间 | 实体编号 | VRPN 刚体名 | MAV_SYS_ID | FCU URL | 控制器所在主机 | 本地到 world 标定 |
|---|---|---|---|---|---|---|
| `/uav1` | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |
| `/uav2` | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |

ROS 机号、VRPN 刚体名和 MAVLink system ID 是三个独立标识，不要求相等。必须一一对应；不能沿用仿真的 `MAV_SYS_ID=N+1` 规则。多机逐架上电核对，确认移动实体 A 只改变 A 的定位，给 A 的命令只到 A 的飞控。

## 2. 接入架构与选择分支

默认实施路线是继续使用 PX4 + MAVROS，保留当前任务接口与 LADRC 加速度控制。若实际飞机使用其他飞控或专有 SDK，应重新实现定位注入、状态反馈、控制输出、模式与解锁接口；不能只换 IP，本文中的 PX4 参数也不再适用。

```text
动捕服务器 → VRPN 客户端 → 每机定位适配器（新增）
                            │ 坐标、刚体外参、时间、质量检查
                            ↓
                    每机 MAVROS 外部定位输入
                            ↓
                         PX4 EKF
                            ↓
             MAVROS local_position/odom + state
                            ↓
             控制器 → world swarm_state → Candidate 调度器
                ↑                               │
                └──────── execution_command ────┘
                │ Minimum Jerk + LADRC + IAPF
                ↓
           MAVROS setpoint_raw/local → 实体飞控 → 电机
```

这一路线让控制器继续使用飞控融合后的状态，动捕与 IMU 融合由飞控承担。**只把 VRPN 发给电脑端控制器，不等于飞控已经得到可用定位。**

实施顺序建议先验证 `px4_position`，再验证 `ladrc_acceleration`。代码支持这两个字符串，但当前 `px4_position` 在 `iapf_dual` 下还会发送加速度前馈，并不自动等于“只有位置字段有效”；见第 5 节。位置模式验证成功也不能替代 LADRC 实机验证，两者实验结果应分别标记。

若要直接用动捕作为控制器状态，需额外实现带速度估计的状态适配器，明确速度坐标、延迟及失效行为，并保证控制输出使用的飞控本地坐标与动捕坐标一致。除非实验明确要求，不作为首轮迁移路线。

## 3. 接口修改清单

以下源文件路径均相对 ROS1 工作区。标为“新增”的名称是建议名称，当前不可当作已有节点或参数使用。

| 接口/模块 | 当前实现 | 实机需要做什么 | 类型 |
|---|---|---|---|
| 仿真入口 | `scripts/run_gazebo_headless.sh`、`scripts/run_multi_uav_sim.sh` | 新建实机入口，不启动 Gazebo/SITL；避免旧脚本删除重建飞行容器 | 新增入口 |
| MAVROS 连接 | `src/ladrc_controller/launch/single_uav.launch`、`swarm.launch` 使用本机 SITL UDP 端口 | 每机显式配置 `fcu_url`、`target_system_id`、`target_component_id`，核对插件配置 | 新增实机 launch |
| VRPN 输入 | 当前工作区无可用 VRPN 接入节点 | 引入 `vrpn_client_ros`，新增坐标/时间/质量适配器 | 新增依赖和代码 |
| 外部定位注入 | 当前由仿真飞控产生状态 | 适配器输出到每机 `mavros/vision_pose/pose` 或 `mavros/odometry/out`，同时配置 EKF | 新增接口 + 飞控配置 |
| 自机里程计 | 控制器订阅 `mavros/local_position/odom`，类型 `nav_msgs/Odometry` | 推荐保留；补充源时间检查、定位质量/重置检查 | 修改有效性检查 |
| 邻机里程计 | 直接订阅 `/uavN/mavros/local_position/odom`，写死 Y 偏移 | 去除 `3.0 * neighbor_id`，改用已统一的 world 状态或每机外参 | **必须改代码** |
| 全局状态 | 发布 `/uavN/swarm_state`，类型 `nav_msgs/Odometry` | 保留调度契约，修改源时间与必要的坐标变换 | 修改构造器和回调 |
| 任务命令 | `/uavN/execution_command`，`UAVExecutionCommand` | 保留；任务区域、速度及机号必须使用实机策略 | 配置 + 验证 |
| 控制输出 | `mavros/setpoint_raw/local`，`mavros_msgs/PositionTarget` | 保留 PX4 路线，核对掩码、yaw、实际固件支持及最终输出限幅 | 核验/必要时改代码 |
| 解锁/模式 | `mavros/cmd/arming`、`mavros/set_mode` | 新增人工许可、人工接管分支、模式应答和落地确认 | **必须改代码** |
| 就绪/完成 | `/uavN/status`、`startup_event` | 增加定位质量和人工许可门槛；旧 `system_ready` 不足以证明实机就绪 | 修改状态逻辑 |
| 状态流请求 | `configure_mavros_stream.py` 硬编码消息 32、100 Hz | 参数化申请频率；检查实际回传与链路负载 | 改代码 + 配置 |
| 策略加载 | `src/lfs_policy/lfs_policy/loader.py`、同步脚本 | 增加独立实机策略入口及校验，防止继续加载冻结仿真参数 | 改代码 + 新配置 |

主要代码定位：[控制器](../src/ladrc_controller/src/ladrc_position_controller_node.cpp)、[启动状态机](../src/ladrc_controller/include/ladrc_controller/startup_state_machine.hpp)、[状态构造器](../src/ladrc_controller/include/ladrc_controller/swarm_state_builder.hpp)、[调度状态读取](../src/location_allocate/src/location_allocate/state_ingest.py)。

## 4. VRPN、定位融合与坐标

### 4.1 VRPN 到 MAVROS

VRPN ROS 客户端使用上游 [vrpn_client_ros](https://github.com/ros-drivers/vrpn_client_ros)，在实机环境中固定版本。当前 Dockerfile 未安装该依赖，实施人员需在对应 Noetic 镜像中安装可用包或固定源码构建，并记录版本。

按客户端实际 launch 确认服务器和端口参数。常见入口和输出如下，IP 必须替换；此命令只启动客户端：

```bash
roslaunch vrpn_client_ros sample.launch server:=<动捕服务器IP>
# 常见输出：/vrpn_client_node/<刚体名>/pose
```

PX4 文档给出了该 VRPN 入口，以及 Pose 到 `vision_pose/pose`、Odometry 到 `odometry/out` 的注入路线。EKF2 路线优先选 vision 输入，不能仅因数据来自动捕就选择 `mavros/mocap/pose`；还要核实所用固件对该消息的融合支持。[PX4 外部定位接入](https://docs.px4.io/main/en/ros/external_position_estimation)

新增适配器必须完成：

1. 按身份表匹配刚体，转换为米和约定的世界坐标；不能只改 `frame_id` 字符串。
2. 转换完整姿态并检查四元数有效性；只有位置时不能伪造固定姿态并启用外部 yaw 融合，应由实际飞控能力确定独立航向来源。
3. 校准刚体原点到机体参考点的刚性外参。若在适配器中已补偿杆臂，飞控端不能再次补偿相同偏移。
4. 保留可解释的源时间，拒绝乱序、过期和无效样本；跟踪丢失时停止正常定位发布并上报状态，不能重复旧位姿并改写为当前时间。
5. 若输出 `nav_msgs/Odometry`，提供经过验证的速度、协方差与 frame 定义；按所用 MAVROS odometry 插件配置 TF。不要把世界速度直接填入声称是机体系的 twist。

推荐首版以 `geometry_msgs/PoseStamped` 注入 `/uavN/mavros/vision_pose/pose`，由 EKF 提供融合后的速度。若实际需要速度/协方差注入，再选 Odometry 路线。同一架飞机避免重复注入同源测量到多个融合入口。

### 4.2 统一坐标的具体改动

定义 `world` 为实验室共同参考系，Z 向上，明确 X/Y 的物理方向和原点。项目沿用 ENU 接口约定；如果采用实验室任意水平轴，必须把相对飞控航向的旋转显式标定，不能假定轴向自然一致。

记 `T_A_B` 为把 B 系坐标变到 A 系的变换，则：

```text
T_world_body = T_world_mocap × T_mocap_rigidbody × T_rigidbody_body
p_world = R_world_local × p_local + t_world_local
v_world = R_world_local × v_local
p_local_target = R_world_localᵀ × (p_world_target - t_world_local)
a_local_target = R_world_localᵀ × a_world_target
```

| 源码位置/约定 | 当前行为 | 实机要求 |
|---|---|---|
| `enu_offset_x/y/z` | 仅平移，自机测量加偏移，目标反向减偏移 | 只有轴向完全一致时才可继续使用；否则引入旋转并同步处理目标、速度、加速度及 yaw |
| 邻机订阅回调 | 邻机 Y 加 `3.0 * neighbor_id` | 必须删除仿真假设；只把自机 offset 改为 0 不够 |
| `toInternalOdometry()` | MAVROS ENU 输入转内部 NED；先用姿态将机体系速度转到世界系 | 保留完整转换关系；替换里程计来源时重新核对 twist 定义 |
| `publishTrajectorySetpoint()` | ROS 端填 ENU，设置 `FRAME_LOCAL_NED`，交给 MAVROS 转换 | 不要因枚举名含 NED 再手动交换 X/Y 或反转 Z |
| `/swarm_state` | `frame_id=world`、`child_frame_id=uavN/base_link_enu`；twist 为世界轴分量，姿态固定单位四元数 | 这是项目已有调度契约，不是完整机体姿态估计；不得直接拿它作为飞控外部 Odometry 输入 |
| `state_ingest.py` | 严格检查上述 frame 名 | 若改变契约，必须同时更新生产端、消费端及测试 |

建议邻机 IAPF 改为订阅 `/uavN/swarm_state`，按其既有世界速度语义直接读取，**不要再调用假定 twist 是机体系的 `toInternalOdometry()`**。也可使用每机独立外参转换原始里程计，但两条路线只能选择一条统一执行。

即使所有飞机使用同一个 VRPN 世界坐标，仍应实测 EKF 本地原点和航向是否一致。共同定位输入并不自动证明所有本地坐标重合。若已验证同轴同原点，offset 可全零；若存在偏移，标定后配置；若飞控运行中重置原点/航向，应中止当前轨迹、重新建立状态与观察器，不能继续使用旧变换。

无桨验证：沿 world 的 +X、+Y、+Z 分别移动各飞机，核对 VRPN、融合里程计、swarm_state 和邻机距离；原地旋转飞机验证世界位置不随杆臂错误绕圈，静止速度接近零。当前输出 `yaw = π/2 - yaw_ref`，调用处常传 `0.0`，因此 ROS 输出会是 `π/2`；实施人员需明确期望航向并核验，不能把这个 0 当作 ENU 零航向。

### 4.3 PX4 配置项：按实际固件核对

以下是配置检索项，不是可直接导入的参数文件；枚举/位掩码按实际固件版本的参数说明填写。新旧版本可能分别使用 `EKF2_EV_CTRL` 与 `EKF2_AID_MASK` 等不同参数。[当前外部定位配置说明](https://docs.px4.io/main/en/ros/external_position_estimation)、[v1.13 对照](https://docs.px4.io/v1.13/en/ros/external_position_estimation)

| 配置主题 | 常见检索项 | 实施要求 |
|---|---|---|
| 外部定位融合 | `EKF2_EV_CTRL`；旧版查 `EKF2_AID_MASK` | 按实际有无位置、速度、航向分别启用，勿盲填固定掩码 |
| 高度来源 | `EKF2_HGT_REF`；旧版查 `EKF2_HGT_MODE` | 选择并验证室内高度基准，核对与地面实际高度的关系 |
| 测量时间与外参 | `EKF2_EV_DELAY`、`EKF2_EV_POS_X/Y/Z` | 用日志标定延迟，明确杆臂由哪一层补偿 |
| 测量噪声与其他融合源 | 实际固件的 EV 噪声、GNSS/气压计/测距/航向配置 | 依据有效传感器和测量质量设置，记录保留/停用理由 |
| 通信 | `MAV_SYS_ID`、对应 `MAV_*` 实例和串口/网络参数 | 确保消息路由、波特率及双向带宽 |
| 动力学限制 | 对应固件的速度、加速度、倾角、推力和悬停推力参数 | 先完成机型基础调试，再验证外部指令通道实际受哪些约束 |
| Offboard 丢失 | `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` | 按位置仍有效/位置失效两种情况验证实际动作 |
| 其他故障 | 对应固件的定位失效、遥控失联、低电量及落地检测参数 | 在实验室条件下验证，不能假定室外返航适合室内 |

不能仅凭 MAVROS 有话题判断 EKF 已融合：还应查看飞控日志中的外部定位接收、融合状态/创新量、位置和航向有效性，确认预飞检查通过，并保存参数导出。

## 5. 控制、启动与故障处理

### 5.1 保留与核验控制输出

当前默认 `ladrc_acceleration` 输出有效加速度和 yaw，忽略位置、速度及 yaw rate；正常有限加速度时 `type_mask=2111`，FORCE 位不置位。应检查实际飞控接收和采用了预期指令，不要把它当成电机推力指令。不要擅自在项目输出叠加重力常数；先按飞控对应版本的加速度接口语义验证。

`px4_position` 的纯位置输出掩码为 2552；有加速度前馈时为 2104。当前 `iapf_dual` 会启用位置模式的加速度前馈，因此若首轮要求纯位置测试，需要明确配置/实现该分支，并检查实际掩码。切换模式应在明确的测试阶段完成；当前参数读取与观察器初始化不能视为支持飞行中任意热切换。

Offboard 需要持续发送 setpoint，预发送后才能进入；失去该流会按飞控配置退出。最低保活要求不能作为控制频率设计目标，应测量真实的连续发布间隔。[PX4 Offboard 说明](https://docs.px4.io/main/en/flight_modes/offboard)

### 5.2 必须补齐的启动分支

现有 `StartupStateMachine` 会自动请求 ARM、OFFBOARD 和起飞，源码没有可直接使用的 `auto_arm=false` 配置。实机 launch 不能仅隐藏调度器就认为不会起飞。

建议新增以下能力（参数名由实施人员确定）：

- 默认进入仅观测、禁止解锁状态；人工许可作为独立状态机输入，而非无限加长 `startup_settle_time`。
- 若人工起飞后接管：确认已解锁、定位有效和目标模式，捕获当前位姿作为接管参考，平滑衔接控制，跳过原来的自动起飞分支。
- 若程序起飞：人工许可后预发送有效参考、请求解锁/模式并等待反馈，再执行受限起飞轨迹。
- 模式被人工切走时退出任务控制，不能自动争抢回 OFFBOARD。重连或控制器重启后必须重新许可，不能自动恢复旧任务。
- 加入降落请求、落地确认与任务清理；落地后再按操作规程上锁。禁止用空中 DISARM 代替正常降落。

**起飞高度陷阱：** `stateMachine()` 把 `hover_hold_z_` 直接赋为 `startup_takeoff_altitude`。当前 1.5 表示飞控本地 ENU 的绝对 Z=1.5，并非“从当前地面上升 1.5 m”。实机应明确改为“捕获地面本地 Z + 相对高度”，或者把世界目标高度转换到本地；同时核对接管时不能再次触发起飞。

### 5.3 故障处理边界

现有 `failsafe` 来自 MAVROS 连接状态和部分 `system_status`，不是完整的 EKF 质量检查。启动失败会锁止并清空任务；定位过期时控制循环停止正常 setpoint 发布。这些行为不构成完整的实机降落流程。

| 事件 | 需要实现并验收的行为 |
|---|---|
| VRPN 遮挡/停止，但 EKF 还持续输出 odom | 同时监测动捕源健康与 EKF 融合健康，不能仅以 odom 频率判断正常 |
| 自机定位过期、跳变或航向/原点重置 | 禁止新任务，退出轨迹，选择已验证的飞控降级/人工接管动作；无有效定位时不能承诺定点悬停 |
| 邻机状态过期 | 显式触发机群任务中止或预定退避/降落策略；不能只将邻机从 IAPF 计算中排除 |
| 调度器或 LLM 服务断开 | 控制端独立保持已定义的安全状态；任务停止/取消如何到达全部飞机必须明确 |
| 控制电脑/链路断开 | 由飞控独立检测 Offboard 丢失，按已验证动作处理 |
| 越界或接近障碍物 | 增加运行时边界监控；规划空间检查不能覆盖跟踪误差及 IAPF 修正后的实际轨迹 |
| 单机故障影响多机 | 通知全队停止新任务，按各机定位有效性和降落区域执行协调退出 |

目前 IAPF 面向邻机，不应视为墙面、天花板和人员避障系统。故障动作需在定位仍有效和定位已失效两种条件下分别验证。停掉 Docker、退出调度器或关闭终端均不是正常降落指令。

## 6. 需要重新标定的项目参数

以下“当前值”来自现有仿真 YAML，**用于定位修改点，不是实机推荐值**。实机值在测量和分阶段验证后填写。

### 6.1 控制器参数

配置来源：[ladrc_params.yaml](../src/ladrc_controller/config/ladrc_params.yaml)、[paper_base.yaml](../src/ladrc_controller/config/paper_base.yaml)。应新建独立实机配置，不直接覆盖冻结文件。

| 参数 | 当前值 | 实机调整依据 |
|---|---|---|
| `control_frequency` | 50 Hz | CPU 调度、网络及飞控处理能力；代码 `dt_=1/frequency`，改变频率需重新验证控制器离散化与超时 |
| `control_mode` | `ladrc_acceleration` | 按第 5 节分阶段验证 |
| `b0_x/y/z` | 1/1/1 | 实机“加速度指令→运动响应”的有效增益，不是直接填质量或电机推力系数 |
| `omega_c_x/y/z` | 1.5/1.5/1.75 | 实机闭环带宽、跟踪误差和饱和情况 |
| `omega_o_x/y/z` | 5/5/7.5 | 观察器响应与测量噪声、延迟的折中，不可只追求高带宽 |
| `max_acceleration_x/y/z` | 5/5/8 m/s² | 水平/垂直可实现加速度、倾角与推力余量 |
| `max_velocity` | 5 m/s | 该参数虽声明，当前节点未见其直接执行速度限幅；必须通过规划约束和运行时监测实现，不能只改它 |
| `startup_takeoff_altitude` | 1.5 m | 按第 5.2 节修正高度语义后配置 |
| `startup_settle_time` / `startup_prestream_time` | 10 / 1.5 s | EKF 稳定与预发送所需实测时间；不能代替人工许可 |
| `startup_speed_tolerance` | 0.15 m/s | 静止速度噪声和起飞稳定判据 |
| `startup_takeoff_position_tolerance` / `startup_takeoff_hold_time` | 0.25 m / 0.5 s | 实际稳定精度及持续时间 |
| `startup_odom_timeout` / `startup_status_timeout` | 0.5 / 2 s | 源数据年龄、链路抖动及可接受反应距离 |
| `startup_runtime_fault_debounce` | 0.5 s | 与前置超时累积后的总反应时间，不应分别独立放宽 |
| `startup_total_timeout` / `startup_max_request_attempts` / `startup_command_retry_interval` | 60 s / 20 / 1 s | 人工操作流程、服务反馈；保留失败锁止 |
| `hover_position_enter/exit_tolerance` | 0.4 / 0.5 m | 任务完成精度，确保退出阈值大于进入阈值 |
| `hover_velocity_enter/exit_tolerance` | 0.3 / 0.4 m/s | 稳态速度噪声与实际速度要求 |
| `hover_stable_hold_time` / `hover_velocity_filter_tau` | 1 / 0.5 s | 稳定持续时间与滤波延迟；不能靠强滤波掩盖振荡 |
| `neighbor_uav_ids` / `neighbor_timeout` | YAML 为 1～10 / 0.2 s | 实际参与飞机集合及邻机状态年龄；每机一致且排除不存在的飞机 |
| `enu_offset_x/y/z` | 由 launch 设置，仿真 Y 随出生点变化 | 仅用于经验证的平移，旋转见第 4 节 |

### 6.2 规划、安全与执行 Profile：必须同步调整

来源：[lfs_policy.paper_current.yaml](../src/lfs_policy/config/lfs_policy.paper_current.yaml)。每个任务的 `ExecutionProfile` 会携带限值和增益，控制器还有独立硬约束。只降低控制器 YAML 或只降低任务速度，都可能产生配置冲突、命令拒绝或不一致行为。

| 策略项 | 当前值 | 实机调整依据/联动项 |
|---|---|---|
| `geometry.workspace_bounds` | 下界 [-15,-10,0.5]；上界 [15,35,15] m | 换成实验室世界坐标的可飞包络，扣除机体、跟踪和制动余量 |
| `geometry.nominal_spacing` | 2.25 m | 实际中心间距、编队人数和空间容量；同时审查 compact/normal/spacious 倍率 |
| `safety.d_hard` / `d_plan_base` | 1.50 / 1.80 m | 旋翼包络、定位误差、相对运动与制动余量；不能为了装进房间机械缩小 |
| `safety.iapf_enter_base/exit_base` | 1.60 / 1.70 m | 保持硬间距 < 进入距离 < 退出距离，覆盖全部 safety factor |
| `safety.iapf_repulsion_base/margin` | 1.0 / 0.25 | 编译后的任务避障参数与执行硬约束同步 |
| `motion_limits.velocity/acceleration/jerk` | 5 / 5 / 10（SI 单位） | 用实机能稳定跟踪的轨迹上限，并验证总输出与跟踪误差 |
| `timing.minimum_duration` | 0.5 s | 短距离任务也应满足速度、加速度、jerk 约束 |
| `timing.auto_style_factors` | smooth 1.30，normal 1.15，aggressive 1.10 | 首轮只开放已验收风格；显式 T 不经过这些倍率，仍需限值复核 |
| `allocator.sample_hz` | 20 Hz | 采样间隔内相对位移与距离余量，离散检查不等于连续安全证明 |
| `execution_profile.baseline_omega_c/omega_o` | [1.5,1.5,1.75] / [5,5,7.5] | 同步控制器基准带宽 |
| `execution_profile.style_gains` | 0.8 / 1.0 / 1.1 | 各风格必须落在实机验证的带宽包络内 |
| `controller_hard_clamps.*` | 带宽包络、速度 5、加速度 5、jerk 10 等 | 同步 `execution_profile_*` 控制器参数，保持命令校验有效 |
| `iapf_runtime.filter_alpha` | 0.2 | 改频率后等效时间响应变化，应重新验证 |

控制器静态 IAPF 还有 `iapf_violation_distance=1.5`、`iapf_enter_distance=1.7`、`iapf_exit_distance=1.8`、`iapf_repulsion_gain=25`、`iapf_position_gain=0.05`、`iapf_position_limit=0.5`、`iapf_accel_gain=0.3`、`iapf_accel_limit=2.0`，以及 `iapf_escape_mode=id_order`、`iapf_escape_gain=0.05`、`iapf_distance_epsilon=0.1`。逐项记录实机取值，并分别检查无任务启动阶段和任务 Profile 激活后的有效参数；不要把静态 `repulsion_gain=25` 与策略中的 1.0 当作同一个可互换的标定值。

间距设计可用以下式子做初步预算，随后通过实测验证：

```text
所需间距 ≥ 两机旋翼包络半径之和
          + 定位与跟踪误差余量
          + 相对闭合速度 × 总检测/通信/执行延迟
          + 保守制动距离
```

注意垂直方向还有下洗影响，不能仅依据球形中心距离安排上下重叠飞行。规划边界应包含整个轨迹及 IAPF 修正空间；当前 `max_velocity` 声明和各类 Profile 上限不能代替对实际速度、最终合成控制量的运行时监测。

### 6.3 时间与状态新鲜度

| 参数/实现 | 当前情况 | 实机要求 |
|---|---|---|
| `state_snapshot.state_timeout` | 0.02208 s | 测量端到端数据年龄分布后设定 |
| `state_snapshot.snapshot_skew` | 0.022043 s | 测量跨机采样偏差与时钟误差 |
| `state_snapshot.fresh_state_wait_timeout` | 0.010 s | 根据正常更新周期确定等待窗口 |
| `allow_receive_time_fallback` / `require_velocity` | false / true | 保留明确的时间语义和有效速度要求 |
| 控制器 `odomCallback()` | 使用 `this->now()` 给 swarm_state 重新打时间戳 | 改为保留/转换已同步的源时间；另外记录接收时间 |
| 邻机有效性 | 按本机接收时间判断超时 | 同时检查源年龄，避免队列积压伪装新数据 |
| MAVROS 流申请 | 消息 32 申请 100 Hz | 实测频率和最长间隔；申请成功不代表达到该频率 |

ROS1 MAVROS 输入已经包含 `header.stamp`；不能照搬节点里“PX4 boot clock 必须重打时间”的旧注释。先确认 MAVROS 的时间同步与所用 VRPN 时间戳语义，再统一到可比较的 ROS 时间域。飞控估计状态时间也不等于最近一次动捕测量时间，须另外监测动捕源年龄及 EKF 外部融合健康。

多机电脑同步时钟，记录最大时钟误差；`/use_sim_time=false`，不依赖 `/clock`。用无桨连续采样记录 P50/P95/P99、最长间隔、乱序/丢失计数和跨机偏差。阈值按“实测延迟与抖动 + 明确余量”设定，同时受运动反应距离约束；链路太慢时先降低运动能力或改通信，不能无限增加 timeout。

## 7. 配置组织与实机启动入口

建议新增下列文件，**本次仅编写文档，这些文件尚未实现**：

```text
src/ladrc_controller/launch/real_swarm.launch
src/ladrc_controller/config/real_lab.yaml
src/ladrc_controller/config/real_vehicles.yaml
src/ladrc_controller/scripts/mocap_pose_adapter.py
src/lfs_policy/config/lfs_policy.real_lab.yaml
scripts/run_real_lab.sh
scripts/check_real_lab_readiness.py
```

实施时解决以下兼容问题：

1. `sync_paper_controller_config.py` 从 `paper_base.yaml` 和冻结策略生成/核对 `ladrc_params.yaml`，且设置 LADRC 模式；当前构建和启动脚本调用它。保留仿真链路，另建实机生成与一致性检查入口。
2. `loader.py` 的 production 路径仅接受 `paper_current`/`paper_frozen`，`load_paper_policy()` 还限定 paper 用途。因此直接写 `policy_status: real_lab` 会被拒绝。需扩展明确的实机状态/用途校验，并检查 `policy_adapter.py`、调度器及 candidate_dispatch 的加载路径；不要关闭生产校验，也不要将实机参数冒充论文冻结参数。
3. 实机策略使用独立 `configuration_id` 和哈希；运动、间距、控制器硬约束保持一致，记录真实标定状态。
4. JSON 与自然语言调度入口都必须显式加载同一实机策略。现有示例可能超出实验室空间，五机验证脚本还包含冻结 ID/间距等断言，不能原样作为实机验收。
5. 当前运行脚本会 `docker rm -f` 同名容器；实机入口必须防止运行中重复启动导致控制流中断。

集中部署时，一台电脑运行各机 MAVROS/控制器及调度器，测量多机同时通信的负载。分布式部署时，明确单一 ROS Master、各主机可达地址、ROS_IP/ROS_HOSTNAME、动态 TCPROS 端口及时间同步，并监测主机失联。不要让多个进程同时向同一飞机发布控制 setpoint。

串口连接配置稳定设备路径和波特率，Docker 映射指定设备；UDP 连接填写真实本地监听端口和飞控/路由器地址。仅在串口或 FCU 路由确实位于本机时使用相应本地地址，不能把 SITL 的 `127.0.0.1` 默认值直接复制到无线实机配置。

实机启动顺序应实现为：网络/时钟 → roscore → VRPN → MAVROS 与定位适配器 → 检查 EKF/坐标/健康 → 控制器仅观测与预发送 → 人工许可或接管 → 单机就绪 → 允许调度。启动入口不得默认立即解锁全部飞机。

## 8. 分阶段验证与验收记录

先由实施人员填写可接受的位置误差、速度误差、最大数据年龄、最大控制间隔、最小机间距、边界余量和故障反应时间。不同机型/实验室没有通用可直接套用的数值；下面各阶段通过后再进入下一阶段。

| 阶段 | 操作 | 通过条件与留存证据 |
|---|---|---|
| A：代码与配置 | 编译，验证策略和消息，检查没有仿真偏移/错误入口 | 配置一致；增加并通过坐标、时间、启动许可与故障分支的针对性测试 |
| B：无桨身份与坐标 | 逐架移动/旋转，启动定位和仅观测节点 | 身份无串机；轴向、尺度、杆臂、航向和速度正确；节点启动不自动解锁 |
| C：无桨故障注入 | 停 VRPN、制造旧时间戳/重复数据、断开 MAVROS、模拟邻机丢失/重置 | 故障被检测；旧数据不能通过新鲜度；不自动重解锁或恢复任务 |
| D：单机基础飞行 | 完成机型基础调试，先用飞控定位控制验证悬停与接管/降落 | 融合稳定；人工接管可靠；误差与反应时间达标 |
| E：单机项目闭环 | 验证 px4_position，再单独验证 LADRC；小位移、低动态轨迹 | 实际掩码正确；无持续振荡或饱和；观察器及跟踪性能达标 |
| F：双机 | 足够间距悬停、平行运动，随后验证避让及单机退出 | 坐标一致；实际最小距离和邻机失效行为达标 |
| G：目标机数 | 从固定 JSON 简单编队开始，再验证复杂任务和自然语言入口 | 全队满足边界、间距、时序、完成判据；语义风格均单独验收 |

在地面/无桨环境完成可复现的故障注入；需要飞行验证的降级动作由实验负责人按实验室流程逐项安排。首轮任务不要直接使用 README 中中心 `[0,9,3]`、半径 4 m 的仿真示例，也不要直接运行原五机仿真验证任务。

可在节点启动后执行以下只读检查（替换实际机号和刚体名）：

```bash
rosparam get /use_sim_time
rostopic list
rostopic type /vrpn_client_node/<刚体名>/pose
rostopic hz /vrpn_client_node/<刚体名>/pose
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/mavros/local_position/odom
rostopic echo -n 1 /uav1/swarm_state
rostopic echo -n 1 /uav1/status
rostopic hz /uav1/mavros/setpoint_raw/local
rostopic echo -n 1 /uav1/mavros/setpoint_raw/local
rosparam get /uav1/ladrc_position_controller
```

`rostopic hz` 只检查到达频率，不能验证采样时间、EKF 融合和轴向正确。实机验收应增加独立数据年龄/最大间隔统计，避免仅凭 `system_ready=True` 放行。

每次实验保存：实际飞控参数、ROS 参数导出、策略文件和哈希、各软件 commit/版本、身份与外参表、操作者及许可记录、任务 JSON、调度 trace、rosbag 和 PX4 ULog。rosbag 至少涵盖各机 VRPN 原始位姿、适配器定位/健康、MAVROS odom/state、swarm_state/status、execution_command、setpoint_raw/local、startup_event、iapf_debug、control_tracking_debug，以及实际使用的时间同步和 TF 信息。

结束顺序：停止新任务 → 通过已实现的实机流程降落/人工接管 → 确认所有飞机落地并上锁 → 停控制器和通信 → 保存日志。若需退出未完成任务，应使用实施后定义的取消/降落接口，不能假设退出调度器会取消飞行。

## 9. 交付给实验人员的最终清单

- [ ] 第 1 节配置表填写完整，硬件/固件和接口路线明确。
- [ ] VRPN 适配、EKF 融合、每机坐标/杆臂/航向标定完成。
- [ ] 移除邻机 `3.0 * neighbor_id` 仿真假设；自机与邻机共用世界坐标。
- [ ] 源时间、接收时间、动捕源健康及 EKF 状态分别可观测。
- [ ] 人工许可、接管、起飞高度语义、取消、降落和故障退出已实现并测试。
- [ ] 实机策略和控制器参数独立管理且一致；不存在未经验证的仿真默认值回退。
- [ ] 单机至多机逐级验收，实际区域、间距和动态能力符合填写的标准。
- [ ] 发布实际可执行的启动/停止命令、配置文件及每项验收证据。

完成这些工作后，实施人员应将本文中的待填写项替换为现场记录，并另附本实验室的操作版文档；在那之前，本文是开发与配置依据，而不是可直接执行的飞行操作手册。
