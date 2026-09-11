# 当前版本验证记录

## 清理范围

本项目仅提供论文 Candidate Mission → Execution Profile → LADRC/IAPF 的 ROS1 仿真链路。
已移除旧调度器/parser/weighted-sum allocator、旧 schema/policy、`UAVSwarmCommand` 消息及订阅、
旧位置 Point 话题、旧参数别名和控制器兼容回调、旧实机/VRPN 入口、旧实验脚本与说明。
当前消息保留 8 种，TrajectoryMetrics 中仅服务旧命令的 safety_factor 字段已删除；
当前任务的安全语义以冻结 policy、Execution Profile 和 ResolutionTrace 为准。

旧版本兼容功能对应的测试已删除；新算法测试和旧输入拒绝测试继续保留。
对外示例位于 `examples/five_uav_mission.json`，不再依赖历史验证目录。
唯一 Dockerfile 从 ROS Noetic 基础镜像构建，不依赖旧项目镜像。

核心数值算法文件的 SHA-256 仍与 [来源清单](../migration_source.json) 一致。
冻结 policy 的 SHA-256 仍为 `6b47d27f4253d7311e79ea51f6dd1cf0d0182e6df24374a94abae0aa6a135858`。
冻结提示词/策略中的历史来源名称保留用于溯源，不对应可运行的旧算法入口。

## 本轮检查

- 删除旧 build/devel 产物后，catkin 完整重新编译成功。
- Python：161 passed，直接运行两个包的 pytest，无需排除旧校准工具测试。
- C++：76 tests，0 errors、0 failures、0 skipped。
- 运行时检查：旧 Python 模块与 UAVSwarmCommand 不可导入，旧命令/Point 状态话题不存在。
- 五机实际 Gazebo/PX4 冷启动，通过自动就绪检测。
- 五机 Circle → 同步并行 Triangle/Line → 五机 Line，15 条命令完整记录，三个运动风格覆盖。
- 45 秒录制共 20529 条状态样本；最小采样机间距 **2.132814 m**。
- 最大最终位置误差 **0.012664 m**；五机均 ready/stable/armed/OFFBOARD，无控制器故障。
- 五机 MAVROS 加速度 setpoint 掩码均为 2111。

上述为工程回归，不是全部论文正式实验；只验证当前正常 Candidate 任务链路。
独立强制对穿诊断会绕过分配器，历史记录显示不能保证 1.50 m 安全间距，不计作安全通过证据。

## 证据

- [构建日志](../validation/current_only/build.log)
- [Python 测试](../validation/current_only/python_tests.log)
- [C++ 测试](../validation/current_only/cpp_tests.log)
- [当前接口检查](../validation/current_only/interface_check.log)
- [Gazebo 任务判定](../validation/current_only/simulation_validation.log)
- [任务运行日志](../validation/current_only/flight/mission.log)
- [飞行指标](../validation/current_only/flight/flight_report.json)
- [ResolutionTrace](../validation/current_only/flight/candidate_resolution_trace.jsonl)

## 本机清理审计

删除清单见 [cleanup_manifest.json](cleanup_manifest.json)。迁移前备份已移出本项目，
存放于 `/home/yihuang/learning/ros1_migration_backup_20260911`，不参与构建或运行。
原始旧 README、实机说明和旧入口不再作为本项目的操作文档。使用方式以根目录 README 为准。
