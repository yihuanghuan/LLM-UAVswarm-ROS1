#!/bin/bash
# ============================================================
# ROS1 + MAVROS 单机 SITL 仿真测试脚本
#
# 用法:
#   终端 1: PX4 SITL + Gazebo
#     cd ~/PX4-Autopilot
#     make px4_sitl gazebo-classic
#
#   终端 2: 本脚本 (ROS1 MAVROS + 控制器)
#     bash ~/ros1_ws/test_single_uav.sh
#
#   终端 3: 发送测试指令
#     docker exec -it ros1_test bash
#     source /opt/ros/noetic/setup.bash
#     source /ros1_ws/devel/setup.bash
#     rostopic pub -1 /uav1/swarm_command ...
# ============================================================

set -e

echo "========================================"
echo " ROS1 + MAVROS 单机 SITL 仿真测试"
echo "========================================"
echo ""
echo "前提: 确保终端 1 已启动 PX4 SITL + Gazebo"
echo "      cd ~/PX4-Autopilot && make px4_sitl gazebo-classic"
echo ""

WS=/home/yihuang/learning/ros1_ws

# 1. 编译 (如果还没编译)
echo "[1/3] 编译 ROS1 代码..."
sudo docker run --rm \
    -v $WS:/ros1_ws \
    ros1-mavros:latest \
    bash -c "source /opt/ros/noetic/setup.bash && cd /ros1_ws && catkin_make 2>&1 | tail -5"

echo ""
echo "[2/3] 启动 ROS1 MAVROS + 控制器 (后台运行)..."
echo "      容器名: ros1_test"
echo ""

# 2. 启动容器 (后台运行)
# --network host: 共享宿主机网络，MAVROS 直接连 PX4 UDP 端口
sudo docker rm -f ros1_test 2>/dev/null || true
sudo docker run -d --name ros1_test \
    --network host \
    -v $WS:/ros1_ws \
    ros1-mavros:latest \
    bash -c "
        source /opt/ros/noetic/setup.bash
        source /ros1_ws/devel/setup.bash

        echo '=== 启动 ROS Master ==='
        roscore &
        sleep 2

        echo '=== 启动 MAVROS (UAV1) ==='
        ROS_NAMESPACE=/uav1 rosrun mavros mavros_node \
            _fcu_url:=udp://:14540@127.0.0.1:14580 \
            _target_system_id:=1 \
            > /tmp/mavros.log 2>&1 &
        sleep 5

        echo '=== 启动 LADRC 控制器 ==='
        ROS_NAMESPACE=/uav1 rosrun ladrc_controller ladrc_position_controller_node \
            _enu_offset_y:=0.0 \
            > /tmp/controller.log 2>&1 &
        sleep 2

        echo '=== 系统就绪 ==='
        echo 'MAVROS 日志: /tmp/mavros.log'
        echo '控制器日志:  /tmp/controller.log'
        echo ''
        echo '在另一个终端执行:'
        echo '  sudo docker exec -it ros1_test bash'
        echo '  source /opt/ros/noetic/setup.bash'
        echo '  source /ros1_ws/devel/setup.bash'
        echo '  rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand ...'

        # 保持容器运行
        tail -f /tmp/controller.log
    "

echo ""
echo "[3/3] 查看日志 (等待无人机起飞)..."
echo ""
sleep 3
echo "--- MAVROS 状态 ---"
sudo docker exec ros1_test bash -c "
    source /opt/ros/noetic/setup.bash 2>/dev/null
    rostopic list 2>/dev/null | grep -E 'uav1|mavros' | head -20
" 2>&1 || echo "(等待 MAVROS 连接 PX4...)"

echo ""
echo "========================================"
echo " 容器已启动!"
echo ""
echo " 查看控制器日志:"
echo "   sudo docker logs -f ros1_test"
echo ""
echo " 进入容器发送指令:"
echo "   sudo docker exec -it ros1_test bash"
echo "   source /opt/ros/noetic/setup.bash"
echo "   source /ros1_ws/devel/setup.bash"
echo ""
echo " 发送飞行指令示例:"
echo "   rostopic pub -1 /uav1/swarm_command uav_swarm_interfaces/UAVSwarmCommand \\"
echo "     '{header: {stamp: now, frame_id: \"world\"}, uav_id: 1, \\"
echo "       target_pos: {x: 3.0, y: 0.0, z: 3.0}, duration: 5.0, \\"
echo "       motion_style: \"normal\", safety_factor: 0.0}'"
echo ""
echo " 停止容器:"
echo "   sudo docker rm -f ros1_test"
echo "========================================"
