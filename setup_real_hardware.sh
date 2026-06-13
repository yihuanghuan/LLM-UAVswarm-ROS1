#!/bin/bash
# ============================================================
# 实机测试环境安装脚本
# 在 Docker 容器内运行一次，安装 VRPN 客户端及依赖
#
# 前提: 容器已启动且可访问互联网
# 用法:
#   sudo docker exec -it ros1_test bash
#   bash /ros1_ws/setup_real_hardware.sh
# ============================================================
set -e

echo "========================================"
echo " 实机测试环境安装"
echo "========================================"
echo ""

# 1. 基础工具
echo "[1/4] 安装基础工具..."
apt-get update -o Acquire::Check-Valid-Until=false
apt-get install -y --no-install-recommends python3-pip git
echo "  完成"

# 2. 安装 VRPN C 库
echo "[2/4] 安装 VRPN 库..."
apt-get install -y --no-install-recommends libvrpn-dev vrpn || {
    echo "  从 apt 安装失败，尝试从源码编译..."
    cd /tmp
    git clone https://github.com/vrpn/vrpn.git
    cd vrpn
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j$(nproc)
    make install
    ldconfig
}
echo "  完成"

# 3. 从源码编译 vrpn_client_ros
echo "[3/4] 编译 vrpn_client_ros..."
source /opt/ros/noetic/setup.bash
mkdir -p /tmp/vrpn_ws/src
cd /tmp/vrpn_ws/src
if [ ! -d vrpn_client_ros ]; then
    git clone https://github.com/ros-drivers/vrpn_client_ros.git
fi
cd /tmp/vrpn_ws
catkin_make
cp -r devel/lib/* /opt/ros/noetic/lib/
cp -r devel/share/* /opt/ros/noetic/share/
rm -rf /tmp/vrpn_ws
echo "  完成"

# 4. 验证安装
echo "[4/4] 验证安装..."
source /opt/ros/noetic/setup.bash
if roslaunch vrpn_client_ros sample.launch --help > /dev/null 2>&1; then
    echo "  vrpn_client_ros 安装成功!"
else
    echo "  警告: vrpn_client_ros roslaunch 检查失败，请手动验证"
fi

echo ""
echo "========================================"
echo " 实机测试环境安装完成!"
echo ""
echo " 启动示例:"
echo "   roslaunch ladrc_controller real_hardware.launch uav_id:=1"
echo "========================================"
