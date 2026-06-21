# ROS1 Noetic + MAVROS 镜像 (基础镜像，不含 VRPN)
FROM ros:noetic-ros-base

# 安装 MAVROS 和编译依赖
RUN apt-get update -o Acquire::Check-Valid-Until=false && \
    apt-get install -y --no-install-recommends \
    ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs \
    libeigen3-dev \
    python3-pip python3-numpy python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# 安装 LLM 调度层 Python 依赖。
# ROS Noetic 基于 Ubuntu 20.04 / Python 3.8，固定版本避免最新 openai 依赖不可用 wheel。
RUN pip3 install --no-cache-dir openai==1.30.5 httpx==0.27.2

# 安装 GeographicLib (MAVROS 依赖)
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# 设置 ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /ros1_ws/devel/setup.bash" >> /root/.bashrc

# 工作目录
WORKDIR /ros1_ws
