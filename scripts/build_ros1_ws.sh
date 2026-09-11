#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE="${ROS1_MAVROS_IMAGE:-ros1-paper:latest}"
DOCKER_CMD="${DOCKER_CMD:-docker}"

echo "==> 工作区: $WS_DIR"
echo "==> Docker 镜像: $IMAGE"
echo "==> 开始 catkin_make"

$DOCKER_CMD run --rm --network host \
  -v "$WS_DIR:/ros1_ws" \
  "$IMAGE" \
  bash -lc "source /opt/ros/noetic/setup.bash && cd /ros1_ws && python3 scripts/sync_paper_controller_config.py --check && catkin_make"

echo "==> 编译完成"
