#!/usr/bin/env bash
set -e

UAV_COUNT="${1:-3}"
CONTAINER_NAME="${ROS1_MULTI_CONTAINER:-ros1_multi_uav}"
IMAGE="${ROS1_MAVROS_IMAGE:-ros1-mavros:latest}"
DOCKER_CMD="${DOCKER_CMD:-docker}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! [[ "$UAV_COUNT" =~ ^[0-9]+$ ]] || [ "$UAV_COUNT" -lt 1 ] || [ "$UAV_COUNT" -gt 10 ]; then
  echo "用法: $0 [UAV_COUNT]"
  echo "UAV_COUNT 范围: 1-10，默认 3"
  exit 2
fi

echo "==> 前提：宿主机已启动 PX4 多机 SITL"
echo "    示例: cd ~/PX4-Autopilot && Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n $UAV_COUNT"
echo "==> 工作区: $WS_DIR"
echo "==> Docker 镜像: $IMAGE"
echo "==> 容器名: $CONTAINER_NAME"

disable_args=()
for uid in $(seq 1 10); do
  if [ "$uid" -gt "$UAV_COUNT" ]; then
    disable_args+=("enable_uav${uid}:=false")
  fi
done

$DOCKER_CMD rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

$DOCKER_CMD run -d --name "$CONTAINER_NAME" \
  --network host \
  -v "$WS_DIR:/ros1_ws" \
  "$IMAGE" \
  bash -lc "
    source /opt/ros/noetic/setup.bash
    cd /ros1_ws
    catkin_make
    source /ros1_ws/devel/setup.bash
    roscore >/tmp/roscore.log 2>&1 &
    sleep 3
    roslaunch ladrc_controller swarm.launch ${disable_args[*]} >/tmp/swarm.launch.log 2>&1 &
    tail -f /tmp/swarm.launch.log
  "

echo "==> 多机 ROS1 容器已启动"
echo "查看日志: $DOCKER_CMD logs -f $CONTAINER_NAME"
echo "进入容器: $DOCKER_CMD exec -it $CONTAINER_NAME bash"
echo "检查 topic:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source /ros1_ws/devel/setup.bash"
echo "  /ros1_ws/scripts/check_multi_uav_topics.sh $UAV_COUNT"
