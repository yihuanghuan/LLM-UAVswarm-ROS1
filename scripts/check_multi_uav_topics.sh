#!/usr/bin/env bash
set -e

UAV_COUNT="${1:-3}"

if ! [[ "$UAV_COUNT" =~ ^[0-9]+$ ]] || [ "$UAV_COUNT" -lt 1 ] || [ "$UAV_COUNT" -gt 10 ]; then
  echo "用法: $0 [UAV_COUNT]"
  echo "UAV_COUNT 范围: 1-10，默认 3"
  exit 2
fi

echo "==> 检查 $UAV_COUNT 架无人机的 ROS topic"
echo "==> 确保已 source /opt/ros/noetic/setup.bash 和 devel/setup.bash"

missing=0

check_topic() {
  local topic="$1"
  if rostopic list | grep -Fx "$topic" >/dev/null; then
    echo "[OK] $topic"
  else
    echo "[缺失] $topic"
    missing=1
  fi
}

for uid in $(seq 1 "$UAV_COUNT"); do
  echo ""
  echo "---- UAV$uid ----"
  check_topic "/uav${uid}/mavros/state"
  check_topic "/uav${uid}/mavros/local_position/odom"
  check_topic "/uav${uid}/mavros/setpoint_position/local"
  check_topic "/uav${uid}/swarm_command"
  check_topic "/uav${uid}/status"
  check_topic "/uav${uid}/odom"
done

echo ""
echo "==> MAVROS 节点:"
rosnode list | grep '/uav.*/mavros' || true

echo ""
echo "==> 控制节点:"
rosnode list | grep '/uav.*/ladrc_position_controller' || true

if [ "$missing" -ne 0 ]; then
  echo "==> 检查失败：存在缺失 topic"
  exit 1
fi

echo "==> 检查通过"
