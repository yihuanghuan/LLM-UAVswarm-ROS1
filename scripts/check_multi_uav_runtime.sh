#!/usr/bin/env bash
set -e

UAV_COUNT="${1:-8}"
TIMEOUT_SEC="${2:-8}"

if ! [[ "$UAV_COUNT" =~ ^[0-9]+$ ]] || [ "$UAV_COUNT" -lt 1 ] || [ "$UAV_COUNT" -gt 10 ]; then
  echo "用法: $0 [UAV_COUNT] [TIMEOUT_SEC]"
  echo "UAV_COUNT 范围: 1-10，默认 8"
  exit 2
fi

if ! [[ "$TIMEOUT_SEC" =~ ^[0-9]+$ ]] || [ "$TIMEOUT_SEC" -lt 1 ]; then
  echo "用法: $0 [UAV_COUNT] [TIMEOUT_SEC]"
  echo "TIMEOUT_SEC 必须为正整数，默认 8"
  exit 2
fi

echo "==> 检查 $UAV_COUNT 架无人机运行时数据，单 topic 超时 ${TIMEOUT_SEC}s"
echo "==> 确保已 source /opt/ros/noetic/setup.bash 和 /ros1_ws/devel/setup.bash"

missing=0

for uid in $(seq 1 "$UAV_COUNT"); do
  echo ""
  echo "---- UAV$uid ----"

  if timeout "$TIMEOUT_SEC" rostopic echo "/uav${uid}/mavros/local_position/odom" -n 1 >/tmp/uav_${uid}_mavros_odom.txt; then
    echo "[OK] /uav${uid}/mavros/local_position/odom 有数据"
  else
    echo "[失败] /uav${uid}/mavros/local_position/odom 无数据"
    missing=1
  fi

  if timeout "$TIMEOUT_SEC" rostopic echo "/uav${uid}/swarm_state" -n 1 >/tmp/uav_${uid}_public_odom.txt; then
    echo "[OK] /uav${uid}/swarm_state 有数据"
    grep -E 'frame_id:|child_frame_id:' /tmp/uav_${uid}_public_odom.txt || true
  else
    echo "[失败] /uav${uid}/swarm_state 无数据"
    missing=1
  fi

  if timeout "$TIMEOUT_SEC" rostopic echo "/uav${uid}/mavros/state" -n 1 >/tmp/uav_${uid}_state.txt; then
    echo "[OK] /uav${uid}/mavros/state 有数据"
    grep -E 'connected:|armed:|mode:' /tmp/uav_${uid}_state.txt || true
    if ! grep -q '^connected: True' /tmp/uav_${uid}_state.txt; then
      echo "[失败] /uav${uid}/mavros/state 未连接飞控"
      missing=1
    fi
  else
    echo "[失败] /uav${uid}/mavros/state 无数据"
    missing=1
  fi
done

if [ "$missing" -ne 0 ]; then
  echo ""
  echo "==> 运行时检查失败"
  exit 1
fi

echo ""
echo "==> 运行时检查通过"
