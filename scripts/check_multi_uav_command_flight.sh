#!/usr/bin/env bash
set -e

UAV_COUNT="${1:-8}"
TARGET_Z="${2:-1.5}"
DURATION_SEC="${3:-8.0}"
TIMEOUT_SEC="${4:-120}"

if ! [[ "$UAV_COUNT" =~ ^[0-9]+$ ]] || [ "$UAV_COUNT" -lt 1 ] || [ "$UAV_COUNT" -gt 10 ]; then
  echo "用法: $0 [UAV_COUNT] [TARGET_Z] [DURATION_SEC] [TIMEOUT_SEC]"
  echo "UAV_COUNT 范围: 1-10，默认 8"
  exit 2
fi

if ! [[ "$TIMEOUT_SEC" =~ ^[0-9]+$ ]] || [ "$TIMEOUT_SEC" -lt 1 ]; then
  echo "用法: $0 [UAV_COUNT] [TARGET_Z] [DURATION_SEC] [TIMEOUT_SEC]"
  echo "TIMEOUT_SEC 必须为正整数，默认 120"
  exit 2
fi

echo "==> 向 $UAV_COUNT 架无人机发布 swarm_command"
echo "==> 目标: X=0.0, Y=3.0*N, Z=${TARGET_Z}, duration=${DURATION_SEC}s"
echo "==> 等待 /uavN/status.is_hover_stable=True，超时 ${TIMEOUT_SEC}s"

for uid in $(seq 1 "$UAV_COUNT"); do
  target_y="$(awk "BEGIN {printf \"%.1f\", ${uid} * 3.0}")"
  echo "发布 UAV${uid}: [0.0, ${target_y}, ${TARGET_Z}]"
  timeout 10 rostopic pub -1 "/uav${uid}/swarm_command" uav_swarm_interfaces/UAVSwarmCommand \
    "{header: {stamp: now, frame_id: 'world'}, uav_id: ${uid}, target_pos: {x: 0.0, y: ${target_y}, z: ${TARGET_Z}}, duration: ${DURATION_SEC}, motion_style: 'smooth', safety_factor: 0.0}"
done

start_time="$(date +%s)"
while true; do
  stable_count=0

  for uid in $(seq 1 "$UAV_COUNT"); do
    if timeout 3 rostopic echo "/uav${uid}/status" -n 1 >"/tmp/uav_${uid}_status.txt" 2>/dev/null &&
       grep -q '^is_hover_stable: True' "/tmp/uav_${uid}_status.txt"; then
      stable_count=$((stable_count + 1))
    fi
  done

  echo "稳定悬停进度: ${stable_count}/${UAV_COUNT}"
  if [ "$stable_count" -eq "$UAV_COUNT" ]; then
    echo "==> 指令飞行检查通过"
    exit 0
  fi

  now="$(date +%s)"
  if [ $((now - start_time)) -ge "$TIMEOUT_SEC" ]; then
    echo "==> 指令飞行检查失败：等待稳定悬停超时"
    for uid in $(seq 1 "$UAV_COUNT"); do
      echo ""
      echo "---- UAV${uid} 最新状态 ----"
      timeout 3 rostopic echo "/uav${uid}/status" -n 1 || true
      timeout 3 rostopic echo "/uav${uid}/odom" -n 1 || true
    done
    exit 1
  fi

  sleep 5
done
