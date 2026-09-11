#!/usr/bin/env bash
set -e

UAV_COUNT="${1:-8}"
CONTAINER_NAME="${ROS1_MULTI_CONTAINER:-ros1_multi_uav}"
DOCKER_CMD="${DOCKER_CMD:-docker}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_FILE="${MINIMAX_ENV_FILE:-$WS_DIR/.env.minimax}"

if [ -f "$ENV_FILE" ]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi

if ! [[ "$UAV_COUNT" =~ ^[0-9]+$ ]] || [ "$UAV_COUNT" -lt 1 ] || [ "$UAV_COUNT" -gt 10 ]; then
  echo "用法: $0 [UAV_COUNT]"
  echo "UAV_COUNT 范围: 1-10，默认 8"
  exit 2
fi

if [ -z "${LLM_API_KEY:-${MINIMAX_API_KEY:-}}" ]; then
  echo "错误: MINIMAX_API_KEY 未设置"
  echo "请先创建 $ENV_FILE，或执行: export MINIMAX_API_KEY='your-api-key'"
  exit 2
fi

if ! $DOCKER_CMD ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "错误: 容器 $CONTAINER_NAME 未运行"
  echo "请先启动多机 ROS1 容器: ./scripts/run_multi_uav_sim.sh $UAV_COUNT"
  exit 2
fi

tty_args=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  tty_args=(-it)
fi

env_args=(-e "LLM_API_KEY=${LLM_API_KEY:-${MINIMAX_API_KEY:-}}")
if [ -n "${LLM_BASE_URL:-}" ]; then
  env_args+=(-e "LLM_BASE_URL=${LLM_BASE_URL}")
fi
if [ -n "${LLM_MODEL_NAME:-}" ]; then
  env_args+=(-e "LLM_MODEL_NAME=${LLM_MODEL_NAME}")
fi

echo "==> 启动 ROS1 LLM 调度层"
echo "==> 容器: $CONTAINER_NAME"
echo "==> UAV_COUNT: $UAV_COUNT"
echo "==> 退出调度终端请输入: q"

$DOCKER_CMD exec "${tty_args[@]}" "${env_args[@]}" "$CONTAINER_NAME" bash -lc "
  source /opt/ros/noetic/setup.bash
  source /ros1_ws/devel/setup.bash
  rosrun location_allocate location_allocate_node _uav_ids:=[$(seq -s, 1 "$UAV_COUNT")] _lfs_runtime_mode:=candidate_v2
"
