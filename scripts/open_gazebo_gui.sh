#!/usr/bin/env bash
set -euo pipefail
if ! pgrep -x gzserver >/dev/null; then
  echo '请先在另一个终端启动 ./scripts/run_gazebo_headless.sh 5，并保持其运行。' >&2
  exit 2
fi
if [ -z "${DISPLAY:-}" ]; then
  echo '请在宿主机图形桌面的终端中运行此脚本（DISPLAY 未设置）。' >&2
  exit 2
fi
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
# Upstream setup script reads optional environment variables.
set +u
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
set -u
exec gzclient --verbose
