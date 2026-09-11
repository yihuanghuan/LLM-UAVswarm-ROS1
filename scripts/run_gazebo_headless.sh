#!/usr/bin/env bash
set -euo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null; then
  echo '已有 PX4/Gazebo 正在运行；请先结束该仿真，避免实例或端口冲突。' >&2
  exit 2
fi
# Preserve upstream model/port setup; replace only GUI lifetime with headless wait.
runner=$(mktemp /tmp/ros1-paper-sitl-XXXXXX.sh)
trap 'rm -f "$runner"' EXIT
python3 - "$PX4_DIR" "$runner" <<'PY'
from pathlib import Path
import sys,shlex
root=Path(sys.argv[1]).resolve()
p=root/'Tools/simulation/gazebo-classic/sitl_multiple_run.sh'
s=p.read_text()
s=s.replace('src_path="$SCRIPT_DIR/../../.."', 'src_path='+shlex.quote(str(root)))
s=s.replace('\ngzclient\n','\nwait\n')
Path(sys.argv[2]).write_text(s)
PY
unset ROS_VERSION
bash "$runner" -n "${1:-3}" -m iris -w empty
