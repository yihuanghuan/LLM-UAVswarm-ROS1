#!/usr/bin/env bash
# Execute inside a sourced ROS1 container with five simulated vehicles ready.
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$WS_DIR/validation/run_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
python3 "$WS_DIR/scripts/wait_for_swarm_ready.py" --ids 1,2,3,4,5 | tee "$OUT/readiness.log"
python3 "$WS_DIR/scripts/record_paper_validation.py" --ids 1,2,3,4,5 --duration 45 --output "$OUT" > "$OUT/recorder.log" 2>&1 &
recorder=$!
trap 'kill "$recorder" 2>/dev/null || true' EXIT
sleep 1
PYTHONUNBUFFERED=1 ROS_HOME="$OUT" rosrun location_allocate candidate_dispatch \
  --mission-json "$WS_DIR/examples/five_uav_mission.json" \
  --uav-ids 1,2,3,4,5 --policy "$WS_DIR/src/lfs_policy/config/lfs_policy.paper_current.yaml" > "$OUT/mission.log" 2>&1
wait "$recorder"
python3 "$WS_DIR/scripts/assert_paper_flight_report.py" "$OUT/flight_report.json" | tee "$OUT/assertions.log"
trap - EXIT
