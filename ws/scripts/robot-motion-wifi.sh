#!/usr/bin/env bash
# Owned motion-only bridge; no sensor relay. Robot must already be standing.
set -eo pipefail
motion_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$motion_dir/robot-source-unitree-ros.sh"
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI
export GO2_MAX_LINEAR=1.20 GO2_MAX_ANGULAR=1.50 GO2_CMD_TIMEOUT=0.5
export GO2_CMD_VEL_HZ=20 GO2_SPORT_RATE_HZ=20
exec 9>/tmp/go2-nav-motion.lock
flock -n 9 || { echo 'Motion bridge already running'; exit 1; }
motion_children=()
cleanup() {
  trap '' INT TERM HUP
  for child in "${motion_children[@]}"; do kill -TERM "$child" 2>/dev/null || true; done
  for child in "${motion_children[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
python3 "$motion_dir/go2_cmd_vel_tcp.py" --role server --bind 0.0.0.0 --port 17999 &
motion_children+=("$!")
# Existing bridge releases RC control and selects velocity walk. No goal is sent.
python3 "$motion_dir/robot_sport_bridge.py" &
motion_children+=("$!")
wait -n "${motion_children[@]}"
