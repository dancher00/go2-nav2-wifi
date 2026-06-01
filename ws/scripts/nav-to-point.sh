#!/usr/bin/env bash
# Navigate on a saved map (localization + Nav2).
# Wi-Fi: starts cmd_vel TCP client in background (relay must run on robot).
# Usage: /ws/scripts/nav-to-point.sh [map.yaml]
# Odom: GO2_ODOM_SOURCE=utlidar|sport (from go2-env.sh / docker .env)
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/go2-env.sh ] && source /ws/scripts/go2-env.sh

MAP="${1:-/ws/maps/my_room.yaml}"
ODOM="${GO2_ODOM_SOURCE:-utlidar}"
case "$ODOM" in
  utlidar|sport) ;;
  *)
    echo "ERROR: GO2_ODOM_SOURCE must be utlidar or sport (got: $ODOM)"
    exit 1
    ;;
esac
TCP_PID=""
cleanup() {
  [[ -n "$TCP_PID" ]] && kill "$TCP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -f "$MAP" ]]; then
  echo "ERROR: map not found: $MAP"
  exit 1
fi

# Kill stale Nav2 / SLAM loc from previous run (avoids lifecycle + duplicate TF).
if pgrep -f 'nav2_slam_loc.launch.py|lifecycle_manager_navigation|/map_server' >/dev/null 2>&1; then
  echo "Stopping previous nav2_slam_loc / map_server..."
  pkill -f 'nav2_slam_loc.launch.py' 2>/dev/null || true
  pkill -f 'lifecycle_manager_navigation' 2>/dev/null || true
  pkill -f '/map_server' 2>/dev/null || true
  pkill -f '/planner_server' 2>/dev/null || true
  pkill -f '/controller_server' 2>/dev/null || true
  pkill -f 'go2_cmd_vel_tcp.py.*client' 2>/dev/null || true
  sleep 2
  ros2 daemon stop 2>/dev/null || true
  sleep 1
  ros2 daemon start 2>/dev/null || true
fi

if [[ "${GO2_NET:-wifi}" == "wifi" ]]; then
  ROBOT="${GO2_ROBOT_IP:-192.168.1.58}"
  PORT="${GO2_CMD_TCP_PORT:-17999}"
  pkill -f 'go2_cmd_vel_tcp.py.*client' 2>/dev/null || true
  sleep 0.3
  python3 /ws/scripts/go2_cmd_vel_tcp.py --role client --host "$ROBOT" --port "$PORT" &
  TCP_PID=$!
  sleep 2
  if ! kill -0 "$TCP_PID" 2>/dev/null; then
    echo "ERROR: cmd_vel TCP client died — copy go2_cmd_vel_tcp.py to robot and restart relay"
    exit 1
  fi
fi

exec ros2 launch go2_nav2 nav2_slam_loc.launch.py "map:=${MAP}" "odom_source:=${ODOM}"
