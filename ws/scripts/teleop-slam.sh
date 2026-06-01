#!/usr/bin/env bash
# Teleop for SLAM over Wi-Fi (starts TCP cmd_vel client) or Ethernet.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/go2-env.sh ] && source /ws/scripts/go2-env.sh

TCP_PID=""
cleanup() {
  [[ -n "$TCP_PID" ]] && kill "$TCP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${GO2_NET:-wifi}" == "wifi" ]]; then
  ROBOT="${GO2_ROBOT_IP:-192.168.1.58}"
  PORT="${GO2_CMD_TCP_PORT:-17999}"
  pkill -f 'go2_cmd_vel_tcp.py.*--role client' 2>/dev/null || true
  sleep 0.2
  python3 /ws/scripts/go2_cmd_vel_tcp.py --role client --host "$ROBOT" --port "$PORT" &
  TCP_PID=$!
  sleep 1
fi

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel
