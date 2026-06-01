#!/usr/bin/env bash
# Forward /cmd_vel to robot over TCP (Wi-Fi). Run before teleop or Nav2.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/go2-env.sh ] && source /ws/scripts/go2-env.sh

ROBOT="${GO2_ROBOT_IP:-192.168.1.58}"
PORT="${GO2_CMD_TCP_PORT:-17999}"
SCRIPT="/ws/scripts/go2_cmd_vel_tcp.py"

exec python3 "$SCRIPT" --role client --host "$ROBOT" --port "$PORT"
