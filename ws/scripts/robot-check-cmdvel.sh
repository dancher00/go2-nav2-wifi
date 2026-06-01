#!/usr/bin/env bash
# Run ON ROBOT while teleop runs on laptop. Checks cmd_vel relay end-to-end.
set -euo pipefail
set +u
source ~/robot-source-unitree-ros.sh 2>/dev/null || source /home/unitree/robot-source-unitree-ros.sh

echo "=== /cmd_vel on robot (internal DDS) ==="
echo "Start on laptop: /ws/scripts/start-cmdvel-tcp-client.sh (or teleop-slam.sh)"
echo "Press 'i' in teleop for 5s..."
timeout 5 ros2 topic hz /cmd_vel 2>&1 | tail -3 || echo "NO /cmd_vel — start TCP client on laptop + teleop"

echo ""
echo "=== cmd_vel TCP server (port ${GO2_CMD_TCP_PORT:-17999}) ==="
pgrep -af 'go2_cmd_vel_tcp.py.*server' || echo "(not running — update robot-relay-wifi.sh + go2_cmd_vel_tcp.py)"

echo ""
echo "=== sport_bridge process ==="
pgrep -af sport_bridge || echo "(none)"

echo ""
echo "=== /api/sport/request publishers ==="
ros2 topic info /api/sport/request -v 2>/dev/null | grep -E 'Publisher|Node name' | head -6 || true

echo ""
echo "If hz=0: restart relay on robot + teleop-slam.sh on laptop."
echo "If hz>0 but robot jerks: update robot_sport_bridge.py (FreeWalk + SwitchJoystick)."
