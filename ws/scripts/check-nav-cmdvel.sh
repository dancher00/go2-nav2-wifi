#!/usr/bin/env bash
# While nav-to-point is running and robot should move — diagnose /cmd_vel chain.
set -euo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/go2-env.sh ] && source /ws/scripts/go2-env.sh

echo "=== /cmd_vel publishers (expect controller_server + maybe go2_goal_pose_nav) ==="
ros2 topic info /cmd_vel -v 2>/dev/null | grep -E 'Publisher|Node name|Reliability' || true

echo ""
echo "=== /cmd_vel rate 5s (should be ~20 Hz while following path) ==="
timeout 5 ros2 topic hz /cmd_vel 2>&1 | tail -4 || echo "NO /cmd_vel traffic"

echo ""
echo "=== one /cmd_vel sample ==="
timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "(no message)"

echo ""
echo "=== cmd_vel TCP client on laptop ==="
pgrep -af 'go2_cmd_vel_tcp.py.*client' || echo "MISSING — nav-to-point.sh should start it"

echo ""
echo "On robot (SSH): bash ~/robot-check-cmdvel.sh"
echo "If laptop hz>0 but robot still: relay/sport_bridge on robot; RC sticks off."
