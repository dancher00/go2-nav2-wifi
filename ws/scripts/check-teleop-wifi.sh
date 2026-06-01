#!/usr/bin/env bash
# Teleop / sport_bridge conflict check (run in Docker after go2-env.sh).
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/go2-env.sh ] && source /ws/scripts/go2-env.sh

echo "=== sport_bridge on LAPTOP (Wi-Fi: must be ZERO) ==="
N=$(ros2 node list 2>/dev/null | grep -c '/sport_bridge' || true)
if [[ "$N" -eq 0 ]]; then
  echo "OK: no sport_bridge on laptop"
elif [[ "$N" -eq 1 ]]; then
  echo "WARN: sport_bridge on laptop — stop it for Wi-Fi teleop:"
  echo "  pkill -f 'go2_nav2.*sport_bridge' || pkill -f sport_bridge"
else
  echo "FAIL: $N sport_bridge nodes on laptop (duplicate)"
fi

echo ""
echo "=== /cmd_vel publishers (laptop) ==="
ros2 topic info /cmd_vel -v 2>/dev/null | grep -E 'Publisher|Node name' || echo "(no /cmd_vel)"

echo ""
echo "=== Press key in teleop, then /cmd_vel rate (5s) ==="
timeout 5 ros2 topic hz /cmd_vel 2>&1 | tail -3 || echo "NO /cmd_vel — start teleop-slam.sh"

echo ""
echo "=== Wi-Fi cmd_vel TCP client (need one) ==="
pgrep -af 'go2_cmd_vel_tcp.py.*client' || echo "MISSING — run /ws/scripts/teleop-slam.sh or start-cmdvel-tcp-client.sh"
