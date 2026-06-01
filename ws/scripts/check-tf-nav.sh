#!/usr/bin/env bash
# TF chain for nav (run while nav-to-point.sh is up).
set -euo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

echo "=== TF map -> base_link (need one chain, not 'unconnected trees') ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | tail -8 || echo "FAIL — no map->odom->base_link"

echo ""
echo "=== /utlidar/robot_odom (robot relay must send) ==="
timeout 3 ros2 topic hz /utlidar/robot_odom 2>&1 | tail -3 || echo "NO robot_odom — start robot-relay-wifi.sh"

echo ""
echo "=== nodes ==="
ros2 node list 2>/dev/null | grep -E 'map_odom|odom_tf|slam_toolbox' || true
