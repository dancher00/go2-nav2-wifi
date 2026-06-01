#!/usr/bin/env bash
# Quick SLAM pipeline check (run inside docker with setup-robot-eth.sh).
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/scripts/setup-robot-eth.sh ] && source /ws/scripts/setup-robot-eth.sh

echo "=== Nodes (need slam_toolbox, go2_odom_tf, go2_cloud_stamp_sync, pointcloud_to_laserscan) ==="
ros2 node list 2>/dev/null | grep -iE 'slam|odom|pointcloud|stamp|sport' || echo "(none)"

echo ""
echo "=== Topics ==="
ros2 topic list 2>/dev/null | grep -E '^/scan|^/map|^/sport|^/utlidar/cloud' || true

echo ""
echo "=== Rates (wait 5s) ==="
timeout 5 ros2 topic hz /scan 2>/dev/null | tail -2 || echo "/scan: NO DATA"
timeout 5 ros2 topic hz /map 2>/dev/null | tail -2 || echo "/map: NO DATA (move robot or start slam_mapping)"

echo ""
echo "=== TF map -> base_link ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -8 || echo "map frame missing — SLAM not started or no scans yet"

echo ""
if ! ros2 node list 2>/dev/null | grep -q slam_toolbox; then
  echo "FIX: start terminal 2: ros2 launch go2_nav2 slam_mapping.launch.py"
fi
