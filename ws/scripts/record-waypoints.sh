#!/usr/bin/env bash
# Record patrol waypoints from RViz 2D Goal Pose → YAML file.
# Usage: /ws/scripts/record-waypoints.sh [output.yaml]
set -euo pipefail
set +u
OUT="${1:-/ws/maps/patrol.yaml}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi
exec ros2 run go2_nav2 go2_waypoint_recorder --ros-args -p "output:=${OUT}"
