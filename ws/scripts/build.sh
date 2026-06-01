#!/usr/bin/env bash
set -euo pipefail
set +u
cd /ws
source /opt/ros/humble/setup.bash
if [[ -f src/unitree_ros2/cyclonedds_ws/install/setup.bash ]]; then
  source src/unitree_ros2/cyclonedds_ws/install/setup.bash
else
  echo "Run /ws/scripts/build-unitree-msgs.sh first"
  exit 1
fi
colcon build --packages-select go2_description go2_nav2 --symlink-install
