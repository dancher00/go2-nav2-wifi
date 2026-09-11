#!/usr/bin/env bash
# Run inside the project ROS Humble container.
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /ws
colcon build --base-paths /ws/src/go2_nav2/rviz_plugins/go2_rviz_controls \
  --packages-select go2_rviz_controls --symlink-install --cmake-args -DBUILD_TESTING=ON
