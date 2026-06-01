#!/usr/bin/env bash
# Build unitree_api + unitree_go message packages (once per workspace).
set -euo pipefail
set +u
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  ros-humble-rosidl-generator-dds-idl \
  python3-colcon-common-extensions
cd /ws/src/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_api unitree_go --symlink-install
echo "Done. Source: source /ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
