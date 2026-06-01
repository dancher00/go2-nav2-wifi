#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
./run.sh bash -lc '
set -e
install-deps.sh full
mkdir -p /ws/src && cd /ws/src
[[ -d unitree_ros2 ]] || git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git
cd /ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "source /ws/install/setup.bash" >> ~/.bashrc
'
