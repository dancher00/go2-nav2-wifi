#!/bin/bash
# Unitree Go2 DDS — use after ./setup-unitree-dds.sh
set +u
source /opt/ros/humble/setup.bash
if [[ -f /ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash ]]; then
  source /ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
else
  echo "ERROR: run on host: cd ~/go2/docker && ./setup-unitree-dds.sh"
  return 1 2>/dev/null || exit 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Robot uses empty ROS_DOMAIN_ID = domain 0. Do NOT use 37 unless robot was rebooted with it.
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds/go2.xml}"
echo "Unitree DDS ready: DOMAIN=$ROS_DOMAIN_ID (use 0, same as robot)"
