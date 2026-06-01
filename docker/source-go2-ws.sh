#!/bin/bash
# Full workspace: Humble + unitree msgs + go2_nav2
set +u
source /opt/ros/humble/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
elif [[ -f /ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash ]]; then
  source /ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
  echo "WARN: /ws/install missing — run /ws/scripts/build.sh"
else
  echo "ERROR: build unitree msgs + go2_nav2 first"
  return 1 2>/dev/null || exit 1
fi
echo "Workspace: $(ros2 pkg list 2>/dev/null | grep -E '^go2_nav2$' || echo 'go2_nav2 NOT in path')"
