#!/usr/bin/env bash
# ON ROBOT: same ROS2 + CycloneDDS env as robot-check-topics.sh (121 topics).
# Usage: source ~/robot-source-unitree-ros.sh
set +u
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
if [[ -f /opt/ros/foxy/setup.bash ]]; then
  source /opt/ros/foxy/setup.bash
else
  echo "No /opt/ros/foxy/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi
for d in \
  /opt/unitree/ros2/setup.bash \
  /opt/unitree/*/setup.bash \
  /home/unitree/cyclonedds_ws/install/setup.bash \
  /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash \
  /home/unitree/unitree_ros2/install/setup.bash \
  /home/unitree/*/install/setup.bash; do
  [[ -f "$d" ]] && source "$d"
done

# Foxy robot often has C++ unitree stack but no unitree_go in default PYTHONPATH.
if ! python3 -c "import unitree_go" 2>/dev/null; then
  _ugo=""
  for _p in \
    /home/unitree/unitree_ros2/cyclonedds_ws/install/unitree_go/lib/python3.8/site-packages \
    /opt/unitree/ros2/local/lib/python3.8/dist-packages; do
    if [[ -d "$_p/unitree_go" ]]; then
      _ugo="$_p"
      break
    fi
  done
  if [[ -n "$_ugo" ]]; then
    export PYTHONPATH="${_ugo}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Do NOT set CYCLONEDDS_URI here — unset + unitree libs = 121 topics (robot-check-topics).
# Explicit cyclonedds.xml breaks ros2 discovery (preflight → 2).
