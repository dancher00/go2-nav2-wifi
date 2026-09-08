#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
if [[ -n "${GO2_NET:-}" && -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
  # shellcheck source=/dev/null
  source /usr/local/bin/setup-robot-net.sh
elif [[ ! -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  echo "WARN: CycloneDDS not installed — run: install-deps.sh minimal"
fi
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
