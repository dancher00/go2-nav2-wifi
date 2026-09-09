#!/usr/bin/env bash
# Inside a project container: start/stop/status; Jetson 3D or laptop modes.
set -euo pipefail
SESSION_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == start ]]; then
  session_replay=0
  if [[ "${2:-}" == lidar3d ]]; then
    for argument in "$@"; do
      [[ "$argument" != --bag ]] || session_replay=1
    done
  fi
  if [[ "$session_replay" == 1 || ( "${2:-}" == lidar3d && "${GO2_LIDAR3D_COMPUTE:-laptop}" == jetson ) ]]; then
    # Offline inputs need ROS packages only, no robot ping or Wi-Fi configuration.
    set +u
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash
    source /ws/install/go2_nav2/share/go2_nav2/package.bash
    export AMENT_PREFIX_PATH="/ws/install/go2_nav2:${AMENT_PREFIX_PATH}"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    if [[ "$session_replay" != 1 ]]; then
      # Only this experiment's processes use internal DDS. Factory config stays intact.
      export ROS_DOMAIN_ID=0
      unset CYCLONEDDS_URI
    fi
  else
    source "$SESSION_SCRIPT_DIR/go2-env.sh"
  fi
  if [[ -f /opt/go2-unitree-msgs/local_setup.bash ]]; then
    source /opt/go2-unitree-msgs/local_setup.bash
  fi
  if [[ "${2:-}" == lidar3d ]]; then
    if [[ "${GO2_LIDAR3D_BACKEND:-pointlio}" == legkilo ]]; then
      source /opt/go2-legkilo/local_setup.bash
    elif [[ -f /opt/go2-lidar3d/local_setup.bash ]]; then
      source /opt/go2-lidar3d/local_setup.bash
    else
      source /ws/lidar3d/install/local_setup.bash
    fi
  fi
fi
# status/stop need no ROS discovery, ping, ROS daemon or robot connection.
exec python3 "$SESSION_SCRIPT_DIR/go2_session.py" "$@"
