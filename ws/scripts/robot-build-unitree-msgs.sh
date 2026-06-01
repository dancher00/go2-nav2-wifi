#!/usr/bin/env bash
# ON ROBOT (Foxy, once): build unitree_go Python msgs for Wi-Fi relay /lf/lowstate.
#
#   bash ~/robot-build-unitree-msgs.sh
#   export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh

set -euo pipefail
set +u

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ERROR: need ROS Foxy on robot (/opt/ros/foxy)"
  exit 1
fi
source /opt/ros/foxy/setup.bash

REPO="${HOME}/unitree_ros2"
INSTALL="${REPO}/cyclonedds_ws/install/setup.bash"
UGO_SITE="${REPO}/cyclonedds_ws/install/unitree_go/lib/python3.8/site-packages"

_apply_unitree_pythonpath() {
  if [[ -f "$INSTALL" ]]; then
    # shellcheck source=/dev/null
    source "$INSTALL"
  fi
  if [[ -d "${UGO_SITE}/unitree_go" ]]; then
    export PYTHONPATH="${UGO_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
}

_unitree_go_ok() {
  _apply_unitree_pythonpath
  python3 -c "
from rosidl_runtime_py.utilities import get_message
get_message('unitree_go/msg/LowState')
get_message('unitree_api/msg/Request')
" 2>/dev/null
}

if _unitree_go_ok; then
  echo "unitree_go + unitree_api OK (skip build)"
  exit 0
fi

if [[ -f "$INSTALL" ]]; then
  echo "install exists but Python import failed — rebuilding unitree_go..."
fi

echo "=== Clone unitree_ros2 (if needed) ==="
if [[ ! -d "$REPO/.git" ]]; then
  git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git "$REPO"
fi

echo "=== Build unitree_api + unitree_go ==="
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  ros-foxy-rosidl-generator-dds-idl \
  2>/dev/null || true

cd "${REPO}/cyclonedds_ws"
colcon build --packages-select unitree_api unitree_go --symlink-install

echo "=== Verify ==="
_apply_unitree_pythonpath
python3 -c "
from rosidl_runtime_py.utilities import get_message
get_message('unitree_go/msg/LowState')
get_message('unitree_api/msg/Request')
print('unitree_go OK')
"

echo ""
echo "Done. Re-run:"
echo "  export GO2_HOST_IP=192.168.1.90"
echo "  bash ~/robot-relay-wifi.sh"
