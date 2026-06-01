#!/bin/bash
# Same as docker/setup-robot-eth.sh — use from container: source /ws/scripts/setup-robot-eth.sh
# docker compose exec skips entrypoint — always source Humble first.
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
  export PATH="/opt/ros/humble/bin:${PATH}"
else
  echo "ERROR: ROS Humble not in image — install-deps.sh nav2"
  return 1 2>/dev/null || exit 1
fi
UNITREE_INSTALL="/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
if [[ -f "$UNITREE_INSTALL" ]]; then
  source "$UNITREE_INSTALL"
else
  echo "WARN: no unitree msgs — run /ws/scripts/build-unitree-msgs.sh"
fi
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
else
  echo "WARN: no go2_nav2 — run /ws/scripts/build.sh"
fi

GO2_HOST_IP="${GO2_HOST_IP:-192.168.123.51}"
GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.123.18}"
CDDS_RUNTIME="/tmp/go2-eth-cyclonedds.xml"

cat > "$CDDS_RUNTIME" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface address="${GO2_HOST_IP}" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${GO2_ROBOT_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI="file://${CDDS_RUNTIME}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# ament_python + symlink-install: package prefix is not added to AMENT_PREFIX_PATH automatically
if [[ -d /ws/install/go2_nav2 ]]; then
  case ":${AMENT_PREFIX_PATH:-}:" in
    *:/ws/install/go2_nav2:*) ;;
    *) export AMENT_PREFIX_PATH="/ws/install/go2_nav2${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
fi

if ! ping -c1 -W1 "${GO2_ROBOT_IP}" &>/dev/null; then
  echo "WARN: no ping to ${GO2_ROBOT_IP}"
fi

