#!/bin/bash
# Wi-Fi / phone hotspot: DDS bind to laptop IP + explicit peer to robot.
# Usage:
#   export GO2_ROBOT_IP=192.168.43.2   # robot IP on phone hotspot (see find-robot-ip.sh)
#   source /ws/scripts/setup-robot-wifi.sh
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

# Default route interface (usually wlp* / wlan* on laptop)
if [[ -z "${GO2_WIFI_IFACE:-}" ]]; then
  GO2_WIFI_IFACE=$(ip route show default 2>/dev/null | awk '{print $5; exit}')
fi
if [[ -z "${GO2_HOST_IP:-}" && -n "${GO2_WIFI_IFACE:-}" ]]; then
  GO2_HOST_IP=$(ip -4 addr show "$GO2_WIFI_IFACE" 2>/dev/null \
    | awk '/inet / {print $2}' | head -1 | cut -d/ -f1)
fi

GO2_ROBOT_IP="${GO2_ROBOT_IP:-}"
if [[ -z "$GO2_ROBOT_IP" ]]; then
  echo "ERROR: set robot IP in hotspot network, e.g.:"
  echo "  export GO2_ROBOT_IP=192.168.43.2"
  echo "  /ws/scripts/find-robot-ip.sh"
  return 1 2>/dev/null || exit 1
fi
if [[ -z "${GO2_HOST_IP:-}" ]]; then
  echo "ERROR: cannot detect laptop Wi-Fi IP. Connect to phone hotspot, then:"
  echo "  export GO2_WIFI_IFACE=wlp2s0"
  echo "  export GO2_HOST_IP=192.168.43.10"
  return 1 2>/dev/null || exit 1
fi

CDDS_RUNTIME="/tmp/go2-wifi-cyclonedds.xml"
cat > "$CDDS_RUNTIME" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface address="${GO2_HOST_IP}" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>${GO2_WIFI_MULTICAST:-false}</AllowMulticast>
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

if [[ -d /ws/install/go2_nav2 ]]; then
  case ":${AMENT_PREFIX_PATH:-}:" in
    *:/ws/install/go2_nav2:*) ;;
    *) export AMENT_PREFIX_PATH="/ws/install/go2_nav2${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
fi

if ! ping -c1 -W2 "${GO2_ROBOT_IP}" &>/dev/null; then
  echo "WARN: no ping to ${GO2_ROBOT_IP}" >&2
fi
