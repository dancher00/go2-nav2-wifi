#!/usr/bin/env bash
# From host: run Wi-Fi DDS check inside container (avoids /ws path on host).
set -euo pipefail
cd "$(dirname "$0")"
ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.58}"
IFACE="${GO2_WIFI_IFACE:-$(ip route show default 2>/dev/null | awk '{print $5; exit}')}"
HOST_IP="${GO2_HOST_IP:-$(ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet / {print $2}' | head -1 | cut -d/ -f1)}"
export GO2_ROBOT_IP="$ROBOT_IP" GO2_HOST_IP="$HOST_IP" GO2_WIFI_IFACE="$IFACE"
echo "Host: laptop=${HOST_IP} robot=${ROBOT_IP} iface=${IFACE}"
docker compose exec \
  -e GO2_ROBOT_IP="$ROBOT_IP" \
  -e GO2_HOST_IP="$HOST_IP" \
  -e GO2_WIFI_IFACE="$IFACE" \
  go2 bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws/scripts/setup-robot-wifi.sh
  ros2 daemon stop 2>/dev/null || true
  sleep 1
  ros2 daemon start
  sleep 2
  /ws/scripts/check-wifi-dds.sh
'
