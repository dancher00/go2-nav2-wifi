#!/usr/bin/env bash
# Wi-Fi DDS check: laptop (Docker host) <-> Go2. Run inside container after setup-robot-wifi.sh
set -eo pipefail

ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.58}"
IFACE="${GO2_WIFI_IFACE:-$(ip route show default 2>/dev/null | awk '{print $5; exit}' || true)}"
HOST_IP="${GO2_HOST_IP:-}"
if [[ -z "$HOST_IP" && -n "$IFACE" ]]; then
  HOST_IP=$(ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet / {print $2}' | head -1 | cut -d/ -f1)
fi
HOST_IP="${HOST_IP:-?}"

echo "=== Wi-Fi DDS check ==="
echo "Laptop:  ${HOST_IP:-?} (${IFACE:-?})"
echo "Robot:   ${ROBOT_IP}"
echo "DOMAIN:  ${ROS_DOMAIN_ID:-unset}"
echo "RMW:     ${RMW_IMPLEMENTATION:-unset}"
echo "CYCLONE: ${CYCLONEDDS_URI:-unset}"
echo ""

if [[ -z "${CYCLONEDDS_URI:-}" ]] || [[ "${CYCLONEDDS_URI}" == *"/etc/cyclonedds/go2.xml"* ]]; then
  echo "WARN: using default /etc/cyclonedds/go2.xml — peer may be wrong."
  echo "      Run:  export GO2_ROBOT_IP=${ROBOT_IP}"
  echo "            source /ws/scripts/setup-robot-wifi.sh"
  echo ""
fi

echo "=== Ping robot ==="
ping -c2 -W2 "$ROBOT_IP" || { echo "FAIL ping — same Wi-Fi? AP isolation?"; exit 1; }

echo ""
echo "=== Cyclone config (peer line) ==="
if [[ -n "${CYCLONEDDS_URI:-}" && -f "${CYCLONEDDS_URI#file://}" ]]; then
  grep -E 'Peer|NetworkInterface' "${CYCLONEDDS_URI#file://}" || true
fi

echo ""
echo "=== ROS discovery (restart daemon, wait 12s) ==="
ros2 daemon stop 2>/dev/null || true
sleep 2
ros2 daemon start
sleep 5
LIST=$(ros2 topic list 2>&1) || true
echo "$LIST" | head -50
N=$(echo "$LIST" | wc -l)
echo ""
echo "Topics: $N"

if echo "$LIST" | grep -q utlidar; then
  echo "OK: utlidar topics visible"
  timeout 5 ros2 topic hz /utlidar/cloud_deskewed 2>&1 | head -3 || echo "WARN: cloud_deskewed slow/no data (robot stack running?)"
else
  echo ""
  echo "FAIL: no utlidar — checklist:"
  echo "  1. Laptop: export GO2_ROBOT_IP=${ROBOT_IP}; source /ws/scripts/setup-robot-wifi.sh"
  echo "  2. Unplug Ethernet cable on robot (DDS often on 192.168.123.18 only)"
  echo "  3. Retry: export GO2_WIFI_MODE=multicast; source /ws/scripts/setup-robot-wifi.sh"
  echo "  4. Robot: bash ~/robot-show-dds.sh — which IP is in cyclonedds xml?"
  echo "  5. Router: disable AP/client isolation"
fi
