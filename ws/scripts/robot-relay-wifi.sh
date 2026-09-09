#!/usr/bin/env bash
# ON ROBOT: internal DDS domain 0 -> isolated Wi-Fi DDS domain 64.
# GO2_HOST_IP=<laptop-ip> bash ~/robot-relay-wifi.sh [--sensors-only]
# Factory DDS configuration is never changed.
set -euo pipefail

SENSORS_ONLY=0
case "${1:-}" in
  --sensors-only) SENSORS_ONLY=1 ;;
  "") ;;
  *) echo "Usage: $0 [--sensors-only]" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo "Too many arguments" >&2; exit 2; }
: "${GO2_HOST_IP:?Set GO2_HOST_IP to the laptop Wi-Fi IP}"
export GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
if [[ ! "$GO2_RELAY_DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] ||
   (( 10#$GO2_RELAY_DOMAIN_ID < 1 || 10#$GO2_RELAY_DOMAIN_ID > 101 )); then
  echo "ERROR: GO2_RELAY_DOMAIN_ID must be an integer from 1 to 101" >&2
  exit 2
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${GO2_RELAY_USE_HUMBLE:-0}" == 1 ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f /opt/go2-unitree-msgs/local_setup.bash ]]; then source /opt/go2-unitree-msgs/local_setup.bash; fi
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
  source "${SCRIPT_DIR}/robot-source-unitree-ros.sh"
fi
set -u
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

GO2_WIFI_IFACE="${GO2_WIFI_IFACE:-$(ip -br link | awk '/^wl/ {print $1; exit}')}"
[[ -n "$GO2_WIFI_IFACE" ]] || { echo "Set GO2_WIFI_IFACE=wlan0" >&2; exit 1; }
[[ "$GO2_WIFI_IFACE" =~ ^[a-zA-Z0-9_.:-]+$ && "$GO2_HOST_IP" =~ ^[a-zA-Z0-9_.:-]+$ ]] ||
  { echo "Invalid Wi-Fi interface or host address" >&2; exit 2; }

export GO2_RELAY_SOCKET="${GO2_RELAY_SOCKET:-/tmp/go2-relay-wifi.sock}"
export GO2_RELAY_READY="${GO2_RELAY_READY:-/tmp/go2-relay-wifi.ready}"
RELAY_XML="${GO2_RELAY_CYCLONEDDS_XML:-/tmp/go2-relay-wifi-cyclonedds.xml}"
# Refuse a second instance; never kill an unrelated bridge.
exec 9>"${GO2_RELAY_SOCKET}.lock"
flock -n 9 || { echo "Relay already running for $GO2_RELAY_SOCKET" >&2; exit 1; }
PIDS=()
cleanup() {
  local pid
  trap '' INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for _ in {1..30}; do
    local alive=0
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [[ "$alive" == 0 ]] && break
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f -- "$GO2_RELAY_SOCKET" "$GO2_RELAY_READY"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
rm -f -- "$GO2_RELAY_SOCKET" "$GO2_RELAY_READY"

cat > "$RELAY_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="${GO2_WIFI_IFACE}" /></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="${GO2_HOST_IP}"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

PY="${SCRIPT_DIR}/robot_relay_wifi.py"
[[ -f "$PY" ]] || { echo "Missing $PY" >&2; exit 1; }
CYCLONEDDS_URI="file://${RELAY_XML}" python3 "$PY" --role pub &
PIDS+=("$!")
PUB_PID=$!
for _ in {1..100}; do
  kill -0 "$PUB_PID" 2>/dev/null || { echo "Publisher exited during startup" >&2; exit 1; }
  [[ -f "$GO2_RELAY_READY" ]] && break
  sleep 0.1
done
[[ -f "$GO2_RELAY_READY" ]] || { echo "Publisher did not become ready" >&2; exit 1; }
python3 "$PY" --role sub &
PIDS+=("$!")

_camera_ok() {
  python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient" 2>/dev/null && return 0
  local backend
  for backend in "${GO2_CAMERA_JPEG_CLI:-}" "${HOME}/bin/go2_camera_jpeg_cli" \
    "${HOME}/go2_camera_jpeg_cli/go2_camera_jpeg_cli" \
    "${HOME}/.go2_camera_build/out/go2_camera_jpeg_cli" "${HOME}/go2_camera_jpeg_cli"; do
    [[ -f "$backend" && -x "$backend" ]] && return 0
  done
  return 1
}
if [[ "${GO2_RELAY_CAMERA:-1}" == 1 ]]; then
  CAM_BRIDGE="${SCRIPT_DIR}/robot_front_camera_bridge.py"
  if [[ -f "$CAM_BRIDGE" ]] && _camera_ok && python3 -c "import cv2" 2>/dev/null; then
    python3 "$CAM_BRIDGE" &
    PIDS+=("$!")
  else
    echo "WARN: camera unavailable; install its backend or use GO2_RELAY_CAMERA=0" >&2
  fi
fi

if [[ "$SENSORS_ONLY" == 1 ]]; then
  echo "SENSOR-ONLY: no cmd_vel TCP server or sport bridge started."
else
  # Both motion processes stay on the onboard domain 0.
  export GO2_CMD_VEL_HZ="${GO2_CMD_VEL_HZ:-20}"
  export GO2_SPORT_RATE_HZ="${GO2_SPORT_RATE_HZ:-$GO2_CMD_VEL_HZ}"
  export GO2_CMD_TIMEOUT="${GO2_CMD_TIMEOUT:-0.5}"
  python3 -c "from unitree_api.msg import Request"
  python3 "${SCRIPT_DIR}/go2_cmd_vel_tcp.py" --role server \
    --bind 0.0.0.0 --port "${GO2_CMD_TCP_PORT:-17999}" &
  PIDS+=("$!")
  python3 "${SCRIPT_DIR}/robot_sport_bridge.py" &
  PIDS+=("$!")
fi

echo "Relay running: onboard domain 0 -> Wi-Fi domain $GO2_RELAY_DOMAIN_ID"
# Any child exit tears down this owned process group, including motion bridges.
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
echo "Relay child exited (status $status); stopping relay." >&2
[[ "$status" != 0 ]] || status=1
exit "$status"
