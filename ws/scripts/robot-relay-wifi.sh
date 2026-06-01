#!/usr/bin/env bash
# ON ROBOT: relay internal Unitree topics (eth0 DDS) to Wi-Fi for laptop.
#
#   export GO2_HOST_IP=192.168.1.90    # laptop Wi-Fi IP
#   bash ~/robot-relay-wifi.sh
#
# Laptop (Docker): export GO2_ROBOT_IP=192.168.1.58; source /ws/scripts/setup-robot-wifi.sh
# Check: bash /ws/scripts/check-wifi-dds.sh
#
# Does NOT modify /home/unitree/cyclonedds_ws/cyclonedds.xml — factory eth0 stays for Unitree stack.

set -euo pipefail

GO2_HOST_IP="${GO2_HOST_IP:-}"
GO2_ROBOT_WIFI_IP="${GO2_ROBOT_WIFI_IP:-192.168.1.58}"
GO2_WIFI_IFACE="${GO2_WIFI_IFACE:-}"
RELAY_XML="${GO2_RELAY_CYCLONEDDS_XML:-/tmp/go2-relay-wifi-cyclonedds.xml}"
export GO2_CMD_VEL_HZ="${GO2_CMD_VEL_HZ:-20}"
export GO2_SPORT_RATE_HZ="${GO2_SPORT_RATE_HZ:-$GO2_CMD_VEL_HZ}"

if [[ -z "$GO2_HOST_IP" ]]; then
  echo "Set laptop IP: export GO2_HOST_IP=192.168.1.90"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_SH="${SCRIPT_DIR}/robot-source-unitree-ros.sh"
[[ -f "$SOURCE_SH" ]] || SOURCE_SH="${HOME}/robot-source-unitree-ros.sh"
[[ -f "$SOURCE_SH" ]] || { echo "Missing robot-source-unitree-ros.sh"; exit 1; }

# shellcheck source=/dev/null
source "$SOURCE_SH"

if [[ -z "$GO2_WIFI_IFACE" ]]; then
  while read -r ifname _; do
    if ip -4 addr show "$ifname" 2>/dev/null | grep -q "inet ${GO2_ROBOT_WIFI_IP}/"; then
      GO2_WIFI_IFACE="$ifname"
      break
    fi
  done < <(ip -br link | awk '{print $1}')
fi
if [[ -z "$GO2_WIFI_IFACE" ]]; then
  GO2_WIFI_IFACE=$(ip -br link 2>/dev/null | awk '/^wl/ {print $1; exit}')
fi
[[ -n "$GO2_WIFI_IFACE" ]] || { echo "Set GO2_WIFI_IFACE=wlan0"; exit 1; }

cat > "$RELAY_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="${GO2_WIFI_IFACE}" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="${GO2_HOST_IP}"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

PY="${SCRIPT_DIR}/robot_relay_wifi.py"
[[ -f "$PY" ]] || PY="${HOME}/robot_relay_wifi.py"
[[ -f "$PY" ]] || { echo "Missing robot_relay_wifi.py next to this script"; exit 1; }
chmod +x "$PY" 2>/dev/null || true

unset CYCLONEDDS_URI
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start
sleep 3
PREFLIGHT=$(ros2 topic list 2>&1) || true
if ! echo "$PREFLIGHT" | grep -q utlidar; then
  echo "WARN: no utlidar in preflight" >&2
fi
if ! echo "$PREFLIGHT" | grep -q lowstate; then
  echo "WARN: no /lf/lowstate" >&2
fi
if ! python3 -c "from rosidl_runtime_py.utilities import get_message; get_message('unitree_go/msg/LowState')" 2>/dev/null; then
  echo "WARN: unitree_go missing — run robot-build-unitree-msgs.sh" >&2
fi

export GO2_RELAY_SOCKET="${GO2_RELAY_SOCKET:-/tmp/go2-relay-wifi.sock}"
export GO2_RELAY_READY="${GO2_RELAY_READY:-/tmp/go2-relay-wifi.ready}"
rm -f "$GO2_RELAY_SOCKET" "$GO2_RELAY_READY"

CAM_BRIDGE="${SCRIPT_DIR}/robot_front_camera_bridge.py"
[[ -f "$CAM_BRIDGE" ]] || CAM_BRIDGE="${HOME}/robot_front_camera_bridge.py"
_camera_ok() {
  python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient" 2>/dev/null && return 0
  for p in \
    "${HOME}/bin/go2_camera_jpeg_cli" \
    "${HOME}/go2_camera_jpeg_cli/go2_camera_jpeg_cli" \
    "${HOME}/.go2_camera_build/out/go2_camera_jpeg_cli"; do
    [[ -f "$p" ]] && [[ -x "$p" ]] && return 0
  done
  [[ -f "${HOME}/go2_camera_jpeg_cli" ]] && [[ -x "${HOME}/go2_camera_jpeg_cli" ]] && return 0
  return 1
}

CAM_PID=""

if [[ "${GO2_RELAY_CAMERA:-1}" == "1" ]] && [[ -f "$CAM_BRIDGE" ]] && _camera_ok; then
  if python3 -c "import cv2" 2>/dev/null; then
    echo "Starting front camera bridge (mono8 ${GO2_CAM_WIDTH:-160}x${GO2_CAM_HEIGHT:-120} @ ${GO2_CAM_FPS:-10} Hz)..."
    python3 "$CAM_BRIDGE" &
    CAM_PID=$!
    sleep 2
    if ! kill -0 "$CAM_PID" 2>/dev/null; then
      echo "WARN: camera bridge exited — try: bash ~/robot-install-camera-sdk.sh"
      CAM_PID=""
    fi
  else
    echo "WARN: python3-opencv missing — skip camera (apt: python3-opencv)"
  fi
elif [[ "${GO2_RELAY_CAMERA:-1}" == "1" ]]; then
  echo "WARN: no camera backend — run: bash ~/robot-install-camera-sdk.sh"
  echo "      (or: bash ~/robot-build-camera-cli.sh)"
fi

cleanup() {
  kill "$PUB_PID" "$CMD_TCP_PID" "$SPORT_PID" 2>/dev/null || true
  [[ -n "$CAM_PID" ]] && kill "$CAM_PID" 2>/dev/null || true
  rm -f "$GO2_RELAY_SOCKET" "$GO2_RELAY_READY"
}
trap cleanup EXIT INT TERM

GO2_CMD_TCP_PORT="${GO2_CMD_TCP_PORT:-17999}"
CMD_TCP="${SCRIPT_DIR}/go2_cmd_vel_tcp.py"
[[ -f "$CMD_TCP" ]] || CMD_TCP="${HOME}/go2_cmd_vel_tcp.py"

# cmd_vel laptop -> robot via TCP (DDS reverse path often fails on Wi-Fi)
unset CYCLONEDDS_URI
CMD_TCP_PID=""
if [[ -f "$CMD_TCP" ]]; then
  echo "cmd_vel TCP server on 0.0.0.0:${GO2_CMD_TCP_PORT} -> /cmd_vel"
  python3 "$CMD_TCP" --role server --bind 0.0.0.0 --port "$GO2_CMD_TCP_PORT" &
  CMD_TCP_PID=$!
  sleep 0.5
else
  echo "WARN: missing go2_cmd_vel_tcp.py — copy from go2 repo ws/scripts/"
fi

SPORT_BRIDGE="${SCRIPT_DIR}/robot_sport_bridge.py"
[[ -f "$SPORT_BRIDGE" ]] || SPORT_BRIDGE="${HOME}/robot_sport_bridge.py"
SPORT_PID=""
if [[ -f "$SPORT_BRIDGE" ]]; then
  if python3 -c "from unitree_api.msg import Request" 2>/dev/null; then
    export GO2_CMD_TIMEOUT="${GO2_CMD_TIMEOUT:-8.0}"
    pkill -f '[r]obot_sport_bridge.py' 2>/dev/null || true
    sleep 0.3
    python3 "$SPORT_BRIDGE" &
    SPORT_PID=$!
    sleep 1
    if ! kill -0 "$SPORT_PID" 2>/dev/null; then
      echo "WARN: sport_bridge exited — build unitree_api: bash ~/robot-build-unitree-msgs.sh"
      SPORT_PID=""
    else
      echo "Sport bridge on robot (teleop/Nav2 cmd_vel via relay)"
    fi
  else
    echo "WARN: unitree_api missing — teleop/Nav2 won't move robot over Wi-Fi"
    echo "      ONE-TIME: bash ~/robot-build-unitree-msgs.sh"
  fi
else
  echo "WARN: missing robot_sport_bridge.py — copy from go2 repo ws/scripts/"
fi

export CYCLONEDDS_URI="file://${RELAY_XML}"
python3 "$PY" --role pub &
PUB_PID=$!

for _ in $(seq 1 50); do
  [[ -f "$GO2_RELAY_READY" ]] && break
  sleep 0.2
done
if [[ ! -f "$GO2_RELAY_READY" ]]; then
  echo "FAIL: pub process did not become ready"
  exit 1
fi

unset CYCLONEDDS_URI
python3 "$PY" --role sub
