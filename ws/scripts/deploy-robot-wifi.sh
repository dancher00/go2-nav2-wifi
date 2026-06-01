#!/usr/bin/env bash
# Deploy Go2 Jetson scripts for Wi-Fi relay stack (laptop Docker + NAVIGATION.md).
#
# Usage:
#   ./deploy-robot-wifi.sh unitree@192.168.1.58          # full (default)
#   ./deploy-robot-wifi.sh --minimal unitree@192.168.1.58
#   GO2_ROBOT_SSH=unitree@IP ./deploy-robot-wifi.sh
set -euo pipefail

MODE=full
if [[ "${1:-}" == "--minimal" ]]; then
  MODE=minimal
  shift
fi

ROBOT="${1:-${GO2_ROBOT_SSH:-unitree@192.168.1.58}}"
DIR="$(cd "$(dirname "$0")" && pwd)"

MINIMAL=(
  robot-source-unitree-ros.sh
  robot-relay-wifi.sh
  robot_relay_wifi.py
  robot_sport_bridge.py
  go2_cmd_vel_tcp.py
  robot-build-unitree-msgs.sh
)

FULL=(
  "${MINIMAL[@]}"
  robot_front_camera_bridge.py
  go2_camera_jpeg_cli.cpp
  robot-bootstrap-unitree-sdk2.sh
  robot-build-camera-cli.sh
  robot-setup-wifi-robot.sh
  robot-install-camera-sdk.sh
  robot-setup-camera.sh
  robot-check-topics.sh
  robot-check-cmdvel.sh
  robot-show-dds.sh
)

if [[ "$MODE" == "minimal" ]]; then
  FILES=("${MINIMAL[@]}")
else
  FILES=("${FULL[@]}")
fi

paths=()
for f in "${FILES[@]}"; do
  p="${DIR}/${f}"
  [[ -f "$p" ]] || { echo "Missing ${p}" >&2; exit 1; }
  paths+=("$p")
done

scp "${paths[@]}" "${ROBOT}:~/"

if [[ "$MODE" == "full" ]]; then
  cmake_list="${DIR}/go2_camera_jpeg_cli/CMakeLists.txt"
  [[ -f "$cmake_list" ]] || { echo "Missing ${cmake_list}" >&2; exit 1; }
  ssh "$ROBOT" "mkdir -p go2_camera_jpeg_cli"
  scp "$cmake_list" "${ROBOT}:~/go2_camera_jpeg_cli/CMakeLists.txt"
fi

echo "Deployed (${MODE}) to ${ROBOT}"
echo ""
echo "On robot (SSH) — once per new Jetson (same as first robot):"
echo "  bash ~/robot-setup-wifi-robot.sh"
echo "  # or step by step: robot-build-unitree-msgs.sh, robot-bootstrap-unitree-sdk2.sh,"
echo "  #   robot-build-camera-cli.sh, apt install python3-opencv"
echo ""
echo "Each session:"
echo "  bash ~/robot-check-topics.sh        # expect utlidar in list"
echo "  export GO2_HOST_IP=<laptop Wi-Fi IP>"
echo "  bash ~/robot-relay-wifi.sh"
echo ""
echo "On laptop: set GO2_ROBOT_IP in docker/.env — see docs/RELAY-WIFI.md"
