#!/usr/bin/env bash
# ON ROBOT (once): same stack as first Go2 Edu — unitree msgs + camera CLI + opencv.
# Laptop IP (same as first robot): export GO2_HOST_IP=192.168.1.90
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"

if [[ -z "${GO2_HOST_IP:-}" ]]; then
  export GO2_HOST_IP=192.168.1.90
fi

bash "${DIR}/robot-build-unitree-msgs.sh"
bash "${DIR}/robot-bootstrap-unitree-sdk2.sh"
bash "${DIR}/robot-build-camera-cli.sh"

if ! python3 -c "import cv2" 2>/dev/null; then
  sudo apt-get update -qq
  sudo apt-get install -y --no-install-recommends python3-opencv 2>/dev/null || true
fi

echo ""
echo "Done. Start relay:"
echo "  export GO2_HOST_IP=<laptop Wi-Fi IP>"
echo "  bash ~/robot-relay-wifi.sh"
echo "Expect: Sport bridge + front camera bridge (mono8 160x120)"
