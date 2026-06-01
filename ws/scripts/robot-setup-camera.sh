#!/usr/bin/env bash
# ON ROBOT: one-shot camera setup. Prefers C++ CLI (uses existing unitree_sdk2 build).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Go2 camera setup ==="
echo "1) C++ JPEG stream (recommended — same SDK as go2_video_client)"

if bash "${SCRIPT_DIR}/robot-build-camera-cli.sh"; then
  echo ""
  echo "OK — restart relay: export GO2_HOST_IP=...; bash ~/robot-relay-wifi.sh"
  exit 0
fi

echo ""
echo "2) Fallback: unitree_sdk2_python (builds CycloneDDS for pip if needed)"
bash "${SCRIPT_DIR}/robot-install-camera-sdk.sh"
echo ""
echo "OK — restart relay: export GO2_HOST_IP=...; bash ~/robot-relay-wifi.sh"
