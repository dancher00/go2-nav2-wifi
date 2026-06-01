#!/usr/bin/env bash
# ON ROBOT (once): install ~/unitree_sdk2 like factory Go2 Edu (needed for robot-build-camera-cli.sh).
set -euo pipefail

SDK="${UNITREE_SDK2:-${HOME}/unitree_sdk2}"

if [[ -d "${SDK}/include" ]] && [[ -f "${SDK}/build/bin/go2_video_client" || -f "${SDK}/lib/libunitree_sdk2.so" || -f "${SDK}/build/lib/libunitree_sdk2.so" ]]; then
  echo "unitree_sdk2 OK at ${SDK}"
  exit 0
fi

if [[ -d "${SDK}/include" ]]; then
  echo "unitree_sdk2 headers at ${SDK}; building..."
else
  echo "Cloning unitree_sdk2 → ${SDK}"
  git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git "${SDK}"
fi

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git libyaml-cpp-dev 2>/dev/null || true

cmake -S "${SDK}" -B "${SDK}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${SDK}/build" -j"$(nproc)"

echo "OK: ${SDK}"
ls -la "${SDK}/build/bin/go2_video_client" 2>/dev/null || ls -la "${SDK}/build/lib/" 2>/dev/null | head -5
