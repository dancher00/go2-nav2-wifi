#!/usr/bin/env bash
# ON ROBOT (once): build ~/go2_camera_jpeg_cli (user-writable dir, no SDK tree edits).
set -euo pipefail

OUT="${HOME}/bin/go2_camera_jpeg_cli"
BUILD="${HOME}/.go2_camera_build"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_find_unitree_sdk2() {
  local d
  for d in \
    "${UNITREE_SDK2:-}" \
    "${HOME}/unitree_sdk2" \
    "/home/unitree/unitree_sdk2" \
    "/opt/unitree/unitree_sdk2" \
    "/usr/local/unitree_sdk2"; do
    [[ -n "$d" ]] || continue
    if [[ -d "$d/include" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

if [[ -d "${HOME}/go2_camera_jpeg_cli" ]] && [[ ! -f "${HOME}/go2_camera_jpeg_cli" ]]; then
  echo "NOTE: ~/go2_camera_jpeg_cli is a folder — binary goes to ${OUT}"
fi
mkdir -p "${HOME}/bin"

if ! SDK="$(_find_unitree_sdk2 || true)"; then
  echo "ERROR: unitree_sdk2 C++ SDK not found." >&2
  echo "  A) bash ~/robot-install-camera-sdk.sh   # Python VideoClient (no C++ SDK)" >&2
  echo "  B) git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2" >&2
  echo "     cd ~/unitree_sdk2 && cmake -B build && cmake --build build -j\$(nproc)" >&2
  echo "     export UNITREE_SDK2=~/unitree_sdk2 && bash ~/robot-build-camera-cli.sh" >&2
  exit 1
fi
echo "Using UNITREE_SDK2=${SDK}"

mkdir -p "$BUILD"
SRC_CPP="${BUILD}/go2_camera_jpeg_cli.cpp"
if [[ -f "${SCRIPT_DIR}/go2_camera_jpeg_cli.cpp" ]]; then
  cp "${SCRIPT_DIR}/go2_camera_jpeg_cli.cpp" "$SRC_CPP"
elif [[ -f "${HOME}/go2_camera_jpeg_cli.cpp" ]]; then
  cp "${HOME}/go2_camera_jpeg_cli.cpp" "$SRC_CPP"
else
  echo "ERROR: missing go2_camera_jpeg_cli.cpp"
  exit 1
fi

if [[ -f "${SCRIPT_DIR}/go2_camera_jpeg_cli/CMakeLists.txt" ]]; then
  cp "${SCRIPT_DIR}/go2_camera_jpeg_cli/CMakeLists.txt" "${BUILD}/CMakeLists.txt"
elif [[ ! -f "${BUILD}/CMakeLists.txt" ]]; then
  echo "ERROR: missing CMakeLists.txt — scp ws/scripts/go2_camera_jpeg_cli/ from laptop"
  exit 1
fi

cmake -S "$BUILD" -B "${BUILD}/out" -DUNITREE_SDK2="$SDK"
cmake --build "${BUILD}/out" -j"$(nproc)"

cp -f "${BUILD}/out/go2_camera_jpeg_cli" "$OUT"
chmod +x "$OUT"
echo "OK: $OUT"
ls -la "$OUT"
