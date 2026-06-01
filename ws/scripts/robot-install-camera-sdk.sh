#!/usr/bin/env bash
# ON ROBOT (once): pip install unitree_sdk2_python (needs CycloneDDS C libs for pip cyclonedds).
set -euo pipefail

REPO="${HOME}/unitree_sdk2_python"
CDDS_INSTALL="${CYCLONEDDS_HOME:-${HOME}/cyclonedds/install}"

if python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient" 2>/dev/null; then
  echo "unitree_sdk2_python already OK"
  exit 0
fi

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  python3-pip python3-opencv python3-numpy git \
  build-essential cmake 2>/dev/null || true

_find_cyclonedds_home() {
  local d
  for d in \
    "${CYCLONEDDS_HOME:-}" \
    "${HOME}/cyclonedds/install" \
    "${HOME}/cyclonedds_ws/install" \
    "/home/unitree/cyclonedds_ws/install"; do
    [[ -n "$d" ]] || continue
    if [[ -f "$d/include/dds/dds.h" ]] || [[ -f "$d/lib/libddsc.so" ]] || \
       [[ -f "$d/lib/aarch64/libddsc.so" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

_build_cyclonedds() {
  echo "=== Building CycloneDDS 0.10.x → ${CDDS_INSTALL} (~3 min) ==="
  local src="${HOME}/cyclonedds"
  if [[ ! -d "$src/.git" ]]; then
    git clone --depth 1 --branch releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git "$src"
  fi
  cmake -S "$src" -B "${src}/build" \
    -DCMAKE_INSTALL_PREFIX="${CDDS_INSTALL}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DENABLE_SECURITY=OFF \
    -DENABLE_SHM=OFF
  cmake --build "${src}/build" -j"$(nproc)" --target install
}

if ! CDDS_HOME="$(_find_cyclonedds_home || true)"; then
  _build_cyclonedds
  CDDS_HOME="${CDDS_INSTALL}"
fi

export CYCLONEDDS_HOME="$CDDS_HOME"
export CMAKE_PREFIX_PATH="${CYCLONEDDS_HOME}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:${LD_LIBRARY_PATH:-}"

SDK="${UNITREE_SDK2:-${HOME}/unitree_sdk2}"
ARCH="$(uname -m)"
if [[ -d "${SDK}/thirdparty/lib/${ARCH}" ]]; then
  export LD_LIBRARY_PATH="${SDK}/thirdparty/lib/${ARCH}:${SDK}/lib:${SDK}/build/lib:${LD_LIBRARY_PATH}"
fi

echo "CYCLONEDDS_HOME=${CYCLONEDDS_HOME}"

if [[ ! -d "$REPO/.git" ]]; then
  git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git "$REPO"
fi

pip3 install --user -e "$REPO"

python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient; print('unitree_sdk2_python OK')"
