#!/usr/bin/env bash
# Build only the pinned CMU ROS2 Point-LIO package, with our ROS I/O patch.
set -euo pipefail
LIO_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIO_WORKSPACE="${1:-/ws/lidar3d}"
LIO_INSTALL="${2:-$LIO_WORKSPACE/install}"
LIO_COMMIT=43d5f54b389b251713f0097893c30fa76c870d54
mkdir -p "$LIO_WORKSPACE"
touch "$LIO_WORKSPACE/COLCON_IGNORE"
if [[ ! -d "$LIO_WORKSPACE/upstream/.git" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/jizhang-cmu/autonomy_stack_go2.git "$LIO_WORKSPACE/upstream"
fi
git -C "$LIO_WORKSPACE/upstream" sparse-checkout set src/slam/point_lio_unilidar
git -C "$LIO_WORKSPACE/upstream" checkout --detach "$LIO_COMMIT"
mkdir -p "$LIO_WORKSPACE/src"
if [[ ! -d "$LIO_WORKSPACE/src/point_lio_unilidar" ]]; then
  cp -a "$LIO_WORKSPACE/upstream/src/slam/point_lio_unilidar" "$LIO_WORKSPACE/src/"
fi
if patch --dry-run -s -p1 -d "$LIO_WORKSPACE/src/point_lio_unilidar" < "$LIO_SCRIPT_DIR/pointlio-wifi.patch"; then
  patch -p1 -d "$LIO_WORKSPACE/src/point_lio_unilidar" < "$LIO_SCRIPT_DIR/pointlio-wifi.patch"
else
  patch --dry-run -s -R -p1 -d "$LIO_WORKSPACE/src/point_lio_unilidar" < "$LIO_SCRIPT_DIR/pointlio-wifi.patch"
fi
set +u  # ROS Humble setup scripts read optional, unset environment variables.
source /opt/ros/humble/setup.bash
set -u
cd "$LIO_WORKSPACE"
MAKEFLAGS=-j1 colcon build --base-paths src --install-base "$LIO_INSTALL" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
printf '%s\n' "$LIO_COMMIT" > "$LIO_INSTALL/pointlio-commit.txt"
sha256sum "$LIO_SCRIPT_DIR/pointlio-wifi.patch" > "$LIO_INSTALL/pointlio-patch.sha256"
