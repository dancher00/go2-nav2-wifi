#!/usr/bin/env bash
# Separate install: never overwrite the stock Point-LIO executable.
set -euo pipefail
leg_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
leg_workspace="${1:-/ws/pointlio-leg}"
leg_install="${2:-$leg_workspace/install}"
if [[ ! -d "$leg_workspace/src/point_lio_unilidar" ]]; then
  bash "$leg_script_dir/build-lidar3d.sh" "$leg_workspace" "$leg_install"
fi
leg_source="$leg_workspace/src/point_lio_unilidar"
if patch --dry-run --forward --batch -s -p1 -d "$leg_source" < "$leg_script_dir/pointlio-leg.patch"; then
  patch -s -p1 -d "$leg_source" < "$leg_script_dir/pointlio-leg.patch"
else
  patch --dry-run --batch -s -R -p1 -d "$leg_source" < "$leg_script_dir/pointlio-leg.patch"
fi
cp "$leg_script_dir"/pointlio-leg/*.hpp "$leg_source/src/"
set +u
source /opt/ros/humble/setup.bash
source /opt/go2-unitree-msgs/local_setup.bash
set -u
cd "$leg_workspace"
MAKEFLAGS=-j1 colcon build --base-paths src --install-base "$leg_install" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
sha256sum "$leg_script_dir/pointlio-leg.patch" "$leg_script_dir"/pointlio-leg/*.hpp > "$leg_install/pointlio-leg.sha256"
