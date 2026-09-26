#!/usr/bin/env bash
set -euo pipefail
map_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ $# == 2 && "$2" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || {
  echo 'Usage: ./save-3d-map.sh <completed-run-directory> <new-map-name>' >&2; exit 2;
}
PYTHONPATH="$map_root/go2_nav2${PYTHONPATH:+:$PYTHONPATH}" python3 -m go2_nav2.lidar3d_map \
  "$1" "$map_root/ws/maps/lidar3d/saved/$2" --mount "$map_root/go2_nav2/config/lidar3d_robot_viz.yaml"
