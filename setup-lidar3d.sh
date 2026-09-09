#!/usr/bin/env bash
# Host: install the isolated 3D image on Jetson (default) or laptop; no motion.
set -euo pipefail
LIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --legkilo ]]; then export GO2_LIDAR3D_BACKEND=legkilo; shift; fi
if [[ "${1:-}" != --laptop ]]; then
  [[ $# == 0 || ( $# == 1 && "$1" == --jetson ) ]] || { echo 'Usage: ./setup-lidar3d.sh [--jetson|--laptop]' >&2; exit 2; }
  exec bash "$LIO_ROOT/setup-lidar3d-jetson.sh"
fi
shift
[[ $# == 0 ]] || { echo 'Usage: ./setup-lidar3d.sh [--jetson|--laptop]' >&2; exit 2; }
: "${GO2_ROBOT_IP:?Set GO2_ROBOT_IP to the robot Wi-Fi address}"
: "${GO2_HOST_IP:?Set GO2_HOST_IP to the laptop Wi-Fi address}"
export GO2_UID="$(id -u)" GO2_GID="$(id -g)"
docker compose -p go2-lidar3d -f "$LIO_ROOT/docker/docker-compose.yml" \
  -f "$LIO_ROOT/docker/docker-compose.lidar3d.yml" up -d --build
docker exec go2-lidar3d bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --base-paths src/go2_nav2 src/go2_description --packages-select go2_nav2 go2_description'
echo 'Ready: ./mapping.sh --3d --laptop'
