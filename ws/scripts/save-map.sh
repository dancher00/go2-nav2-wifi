#!/usr/bin/env bash
# Source the ROS environment first. Existing map names are never overwritten.
set -euo pipefail
exec python3 -m go2_nav2.map_bundle save "${1:-my_room}" "${2:-/ws/maps}"
