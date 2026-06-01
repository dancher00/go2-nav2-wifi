#!/usr/bin/env bash
# Build CycloneDDS 0.10.x like on Go2 robot (fixes Humble apt DDS not seeing robot)
set -euo pipefail
cd "$(dirname "$0")"
chmod +x source-unitree.sh

echo "=== Unitree CycloneDDS 0.10.x build in Docker (~10–30 min) ==="
./run.sh bash -lc '
set -e
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  git build-essential cmake \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev

mkdir -p /ws/src
cd /ws/src
[[ -d unitree_ros2 ]] || git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git

CDDS_WS=/ws/src/unitree_ros2/cyclonedds_ws
mkdir -p "$CDDS_WS/src" && cd "$CDDS_WS/src"
[[ -d cyclonedds ]] || git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git
[[ -d rmw_cyclonedds ]] || git clone --depth 1 -b humble https://github.com/ros2/rmw_cyclonedds.git

cd "$CDDS_WS"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "cyclonedds_ws build done"
'

echo ""
echo "In container after ./run.sh:"
echo "  source /usr/local/bin/source-unitree.sh"
echo "  check-robot.sh"
