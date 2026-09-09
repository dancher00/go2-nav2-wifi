#!/usr/bin/env bash
# Laptop: install only into the Jetson's isolated 3D experiment directory.
set -euo pipefail
LIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${GO2_ROBOT_IP:?Set GO2_ROBOT_IP to the Jetson Wi-Fi address}"
robot_user="${GO2_ROBOT_USER:-unitree}"
[[ "$GO2_ROBOT_IP" =~ ^[a-zA-Z0-9_.:-]+$ && "$robot_user" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
target="$robot_user@$GO2_ROBOT_IP"
backend="${GO2_LIDAR3D_BACKEND:-pointlio}"
[[ "$backend" == pointlio || "$backend" == legkilo ]] || exit 2
mkdir -p "$LIO_ROOT/ws/log"
stage="$(mktemp -d "$LIO_ROOT/ws/log/jetson-setup.XXXXXX")"
socket="$stage/ssh"
cleanup() { ssh -S "$socket" -O exit "$target" >/dev/null 2>&1 || true; }
trap cleanup EXIT
ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 -S "$socket" "$target" true
tar -czf "$stage/runtime.tar.gz" -C "$LIO_ROOT" go2_nav2 \
  docker/Dockerfile.lidar3d docker/Dockerfile.lidar3d.dockerignore \
  docker/Dockerfile.legkilo docker/Dockerfile.legkilo.dockerignore ws/scripts/legkilo-go2.patch \
  ws/scripts/build-lidar3d.sh ws/scripts/pointlio-wifi.patch ws/scripts/go2-session.sh \
  ws/scripts/go2_session.py ws/scripts/session_preflight.py ws/scripts/export-lidar3d.py \
  ws/scripts/robot-relay-wifi.sh ws/scripts/robot_relay_wifi.py
ssh -S "$socket" "$target" 'mkdir -p ~/go2-nav2-lidar3d-onboard'
scp -q -o "ControlPath=$socket" "$stage/runtime.tar.gz" "$target:go2-nav2-lidar3d-onboard/"
ssh -S "$socket" "$target" bash -s -- "$backend" <<'REMOTE'
set -euo pipefail
cd ~/go2-nav2-lidar3d-onboard
tar xzf runtime.tar.gz
rm runtime.tar.gz
mkdir -p ws/src
docker build --build-arg GO2_UID="$(id -u)" --build-arg GO2_GID="$(id -g)" \
  -f docker/Dockerfile.lidar3d -t go2-lidar3d:local .
onboard_image=go2-lidar3d:local
if [[ "$1" == legkilo ]]; then
  docker build -f docker/Dockerfile.legkilo -t go2-legkilo:local .
  onboard_image=go2-legkilo:local
fi
if docker inspect go2-lidar3d-onboard >/dev/null 2>&1; then
  if [[ "$(docker inspect -f '{{.State.Running}}' go2-lidar3d-onboard)" == true ]]; then
    docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d
  fi
  docker rm -f go2-lidar3d-onboard
fi
docker run -d --name go2-lidar3d-onboard --network host --user "$(id -u):$(id -g)" \
  -e GO2_LIDAR3D_COMPUTE=jetson -e GO2_NET=wifi -e GO2_RELAY_DOMAIN_ID=64 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$PWD/ws:/ws" -v "$PWD/go2_nav2:/ws/src/go2_nav2" -w /ws \
  --entrypoint /bin/sleep "$onboard_image" infinity
docker exec go2-lidar3d-onboard bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --base-paths src/go2_nav2 --packages-select go2_nav2'
REMOTE
if [[ "$backend" == legkilo ]]; then
  echo 'Installed on Jetson. Start from the laptop: ./mapping.sh --3d --legkilo'
else
  echo 'Installed on Jetson. Start from the laptop: ./mapping.sh --3d'
fi
