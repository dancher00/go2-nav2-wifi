#!/usr/bin/env bash
# Own a separate Point-LIO instance plus camera coloring. No robot motion commands.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot="${GO2_ROBOT_IP:-192.168.8.245}"
user="${GO2_ROBOT_USER:-unitree}"
[[ "$robot" =~ ^[0-9.]+$ && "$user" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
SSH=(ssh -o ConnectTimeout=8)
SCP=(scp)
if [[ -n "${GO2_SSH_CONTROL_PATH:-}" ]]; then
 SSH+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH");SCP+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
fi
target="$user@$robot"
case "${1:-help}" in
 start-preview)
  camera_state="$("${SSH[@]}" "$target" 'docker inspect --format "{{.State.Status}}" go2-stock-camera 2>/dev/null || true')"
  if [[ "$camera_state" != running ]]; then
   if [[ -n "$camera_state" ]]; then bash "$ROOT/stock-camera.sh" stop; fi
   bash "$ROOT/stock-camera.sh" start
  fi
  "${SSH[@]}" "$target" 'mkdir -p ~/go2-pointlio-camera-fusion/mapper/go2_nav2'
  "${SCP[@]}" "$ROOT/go2_nav2/go2_nav2/cloud_stamp_sync.py" "$ROOT/go2_nav2/go2_nav2/sensor_time.py" "$ROOT/go2_nav2/go2_nav2/__init__.py" "$target:go2-pointlio-camera-fusion/mapper/go2_nav2/"
  "${SCP[@]}" "${GO2_FUSION_LIO_CONFIG:-$ROOT/go2_nav2/config/pointlio_go2.yaml}" "$target:go2-pointlio-camera-fusion/mapper/pointlio_go2.yaml"
  "${SCP[@]}" "$ROOT/ws/scripts/camera_fusion/mapper.py" "$target:go2-pointlio-camera-fusion/mapper/"
  "${SSH[@]}" "$target" bash -s <<'REMOTE'
set -euo pipefail
cd ~/go2-pointlio-camera-fusion
if docker inspect go2-camera-pointlio >/dev/null 2>&1; then echo 'Owned Point-LIO container exists; use stop first.'; exit 1; fi
session="$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "pointlio/$session"
ln -sfn "$session" pointlio/latest
docker run -d --name go2-camera-pointlio --network host --init --memory 2500m \
 --stop-signal SIGINT --stop-timeout 50 -e ROS_DOMAIN_ID=0 \
 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
 -e 'CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>' \
 -v "$PWD/mapper:/work:ro" -v "$PWD/pointlio/$session:/data" \
 --entrypoint /bin/bash go2-legkilo:local \
 -c 'source /opt/ros/humble/setup.bash && source /opt/go2-lidar3d/setup.bash && test "$(cut -d " " -f 1 /opt/go2-lidar3d/pointlio-patch.sha256)" = a1e8d3e215bb9f38e4e45269e1a9fbd07341193113d70e962b38ae669877b738 && exec python3 -u /work/mapper.py'
REMOTE
  mkdir -p "$ROOT/ws/log"
  python3 - "$ROOT/go2_nav2/config/camera_fusion_preview.json" "$ROOT/ws/log/owned-fusion-preview.json" <<'CONFIG'
import json,sys
c=json.load(open(sys.argv[1]))
c.update(map_frame='camera_fusion_map',odom_topic='/camera_fusion/lio/odom',cloud_topic='/camera_fusion/lio/registered')
with open(sys.argv[2],'w') as f: json.dump(c,f,indent=2)
CONFIG
  if ! GO2_FUSION_CALIBRATION="$ROOT/ws/log/owned-fusion-preview.json" bash "$ROOT/camera-fusion.sh" start-preview; then
   "${SSH[@]}" "$target" 'docker stop go2-camera-pointlio'
   exit 1
  fi
  ;;
 capture-static)
  "${SCP[@]}" "$ROOT/ws/scripts/camera_fusion/capture_static_raw.py" "$ROOT/ws/scripts/camera_fusion/check_projection.py" "$target:go2-pointlio-camera-fusion/work/camera_fusion/"
  "${SSH[@]}" "$target" bash -s <<'CAPTURE'
set -euo pipefail
folder="/data/static-raw-$(date -u +%Y%m%dT%H%M%S)"
docker exec -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI=file:///ipc/native.xml go2-camera-fusion bash -lc 'source /opt/ros/humble/setup.bash && exec python3 /work/camera_fusion/capture_static_raw.py --output "$1"' bash "$folder"
docker exec go2-camera-fusion python3 /work/camera_fusion/check_projection.py "$folder"
CAPTURE
  ;;
 restart-preview)
  bash "$ROOT/camera-fusion.sh" stop
  GO2_FUSION_CALIBRATION="$ROOT/ws/log/owned-fusion-preview.json" bash "$ROOT/camera-fusion.sh" start-preview
  ;;
 stop)
  bash "$ROOT/camera-fusion.sh" stop
  "${SSH[@]}" "$target" 'docker stop go2-camera-pointlio && docker logs go2-camera-pointlio > ~/go2-pointlio-camera-fusion/pointlio/latest/launch.log 2>&1 && docker rm go2-camera-pointlio'
  ;;
 rviz) GO2_FUSION_MAP_FRAME=camera_fusion_map bash "$ROOT/camera-fusion.sh" rviz ;;
 status|save|check-projection) bash "$ROOT/camera-fusion.sh" "$1" ;;
 *) echo 'Usage: bash fusion-mapping.sh {start-preview|capture-static|restart-preview|check-projection|status|save|rviz|stop}' ;;
esac
