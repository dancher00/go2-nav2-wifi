#!/usr/bin/env bash
# Optional read-only Point-LIO color/density add-on. Never starts/stops Point-LIO.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot="${GO2_ROBOT_IP:-192.168.8.245}"
user="${GO2_ROBOT_USER:-unitree}"
domain="${GO2_CAMERA_DOMAIN:-65}"
host="${GO2_HOST_IP:-$(ip -4 route get "$robot" | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1);exit}}')}"
[[ "$robot" =~ ^[0-9.]+$ && "$host" =~ ^[0-9.]+$ && "$user" =~ ^[a-zA-Z0-9_-]+$ && "$domain" =~ ^[0-9]+$ ]] || exit 2
(( domain <= 232 )) || exit 2
SSH=(ssh -o ConnectTimeout=8)
SCP=(scp)
if [[ -n "${GO2_SSH_CONTROL_PATH:-}" ]]; then
 SSH+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH");SCP+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
fi
target="$user@$robot"
case "${1:-help}" in
 start|start-preview|start-da3)
  mode="$1"
  calibration="${GO2_FUSION_CALIBRATION:-$ROOT/go2_nav2/config/camera_fusion_preview.json}"
  python3 - "$calibration" "$mode" <<'PY'
import json,sys
c=json.load(open(sys.argv[1]))
if sys.argv[2]!='start-preview' and not (c.get('calibration_verified') is True and c.get('timing_verified') is True):
 raise SystemExit('Verified camera calibration and timing are required. start-preview explicitly permits stationary nominal coloring only.')
PY
  "${SSH[@]}" "$target" 'mkdir -p ~/go2-pointlio-camera-fusion/work/camera_fusion ~/go2-pointlio-camera-fusion/work/da3'
  "${SCP[@]}" "$ROOT"/ws/scripts/camera_fusion/*.py "$target:go2-pointlio-camera-fusion/work/camera_fusion/"
  "${SCP[@]}" "$ROOT/ws/scripts/da3/live_worker.py" "$ROOT/ws/scripts/da3/benchmark.py" "$target:go2-pointlio-camera-fusion/work/da3/"
  "${SCP[@]}" "$calibration" "$target:go2-pointlio-camera-fusion/work/calibration.json"
  "${SSH[@]}" "$target" bash -s -- "$robot" "$host" "$domain" "$mode" <<'REMOTE'
set -euo pipefail
cd ~/go2-pointlio-camera-fusion
if docker inspect go2-camera-fusion >/dev/null 2>&1; then echo 'Fusion container exists; use status/stop before a new session.'; exit 1; fi
# Reuse the isolated image; no changes to the other agent's container or SDK.
docker image inspect go2-stock-camera:local >/dev/null
ipc="/dev/shm/go2-camera-fusion-$(id -u)"
mkdir -p "$ipc"
rm -f "$ipc"/*.npz "$ipc/status.json" "$ipc/save.request"
cp work/calibration.json "$ipc/calibration.json"
cat > "$ipc/native.xml" <<XML
<CycloneDDS><Domain Id="0"><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>
XML
cat > "$ipc/wifi.xml" <<XML
<CycloneDDS><Domain Id="$3"><General><Interfaces><NetworkInterface address="$1"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="$2"/><Peer address="$1"/></Peers></Discovery></Domain></CycloneDDS>
XML
preview=0;da3=0
[[ "$4" == start-preview ]] && preview=1
if [[ "$4" == start-da3 ]]; then
 da3=1
 python3 - <<'PY'
import hashlib,json
from pathlib import Path
p=Path.home()/'go2-stock-camera/da3/models'
assert hashlib.sha256((p/'da3-small-308-fp16.engine').read_bytes()).hexdigest()==json.loads((p/'trt-validated.json').read_text())['engine_sha256']
PY
fi
session="$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "data/$session"
ln -sfn "$session" data/latest
docker run -d --name go2-camera-fusion --network host --init --cpus 1.5 --memory 1200m \
 --stop-signal SIGINT --stop-timeout 25 --user "$(id -u):$(id -g)" \
 -e FUSION_DOMAIN="$3" -e FUSION_PREVIEW="$preview" -e FUSION_DA3="$da3" \
 -e ROS_LOG_DIR=/tmp/fusion-ros -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
 -v "$PWD/work:/work:ro" -v "$ipc:/ipc" -v "$PWD/data/$session:/data" \
 --entrypoint /bin/bash go2-stock-camera:local \
 -c 'source /opt/ros/humble/setup.bash && exec python3 -u /work/camera_fusion/supervise.py'
if [[ "$da3" == 1 ]]; then
 OPENBLAS_NUM_THREADS=1 nohup python3 -u work/camera_fusion/host_da3.py \
  --python "$HOME/go2-stock-camera/da3/venv/bin/python" --worker "$PWD/work/da3/live_worker.py" \
  --engine "$HOME/go2-stock-camera/da3/models/da3-small-308-fp16.engine" --ipc "$ipc" \
  > "data/$session/da3.log" 2>&1 < /dev/null &
fi
printf 'Fusion session: %s; preview=%s; DA3=%s. Point-LIO remains independently owned.\n' "$session" "$preview" "$da3"
REMOTE
  ;;
 check-projection)
  "${SCP[@]}" "$ROOT/ws/scripts/camera_fusion/check_projection.py" "$target:go2-pointlio-camera-fusion/work/camera_fusion/check_projection.py"
  "${SSH[@]}" "$target" 'docker exec go2-camera-fusion python3 /work/camera_fusion/check_projection.py /data/session'
  ;;
 status)
  "${SSH[@]}" "$target" 'docker ps -a --filter name=^/go2-camera-fusion$; cat "/dev/shm/go2-camera-fusion-$(id -u)/status.json" 2>/dev/null; docker logs --tail 8 go2-camera-fusion'
  ;;
 save)
  "${SSH[@]}" "$target" 'docker inspect --format "{{.State.Running}}" go2-camera-fusion | grep -qx true && touch "/dev/shm/go2-camera-fusion-$(id -u)/save.request"'
  ;;
 stop)
  "${SSH[@]}" "$target" 'docker stop go2-camera-fusion && docker logs go2-camera-fusion > ~/go2-pointlio-camera-fusion/data/latest/launch.log 2>&1 && docker rm go2-camera-fusion'
  ;;
 rviz)
  mkdir -p "$ROOT/ws/log"
  cat > "$ROOT/ws/log/camera-fusion-dds.xml" <<XML
<CycloneDDS><Domain Id="$domain"><General><Interfaces><NetworkInterface address="$host"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="$robot"/></Peers></Discovery></Domain></CycloneDDS>
XML
  frame="${GO2_FUSION_MAP_FRAME:-lidar3d_map}"
  [[ "$frame" =~ ^[a-zA-Z0-9_]+$ ]] || exit 2
  sed "s/lidar3d_map/$frame/g" "$ROOT/go2_nav2/rviz/camera-fusion.rviz" > "$ROOT/ws/log/camera-fusion.rviz"
  exec docker run --rm --name go2-camera-fusion-rviz --network host \
   -e DISPLAY -e XAUTHORITY=/tmp/fusion.xauth -e ROS_DOMAIN_ID="$domain" \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///tmp/dds.xml -e LIBGL_ALWAYS_SOFTWARE=1 \
   -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/fusion.xauth:ro" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
   -v "$ROOT/ws/log/camera-fusion-dds.xml:/tmp/dds.xml:ro" -v "$ROOT/ws/log/camera-fusion.rviz:/tmp/fusion.rviz:ro" \
   --entrypoint /bin/bash "${GO2_RVIZ_IMAGE:-go2-humble:local}" \
   -c 'source /opt/ros/humble/setup.bash && exec rviz2 -d /tmp/fusion.rviz'
  ;;
 *) echo 'Usage: bash camera-fusion.sh {start|start-preview|start-da3|check-projection|status|save|stop|rviz}' ;;
esac
