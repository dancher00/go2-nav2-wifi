#!/usr/bin/env bash
# Isolated DA3 experiment; never controls the LiDAR or camera services.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot="${GO2_ROBOT_IP:-192.168.8.245}"
user="${GO2_ROBOT_USER:-unitree}"
domain="${GO2_CAMERA_DOMAIN:-65}"
[[ "$robot" =~ ^[0-9.]+$ && "$user" =~ ^[a-zA-Z0-9_-]+$ && "$domain" =~ ^[0-9]+$ ]] || exit 2
SSH=(ssh -o ConnectTimeout=8)
SCP=(scp)
if [[ -n "${GO2_SSH_CONTROL_PATH:-}" ]]; then
  SSH+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
  SCP+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
fi
target="$user@$robot"
case "${1:-help}" in
  start)
    "${SSH[@]}" "$target" 'mkdir -p ~/go2-stock-camera/da3'
    "${SCP[@]}" "$ROOT"/ws/scripts/da3/*.py "$target:go2-stock-camera/da3/"
    "${SSH[@]}" "$target" bash -s -- "$domain" <<'REMOTE'
set -euo pipefail
cd ~/go2-stock-camera/da3
[[ -f models/model.onnx && -x venv/bin/python ]] || { echo 'Prepare the model/venv as documented in docs/DA3-ON-JETSON.md'; exit 1; }
if docker inspect go2-da3-preview >/dev/null 2>&1; then echo 'DA3 preview already exists. Use status/stop.'; exit 1; fi
if [[ -f worker.pid ]] && kill -0 "$(cat worker.pid)" 2>/dev/null; then echo 'Worker PID exists. Inspect it before restart.'; exit 1; fi
ipc="/dev/shm/go2-da3-$(id -u)"
mkdir -p "$ipc"
rm -f "$ipc/input.npz" "$ipc/output.npz"
backend=ort
model=models/model.onnx
if [[ -f models/trt-validated.json && -f models/da3-small-308-fp16.engine ]]; then
  venv/bin/python - <<'PY'
import hashlib,json
from pathlib import Path
assert hashlib.sha256(Path('models/da3-small-308-fp16.engine').read_bytes()).hexdigest()==json.loads(Path('models/trt-validated.json').read_text())['engine_sha256'], 'Engine changed since validation'
PY
  backend=trt
  model=models/da3-small-308-fp16.engine
fi
session="$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "live/$session"
mkdir -p records
ln -sfn "$session" live/latest
OPENBLAS_NUM_THREADS=1 nohup nice -n 15 venv/bin/python -u live_worker.py --ipc "$ipc" --backend "$backend" --model "$model" --duration 1800 > "live/$session/worker.log" 2>&1 < /dev/null &
echo $! > worker.pid
trap 'kill "$(cat worker.pid)" 2>/dev/null || true' ERR
docker run -d --name go2-da3-preview --network host --init --cpus 1 --memory 384m \
  --stop-signal SIGINT --user "$(id -u):$(id -g)" \
  -e ROS_DOMAIN_ID="$1" -e ROS_LOG_DIR=/tmp/da3-ros \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///work/cyclonedds.xml \
  -v "$HOME/go2-stock-camera:/work:ro" -v "$ipc:/ipc" -v "$PWD/records:/records" \
  --entrypoint /bin/bash go2-stock-camera:local \
  -c 'source /opt/ros/humble/setup.bash && exec python3 /work/da3/ros_preview.py --ipc /ipc --records /records'
printf 'DA3 preview backend=%s; worker limited to 30 minutes; relative coordinates only.\n' "$backend"
REMOTE
    ;;
  record)
    "${SSH[@]}" "$target" 'docker inspect --format "{{.State.Running}}" go2-da3-preview | grep -qx true && touch "/dev/shm/go2-da3-$(id -u)/record.request"'
    echo 'Recording camera keyframes for up to 120 seconds / 256 MiB on Jetson.'
    ;;
  scene)
    scene="${2:-walk-return-16}"
    [[ "$scene" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
    "${SCP[@]}" "$ROOT/ws/scripts/da3/publish_scene.py" "$target:go2-stock-camera/da3/"
    "${SSH[@]}" "$target" bash -s -- "$domain" "$scene" <<'REMOTE'
set -euo pipefail
cd ~/go2-stock-camera
[[ -f "da3/results/$2/relative_scene.ply" ]]
docker run -d --name go2-da3-scene --network host --init --cpus 1 --memory 512m \
  --stop-signal SIGINT --user "$(id -u):$(id -g)" \
  -e ROS_DOMAIN_ID="$1" -e ROS_LOG_DIR=/tmp/da3-scene-ros \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///work/cyclonedds.xml \
  -v "$PWD:/work:ro" --entrypoint /bin/bash go2-stock-camera:local \
  -c 'source /opt/ros/humble/setup.bash && exec python3 /work/da3/publish_scene.py "$1"' bash "/work/da3/results/$2"
REMOTE
    ;;
  stop-scene)
    "${SSH[@]}" "$target" 'docker stop go2-da3-scene && docker rm go2-da3-scene'
    ;;
  status)
    "${SSH[@]}" "$target" 'docker ps -a --filter name=^/go2-da3-preview$; tail -4 ~/go2-stock-camera/da3/live/latest/worker.log; docker logs --tail 5 go2-da3-preview'
    ;;
  stop)
    "${SSH[@]}" "$target" bash -s <<'REMOTE'
set -euo pipefail
cd ~/go2-stock-camera/da3
if [[ -f worker.pid ]]; then
  pid="$(cat worker.pid)"
  if [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]]; then
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
    [[ "$cmd" == *'python -u live_worker.py --ipc /dev/shm/go2-da3-'* ]] || { echo 'PID is not our worker; refusing to signal it'; exit 1; }
    kill "$pid"
  fi
  rm worker.pid
fi
if docker inspect go2-da3-preview >/dev/null 2>&1; then
  docker stop go2-da3-preview
  docker logs go2-da3-preview > live/latest/ros.log 2>&1
  docker rm go2-da3-preview
fi
REMOTE
    ;;
  rviz|rviz-scene)
    config=da3-preview
    container=go2-da3-rviz
    if [[ "$1" == rviz-scene ]]; then config=da3-scene; container=go2-da3-scene-rviz; fi
    host="${GO2_HOST_IP:-$(ip -4 route get "$robot" | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1);exit}}')}"
    [[ "$host" =~ ^[0-9.]+$ ]] || exit 2
    mkdir -p "$ROOT/ws/log"
    cat > "$ROOT/ws/log/da3-dds.xml" <<XML
<CycloneDDS><Domain Id="$domain"><General><Interfaces><NetworkInterface address="$host"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="$robot"/></Peers></Discovery></Domain></CycloneDDS>
XML
    exec docker run --rm --name "$container" --network host \
      -e DISPLAY -e XAUTHORITY=/tmp/da3.xauth -e ROS_DOMAIN_ID="$domain" \
      -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///tmp/dds.xml \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/da3.xauth:ro" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v "$ROOT/ws/log/da3-dds.xml:/tmp/dds.xml:ro" \
      -v "$ROOT/go2_nav2/rviz/$config.rviz:/tmp/da3.rviz:ro" \
      --entrypoint /bin/bash "${GO2_RVIZ_IMAGE:-go2-humble:local}" \
      -c 'source /opt/ros/humble/setup.bash && exec rviz2 -d /tmp/da3.rviz'
    ;;
  *) echo 'Usage: bash da3-preview.sh {start|status|stop|rviz|record|scene [result-name]|rviz-scene|stop-scene}' ;;
esac
