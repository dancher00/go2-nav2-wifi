#!/usr/bin/env bash
# Optional stock-camera adapter. Never starts/stops the LiDAR backend.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_ip="${GO2_ROBOT_IP:-192.168.8.245}"
host_ip="${GO2_HOST_IP:-$(ip -4 route get "$robot_ip" | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1);exit}}')}"
robot_user="${GO2_ROBOT_USER:-unitree}"
domain="${GO2_CAMERA_DOMAIN:-65}"
[[ "$robot_ip" =~ ^[0-9.]+$ && "$host_ip" =~ ^[0-9.]+$ && "$robot_user" =~ ^[a-zA-Z0-9_-]+$ && "$domain" =~ ^[0-9]+$ ]] || exit 2
(( domain <= 232 )) || exit 2
SSH=(ssh -o ConnectTimeout=8)
SCP=(scp)
if [[ -n "${GO2_SSH_CONTROL_PATH:-}" ]]; then
  SSH+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
  SCP+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
else
  SSH+=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$HOME/.ssh/stock-camera-%C")
  SCP+=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$HOME/.ssh/stock-camera-%C")
fi
target="$robot_user@$robot_ip"
case "${1:-help}" in
  start)
    "${SSH[@]}" "$target" 'mkdir -p ~/go2-stock-camera'
    "${SCP[@]}" "$ROOT/ws/scripts/stock_camera.py" "$target:go2-stock-camera/stock_camera.py"
    "${SCP[@]}" "$ROOT/docker/Dockerfile.stock-camera" "$target:go2-stock-camera/Dockerfile"
    "${SSH[@]}" "$target" bash -s -- "$robot_ip" "$host_ip" "$domain" <<'REMOTE'
set -euo pipefail
cd ~/go2-stock-camera
if docker inspect go2-stock-camera >/dev/null 2>&1; then
  echo 'Stock-camera container exists; inspect status or stop it explicitly.' >&2
  exit 1
fi
cli=''
for candidate in "$HOME/bin/go2_camera_jpeg_cli" "$HOME/.go2_camera_build/out/go2_camera_jpeg_cli" "$HOME/go2_camera_jpeg_cli/go2_camera_jpeg_cli"; do
  if [[ -x "$candidate" && -f "$candidate" ]]; then cli="$candidate"; break; fi
done
[[ -n "$cli" ]] || { echo 'Build the existing project camera CLI on Jetson first.' >&2; exit 1; }
sdk_lib="$HOME/unitree_sdk2/thirdparty/lib/$(uname -m)"
[[ -d "$sdk_lib" ]] || { echo 'Unitree SDK library directory missing' >&2; exit 1; }
docker image inspect go2-humble:local >/dev/null
if ! docker image inspect go2-stock-camera:local >/dev/null 2>&1; then
  docker build -t go2-stock-camera:local - < Dockerfile
fi
cat > cyclonedds.xml <<XML
<CycloneDDS><Domain Id="$3"><General><Interfaces><NetworkInterface address="$1"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="$2"/><Peer address="$1"/></Peers></Discovery></Domain></CycloneDDS>
XML
session="$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "data/$session"
ln -sfn "$session" data/latest
docker run -d --name go2-stock-camera --network host --init --cpus 1 --memory 512m \
  --stop-signal SIGINT --stop-timeout 10 --user "$(id -u):$(id -g)" \
  -e ROS_LOG_DIR=/tmp/stock-camera-ros -e ROS_DOMAIN_ID="$3" \
  -e GO2_CAMERA_SDK_LIB="$sdk_lib" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///work/cyclonedds.xml \
  -v "$PWD:/work:ro" -v "$PWD/data/$session:/data" -v "$cli:/opt/stock-reader:ro" \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -v "$sdk_lib:$sdk_lib:ro" --entrypoint /bin/bash go2-stock-camera:local \
  -c 'source /opt/ros/humble/setup.bash && exec python3 /work/stock_camera.py --cli /opt/stock-reader --interface eth0 --record-dir /data/capture --record-seconds 30'
printf 'Session: %s; camera DDS domain: %s; capture limited to 30 seconds.\n' "$session" "$3"
REMOTE
    ;;
  status) "${SSH[@]}" "$target" 'docker ps -a --filter name=^/go2-stock-camera$; docker logs --tail 8 go2-stock-camera' ;;
  stop) "${SSH[@]}" "$target" 'docker stop go2-stock-camera && docker logs go2-stock-camera > ~/go2-stock-camera/data/latest/launch.log 2>&1 && docker rm go2-stock-camera' ;;
  *) echo 'Usage: bash stock-camera.sh {start|status|stop}. Default output DDS domain 65; GO2_CAMERA_DOMAIN selects another domain.' ;;
esac
