#!/usr/bin/env bash
# Laptop: one sensor-only mapping session. No robot motion commands.
set -euo pipefail
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: ./mapping.sh  (close RViz or press Ctrl+C to stop)'
  echo 'Optional: GO2_CONTAINER=go2-humble GO2_ROBOT_USER=unitree ./mapping.sh'
  exit 0
fi
[[ $# == 0 ]] || { echo 'Usage: ./mapping.sh' >&2; exit 2; }
: "${DISPLAY:?Run this command in a terminal on the laptop desktop}"
MAPPING_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${GO2_CONTAINER:-}" ]]; then
  for candidate in go2-nav2-live-test go2-humble; do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$candidate" 2>/dev/null)" == true ]]; then
      GO2_CONTAINER="$candidate"
      break
    fi
  done
fi
: "${GO2_CONTAINER:?Start the project Docker container first}"
container_env="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$GO2_CONTAINER")"
while IFS='=' read -r key value; do
  case "$key" in
    GO2_HOST_IP) robot_host_ip="$value" ;;
    GO2_ROBOT_IP) robot_ip="$value" ;;
    GO2_RELAY_DOMAIN_ID) relay_domain="$value" ;;
    GO2_NET) robot_net="$value" ;;
  esac
done <<< "$container_env"
[[ "${robot_net:-wifi}" == wifi ]] || { echo 'This launcher is for Wi-Fi.' >&2; exit 1; }
: "${robot_host_ip:?Set GO2_HOST_IP in the container environment}"
: "${robot_ip:?Set GO2_ROBOT_IP in the container environment}"
relay_domain="${relay_domain:-64}"
robot_user="${GO2_ROBOT_USER:-unitree}"
[[ "$robot_host_ip" =~ ^[a-zA-Z0-9_.:-]+$ && "$robot_ip" =~ ^[a-zA-Z0-9_.:-]+$ &&
   "$robot_user" =~ ^[a-zA-Z0-9_-]+$ && "$relay_domain" =~ ^[0-9]{1,3}$ ]] ||
  { echo 'Invalid robot address, username or DDS domain.' >&2; exit 2; }
container_uid="$(docker exec "$GO2_CONTAINER" id -u)"
xuser="$(getent passwd "$container_uid" | cut -d: -f1)"
: "${xuser:?Container UID must have a local host account for X11}"
mkdir -p "$MAPPING_ROOT/ws/log"
log_dir="$(mktemp -d "$MAPPING_ROOT/ws/log/mapping.XXXXXX")"
owner="${log_dir##*/}"
ssh_socket="$log_dir/ssh"
target="$robot_user@$robot_ip"
mapping_started=0
x_added=0
cleanup() {
  trap '' INT TERM HUP
  if [[ "$mapping_started" == 1 ]]; then
    docker exec "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh stop mapping --owner "$owner" >/dev/null 2>&1 || true
  fi
  # Private SSH master: closes only the remote relay started by this invocation.
  ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true
  [[ "$x_added" == 0 ]] || xhost "-SI:localuser:$xuser" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
echo "Connecting to $target (SSH may ask for your robot password)..."
ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 -S "$ssh_socket" "$target" true
ssh -tt -o BatchMode=yes -o ServerAliveInterval=3 -o ServerAliveCountMax=3 -S "$ssh_socket" "$target" \
  "exec env GO2_HOST_IP=$robot_host_ip GO2_RELAY_DOMAIN_ID=$relay_domain GO2_RELAY_CAMERA=0 bash ~/robot-relay-wifi.sh --sensors-only" \
  >"$log_dir/relay.log" 2>&1 &
relay_pid=$!
for ((attempt=0; attempt<300; attempt++)); do
  if grep -q 'Relay running:' "$log_dir/relay.log"; then break; fi
  if ! kill -0 "$relay_pid" 2>/dev/null; then cat "$log_dir/relay.log" >&2; exit 1; fi
  sleep .1
done
grep -q 'Relay running:' "$log_dir/relay.log" || { cat "$log_dir/relay.log" >&2; exit 1; }
if ! xhost | grep -Fxq "SI:localuser:$xuser"; then
  xhost "+SI:localuser:$xuser" >/dev/null
  x_added=1
fi
echo "Sensor-only mapping + RViz. Close RViz or Ctrl+C to stop. Logs: $log_dir"
mapping_started=1
docker exec -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_CONFIG_HOME=/tmp/go2-mapping-config \
  "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh start mapping --rviz --owner "$owner" \
  >"$log_dir/slam.log" 2>&1 &
mapping_pid=$!
set +e
wait -n "$relay_pid" "$mapping_pid"
result=$?
set -e
if [[ "$result" != 0 ]]; then tail -n 25 "$log_dir/relay.log" "$log_dir/slam.log" >&2; fi
exit "$result"
