#!/usr/bin/env bash
# Laptop: one sensor-only mapping session. No robot motion commands.
set -euo pipefail
mapping_mode=mapping
relay_profile=default
if [[ "${1:-}" == --view ]]; then
  mapping_mode=sensors-viz
  shift
elif [[ "${1:-}" == --3d ]]; then
  mapping_mode=lidar3d
  relay_profile=lidar3d
  shift
fi
backend="${GO2_LIDAR3D_BACKEND:-pointlio}"
if [[ "$mapping_mode" == lidar3d && "${1:-}" == --legkilo ]]; then backend=legkilo; shift; fi
if [[ "$mapping_mode" == lidar3d && "${1:-}" == --pointlio-leg ]]; then backend=pointlio_leg; shift; fi
compute="${GO2_LIDAR3D_COMPUTE:-jetson}"
if [[ "$mapping_mode" == lidar3d && "${1:-}" == --laptop ]]; then
  compute=laptop
  shift
fi
planning=0
if [[ "$mapping_mode" == lidar3d && "${1:-}" == --plan ]]; then planning=1; shift; fi
navigation=0
if [[ "$mapping_mode" == lidar3d && "${1:-}" == --nav ]]; then navigation=1; planning=1; shift; fi
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: ./mapping.sh [--view | --3d [--legkilo|--pointlio-leg|--laptop|--plan|--nav]]  (close RViz or press Ctrl+C to stop)'
  echo '--view shows the live 3D LiDAR cloud and front camera without SLAM or map output.'
  echo '--pointlio-leg selects the separately built experimental L1-gyro/leg Point-LIO.'
  echo '--nav enables supervised level-floor driving from RViz goals; stand the robot first.'
  echo '3D computes on Jetson by default; --laptop preserves the Wi-Fi raw-sensor experiment.'
  echo 'Optional: GO2_CONTAINER=go2-humble GO2_ROBOT_USER=unitree ./mapping.sh'
  echo 'GO2_RVIZ=0 disables the window; 3D sessions save automatically on stop.'
  echo 'GO2_RECORD=1 records 3D backend inputs for offline diagnosis.'
  exit 0
fi
[[ $# == 0 ]] || { echo 'Usage: ./mapping.sh [--view | --3d [--legkilo|--pointlio-leg|--laptop|--plan|--nav]]' >&2; exit 2; }
[[ "$compute" == jetson || "$compute" == laptop ]] || { echo 'GO2_LIDAR3D_COMPUTE must be jetson or laptop' >&2; exit 2; }
[[ "$backend" == pointlio || "$backend" == legkilo || "$backend" == pointlio_leg ]] || { echo "Unknown 3D backend" >&2; exit 2; }
[[ "$backend" != legkilo || "$compute" == jetson ]] || { echo "Leg-KILO launcher currently supports Jetson; replay is available through go2-session." >&2; exit 2; }
[[ "$backend" != pointlio_leg || "$compute" == jetson ]] || { echo 'Point-LIO leg inputs require Jetson DDS; use isolated bag replay for laptop tests.' >&2; exit 2; }
rviz="${GO2_RVIZ:-1}"
if [[ "$mapping_mode" == sensors-viz ]]; then rviz=1; fi
if [[ "$planning" == 1 ]]; then
  [[ "$backend" == pointlio && "$compute" == jetson && "$rviz" == 1 ]] || {
    echo '--plan previews Point-LIO planning with Jetson SLAM and laptop RViz.' >&2; exit 2;
  }
fi
record=0
[[ "${GO2_RECORD:-0}" != 1 ]] || record=1
[[ "$rviz" == 0 || "$rviz" == 1 ]] || { echo 'GO2_RVIZ must be 0 or 1' >&2; exit 2; }
if [[ "$rviz" == 1 ]]; then
  : "${DISPLAY:?Run this command in a terminal on the laptop desktop}"
fi
MAPPING_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${GO2_CONTAINER:-}" ]]; then
  candidates=(go2-nav2-live-test go2-humble)
  [[ "$mapping_mode" != sensors-viz ]] || candidates=(go2-humble go2-nav2-live-test)
  [[ "$mapping_mode" != lidar3d ]] || candidates=(go2-lidar3d "${candidates[@]}")
  # Prefer the prepared 3D environment even if a generic container is running.
  if [[ "$mapping_mode" == lidar3d ]] && docker inspect go2-lidar3d >/dev/null 2>&1; then
    docker start go2-lidar3d >/dev/null
    GO2_CONTAINER=go2-lidar3d
  fi
  for candidate in "${candidates[@]}"; do
    [[ -z "${GO2_CONTAINER:-}" ]] || break
    if [[ "$(docker inspect -f '{{.State.Running}}' "$candidate" 2>/dev/null)" == true ]]; then
      GO2_CONTAINER="$candidate"
      break
    fi
  done
  if [[ -z "${GO2_CONTAINER:-}" ]]; then
    for candidate in "${candidates[@]}"; do
      if docker inspect "$candidate" >/dev/null 2>&1; then
        docker start "$candidate" >/dev/null
        GO2_CONTAINER="$candidate"
        break
      fi
    done
  fi
fi
: "${GO2_CONTAINER:?Start the project Docker container first}"
if [[ "$mapping_mode" == lidar3d && "$rviz" == 1 ]]; then
  docker exec "$GO2_CONTAINER" bash -c '
    source /ws/scripts/go2-env.sh || exit
    python3 -c "from unitree_go.msg import LowState; from scipy.spatial.transform import Rotation"
  ' || { echo "3D visualization dependencies missing in $GO2_CONTAINER (unitree_go / scipy)." >&2; exit 1; }
fi
if [[ "$navigation" == 1 ]]; then
  docker exec "$GO2_CONTAINER" test -f /ws/install/go2_rviz_controls/lib/libgo2_rviz_controls.so ||
    docker exec "$GO2_CONTAINER" bash /ws/scripts/build-rviz-controls.sh
fi
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
route_host_ip="$(ip -4 route get "$robot_ip" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
: "${route_host_ip:?No IPv4 route to $robot_ip}"
if [[ "$robot_host_ip" != "$route_host_ip" ]]; then
  echo "GO2_HOST_IP $robot_host_ip is stale; using route source $route_host_ip for $robot_ip."
  robot_host_ip="$route_host_ip"
fi
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
jetson_started=0
motion_pid=''
session_mode="$mapping_mode"
if [[ "$mapping_mode" == lidar3d && "$compute" == jetson ]]; then session_mode=lidar3d-viz; fi
x_added=0
cleanup() {
  trap '' INT TERM HUP
  if [[ -n "$motion_pid" ]]; then
    kill -TERM "$motion_pid" 2>/dev/null || true
    wait "$motion_pid" 2>/dev/null || true
  fi
  if [[ "$mapping_started" == 1 ]]; then
    docker exec "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh stop "$session_mode" --owner "$owner" >/dev/null 2>&1 || true
  fi
  if [[ "$jetson_started" == 1 ]]; then
    ssh -o BatchMode=yes -S "$ssh_socket" "$target" "docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d --owner $owner" >/dev/null 2>&1 || true
    saved_run="$(sed -n 's@^3D result directory: /ws/maps/lidar3d/\(run-[a-zA-Z0-9_-]*\).*@\1@p' "$log_dir/slam.log" | head -1)"
    if [[ -n "$saved_run" ]] && grep -q '^Result validation failed' "$log_dir/slam.log"; then
      echo "3D session produced no complete map; partial result remains on Jetson: $saved_run" >&2
    elif [[ -n "$saved_run" ]]; then
      python3 "$MAPPING_ROOT/ws/scripts/fetch-lidar3d.py" --target "$target" --ssh-socket "$ssh_socket" \
        --run "$saved_run" --destination "$MAPPING_ROOT/ws/maps/lidar3d" || \
        echo "Transfer incomplete: results remain on Jetson; see $log_dir/slam.log" >&2
    fi
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
if [[ "$mapping_mode" == lidar3d && "$backend" == pointlio ]]; then
  echo "Keep the robot still until the first 3D map appears; initialization takes a few seconds."
fi
ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 -S "$ssh_socket" "$target" true
if [[ "$mapping_mode" == lidar3d && "$compute" == jetson ]]; then
  if [[ "$backend" == pointlio_leg ]]; then
    ssh -o BatchMode=yes -S "$ssh_socket" "$target" \
      'docker exec go2-lidar3d-onboard test -f /ws/pointlio-leg/install/local_setup.bash' || {
      echo 'Build the experimental Point-LIO leg install on Jetson first (ws/scripts/build-pointlio-leg.sh).' >&2
      exit 1
    }
  fi
  # Isolated experiment checkout; no changes to the robot's main repository.
  tar -czf "$log_dir/runtime.tar.gz" -C "$MAPPING_ROOT" go2_nav2 \
    ws/scripts/go2-session.sh ws/scripts/go2_session.py ws/scripts/session_preflight.py \
    ws/scripts/export-lidar3d.py ws/scripts/robot-relay-wifi.sh ws/scripts/robot_relay_wifi.py
  scp -q -o "ControlPath=$ssh_socket" "$log_dir/runtime.tar.gz" "$target:go2-nav2-lidar3d-onboard/runtime.tar.gz"
  ssh -S "$ssh_socket" "$target" 'cd ~/go2-nav2-lidar3d-onboard && tar xzf runtime.tar.gz && rm runtime.tar.gz && docker start go2-lidar3d-onboard >/dev/null'
  jetson_started=1
  ssh -tt -o BatchMode=yes -o ServerAliveInterval=3 -o ServerAliveCountMax=3 -S "$ssh_socket" "$target" \
    "exec docker exec -e GO2_LIDAR3D_BACKEND=$backend -e GO2_LIDAR3D_COMPUTE=jetson -e GO2_HOST_IP=$robot_host_ip -e GO2_RELAY_DOMAIN_ID=$relay_domain -e GO2_RECORD=$record go2-lidar3d-onboard bash /ws/scripts/go2-session.sh start lidar3d --owner $owner" \
    >"$log_dir/slam.log" 2>&1 &
  mapping_pid=$!
  echo "SLAM: Jetson; RViz: laptop. Logs: $log_dir"
  echo "On stop: results move to laptop ws/maps/lidar3d/ after checksum verification."
  if [[ "$rviz" == 1 ]]; then
    if ! xhost | grep -Fxq "SI:localuser:$xuser"; then
      xhost "+SI:localuser:$xuser" >/dev/null
      x_added=1
    fi
    mapping_started=1
    if [[ "$navigation" == 1 ]]; then
      scp -q -o "ControlPath=$ssh_socket" "$MAPPING_ROOT/ws/scripts/robot-motion-wifi.sh" \
        "$MAPPING_ROOT/ws/scripts/robot_sport_bridge.py" "$MAPPING_ROOT/ws/scripts/go2_cmd_vel_tcp.py" \
        "$MAPPING_ROOT/ws/scripts/robot-source-unitree-ros.sh" "$target:go2-nav2-lidar3d-onboard/"
      ssh -tt -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=2 -S "$ssh_socket" "$target" \
        'exec bash ~/go2-nav2-lidar3d-onboard/robot-motion-wifi.sh' >"$log_dir/motion.log" 2>&1 &
      motion_pid=$!
    fi
    docker exec -e GO2_HOST_IP="$robot_host_ip" -e GO2_ROBOT_IP="$robot_ip" -e GO2_RELAY_DOMAIN_ID="$relay_domain" \
      -e GO2_LIDAR3D_NAV="$navigation" -e GO2_LIDAR3D_PLAN="$planning" -e GO2_LIDAR3D_BACKEND="$backend" -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 -e XDG_CONFIG_HOME=/tmp/go2-mapping-config \
      "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh start lidar3d-viz --owner "$owner" \
      >"$log_dir/rviz.log" 2>&1 &
    viz_pid=$!
    set +e
    if [[ -n "$motion_pid" ]]; then
      wait -n "$mapping_pid" "$viz_pid" "$motion_pid"
    else
      wait -n "$mapping_pid" "$viz_pid"
    fi
  else
    set +e
    wait "$mapping_pid"
  fi
  result=$?
  set -e
  if [[ "$result" != 0 ]]; then tail -n 25 "$log_dir/slam.log" "$log_dir/rviz.log" 2>/dev/null >&2; fi
  exit "$result"
fi
relay_script='~/robot-relay-wifi.sh'
relay_trace_env=''
relay_camera=0
if [[ "$mapping_mode" == sensors-viz ]]; then relay_camera=1; fi
if [[ "$mapping_mode" == lidar3d ]]; then
  # Deploy only the three sensor-relay files; preserve the robot's main checkout.
  ssh -S "$ssh_socket" "$target" 'mkdir -p ~/go2-nav2-lidar3d-relay'
  scp -q -o "ControlPath=$ssh_socket" \
    "$MAPPING_ROOT/ws/scripts/robot-relay-wifi.sh" \
    "$MAPPING_ROOT/ws/scripts/robot_relay_wifi.py" \
    "$MAPPING_ROOT/ws/scripts/robot-source-unitree-ros.sh" \
    "$target:go2-nav2-lidar3d-relay/"
  relay_script='~/go2-nav2-lidar3d-relay/robot-relay-wifi.sh'
  if [[ "${GO2_TRACE:-0}" == 1 ]]; then
    relay_trace_env="GO2_RELAY_TRACE_PREFIX=/tmp/go2-$owner"
    echo "Source trace after stop: $target:/tmp/go2-$owner-source.jsonl"
  fi
fi
ssh -tt -o BatchMode=yes -o ServerAliveInterval=3 -o ServerAliveCountMax=3 -S "$ssh_socket" "$target" \
  "exec env GO2_HOST_IP=$robot_host_ip GO2_RELAY_DOMAIN_ID=$relay_domain GO2_RELAY_PROFILE=$relay_profile GO2_RELAY_CAMERA=$relay_camera $relay_trace_env bash $relay_script --sensors-only" \
  >"$log_dir/relay.log" 2>&1 &
relay_pid=$!
for ((attempt=0; attempt<300; attempt++)); do
  if grep -q 'Relay running:' "$log_dir/relay.log"; then break; fi
  if ! kill -0 "$relay_pid" 2>/dev/null; then cat "$log_dir/relay.log" >&2; exit 1; fi
  sleep .1
done
grep -q 'Relay running:' "$log_dir/relay.log" || { cat "$log_dir/relay.log" >&2; exit 1; }
if [[ "$rviz" == 1 ]] && ! xhost | grep -Fxq "SI:localuser:$xuser"; then
  xhost "+SI:localuser:$xuser" >/dev/null
  x_added=1
fi
if [[ "$mapping_mode" == sensors-viz ]]; then
  echo "Live LiDAR and camera view. Close RViz or Ctrl+C to stop. Logs: $log_dir"
else
  echo "Sensor-only $mapping_mode. Close RViz or Ctrl+C to stop. Logs: $log_dir"
fi
mapping_started=1
session_args=(start "$mapping_mode")
[[ "$rviz" == 0 || "$mapping_mode" == sensors-viz ]] || session_args+=(--rviz)
session_args+=(--owner "$owner")
docker exec -e DISPLAY="${DISPLAY:-}" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_CONFIG_HOME=/tmp/go2-mapping-config \
  -e GO2_HOST_IP="$robot_host_ip" -e GO2_ROBOT_IP="$robot_ip" -e GO2_RELAY_DOMAIN_ID="$relay_domain" \
  -e GO2_RECORD="${GO2_RECORD:-0}" \
  -e GO2_LIDAR3D_COMPUTE=laptop \
  "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh "${session_args[@]}" \
  >"$log_dir/slam.log" 2>&1 &
mapping_pid=$!
set +e
wait -n "$relay_pid" "$mapping_pid"
result=$?
set -e
if [[ "$result" != 0 ]]; then tail -n 25 "$log_dir/relay.log" "$log_dir/slam.log" >&2; fi
exit "$result"
