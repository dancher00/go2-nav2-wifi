#!/usr/bin/env bash
# Laptop controller: all sensor/SLAM processes run on the Jetson.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_ip="${GO2_ROBOT_IP:-192.168.8.245}"
robot_user="${GO2_ROBOT_USER:-unitree}"
host_ip="${GO2_HOST_IP:-$(ip -4 route get "$robot_ip" | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1);exit}}')}"
for address in "$robot_ip" "$host_ip"; do
    [[ "$address" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'IPv4 addresses required' >&2; exit 2; }
done
[[ "$robot_user" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
SSH=(ssh -o ConnectTimeout=10)
SCP=(scp)
if [[ -n "${GO2_SSH_CONTROL_PATH:-}" ]]; then
    SSH+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
    SCP+=(-o "ControlPath=$GO2_SSH_CONTROL_PATH")
else
    # Reuse a short-lived connection so setup prompts for a password only once.
    SSH+=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$HOME/.ssh/d435i-%C")
    SCP+=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$HOME/.ssh/d435i-%C")
fi
target="$robot_user@$robot_ip"
write_dds() {
    cat <<XML
<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="65"><General>
<Interfaces><NetworkInterface address="$1"/></Interfaces><AllowMulticast>false</AllowMulticast>
</General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
<Peers><Peer address="$2"/><Peer address="$1"/></Peers></Discovery></Domain></CycloneDDS>
XML
}
case "${1:-help}" in
    setup)
        stage="$(mktemp -d)"
        trap 'rm -rf "$stage"' EXIT
        write_dds "$robot_ip" "$host_ip" | sed 's@</CycloneDDS>@<Domain Id="0"><General><Interfaces><NetworkInterface name="eth0"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>@' > "$stage/cyclonedds.xml"
        tar -czf "$stage/runtime.tar.gz" -C "$ROOT" \
            docker/Dockerfile.d435i docker/Dockerfile.d435i.dockerignore \
            go2_nav2/launch/d435i_mapping.launch.py go2_nav2/go2_nav2/sensor_time.py \
            go2_nav2/config/d435i_camera.yaml \
            ws/scripts/check-d435i.py ws/scripts/d435i_go2_odom.py
        "${SSH[@]}" "$target" 'mkdir -p ~/go2-d435i-visual-slam'
        "${SCP[@]}" "$stage/runtime.tar.gz" "$stage/cyclonedds.xml" "$target:go2-d435i-visual-slam/"
        "${SSH[@]}" "$target" 'cd ~/go2-d435i-visual-slam && tar xzf runtime.tar.gz && rm runtime.tar.gz && docker build -f docker/Dockerfile.d435i -t go2-d435i:local .'
        ;;
    start)
        launch_args=("${@:2}")
        for arg in "${launch_args[@]}"; do
            [[ "$arg" =~ ^camera_[xyz]:=[+-]?[0-9]+([.][0-9]+)?$ || "$arg" =~ ^odom_source:=(go2|rgbd)$ ]] || { echo "Invalid launch option: $arg" >&2; exit 2; }
        done
        "${SSH[@]}" "$target" bash -s -- "${launch_args[@]}" <<'REMOTE'
set -euo pipefail
cd ~/go2-d435i-visual-slam
if docker inspect go2-d435i-onboard >/dev/null 2>&1; then
    echo 'D435i container already exists. Use status, or stop before a new session.' >&2
    exit 1
fi
session="$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "data/$session"
ln -sfn "$session" data/latest
devices=()
for device in /dev/video* /dev/hidraw*; do
    [[ -e "$device" ]] && devices+=(--device "$device")
done
for device in /dev/iio:device*; do
    [[ -e "$device" ]] || continue
    major=$((16#$(stat -c %t "$device")))
    devices+=(--mount "type=bind,source=$device,target=$device" --device-cgroup-rule "c $major:* rmw")
done
# Native librealsense configures the D435i IIO triggers through sysfs.
# Bind only RealSense USB device subtrees writable, never all of /sys.
for usb in /sys/bus/usb/devices/*; do
    [[ -f "$usb/idVendor" && -f "$usb/idProduct" ]] || continue
    if [[ "$(cat "$usb/idVendor")" == 8086 && "$(cat "$usb/idProduct")" == 0b3a ]]; then
        camera_sys="$(readlink -f "$usb")"
        devices+=(-v "$camera_sys:$camera_sys:rw")
    fi
done
docker run -d --name go2-d435i-onboard --network host --init \
    --stop-signal SIGINT --stop-timeout 45 \
    --device-cgroup-rule 'c 189:* rmw' \
    -v /dev/bus/usb:/dev/bus/usb \
    "${devices[@]}" \
    -e ROS_DOMAIN_ID=65 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///work/cyclonedds.xml \
    -v "$PWD:/work:ro" -v "$PWD/data/$session:/data" \
    go2-d435i:local bash -c 'source /opt/ros/humble/setup.bash && exec ros2 launch /work/go2_nav2/launch/d435i_mapping.launch.py "$@"' bash "$@"
printf 'Session: %s\n' "$session"
REMOTE
        ;;
    stop)
        "${SSH[@]}" "$target" 'docker stop -t 45 go2-d435i-onboard && docker logs go2-d435i-onboard > ~/go2-d435i-visual-slam/data/latest/launch.log 2>&1 && docker rm go2-d435i-onboard'
        ;;
    status)
        "${SSH[@]}" "$target" 'docker ps -a --filter name=^/go2-d435i-onboard$; docker logs --tail 30 go2-d435i-onboard'
        ;;
    check)
        "${SSH[@]}" "$target" 'docker exec go2-d435i-onboard bash -c "source /opt/ros/humble/setup.bash && python3 /work/ws/scripts/check-d435i.py"'
        ;;
    rviz)
        mkdir -p "$ROOT/ws/log"
        write_dds "$host_ip" "$robot_ip" > "$ROOT/ws/log/d435i-dds.xml"
        exec docker run --rm --name go2-d435i-rviz --network host \
            -e DISPLAY -e XAUTHORITY=/tmp/d435i.xauth \
            -e ROS_DOMAIN_ID=65 -e ROS_LOCALHOST_ONLY=0 \
            -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            -e CYCLONEDDS_URI=file:///tmp/d435i-dds.xml \
            -e LIBGL_ALWAYS_SOFTWARE=1 \
            -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/d435i.xauth:ro" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
            -v "$ROOT/ws/log/d435i-dds.xml:/tmp/d435i-dds.xml:ro" \
            -v "$ROOT/go2_nav2/rviz/d435i.rviz:/tmp/d435i.rviz:ro" \
            --entrypoint /bin/bash "${GO2_RVIZ_IMAGE:-go2-humble:local}" \
            -c 'source /opt/ros/humble/setup.bash && exec rviz2 -d /tmp/d435i.rviz'
        ;;
    *) echo 'Usage: ./d435i.sh {setup|start|status|check|rviz|stop}' ;;
esac
