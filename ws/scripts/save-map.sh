#!/usr/bin/env bash
# Save /map to PGM+YAML (SLAM must be running in another terminal).
set -euo pipefail
set +u
NAME="${1:-my_room}"
DIR="${2:-/ws/maps}"
mkdir -p "$DIR"
FILE="${DIR}/${NAME}"

if ! ros2 pkg prefix nav2_map_server &>/dev/null; then
  echo "ERROR: install nav2 in THIS container: install-deps.sh nav2 && exec bash"
  exit 1
fi

echo "Waiting for /map (up to 30s)..."
if ! timeout 30 ros2 topic echo /map --once &>/tmp/map_once.log; then
  echo "ERROR: no /map message. Start in another terminal:"
  echo "  ros2 launch go2_nav2 slam_mapping.launch.py"
  echo "  (walk slowly 30s, check: ros2 topic hz /map)"
  cat /tmp/map_once.log 2>/dev/null | tail -5
  exit 1
fi
ros2 run nav2_map_server map_saver_cli -f "$FILE" --ros-args -p save_map_timeout:=10.0

if [[ ! -f "${FILE}.yaml" || ! -f "${FILE}.pgm" ]]; then
  echo "ERROR: files not created."
  exit 1
fi

ls -la "${FILE}.yaml" "${FILE}.pgm"

if ros2 service list 2>/dev/null | grep -q '/slam_toolbox/serialize_map'; then
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${FILE}'}" 2>/dev/null || true
  if [[ ! -f "${FILE}.posegraph" ]]; then
    echo "WARN: no ${FILE}.posegraph — re-save while slam_mapping runs"
  fi
else
  echo "WARN: slam_toolbox not running — only .pgm/.yaml; re-save with SLAM up for .posegraph"
fi
