#!/usr/bin/env bash
# Kill legacy odom_tf (node /odom_tf). Keeps go2_odom_tf running.
set -euo pipefail

mapfile -t PIDS < <(
  pgrep -f '(/go2_control/odom_tf|/go2_nav2/odom_tf) --ros-args' 2>/dev/null || true
)

if ((${#PIDS[@]} == 0)); then
  echo "No stale odom_tf process (good)."
  exit 0
fi

echo "Killing stale odom_tf PIDs: ${PIDS[*]}"
kill "${PIDS[@]}" 2>/dev/null || true
sleep 1
mapfile -t LEFT < <(
  pgrep -f '(/go2_control/odom_tf|/go2_nav2/odom_tf) --ros-args' 2>/dev/null || true
)
if ((${#LEFT[@]} > 0)); then
  echo "Force kill: ${LEFT[*]}"
  kill -9 "${LEFT[@]}" 2>/dev/null || true
fi
echo "Done. Run: /ws/scripts/check-tf-odom.sh"
