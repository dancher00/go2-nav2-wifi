#!/usr/bin/env bash
# Detect conflicting odom->base_link / z drift (run inside docker with Nav2 up).
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

echo "=== Nodes that may publish TF ==="
NODES=$(ros2 node list 2>/dev/null | grep -iE 'odom|tf|amcl|sport|robot_state' || true)
echo "$NODES"

if echo "$NODES" | grep -qx '/odom_tf'; then
  echo ""
  echo "ERROR: /odom_tf is running (OLD). It publishes odom->base_link with Sport z (~0.31)."
  echo "       You need ONLY /go2_odom_tf. Fix:"
  if pgrep -f '(/go2_control/odom_tf|/go2_nav2/odom_tf) --ros-args' &>/dev/null; then
    echo "         → Zombie on this machine (docker pid:host). Run:"
    echo "           /ws/scripts/kill-stale-odom-tf.sh"
  fi
  echo "         1) Stop extra docker shells: docker ps  (leave one ./docker/run.sh)"
  echo "         2) Ctrl+C nav2, then kill-stale-odom-tf.sh, restart nav2_slam_loc"
  echo "         3) On robot: ros2 node list | grep odom — stop duplicate if still there"
fi

echo ""
echo "=== odom -> base_link (direct from go2_odom_tf; z~0.31 = stale Sport odom_tf zombie) ==="
ODOM_BL=$(timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -E '^- Translation' || true)
if [[ -n "$ODOM_BL" ]]; then
  echo "$ODOM_BL"
  echo "WARN: check z should be ~0.0 (planar odom)."
else
  echo "(no transform — is go2_odom_tf running?)"
fi

echo ""
echo "=== odom -> base_footprint (expect z = 0.0 stable) ==="
timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -E '^- Translation' || echo "(no transform)"

echo ""
echo "=== map -> base_footprint (expect z ~ 0.0 stable after Pose Estimate) ==="
timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -E '^- Translation' || echo "(no map frame yet)"

echo ""
if echo "$NODES" | grep -qx '/go2_odom_tf' && ! echo "$NODES" | grep -qx '/odom_tf'; then
  echo "OK: go2_odom_tf + go2_description. For nav: nav2_slam_loc + scan on walls."
fi
