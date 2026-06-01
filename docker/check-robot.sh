#!/usr/bin/env bash
# Run inside container after: source /usr/local/bin/source-unitree.sh
set -eo pipefail

source /usr/local/bin/source-unitree.sh

ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.91}"
echo "=== Network (GO2_ROBOT_IP=$ROBOT_IP) ==="
ping -c2 -W2 "$ROBOT_IP" || echo "Robot ping FAIL — Wi-Fi: export GO2_ROBOT_IP=... source setup-robot-wifi.sh"

echo ""
echo "=== ROS 2 discovery (15s) ==="
sleep 5
ros2 daemon stop 2>/dev/null || true
sleep 2
ros2 topic list 2>&1 | tee /tmp/topics.txt
COUNT=$(wc -l < /tmp/topics.txt)
echo "Topics found: $COUNT"

if [[ "$COUNT" -le 3 ]]; then
  echo ""
  echo "Still few topics. Robot must use DOMAIN 0 (empty ROS_DOMAIN_ID)."
  echo "  Do NOT export ROS_DOMAIN_ID=37 on PC or robot."
  echo "  On robot SSH: unset ROS_DOMAIN_ID; open NEW shell; ros2 topic list"
  echo "  Router: disable Wi-Fi client isolation."
fi

grep -E 'utlidar|sport|uslam' /tmp/topics.txt || true
