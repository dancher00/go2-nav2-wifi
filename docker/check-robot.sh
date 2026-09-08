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
  echo "Still few topics. Wi-Fi relay and laptop must share GO2_RELAY_DOMAIN_ID (default 64)."
  echo "  Robot internal DDS and direct Ethernet must stay on domain 0."
  echo "  Re-source the appropriate setup-robot-wifi.sh / setup-robot-eth.sh."
  echo "  Router: disable Wi-Fi client isolation."
fi

grep -E 'utlidar|sport|uslam' /tmp/topics.txt || true
