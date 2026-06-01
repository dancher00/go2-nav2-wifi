#!/bin/bash
# Run ON ROBOT (SSH). Find Unitree ROS topics — without Wi-Fi-only DDS isolation.
#   scp ~/go2/ws/scripts/robot-check-topics.sh unitree@192.168.1.58:~/
#   ssh unitree@192.168.1.58  → choose foxy (1)
#   bash ~/robot-check-topics.sh

set +u
echo "=== 1) Without custom CYCLONEDDS (factory discovery) ==="
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
if [[ -f /opt/ros/foxy/setup.bash ]]; then
  source /opt/ros/foxy/setup.bash
else
  echo "No /opt/ros/foxy"; exit 1
fi
for d in /opt/unitree/*/setup.bash /home/unitree/*/install/setup.bash; do
  [[ -f "$d" ]] && echo "Sourcing $d" && source "$d"
done
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start
sleep 3
ros2 topic list | tee /tmp/go2-topics.txt
N=$(wc -l < /tmp/go2-topics.txt)
echo "Count: $N"
grep -E 'utlidar|lowstate|sport|api/sport' /tmp/go2-topics.txt || echo "No utlidar/lowstate — robot stack may be OFF"

echo ""
echo "=== 2) ROS processes ==="
ps aux 2>/dev/null | grep -E '[r]os2|[r]cl|unitree|utlidar' | head -15 || true

echo ""
echo "=== 3) Env (login shell) ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo "RMW=${RMW_IMPLEMENTATION:-unset}"

echo ""
echo "=== 4) Robot IPs (DDS often binds to Ethernet first) ==="
hostname -I 2>/dev/null || true
echo "If you see 192.168.123.18 AND 192.168.1.58: unplug Ethernet cable for Wi-Fi relay."

echo ""
echo "If step 1 is empty: power robot ON, stand mode, wait 30s, retry."
echo "When utlidar topics appear, start relay: bash ~/robot-relay-wifi.sh (see docs/RELAY-WIFI.md)."
