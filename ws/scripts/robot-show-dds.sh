#!/bin/bash
# On robot: show Unitree CycloneDDS config (where topics actually bind).
set +u
source /opt/ros/foxy/setup.bash 2>/dev/null || true
if [[ -f /home/unitree/cyclonedds_ws/install/setup.bash ]]; then
  source /home/unitree/cyclonedds_ws/install/setup.bash
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo ""
echo "IPs:"
hostname -I 2>/dev/null || true
if [[ -n "${CYCLONEDDS_URI:-}" && -f "${CYCLONEDDS_URI#file://}" ]]; then
  echo ""
  echo "=== cyclonedds xml ==="
  cat "${CYCLONEDDS_URI#file://}"
fi
