#!/bin/bash
# source setup-robot-net.sh  — Unitree DDS on Wi-Fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [[ ! -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
  echo "ERROR: run install-deps.sh minimal first"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  return 1 2>/dev/null || exit 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ -f /etc/cyclonedds/go2.xml ]]; then
  export CYCLONEDDS_URI=file:///etc/cyclonedds/go2.xml
elif [[ -n "${CYCLONEDDS_URI:-}" ]]; then
  :
else
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp2s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
fi

echo "RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID"
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
