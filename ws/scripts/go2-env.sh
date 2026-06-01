#!/bin/bash
# One-shot env for Docker shell: ROS + go2_nav2 + Wi-Fi or Ethernet DDS.
# Auto-sourced from ~/.bashrc in container (GO2_AUTO_ENV=1).
# Manual: source /ws/scripts/go2-env.sh

if [[ -n "${GO2_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi

export GO2_NET="${GO2_NET:-wifi}"
export GO2_ODOM_SOURCE="${GO2_ODOM_SOURCE:-utlidar}"
export GO2_CMD_VEL_HZ="${GO2_CMD_VEL_HZ:-20}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GO2_ENV_LOADED=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

case "$GO2_NET" in
  eth)
    export GO2_HOST_IP="${GO2_HOST_IP:-192.168.123.51}"
    export GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.123.18}"
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/setup-robot-eth.sh"
    ;;
  wifi)
    export GO2_HOST_IP="${GO2_HOST_IP:-192.168.1.90}"
    export GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.58}"
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/setup-robot-wifi.sh"
    ;;
  *)
    echo "Unknown GO2_NET=$GO2_NET (use wifi or eth)" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac
