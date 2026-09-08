#!/bin/bash
# Source the same runtime network configuration used by workspace launchers.
case "${GO2_NET:-wifi}" in
  wifi|eth)
    source "$(dirname "${BASH_SOURCE[0]}")/setup-robot-${GO2_NET:-wifi}.sh" || return $?
    ;;
  *)
    echo "ERROR: GO2_NET must be wifi or eth" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac
echo "RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID"
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
