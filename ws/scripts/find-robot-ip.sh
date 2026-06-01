#!/usr/bin/env bash
# Find Go2 on current Wi-Fi / phone hotspot (ping sweep + optional SSH hint).
set -euo pipefail

IFACE="${GO2_WIFI_IFACE:-$(ip route show default 2>/dev/null | awk '{print $5; exit}')}"
if [[ -z "$IFACE" ]]; then
  echo "ERROR: no default route. Connect laptop to phone hotspot first."
  exit 1
fi

HOST_CIDR=$(ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet / {print $2}' | head -1)
if [[ -z "$HOST_CIDR" ]]; then
  echo "ERROR: no IPv4 on $IFACE — connect to hotspot."
  exit 1
fi
HOST_IP="${HOST_CIDR%%/*}"

BASE="${HOST_IP%.*}"
echo "=== Scan ${BASE}.0/24 on $IFACE (laptop ${HOST_IP}) ==="
echo "Looking for robot (ping + ssh unitree@IP)..."
echo ""

FOUND=()
for i in $(seq 1 254); do
  ip="${BASE}.${i}"
  [[ "$ip" == "${HOST_IP%/*}" ]] && continue
  if ping -c1 -W1 "$ip" &>/dev/null; then
    FOUND+=("$ip")
    hint=""
    if timeout 2 ssh -o BatchMode=yes -o ConnectTimeout=1 -o StrictHostKeyChecking=no \
      "unitree@${ip}" "hostname" &>/dev/null; then
      hint="  <- likely Go2 (ssh unitree OK)"
    fi
    echo "  $ip$hint"
  fi
done

echo ""
if [[ ${#FOUND[@]} -eq 0 ]]; then
  echo "No hosts found. Check: robot ON, same hotspot SSID, AP isolation OFF on phone."
  exit 1
fi

echo "Then in every container terminal:"
echo "  export GO2_ROBOT_IP=<robot_ip>"
echo "  source /ws/scripts/setup-robot-wifi.sh"
