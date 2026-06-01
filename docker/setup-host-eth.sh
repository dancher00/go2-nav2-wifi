#!/usr/bin/env bash
# Run on HOST (not in Docker): static IP for Go2 Ethernet port
set -euo pipefail
cd "$(dirname "$0")"

IP_CIDR="${GO2_ETH_IP:-192.168.123.51/24}"
ROBOT_IP="${GO2_ROBOT_IP:-192.168.123.18}"

if [[ -z "${GO2_ETH_IFACE:-}" ]]; then
  # USB-Ethernet to Go2: first enx* with link, else any enx*
  GO2_ETH_IFACE=$(ip -br link 2>/dev/null | awk '/^enx/ && $2=="UP" {print $1; exit}')
  GO2_ETH_IFACE="${GO2_ETH_IFACE:-$(ip -br link 2>/dev/null | awk '/^enx/ {print $1; exit}')}"
fi

if [[ -z "${GO2_ETH_IFACE:-}" ]]; then
  echo "ERROR: no USB-Ethernet (enx*) found. Plug cable into Go2 RJ45 and PC."
  echo "Or set: GO2_ETH_IFACE=your_iface ./setup-host-eth.sh"
  exit 1
fi

echo "Interface: $GO2_ETH_IFACE -> $IP_CIDR"
ip -br link show "$GO2_ETH_IFACE" || true

if ip -br link show "$GO2_ETH_IFACE" | grep -q 'DOWN\|NO-CARRIER'; then
  echo "WARN: $GO2_ETH_IFACE is DOWN / no carrier — check cable and robot power."
fi

sudo nmcli con delete go2-eth 2>/dev/null || true
sudo nmcli con add type ethernet ifname "$GO2_ETH_IFACE" con-name go2-eth \
  ipv4.addresses "$IP_CIDR" ipv4.method manual ipv6.method disabled
sudo nmcli con up go2-eth

echo ""
echo "Test:"
ping -c2 "$ROBOT_IP" || echo "No ping — cable, robot ON, correct enx interface?"
