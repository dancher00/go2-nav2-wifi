#!/usr/bin/env bash
# Finite transport experiment on the laptop: only UDP arriving from the robot.
# Run with NET_ADMIN in the laptop network namespace. No robot settings changed.
set -euo pipefail
[[ $# -ge 7 ]] || { echo 'Usage: with-lidar3d-netem.sh INTERFACE ROBOT_IP DELAY_MS JITTER_MS RATE_MBIT -- COMMAND...' >&2; exit 2; }
netem_dev="$1" netem_robot="$2" netem_delay="$3" netem_jitter="$4" netem_rate="$5"
shift 5
[[ "$1" == -- ]] || exit 2
shift
[[ "$netem_dev" =~ ^[a-zA-Z0-9_.-]+$ && "$netem_robot" =~ ^[0-9.]+$ ]] || exit 2
for number in "$netem_delay" "$netem_jitter" "$netem_rate"; do [[ "$number" =~ ^[0-9]+$ ]] || exit 2; done
# Refuse to replace any existing ingress policy or our named test interface.
if tc qdisc show dev "$netem_dev" | grep -Eq 'clsact|ingress'; then
  echo 'Existing ingress policy; leave it untouched.' >&2; exit 1
fi
ip link add go2lioifb type ifb
netem_ingress=0
cleanup_netem() {
  [[ "$netem_ingress" == 0 ]] || tc qdisc del dev "$netem_dev" clsact
  ip link del go2lioifb
}
trap cleanup_netem EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
ip link set go2lioifb up
netem_args=(limit 256 delay "${netem_delay}ms")
[[ "$netem_jitter" == 0 ]] || netem_args+=("${netem_jitter}ms" distribution normal)
tc qdisc add dev go2lioifb root netem "${netem_args[@]}" rate "${netem_rate}mbit"
tc qdisc add dev "$netem_dev" clsact
netem_ingress=1
tc filter add dev "$netem_dev" ingress protocol ip pref 164 flower \
  src_ip "$netem_robot" ip_proto udp action mirred egress redirect dev go2lioifb
date -u +'%FT%TZ'
tc -s qdisc show dev go2lioifb
set +e
"$@"
netem_result=$?
set -e
tc -s qdisc show dev go2lioifb
tc -s filter show dev "$netem_dev" ingress
date -u +'%FT%TZ'
exit "$netem_result"
