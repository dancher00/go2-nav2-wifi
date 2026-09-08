#!/usr/bin/env bash
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/go2-session.sh" start navigation --map "${1:-/ws/maps/my_room.yaml}"
