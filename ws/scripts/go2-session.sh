#!/usr/bin/env bash
# Inside the laptop container: start mapping|navigation|teleop|transport; stop/status.
set -euo pipefail
SESSION_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == start ]]; then
  source "$SESSION_SCRIPT_DIR/go2-env.sh"
fi
# status/stop need no ROS discovery, ping, ROS daemon or robot connection.
exec python3 "$SESSION_SCRIPT_DIR/go2_session.py" "$@"
