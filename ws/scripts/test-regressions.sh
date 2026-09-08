#!/usr/bin/env bash
# Run on the laptop/CI host. No robot network, writable source mount or motion.
set -euo pipefail
if [[ "${1:-}" == --help ]]; then
  echo "Usage: bash ws/scripts/test-regressions.sh [docker-image]"
  echo "Default image: go2-humble:local. Build it first from docker/."
  exit 0
fi
if (( $# > 1 )); then
  echo "Usage: bash ws/scripts/test-regressions.sh [docker-image]" >&2
  exit 2
fi
TEST_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEST_IMAGE="${1:-go2-humble:local}"
docker run --rm --network none --read-only --tmpfs /tmp \
  -e ROS_LOG_DIR=/tmp/ros-log \
  -e ROS_DOMAIN_ID=87 \
  -e ROS_LOCALHOST_ONLY=1 \
  -e CYCLONEDDS_URI= \
  --entrypoint /bin/bash \
  -v "${TEST_REPO_ROOT}:/repo:ro" \
  "$TEST_IMAGE" \
  -c 'source /opt/ros/humble/setup.bash && python3 -B -m unittest discover -s /repo/tests -v'
