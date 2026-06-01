#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
xhost +local:docker 2>/dev/null || true
docker compose build go2
docker compose run --rm go2 "$@"
