#!/usr/bin/env bash
# Interactive shell with go2 env (works without docker rebuild).
set -euo pipefail
cd "$(dirname "$0")"
xhost +local:docker 2>/dev/null || true
[[ -f .env ]] || cp -n .env.example .env 2>/dev/null || true
docker compose up -d go2
# -it: interactive TTY (without it bash exits immediately)
# source go2-env then interactive bash (message once; GO2_ENV_LOADED blocks duplicate)
docker compose exec -it go2 bash -c 'source /ws/scripts/go2-env.sh; exec bash -i'
