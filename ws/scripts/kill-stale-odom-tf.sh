#!/usr/bin/env bash
# Compatibility notice: process-name matches do not establish ownership.
set -euo pipefail
echo 'No processes were stopped. Use go2-session.sh status, then stop mapping or stop navigation.' >&2
echo 'For an unmanaged legacy launch, use Ctrl+C in its original terminal.' >&2
exit 1
