#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
DEB_DIR="$(cd .. && pwd)/debs"
mkdir -p "$DEB_DIR"
chmod 777 "$DEB_DIR" 2>/dev/null || true

echo "Downloading to $DEB_DIR ..."
docker compose run --rm -v "$DEB_DIR:/out:rw" go2 bash -lc '
set -e
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y --download-only --no-install-recommends ros-humble-rmw-cyclonedds-cpp
sudo cp -v /var/cache/apt/archives/*.deb /out/
sudo chmod -R a+r /out/
ls -lh /out/*.deb | wc -l
'
