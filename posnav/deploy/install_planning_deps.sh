#!/usr/bin/env bash
# Run inside the G1 PosNav container. Keep numerical packages off the ROS base image.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-skimage
source "$ROOT/posnav/deploy/env.sh"
/usr/bin/python3 -m pip install --no-deps --target "$ROOT/.cache/planning_python" 'scikit-fmm==2023.4.2'
