#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
colcon --log-base "$ROOT/.cache/nav_interfaces_ws/log" build \
  --base-paths "$ROOT/posnav/interfaces/nav_interfaces" \
  --build-base "$ROOT/.cache/nav_interfaces_ws/build" \
  --install-base "$ROOT/.cache/nav_interfaces_ws/install" \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
