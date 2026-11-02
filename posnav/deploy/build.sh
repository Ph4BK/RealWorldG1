#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$ROOT/posnav/deploy/env.sh"
export PATH="/opt/ros/humble/bin:/usr/bin:/bin"
export CMAKE_BUILD_PARALLEL_LEVEL=2
bash "$ROOT/posnav/deploy/build_nav_interfaces.sh"
colcon --log-base "$ROOT/.cache/fastlio_ws/log" build \
  --base-paths "$ROOT/posnav/source/open3d_loc" \
  --build-base "$ROOT/.cache/fastlio_ws/build" \
  --install-base "$ROOT/.cache/fastlio_ws/install" \
  --packages-select open3d_loc --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
colcon --log-base "$ROOT/.cache/rviz_panel_ws/log" build \
  --base-paths "$ROOT/fastlio/rviz/pose_replay_panel" \
  --build-base "$ROOT/.cache/rviz_panel_ws/build" \
  --install-base "$ROOT/.cache/rviz_panel_ws/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
