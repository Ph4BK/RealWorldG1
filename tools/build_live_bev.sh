#!/usr/bin/env bash
# Assumes the existing FAST-LIO workspace and startup localization patches are installed.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SOURCE="$ROOT/.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID"
PATCH="$ROOT/fastlio/patches/localization_live_status.patch"
if ! grep -q pub_live_registration_ "$SOURCE/open3d_loc/src/global_localization.cpp"; then
  git -C "$SOURCE" apply --check "$PATCH"
  git -C "$SOURCE" apply "$PATCH"
fi
G1_ROS_DISTRO=${ROS_DISTRO:-jazzy}
source "/opt/ros/$G1_ROS_DISTRO/setup.bash"
export PATH="/opt/ros/$G1_ROS_DISTRO/bin:/usr/bin:/bin"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$ROOT/.cache/fastlio_ws/log" build \
  --base-paths "$SOURCE" --build-base "$ROOT/.cache/fastlio_ws/build" \
  --install-base "$ROOT/.cache/fastlio_ws/install" --packages-select open3d_loc --executor sequential
colcon --log-base "$ROOT/.cache/rviz_panel_ws/log" build \
  --base-paths "$ROOT/fastlio/rviz/pose_replay_panel" \
  --build-base "$ROOT/.cache/rviz_panel_ws/build" \
  --install-base "$ROOT/.cache/rviz_panel_ws/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
