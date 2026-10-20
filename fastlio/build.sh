#!/usr/bin/env bash
# Run after installing system dependencies and fetching the pinned source/Open3D.
set -eo pipefail
g1_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
g1_distro=${ROS_DISTRO:-jazzy}
source "/opt/ros/${g1_distro}/setup.bash"
export PATH="/opt/ros/${g1_distro}/bin:/usr/bin:/bin"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
cd "${g1_root}/.cache/fastlio_ws"
colcon build --base-paths src --packages-select fast_lio open3d_loc --executor sequential \
  --cmake-args -DWITH_LIVOX_CUSTOM_MSG=OFF -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpen3D_DIR="${Open3D_DIR:-${g1_root}/.cache/open3d/lib/cmake/Open3D}"
