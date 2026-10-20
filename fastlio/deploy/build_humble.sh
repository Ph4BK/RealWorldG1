#!/usr/bin/env bash
# Run inside the existing ARM64 Humble environment, with this deployment at /work.
set -eo pipefail
source /opt/ros/humble/setup.bash
export PATH=/opt/ros/humble/bin:/usr/bin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd /work/ws
colcon build --packages-select fast_lio --executor sequential --cmake-args \
  -DWITH_LIVOX_CUSTOM_MSG=OFF -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
