#!/usr/bin/env bash
set -eo pipefail
source /work/deploy/env_humble.sh
mkdir -p /work/runs/production
export ROS_LOG_DIR=/work/runs/production/ros_logs
exec /work/ws/install/fast_lio/lib/fast_lio/fastlio_mapping \
  --ros-args --params-file /work/deploy/g1_live.yaml
