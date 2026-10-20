#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -ne 1 ]]; then echo "Usage: $0 relocalization_1"; exit 2; fi
BAG="$1"; shift
if [[ ! -d "$BAG" ]]; then BAG="$ROOT/datasets/$BAG/bag"; fi
[[ -d "$BAG" ]] || { echo "Bag not found: $BAG"; exit 2; }
BAG=$(realpath "$BAG")
DATASET=$(dirname "$BAG")
DATASET_NAME=$(basename "$DATASET")
[[ -f "$BAG/metadata.yaml" ]] || { echo "Expected rosbag directory: $BAG"; exit 2; }
mkdir -p "$ROOT/.cache"
exec 9>"$ROOT/.cache/pose_init.lock"
flock -n 9 || { echo "Another pose-init UI is running; close it first."; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$ROOT/.cache/fastlio_ws/install/setup.bash"
source "$ROOT/.cache/rviz_panel_ws/install/setup.bash"
export LD_LIBRARY_PATH="$ROOT/.cache/open3d/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMP_NUM_THREADS=4
RUN="$ROOT/.cache/pose_init/$DATASET_NAME"
SELECTION="$DATASET/initial_pose"
mkdir -p "$RUN"
export ROS_LOG_DIR="$RUN/ros_logs"
"$HOME/miniconda3/envs/unigoal5060_py39/bin/python" - "$ROOT/maps/full_lab_v1/map.pcd" "$RUN/map_xyz.npy" <<'PY'
import sys,numpy as np,open3d as o
np.save(sys.argv[2],np.asarray(o.io.read_point_cloud(sys.argv[1]).points).astype(np.float32))
PY
/usr/bin/python3 "$ROOT/tools/g1_pose_init.py" --bag "$BAG" --map-dir "$ROOT/maps/full_lab_v1" --map-xyz "$RUN/map_xyz.npy" --output "$SELECTION" --replay-output "$ROOT/diagnostics/$DATASET_NAME/replay" >"$RUN/interface.log" 2>&1 &
BRIDGE=$!
cleanup() { kill -TERM "$BRIDGE" 2>/dev/null || true; wait "$BRIDGE" 2>/dev/null || true; }
trap cleanup EXIT
for i in {1..100}; do
  if grep -q READY "$RUN/interface.log"; then break; fi
  if ! kill -0 "$BRIDGE" 2>/dev/null; then cat "$RUN/interface.log"; exit 1; fi
  sleep .1
done
if ! grep -q READY "$RUN/interface.log"; then cat "$RUN/interface.log"; exit 1; fi
echo "Dataset: $DATASET_NAME"
echo "Initial pose (overwritten on each selection): $SELECTION"
echo "Runtime logs/cache: $RUN"
echo 'Draw/update a 2D Pose Estimate arrow, then click the replay button in the G1 panel.'
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
rviz2 -d "$ROOT/fastlio/rviz/pose_init.rviz" --ros-args -p use_sim_time:=true >"$RUN/rviz.log" 2>&1
