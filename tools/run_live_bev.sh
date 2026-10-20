#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$ROOT/.cache/live_bev"
exec 9>"$ROOT/.cache/pose_init.lock"
flock -n 9 || { echo 'Another initial-pose UI is running; close it first.'; exit 1; }
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$ROOT/.cache/fastlio_ws/install/setup.bash"
source "$ROOT/.cache/rviz_panel_ws/install/setup.bash"
export LD_LIBRARY_PATH="$ROOT/.cache/open3d/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=${G1_DISCOVERY_RANGE:-SUBNET}
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-4}
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export ROS_LOG_DIR="$ROOT/.cache/live_bev/ros_logs"
echo "Live BEV: domain=$ROS_DOMAIN_ID, discovery=$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo 'Requires exactly one external FAST-LIO publisher. Navigation moves G1 only after Start Navigation.'
G1_LIVE_PYTHON=${G1_LIVE_PYTHON:-/usr/bin/python3}
if [[ -x "$ROOT/.cache/planning_venv/bin/python" && "$G1_LIVE_PYTHON" == /usr/bin/python3 ]]; then
  G1_LIVE_PYTHON="$ROOT/.cache/planning_venv/bin/python"
fi
exec "$G1_LIVE_PYTHON" "$ROOT/tools/live_bev_node.py" "$@"
