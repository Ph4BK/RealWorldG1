#!/usr/bin/env bash
# Records LiDAR point clouds and LiDAR IMU messages.
# Manual, read-only recording. Run on PC2; Ctrl+C closes the rosbag.
set -euo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo 'Usage: bash RealWorldG1/tools/record_g1_lidar_imu.sh [output_parent]'
  echo 'Default output_parent: /home/unitree/g1_diagnostics'
  echo 'Records only /utlidar/cloud_livox_mid360 and /utlidar/imu_livox_mid360. Stop with Ctrl+C.'
  exit 0
fi
if [[ $# -gt 1 || ${1:-} == --* ]]; then
  echo 'Invalid arguments; use --help.' >&2; exit 2
fi
if [[ ! -t 0 || ! -t 1 ]]; then
  echo 'Run interactively in a PC2 terminal so Ctrl+C can stop recording cleanly.' >&2
  exit 2
fi
command -v docker >/dev/null
parent=${1:-/home/unitree/g1_diagnostics}
mkdir -p -- "$parent"
parent=$(cd -- "$parent" && pwd)
if [[ "$parent" == *,* ]]; then
  echo 'Output path must not contain commas (Docker mount syntax).' >&2; exit 2
fi
session=$(mktemp -d "$parent/lidar_imu_$(date +%Y%m%d_%H%M%S)_XXXXXXXX")
echo "Bag output: $session/bag"
echo 'Wait for both topic subscriptions and Recording... before moving.'
echo 'Keep still for about 5 seconds at the start for later IMU initialization.'
echo 'Stop with Ctrl+C once; wait for the shell prompt before closing SSH.'
exec docker run --rm --init -it --network host \
  --mount "type=bind,src=$session,dst=/data" \
  --env RECORD_UID="$(id -u)" --env RECORD_GID="$(id -g)" \
  --entrypoint bash \
  horizonrobotics/holomotion@sha256:dc3fbe9dafacb49f36660690958928f8c0f86d1e9afd2647ac6a0a4c008b5c1f -lc '
    set -eo pipefail
    trap "chown -R ${RECORD_UID}:${RECORD_GID} /data" EXIT
    source /root/unitree_ros2/setup.sh
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" /></Interfaces></General></Domain></CycloneDDS>"
    ros2 bag record --storage sqlite3 --output /data/bag \
      /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360
  '
