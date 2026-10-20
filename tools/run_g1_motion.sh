#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Strip the optional trailing flag only for argument validation.
ARGS=("$@")
if [[ "${!#}" == --calibrate && $# -ge 4 ]]; then
  set -- "${@:1:$#-1}"
fi
if [[ $# == 1 && "$1" == --self-test ]]; then
  NETWORK=none
elif [[ $# == 1 && "$1" == --odom ]]; then
  NETWORK=host
elif [[ ( $# == 3 || $# == 4 ) && "$1" != --* && "$2" != --* && "$3" != --* && "${4:-}" != --* ]]; then
  NETWORK=host
else
  echo 'Usage: run_g1_motion.sh vx [vy] yaw_rate duration [--calibrate]; or --odom; or --self-test' >&2
  exit 2
fi
CACHE="$ROOT/.cache/g1_official_c79c88db1287"
mkdir -p "$CACHE/motion_build"
# Serialize formal motion calls using this deployment root.
exec flock -n "$CACHE/session.lock" docker run --rm --network "$NETWORK" \
  --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
  --mount "type=bind,src=$CACHE/motion_build,dst=/tmp/g1_motion_build" \
  --entrypoint bash horizonrobotics/holomotion@sha256:dc3fbe9dafacb49f36660690958928f8c0f86d1e9afd2647ac6a0a4c008b5c1f -lc '
    set -eo pipefail
    source /root/unitree_ros2/setup.sh
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" /></Interfaces></General></Domain></CycloneDDS>"
    cmake -S /workspace/RealWorldG1/motion -B /tmp/g1_motion_build
    cmake --build /tmp/g1_motion_build -j2
    if [[ "$1" == --self-test ]]; then
      exec /usr/bin/python3 /workspace/RealWorldG1/motion/offline_test.py
    fi
    exec /tmp/g1_motion_build/g1_motion "$@"
  ' -- "${ARGS[@]}"
