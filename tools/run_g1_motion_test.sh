#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE='horizonrobotics/holomotion@sha256:dc3fbe9dafacb49f36660690958928f8c0f86d1e9afd2647ac6a0a4c008b5c1f'
NETWORK=none
INTERACTIVE=()
for arg in "$@"; do
  if [[ "$arg" == --execute ]]; then
    if [[ ! -t 0 || ! -t 1 ]]; then
      echo 'Execution requires a local interactive terminal and an on-site supervisor.' >&2
      exit 2
    fi
    NETWORK=host
    INTERACTIVE=(-it)
  fi
done
exec docker run --rm "${INTERACTIVE[@]}" --network "$NETWORK" \
  --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
  --entrypoint bash "$IMAGE" -lc '
    set -eo pipefail
    source /root/miniconda3/etc/profile.d/conda.sh
    conda activate holomotion_deploy
    source /root/unitree_ros2/setup.sh
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export PYTHONDONTWRITEBYTECODE=1
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
    cd /workspace
    exec python RealWorldG1/tools/test_g1_motion.py "$@"
  ' -- "$@"
