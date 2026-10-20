#!/usr/bin/env bash
# All sensor subscriptions and localization stay inside G1's host DDS network.
source /root/unitree_ros2/setup.sh
export ROS_DISTRO=humble ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" /></Interfaces></General></Domain></CycloneDDS>'
export OMP_NUM_THREADS=2

# Prefer matching Ubuntu NumPy/SciPy binaries over image-wide pip NumPy 2.x.
G1_PLANNING_LIB="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/.cache/planning_python"
export PYTHONPATH="/usr/lib/python3/dist-packages:$G1_PLANNING_LIB:${PYTHONPATH:-}"
