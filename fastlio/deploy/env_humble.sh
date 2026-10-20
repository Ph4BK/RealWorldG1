#!/usr/bin/env bash
source /root/unitree_ros2/setup.sh
if [[ -f /work/ws/install/setup.bash ]]; then source /work/ws/install/setup.bash; fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" /></Interfaces></General></Domain></CycloneDDS>'
export OMP_NUM_THREADS=2
