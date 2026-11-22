# 619 Go2-cleaned BEV map package

bev_occupancy.npz is the byte-for-byte Go2 cleaned UNEXPANDED grid, not a PCD
projection. The original all-height ICP map.pcd is unchanged. no_go.npz locks
ONLY the explicit outside-room policy; ordinary obstacles remain dynamically
clearable. Runtime inflation stays 5 cells / 0.25m by user decision, not Go2's
0.35m. This is a user-selected margin, not full-body clearance certification.
allow_initial_clear=false; a blocked current cell must reject navigation.
The live observation height band/count rules do NOT describe static-map creation.
An initialization-ready package is NOT motion permission or accuracy acceptance.
Rebuild this variant with --bev-source go2-grid. build_static_bev.py copies the
retained grid reference for this variant; it must not reproject map.pcd instead.
There is no point-count grid for cleaned occupancy; --counts is rejected.

## Coordinates and limits

map=619 scene, X forward / Y left / Z up; T_map_from_scene.txt is identity,
NOT a sensor/body extrinsic. The initial Z value and its exact measurement source
are in metadata.yaml. A lidar-dome tape measurement approximates IMU height;
do not claim this is a separately surveyed IMU or body/feet reference.
XY/yaw are selected at the current position, not fixed to the scene origin.
The old map may be stale and cleaned Go2 occupancy does not certify all taller
G1 obstacles or missing/occluded space. Dynamic perception remains necessary.

## Offline verification and rebuild

python tools/build_g1_619_map.py --bev-source go2-grid --output-dir <NEW_DIR>   --imu-height-m <HEIGHT> --imu-height-source <RECORD> --preview
python tools/build_g1_619_map.py --validate-only <PACKAGE> --require-runtime-ready

Never overwrite the original PCD-projection package. Validation checks inherited
grid bytes, all-height PCD bytes, immutable room policy, checksums, and the exact
production LiveBEV constructor file-loading block. It is not a live ROS/ICP or
motion test. Select BOTH --map-dir and --config runtime_live_bev.yaml explicitly.
Deploy the matching updated live_bev_core/node and navigation/action/planner
sources together; old runtime code does not enforce the new no-go declaration.
This README does not authorize deployment, node startup or physical motion.

