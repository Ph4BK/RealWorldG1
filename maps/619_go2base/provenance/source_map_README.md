# D3-619 地图 v1（2026-10-21）

## 坐标系

所有文件在同一坐标系：原点 = 机器人起步位置车体中心（两驱动轮中点）的地面投影；x 朝车头方向，y 朝左，z 朝上；单位米。

## 文件

- `pointcloud/`：激光雷达 SLAM 建的整间房点云。`map.pcd` / `map.ply`（约 261 万点）、`map_5cm.pcd` / `map_5cm.ply`（5 cm 下采样，约 45 万点）；`b6_trajectory_tum.txt` 是建图时机器人的轨迹（TUM：时间 x y z qx qy qz qw）。
- `grid/`：2D 占据栅格，Nav2 `map_server` 格式（`D3-619.pgm` + `D3-619.yaml`），5 cm / 格。黑 = 障碍（离地 0.10–1.80 m 的点），白 = 地面，灰 = 未知。**未按车体膨胀障碍**。`D3-619_preview.png` 带建图轨迹，仅供查看。
- `rgbd_conceptgraphs/`：ConceptGraphs / NICE-SLAM 的 Replica 格式，483 帧，1280 × 960。
  - `results/frame*.jpg`：彩色图（已去畸变）
  - `results/depth*.png`：深度，16 位，`png_depth_scale = 1000`（单位 mm），0 = 无效
  - `traj.txt`：每行一帧，16 个数 = 4 × 4 相机到世界变换（c2w，行优先）；相机系为 OpenCV（x 右、y 下、z 前）
  - `replica_config.yaml`：内参（去畸变后）
  - `results/mask*.png`：机器人车体掩膜（255 = 车体），ConceptGraphs 不读，可忽略
  - `traj_tum.txt`、`manifest.json`、`alignment_check.json`：同一位姿的 TUM 形式、逐帧来源、位姿自查结果
- `3dgs/point_cloud.ply`：标准 3DGS PLY（150 万个高斯），可用 SuperSplat 等查看器打开。`metrics.json` 为训练指标，`test_renders/` 为未参与训练的视角对照（左实拍、右渲染）。

## 覆盖与精度

- 点云与栅格：整间房。RGB-D 与 3DGS：只覆盖沙发区一带（机器人那次运行走过的范围）。
- 位姿来自激光雷达 SLAM。RGB-D 自查：深度点到雷达点云的偏差，逐帧中位数全部 ≤ 4 cm（总体中位 2.8 cm）。
- 3DGS 在未参与训练的视角上 PSNR 25.7、SSIM 0.91；近处物体清楚，远处（窗户、天花板一带）偏糊——那些方向深度相机测不到。
- 数据录于 2026-10-12、10-13；之后房间布置的变化没有反映。
