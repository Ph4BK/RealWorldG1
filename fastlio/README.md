# G1 MID360 离线 FAST-LIO 验证

本目录记录本地 Jazzy 离线验证配置。正式部署目标仍为 G1 ROS2 Humble；
本机编译通过不代表已在 G1 上验证，x86_64 的二进制也不能复制到 ARM64 使用。

上游：<https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID>

- 分支：`humble`
- 固定提交：`df4772ec4797172430e7efe990711d09a529f4ad`
- 源码/构建目录：`../.cache/fastlio_ws`
- 本地修改：`patches/unitree_pointcloud2_and_build.patch`
- 不修改原始 rosbag，不连接机器人，不发布运动指令。

## 环境

本地 Ubuntu 24.04、ROS2 Jazzy；已安装 PCL 1.14、yaml-cpp、
`ros-jazzy-pcl-ros`、`ros-jazzy-pcl-conversions`、`ros-jazzy-cv-bridge`、
`ros-jazzy-image-transport`、`libc++-dev`、`libc++abi-dev`。

Open3D C++ 使用官方 CPU 包：
<https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz>

SHA256：`2e525fd2afe7e80907d2b6e3c66b69e2ef1481dbceed5727b749a6e65cba5720`

解压位置：`../.cache/open3d`。运行定位节点时需把该目录的 `lib` 加入
`LD_LIBRARY_PATH`。上游使用定制 Open3D API，补丁将 RANSAC 随机种子接口
改为官方 0.19 API，并显式链接其 LLVM C++ 运行库。

本地 ROS2 提供 C++ 节点所需的消息、DDS、TF、参数和 rosbag 回放组件。
FAST-LIO 数学算法并非必须依赖 ROS；只读分析 SQLite/CDR 数据也可以不用启动节点。
Humble 与 Jazzy 的可移植对象是源码、配置和数据，不是构建产物；正式部署需在
Humble 上重新编译和回放验证。Humble 设备应使用匹配 CPU 架构的 Open3D 构建。

## 输入适配

`lidar_type: 5` 是补丁新增路径，直接读取 `PointCloud2`：

| 字段 | 类型 | 字节偏移 |
|---|---|---|
| x/y/z | float32 | 0/4/8 |
| intensity | float32 | 12 |
| ring | uint16 | 16 |
| time | float32 | 18 |

实测 `point_step=22`；`ring=0..3`；逐点 time 每帧约 1e8，结合 10 Hz 帧周期
和接收时间推定为纳秒偏移。`timestamp_unit: 3` 将其转换成 FAST-LIO 内部毫秒。
没有通过改名或补零伪造 CustomMsg；编译时可关闭 CustomMsg，从而不需要 Livox SDK。

IMU 约 200 Hz，静止加速度模长约 1，FAST-LIO 源码会进行重力模长归一化。
本实验保留原始 IMU 数值和轴向；平移外参为 `[-0.011,-0.02329,0.04412]`，
旋转外参暂用上游单位阵，关闭在线外参估计。该设置不等于完成外参标定。

补丁还增大 IMU 队列、允许 best-effort 输入，并从内部 ikd-Tree 保存地图。
上游 `/map_save` 原本保存地图发布缓存，关闭地图发布会产生空 PCD。

## 复现

首次准备新的工作目录时检出以上固定提交，再应用补丁；不要重复应用：

```bash
git clone --branch humble https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git \
  RealWorldG1/.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID
git -C RealWorldG1/.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID \
  checkout df4772ec4797172430e7efe990711d09a529f4ad
git -C RealWorldG1/.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID \
  apply ../../../../fastlio/patches/unitree_pointcloud2_and_build.patch
bash RealWorldG1/fastlio/build.sh
```

从 UniGoal 根目录运行，输出目录必须是新目录：

```bash
source /opt/ros/jazzy/setup.bash
source RealWorldG1/.cache/fastlio_ws/install/setup.bash
export LD_LIBRARY_PATH="$PWD/RealWorldG1/.cache/open3d/lib:$LD_LIBRARY_PATH"
/usr/bin/python3 RealWorldG1/tools/replay_fastlio_bag.py \
  RealWorldG1/datasets/small_area_walk/bag \
  RealWorldG1/diagnostics/fastlio_new_run --rate 0.25
```

工具默认使用独立 ROS domain 91 和 localhost 发现；需要本机 DDS 套接字权限。
`summary.json` 的 `success` 只表示流程完成并保存非空地图，
`quality_validated` 仍为 false，不代表轨迹或定位精度通过验证。

输出包括 `map_raw.pcd`、`odometry.csv`、全部收到的配准点云 `registered_xyz.npy` 和日志。
点云与位姿均为 FAST-LIO `camera_init` 系，位姿参考点为 IMU/body，不是已标定的 G1 机体中心。

生成 XY 叠加和轨迹图（绘图 Python 需要 numpy/matplotlib）：

```bash
python RealWorldG1/tools/plot_fastlio_bev.py RealWorldG1/diagnostics/fastlio_new_run \
  --audit RealWorldG1/diagnostics/fastlio_20260922/bag_audit.json
```

`bev_xy_raw.png` 保留 FAST-LIO 原坐标；`bev_xy_leveled.png` 对点云和轨迹共同施加
基于首 2 秒加速度的重力调平旋转。无高度筛选、无尺度修正。精确变换保存在
`T_level_from_camera_init.txt`，调平不会修复轨迹漂移。

## 历史失败记录（已定位配置问题）

首轮 0.5 倍速、0.15 米体素回放出现明显轨迹发散。
最终可视化运行位于 `../diagnostics/fastlio_20260922/mapping_visual/`，
使用 0.25 倍速、默认 0.5 米体素和扩大的 IMU 队列：

- 收到 298 帧配准点云、298 个位姿，共 1,039,812 点。
- 地图和轨迹已投影为 `bev_xy_raw.png`、`bev_xy_leveled.png`。
- 轨迹三维长度约 23.91 米，起终点距离约 3.44 米；这些都是估计值，不是真值。
- 开始约 2 秒的估计位移为 0.397 米，而采样的原始点云配准显示该阶段基本静止。
- 调平后的轨迹 Z 跨度约 1.97 米，俯视图中可见明显结构重影，因此质量未通过验证。

依赖安装、建图/定位节点编译和非空地图保存已验证；没有将这张可疑地图用于
宣称有效的全局定位结果。仅延长队列和降低回放速度没有消除问题。2026-09-27 静止对照已定位到本地配置中地图窗口与检测距离不匹配，以及该场景的降采样过粗；详见下节。
目前只有同一段建图数据，后续即使在其自身生成的地图上定位成功，也仅是流程回归，
不能代替独立定位数据或真值精度验证。应在地图质量确认后再开展定位回放。

## 2026-09-27 静止片段修正

当前默认参数为 cube_side_length=1000、det_range=100、surf/map 体素=0.15 米。
上游 MOV_THRESHOLD=1.5，地图窗口必须满足 cube_side_length > 3 * det_range；
原先 200/100 的组合会在静止时触发地图窗口移动和错误的局部地图删除。
回放工具现在在启动前检查这一条件。

相同开头约 3 秒数据：原配置末尾位置距初始化原点 63.17 厘米，
只修地图窗口后 12.77 厘米，再减小体素后 0.742 厘米（全段最大 1.085 厘米）。
28 帧预处理点云逐点核对 XYZ 及内部时间换算无误。
静止测试不能独立验证厂商 time 字段语义、运动同步和外参，也不能提供绝对定位精度。

完整实验记录：[静止诊断报告](records/static_20260927/REPORT.md)。
诊断补丁 static_diagnostics.patch 只用于单独的诊断源码，不属于部署补丁。

修正后的完整回放位于 `../diagnostics/static_20260927/full_corrected/`，298 帧，
调平轨迹范围约 0.621 × 0.411 米，Z 跨度约 0.035 米；开头约 2 秒位移 2.97 mm。
`bev_xy_leveled.png` 可供人工核对。房间结构明显更集中，仍需真实运动或独立数据验证。

## 正式 BEV 坐标约定（2026-09-27 用户确认）

用户已根据实验室布局确认 full_corrected 的调平地图和轨迹在视觉上合理。
后续 BEV 正式采用 bev_xy_leveled 对应的重力调平投影；bev_xy_raw 仅用于坐标诊断。
该人工确认不代表有外部真值的数值精度验证，quality_validated 仍保留 false。
cube_side_length 固定为 1000.0，当前 det_range=100.0，保留窗口参数关系检查。
当前体素 0.15/0.15 米作为实验基线，后续再分别调优。

- Z：静止初始化时平均加速度（比力）归一化，指向上方，与重力相反。
- X：将 camera_init 的 +X 投影到水平面并归一化；不是磁北，也不代表已标定机体前向。
- Y：Z × X，形成右手坐标系。该旋转对整张地图固定，不随每帧机体姿态变化。
- 点云和机器人位置应用同一个 T_level_from_camera_init；姿态也应左乘对应旋转，
  再计算 BEV 航向。不能只翻转图片或只变换点云。
- 对当前 bag 使用首 2 秒已确认近似静止的加速度均值；换数据需重新验证初始化静止条件。
- 当前数据 camera_init 的 +Z 近似向下，偏离竖直向下约 9.04 度。
  调平矩阵近似 diag(1,-1,-1)，行列式 +1：XY 图表现为 Y 翻转，实质是三维旋转。
- 保存地图时同时保存该固定变换。以后在这张地图上定位必须复用这张地图的变换，
  不能根据每次定位启动姿态重新定义同一个 BEV 地图坐标系。
- 当前调平原点仍是初始化原点，并未移到地面；高度筛选时需另行确定地面高度。

已核对公开默认值（两项均为 0.5 米；属于发布配置，不表示对当前场景最优）：
- deepglint MID360: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/blob/humble/FAST_LIO/config/mid360.yaml
- 原始 FAST-LIO MID360: https://github.com/hku-mars/FAST_LIO/blob/main/launch/mapping_mid360.launch
- 原始 FAST-LIO AVIA: https://github.com/hku-mars/FAST_LIO/blob/main/launch/mapping_avia.launch

2026-09-27 用户授权清空历史 diagnostics 并运行 3×3 体素网格实验。历史根因报告和参数对照保存在 fastlio/records/static_20260927；历史原始回放产物已删除，文中旧 diagnostics 路径不再有效。新实验以 diagnostics/voxel_grid_20260927 为准。

## 默认体素更新（2026-09-27 用户确认）

默认 filter_size_surf / filter_size_map 已改为 0.10 / 0.10 米；cube_side_length 保持 1000.0。该组合已完成九组网格比较及本地单独 1 倍速回放。上文的 0.15/0.15 是历史基线。0.05/0.05 使用独立实验配置 config/experiments/g1_mid360_voxel_005.yaml，不改变默认值。

## G1 Humble 机载部署完成（2026-09-27）

PC2 原生 ARM64 编译与 90 秒实时测试、一倍速录制运动数据回放均完成，0.10/0.10 满足本次 10 Hz 输入。主要计算中位数约 35 ms、最大约 46 ms，详细记录见 [机载验证报告](../diagnostics/g1_deployment_20260927/REPORT.md)。[机载启动与停止](deploy/README.md)。真实传感器节点已留运行；未设置开机自启。

## 当前近距离过滤配置（2026-10-12）

G1 实时、回放及本地运行配置中的 `preprocess.blind` 统一调整为 **0.3 m**，表示以雷达原点计算的三维距离阈值。机载实时入口读取 `/work/deploy/g1_live.yaml`，对应宿主机 `/home/unitree/g1_fastlio_20260927/deploy/g1_live.yaml`。FAST-LIO 在启动时读取该值，配置修改后需重启进程，再重新初始化地图定位。

该设置与 BEV 的 0.3～2 m 圆环更新范围是两个独立参数。历史实验记录、正式地图的 `provenance/mapping_config.yaml` 和上游配置保留原值，用于复现当时的结果；已有静态地图不重新生成。
