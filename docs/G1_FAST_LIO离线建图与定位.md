# G1 FAST-LIO 离线建图与定位

记录日期：2026-09-22。

## 状态

已完成上游下载、全包输入检查和配置准备。**尚未编译运行 FAST-LIO，未生成建图
或定位结果**，下面的运行命令为依赖准备完成后的步骤，不代表已验证成功。

- 上游：`third_party/FAST_LIO_LOCALIZATION_HUMANOID`，`humble` 分支。
- 固定提交：`df4772ec4797172430e7efe990711d09a529f4ad`，上游源码未修改。
- 原始包：`datasets/small_area_walk/bag`，只读检查，未转换或修改。
- 配置：`config/fastlio_g1_pointcloud2.yaml`。
- 实测报告：`diagnostics/lidar_imu_20260922_210955_audit/summary.json`。

## 实测输入与解析方式

| 项目 | 实测结果 |
| --- | --- |
| 点云 | 301 帧，约 30.08 s，9.97 Hz，`livox_frame` |
| IMU | 6026 条，约 30.13 s，200.00 Hz，`livox_frame` |
| 字段 | float32 `x/y/z/intensity/time`，uint16 `ring` |
| 字节布局 | x/y/z/intensity/ring/time 位于 0/4/8/12/16/18，point_step=22 |
| ring | 0、1、2、3 |
| time | 帧内单调；末点约 1e8，按 ns 解释约 100 ms，符合实测帧周期 |
| 点数 | 6,036,000；其中全零点 2,775,086；没有非有限 XYZ |
| 消息时间 | 两话题 header 时间各自严格递增；第一帧点云起点早于首条 IMU 约 72.6 ms |
| 初始 2 秒平均加速度 | [0.1567105, -0.0171088, -0.9904853]，模长 1.0029516 |

时间单位 ns 是根据数值与帧周期推断的；字段本身没有编码单位。接收时间相对点云
header 的中位差为约 104 ms，而 IMU 为约 2.89 ms，支持点云 header 接近帧起点
的解释，但不等于完成硬件时间同步标定。推算扫描结束到下一帧起点的偏差范围
约 -8.12 到 +7.66 ms，需要在建图运行时关注。

**无需把这份 PointCloud2 转成 CustomMsg。** 上游 `preprocess.cpp` 的
`velodyne_handler` 使用的字段恰好匹配；设置 `lidar_type: 2`、
`feature_extract_enable: false`、`timestamp_unit: 3`、`scan_line: 4`，即可走
保留真实逐点时间的输入路径。名称表示所复用的解析器，不是把 MID360 当成旋转雷达。
所有帧最后一个点的 time 均为正，该路径不会进入按方位角估算时间的分支。

不能照抄上游 YAML 的注释将 type=4 当作任意点云模式：实际枚举 type=4 为
MID360，解析 `line/tag` 并按方位角估时，不匹配本包已有的 `ring/time`。
默认 type=1 则订阅 CustomMsg，与本包类型不匹配。

零点会被 blind 距离过滤剔除，不需要修改原始 rosbag。当前运行配置的 `preprocess.blind` 已于 2026-10-12 调整为 **0.3 m**；本文原实验及静态地图构建时使用的 0.5 m 参数仍保留在历史记录中。
首帧没有完整的 IMU 覆盖，应作为初始化边界看待，不补造 IMU 样本。

## 外参、单位和坐标约定

采用用户指定的 `extrinsic_T: [-0.011, -0.02329, 0.04412]`。
源码约定为 **p_imu = R * p_lidar + T**，不要把方向反过来。
旋转先采用上游 identity 作为待验证假设；首次对照关闭在线外参估计，避免短包中
外参自由变化。这不代表外参已经实测标定。

初始加速度模长约 1，符合 g 单位的数值特征。此版本 `IMU_Processing.hpp`
用初始平均加速度的模长归一化，再乘重力常量，因此此配置不额外乘 9.81。
IMU orientation 全零不能作为有效四元数使用，FAST-LIO 依赖加速度和角速度初始化。

目前保留原始点云/IMU 坐标，不在输入上额外施加 180° 翻转。点云和 IMU 的
同名 frame_id 不能独立证明轴向一致；也不能仅凭直接累计重影证明准确坐标关系。
若后续旋转输入坐标，必须同时处理 IMU、点云及外参表达。

FAST-LIO 的初始世界坐标不应直接假设为地面 z=0、z 轴朝上。建图后需要检查重力
方向与地面，并将点云、轨迹和后续定位初值用同一个刚体变换转换到 BEV 地图坐标。
不能照抄 Go2 的高度阈值、比例修正、雷达到机体偏移。

## 环境限制

当前主机 Ubuntu 24.04 + ROS2 Jazzy；上游针对 Humble，Jazzy 兼容性尚未验证。
本机缺少 PCL 开发包和 pcl_ros/pcl_conversions。执行安装时得到：

```text
sudo: a password is required
```

本次没有安装成功任何依赖。若在该主机继续，可在用户终端先安装：

```bash
sudo apt-get install --no-install-recommends ros-jazzy-pcl-ros ros-jazzy-pcl-conversions
```

这并不代表依赖已齐全：上游仍硬依赖 `livox_ros_driver2` 的消息定义，即使使用
PointCloud2 也需要该包参与编译；下载的 humble 分支未包含它。定位模块还需要
Open3D C++ 库（本机未发现），且 `open3d_loc/CMakeLists.txt` 写死了作者的
`/home/sax/open3d141/lib/cmake/Open3D` 路径。后续必须准备相应依赖并调整该路径。
也可选择已具备这些依赖的 Ubuntu 22.04/Humble 环境。

## 重跑输入检查

从 UniGoal 根目录执行（此命令已实测成功）：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 RealWorldG1/tools/audit_lidar_imu_bag.py \
  RealWorldG1/datasets/small_area_walk/bag \
  --output RealWorldG1/diagnostics/lidar_imu_20260922_210955_audit/summary.json
```

## 编译通过后的回放步骤（待运行）

先 source ROS 及编译后的 install/setup.bash。在单独的输出目录启动 mapping，避免
覆盖旧实验文件；以下绝对路径对应当前机器：

```bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file /home/tim/UniGoal/RealWorldG1/config/fastlio_g1_pointcloud2.yaml \
  -p map_file_path:="$PWD/g1_map.pcd"
```

另一终端回放，不要用 --loop（循环会导致消息时间倒退）：

```bash
ros2 bag play \
  /home/tim/UniGoal/RealWorldG1/datasets/small_area_walk/bag \
  --clock --rate 0.5
```

回放前订阅/录制 `/Odometry_loc`、`/cloud_registered_1` 和 `/path_1` 以保留
轨迹与点云。等待处理完成后，上游提供保存服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

确认输出 PCD 实际存在且非空，再评估墙面重影、轨迹连续性、静止漂移、地面方向，
不能仅以节点启动成功判定建图有效。

## 在地图上定位与 BEV 接入

建图成功后，将地图路径配置到 `open3d_loc`，按实测坐标设置初值以及 IMU、机体、
导航中心之间的变换，再同时运行 FAST-LIO 和地图配准。

同一 bag 在自身生成的地图上回放只能验证流程与自洽性，**不能作为独立定位精度
验证**；可靠验证需要另一段独立录制的数据。当前没有独立定位 bag。

地图定位得到 map 中的位姿后，先转换到 G1 规划所用的机体参考点，再提取平面
x/y/yaw。BEV 和机器人必须使用同一个地图原点、分辨率和轴向约定；三维地图仍需
高度过滤与占据/空闲/未知栅格处理，定位仓库本身不替代这一步。
