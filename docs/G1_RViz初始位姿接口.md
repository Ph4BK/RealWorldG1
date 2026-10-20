> 当前正式的 10 帧实时地图更新与机载部署，以 [正式实现文档](G1_PosNav真机部署.md) 为准；用户已于 2026-10-06 完成真机效果确认。本文保留初始化接口与历史验证记录。

# RViz 初始位姿接口

本地 ROS2 Jazzy 的 RViz2 已安装。G1 上仍为 Humble，本界面用于本地离线 bag，不需要在机器人上安装 RViz。

从项目根目录运行：

```bash
RealWorldG1/tools/run_pose_init.sh relocalization_1
```

窗口中的 **G1 Localization Replay** 面板显示当前数据包、x/y/yaw/z 和回放状态；地图以俯视图显示。

1. 若该数据集已有有效初值，启动时自动加载并显示绿色箭头，可以直接点击按钮。
2. 若没有初值，点击工具栏 **2D Pose Estimate**，在机器人启动位置按下左键，向前方拖动后松开。起点给出 x/y，方向给出 yaw，箭头长度不影响位姿。
3. 点击面板中的 **Start Localization Replay (1x)** 按钮，启动 FAST-LIO 和 open3d_loc，使用保存的初值回放所选 bag。

画箭头仅保存初值，不触发回放。缺少有效的 x、y、yaw 时点击按钮会弹窗提示“Missing valid x, y, or yaw. Please draw an arrow using 2D Pose Estimate first.”。0 是有效数值，不视为空。回放过程中禁用启动按钮并禁止修改初值，防止重复启动或中途改写初值。回放完成后显示成功/失败状态。

关闭 RViz 会退出接口并停止仍在运行的回放。所有程序均为本地离线处理，不发送运动命令。不需要另开终端，也没有自动回放命令行选项。

初值固定保存在所选数据集下，例如 `datasets/relocalization_1/initial_pose/`：

- `initial_pose.json`：手动 x/y/yaw、自动高度、完整旋转（四元数和带约定的欧拉角）、变换矩阵、输入 bag 和地图。
- `localization.yaml`：供 open3d_loc 读取的完整配置，initialpose 含 x/y/z 和三维旋转的三个欧拉角。
- `T_map_from_camera_init.txt`：完整初始变换。
- `T_level_rotation.txt`、`imu_startup.json`：本次 bag 开头 IMU 推导的调平信息。

同一 bag 重复启动使用同一个目录，每次松开新箭头直接覆盖旧初值，不创建时间戳或随机后缀目录。再次打开界面会加载旧的手动 x/y/yaw，并用当前数据集的 IMU 与地图高度重建完整三维初值；无需再次画箭头。

日志与地图显示缓存固定保存在 `.cache/pose_init/relocalization_1/`，interface.log、rviz.log 和 map_xyz.npy 每次启动覆盖。它们不是实验初值。

启用回放时，结果位于 `diagnostics/relocalization_1/replay/`，运行总日志位于 `diagnostics/relocalization_1/replay.log`。每次点击开始按钮会直接清理并覆盖该数据集的旧 replay/ 结果以及 replay.log，不生成历史副本。回放进行中禁止重复启动。初值目录、原始 bag 和固定地图保持不变。

## 初值约定

z 来自 maps/full_lab_v1/metadata.yaml 的 initial_imu_xyz_map[2]（约 1.261 m）；重力方向来自所选 bag 前 2 秒 IMU 均值（录制时保持前 5 秒静止）。做加速度变化与角速度阈值检查，但不据此声称可检测所有非静止情况，例如匀速运动。

IMU +X 在水平面的投影暂作为导航前向轴，尚未额外标定 IMU 与机体前向的偏航外参。点击指定的是 bag 启动时的位置与朝向，不是回放运行中任意时刻的机器人位姿。当前只支持 SQLite db3 bag。

转换 R = Rz(yaw) * R_level，t = [x, y, z]。/initialpose_2d 接收 RViz 二维箭头；/initialpose 发布转换后的三维初值。不得将 RViz 直接改回 /initialpose，否则会绕过高度及倒装调平处理。

注意：上游配置 initialpose 的欧拉角由 Rx * Ry * Rz 组合，单位度；脚本采用 SciPy intrinsic XYZ 导出，与上游源码一致。不要把配置中最后三个数直接当作用户输入的平面 yaw。

参考地图使用独立 /reference_map 话题、可靠且 transient-local QoS；不依赖上游 /map 一次性发布。静态 TF odom -> camera_init 由本接口提供，故回放期间应保持界面节点运行。转换节点不修改固定地图。原仓库的配准阈值与分辨率属于后续实验参数，不能将预置参数视为已验证最优值。

## 验证与环境

已检查三个 relocalization bag 的开头 IMU；验证了倒装示例、航向组合、上游欧拉角回构，以及 ROS 二维输入到三维输出链路。接口测试箭头 (3, -2, 90°) 仅用于软件检查，不是真实实验初值，也未用它执行定位实验。

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 RealWorldG1/tests/test_pose_init_transform.py
```

入口使用系统 ROS Python，地图读取缓存使用现有 unigoal5060_py39 环境中的 Open3D。默认软件 OpenGL，可通过 LIBGL_ALWAYS_SOFTWARE=0 尝试硬件渲染。ROS_DOMAIN_ID 默认 91，仅本机发现；同时仅允许启动一个接口实例。

## 重定位回放输出（精简默认）

`replay_fastlio_bag.py` 带 `--localization-config` 时，默认只保存：

- `localization.csv`：地图坐标系位姿轨迹，列为 stamp,x,y,z,qx,qy,qz,qw。
- `localization_bev.png`：回放成功结束后自动生成，固定地图 BEV 上叠加本次定位轨迹及首末有效定位点。使用地图自身坐标，不对定位轨迹重复调平；初始化成功前没有定位输出的区间不绘制。
- `summary.json`：成功状态、帧数、回放耗时和配准 fitness 范围；运行成功不等于定位精度已验证。
- mapping/localization/playback、静态 TF 及 ROS 运行日志。

默认不采集累积点云，不调用 `/map_save`，不保存 `registered_xyz.npy`、`map_raw.pcd`、`odometry.csv`、`output_timing.csv`。同时覆盖节点参数，关闭 FAST-LIO 的 PCD 保存及定位节点的 scan 保存。FAST-LIO 内部地图、实时点云发布和 RViz 白色扫描显示仍正常运行；固定参考地图不被修改。

确需排查问题时，在直接调用 `replay_fastlio_bag.py` 的命令末尾加入 `--save-debug-artifacts` 恢复详细输出。不带 `--localization-config` 的建图回放继续保留原有完整输出。

输出策略经模拟 ROS 消息及进程的三种模式回归检查验证；本次修改没有重新执行真实重定位实验。

## RViz 面板构建

自定义原生 RViz 面板源码位于 `fastlio/rviz/pose_replay_panel/`，安装在 `.cache/rviz_panel_ws/install/`，入口自动加载。需要重新构建时：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base RealWorldG1/.cache/rviz_panel_ws/log build \
  --base-paths RealWorldG1/fastlio/rviz/pose_replay_panel \
  --build-base RealWorldG1/.cache/rviz_panel_ws/build \
  --install-base RealWorldG1/.cache/rviz_panel_ws/install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

面板调用 `/g1/start_replay` Trigger 服务，订阅 `/g1/pose_init_status` 显示状态。按钮流程测试使用替代播放进程，验证缺少初值、标记不启动、显式按钮启动、防重复和重新加载，不使用测试初值执行真实定位。

## 已修复的首次实测启动问题

2026-09-29 对 relocalization_1 实际回放后修复：

- 直接启动可执行文件时默认节点名为 global_loc_node，与配置键 global_localization_node 不同。回放脚本现显式传入 `-r __node:=global_localization_node`，并在播放前检查节点是否退出。
- 上游未检查 initialpose 数组长度，参数未加载时会越界崩溃；已增加六元素检查。
- FAST-LIO 配准点云包含法向量和曲率字段；旧转换逻辑检查第 4 个字段，可能返回空点云。定位回调现明确只读 XYZ，不依赖附加字段顺序。
- 定位构造函数曾在 TF 发现前读取外参，失败后继续使用无效矩阵。现等待必要的静态 TF，超时明确失败。
- RViz 现使用 `use_sim_time:=true`，重复回放时随 rosbag 时钟回退清空旧 TF，避免 TF_OLD_DATA 导致扫描不可见。

定位源码修复归档在 `fastlio/patches/localization_startup.patch`，重新获取上游源码时需与已有适配补丁一并应用，再编译 open3d_loc。ICP 配准算法及人工初值没有为通过测试而修改。失败运行的日志保存在 `.cache/pose_init/relocalization_1/` 中，正式输出仍使用 `diagnostics/relocalization_1/replay/`。

轨迹绘图使用现有 unigoal5060_py39 环境；如需替换绘图解释器，可设置 `G1_PLOT_PYTHON`。GUI 保持运行状态直到图片生成完毕。

## 实时定位与 BEV 更新入口

实时版本使用独立入口，原离线 bag 界面保持原样：

```bash
RealWorldG1/tools/run_live_bev.sh
```

先按 `fastlio/deploy/README.md` 在 G1 上运行一个 FAST-LIO 实例。此入口连接已经运行的 FAST-LIO，不自动启动、重启或停止机载程序；仅管理本机的定位、地图更新和 RViz 子进程，不发布运动指令。
本机必须能通过 DDS 收到 `/utlidar/imu_livox_mid360`、`/Odometry_loc`、`/cloud_registered_1`。默认 ROS_DOMAIN_ID=0、SUBNET 发现、真实时间；跨 WSL/不同网段时还需实际打通 DDS 网络，SSH 可连通也不能单独证明 DDS 可用。缺少输入时界面等待，不以空点云清除地图。

1. G1 保持静止，等待实时 IMU 与里程计积累约 2 秒。
2. 点击 **2D Pose Estimate**，在地图上画出机器人当前 IMU 参考点的位置和前向方向。
3. 点击 **Start Live Localization + BEV**。画箭头本身不启动定位；程序不会自动沿用上次会话的位置。
4. 等待配准有效，窗口积满 10 帧，状态显示 **Updating**，再遥控行走观察。
5. **Stop Localization + Updates** 结束本机定位并冻结任务地图，外部 FAST-LIO 继续运行；再次开始需重新画箭头。
6. **Restore Static Task Map** 恢复静态底图、清空窗口；定位运行时会重新收满 10 帧再更新。
7. 关闭 RViz 会结束该入口及其定位子进程。重新打开从原静态地图开始。

实时箭头代表当前位姿而非 FAST-LIO 的启动原点。程序由实时 IMU 估计调平旋转，结合地图中记录的 IMU 高度，再计算：
`T_map_from_odom = T_map_from_current_imu × inverse(T_odom_from_current_imu)`。
画箭头和点击开始之间若发生显著位移/转向会拒绝启动并要求重画。IMU +X 的水平投影仍是导航前向，base_link/motion_link 在此定位流程中仍为 IMU 参考点的别名，不代表已标定 G1 机体中心外参。

### 实时地图与显示

配置在 `fastlio/config/live_bev.yaml`；静态底图为 `maps/full_lab_v1/bev_occupancy.npz`。默认最近 10 帧有效点云，`map` 系高度闭区间 `[0.05,1.3] m`，5 cm/格、点数 >=2 为障碍。按点云时间戳匹配里程计，不按接收顺序拼接机器人位姿。点云已处于 `camera_init` 世界系，只应用定位的 map←camera_init 变换，不重复乘机器人位姿。

覆盖区域为以当前 IMU XY 位置为圆心、半径 2 m 的完整圆形区域（含圆心），与朝向无关。先融合未膨胀占据图，区域内直接覆盖、区域外保留历史结果；本轮显示均不膨胀。固定地图文件从不回写。实验开始前必须卸下 G1 雷达前的挡板；区域内其他遮挡暂不额外处理。

三个地图图层均为二值：

- **Original static BEV**：原正式静态图。
- **Last 10 frames local BEV**：本窗口的局部黑白格，仅绘制圆形区域，外部完全透明。
- **Updated task BEV**：累计覆盖后的完整任务图，默认开启。

建议每次只打开一个地图图层；机器人箭头与圆形边界可同时显示。没有新增/清除障碍差异图层。
RViz 渲染为 10 FPS，动态地图图形最多 5 Hz（`map_display_hz`），不改变按有效点云更新的融合及 OccupancyGrid 发布频率。停止和恢复静态地图时立即刷新显示。
局部图对应 `/g1/bev/local` OccupancyGrid，区域外值为 -1；RViz 使用 `/g1/bev/local_display` 黑白格 Marker 渲染，以保证区域外透明，而非 ROS 默认的未知区域灰色。静态与任务地图也使用黑白 MarkerArray（`/g1/bev/static_display`、`/g1/bev/task_display`）显示，绕过当前软件 OpenGL 下标准 Map 图层的 indexed-texture 着色器错误；标准 OccupancyGrid 话题仍保留。

### 有效性与时间

断流、空/非法点云、时间不匹配或配准无效时清空窗口并暂停覆盖，任务地图保留。恢复后重新累计 10 帧。显著定位变换跳变或里程计时钟回退时恢复静态底图，避免旧错误更新残留；时钟回退需要停止并重新初始化。

定位源码增加 `/g1/localization_update` JSON 消息，每次 ICP 完成发送一次，包含实际使用的扫描窗口末时间 `stamp`、`fitness`、`accepted` 及行优先 4×4 map←camera_init `transform`。重复发送旧时间戳不会延长有效期。地图节点不使用高频传播的旧置信度来判断 ICP 新鲜度。

定位时间暂时领先于 BEV 已处理里程计超过 0.2 s 时，先进入最多 10 条、最长 1 秒的等待队列，里程计赶上后自动重新校验；持续不匹配才报错并暂停。等待不延长定位新鲜度，也不跳过已有健康检查。状态话题包含等待条数、累计延后结果数及等待超时数。实现和现场诊断见 [部署文档](G1_PosNav真机部署.md) 第 3.5 节。
初始工程阈值：输入超时 1 s、10 帧窗口最大跨度 2 s、配准超时 6 s、fitness >=0.7、点云/里程计时间差 <=0.03 s；定位修正跳变超过 0.25 m 或 10° 时重置地图。这些不是定位精度保证，现场可在配置中调整。

当前本地 FAST-LIO 的配准点云与里程计均使用扫描末时间；旧机载版本可能不同。如果界面报告时间不匹配，核对实际发布的 header，再设置 `cloud_stamp_offset_s`，不要仅放宽匹配容差掩盖错位。

### 文件与重建

默认只在 `.cache/live_bev/` 保存当前初值、定位配置和有容量上限的定位/RViz 日志（各 2 MB×2）；不创建 diagnostics 历史目录，不保存累计点云、逐帧图像或任务地图，不自动录 bag。
`--headless` 可不启动 RViz，用于接口测试；`--config` 可选择更新参数文件。

首次在其他环境重建：先完成现有 FAST-LIO/Open3D 和 `localization_startup.patch` 适配，再执行：

```bash
RealWorldG1/tools/build_live_bev.sh
```

该脚本应用 `fastlio/patches/localization_live_status.patch`（若尚未应用），编译定位节点与两个 RViz 面板。该补丁同时在定位退出时 join 工作线程，避免 std::thread 析构导致进程异常退出。

验证命令：

```bash
/usr/bin/python3 -m unittest RealWorldG1.tests.test_live_bev_core -v
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  /usr/bin/python3 -m unittest RealWorldG1.tests.test_live_bev_node -v
```

2026-10-06 本地验证：6 项核心测试及 3 项 ROS 消息/初始化流程测试通过；独立 localhost ROS 域回放 relocalization_1，实时融合节点完成 241 次覆盖更新，收到 12 次 ICP 通知。测试产物写入 `/tmp`，未覆盖已有重定位结果。回放验证不代表遥控真机验收；当次运行环境到 G1 的 192.168.123.164 无路由。

完整链路回放检查（额外运行约一分钟，测试输出仅在 `/tmp`）：

```bash
source /opt/ros/jazzy/setup.bash
source RealWorldG1/.cache/fastlio_ws/install/setup.bash
export LD_LIBRARY_PATH="$PWD/RealWorldG1/.cache/open3d/lib:${LD_LIBRARY_PATH:-}"
ROS_DOMAIN_ID=194 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OMP_NUM_THREADS=4 \
  /usr/bin/python3 RealWorldG1/tests/check_live_bev_replay.py
```

### G1 机载部署与浏览器操作（2026-10-06）

已新增独立机载容器 `g1-posnav-humble`，RViz、定位和 BEV 均在 G1 运行，电脑通过 SSH 隧道和浏览器画初始位姿。电脑入口改用 `RealWorldG1/tools/run_g1_live_bev.sh`。部署布局、启动流程和实测结果见 [机载 Live BEV 说明](../posnav/README.md)。此方案已验证实时传感器接收和桌面显示，现场定位与遥控行走更新仍需按真实位姿验收。
