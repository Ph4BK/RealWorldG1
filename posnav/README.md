# G1 机载 Live BEV 与浏览器桌面

正式实现已于 2026-10-06 经用户真机确认。地图更新逻辑和完整操作说明见 [G1 PosNav 真机部署](../docs/G1_PosNav真机部署.md)。

定位、实时 BEV、RViz 均运行在 G1。电脑通过 SSH 隧道访问 noVNC，浏览器可点击按钮、拖画初始位姿箭头。无需电脑与 G1 之间的 ROS/DDS 发现，也无需修改 WSL 网络模式。

## 日常启动

**实验开始前必须卸下 G1 雷达前的挡板。** 更新范围固定为以机体 IMU 平面位置为圆心、距离 0.3～2 m 的圆环区域，与机器人朝向无关。每次定位会话首次有效位姿输出时，一次性清空该位置半径 0.3 m 内的任务地图；后续内圈保留历史内容。

2026-10-12：RViz 渲染改为 10 FPS，动态地图图形最多 5 Hz；地图融合和标准 OccupancyGrid 仍随有效点云更新。定位时间暂时领先于 BEV 里程计回调时，最多等待 1 秒自动重试，保留原有时间容差及健康检查。配置入口为 `fastlio/config/live_bev.yaml`，详见部署文档第 3.5、4 节。更新 Python/YAML/RViz 配置无需重新编译，重新启动界面后生效。

先确保 G1 上已有 FAST-LIO（以下长驻命令需保留终端）：

```bash
ssh unitree@192.168.123.164
docker start g1-fastlio-humble
docker exec -it g1-fastlio-humble bash /work/deploy/start_live.sh
```

如果 FAST-LIO 已运行，不要重复启动。在电脑的 UniGoal 目录另开终端：

```bash
RealWorldG1/tools/run_g1_live_bev.sh
```

可选单 STG 误差记录：使用 `RealWorldG1/tools/run_g1_live_bev.sh --record-stg-errors`。
连续导航中逐段保存约定的位姿、异常及无异常段的二维 L2 误差到
`RealWorldG1/stg_error_results/*.jsonl`；不加参数时保持原有行为。
字段和异常规则见部署文档第 9.10 节。此选项已于 2026-10-26 同步机载，并通过隔离 ROS 域测试。
完整实验记录新增可选参数 `--record-experiment`，保存带时间戳的事件、轨迹、规划/STG、
任务结果、关键地图快照，以及静态地图、配置和关键源码副本。
**2026-10-26 已同步 G1，并通过机载隔离 ROS 域测试和无目标会话启停检查。**
入口为 `RealWorldG1/tools/run_g1_live_bev.sh --record-experiment`，
结果保存在机载 `RealWorldG1/posnav_experiments/<UTC时间>/`。
可与 `--record-stg-errors` 同时使用，但完整实验记录自身已包含 STG 分段数据。
使用、输出格式和完整性判定见部署文档第 9.14 节。
2026-10-26 用户确认固定当前 STG navigation，实现摘要及两组共 14 条有效记录的统计见
[部署文档第 9.12～9.13 节](../docs/G1_PosNav真机部署.md#912-当前-stg-navigation-实现固定2026-10-26-用户确认)。

输入 G1 SSH 密码，终端显示 READY 后在浏览器打开：

<http://127.0.0.1:6081/vnc.html?autoconnect=1&resize=scale>

WSL 中若存在 `ssh.exe`，脚本优先使用 Windows OpenSSH，让隧道直接终止在 Windows；否则使用 Linux `ssh`。首次连接核对 G1 主机指纹。`G1_TARGET` 可覆盖默认 SSH 目标。

保持 G1 静止至少 2 秒，使用 RViz 的 **2D Pose Estimate** 标出当前 IMU 位置与机头方向，再点击 **Start Live Localization + BEV**。定位有效、窗口累计 10 帧后再用遥控器行走。三层地图按需单独勾选：原静态图、最近 10 帧局部二值图、更新后的任务图。初始化前尚未发布 map 的 TF 时，RViz Global Status 可能显示 Error；定位后若仍报错，展开查看具体原因。

终端保持打开。关闭 RViz 或 Ctrl+C 会停止本次桌面、地图节点及其定位子进程；外部 FAST-LIO 继续运行。下次启动重新选择初始位姿。旧 `tools/run_live_bev.sh` 是执行环境内的入口，电脑日常使用新 `run_g1_live_bev.sh`。

## 部署布局

- G1 宿主目录：`/home/unitree/g1_posnav/RealWorldG1`。
- 独立容器：`g1-posnav-humble`，目录挂载到 `/workspace/RealWorldG1`；host 网络，4 CPU 配额、8 GiB 内存、256 MiB shm，无自动启动策略。
- 原 `g1-fastlio-humble` 容器保持原有配置。
- ROS Humble、CycloneDDS、domain 0、eth0；订阅 `/utlidar/imu_livox_mid360`、`/Odometry_loc`、`/cloud_registered_1`。
- Xvfb + Mesa llvmpipe + Openbox 本地渲染；x11vnc 仅监听 `127.0.0.1:5901`，noVNC/websockify 仅监听 `127.0.0.1:6081`，通过 SSH 访问。
- 正式地图按 `maps/full_lab_v1/package_manifest.json` 校验并复制；参数仍为高度 0.05～1.3 m、5 cm/格、count ≥ 2。
- 默认 `.cache/live_bev/` 保存有容量上限的组件日志和当前运行配置；不自动生成实验结果包或录 bag。
  加 `--record-experiment` 后，本次导航/定位/RViz 组件日志和运行配置写入独立实验目录；不录原始点云/IMU bag。

## 重建依据

本次基于机上已有镜像 `sha256:c79c88db128791614adf1628942a04c74566c97ed7a394b85ac7c610f79e23ee` 创建独立容器，入口为 `bash -lc 'exec sleep infinity'`。仅在新容器安装：

```bash
apt-get update
apt-get install --no-install-recommends libopen3d-dev xvfb x11vnc novnc websockify openbox mesa-utils
```

基础镜像原有 `cuda-compat-12-2` 缺少 `nvidia-l4t-core` 依赖；本次先确认 apt 模拟结果只移除此包，再于新 CPU 容器内执行 `apt-get --fix-broken install`。不对其他容器照搬修复命令。

本地打包与机载构建入口：

```bash
python3 RealWorldG1/tools/package_g1_posnav.py /tmp/g1-posnav.tar.gz
# 将归档解压到 G1 的 /home/unitree/g1_posnav 后：
docker exec g1-posnav-humble bash /workspace/RealWorldG1/posnav/deploy/build.sh
```

打包器使用本地已应用定位补丁的 `.cache/fastlio_ws/src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc`，不是裸上游源码。ARM 适配包括 Humble 的 tf2 `.h` 头文件、系统 libstdc++、Open3D 0.14 API；实时定位使用 ICP，旧版 Open3D 不支持的带随机种子 FPFH 会明确报错。每个归档含源文件 SHA-256 清单。

`start_desktop.sh` 先清除基础镜像含无效 Tegra 占位库的 `LD_LIBRARY_PATH`，再载入 ROS 工作空间，保证使用 Mesa 软件渲染。`env.sh` 优先使用系统 NumPy 1.21.5 / SciPy 1.8，避开镜像 pip NumPy 2.x 与系统 SciPy 的 ABI 冲突。

## 2026-10-06 验证

G1 ARM 原生编译定位及 RViz 面板成功，6 项 BEV 核心测试与 3 项 ROS 节点测试通过。实际截图确认 RViz 显示正式静态地图和面板。SSH 隧道中 HTTP 返回 200、WebSocket 返回 101，并收到 VNC 协议问候；两个远程服务仅监听 loopback。6 秒只读订阅收到 IMU 1201 条、点云 60 帧、里程计 60 条，对应点云和里程计时间戳差为 0。

用户随后已在 G1 上确认 10 帧配置的实时局部地图替换效果良好，将当前实现确认为正式实现。部署自动检查本身未启动运动控制，未用虚构初始位姿启动真机定位。

### 图层切换崩溃修复（2026-10-06）

现场 core dump 显示 RViz Humble 的 `PoseDisplaySelectionHandler::setMessage` 在图层关闭后的消息处理路径发生 SIGSEGV。初始箭头和机器人当前位姿改用 `Marker.ARROW` 显示，原 PoseStamped 话题仍保留；新增显示话题 `/g1/live_bev/initial_arrow_display` 和 `/g1/live_bev/robot_display`。在 G1 独立 ROS 域持续发布测试箭头并反复勾选/取消两种图层验证，未启动真机定位。

桌面启动的端口预检查采用 `SO_REUSEADDR`，允许连接结束后的 TIME_WAIT 状态，仍拒绝实际监听服务；避免崩溃后立即重连误报端口占用。遇到真实端口占用时仍不自动杀死其他服务。

## 手动规划预览

新增 **Compute FMM + STG macro** 按钮：先画红色目标，点击后用当前任务地图及定位快照计算，只显示一个紫色 STG 点，不执行运动。完整参数见正式实现文档第 8 节。机载首次安装依赖：

```bash
docker exec g1-posnav-humble bash /workspace/RealWorldG1/posnav/deploy/install_planning_deps.sh
```


### G1 闭环运控（2026-10-13）

实时界面同时启动空闲的 `g1_navigation_node.py`。定位与地图就绪、画好目标后，点击 **Start Navigation** 才会发送运动指令；**Stop Navigation** 停止当前任务，重画目标同样停止旧任务。参数、暂停恢复及验收状态见 `docs/G1_PosNav真机部署.md` 第 9 节。导航直接发布 G1 API 7105，每条有效期 0.1 s、50 Hz 刷新，不等待响应；不自动切换运动模式。

### ROS 2 Action 与 TCP（2026-11-02）

导航节点同时提供 `/nav/navigate_to`（`nav_interfaces/action/NavigateTo`）。RViz 和 Action 共用控制器，已有任务时拒绝新启动。机载先运行 `bash posnav/deploy/build_nav_interfaces.sh`；全量 `build.sh` 也包含此步骤。可独立启动 StarCo TCP 网桥，详细命令与接口语义见 [G1 Action 接口与 TCP 联调](../docs/G1_Action接口与TCP联调.md)。当前仍为 D1-803 地图，未配置 D3-619 配准；成功半径 0.50 m，暂不精调目标 yaw。
