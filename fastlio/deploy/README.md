# G1 PC2 FAST-LIO Humble 部署

部署日期：2026-09-27。只运行定位，不发布运动控制请求。

- SSH：unitree@192.168.123.164，登录信息见 docs/G1连接方式与状态确认.md。
- 机载目录：/home/unitree/g1_fastlio_20260927。
- 容器：g1-fastlio-humble，host 网络，目录挂载为 /work。
- 固定镜像：horizonrobotics/holomotion@sha256:dc3fbe9dafacb49f36660690958928f8c0f86d1e9afd2647ac6a0a4c008b5c1f。
- 宿主 Ubuntu20.04 ARM64，容器 ROS2 Humble，PCL1.12，Eigen3.4。
- 工作空间：/work/ws，只编译 fast_lio，WITH_LIVOX_CUSTOM_MSG=OFF。
- 输入原生 PointCloud2 和 IMU；无 Livox 消息转换，无 Open3D 依赖。
- 默认 surf/map=0.10/0.10，cube=1000，det_range=100，其他外参与本地一致。
- 近距离过滤 `preprocess.blind=0.3 m`（2026-10-12 更新），实时与回放配置一致；启动时加载，修改后需重启 FAST-LIO 并重新初始化定位。
- DDS：CycloneDDS，eth0，ROS_DOMAIN_ID=0。

## 实时运行

在 G1 SSH 终端中运行（确认容器内没有另一个 fastlio_mapping）：

```bash
docker start g1-fastlio-humble
docker exec -it g1-fastlio-humble bash /work/deploy/start_live.sh
```

当前台运行时 Ctrl+C 正常退出。后台运行的专用容器中可使用：

```bash
docker exec g1-fastlio-humble pkill -INT -x fastlio_mapping
```

程序不会自动开机启动；容器停止后需要重新启动节点。不要同时运行多个实例。
生产配置 g1_live.yaml 使用真实时间，关闭持续 PCD 累积与逐帧性能日志，避免无限增长。
因此该配置下 /map_save 不会保存地图；需要建图保存时另用启用 pcd_save 的有限时长配置。
保留稠密配准点云输出。输出 /Odometry_loc、/cloud_registered_1、/path_1。
输出参考系仍是 FAST-LIO camera_init；IMU 作为导航参考点。此部署尚不包含固定地图重定位、
重力调平后的导航位姿接口或 FMM。

## 性能验证

- benchmark_live.py：有时限地启动节点，临时启用 runtime_pos_log_enable，采样输入/输出后停止。
- monitor_live.py：只读记录输入与位姿频率、wall-minus-header 延迟。
- g1_replay.yaml：离线 use_sim_time 配置，运行时必须换到独立 ROS 域，防止真实输入与回放混合。
- 每帧计时来自源码的 t5-t0，覆盖 IMU 处理、地图维护、降采样、滤波更新和增量地图等主要计算，
  不含所有消息预处理、发布和日志开销。端到端延迟另由监控采集。
- 实时延迟使用 PC2 系统时间减传感器时间戳，依赖设备间时间同步；实测 IMU 到达延迟约 2 ms。
- 在线点云 header 是扫描起始时间，位姿 header 是扫描末时间，二者延迟不能直接相减当作计算耗时。

本地原始结果：diagnostics/g1_deployment_20260927/。
机载原始结果：/work/runs/live_010 和 /work/runs/replay_010_1x。
