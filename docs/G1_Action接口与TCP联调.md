# G1 PosNav Action 与 TCP 初版

本接口复用 StarCo 提交 `8097d5f076dd5d3b87756d3716ebeca836f13e5c` 的 NavigateTo 定义和 TCP 网桥。它接入现有 G1 控制器；RViz 操作入口继续保留。导航核心、两端转向和速度参数未因接口适配修改。

## 当前坐标约定

默认仍使用 D1-803 的 `maps/full_lab_v1`。`fastlio/config/live_bev.yaml` 中 `action.coordinate_frame=g1_d1_803_map`，`map_from_scene_xy_yaw=[0,0,0]`，即外部坐标数值与当前 G1 map 相同，参考点为 IMU。这里的 scene 是 Action 外部坐标层，并不代表已有 D3-619 配准。

变换为 `p_map=R(theta)*p_scene+[tx,ty]`，yaw 同样加 theta；反馈使用逆变换。变换在导航节点启动时读取，任务间保持不变。请求 `s_*` 仅记录，不重置定位或逐段拟合变换。

Action 位姿的 ROS `header.frame_id` 固定为 `scene`，以满足 StarCo 对终点接续的检查。TCP 外层 `coordinate_frame` 使用上述配置的完整标识。网桥会拒绝 D3-619 场景请求进入当前 803 配置。到 619 重新部署并验证定位/坐标关系后，再修改配置；不要只更改名称来通过检查。

## 接口与行为

- 类型：`nav_interfaces/action/NavigateTo`；名称：`/nav/navigate_to`。源文件位于 `posnav/interfaces/nav_interfaces`，在目标 ROS 发行版构建，不复制其它发行版的二进制。
- 启动时空闲，不自动执行任务。Action 和 RViz 共用一个运动指令发布器；已有导航或已预留的 Action 任务时，拒绝新启动。
- Goal 中数值须有限；z/roll/pitch 为零，yaw 为 [-pi,pi] 弧度，容差/timeout/max_speed 为正。当前固定步行速度 0.5 m/s，拒绝低于该值的 max_speed。
- 请求 pos_tol/yaw_tol 保留并记录，但初版不据此改变控制器：成功为估计 XY 距离 <=0.50 m，沿用现有控制器停止确认，不执行末端 yaw 精调。`remaining_yaw` 仅报告误差。
- Feedback 最大约 10 Hz：stage 是实际状态（暂停时附原因）；current_pose、remaining_dist、remaining_yaw 为真实估计。位姿时间戳沿用最近输入的源时间戳，暂停时不把旧位姿改成新时间。
- 成功：ROS 状态 4，success=true，reason=arrived；Action 取消：状态 5；超时、RViz Stop/重画目标、定位停止、执行异常：状态 6。先发送零速度再结束。内部 plan_wait/障碍等待仍是任务执行过程，不自动判为失败。
- 整体超时采用单调时钟，包含等待和暂停时间。断线取消与心跳由 TCP 网桥处理；ROS 直连客户端断线本身不会触发立即取消，任务仍受整体超时限制。
- final_pose 是实际终点，internal_error 是实际 XY 估计误差；位姿陈旧时返回 pose_stale（没有位姿时 no_pose），internal_error=-1，禁止用于下一段位姿接续。
- uncertainty=-1 表示暂无估计；blocked_distance=-1、blocked_bearing=0 表示暂无对应障碍测量。普通导航失败原因还包括 no_stg、goal_redrawn、stopped_by_user、localization_stopped、navigation_error。
- 开启 `--record-experiment` 时，记录 Action Goal UUID、关联 trial、请求与有效容差、坐标变换、开始和结束事件；日志包包含接口及网桥源码。
- `/nav/planned_path` 是 StarCo 的可选可视化输入，本版没有新增此话题。现有 RViz STG 显示继续工作。

## 构建与启动

在 G1 的 `g1-posnav-humble` 容器内，项目路径为 `/workspace/RealWorldG1`：

```bash
cd /workspace/RealWorldG1
bash posnav/deploy/build_nav_interfaces.sh
```

全量重新部署运行 `posnav/deploy/build.sh` 时也会构建此接口。原有启动命令继续使用：

```bash
bash RealWorldG1/tools/run_g1_live_bev.sh --record-experiment
```

该命令在电脑项目父目录执行，通过 SSH 打开原 RViz 桌面；导航子节点同时提供 Action Server。仍需按原流程启动 FAST-LIO、选择初始位姿并启动定位。

TCP 网桥独立启动。另开终端，在同一容器内执行：

```bash
cd /workspace/RealWorldG1
bash posnav/deploy/run_tcp_bridge.sh --allow-motion
```

默认监听 `192.168.123.164:8888`。可用 `G1_BRIDGE_BIND`、`G1_BRIDGE_PORT`、`G1_BRIDGE_CONFIG`、`G1_BRIDGE_RUNTIME` 覆盖部署路径和地址。启动网桥本身不发送目标。不加 `--allow-motion` 仅允许连接检查，不转发运动任务。

首次运行会在 `.cache/tcp_bridge/.token` 创建权限 600 的共享口令。将其填到 StarCo 客户端 TCP 口令框，IP 改为 G1 地址。口令不提交 Git，不打进实验包。运行日志和未完成请求状态位于 `.cache/tcp_bridge`；保留上游 pending_goal 保护，不自动删除或重放。

网桥与导航节点须使用同一个配置，且修改后都重启。ROS_DOMAIN_ID=0。D1-803 只做本地地图接口测试；当前 D3-619 StarCo 默认场景应因坐标标识不符而被拒绝。

不要在 G1 使用 Go2 的 dog_services.py 或 CAPO 启动脚本。迁移 619 另一台 G1 时，应重新构建接口并按实际地址配置监听，不能只复制 803 的编译产物。

## 验证边界

`tests/test_g1_action_server.py` 使用真实 ROS ActionClient/ActionServer，在非零 ROS 域、localhost 上运行，运动发布器替换为内存收集器。覆盖成功、反馈、取消、超时、互斥、RViz、变换和陈旧位姿；不构成真实机器人运动或 D3-619 地图验证。

2026-11-02 在 D1-803 的 G1 Humble 容器中通过 168 项测试，其中 12 项为新增 Action/TCP/日志关联测试。真实 TCP 回环连接通过上游网桥转发到 Action Server 并返回 arrived；另验证规划重试不提前终止。所有测试使用隔离 ROS 域并替换运动出口，没有执行真机行走。
