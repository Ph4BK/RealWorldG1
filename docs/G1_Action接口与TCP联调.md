# G1 PosNav Action 与 TCP 初版

本接口复用 StarCo 提交 `8097d5f076dd5d3b87756d3716ebeca836f13e5c` 的 NavigateTo 定义和 TCP 网桥。它接入现有 G1 控制器；RViz 操作入口继续保留。导航核心、两端转向和速度参数未因接口适配修改。

## 当前坐标约定

默认仍使用 D1-803 的 `maps/full_lab_v1`。`fastlio/config/live_bev.yaml` 中 `action.coordinate_frame=g1_d1_803_map`，`map_from_scene_xy_yaw=[0,0,0]`，即外部坐标数值与当前 G1 map 相同，参考点为 IMU。这里的 scene 是 Action 外部坐标层，并不代表已有 D3-619 配准。

变换为 `p_map=R(theta)*p_scene+[tx,ty]`，yaw 同样加 theta；反馈使用逆变换。变换在导航节点启动时读取，任务间保持不变。请求 `s_*` 仅记录，不重置定位或逐段拟合变换。

Action 位姿的 ROS `header.frame_id` 固定为 `scene`，以满足 StarCo 对终点接续的检查。TCP 外层 `coordinate_frame` 使用上述配置的完整标识。网桥会拒绝 D3-619 场景请求进入当前 803 配置。到 619 重新部署并验证定位/坐标关系后，再修改配置；不要只更改名称来通过检查。

## 接口与行为

- 类型：`nav_interfaces/action/NavigateTo`；名称：`/nav/navigate_to`。源文件位于 `posnav/interfaces/nav_interfaces`，在目标 ROS 发行版构建，不复制其它发行版的二进制。
- 启动时空闲，不自动执行任务。Action 和 RViz 共用一个运动指令发布器；已有导航或已预留的 Action 任务时，拒绝新启动。
- Goal 中数值须有限；z/roll/pitch 为零，yaw 为 [-pi,pi] 弧度，容差/timeout/max_speed 为正。当前 STG 步行速度 0.5 m/s，末端前进速度 0.25 m/s；max_speed 按 STG 速度检查，拒绝低于 0.5 m/s 的请求。
- 请求 pos_tol/yaw_tol 保留并记录，均不直接覆盖控制参数。2026-11-09 起执行 pre + 末端位置接近 + 最终 yaw 对准，完成一轮并停止等待后返回 arrived；不承诺请求容差已验收，实际 XY/yaw 误差另行记录。详见部署文档第 9.16 节。
- Feedback 最大约 10 Hz：stage 是实际状态（暂停时附原因）；current_pose、remaining_dist、remaining_yaw 为真实估计。位姿时间戳沿用最近输入的源时间戳，暂停时不把旧位姿改成新时间。
- 成功：ROS 状态 4，success=true，reason=arrived；Action 取消：状态 5；超时、RViz Stop/重画目标、定位停止、执行异常：状态 6。先发送零速度再结束。内部 plan_wait/障碍等待仍是任务执行过程，不自动判为失败。
- 整体超时采用单调时钟，包含等待和暂停时间。断线取消与心跳由 TCP 网桥处理；ROS 直连客户端断线本身不会触发立即取消，任务仍受整体超时限制。
- final_pose 是实际终点，internal_error 是实际 XY 估计误差；位姿陈旧时返回 pose_stale（没有位姿时 no_pose），internal_error=-1，禁止用于下一段位姿接续。
- uncertainty=-1 表示暂无估计；blocked_distance=-1、blocked_bearing=0 表示暂无对应障碍测量。普通导航失败原因还包括 no_stg、goal_redrawn、stopped_by_user、localization_stopped、navigation_error。
- 开启 `--record-experiment` 时，记录 Action Goal UUID、关联 trial、请求与有效容差、坐标变换、开始和结束事件；日志包包含接口及网桥源码。
- `/nav/planned_path` 是 StarCo 的可选可视化输入，本版没有新增此话题。现有 RViz STG 显示继续工作。

### 目标显示同步（2026-11-02，已同步 G1）

导航节点通过 `/g1/navigation/goal` 发布 map 系 `PoseStamped` 目标状态，使用
Reliable、Transient Local、depth 1。手动选择有效目标及成功启动导航（含 Action）时更新；
被拒绝的 Action 不替换已有目标。Live BEV 订阅该状态，更新红色箭头、面板目标和
`goal_pose.json`，RViz 后打开也能显示保留的最新箭头。箭头包含请求的 yaw。
2026-11-09 起控制器同时执行最终 yaw 对准，具体完成语义见上文与部署文档第 9.16 节。

该状态话题不触发导航、取消或定位重置。手动画目标仍使用
`/g1/live_bev/goal_2d`，保留取消旧任务的语义；不能用此输入话题转发 Action 目标。
目标显示更新只清除旧的手动规划缓存，不删除导航已经发布的紫色 STG。
部署时需同时更新 `g1_navigation_node.py` 与 `live_bev_node.py` 并重启本次 PosNav 会话。

外部人工目标客户端只连接已运行的 TCP 网桥，不启动网桥、FAST-LIO 或 PosNav。
`run_tcp_bridge.sh` 也不启动导航节点。当前桌面入口启动 Live BEV，后者拉起
`g1_navigation_node.py` 和 RViz；导航节点同时提供 Action Server。
浏览器访问 6081 仅连接已运行的桌面，不新建导航进程。

本地 ROS Jazzy / Fast DDS、localhost 隔离域验证共 123 项通过：Action/TCP 14 项
（新增目标显示链路 2 项），BEV 节点、导航状态机、STG 记录及 FMM 回归 109 项。
新增验证覆盖 scene→map 变换、显示节点及 Marker 订阅者后加入、目标同步不取消 Action
且不删除 STG、忙时拒绝不改目标，以及 Action→手动画目标→Action 的连续切换。
运动出口替换为内存接收器，未连接 G1，未做真机运动或 RViz 图形窗口验收。

随后已将两个节点脚本、Action 测试、本接口文档及部署文档第 6 节同步到 G1。
同步前核对 5 个机载旧文件与上一部署版本一致，旧文件及两份部署清单备份在机载
`RealWorldG1/.cache/goal_display_backups/goal_display_deployment_20261102T133021Z/`。
机载 Humble / CycloneDDS 在 `ROS_DOMAIN_ID=157`、`ROS_LOCALHOST_ONLY=1` 下通过
上述全部 **123 项测试**，运动出口替换为内存记录。部署清单按原有覆盖范围更新。
同步时正式 PosNav 未运行；只启动空闲容器并执行隔离测试，未启动正式桌面、定位、
网桥或真机运动，已有 FAST-LIO 保持运行。下次按部署文档第 6 节启动即加载新实现；
红箭头的现场 GUI 效果仍待实际导航时确认。

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


## 近场脱困反馈（2026-11-09）

运控新增定时平移脱困；触发、扇区和次数规则见部署文档第 9.15 节。
Action 仍持有原目标，脱困方向/等待通过既有 Feedback `stage` 字符串上报，
例如 `recovery_left (1/3)`、`recovery_settle (1/3)`，输入失效时仍附暂停原因。
位置、剩余距离和 yaw 均对原目标计算，不把脱困方向当成新导航目标。
第三次脱困后仍可恢复正常规划；再次需要脱困时返回 ROS status 6、
`success=false, reason=recovery_exhausted`。取消和整体超时包含脱困及其等待时间。
TCP 网桥不需要修改，客户端可直接显示 stage，或增加中文名称映射；`.action` 无需重编译。
本功能采用固定 0.5 m/s、1.0 s 平移和 0.5 s 停止等待，完整参数及验证边界以部署文档为准。


2026-11-09 已同步机载；独立临时副本的 179 项机载隔离测试全部通过，包含脱困
Feedback、取消/整体超时、`recovery_exhausted` 终态和非零 vy 的指令/日志。
无真实机器人运动；备份与验证边界详见部署文档第 9.15 节。


## 完整末端精调与完成语义（2026-11-09）

当前生产控制器增加 pre/STG 交接、位置接近和最终 yaw 阶段，参数与受阻规则见
部署文档第 9.16 节。Feedback stage 新增 fine_approach、fine_obstacle、
fine_position_settle、heading_align、fine_yaw_settle。原始 Goal、坐标变换、取消、
整体超时及红色目标显示保持原语义；无需重编译 .action 或修改 TCP 网桥。

为保持客户端互通，完成一轮末端流程后仍返回 ros_goal_status=4、success=true、
reason=arrived，但“到达”的有效规则已由历史 XY≤0.50 m 改为末端流程完成。
pos_tol/yaw_tol 是请求记录，不代表本轮已满足的误差上限。
final_pose/internal_error 保持定位估计含义；源位姿陈旧时仍禁止报告成功。
Action 事件记录 effective_pos_tol=null、position_tolerance_checked=false、
final_yaw_control=true 及实际控制的 yaw_stop_tol_rad=0.1。
停后实际 yaw 误差可由 final_pose 与 g_yaw 计算，实验结果也直接保存。
客户端应更新有效规则说明，不能继续标注“不控制最终 yaw”或将请求容差视为验收结果。


本功能已于 2026-11-09 同步机载，200 项机载隔离回归及真实 ROS/DDS + 模拟机体闭环通过。
未执行实际机器人运动或新的末端精度验收，备份及测试记录见部署文档第 9.16 节。
