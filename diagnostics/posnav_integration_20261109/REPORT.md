# G1 与陈梓昕客户端联调：两次到达与一次前进脱困

整理日期：2026-11-09（北京时间）。实际实验日期、起止时间未提供；目录日期是归档日期。

## 材料来源

用户将本终端输出作为两次成功真机任务及与陈梓昕联调的材料提供。
原文件逐字节复制为 [navigation_status_terminal.txt](navigation_status_terminal.txt)，保留原有 CRLF、
命令提示符和省略号。SHA-256 及整理文件的校验值见 [checksums.sha256](checksums.sha256)。
机器可读摘要见 [summary.json](summary.json)。

原文共 49 条状态消息，**49 条均含字符串截断 `...`**，不能将其解析为完整 JSON，
也不能补回省略字段。本文只提取明确可见的 state、message 等片段。
没有时间戳、目标位姿、Action UUID、TCP/Action 终态、指令流或地图快照。
文中的“任务 1/2”为按消息顺序划分的归档编号，不是服务端 trial 或请求标识。

## 可见结果

| 任务 | 原文起止行 | 状态消息数 | 观察到的控制器过程 | 终态 |
|---|---|---:|---|---|
| 1 | 2–40 | 20 | 多段 STG 转向、前进，包含一次 obstacle 状态；未见脱困状态 | Arrived within 0.50 m |
| 2 | 44–98 | 28 | 多段 STG；一次前进脱困、恢复导航；后续曾 Waiting for pose 并恢复 | Arrived within 0.50 m |

两次任务之间还有一条 Goal redrawn 状态，不计入上述任务内消息数。
obstacle 表示控制器进入障碍等待，不据此认定碰撞；Waiting for pose 表示反馈等待，
没有时间戳不能量化其持续时间。

第二次任务中的关键状态链路：

```text
plan
→ recovery_prepare     Preparing recovery (1/3)                 原文第 68 行
→ recovery_forward     Recovery forward (1/3)，计数可见为 1     原文第 70 行
→ recovery_settle      Timed motion complete                    原文第 72 行
→ turn / turn_settle / obstacle / walk / walk_settle
→ 后续 STG 导航（含一次 Waiting for pose 后恢复）
→ idle                Arrived within 0.50 m                     原文第 98 行
```

## 组会上可使用的表述

> 现场完成两次 G1 导航任务，控制器均按目标 XY 0.50 m 容差报告到达。
> 第二次任务触发一次向前脱困，完成定时动作及停止等待后恢复 STG 导航，最终到达。
> 结合现场执行反馈，该记录支持“前进脱困与正常导航衔接已获得初步真机功能验证”。

与陈梓昕联调的目标来源依据用户现场说明。该话题自身不标识目标是来自 Action 还是 RViz，
若展示完整的“客户端发送—网桥—Action—执行—结果回传”证据，需配套客户端日志或截图，
最好保留相同请求 UUID 的 terminal.json / events.jsonl。

## 结论范围

- 支持展示两次控制器到达及其中一次前进脱困后继续完成任务的功能链路。
- 不能从状态输出单独测得实际平移距离、0.5 m/s 实际速度、1.0 s 动作时长或 0.5 s 等待时长；这些是部署参数，日志没有时间依据。
- 不能据此确认近场占据一定来自噪声，或复算四扇区占据格数量及选向正确性；截断部分没有恢复。
- 未覆盖左/右/后脱困、三次上限、次数清零、脱困取消和整体超时的真机验收；不能用这两次任务估计总体成功率。
- Arrived 表示定位估计下的 XY 容差达标，不代表外部真值精度或最终 yaw 达标。

## 归档与后续记录

本目录的报告、摘要和校验清单纳入普通 Git；原始终端文本按 diagnostics 的运行日志规则
仅保留本地，分享组会材料或交接时需一并复制，单独克隆仓库不会获得该原文。
不放入 `.cache`，也不伪装为 `posnav_experiments` 自动生成的完整实验包。

下次在已加载 G1 ROS 环境的终端中，可使用全长字符串输出，并选择一个新的文件名：

```bash
ros2 topic echo /g1/navigation/status --full-length | tee navigation_status_run03.txt
```

该命令仍不额外添加消息时间戳。需要复核触发条件、速度指令、地图与轨迹时，
启动 PosNav 时加 `--record-experiment`，并保存客户端任务日志及现场视频。
