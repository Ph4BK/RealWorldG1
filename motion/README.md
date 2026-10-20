# G1 基础动作与标定入口

机载统一目录：`/home/unitree/g1_posnav/RealWorldG1`。
下列命令先在 G1 PC2 上执行 `cd /home/unitree/g1_posnav`，不会自动从电脑 SSH 到机器人。
本目录负责单次动作与标定；连续导航由 `tools/g1_navigation_node.py` 执行，
其持续刷新速度及主动零速度停止逻辑见 [PosNav 部署说明](../docs/G1_PosNav真机部署.md)。

采用已在本机验证的 **FSM 501 + 官方 ROS 2 C++ LocoClient**。
前进、后退、左转、右转正常只发送一次速度与持续时间，不追加零速度；
中断、请求异常、缺少及时响应或里程计失效时仍尝试零速度回退。
现有单次现场验证不等于所有参数、重复性或其他固件均已验证。

## 开机操作（本机三腰 G1）

| 顺序 | 遥控操作 | 模式 | 实测 FSM |
| --- | --- | --- | --- |
| 1 | L2 + B | 阻尼 | 1 |
| 2 | L2 + ↑ | 预备 | 4 |
| 3 | R1 + Y，按住 2 秒 | 常规运控 | 501 |

程序只查询并要求 501，不自动切模式。待姿态稳定、净空及现场接管就绪后执行。

## 使用

优先使用具名参数入口，见 [基本动作标定](../docs/基本动作标定.md)：

```bash
python3 RealWorldG1/tools/calibrate_g1_motion.py --speed 0.2 --duration 2
```

**有效命令直接执行，没有预览或执行开关。普通模式静默且不保存实验数据。**
加 `--calibrate` 才记录 PosNav 到期后 0–2 s 的位姿和差分速度，保存 JSON/CSV，实验结束后用 `--plot` 汇总绘图；
采集标定数据要求 PosNav 定位已运行；`--plot` 只读取已有记录，无需定位或机器人运动，
绘图需要 matplotlib，详见上面的基本动作标定说明。
底层入口也直接执行，位置参数为 vx(m/s)、vy(m/s)、yaw_rate(rad/s)、duration(s)；
兼容旧的三个位置参数（省略 vy，默认为 0）：

```bash
bash RealWorldG1/tools/run_g1_motion.sh 0.2 0 2
```

Python 对象构造本身不运动，调用 move/turn/pulse 就会执行：

```python
from RealWorldG1.src.g1_motion_executor import G1MotionExecutor
executor = G1MotionExecutor()
result = executor.move(vx=0.2, duration=2.0)
# 单独站稳后再调用下一次，不要直接循环执行
```

Python move() 默认 vx=0.2、duration=1；turn() 默认 yaw_rate=0.15、duration=1。
命令行必须显式提供参数，不使用这些 Python 默认值。

## 校验、返回与限制

- 参数须为有限数、时长大于 0；允许 vx+vy 平移，不允许非零平移与转向组合；没有应用层速度/时长上限。
- `move(vx=..., vy=..., duration=...)` 支持组合平移；横移及组合平移尚未真机验证。
- 要求 FSM 501、新鲜且静止的起步里程计；0.25 秒未更新则异常回退。
- 15 cm / 0.2 rad 位移截断已移除；绳索不能保证避免摆动或碰撞。
- 四种非零单轴动作正常到期不追加零速度。零速度显式请求及异常回退仍可用。
- 回退不等待原运动 ACK；官方响应等待最长 5 秒，不是硬实时停止机制。
- SIGINT/SIGTERM 会请求回退；断网、崩溃和 SIGKILL 无法保证停止。
- command_completed 仅表示指令流程成功，不证明最终距离或已站稳。
- 普通动作不再返回 `odom_before`；标定模式返回 PosNav map 系平面位姿序列，起步保护仍读取 `/dog_odom`。
- `calibrate_g1_motion.py --odom` 或底层 `run_g1_motion.sh --odom` 仅查询当前 odom，不发任何 sport 请求。
- 主机锁防止使用同一部署根目录的本入口并发执行，不能阻止独立导航节点、其他程序或遥控器。
- 每次调用启动容器及 ROS 节点；当前适合手动单动作标定，不是高频速度流。

历史实验入口 `--duration-only-test` 及其额外 3 秒观察逻辑已删除；新的 `--calibrate` 单独采集到期后 0–2 s；
`--execute`、`--dry-run`、`--log` 也不再是正式入口参数。
旧文档中的命令仅是实验历史，不能按旧方式使用。

## 断网自测

```bash
bash RealWorldG1/tools/run_g1_motion.sh --self-test
```

这是断网维护模式，使用 Docker --network none 和模拟服务，不发真机指令。
测试覆盖正常单次请求、FSM 错误、无/过期里程计、拒绝响应、响应缺失、中断，
以及旧参数拒绝。构建缓存持久化；测试使用模拟服务，不等同于真机动作验收。

横向参数新增后的验证状态见 [连接说明](../docs/G1连接方式与状态确认.md)。

## 已验证状态与数据位置

2026-10-12 的标定工具部署通过官方镜像编译、28 项机载 Python 单元测试及
26 个断网集成场景。随后已完成前进、左转、右转各 10 次真机标定并选定初版控制参数；
连续 STG 导航的真机功能验证另见 PosNav 部署说明第 9.9 节。

2026-10-20，早期运控部署内容合入上述正式目录。30 组原始 JSON/CSV 及最新
`Forward velocity` 汇总图统一保存在 `RealWorldG1/calibration_results/`，与本地内容一致。
仅在 G1 上通过 `calibrate_g1_motion.py --plot` 生成汇总后复制回本地，原始记录不改写。
