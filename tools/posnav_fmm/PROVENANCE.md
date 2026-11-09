# 算法来源

2026-10-06 从本仓库 Go2 实现复制，运行时不引用 RealWorldG1 外的代码。

- `src/utils/fmm/fmm_planner_policy.py` SHA-256: `3c1372973ccf51e6c0d79768eba4d8072d97c150fc16e01786ff14e70bac9487`
- `src/envs/real_world_local_planner.py` SHA-256: `aac9ed808560f20d65533878ef98f87957e6b5c40375d38220fc93bc65cd9d36`
- `src/agent/unigoal/agent.py` SHA-256: `a17e8c12c5856d3b88044b17205146de1f5cdc5e9dc342c631c1d179aaa04114`

保留 FMM mask/dist、set_multi_goal、STG 数值算法及 supercover 选点；删去未使用的另一版选点、自动移动目标和缩放分支及无关依赖。包装层参数由 G1 配置指定。

2026-10-26 G1 包装层调整：移除起点 3×3 强制清空；macro 直线检查覆盖第一个候选；
全部候选失败时复用同一距离场，从原起点按 1 格递减搜索到 4 格，每个半径仅检查最优候选。
删除独立的 2 格回退，不施加最小实际距离约束。FMM 数值内核保持不变。

2026-11-09 末端精调适配：内核不再以 FMM 剩余距离小于 0.25 m 返回 stop；
包装层在原始 XY 距离不超过 0.50 m 且膨胀图整段可通行时，直接返回原始目标坐标作为
最后 STG。去除包装层近目标距离比大于 2 时的停止，允许近处障碍需要绕行时继续规划。
圆周候选、距离场数值计算、缩步和 supercover 通行检查沿用原算法。
