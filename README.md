# RealWorldG1

Unitree G1 真机定位、实时 BEV 与位置导航工作区，已形成包含末端精调的完整 PosNav 实验实现。

导航流程已包含 FMM/STG 到 pre、末端位置接近、最终 yaw 对准和停止等待，
并提供近场脱困、ROS 2 Action/TCP 接口、RViz 目标同步及实验记录。
本版已完成真机功能验证；`arrived` 表示一轮流程完成，请求的位置与 yaw 容差尚不作为正式验收条件。

## 使用入口

- [G1 PosNav 真机部署与日常操作](docs/G1_PosNav真机部署.md)：环境、启动顺序、参数和控制流程。
- [Action 与 TCP 联调](docs/G1_Action接口与TCP联调.md)：任务规划层接入、状态与结果语义。
- [PosNav 运行环境](posnav/README.md)：机载容器、显示和构建入口。
- [实验结果归档](diagnostics/README.md)及[本版最终位姿实验报告](diagnostics/posnav_final_approach_20261109/REPORT.md)。
- [变更记录](CHANGELOG.md)。

## 当前验证结果

2026-11-09 的同一组 8 次完整任务中，最终 map 定位坐标的 L2 误差平均 **8.13 cm**
（范围 1.93–17.36 cm），yaw 绝对误差平均 **3.70°**（范围 0.99–6.19°）。
结果在最终 yaw 停止等待后统计，不代表独立物理真值或已协商的上层验收标准。
逐次结果、计算脚本和图表随报告保留。

## Git 归档范围

Git 保留运行源码、配置、正式地图目录、测试、部署清单及选定实验报告/CSV/配图。
原始 `posnav_experiments/`、`.cache/`、完整组件日志、构建环境、`maps.zip` 和本地参考资料
由 `.gitignore` 排除。分析脚本需配套本地原始实验包才能从头重算，报告内说明了来源。
