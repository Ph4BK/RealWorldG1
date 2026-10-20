# 实验结果归档

本目录在 Git 中只收录实验报告、关键摘要和必要轨迹/配图。
原始 bag 另行交接；详细运行日志、逐帧性能采样和中间点云不纳入普通 Git，
已有本地文件继续保留。性能摘要是当次测量记录，仅凭摘要无法重新计算全部指标。

## 重定位回放

`relocalization_1/`、`relocalization_2/`、`relocalization_3/` 各保留：

- `replay/summary.json`：流程状态、位姿数、耗时和配准 fitness 范围。
- `replay/localization.csv`：地图坐标系中的定位轨迹。
- `replay/localization_bev.png`：固定地图上的轨迹预览。

2026-10-20 检查时三组摘要均为 `success: true`。第 2 组由用户当日重新回放，
新结果为 345 个里程计位姿、332 个全局定位位姿，已生成轨迹图。
它替换了此前被中断的运行结果；用户说明更早也曾成功运行。
回放工具每次开始会覆盖该数据集的旧结果，本目录不是全部运行历史。
`success` 表示流程完成，`quality_validated: false` 表示尚无独立真值精度验收。

实验初值和人工路线核对图见 `../datasets/relocalization_N/`；
人工图不是精确轨迹真值。正式地图见 `../maps/full_lab_v1/`。

## 机载性能与历史排查

`g1_deployment_20260927/REPORT.md` 及所收录的配置、JSON 摘要记录
2026-09-27 的机载 FAST-LIO 实时测试和一倍速回放结果。
文中进程、容器和“留运行”状态仅指当次测试结束时，不代表当前机载状态。
详细 CSV、日志、`map_raw.pcd` 和 `registered_xyz.npy` 未随 Git 分发。

静止漂移排查的报告、配置、对比图和数值摘要保存在
`../fastlio/records/static_20260927/`。历史文档提到的
`static_20260927/full_corrected/`、`voxel_grid_20260927/`、
`full_lab_walk/` 等旧诊断目录已清理，不能作为当前可用路径。
2026-09-22 的离线准备文档和更早输入审计路径也属于历史记录；
当前部署状态以 `../docs/G1_PosNav真机部署.md` 为准。
