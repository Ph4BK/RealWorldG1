# relocalization_3 人工轨迹标注

[relocalization_3.png](relocalization_3.png) 对应同级上层目录中的 `bag/`。
由用户在 full_lab_walk 重力调平 BEV 上手动绘制，表示大致行走路线，供重定位结果目视核对。
不是精确轨迹真值，不提供逐帧时间对应或精确姿态标定；无初值全局重定位时不作为定位初值输入。
底图来自 `diagnostics/full_lab_walk/mapping/bev_xy_leveled_detail_no_trajectory.png`。
