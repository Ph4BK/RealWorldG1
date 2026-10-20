# full_lab_v1 正式地图包

三维定位地图与静态导航 BEV 共用 map 坐标系，单位米，拟合地面为 z=0。

## 正式文件

- map.pcd：固定三维定位地图，保持已有地图内容及路径不变。
- bev_occupancy.npz：静态未膨胀占据栅格，含 occ（bool）、origin、res。
- bev_config.json：BEV 的完整生成参数、坐标约定及源 PCD 校验和。
- metadata.yaml：三维地图生成记录；现有定位初始化程序还读取 initial_imu_xyz_map 的高度，必须保留。
- T_map_from_camera_init.txt、T_map_from_gravity_level.txt：原建图坐标转换，不是每次启动的新定位变换。
- provenance/mapping_config.yaml、provenance/freeze_reference_map.py：原三维建图配置及固定地图生成脚本。
- package_manifest.json：正式文件清单与校验和；只部署这里列出的文件及本清单即可。
- checksums.sha256：正式文件的 SHA-256 校验清单。

正式 BEV：高度 0.05 <= z <= 1.3 m；5 cm/格；每格点数 >= 2 为障碍，其余格子均为空闲。
点数来自已降采样的 map.pcd，不是观测帧数。occ[row, col] 的行对应 Y，列对应 X。
shape=204×204，左下角 origin=(-1.7,-7.1) m；没有膨胀，也不含未知类别。
每次任务重新加载为内存地图，先融合实时观测，再膨胀用于规划；不回写磁盘静态图。

## 按需重建 BEV 与预览

在仓库根目录，使用安装了 numpy/open3d 的 Python（预览另需 matplotlib/Pillow）：

```bash
python RealWorldG1/tools/build_static_bev.py --output-dir /tmp/g1-static-bev --counts --preview
```

输出目录中不能已有同名产物。重建只依赖正式 map.pcd 与 bev_config.json，不依赖 diagnostics。
counts 和 PNG 是按需生成的派生文件，不属于正式归档。脚本保留在仓库 tools/ 中。

## 原三维地图的来源

原始录制为 RealWorldG1/datasets/full_lab_walk/bag，保留在地图包外。
用 tools/replay_fastlio_bag.py 和 provenance/mapping_config.yaml 回放生成累计点云与轨迹；
用 tools/audit_lidar_imu_bag.py、tools/plot_fastlio_bev.py 得到重力调平变换；
再用归档的 provenance/freeze_reference_map.py 生成固定地图。原算法环境也需要保留。
回放及重新拟合的结果不承诺逐字节相同，因此正式 map.pcd 与变换始终保留，不用重建结果自动替换。

本包旧版 source/、PNG、trajectory_map.csv 及已批准的静态建图实验输出已删除。
删除记录见 RealWorldG1/maps/static_map_cleanup_review.json；原始 bag、重定位结果和机载部署性能记录保留。
metadata.yaml 内 source_cloud/source_run 是历史来源记录，清理后不保证这些中间路径仍存在。
