# G1 雷达与 IMU 数据集

Git 仅收录本说明、bag 的 `metadata.yaml`、已有传输校验文件、重定位初值和人工标注。
原始 `.db3` bag 保留在本地及机载数据目录，需另行交接；仅克隆仓库不能直接执行 bag 回放。
校验文件中的路径是当次传输记录，不保证在接收方机器上存在。

| 目录 | 录制范围 | 时长 |
|---|---|---|
| small_area_walk/bag | 机器人小范围走动 | 约 30 秒 |
| full_lab_walk/bag | 整个实验室范围走动 | 约 108 秒 |

本地根目录：RealWorldG1/datasets/；G1 根目录：/home/unitree/g1_diagnostics/。
两端使用相同目录名。仅重命名外层目录，bag 内部文件及时间戳保持原样。
历史实验报告中的原始输入路径可能使用改名前名称；小范围数据对应原先的 30 秒输入。

重定位验证数据：`relocalization_1/bag`，本地与 G1 同名；传输校验和见该目录的 transfer_sha256.json。

重定位验证数据：`relocalization_2/bag`，本地与 G1 同名；传输校验和见该目录的 transfer_sha256.json。

重定位验证数据：`relocalization_3/bag`，本地与 G1 同名；传输校验和见该目录的 transfer_sha256.json。

三份重定位数据的人工轨迹核对图分别保存在各自 `annotations/relocalization_N.png`（N=1,2,3），仅用于事后目视核对，不作为全局重定位初值或精确真值。
