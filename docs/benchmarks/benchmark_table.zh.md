# 基准结果

ATE 是完成时间关联（误差不超过 0.1 s）和 SE(3) Umeyama 对齐后的平移 RMSE，
不做尺度修正。数值越低越好。

HILTI 使用稀疏控制点 GT；S3E 与 Newer College 使用连续 GT。README 的基准快照
剔除了两条 ATE 约为 0.5 m 的较长 Newer College 轨迹，这里保留完整明细。

## 逐序列 ATE

| 数据集 | 序列 | 机器人 | GT 长度 (m) | 估计轨迹长度 (m) | 指标 | 结果 (m) |
|---|---|---:|---:|---:|---|---:|
| HILTI | Site 1 · Handheld 1 | reference | 81.32 | 205.5 | 稀疏 ATE | 0.030 |
| HILTI | Site 1 · Handheld 2 | reference | 84.39 | 163.9 | 稀疏 ATE | 0.052 |
| HILTI | Site 1 · Handheld 3 | reference | 81.89 | 165.2 | 稀疏 ATE | 0.108 |
| HILTI | Site 1 · Handheld 4 | reference | 52.88 | 300.0 | 稀疏 ATE | 0.053 |
| HILTI | Site 1 · Handheld 5 | reference | 45.94 | 150.1 | 稀疏 ATE | 0.045 |
| S3E | Campus Road 3 | Alpha | 983.3 | 940.0 | ATE | 0.876 |
| S3E | Campus Road 3 | Carol | 986.9 | 948.9 | ATE | 1.050 |
| S3E | Dormitory 1 | Alpha | 727.0 | 724.0 | ATE | 0.478 |
| S3E | Dormitory 1 | Bob | 719.3 | 722.6 | ATE | 1.378 |
| S3E | Dormitory 1 | Carol | 721.9 | 738.0 | ATE | 0.804 |
| S3E | Library 1 | Alpha | 507.6 | 520.4 | ATE | 0.988 |
| S3E | Library 1 | Bob | 517.2 | 530.6 | ATE | 1.671 |
| S3E | Library 1 | Carol | 498.9 | 508.1 | ATE | 1.339 |
| S3E | Library 2 | Alpha | 521.6 | 505.0 | ATE | 1.238 |
| S3E | Library 2 | Bob | 523.9 | 505.5 | ATE | 1.683 |
| S3E | Library 2 | Carol | 520.3 | 505.8 | ATE | 0.923 |
| S3E | Square 1 | Alpha | 546.0 | 542.7 | ATE | 1.166 |
| S3E | Square 1 | Bob | 496.5 | 495.9 | ATE | 0.748 |
| S3E | Square 1 | Carol | 529.2 | 531.7 | ATE | 0.866 |
| S3E | Square 3 | Alpha | 487.4 | 463.7 | ATE | 1.835 |
| S3E | Square 3 | Bob | 569.6 | 559.8 | ATE | 1.932 |
| S3E | Square 3 | Carol | 563.8 | 541.0 | ATE | 0.912 |
| Newer College | Short Experiment | reference | 1609.4 | 1422.7 | ATE | 0.502 |
| Newer College | Long Experiment | reference | 3063.4 | 2603.4 | ATE | 0.494 |
| Newer College | Quad with Dynamics | reference | 479.0 | 476.0 | ATE | 0.148 |
| Newer College | Dynamic Spinning | reference | 97.20 | 71.48 | ATE | 0.100 |
| Newer College | Parkland Mound | reference | 695.7 | 654.8 | ATE | 0.182 |

## 轨迹对比

蓝色实线为 Sapphire，橙色虚线为 GT，叉号表示 HILTI 控制点。

### HILTI 2023

| 轨迹结果 | 轨迹结果 |
|---|---|
| ![HILTI · Site 1 · Handheld 1](trajectories/hilti_site1_handheld_1.png) | ![HILTI · Site 1 · Handheld 2](trajectories/hilti_site1_handheld_2.png) |
| ![HILTI · Site 1 · Handheld 3](trajectories/hilti_site1_handheld_3.png) | ![HILTI · Site 1 · Handheld 4](trajectories/hilti_site1_handheld_4.png) |
| ![HILTI · Site 1 · Handheld 5](trajectories/hilti_site1_handheld_5.png) | |

### S3E

| 轨迹结果 | 轨迹结果 |
|---|---|
| ![S3E · Campus Road 3 · Alpha](trajectories/s3e_campus_road_3_alpha.png) | ![S3E · Campus Road 3 · Carol](trajectories/s3e_campus_road_3_carol.png) |
| ![S3E · Dormitory 1 · Alpha](trajectories/s3e_dormitory_1_alpha.png) | ![S3E · Dormitory 1 · Bob](trajectories/s3e_dormitory_1_bob.png) |
| ![S3E · Dormitory 1 · Carol](trajectories/s3e_dormitory_1_carol.png) | ![S3E · Library 1 · Alpha](trajectories/s3e_library_1_alpha.png) |
| ![S3E · Library 1 · Bob](trajectories/s3e_library_1_bob.png) | ![S3E · Library 1 · Carol](trajectories/s3e_library_1_carol.png) |
| ![S3E · Library 2 · Alpha](trajectories/s3e_library_2_alpha.png) | ![S3E · Library 2 · Bob](trajectories/s3e_library_2_bob.png) |
| ![S3E · Library 2 · Carol](trajectories/s3e_library_2_carol.png) | ![S3E · Square 1 · Alpha](trajectories/s3e_square_1_alpha.png) |
| ![S3E · Square 1 · Bob](trajectories/s3e_square_1_bob.png) | ![S3E · Square 1 · Carol](trajectories/s3e_square_1_carol.png) |
| ![S3E · Square 3 · Alpha](trajectories/s3e_square_3_alpha.png) | ![S3E · Square 3 · Bob](trajectories/s3e_square_3_bob.png) |
| ![S3E · Square 3 · Carol](trajectories/s3e_square_3_carol.png) | |

### Newer College 2020

| 轨迹结果 | 轨迹结果 |
|---|---|
| ![Newer College · Short Experiment](trajectories/newer_college_01_short_experiment.png) | ![Newer College · Long Experiment](trajectories/newer_college_02_long_experiment.png) |
| ![Newer College · Quad with Dynamics](trajectories/newer_college_05_quad_with_dynamics.png) | ![Newer College · Dynamic Spinning](trajectories/newer_college_06_dynamic_spinning.png) |
| ![Newer College · Parkland Mound](trajectories/newer_college_07_parkland_mound.png) | |
