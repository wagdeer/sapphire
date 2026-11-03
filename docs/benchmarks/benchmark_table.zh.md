# 基准结果

ATE 是完成时间关联和 SE(3) Umeyama 对齐后的平移 RMSE，不做尺度修正。
数值越低越好。

HILTI 2022 和 HILTI 2023 使用稀疏测量控制点，最大时间关联间隔为 2.0 s；
S3E 和 Newer College 使用连续 GT，最大时间关联间隔为 0.1 s。因此稀疏控制点
与连续 GT 的结果不能视为完全相同的评估协议。

汇总统计采用各数据集明确声明的纳入范围；部分覆盖的结果保留并标记为
`partial`；本地没有完整数据包的序列标记为 `未评估`。Newer College 的五条
有效轨迹全部纳入统计，不根据 ATE 高低剔除结果。

## 汇总结果

| 数据集 | 纳入轨迹 | 纳入 GT 覆盖 | 平均 ATE (m) | ATE 中位数 (m) |
|---|---:|---:|---:|---:|
| HILTI 2022 | 10 | 103 个控制点 | 0.260 | 0.082 |
| HILTI 2023 | 5 | 346.4 m 控制点路径 | 0.058 | 0.052 |
| S3E | 17 | 10.42 km | 1.17 | 1.05 |
| Newer College 2020 | 5 | 5.94 km | 0.2846 | 0.1817 |

## 逐序列 ATE

HILTI 2022 结果使用数据集专用手持平台配置和稀疏控制点 GT。估计位姿在
SE(3) 对齐前从 IMU 坐标系转换到测量端点。回放使用 0.5× 序列速率以保证帧
处理稳定，因此这里反映的是轨迹精度，不代表实时吞吐性能。

| 数据集 | 序列 | 机器人 | GT 覆盖 | 估计轨迹长度 (m) | 指标 | ATE RMSE (m) | 状态 |
|---|---|---:|---:|---:|---|---:|---|
| HILTI 2022 | exp01_construction_ground_level | reference | 13/13 | — | 稀疏 ATE | 0.016 | 完整 |
| HILTI 2022 | exp02_construction_multilevel | reference | 22/22 | — | 稀疏 ATE | 0.098 | 完整 |
| HILTI 2022 | exp03_construction_stairs | reference | 17/17 | — | 稀疏 ATE | 0.113 | 完整 |
| HILTI 2022 | exp04_construction_upper_level | reference | 7/7 | — | 稀疏 ATE | 0.031 | 完整 |
| HILTI 2022 | exp05_construction_upper_level_2 | reference | 6/6 | — | 稀疏 ATE | 0.019 | 完整 |
| HILTI 2022 | exp06_construction_upper_level_3 | reference | 7/7 | — | 稀疏 ATE | 0.033 | 完整 |
| HILTI 2022 | exp07_long_corridor | reference | 6/6 | — | 稀疏 ATE | 0.114 | 完整 |
| HILTI 2022 | exp09_cupola | reference | 16/16 | — | 稀疏 ATE | 1.810 | 完整 |
| HILTI 2022 | exp10_cupola_2 | reference | 3/14 | — | 稀疏 ATE | 0.095 | 部分覆盖 |
| HILTI 2022 | exp11_lower_gallery | reference | 5/5 | — | 稀疏 ATE | 0.296 | 完整 |
| HILTI 2022 | exp14_basement_2 | reference | 4/4 | — | 稀疏 ATE | 0.066 | 完整 |
| HILTI 2022 | exp15_attic_to_upper_gallery | reference | — | — | 稀疏 ATE | — | 未评估 |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | reference | 7/9 | — | 稀疏 ATE | 0.320 | 部分覆盖 |
| HILTI 2022 | exp18_corridor_lower_gallery_2 | reference | — | — | 稀疏 ATE | — | 未评估 |
| HILTI 2022 | exp21_outside_building | reference | — | — | 稀疏 ATE | — | 未评估 |
| HILTI 2022 | exp23_the_sheldonian_slam | reference | — | — | 稀疏 ATE | — | 未评估 |
| HILTI 2023 | Site 1 · Handheld 1 | reference | 81.32 m | 205.5 | 稀疏 ATE | 0.030 | 完整 |
| HILTI 2023 | Site 1 · Handheld 2 | reference | 84.39 m | 163.9 | 稀疏 ATE | 0.052 | 完整 |
| HILTI 2023 | Site 1 · Handheld 3 | reference | 81.89 m | 165.2 | 稀疏 ATE | 0.108 | 完整 |
| HILTI 2023 | Site 1 · Handheld 4 | reference | 52.88 m | 300.0 | 稀疏 ATE | 0.053 | 完整 |
| HILTI 2023 | Site 1 · Handheld 5 | reference | 45.94 m | 150.1 | 稀疏 ATE | 0.045 | 完整 |
| S3E | Campus Road 3 | Alpha | 983.3 m | 940.0 | ATE | 0.876 | 完整 |
| S3E | Campus Road 3 | Carol | 986.9 m | 948.9 | ATE | 1.050 | 完整 |
| S3E | Dormitory 1 | Alpha | 727.0 m | 724.0 | ATE | 0.478 | 完整 |
| S3E | Dormitory 1 | Bob | 719.3 m | 722.6 | ATE | 1.378 | 完整 |
| S3E | Dormitory 1 | Carol | 721.9 m | 738.0 | ATE | 0.804 | 完整 |
| S3E | Library 1 | Alpha | 507.6 m | 520.4 | ATE | 0.988 | 完整 |
| S3E | Library 1 | Bob | 517.2 m | 530.6 | ATE | 1.671 | 完整 |
| S3E | Library 1 | Carol | 498.9 m | 508.1 | ATE | 1.339 | 完整 |
| S3E | Library 2 | Alpha | 521.6 m | 505.0 | ATE | 1.238 | 完整 |
| S3E | Library 2 | Bob | 523.9 m | 505.5 | ATE | 1.683 | 完整 |
| S3E | Library 2 | Carol | 520.3 m | 505.8 | ATE | 0.923 | 完整 |
| S3E | Square 1 | Alpha | 546.0 m | 542.7 | ATE | 1.166 | 完整 |
| S3E | Square 1 | Bob | 496.5 m | 495.9 | ATE | 0.748 | 完整 |
| S3E | Square 1 | Carol | 529.2 m | 531.7 | ATE | 0.866 | 完整 |
| S3E | Square 3 | Alpha | 487.4 m | 463.7 | ATE | 1.835 | 完整 |
| S3E | Square 3 | Bob | 569.6 m | 559.8 | ATE | 1.932 | 完整 |
| S3E | Square 3 | Carol | 563.8 m | 541.0 | ATE | 0.912 | 完整 |
| Newer College | Short Experiment | reference | 1609.4 m | 1422.7 | ATE | 0.502 | 完整 |
| Newer College | Long Experiment | reference | 3063.4 m | 2603.4 | ATE | 0.494 | 完整 |
| Newer College | Quad with Dynamics | reference | 479.0 m | 476.0 | ATE | 0.148 | 完整 |
| Newer College | Dynamic Spinning | reference | 97.20 m · 81.6% | 71.01 | ATE | 0.098 | 最新重跑 · 部分覆盖 |
| Newer College | Parkland Mound | reference | 695.7 m | 654.8 | ATE | 0.182 | 完整 |

`exp10` 在约 88 s 后丢跟踪，只覆盖前 3/14 个控制点；`exp16` 在约 156 s 后
丢跟踪，只覆盖 7/9 个控制点。两条结果的 RMSE 只描述已匹配前缀，不参与完整
序列汇总。四个未评估序列目前在本地没有完整 ROS bag。

对于稀疏控制点 HILTI，GT 长度是测量控制点折线长度，而估计轨迹长度覆盖完整
回放过程，因此两列不能直接作为同一口径比较。

`Dynamic Spinning` 最新重跑覆盖 81.6% 的 GT 轨迹，开头约 4.7 s 用于 IMU
初始化，ATE 为 0.0979 m。使用同一版当前评估器重算时，上一版轨迹覆盖 91.1%，
ATE 为 0.1004 m。因此本次是较短匹配区间上的小幅改善，不能视为完整覆盖率下
的直接对比。

表格采用各序列最新重跑结果，并标注覆盖状态。为便于判断可复现性，`exp03`
另外两次完整运行的 RMSE 为 0.119 m 和 0.114 m，而更早的批量回放结果为
2.021 m；`exp11` 早先一次完整运行为 0.437 m，最新完整运行为 0.296 m。
单独一次最好结果不足以代表运行稳定性，因此这些波动在此明确保留。

## 轨迹对比

蓝色实线为 Sapphire，橙色虚线为 GT，叉号表示 HILTI 控制点。

### HILTI 2022

| 轨迹结果 | 轨迹结果 |
|---|---|
| ![HILTI 2022 · exp01 施工场地地面层](trajectories/hilti_2022_exp01_construction_ground_level.png) | ![HILTI 2022 · exp02 施工场地多层](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| ![HILTI 2022 · exp03 施工场地楼梯](trajectories/hilti_2022_exp03_construction_stairs.png) | ![HILTI 2022 · exp04 施工场地上层](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| ![HILTI 2022 · exp05 施工场地上层 2](trajectories/hilti_2022_exp05_construction_upper_level_2.png) | ![HILTI 2022 · exp06 施工场地上层 3](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| ![HILTI 2022 · exp07 长走廊](trajectories/hilti_2022_exp07_long_corridor.png) | ![HILTI 2022 · exp09 穹顶](trajectories/hilti_2022_exp09_cupola.png) |
| ![HILTI 2022 · exp10 穹顶 2 部分覆盖](trajectories/hilti_2022_exp10_cupola_2.png) | ![HILTI 2022 · exp11 下层画廊](trajectories/hilti_2022_exp11_lower_gallery.png) |
| ![HILTI 2022 · exp14 地下室 2](trajectories/hilti_2022_exp14_basement_2.png) | ![HILTI 2022 · exp16 阁楼到上层画廊 2 部分覆盖](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |

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
