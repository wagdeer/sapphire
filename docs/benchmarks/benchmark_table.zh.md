# 基准结果

ATE 是时间关联和 SE(3) Umeyama 对齐之后的位置误差 RMSE，不做尺度修正。
数值越低越好。汇总里的平均值和中位数按轨迹条数等权计算，不按路径长度加权，
也不是把所有位姿合在一起再算一次 RMSE。

两套关联方式不能当成同一种协议：

- HILTI 2022 和 HILTI 2023 使用稀疏测量控制点，最大间隔 2.0 s。HILTI 2023
  的参考朝向是常数，所以只评平移。Site 1 每条结果用 3 或 4 个控制点。
  这些行里的「GT 覆盖」是控制点连成的折线长度，短于控制点之间的传感器路径。
- S3E 和 Newer College 的最大间隔是 0.1 s。Dormitory 1、Library 1、Square 1
  的真值四元数也是常数。Library 2、Square 3、Campus Road 3 带朝向。

汇总纳入规则：

- HILTI 2022：控制点全部匹配上的 10 条，合计 103 个点。`exp10` 和 `exp16`
  中途丢失跟踪，表中保留为部分覆盖，不进入均值、中位数和这 103 个点。
  `exp15`、`exp18`、`exp21`、`exp23` 没有回放。`exp03` 和 `exp11` 是完整重跑；
  更早的批量轨迹 2.021 m 和 0.437 m 不是这里公布的数字。`exp09` 的 1.810 m
  把均值拉到了明显高于中位数的位置。
- HILTI 2023：只含 Site 1 手持 1–5。Site 2 也评过，列在表后，不进入 0.058 m
  这个均值。
- S3E：逐序列表中的 18 条，包含 Campus Road 3 的 Bob。Laboratory 1–4 只有
  起终点真值，不是 ATE。Teaching Building 1 不混入，因为控制路径只有几十米，
  估计轨迹约 600 m。Tunnel 1 已经跑过，但在确认估计和真值是同一条轨迹之前
  不进入均值。Campus Road 1 和 Campus Road 2 不在本表中。
- Newer College：五条都进入均值，包括真值覆盖 81.6% 的 Dynamic Spinning。
  对齐前，估计位姿从 Ouster 雷达系变到数据集 base 系，也就是左相机中心，
  该刚体变换的杆臂为 0.1012 m。汇总使用每条 RMSE 的完整精度。

## 汇总结果

| 数据集 | 纳入轨迹 | 纳入 GT 覆盖 | 平均 ATE (m) | ATE 中位数 (m) |
|---|---:|---:|---:|---:|
| HILTI 2022 | 10 | 103 个控制点 | 0.260 | 0.082 |
| HILTI 2023 Site 1 手持 | 5 | 346.4 m 控制点路径 | 0.058 | 0.052 |
| S3E | 18 | 11.39 km | 1.29 | 1.11 |
| Newer College 2020 | 5 | 5.94 km | 0.2639 | 0.1609 |

## 逐序列 ATE

HILTI 2022 结果使用数据集专用手持平台配置和稀疏控制点 GT。估计位姿在
SE(3) 对齐前从 IMU 坐标系转换到测量端点。

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
| HILTI 2023 | Site 1 · Handheld 4 | reference | 52.88 m | 300.0 | 稀疏 ATE | 0.053 | 完整 · 有一处超过 120 s 的真值间隔 |
| HILTI 2023 | Site 1 · Handheld 5 | reference | 45.94 m | 150.1 | 稀疏 ATE | 0.045 | 完整 |
| S3E | Campus Road 3 | Alpha | 983.3 m | 940.0 | ATE | 0.876 | 完整 |
| S3E | Campus Road 3 | Bob | 967.6 m | 937.4 | ATE | 3.330 | 完整 |
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
| Newer College | Short Experiment | reference | 1609.4 m · 98.1% | 1421.9 | ATE | 0.4729 | 完整 |
| Newer College | Long Experiment | reference | 3063.4 m · 97.5% | 2599.8 | ATE | 0.4597 | 完整 |
| Newer College | Quad with Dynamics | reference | 479.0 m · 98.0% | 474.9 | ATE | 0.1332 | 完整 |
| Newer College | Dynamic Spinning | reference | 97.20 m · 81.6% | 77.10 | ATE | 0.0927 | 纳入 · 覆盖真值的 81.6% |
| Newer College | Parkland Mound | reference | 695.7 m · 96.0% | 654.9 | ATE | 0.1609 | 完整 |

## 已评估，但不进入汇总

这些结果使用与上表相同的对齐设置，但不进入前面的四个均值。

| 数据集 | 序列 | 机器人 | GT 覆盖 | ATE RMSE (m) | 不进入均值的原因 |
|---|---|---|---|---:|---|
| HILTI 2023 | Site 2 · Handheld 4 | reference | 6.68 m，2 个控制点 | 0.034 | 只有两个控制点 |
| HILTI 2023 | Site 2 · Handheld 5 | reference | 53.06 m，4/4 | 14.251 | 这条记录上轨迹发散 |
| HILTI 2023 | Site 2 · Handheld 6 | reference | — | — | Umeyama 对齐失败：协方差退化 |
| HILTI 2023 | Site 2 · Robot 2 | reference | 56.62 m，3 个控制点 | 0.058 | 机器人平台，外参不同 |
| S3E | Tunnel 1 | Alpha | 522.0 m | 40.780 | 暂不发布：时间覆盖完整，但估计路径比真值短 78 m，对应关系尚未确认 |
| S3E | Tunnel 1 | Bob | 502.4 m | 47.018 | 与 Alpha 同一项待确认 |
| S3E | Tunnel 1 | Carol | 501.1 m | 3.309 | 同一项待确认；路径比真值短 25 m |

Teaching Building 1 也评过（Alpha 0.142 m，Bob 0.840 m，Carol 0.577 m）。
它的控制路径只有 36–68 m，估计轨迹约 600–740 m，并且有超过 5 s 的间隔，
所以不与上面的连续轨迹混在一起。Laboratory 1–4 只有起终点真值，不作为 ATE 公布。

Campus Road 3 的 Bob 已经计入 S3E 均值。这次运行没有生成轨迹图。

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
