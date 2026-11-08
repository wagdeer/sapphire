# 基准结果

本页由 2026-11-08 全量 1.0 倍速回放生成。每条轨迹都从第一帧绘制到最后一帧；对齐参数由匹配的 GT 点估计后应用到完整轨迹。

HILTI 使用稀疏控制点、2.0 s 关联；S3E 和 Newer College 使用 0.1 s 关联。ATE 为 SE(3) Umeyama 对齐后的平移 RMSE，不做尺度修正。

## 汇总结果

| 集合 | 完整运行数 | 平均 ATE (m) | ATE 中位数 (m) |
|---|---:|---:|---:|
| HILTI 2022 完整结果 | 12 | 0.419024 | 0.087948 |
| HILTI 2023 Site 1 手持 | 5 | 0.074912 | 0.061816 |
| S3E 主表 18 条 | 18 | 2.754692 | 1.276451 |
| Newer College | 5 | 0.288886 | 0.284682 |

本轮为 2026-11-08 全量 1.0 倍速回放。HILTI 2022 12 条序列均完成完整控制点覆盖；S3E `Campus_Road_1/2` 因缺少 db3 未运行。S3E Laboratory 1–4 使用 endpoint 指标，不进入连续 ATE 汇总。

## 跨管线基线

另一次 evalio 0.6.1 评测（`evalio_20261107`）对 KISS-ICP、DLIO 和 GenZ-ICP
跑了本地所有具有内置 adapter 的记录，包括 HILTI 2022 的 `basement_2`、
`attic_to_upper_gallery_2`，以及 Newer College 2020 的全部 5 条序列。
21 个算法/序列组合中 17 个完整完成，4 个 GenZ-ICP 运行中途停止；部分轨迹和
指标都保留。HILTI 2023 与 S3E 因 evalio 0.6.1 没有内置 adapter 而未纳入。

### Newer College 2020 · 与 Sapphire 同口径

保存的 evalio 轨迹按 0.1 s 关联后，重新使用 SE(3) Umeyama 对齐、不做尺度修正，
计算平移 RMSE。这与下方 Sapphire Newer College 结果的指标定义一致，因此 5 条
序列的汇总可以直接比较。未改动的 Sapphire 对比行来自 2026-11-06 benchmark；2026-11-08 全量重跑没有重新做并排对比。

| 管线 | 版本 | 完整运行数 | 平均 ATE (m) | 中位数 ATE (m) |
|---|---|---:|---:|---:|
| Sapphire | 2026-11-06 benchmark | 5/5 | 0.263176 | 0.158747 |
| DLIO | 1.1.1 | 5/5 | 0.221192 | 0.164614 |
| KISS-ICP | 1.3.0 | 5/5 | 0.540744 | 0.583880 |
| GenZ-ICP | 0.3.2 | 1/5 | 0.133740 | 0.133740 |

GenZ-ICP 只完整完成了 `quad_with_dynamics`，所以它的单条均值不能视为 5 条序列
对比。下方仍保留 GenZ-ICP 的部分结果，但它们不会进入任何完整运行汇总。

### evalio 原始指标

下表保留 evalio 原生口径：最近时间戳关联和 first-pose alignment。它不能与
Sapphire 表格中的 SE(3) Umeyama 数值混为同一排名。

| 数据集 | 管线 | 版本 | 完整运行数 | 平均 ATE (m) | 中位数 ATE (m) |
|---|---|---|---:|---:|---:|
| HILTI 2022 | KISS-ICP | 1.3.0 | 2/2 | 10.552516 | 10.552516 |
| HILTI 2022 | DLIO | 1.1.1 | 2/2 | 16.817070 | 16.817070 |
| HILTI 2022 | GenZ-ICP | 0.3.2 | 2/2 | 10.192442 | 10.192442 |
| Newer College 2020 | KISS-ICP | 1.3.0 | 5/5 | 1.028167 | 0.803811 |
| Newer College 2020 | DLIO | 1.1.1 | 5/5 | 2.686118 | 2.831087 |
| Newer College 2020 | GenZ-ICP | 0.3.2 | 1/5 | 0.181977 | 0.181977 |

### evalio 逐序列结果

`SE(3)` 列用上文的 Sapphire 同口径重新计算保存的 Newer College 轨迹。
标为 `partial` 的数值只覆盖失败运行已保存的前缀，不能作为完整结果参与排名。
HILTI 在 evalio 中使用的真值是密集 `*_imu.txt`，因此这里不报告对应的
稀疏控制点 SE(3) 数值。

| 数据集 | 序列 | 管线 | 状态 | 位姿覆盖 | evalio ATE (m) | SE(3) ATE (m) | 轨迹图 |
|---|---|---|---|---:|---:|---:|---|
| HILTI 2022 | attic_to_upper_gallery_2 | KISS-ICP | complete | 100.0% | 12.445 | — | [plot](evalio/hilti_2022__attic_to_upper_gallery_2.png) |
| HILTI 2022 | attic_to_upper_gallery_2 | DLIO | complete | 100.0% | 17.125 | — | [plot](evalio/hilti_2022__attic_to_upper_gallery_2.png) |
| HILTI 2022 | attic_to_upper_gallery_2 | GenZ-ICP | complete | 100.0% | 20.191 | — | [plot](evalio/hilti_2022__attic_to_upper_gallery_2.png) |
| HILTI 2022 | basement_2 | KISS-ICP | complete | 100.0% | 8.660 | — | [plot](evalio/hilti_2022__basement_2.png) |
| HILTI 2022 | basement_2 | DLIO | complete | 100.0% | 16.509 | — | [plot](evalio/hilti_2022__basement_2.png) |
| HILTI 2022 | basement_2 | GenZ-ICP | complete | 100.0% | 0.194 | — | [plot](evalio/hilti_2022__basement_2.png) |
| Newer College 2020 | dynamic_spinning | KISS-ICP | complete | 100.0% | 0.804 | 0.584 | [plot](evalio/newer_college_2020__dynamic_spinning.png) |
| Newer College 2020 | dynamic_spinning | DLIO | complete | 100.0% | 0.115 | 0.095 | [plot](evalio/newer_college_2020__dynamic_spinning.png) |
| Newer College 2020 | dynamic_spinning | GenZ-ICP | fail; 557/1202 poses | 46.3% | 11.529 | 9.426 partial | [plot](evalio/newer_college_2020__dynamic_spinning.png) |
| Newer College 2020 | long_experiment | KISS-ICP | complete | 100.0% | 1.924 | 1.057 | [plot](evalio/newer_college_2020__long_experiment.png) |
| Newer College 2020 | long_experiment | DLIO | complete | 100.0% | 2.831 | 0.377 | [plot](evalio/newer_college_2020__long_experiment.png) |
| Newer College 2020 | long_experiment | GenZ-ICP | fail; 13088/26560 poses | 49.3% | 0.979 | 0.335 partial | [plot](evalio/newer_college_2020__long_experiment.png) |
| Newer College 2020 | parkland_mound | KISS-ICP | complete | 100.0% | 0.505 | 0.279 | [plot](evalio/newer_college_2020__parkland_mound.png) |
| Newer College 2020 | parkland_mound | DLIO | complete | 100.0% | 4.013 | 0.165 | [plot](evalio/newer_college_2020__parkland_mound.png) |
| Newer College 2020 | parkland_mound | GenZ-ICP | fail; 1630/5009 poses | 32.5% | 1.517 | 1.459 partial | [plot](evalio/newer_college_2020__parkland_mound.png) |
| Newer College 2020 | quad_with_dynamics | KISS-ICP | complete | 100.0% | 0.247 | 0.187 | [plot](evalio/newer_college_2020__quad_with_dynamics.png) |
| Newer College 2020 | quad_with_dynamics | DLIO | complete | 100.0% | 1.109 | 0.123 | [plot](evalio/newer_college_2020__quad_with_dynamics.png) |
| Newer College 2020 | quad_with_dynamics | GenZ-ICP | complete | 100.0% | 0.182 | 0.134 | [plot](evalio/newer_college_2020__quad_with_dynamics.png) |
| Newer College 2020 | short_experiment | KISS-ICP | complete | 100.0% | 1.661 | 0.597 | [plot](evalio/newer_college_2020__short_experiment.png) |
| Newer College 2020 | short_experiment | DLIO | complete | 100.0% | 5.363 | 0.346 | [plot](evalio/newer_college_2020__short_experiment.png) |
| Newer College 2020 | short_experiment | GenZ-ICP | fail; 6083/15302 poses | 39.8% | 1.094 | 0.299 partial | [plot](evalio/newer_college_2020__short_experiment.png) |

![evalio ATE 总览](evalio/ate_overview.png)

总览图使用与 evalio 完全相同的原生口径加入 Sapphire：对 evalio 密集 GT 做最近
时间戳关联和 first-pose alignment。Sapphire 的原始完整轨迹先转换到对应数据集的
IMU 帧再评测。该图与上方的 SE(3) 表口径不同。图中只包含完整运行；GenZ-ICP
失败前保留的部分轨迹仍在逐序列表里。Sapphire 数值见
[sapphire_evalio_native_ate.csv](evalio/sapphire_evalio_native_ate.csv)。

完整的 evalio 结果表见 [ate_summary.csv](evalio/ate_summary.csv)；
重新计算的 Newer College SE(3) 数值见
[newer_college_se3_ate.csv](evalio/newer_college_se3_ate.csv)。运行来源记录在
[run_manifest.json](evalio/run_manifest.json)，对应配置为
[evalio_baseline.yaml](evalio/evalio_baseline.yaml)。

## 逐序列结果

| 数据 | 序列 | Agent | 指标 | 结果 (m) | GT 覆盖 | 估计长度 (m) | GT 长度 (m) | 状态 | 计入均值 | 轨迹 |
|---|---|---|---|---:|---:|---:|---:|---|---|---|
| HILTI 2022 | exp01_construction_ground_level | Reference | 稀疏 ATE | 0.014 | 13/13 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp01_construction_ground_level.png) |
| HILTI 2022 | exp02_construction_multilevel | Reference | 稀疏 ATE | 0.081 | 22/22 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| HILTI 2022 | exp03_construction_stairs | Reference | 稀疏 ATE | 0.095 | 17/17 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp03_construction_stairs.png) |
| HILTI 2022 | exp04_construction_upper_level | Reference | 稀疏 ATE | 0.031 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| HILTI 2022 | exp05_construction_upper_level_2 | Reference | 稀疏 ATE | 0.022 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp05_construction_upper_level_2.png) |
| HILTI 2022 | exp06_construction_upper_level_3 | Reference | 稀疏 ATE | 0.033 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| HILTI 2022 | exp07_long_corridor | Reference | 稀疏 ATE | 0.114 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp07_long_corridor.png) |
| HILTI 2022 | exp09_cupola | Reference | 稀疏 ATE | 1.107 | 16/16 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp09_cupola.png) |
| HILTI 2022 | exp10_cupola_2 | Reference | 稀疏 ATE | 2.821 | 14/14 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp10_cupola_2.png) |
| HILTI 2022 | exp11_lower_gallery | Reference | 稀疏 ATE | 0.276 | 5/5 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp11_lower_gallery.png) |
| HILTI 2022 | exp14_basement_2 | Reference | 稀疏 ATE | 0.051 | 4/4 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp14_basement_2.png) |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | Reference | 稀疏 ATE | 0.384 | 9/9 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| Site 1 · Handheld 1 | site1_handheld_1 | Reference | 稀疏 ATE | 0.028 | 4/4 | 205.2 | 81.32 | ok | yes | [plot](trajectories/hilti_site1_handheld_1.png) |
| Site 1 · Handheld 2 | site1_handheld_2 | Reference | 稀疏 ATE | 0.062 | 4/4 | 163.9 | 84.39 | ok | yes | [plot](trajectories/hilti_site1_handheld_2.png) |
| Site 1 · Handheld 3 | site1_handheld_3 | Reference | 稀疏 ATE | 0.142 | 4/4 | 164.6 | 81.89 | ok | yes | [plot](trajectories/hilti_site1_handheld_3.png) |
| Site 1 · Handheld 4 | site1_handheld_4 | Reference | 稀疏 ATE | 0.090 | 3/3 | 299.6 | 52.88 | warning; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across | yes | [plot](trajectories/hilti_site1_handheld_4.png) |
| Site 1 · Handheld 5 | site1_handheld_5 | Reference | 稀疏 ATE | 0.053 | 3/3 | 149.3 | 45.94 | ok | yes | [plot](trajectories/hilti_site1_handheld_5.png) |
| Site 2 · Handheld 4 | site2_handheld_4 | Reference | 稀疏 ATE | — | 2/2 | 65.5 | 6.68 | invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible | no | [plot](trajectories/hilti_site2_handheld_4.png) |
| Site 2 · Handheld 5 | site2_handheld_5 | Reference | 稀疏 ATE | 14.311 | 4/4 | 126.2 | 53.06 | ok | no | [plot](trajectories/hilti_site2_handheld_5.png) |
| Site 2 · Handheld 6 | site2_handheld_6 | Reference | 稀疏 ATE | — | 2/2 | 110.9 | 3.38 | invalid; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible | no | [plot](trajectories/hilti_site2_handheld_6.png) |
| Site 2 · Robot 2 | site2_robot_2 | Reference | 稀疏 ATE | — | — | — | — | failed; ValueError: hilti_2023/site2_robot_2/trajectory.tum: only 0 robot control point(s) matched | no | [plot](trajectories/hilti_site2_robot_2.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Alpha | ATE | 0.865 | 6310/61352 | 939.9 | 983.32 | ok | yes | [plot](trajectories/s3e_campus_road_3_alpha.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Bob | ATE | 3.543 | 5702/54175 | 936.6 | 967.57 | ok | yes | [plot](trajectories/s3e_campus_road_3_bob.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Carol | ATE | 26.762 | 5114/63771 | 1310.5 | 986.94 | ok | yes | [plot](trajectories/s3e_campus_road_3_carol.png) |
| Dormitory 1 | S3E_Dormitory_1 | Alpha | ATE | 0.476 | 470/508 | 724.0 | 727.02 | ok | yes | [plot](trajectories/s3e_dormitory_1_alpha.png) |
| Dormitory 1 | S3E_Dormitory_1 | Bob | ATE | 1.329 | 608/626 | 723.4 | 719.33 | ok | yes | [plot](trajectories/s3e_dormitory_1_bob.png) |
| Dormitory 1 | S3E_Dormitory_1 | Carol | ATE | 0.833 | 366/479 | 737.3 | 721.92 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_dormitory_1_carol.png) |
| Laboratory 1 | S3E_Laboratory_1 | Alpha | 端点 | 928.391 | 2/2 | 1.0 | 929.07 | warning; GT repair applied to 1 malformed line(s); normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_alpha.png) |
| Laboratory 1 | S3E_Laboratory_1 | Bob | 端点 | 97.404 | 2/2 | 0.2 | 97.26 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_bob.png) |
| Laboratory 1 | S3E_Laboratory_1 | Carol | 端点 | 158.408 | 2/2 | 0.2 | 158.45 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_carol.png) |
| Laboratory 2 | S3E_Laboratory_2 | Alpha | 端点 | 390.085 | 2/2 | 0.5 | 390.08 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_alpha.png) |
| Laboratory 2 | S3E_Laboratory_2 | Bob | 端点 | 74.565 | 2/2 | 21.0 | 86.43 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_bob.png) |
| Laboratory 2 | S3E_Laboratory_2 | Carol | 端点 | 111.494 | 2/2 | 3.0 | 111.27 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_carol.png) |
| Laboratory 3 | S3E_Laboratory_3 | Alpha | 端点 | 61.566 | 2/2 | 0.1 | 61.55 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_alpha.png) |
| Laboratory 3 | S3E_Laboratory_3 | Bob | 端点 | 16.837 | 2/2 | 0.2 | 16.97 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_bob.png) |
| Laboratory 3 | S3E_Laboratory_3 | Carol | 端点 | 160.810 | 2/2 | 38.1 | 138.38 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_carol.png) |
| Laboratory 4 | S3E_Laboratory_4 | Alpha | 端点 | 271.678 | 2/2 | 3.6 | 271.85 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_alpha.png) |
| Laboratory 4 | S3E_Laboratory_4 | Bob | 端点 | 72.265 | 2/2 | 3.7 | 71.33 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_bob.png) |
| Laboratory 4 | S3E_Laboratory_4 | Carol | 端点 | 70.685 | 2/2 | 5.4 | 72.30 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_carol.png) |
| Library 1 | S3E_Library_1 | Alpha | ATE | 0.970 | 370/400 | 520.4 | 507.57 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_alpha.png) |
| Library 1 | S3E_Library_1 | Bob | ATE | 1.722 | 366/379 | 530.8 | 517.25 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_bob.png) |
| Library 1 | S3E_Library_1 | Carol | ATE | 1.348 | 324/377 | 508.1 | 498.95 | warning; 4 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_carol.png) |
| Library 2 | S3E_Library_2 | Alpha | ATE | 0.832 | 3434/34555 | 504.2 | 521.64 | ok | yes | [plot](trajectories/s3e_library_2_alpha.png) |
| Library 2 | S3E_Library_2 | Bob | ATE | 1.699 | 3524/30843 | 505.8 | 523.93 | ok | yes | [plot](trajectories/s3e_library_2_bob.png) |
| Library 2 | S3E_Library_2 | Carol | ATE | 0.893 | 3265/30840 | 504.8 | 520.25 | ok | yes | [plot](trajectories/s3e_library_2_carol.png) |
| Square 1 | S3E_Square_1 | Alpha | ATE | 1.224 | 336/384 | 542.1 | 546.05 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_square_1_alpha.png) |
| Square 1 | S3E_Square_1 | Bob | ATE | 0.726 | 410/456 | 495.1 | 496.53 | ok | yes | [plot](trajectories/s3e_square_1_bob.png) |
| Square 1 | S3E_Square_1 | Carol | ATE | 1.527 | 307/338 | 529.7 | 529.18 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_square_1_carol.png) |
| Square 3 | S3E_Square_3 | Alpha | ATE | 1.812 | 2769/40657 | 464.5 | 487.36 | ok | yes | [plot](trajectories/s3e_square_3_alpha.png) |
| Square 3 | S3E_Square_3 | Bob | ATE | 2.097 | 2587/26149 | 554.7 | 569.63 | ok | yes | [plot](trajectories/s3e_square_3_bob.png) |
| Square 3 | S3E_Square_3 | Carol | ATE | 0.926 | 2509/32969 | 540.9 | 563.78 | ok | yes | [plot](trajectories/s3e_square_3_carol.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Alpha | ATE | 0.071 | 123/139 | 625.3 | 36.10 | warning; 1 GT gap(s) exceed 5.000 s; gaps were not interpolated across | no | [plot](trajectories/s3e_teaching_building_1_alpha.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Bob | ATE | — | 17/119 | 769.4 | 43.36 | invalid; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across; TrajectoryError: only 17 poses matched; minimum is 20 | no | [plot](trajectories/s3e_teaching_building_1_bob.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Carol | ATE | 0.568 | 137/148 | 655.7 | 68.08 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | no | [plot](trajectories/s3e_teaching_building_1_carol.png) |
| Tunnel 1 | S3E_Tunnel_1 | Alpha | ATE | 92.462 | 2410/29868 | 545.6 | 521.96 | ok | no | [plot](trajectories/s3e_tunnel_1_alpha.png) |
| Tunnel 1 | S3E_Tunnel_1 | Bob | ATE | 55.729 | 2587/26337 | 402.2 | 502.39 | ok | no | [plot](trajectories/s3e_tunnel_1_bob.png) |
| Tunnel 1 | S3E_Tunnel_1 | Carol | ATE | 3.382 | 2695/28955 | 474.6 | 501.10 | ok | no | [plot](trajectories/s3e_tunnel_1_carol.png) |
| Short Experiment | 01_short_experiment | Reference | ATE | 0.473 | 14466/15301 | 1421.7 | 1609.40 | ok | yes | [plot](trajectories/newer_college_01_short_experiment.png) |
| Long Experiment | 02_long_experiment | Reference | ATE | 0.460 | 23374/26559 | 2598.3 | 3063.42 | ok | yes | [plot](trajectories/newer_college_02_long_experiment.png) |
| Quad with Dynamics | 05_quad_with_dynamics | Reference | ATE | 0.134 | 3491/3954 | 474.8 | 479.04 | ok | yes | [plot](trajectories/newer_college_05_quad_with_dynamics.png) |
| Dynamic Spinning | 06_dynamic_spinning | Reference | ATE | 0.093 | 955/1197 | 77.1 | 97.20 | ok | yes | [plot](trajectories/newer_college_06_dynamic_spinning.png) |
| Parkland Mound | 07_parkland_mound | Reference | ATE | 0.285 | 4465/5006 | 654.6 | 695.66 | ok | yes | [plot](trajectories/newer_college_07_parkland_mound.png) |


## 评测异常

- **Site 2 · Handheld 4 / site2_handheld_4 / Reference**: invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Handheld 6 / site2_handheld_6 / Reference**: invalid; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Robot 2 / site2_robot_2 / Reference**: failed; ValueError: hilti_2023/site2_robot_2/trajectory.tum: only 0 robot control point(s) matched
- **Teaching Building 1 / S3E_Teaching_Building_1 / Bob**: invalid; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across; TrajectoryError: only 17 poses matched; minimum is 20

S3E Laboratory 1–4 只有起终点真值，因此使用 endpoint error，不与连续轨迹 ATE 混合。

Tunnel 1 Bob 为保留可追溯性继续记录 `55.729 m`，但维护者复查发现该段真值的
路径和时间质量不可靠，而估计轨迹与点云目视一致。因此该行只作为 GT 诊断，
不能视为算法失败，也不参与跨数据集排名。

## 轨迹对比

轨迹图按数据集分组；列表也包含未进入汇总均值的运行。

| 轨迹结果 | 轨迹结果 |
|---|---|
| ![HILTI 2022 · exp01_construction_ground_level · Reference](trajectories/hilti_2022_exp01_construction_ground_level.png) | ![HILTI 2022 · exp02_construction_multilevel · Reference](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| ![HILTI 2022 · exp03_construction_stairs · Reference](trajectories/hilti_2022_exp03_construction_stairs.png) | ![HILTI 2022 · exp04_construction_upper_level · Reference](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| ![HILTI 2022 · exp05_construction_upper_level_2 · Reference](trajectories/hilti_2022_exp05_construction_upper_level_2.png) | ![HILTI 2022 · exp06_construction_upper_level_3 · Reference](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| ![HILTI 2022 · exp07_long_corridor · Reference](trajectories/hilti_2022_exp07_long_corridor.png) | ![HILTI 2022 · exp09_cupola · Reference](trajectories/hilti_2022_exp09_cupola.png) |
| ![HILTI 2022 · exp10_cupola_2 · Reference](trajectories/hilti_2022_exp10_cupola_2.png) | ![HILTI 2022 · exp11_lower_gallery · Reference](trajectories/hilti_2022_exp11_lower_gallery.png) |
| ![HILTI 2022 · exp14_basement_2 · Reference](trajectories/hilti_2022_exp14_basement_2.png) | ![HILTI 2022 · exp16_attic_to_upper_gallery_2 · Reference](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| ![Site 1 · Handheld 1 · site1_handheld_1 · Reference](trajectories/hilti_site1_handheld_1.png) | ![Site 1 · Handheld 2 · site1_handheld_2 · Reference](trajectories/hilti_site1_handheld_2.png) |
| ![Site 1 · Handheld 3 · site1_handheld_3 · Reference](trajectories/hilti_site1_handheld_3.png) | ![Site 1 · Handheld 4 · site1_handheld_4 · Reference](trajectories/hilti_site1_handheld_4.png) |
| ![Site 1 · Handheld 5 · site1_handheld_5 · Reference](trajectories/hilti_site1_handheld_5.png) | ![Site 2 · Handheld 4 · site2_handheld_4 · Reference](trajectories/hilti_site2_handheld_4.png) |
| ![Site 2 · Handheld 5 · site2_handheld_5 · Reference](trajectories/hilti_site2_handheld_5.png) | ![Campus Road 3 · S3E_Campus_Road_3 · Alpha](trajectories/s3e_campus_road_3_alpha.png) |
| ![Campus Road 3 · S3E_Campus_Road_3 · Bob](trajectories/s3e_campus_road_3_bob.png) | ![Campus Road 3 · S3E_Campus_Road_3 · Carol](trajectories/s3e_campus_road_3_carol.png) |
| ![Dormitory 1 · S3E_Dormitory_1 · Alpha](trajectories/s3e_dormitory_1_alpha.png) | ![Dormitory 1 · S3E_Dormitory_1 · Bob](trajectories/s3e_dormitory_1_bob.png) |
| ![Dormitory 1 · S3E_Dormitory_1 · Carol](trajectories/s3e_dormitory_1_carol.png) | ![Laboratory 1 · S3E_Laboratory_1 · Alpha](trajectories/s3e_laboratory_1_alpha.png) |
| ![Laboratory 1 · S3E_Laboratory_1 · Bob](trajectories/s3e_laboratory_1_bob.png) | ![Laboratory 1 · S3E_Laboratory_1 · Carol](trajectories/s3e_laboratory_1_carol.png) |
| ![Laboratory 2 · S3E_Laboratory_2 · Alpha](trajectories/s3e_laboratory_2_alpha.png) | ![Laboratory 2 · S3E_Laboratory_2 · Bob](trajectories/s3e_laboratory_2_bob.png) |
| ![Laboratory 2 · S3E_Laboratory_2 · Carol](trajectories/s3e_laboratory_2_carol.png) | ![Laboratory 3 · S3E_Laboratory_3 · Alpha](trajectories/s3e_laboratory_3_alpha.png) |
| ![Laboratory 3 · S3E_Laboratory_3 · Bob](trajectories/s3e_laboratory_3_bob.png) | ![Laboratory 3 · S3E_Laboratory_3 · Carol](trajectories/s3e_laboratory_3_carol.png) |
| ![Laboratory 4 · S3E_Laboratory_4 · Alpha](trajectories/s3e_laboratory_4_alpha.png) | ![Laboratory 4 · S3E_Laboratory_4 · Bob](trajectories/s3e_laboratory_4_bob.png) |
| ![Laboratory 4 · S3E_Laboratory_4 · Carol](trajectories/s3e_laboratory_4_carol.png) | ![Library 1 · S3E_Library_1 · Alpha](trajectories/s3e_library_1_alpha.png) |
| ![Library 1 · S3E_Library_1 · Bob](trajectories/s3e_library_1_bob.png) | ![Library 1 · S3E_Library_1 · Carol](trajectories/s3e_library_1_carol.png) |
| ![Library 2 · S3E_Library_2 · Alpha](trajectories/s3e_library_2_alpha.png) | ![Library 2 · S3E_Library_2 · Bob](trajectories/s3e_library_2_bob.png) |
| ![Library 2 · S3E_Library_2 · Carol](trajectories/s3e_library_2_carol.png) | ![Square 1 · S3E_Square_1 · Alpha](trajectories/s3e_square_1_alpha.png) |
| ![Square 1 · S3E_Square_1 · Bob](trajectories/s3e_square_1_bob.png) | ![Square 1 · S3E_Square_1 · Carol](trajectories/s3e_square_1_carol.png) |
| ![Square 3 · S3E_Square_3 · Alpha](trajectories/s3e_square_3_alpha.png) | ![Square 3 · S3E_Square_3 · Bob](trajectories/s3e_square_3_bob.png) |
| ![Square 3 · S3E_Square_3 · Carol](trajectories/s3e_square_3_carol.png) | ![Teaching Building 1 · S3E_Teaching_Building_1 · Alpha](trajectories/s3e_teaching_building_1_alpha.png) |
| ![Teaching Building 1 · S3E_Teaching_Building_1 · Bob](trajectories/s3e_teaching_building_1_bob.png) | ![Teaching Building 1 · S3E_Teaching_Building_1 · Carol](trajectories/s3e_teaching_building_1_carol.png) |
| ![Tunnel 1 · S3E_Tunnel_1 · Alpha](trajectories/s3e_tunnel_1_alpha.png) | ![Tunnel 1 · S3E_Tunnel_1 · Bob](trajectories/s3e_tunnel_1_bob.png) |
| ![Tunnel 1 · S3E_Tunnel_1 · Carol](trajectories/s3e_tunnel_1_carol.png) | ![Short Experiment · 01_short_experiment · Reference](trajectories/newer_college_01_short_experiment.png) |
| ![Long Experiment · 02_long_experiment · Reference](trajectories/newer_college_02_long_experiment.png) | ![Quad with Dynamics · 05_quad_with_dynamics · Reference](trajectories/newer_college_05_quad_with_dynamics.png) |
| ![Dynamic Spinning · 06_dynamic_spinning · Reference](trajectories/newer_college_06_dynamic_spinning.png) | ![Parkland Mound · 07_parkland_mound · Reference](trajectories/newer_college_07_parkland_mound.png) |

独立 HTML 版本：[ate_report.html](ate_report.html)。
