# 基准结果

本页来自规范运行 `20261108T153222Z`，全部以 1.0 倍速回放且不启动 RViz。轨迹图从原始完整估计轨迹绘制；对齐由匹配的 GT 样本估计后应用到全部估计位姿。

HILTI 使用稀疏控制点和 2.0 s 关联窗口，S3E 与 Newer College 使用 0.1 s。连续 ATE、稀疏 ATE 和 endpoint 误差不混合汇总。HILTI 2022 汇总只包含完整覆盖的结果，未完整覆盖和无效结果仍保留在逐序列表中。

## 汇总结果

| 汇总 | 运行数 | 本轮均值 | 本轮中位数 | 上轮运行数 | 上轮均值 | 上轮中位数 |
|---|---:|---:|---:|---:|---:|---:|
| HILTI 2022 完整覆盖稀疏 ATE | 8 | 0.3442 | 0.0442 | 8 | 0.0551 | 0.0420 |
| HILTI 2023 Site 1 手持 | 5 | 0.1359 | 0.0588 | 5 | 0.0749 | 0.0618 |
| HILTI 2023 全部有效结果 | 7 | 2.2406 | 0.0689 | 6 | 2.4476 | 0.0758 |
| S3E 主表连续 ATE | 18 | 1.1865 | 0.9452 | 18 | 2.7547 | 1.2765 |
| S3E 全部连续 ATE | 24 | 3.0201 | 0.9452 | 23 | 8.7738 | 1.3289 |
| S3E Laboratory 端点误差 | 12 | 199.4056 | 104.4437 | 12 | 201.1823 | 104.4493 |
| Newer College 连续 ATE | 5 | 0.2377 | 0.1562 | 5 | 0.2889 | 0.2847 |

### 统一参数鲁棒性结论

- HILTI 2022 的 8 条完整覆盖运行，平均误差由同集上轮的 `0.0551 m` 变为 `0.3442 m`；全量回放还留下 3 条覆盖不完整和 1 条无法对齐的轨迹，说明该组参数在 HILTI 上不够稳健。
- HILTI 2023 Site 1 手持的平均误差由 `0.0749 m` 变为 `0.1359 m`，中位数由 `0.0618 m` 变为 `0.0588 m`。均值退化、中位数略好，结果呈明显长尾。
- S3E 主表 18 条的平均误差由 `2.7547 m` 降至 `1.1865 m`，中位数由 `1.2765 m` 降至 `0.9452 m`，是本轮最明确的改善。
- Newer College 5 条的平均误差由 `0.2889 m` 降至 `0.2377 m`，中位数由 `0.2847 m` 降至 `0.1562 m`。
- HILTI 2023 Robot 2 使用独立传感器配置，得到 `0.0689 m` 的有效结果。

逐序列变化明细见[统一参数回归](unified_parameter_regression.zh.md)。

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

| 数据集 | 序列 | Agent | 指标 | 结果 (m) | 覆盖 | 状态 | 主汇总 | 图 |
|---|---|---|---|---:|---:|---|---|---|
| HILTI 2022 | exp01_construction_ground_level | reference | 稀疏 ATE | 0.016 | 13/13 | ok | 是 | [plot](trajectories/hilti_2022_exp01_construction_ground_level.png) |
| HILTI 2022 | exp02_construction_multilevel | reference | 稀疏 ATE | 0.392 | 22/22 | ok | 是 | [plot](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| HILTI 2022 | exp03_construction_stairs | reference | 稀疏 ATE | 1.449 | 17/17 | ok | 是 | [plot](trajectories/hilti_2022_exp03_construction_stairs.png) |
| HILTI 2022 | exp04_construction_upper_level | reference | 稀疏 ATE | 0.033 | 7/7 | ok | 是 | [plot](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| HILTI 2022 | exp05_construction_upper_level_2 | reference | 稀疏 ATE | 0.012 | 6/6 | ok | 是 | [plot](trajectories/hilti_2022_exp05_construction_upper_level_2.png) |
| HILTI 2022 | exp06_construction_upper_level_3 | reference | 稀疏 ATE | 0.028 | 7/7 | ok | 是 | [plot](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| HILTI 2022 | exp07_long_corridor | reference | 稀疏 ATE | 0.056 | 6/6 | ok | 是 | [plot](trajectories/hilti_2022_exp07_long_corridor.png) |
| HILTI 2022 | exp09_cupola | reference | 稀疏 ATE | 0.600 | 5/16 | incomplete_coverage | 否 | [plot](trajectories/hilti_2022_exp09_cupola.png) |
| HILTI 2022 | exp10_cupola_2 | reference | 稀疏 ATE | 4.243 | 3/14 | incomplete_coverage | 否 | [plot](trajectories/hilti_2022_exp10_cupola_2.png) |
| HILTI 2022 | exp11_lower_gallery | reference | 稀疏 ATE | 1.455 | 4/5 | incomplete_coverage | 否 | [plot](trajectories/hilti_2022_exp11_lower_gallery.png) |
| HILTI 2022 | exp14_basement_2 | reference | 稀疏 ATE | 0.766 | 4/4 | ok | 是 | [plot](trajectories/hilti_2022_exp14_basement_2.png) |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | reference | 稀疏 ATE | — | — | invalid | 否 | [plot](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| HILTI 2023 | Site 1 · Handheld 1 | reference | 稀疏 ATE | 0.022 | 4/4 | ok | 是 | [plot](trajectories/hilti_site1_handheld_1.png) |
| HILTI 2023 | Site 1 · Handheld 2 | reference | 稀疏 ATE | 0.052 | 4/4 | ok | 是 | [plot](trajectories/hilti_site1_handheld_2.png) |
| HILTI 2023 | Site 1 · Handheld 3 | reference | 稀疏 ATE | 0.133 | 4/4 | ok | 是 | [plot](trajectories/hilti_site1_handheld_3.png) |
| HILTI 2023 | Site 1 · Handheld 4 | reference | 稀疏 ATE | 0.059 | 3/3 | warning | 是 | [plot](trajectories/hilti_site1_handheld_4.png) |
| HILTI 2023 | Site 1 · Handheld 5 | reference | 稀疏 ATE | 0.414 | 3/3 | ok | 是 | [plot](trajectories/hilti_site1_handheld_5.png) |
| HILTI 2023 | Site 2 · Handheld 4 | reference | 稀疏 ATE | — | 2/2 | invalid | 否 | [plot](trajectories/hilti_site2_handheld_4.png) |
| HILTI 2023 | Site 2 · Handheld 5 | reference | 稀疏 ATE | 14.936 | 4/4 | ok | 否 | [plot](trajectories/hilti_site2_handheld_5.png) |
| HILTI 2023 | Site 2 · Handheld 6 | reference | 稀疏 ATE | — | 2/2 | invalid | 否 | [plot](trajectories/hilti_site2_handheld_6.png) |
| HILTI 2023 | Site 2 · Robot 2 | reference | 稀疏 ATE | 0.069 | 3/3 | ok | 否 | [plot](trajectories/hilti_site2_robot_2.png) |
| S3E | Campus Road 3 | alpha | ATE | 0.822 | 7017/61352 | ok | 是 | [plot](trajectories/s3e_campus_road_3_alpha.png) |
| S3E | Campus Road 3 | bob | ATE | 1.823 | 7180/54175 | ok | 是 | [plot](trajectories/s3e_campus_road_3_bob.png) |
| S3E | Campus Road 3 | carol | ATE | 0.919 | 7504/63771 | ok | 是 | [plot](trajectories/s3e_campus_road_3_carol.png) |
| S3E | Dormitory 1 | alpha | ATE | 0.591 | 466/508 | ok | 是 | [plot](trajectories/s3e_dormitory_1_alpha.png) |
| S3E | Dormitory 1 | bob | ATE | 1.401 | 558/626 | ok | 是 | [plot](trajectories/s3e_dormitory_1_bob.png) |
| S3E | Dormitory 1 | carol | ATE | 0.806 | 465/479 | warning | 是 | [plot](trajectories/s3e_dormitory_1_carol.png) |
| S3E | Laboratory 1 | alpha | 端点误差 | 928.355 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_1_alpha.png) |
| S3E | Laboratory 1 | bob | 端点误差 | 97.289 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_1_bob.png) |
| S3E | Laboratory 1 | carol | 端点误差 | 158.375 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_1_carol.png) |
| S3E | Laboratory 2 | alpha | 端点误差 | 389.872 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_2_alpha.png) |
| S3E | Laboratory 2 | bob | 端点误差 | 86.446 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_2_bob.png) |
| S3E | Laboratory 2 | carol | 端点误差 | 111.598 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_2_carol.png) |
| S3E | Laboratory 3 | alpha | 端点误差 | 61.695 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_3_alpha.png) |
| S3E | Laboratory 3 | bob | 端点误差 | 14.130 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_3_bob.png) |
| S3E | Laboratory 3 | carol | 端点误差 | 138.720 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_3_carol.png) |
| S3E | Laboratory 4 | alpha | 端点误差 | 262.521 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_4_alpha.png) |
| S3E | Laboratory 4 | bob | 端点误差 | 71.522 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_4_bob.png) |
| S3E | Laboratory 4 | carol | 端点误差 | 72.346 | 2/2 | warning | 否 | [plot](trajectories/s3e_laboratory_4_carol.png) |
| S3E | Library 1 | alpha | ATE | 0.965 | 361/400 | warning | 是 | [plot](trajectories/s3e_library_1_alpha.png) |
| S3E | Library 1 | bob | ATE | 1.646 | 369/379 | warning | 是 | [plot](trajectories/s3e_library_1_bob.png) |
| S3E | Library 1 | carol | ATE | 1.328 | 354/377 | warning | 是 | [plot](trajectories/s3e_library_1_carol.png) |
| S3E | Library 2 | alpha | ATE | 0.925 | 3471/34555 | ok | 是 | [plot](trajectories/s3e_library_2_alpha.png) |
| S3E | Library 2 | bob | ATE | 0.897 | 4180/30843 | ok | 是 | [plot](trajectories/s3e_library_2_bob.png) |
| S3E | Library 2 | carol | ATE | 0.918 | 3665/30840 | ok | 是 | [plot](trajectories/s3e_library_2_carol.png) |
| S3E | Square 1 | alpha | ATE | 1.174 | 325/384 | warning | 是 | [plot](trajectories/s3e_square_1_alpha.png) |
| S3E | Square 1 | bob | ATE | 0.784 | 446/456 | ok | 是 | [plot](trajectories/s3e_square_1_bob.png) |
| S3E | Square 1 | carol | ATE | 0.596 | 330/338 | warning | 是 | [plot](trajectories/s3e_square_1_carol.png) |
| S3E | Square 3 | alpha | ATE | 1.936 | 2811/40657 | ok | 是 | [plot](trajectories/s3e_square_3_alpha.png) |
| S3E | Square 3 | bob | ATE | 2.063 | 3440/26149 | ok | 是 | [plot](trajectories/s3e_square_3_bob.png) |
| S3E | Square 3 | carol | ATE | 1.762 | 3010/32969 | ok | 是 | [plot](trajectories/s3e_square_3_carol.png) |
| S3E | Teaching Building 1 | alpha | ATE | 0.157 | 135/139 | warning | 否 | [plot](trajectories/s3e_teaching_building_1_alpha.png) |
| S3E | Teaching Building 1 | bob | ATE | 0.920 | 113/119 | warning | 否 | [plot](trajectories/s3e_teaching_building_1_bob.png) |
| S3E | Teaching Building 1 | carol | ATE | 0.572 | 143/148 | warning | 否 | [plot](trajectories/s3e_teaching_building_1_carol.png) |
| S3E | Tunnel 1 | alpha | ATE | 3.781 | 3140/29868 | ok | 否 | [plot](trajectories/s3e_tunnel_1_alpha.png) |
| S3E | Tunnel 1 | bob | ATE | 42.415 | 2777/26337 | ok | 否 | [plot](trajectories/s3e_tunnel_1_bob.png) |
| S3E | Tunnel 1 | carol | ATE | 3.281 | 3595/28955 | ok | 否 | [plot](trajectories/s3e_tunnel_1_carol.png) |
| Newer College 2020 | Short Experiment | reference | ATE | 0.415 | 11427/15301 | ok | 是 | [plot](trajectories/newer_college_01_short_experiment.png) |
| Newer College 2020 | Long Experiment | reference | ATE | 0.397 | 19688/26559 | ok | 是 | [plot](trajectories/newer_college_02_long_experiment.png) |
| Newer College 2020 | Quad with Dynamics | reference | ATE | 0.124 | 2891/3954 | ok | 是 | [plot](trajectories/newer_college_05_quad_with_dynamics.png) |
| Newer College 2020 | Dynamic Spinning | reference | ATE | 0.096 | 1149/1197 | ok | 是 | [plot](trajectories/newer_college_06_dynamic_spinning.png) |
| Newer College 2020 | Parkland Mound | reference | ATE | 0.156 | 3691/5006 | ok | 是 | [plot](trajectories/newer_college_07_parkland_mound.png) |

## 评测异常

- **exp09_cupola / exp09_cupola / reference**: incomplete_coverage
- **exp10_cupola_2 / exp10_cupola_2 / reference**: incomplete_coverage
- **exp11_lower_gallery / exp11_lower_gallery / reference**: incomplete_coverage
- **exp16_attic_to_upper_gallery_2 / exp16_attic_to_upper_gallery_2 / reference**: invalid; GeometryException: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Handheld 4 / site2_handheld_4 / reference**: invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Handheld 6 / site2_handheld_6 / reference**: invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible

S3E Laboratory 1–4 只有 endpoint 真值，因此不进入连续 ATE 汇总。`Campus_Road_1/2` 的 db3 与 GT 缺失，未运行。

## 轨迹对比

| Trajectory | Trajectory |
|---|---|
| ![exp01_construction_ground_level · reference](trajectories/hilti_2022_exp01_construction_ground_level.png) | ![exp02_construction_multilevel · reference](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| ![exp03_construction_stairs · reference](trajectories/hilti_2022_exp03_construction_stairs.png) | ![exp04_construction_upper_level · reference](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| ![exp05_construction_upper_level_2 · reference](trajectories/hilti_2022_exp05_construction_upper_level_2.png) | ![exp06_construction_upper_level_3 · reference](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| ![exp07_long_corridor · reference](trajectories/hilti_2022_exp07_long_corridor.png) | ![exp09_cupola · reference](trajectories/hilti_2022_exp09_cupola.png) |
| ![exp10_cupola_2 · reference](trajectories/hilti_2022_exp10_cupola_2.png) | ![exp11_lower_gallery · reference](trajectories/hilti_2022_exp11_lower_gallery.png) |
| ![exp14_basement_2 · reference](trajectories/hilti_2022_exp14_basement_2.png) | ![exp16_attic_to_upper_gallery_2 · reference](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| ![Site 1 · Handheld 1 · reference](trajectories/hilti_site1_handheld_1.png) | ![Site 1 · Handheld 2 · reference](trajectories/hilti_site1_handheld_2.png) |
| ![Site 1 · Handheld 3 · reference](trajectories/hilti_site1_handheld_3.png) | ![Site 1 · Handheld 4 · reference](trajectories/hilti_site1_handheld_4.png) |
| ![Site 1 · Handheld 5 · reference](trajectories/hilti_site1_handheld_5.png) | ![Site 2 · Handheld 4 · reference](trajectories/hilti_site2_handheld_4.png) |
| ![Site 2 · Handheld 5 · reference](trajectories/hilti_site2_handheld_5.png) | ![Site 2 · Handheld 6 · reference](trajectories/hilti_site2_handheld_6.png) |
| ![Site 2 · Robot 2 · reference](trajectories/hilti_site2_robot_2.png) | ![Campus Road 3 · alpha](trajectories/s3e_campus_road_3_alpha.png) |
| ![Campus Road 3 · bob](trajectories/s3e_campus_road_3_bob.png) | ![Campus Road 3 · carol](trajectories/s3e_campus_road_3_carol.png) |
| ![Dormitory 1 · alpha](trajectories/s3e_dormitory_1_alpha.png) | ![Dormitory 1 · bob](trajectories/s3e_dormitory_1_bob.png) |
| ![Dormitory 1 · carol](trajectories/s3e_dormitory_1_carol.png) | ![Laboratory 1 · alpha](trajectories/s3e_laboratory_1_alpha.png) |
| ![Laboratory 1 · bob](trajectories/s3e_laboratory_1_bob.png) | ![Laboratory 1 · carol](trajectories/s3e_laboratory_1_carol.png) |
| ![Laboratory 2 · alpha](trajectories/s3e_laboratory_2_alpha.png) | ![Laboratory 2 · bob](trajectories/s3e_laboratory_2_bob.png) |
| ![Laboratory 2 · carol](trajectories/s3e_laboratory_2_carol.png) | ![Laboratory 3 · alpha](trajectories/s3e_laboratory_3_alpha.png) |
| ![Laboratory 3 · bob](trajectories/s3e_laboratory_3_bob.png) | ![Laboratory 3 · carol](trajectories/s3e_laboratory_3_carol.png) |
| ![Laboratory 4 · alpha](trajectories/s3e_laboratory_4_alpha.png) | ![Laboratory 4 · bob](trajectories/s3e_laboratory_4_bob.png) |
| ![Laboratory 4 · carol](trajectories/s3e_laboratory_4_carol.png) | ![Library 1 · alpha](trajectories/s3e_library_1_alpha.png) |
| ![Library 1 · bob](trajectories/s3e_library_1_bob.png) | ![Library 1 · carol](trajectories/s3e_library_1_carol.png) |
| ![Library 2 · alpha](trajectories/s3e_library_2_alpha.png) | ![Library 2 · bob](trajectories/s3e_library_2_bob.png) |
| ![Library 2 · carol](trajectories/s3e_library_2_carol.png) | ![Square 1 · alpha](trajectories/s3e_square_1_alpha.png) |
| ![Square 1 · bob](trajectories/s3e_square_1_bob.png) | ![Square 1 · carol](trajectories/s3e_square_1_carol.png) |
| ![Square 3 · alpha](trajectories/s3e_square_3_alpha.png) | ![Square 3 · bob](trajectories/s3e_square_3_bob.png) |
| ![Square 3 · carol](trajectories/s3e_square_3_carol.png) | ![Teaching Building 1 · alpha](trajectories/s3e_teaching_building_1_alpha.png) |
| ![Teaching Building 1 · bob](trajectories/s3e_teaching_building_1_bob.png) | ![Teaching Building 1 · carol](trajectories/s3e_teaching_building_1_carol.png) |
| ![Tunnel 1 · alpha](trajectories/s3e_tunnel_1_alpha.png) | ![Tunnel 1 · bob](trajectories/s3e_tunnel_1_bob.png) |
| ![Tunnel 1 · carol](trajectories/s3e_tunnel_1_carol.png) | ![Short Experiment · reference](trajectories/newer_college_01_short_experiment.png) |
| ![Long Experiment · reference](trajectories/newer_college_02_long_experiment.png) | ![Quad with Dynamics · reference](trajectories/newer_college_05_quad_with_dynamics.png) |
| ![Dynamic Spinning · reference](trajectories/newer_college_06_dynamic_spinning.png) | ![Parkland Mound · reference](trajectories/newer_college_07_parkland_mound.png) |

独立 HTML 页面：[ate_report.html](ate_report.html)。
