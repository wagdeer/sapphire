# 统一参数逐序列回归

基线为上一轮全量运行。`metric threshold` 仅表示变化超过 0.01 m 或上轮的 10%，是排查标记，不直接等同于算法故障；状态和覆盖变化均保留。

| Dataset | Sequence | Agent | Before | Current | Delta | Coverage | Flags |
|---|---|---|---:|---:|---:|---:|---|
| HILTI 2022 | exp01_construction_ground_level | reference | 0.014 | 0.016 | +0.003 | 100.0% -> 100.0% | — |
| HILTI 2022 | exp02_construction_multilevel | reference | 0.081 | 0.392 | +0.311 | 100.0% -> 100.0% | metric threshold |
| HILTI 2022 | exp03_construction_stairs | reference | 0.095 | 1.449 | +1.354 | 100.0% -> 100.0% | metric threshold |
| HILTI 2022 | exp04_construction_upper_level | reference | 0.031 | 0.033 | +0.002 | 100.0% -> 100.0% | — |
| HILTI 2022 | exp05_construction_upper_level_2 | reference | 0.022 | 0.012 | -0.010 | 100.0% -> 100.0% | — |
| HILTI 2022 | exp06_construction_upper_level_3 | reference | 0.033 | 0.028 | -0.004 | 100.0% -> 100.0% | — |
| HILTI 2022 | exp07_long_corridor | reference | 0.114 | 0.056 | -0.058 | 100.0% -> 100.0% | metric threshold |
| HILTI 2022 | exp09_cupola | reference | 1.107 | 0.600 | -0.507 | 100.0% -> 31.2% | status ok -> incomplete_coverage; coverage -68.8 pp; metric threshold |
| HILTI 2022 | exp10_cupola_2 | reference | 2.821 | 4.243 | +1.422 | 100.0% -> 21.4% | status ok -> incomplete_coverage; coverage -78.6 pp; metric threshold |
| HILTI 2022 | exp11_lower_gallery | reference | 0.276 | 1.455 | +1.179 | 100.0% -> 80.0% | status ok -> incomplete_coverage; coverage -20.0 pp; metric threshold |
| HILTI 2022 | exp14_basement_2 | reference | 0.051 | 0.766 | +0.715 | 100.0% -> 100.0% | metric threshold |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | reference | 0.384 | — | — | — | status ok -> invalid |
| HILTI 2023 | site1_handheld_1 | reference | 0.028 | 0.022 | -0.006 | 100.0% -> 100.0% | — |
| HILTI 2023 | site1_handheld_2 | reference | 0.062 | 0.052 | -0.010 | 100.0% -> 100.0% | — |
| HILTI 2023 | site1_handheld_3 | reference | 0.142 | 0.133 | -0.009 | 100.0% -> 100.0% | — |
| HILTI 2023 | site1_handheld_4 | reference | 0.090 | 0.059 | -0.031 | 100.0% -> 100.0% | metric threshold |
| HILTI 2023 | site1_handheld_5 | reference | 0.053 | 0.414 | +0.360 | 100.0% -> 100.0% | metric threshold |
| HILTI 2023 | site2_handheld_4 | reference | — | — | — | 100.0% -> 100.0% | — |
| HILTI 2023 | site2_handheld_5 | reference | 14.311 | 14.936 | +0.625 | 100.0% -> 100.0% | — |
| HILTI 2023 | site2_handheld_6 | reference | — | — | — | 100.0% -> 100.0% | — |
| HILTI 2023 | site2_robot_2 | reference | — | 0.069 | — | — | — |
| Newer College | 01_short_experiment | reference | 0.473 | 0.415 | -0.058 | 100.0% -> 100.0% | metric threshold |
| Newer College | 02_long_experiment | reference | 0.460 | 0.397 | -0.063 | 100.0% -> 100.0% | metric threshold |
| Newer College | 05_quad_with_dynamics | reference | 0.134 | 0.124 | -0.009 | 99.4% -> 99.4% | — |
| Newer College | 06_dynamic_spinning | reference | 0.093 | 0.096 | +0.003 | 100.0% -> 100.0% | — |
| Newer College | 07_parkland_mound | reference | 0.285 | 0.156 | -0.129 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Campus_Road_3 | alpha | 0.865 | 0.822 | -0.042 | 100.0% -> 100.0% | — |
| S3E | S3E_Campus_Road_3 | bob | 3.543 | 1.823 | -1.720 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Campus_Road_3 | carol | 26.762 | 0.919 | -25.843 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Dormitory_1 | alpha | 0.476 | 0.591 | +0.115 | 92.5% -> 91.7% | metric threshold |
| S3E | S3E_Dormitory_1 | bob | 1.329 | 1.401 | +0.072 | 97.1% -> 89.1% | coverage -8.0 pp |
| S3E | S3E_Dormitory_1 | carol | 0.833 | 0.806 | -0.027 | 76.4% -> 97.1% | — |
| S3E | S3E_Laboratory_1 | alpha | 928.391 | 928.355 | -0.036 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_1 | bob | 97.404 | 97.289 | -0.115 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_1 | carol | 158.408 | 158.375 | -0.033 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_2 | alpha | 390.085 | 389.872 | -0.213 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_2 | bob | 74.565 | 86.446 | +11.881 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Laboratory_2 | carol | 111.494 | 111.598 | +0.104 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_3 | alpha | 61.566 | 61.695 | +0.129 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_3 | bob | 16.837 | 14.130 | -2.707 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Laboratory_3 | carol | 160.810 | 138.720 | -22.090 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Laboratory_4 | alpha | 271.678 | 262.521 | -9.156 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_4 | bob | 72.265 | 71.522 | -0.743 | 100.0% -> 100.0% | — |
| S3E | S3E_Laboratory_4 | carol | 70.685 | 72.346 | +1.661 | 100.0% -> 100.0% | — |
| S3E | S3E_Library_1 | alpha | 0.970 | 0.965 | -0.005 | 92.5% -> 90.2% | coverage -2.3 pp |
| S3E | S3E_Library_1 | bob | 1.722 | 1.646 | -0.075 | 96.6% -> 97.4% | — |
| S3E | S3E_Library_1 | carol | 1.348 | 1.328 | -0.020 | 85.9% -> 93.9% | — |
| S3E | S3E_Library_2 | alpha | 0.832 | 0.925 | +0.093 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Library_2 | bob | 1.699 | 0.897 | -0.802 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Library_2 | carol | 0.893 | 0.918 | +0.025 | 100.0% -> 100.0% | — |
| S3E | S3E_Square_1 | alpha | 1.224 | 1.174 | -0.050 | 87.5% -> 84.6% | coverage -2.9 pp |
| S3E | S3E_Square_1 | bob | 0.726 | 0.784 | +0.058 | 89.9% -> 97.8% | — |
| S3E | S3E_Square_1 | carol | 1.527 | 0.596 | -0.932 | 90.8% -> 97.6% | metric threshold |
| S3E | S3E_Square_3 | alpha | 1.812 | 1.936 | +0.124 | 100.0% -> 100.0% | — |
| S3E | S3E_Square_3 | bob | 2.097 | 2.063 | -0.034 | 100.0% -> 100.0% | — |
| S3E | S3E_Square_3 | carol | 0.926 | 1.762 | +0.836 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Teaching_Building_1 | alpha | 0.071 | 0.157 | +0.086 | 88.5% -> 97.1% | metric threshold |
| S3E | S3E_Teaching_Building_1 | bob | — | 0.920 | — | 14.3% -> 95.0% | — |
| S3E | S3E_Teaching_Building_1 | carol | 0.568 | 0.572 | +0.005 | 92.6% -> 96.6% | — |
| S3E | S3E_Tunnel_1 | alpha | 92.462 | 3.781 | -88.681 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Tunnel_1 | bob | 55.729 | 42.415 | -13.315 | 100.0% -> 100.0% | metric threshold |
| S3E | S3E_Tunnel_1 | carol | 3.382 | 3.281 | -0.102 | 100.0% -> 100.0% | — |
