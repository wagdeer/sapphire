# Benchmark results

This page is generated from the 2026-11-08 full replay at playback rate 1.0. Every trajectory plot draws the complete estimate from its first to last pose. Alignment is estimated from matched GT points and then applied to the full trajectory.

HILTI uses sparse control points and a 2.0 s association window. S3E and Newer College use 0.1 s. ATE is translation-part RMSE after SE(3) Umeyama alignment without scale correction.

## Aggregate results

| Collection | Complete runs | Mean ATE (m) | Median ATE (m) |
|---|---:|---:|---:|
| HILTI 2022 complete results | 12 | 0.419024 | 0.087948 |
| HILTI 2023 Site 1 Handheld | 5 | 0.074912 | 0.061816 |
| S3E headline 18 | 18 | 2.754692 | 1.276451 |
| Newer College | 5 | 0.288886 | 0.284682 |

This is the 2026-11-08 full replay at playback rate 1.0. All 12 HILTI 2022 sequences reached full control-point coverage. S3E `Campus_Road_1/2` are not runnable because their db3 bags are absent. S3E Laboratory 1–4 use endpoint metrics and do not enter continuous ATE aggregates.

## Cross-pipeline baselines

A separate evalio 0.6.1 run (`evalio_20261107`) compares KISS-ICP, DLIO, and
GenZ-ICP on every
local recording with a built-in adapter. It covers HILTI 2022
`basement_2` and `attic_to_upper_gallery_2`, plus all five Newer College 2020
sequences. Of 21 pipeline/sequence combinations, 17 completed and four GenZ-ICP
runs stopped part-way; their partial trajectories and metrics are retained.
HILTI 2023 and S3E were not included because evalio 0.6.1 has no adapter for
them.

### Newer College 2020 · Sapphire-compatible metric

The saved evalio trajectories were timestamp-associated at 0.1 s and
re-evaluated with translation-part RMSE after SE(3) Umeyama alignment and no
scale correction. This matches the metric used by Sapphire's Newer College
rows below, so these five-sequence results can be compared directly. The
unchanged Sapphire row is from the 2026-11-06 benchmark; no new side-by-side
run was performed for the 2026-11-08 replay.

| Pipeline | Version | Complete runs | Mean ATE (m) | Median ATE (m) |
|---|---|---:|---:|---:|
| Sapphire | 2026-11-06 benchmark | 5/5 | 0.263176 | 0.158747 |
| DLIO | 1.1.1 | 5/5 | 0.221192 | 0.164614 |
| KISS-ICP | 1.3.0 | 5/5 | 0.540744 | 0.583880 |
| GenZ-ICP | 0.3.2 | 1/5 | 0.133740 | 0.133740 |

GenZ-ICP completed only `quad_with_dynamics`; its one-run mean is not a
five-sequence comparison. The partial GenZ-ICP results are shown below but are
excluded from all complete-run aggregates.

### evalio published metric

The following values preserve evalio's native protocol: nearest-timestamp
association and first-pose alignment. They are not interchangeable with the
SE(3) Umeyama values in the Sapphire tables.

| Dataset | Pipeline | Version | Complete runs | Mean ATE (m) | Median ATE (m) |
|---|---|---|---:|---:|---:|
| HILTI 2022 | KISS-ICP | 1.3.0 | 2/2 | 10.552516 | 10.552516 |
| HILTI 2022 | DLIO | 1.1.1 | 2/2 | 16.817070 | 16.817070 |
| HILTI 2022 | GenZ-ICP | 0.3.2 | 2/2 | 10.192442 | 10.192442 |
| Newer College 2020 | KISS-ICP | 1.3.0 | 5/5 | 1.028167 | 0.803811 |
| Newer College 2020 | DLIO | 1.1.1 | 5/5 | 2.686118 | 2.831087 |
| Newer College 2020 | GenZ-ICP | 0.3.2 | 1/5 | 0.181977 | 0.181977 |

### evalio per-sequence results

`SE(3)` repeats the metric on saved Newer College trajectories with the
Sapphire-compatible alignment described above. A value marked `partial` covers
only the saved prefix of a failed run and must not be ranked as a full result.
HILTI uses dense `*_imu.txt` GT in evalio, so no corresponding sparse-control
SE(3) value is reported here.

| Dataset | Sequence | Pipeline | Status | Pose coverage | evalio ATE (m) | SE(3) ATE (m) | Plot |
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

![evalio ATE overview](evalio/ate_overview.png)

The overview now includes Sapphire under the same evalio-native protocol:
nearest-timestamp association and first-pose alignment against the evalio
dense GT. Sapphire raw full trajectories are transformed into the dataset's
IMU frame before evaluation. This figure is separate from the SE(3) table
above. Only completed runs enter the overview; failed GenZ-ICP prefixes remain
in the per-sequence table. Sapphire's values are in
[sapphire_evalio_native_ate.csv](evalio/sapphire_evalio_native_ate.csv).

The complete evalio result table is available as
[ate_summary.csv](evalio/ate_summary.csv); the recomputed Newer College
SE(3) values are in
[newer_college_se3_ate.csv](evalio/newer_college_se3_ate.csv). Run provenance
is recorded in [run_manifest.json](evalio/run_manifest.json), with the
[evalio configuration](evalio/evalio_baseline.yaml).

## Per-sequence results

| Dataset | Sequence | Agent | Metric | Result (m) | GT coverage | Est. length (m) | GT length (m) | Status | In mean | Plot |
|---|---|---|---|---:|---:|---:|---:|---|---|---|
| HILTI 2022 | exp01_construction_ground_level | Reference | sparse ATE | 0.014 | 13/13 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp01_construction_ground_level.png) |
| HILTI 2022 | exp02_construction_multilevel | Reference | sparse ATE | 0.081 | 22/22 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| HILTI 2022 | exp03_construction_stairs | Reference | sparse ATE | 0.095 | 17/17 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp03_construction_stairs.png) |
| HILTI 2022 | exp04_construction_upper_level | Reference | sparse ATE | 0.031 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| HILTI 2022 | exp05_construction_upper_level_2 | Reference | sparse ATE | 0.022 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp05_construction_upper_level_2.png) |
| HILTI 2022 | exp06_construction_upper_level_3 | Reference | sparse ATE | 0.033 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| HILTI 2022 | exp07_long_corridor | Reference | sparse ATE | 0.114 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp07_long_corridor.png) |
| HILTI 2022 | exp09_cupola | Reference | sparse ATE | 1.107 | 16/16 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp09_cupola.png) |
| HILTI 2022 | exp10_cupola_2 | Reference | sparse ATE | 2.821 | 14/14 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp10_cupola_2.png) |
| HILTI 2022 | exp11_lower_gallery | Reference | sparse ATE | 0.276 | 5/5 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp11_lower_gallery.png) |
| HILTI 2022 | exp14_basement_2 | Reference | sparse ATE | 0.051 | 4/4 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp14_basement_2.png) |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | Reference | sparse ATE | 0.384 | 9/9 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| Site 1 · Handheld 1 | site1_handheld_1 | Reference | sparse ATE | 0.028 | 4/4 | 205.2 | 81.32 | ok | yes | [plot](trajectories/hilti_site1_handheld_1.png) |
| Site 1 · Handheld 2 | site1_handheld_2 | Reference | sparse ATE | 0.062 | 4/4 | 163.9 | 84.39 | ok | yes | [plot](trajectories/hilti_site1_handheld_2.png) |
| Site 1 · Handheld 3 | site1_handheld_3 | Reference | sparse ATE | 0.142 | 4/4 | 164.6 | 81.89 | ok | yes | [plot](trajectories/hilti_site1_handheld_3.png) |
| Site 1 · Handheld 4 | site1_handheld_4 | Reference | sparse ATE | 0.090 | 3/3 | 299.6 | 52.88 | warning; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across | yes | [plot](trajectories/hilti_site1_handheld_4.png) |
| Site 1 · Handheld 5 | site1_handheld_5 | Reference | sparse ATE | 0.053 | 3/3 | 149.3 | 45.94 | ok | yes | [plot](trajectories/hilti_site1_handheld_5.png) |
| Site 2 · Handheld 4 | site2_handheld_4 | Reference | sparse ATE | — | 2/2 | 65.5 | 6.68 | invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible | no | [plot](trajectories/hilti_site2_handheld_4.png) |
| Site 2 · Handheld 5 | site2_handheld_5 | Reference | sparse ATE | 14.311 | 4/4 | 126.2 | 53.06 | ok | no | [plot](trajectories/hilti_site2_handheld_5.png) |
| Site 2 · Handheld 6 | site2_handheld_6 | Reference | sparse ATE | — | 2/2 | 110.9 | 3.38 | invalid; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible | no | [plot](trajectories/hilti_site2_handheld_6.png) |
| Site 2 · Robot 2 | site2_robot_2 | Reference | sparse ATE | — | — | — | — | failed; ValueError: hilti_2023/site2_robot_2/trajectory.tum: only 0 robot control point(s) matched | no | [plot](trajectories/hilti_site2_robot_2.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Alpha | ATE | 0.865 | 6310/61352 | 939.9 | 983.32 | ok | yes | [plot](trajectories/s3e_campus_road_3_alpha.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Bob | ATE | 3.543 | 5702/54175 | 936.6 | 967.57 | ok | yes | [plot](trajectories/s3e_campus_road_3_bob.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Carol | ATE | 26.762 | 5114/63771 | 1310.5 | 986.94 | ok | yes | [plot](trajectories/s3e_campus_road_3_carol.png) |
| Dormitory 1 | S3E_Dormitory_1 | Alpha | ATE | 0.476 | 470/508 | 724.0 | 727.02 | ok | yes | [plot](trajectories/s3e_dormitory_1_alpha.png) |
| Dormitory 1 | S3E_Dormitory_1 | Bob | ATE | 1.329 | 608/626 | 723.4 | 719.33 | ok | yes | [plot](trajectories/s3e_dormitory_1_bob.png) |
| Dormitory 1 | S3E_Dormitory_1 | Carol | ATE | 0.833 | 366/479 | 737.3 | 721.92 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_dormitory_1_carol.png) |
| Laboratory 1 | S3E_Laboratory_1 | Alpha | endpoint | 928.391 | 2/2 | 1.0 | 929.07 | warning; GT repair applied to 1 malformed line(s); normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_alpha.png) |
| Laboratory 1 | S3E_Laboratory_1 | Bob | endpoint | 97.404 | 2/2 | 0.2 | 97.26 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_bob.png) |
| Laboratory 1 | S3E_Laboratory_1 | Carol | endpoint | 158.408 | 2/2 | 0.2 | 158.45 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_1_carol.png) |
| Laboratory 2 | S3E_Laboratory_2 | Alpha | endpoint | 390.085 | 2/2 | 0.5 | 390.08 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_alpha.png) |
| Laboratory 2 | S3E_Laboratory_2 | Bob | endpoint | 74.565 | 2/2 | 21.0 | 86.43 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_bob.png) |
| Laboratory 2 | S3E_Laboratory_2 | Carol | endpoint | 111.494 | 2/2 | 3.0 | 111.27 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_2_carol.png) |
| Laboratory 3 | S3E_Laboratory_3 | Alpha | endpoint | 61.566 | 2/2 | 0.1 | 61.55 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_alpha.png) |
| Laboratory 3 | S3E_Laboratory_3 | Bob | endpoint | 16.837 | 2/2 | 0.2 | 16.97 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_bob.png) |
| Laboratory 3 | S3E_Laboratory_3 | Carol | endpoint | 160.810 | 2/2 | 38.1 | 138.38 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_3_carol.png) |
| Laboratory 4 | S3E_Laboratory_4 | Alpha | endpoint | 271.678 | 2/2 | 3.6 | 271.85 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_alpha.png) |
| Laboratory 4 | S3E_Laboratory_4 | Bob | endpoint | 72.265 | 2/2 | 3.7 | 71.33 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_bob.png) |
| Laboratory 4 | S3E_Laboratory_4 | Carol | endpoint | 70.685 | 2/2 | 5.4 | 72.30 | warning; normalized 2 GT quaternion(s); endpoint metric uses the first and last poses; GT timestamps are ignored | no | [plot](trajectories/s3e_laboratory_4_carol.png) |
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


## Evaluation failures

- **Site 2 · Handheld 4 / site2_handheld_4 / Reference**: invalid; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Handheld 6 / site2_handheld_6 / Reference**: invalid; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across; TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Robot 2 / site2_robot_2 / Reference**: failed; ValueError: hilti_2023/site2_robot_2/trajectory.tum: only 0 robot control point(s) matched
- **Teaching Building 1 / S3E_Teaching_Building_1 / Bob**: invalid; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across; TrajectoryError: only 17 poses matched; minimum is 20

S3E Laboratory 1–4 provide endpoint-only ground truth and use endpoint error;
they are not mixed with continuous-trajectory ATE.

Tunnel 1 Bob retains its `55.729 m` metric for provenance, but maintainer
review found the ground-truth path/time quality unreliable while the estimated
trajectory and point cloud remain visually consistent. It is a GT diagnostic,
not an algorithm failure or ranking result.

## Trajectory overlays

Plots are grouped by dataset and also include runs outside the headline means.

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

Standalone HTML version: [ate_report.html](ate_report.html).
