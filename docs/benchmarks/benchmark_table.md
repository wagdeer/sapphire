# Benchmark results

This page is generated from the 2026-11-06 benchmark run. Most trajectory plots draw the complete estimate from its first to last pose. Campus Road 3 Carol is the exception: its plot stops at 784.746 s immediately before a 3.299 s timestamp gap and divergence. Its ATE, path length, and GT coverage still describe the full run. The alignment is estimated from matched GT points and then applied to the full trajectory. Green circles mark starts and red squares mark the plotted end.

HILTI uses sparse control points and a 2.0 s association window. S3E and Newer College use 0.1 s. ATE is translation-part RMSE after SE(3) Umeyama alignment without scale correction.

## Aggregate results

| Collection | Complete runs | Mean ATE (m) | Median ATE (m) |
|---|---:|---:|---:|
| HILTI 2022 complete results | 9 | 0.225670 | 0.061493 |
| HILTI 2023 Site 1 Handheld | 5 | 0.068309 | 0.061729 |
| S3E headline 18 | 18 | 1.939448 | 0.980836 |
| Newer College | 5 | 0.263176 | 0.158747 |

HILTI 2022 `exp03_construction_stairs` covered only 7/17 control points in this run and is excluded from the complete-result mean. `exp10` and `exp16` were also partial. Every numeric result is still listed below with its actual status.

Campus Road 3 Carol covers the complete recording: 7,455 of 7,455 estimated poses have a timestamp-associated GT sample (100.0%), representing 7,455 of 63,771 dense GT samples. The trajectory jumps after a 3.299 s pose gap beginning at 784.746 s, so the plot omits the post-divergence segment; the reported ATE and path lengths include the full run.

## Per-sequence results

| Dataset | Sequence | Agent | Metric | Result (m) | GT coverage | Est. length (m) | GT length (m) | Status | In mean | Plot |
|---|---|---|---|---:|---:|---:|---:|---|---|---|
| HILTI 2022 | exp01_construction_ground_level | Reference | sparse ATE | 0.016 | 13/13 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp01_construction_ground_level.png) |
| HILTI 2022 | exp02_construction_multilevel | Reference | sparse ATE | 0.097 | 22/22 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| HILTI 2022 | exp03_construction_stairs | Reference | sparse ATE | 0.156 | 7/17 | — | — | incomplete_coverage | no | [plot](trajectories/hilti_2022_exp03_construction_stairs.png) |
| HILTI 2022 | exp04_construction_upper_level | Reference | sparse ATE | 0.030 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| HILTI 2022 | exp05_construction_upper_level_2 | Reference | sparse ATE | 0.020 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp05_construction_upper_level_2.png) |
| HILTI 2022 | exp06_construction_upper_level_3 | Reference | sparse ATE | 0.035 | 7/7 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| HILTI 2022 | exp07_long_corridor | Reference | sparse ATE | 0.138 | 6/6 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp07_long_corridor.png) |
| HILTI 2022 | exp09_cupola | Reference | sparse ATE | 1.408 | 16/16 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp09_cupola.png) |
| HILTI 2022 | exp10_cupola_2 | Reference | sparse ATE | 0.919 | 4/14 | — | — | incomplete_coverage | no | [plot](trajectories/hilti_2022_exp10_cupola_2.png) |
| HILTI 2022 | exp11_lower_gallery | Reference | sparse ATE | 0.226 | 5/5 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp11_lower_gallery.png) |
| HILTI 2022 | exp14_basement_2 | Reference | sparse ATE | 0.061 | 4/4 | — | — | ok | yes | [plot](trajectories/hilti_2022_exp14_basement_2.png) |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | Reference | sparse ATE | 0.386 | 7/9 | — | — | incomplete_coverage | no | [plot](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |
| Site 1 · Handheld 1 | site1_handheld_1 | Reference | sparse ATE | 0.026 | 4/4 | 205.5 | 81.32 | ok | yes | [plot](trajectories/hilti_site1_handheld_1.png) |
| Site 1 · Handheld 2 | site1_handheld_2 | Reference | sparse ATE | 0.062 | 4/4 | 163.9 | 84.39 | ok | yes | [plot](trajectories/hilti_site1_handheld_2.png) |
| Site 1 · Handheld 3 | site1_handheld_3 | Reference | sparse ATE | 0.122 | 4/4 | 164.6 | 81.89 | ok | yes | [plot](trajectories/hilti_site1_handheld_3.png) |
| Site 1 · Handheld 4 | site1_handheld_4 | Reference | sparse ATE | 0.080 | 3/3 | 299.6 | 52.88 | warning; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across | yes | [plot](trajectories/hilti_site1_handheld_4.png) |
| Site 1 · Handheld 5 | site1_handheld_5 | Reference | sparse ATE | 0.052 | 3/3 | 149.3 | 45.94 | ok | yes | [plot](trajectories/hilti_site1_handheld_5.png) |
| Site 2 · Handheld 4 | site2_handheld_4 | Reference | sparse ATE | 0.102 | 2/2 | 65.49 | 6.68 | ok | no | [plot](trajectories/hilti_site2_handheld_4.png) |
| Site 2 · Handheld 5 | site2_handheld_5 | Reference | sparse ATE | 14.318 | 4/4 | 126.1 | 53.06 | ok | no | [plot](trajectories/hilti_site2_handheld_5.png) |
| Site 2 · Handheld 6 | site2_handheld_6 | Reference | sparse ATE | — | 2/2 | 110.9 | 3.38 | invalid; 1 GT gap(s) exceed 120.000 s; gaps were not interpolated across | no | — |
| Site 2 · Robot 2 | site2_robot_2 | Reference | sparse ATE | — | — | — | — | failed | no | — |
| Campus Road 3 | S3E_Campus_Road_3 | Alpha | ATE | 0.906 | 100.0% | 940.5 | 983.3 | ok | yes | [plot](trajectories/s3e_campus_road_3_alpha.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Bob | ATE | 4.420 | 100.0% | 937.6 | 967.6 | ok | yes | [plot](trajectories/s3e_campus_road_3_bob.png) |
| Campus Road 3 | S3E_Campus_Road_3 | Carol | ATE | 12.660 | 100.0% | 979.2 | 986.9 | warning; full-run metrics; 7455/7455 associated estimate poses (7455/63771 GT samples); plot truncated at 784.746 s before divergence | yes | [plot](trajectories/s3e_campus_road_3_carol.png) |
| Dormitory 1 | S3E_Dormitory_1 | Alpha | ATE | 0.490 | 96.1% | 724.0 | 727.0 | ok | yes | [plot](trajectories/s3e_dormitory_1_alpha.png) |
| Dormitory 1 | S3E_Dormitory_1 | Bob | ATE | 1.474 | 96.2% | 723.2 | 719.3 | ok | yes | [plot](trajectories/s3e_dormitory_1_bob.png) |
| Dormitory 1 | S3E_Dormitory_1 | Carol | ATE | 0.813 | 95.4% | 737.9 | 721.9 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_dormitory_1_carol.png) |
| Laboratory 1 | S3E_Laboratory_1 | Alpha | endpoint | 928.511 | 100.0% | 1.01 | 929.1 | warning; GT repair applied to 1 malformed line(s) | no | [plot](trajectories/s3e_laboratory_1_alpha.png) |
| Laboratory 1 | S3E_Laboratory_1 | Bob | endpoint | 96.932 | 100.0% | 0.47 | 97.26 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_1_bob.png) |
| Laboratory 1 | S3E_Laboratory_1 | Carol | endpoint | 158.975 | 100.0% | 3.51 | 158.5 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_1_carol.png) |
| Laboratory 2 | S3E_Laboratory_2 | Alpha | endpoint | 382.764 | 100.0% | 7.54 | 390.1 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_2_alpha.png) |
| Laboratory 2 | S3E_Laboratory_2 | Bob | endpoint | 86.842 | 100.0% | 3.13 | 86.43 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_2_bob.png) |
| Laboratory 2 | S3E_Laboratory_2 | Carol | endpoint | 111.473 | 100.0% | 0.31 | 111.3 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_2_carol.png) |
| Laboratory 3 | S3E_Laboratory_3 | Alpha | endpoint | 61.361 | 100.0% | 1.37 | 61.55 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_3_alpha.png) |
| Laboratory 3 | S3E_Laboratory_3 | Bob | endpoint | 17.032 | 100.0% | 0.09 | 16.97 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_3_bob.png) |
| Laboratory 3 | S3E_Laboratory_3 | Carol | endpoint | 138.537 | 100.0% | 0.46 | 138.4 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_3_carol.png) |
| Laboratory 4 | S3E_Laboratory_4 | Alpha | endpoint | 271.334 | 100.0% | 3.78 | 271.8 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_4_alpha.png) |
| Laboratory 4 | S3E_Laboratory_4 | Bob | endpoint | 71.298 | 100.0% | 0.16 | 71.33 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_4_bob.png) |
| Laboratory 4 | S3E_Laboratory_4 | Carol | endpoint | 71.869 | 100.0% | 3.93 | 72.30 | warning; normalized 2 GT quaternion(s) | no | [plot](trajectories/s3e_laboratory_4_carol.png) |
| Library 1 | S3E_Library_1 | Alpha | ATE | 0.965 | 93.0% | 520.4 | 507.6 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_alpha.png) |
| Library 1 | S3E_Library_1 | Bob | ATE | 1.096 | 98.9% | 531.1 | 517.2 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_bob.png) |
| Library 1 | S3E_Library_1 | Carol | ATE | 1.379 | 96.8% | 508.8 | 498.9 | warning; 4 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_library_1_carol.png) |
| Library 2 | S3E_Library_2 | Alpha | ATE | 0.828 | 100.0% | 503.8 | 521.6 | ok | yes | [plot](trajectories/s3e_library_2_alpha.png) |
| Library 2 | S3E_Library_2 | Bob | ATE | 0.838 | 100.0% | 506.7 | 523.9 | ok | yes | [plot](trajectories/s3e_library_2_bob.png) |
| Library 2 | S3E_Library_2 | Carol | ATE | 0.932 | 100.0% | 505.6 | 520.3 | ok | yes | [plot](trajectories/s3e_library_2_carol.png) |
| Square 1 | S3E_Square_1 | Alpha | ATE | 1.165 | 91.4% | 542.9 | 546.0 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_square_1_alpha.png) |
| Square 1 | S3E_Square_1 | Bob | ATE | 0.735 | 97.6% | 499.1 | 496.5 | ok | yes | [plot](trajectories/s3e_square_1_bob.png) |
| Square 1 | S3E_Square_1 | Carol | ATE | 0.997 | 97.6% | 532.1 | 529.2 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | yes | [plot](trajectories/s3e_square_1_carol.png) |
| Square 3 | S3E_Square_3 | Alpha | ATE | 2.278 | 100.0% | 460.2 | 487.4 | ok | yes | [plot](trajectories/s3e_square_3_alpha.png) |
| Square 3 | S3E_Square_3 | Bob | ATE | 2.000 | 100.0% | 558.5 | 569.6 | ok | yes | [plot](trajectories/s3e_square_3_bob.png) |
| Square 3 | S3E_Square_3 | Carol | ATE | 0.935 | 100.0% | 541.9 | 563.8 | ok | yes | [plot](trajectories/s3e_square_3_carol.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Alpha | ATE | 0.153 | 96.4% | 624.4 | 36.10 | warning; 1 GT gap(s) exceed 5.000 s; gaps were not interpolated across | no | [plot](trajectories/s3e_teaching_building_1_alpha.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Bob | ATE | 0.945 | 93.3% | 742.0 | 43.36 | warning; 3 GT gap(s) exceed 5.000 s; gaps were not interpolated across | no | [plot](trajectories/s3e_teaching_building_1_bob.png) |
| Teaching Building 1 | S3E_Teaching_Building_1 | Carol | ATE | 0.575 | 98.0% | 655.8 | 68.08 | warning; 2 GT gap(s) exceed 5.000 s; gaps were not interpolated across | no | [plot](trajectories/s3e_teaching_building_1_carol.png) |
| Tunnel 1 | S3E_Tunnel_1 | Alpha | ATE | 81.545 | 100.0% | 986.1 | 522.0 | ok | no | [plot](trajectories/s3e_tunnel_1_alpha.png) |
| Tunnel 1 | S3E_Tunnel_1 | Bob | ATE | 52.163 | 100.0% | 412.5 | 502.4 | ok | no | [plot](trajectories/s3e_tunnel_1_bob.png) |
| Tunnel 1 | S3E_Tunnel_1 | Carol | ATE | 4.640 | 100.0% | 473.5 | 501.1 | ok | no | [plot](trajectories/s3e_tunnel_1_carol.png) |
| Short Experiment | 01_short_experiment | Reference | ATE | 0.470 | 100.0% | 1419.9 | 1609.4 | ok | yes | [plot](trajectories/newer_college_01_short_experiment.png) |
| Long Experiment | 02_long_experiment | Reference | ATE | 0.455 | 100.0% | 2595.0 | 3063.4 | ok | yes | [plot](trajectories/newer_college_02_long_experiment.png) |
| Quad with Dynamics | 05_quad_with_dynamics | Reference | ATE | 0.134 | 99.4% | 474.5 | 479.0 | ok | yes | [plot](trajectories/newer_college_05_quad_with_dynamics.png) |
| Dynamic Spinning | 06_dynamic_spinning | Reference | ATE | 0.097 | 100.0% | 76.45 | 97.20 | ok | yes | [plot](trajectories/newer_college_06_dynamic_spinning.png) |
| Parkland Mound | 07_parkland_mound | Reference | ATE | 0.159 | 100.0% | 654.0 | 695.7 | ok | yes | [plot](trajectories/newer_college_07_parkland_mound.png) |

## Evaluation failures

- **Site 2 · Handheld 6 / site2_handheld_6 / Reference**: TrajectoryError: evo APE calculation failed: Degenerate covariance rank, Umeyama alignment is not possible
- **Site 2 · Robot 2 / site2_robot_2 / Reference**: ValueError: /workspace/results/nightly_2026-11-06/hilti_2023/site2_robot_2/trajectory.tum: only 1 robot control point(s) matched

S3E Laboratory 1–4 provide endpoint-only ground truth and use endpoint error; they are not mixed with continuous-trajectory ATE.

## Trajectory overlays

Plots are grouped by dataset and also include runs outside the headline means. The Campus Road 3 Carol plot is explicitly truncated at its pre-divergence pose.

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
