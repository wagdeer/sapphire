# Benchmark results

Translation ATE is the RMSE of position error after timestamp association and
SE(3) Umeyama alignment, without scale correction. Lower is better. The mean
and median in the summary are unweighted across runs. They are not weighted by
path length, and they are not one RMSE computed on every pose at once.

Two association settings are in use, so the datasets are not one protocol:

- HILTI 2022 and HILTI 2023 use sparse surveyed control points and a maximum
  gap of 2.0 s. HILTI 2023 reference orientations are constant, so only
  translation is scored. Each Site 1 score uses 3 or 4 control points. The
  "GT coverage" length on those rows is the polyline through the control
  points. It is shorter than the sensor path between them.
- S3E and Newer College use a maximum gap of 0.1 s. Dormitory 1, Library 1,
  and Square 1 also have a constant ground-truth quaternion. Library 2,
  Square 3, and Campus Road 3 include orientation.

What enters the summary:

- HILTI 2022: the current rerun completes 11 of the 12 downloaded sequences;
  `exp16` remains partial at 7/9 control points. The like-for-like summary
  uses the same 10 runs and 103 control points as the previous baseline so
  that the comparison is not changed by newly recovered `exp10` coverage.
  `exp10` is now complete at 14/14 points but has a large 1.956 m ATE, so the
  all-complete aggregate uses 11 runs and 117 points and is reported
  separately. `exp15`, `exp18`, `exp21`, and `exp23` were not replayed.
  `exp03` has two stable current-profile runs at 0.1317 and 0.1333 m.
- HILTI 2023: Site 1 handheld 1–5 only. Site 2 was also evaluated and is
  listed after the table. It is not in the 0.058 m mean.
- S3E: the 18 runs in the per-sequence table, including Campus Road 3 Bob.
  Laboratory 1–4 only have endpoint ground truth, so they are not ATE.
  Teaching Building 1 is left out because its control path is a few dozen
  metres against an estimate of about 600 m. Tunnel 1 was run and is withheld
  until the estimate and the ground truth are confirmed to be the same
  trajectory. Campus Road 1 and Campus Road 2 are not in this table.
- Newer College: all five runs, including Dynamic Spinning at 81.6% ground-truth
  coverage. Before alignment, the estimate is transformed from the Ouster
  LiDAR frame into the dataset base frame at the left-camera centre. The lever
  arm of that rigid transform is 0.1012 m. The summary uses the unrounded
  RMSE of each run.

## Aggregate results

| Dataset | Included runs | Included GT coverage | Mean ATE (m) | Median ATE (m) |
|---|---:|---:|---:|---:|
| HILTI 2022 · like-for-like | 10 | 103 control points | 0.197 | 0.077 |
| HILTI 2022 · current all-complete | 11 | 117 control points | 0.357 | 0.093 |
| HILTI 2023 Site 1 handheld | 5 | 346.4 m control path | 0.058 | 0.052 |
| S3E | 18 | 11.39 km | 1.29 | 1.11 |
| Newer College 2020 | 5 | 5.94 km | 0.2639 | 0.1609 |

## Per-sequence ATE

The HILTI 2022 results use a dataset-specific handheld profile and sparse
control-point GT. Estimates are transformed from the IMU frame to the
measurement tip before alignment.

| Dataset | Sequence | Agent | GT coverage | Estimated travel (m) | Metric | ATE RMSE (m) | Status |
|---|---|---:|---:|---:|---|---:|---|
| HILTI 2022 | exp01_construction_ground_level | reference | 13/13 | — | sparse ATE | 0.016 | complete |
| HILTI 2022 | exp02_construction_multilevel | reference | 22/22 | — | sparse ATE | 0.093 | complete |
| HILTI 2022 | exp03_construction_stairs | reference | 17/17 | — | sparse ATE | 0.132 | complete |
| HILTI 2022 | exp04_construction_upper_level | reference | 7/7 | — | sparse ATE | 0.031 | complete |
| HILTI 2022 | exp05_construction_upper_level_2 | reference | 6/6 | — | sparse ATE | 0.019 | complete |
| HILTI 2022 | exp06_construction_upper_level_3 | reference | 7/7 | — | sparse ATE | 0.036 | complete |
| HILTI 2022 | exp07_long_corridor | reference | 6/6 | — | sparse ATE | 0.118 | complete |
| HILTI 2022 | exp09_cupola | reference | 16/16 | — | sparse ATE | 1.239 | complete |
| HILTI 2022 | exp10_cupola_2 | reference | 14/14 | — | sparse ATE | 1.956 | complete |
| HILTI 2022 | exp11_lower_gallery | reference | 5/5 | — | sparse ATE | 0.229 | complete |
| HILTI 2022 | exp14_basement_2 | reference | 4/4 | — | sparse ATE | 0.060 | complete |
| HILTI 2022 | exp15_attic_to_upper_gallery | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | reference | 7/9 | — | sparse ATE | 0.304 | partial |
| HILTI 2022 | exp18_corridor_lower_gallery_2 | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp21_outside_building | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp23_the_sheldonian_slam | reference | — | — | sparse ATE | — | not evaluated |

### HILTI 2022 ESKF ablation

The production profile keeps directional pose injection and a position-based
velocity observer. Replacing its pose gain with ordinary Kalman pose rows
degrades both representative sequences substantially.

| Sequence | Directional gain 8 | Directional gain 0 | Kalman pose gain 8 | Kalman pose gain 0 |
|---|---:|---:|---:|---:|
| `exp03_construction_stairs` | 0.132 m | **0.086 / 0.104 m** | 0.645 m | 35.3% GT coverage |
| `exp07_long_corridor` | **0.118 m** | 0.165 m | 0.957 m | 7.835 m |

With directional pose retained, the velocity-gain sweep is:

| Sequence | gain 0 | gain 2 | gain 4 | gain 6 | gain 8 |
|---|---:|---:|---:|---:|---:|
| `exp03_construction_stairs` | **0.086 / 0.104 m** | 29.4% coverage, twice | 0.153 m | 0.160 m | 0.132 m |
| `exp07_long_corridor` | 0.165 m | 0.219 m | 0.156 m | 0.127 m | **0.118 m** |

`exp09_cupola` remains complete at 1.239 m with gain 8, while gain 0 loses
tracking and reaches only 37.5% coverage. Gain 2 reproducibly fails at about
69 s in both runs. The detailed experiment, logs, plots, and NIS discussion
are in `docs/gal3-experiments-2026-11-05/` in the workspace.
| HILTI 2023 | Site 1 · Handheld 1 | reference | 81.32 m | 205.5 | sparse ATE | 0.030 | complete |
| HILTI 2023 | Site 1 · Handheld 2 | reference | 84.39 m | 163.9 | sparse ATE | 0.052 | complete |
| HILTI 2023 | Site 1 · Handheld 3 | reference | 81.89 m | 165.2 | sparse ATE | 0.108 | complete |
| HILTI 2023 | Site 1 · Handheld 4 | reference | 52.88 m | 300.0 | sparse ATE | 0.053 | complete · one GT gap over 120 s |
| HILTI 2023 | Site 1 · Handheld 5 | reference | 45.94 m | 150.1 | sparse ATE | 0.045 | complete |
| S3E | Campus Road 3 | Alpha | 983.3 m | 940.0 | ATE | 0.876 | complete |
| S3E | Campus Road 3 | Bob | 967.6 m | 937.4 | ATE | 3.330 | complete |
| S3E | Campus Road 3 | Carol | 986.9 m | 948.9 | ATE | 1.050 | complete |
| S3E | Dormitory 1 | Alpha | 727.0 m | 724.0 | ATE | 0.478 | complete |
| S3E | Dormitory 1 | Bob | 719.3 m | 722.6 | ATE | 1.378 | complete |
| S3E | Dormitory 1 | Carol | 721.9 m | 738.0 | ATE | 0.804 | complete |
| S3E | Library 1 | Alpha | 507.6 m | 520.4 | ATE | 0.988 | complete |
| S3E | Library 1 | Bob | 517.2 m | 530.6 | ATE | 1.671 | complete |
| S3E | Library 1 | Carol | 498.9 m | 508.1 | ATE | 1.339 | complete |
| S3E | Library 2 | Alpha | 521.6 m | 505.0 | ATE | 1.238 | complete |
| S3E | Library 2 | Bob | 523.9 m | 505.5 | ATE | 1.683 | complete |
| S3E | Library 2 | Carol | 520.3 m | 505.8 | ATE | 0.923 | complete |
| S3E | Square 1 | Alpha | 546.0 m | 542.7 | ATE | 1.166 | complete |
| S3E | Square 1 | Bob | 496.5 m | 495.9 | ATE | 0.748 | complete |
| S3E | Square 1 | Carol | 529.2 m | 531.7 | ATE | 0.866 | complete |
| S3E | Square 3 | Alpha | 487.4 m | 463.7 | ATE | 1.835 | complete |
| S3E | Square 3 | Bob | 569.6 m | 559.8 | ATE | 1.932 | complete |
| S3E | Square 3 | Carol | 563.8 m | 541.0 | ATE | 0.912 | complete |
| Newer College | Short Experiment | reference | 1609.4 m · 98.1% | 1421.9 | ATE | 0.4729 | complete |
| Newer College | Long Experiment | reference | 3063.4 m · 97.5% | 2599.8 | ATE | 0.4597 | complete |
| Newer College | Quad with Dynamics | reference | 479.0 m · 98.0% | 474.9 | ATE | 0.1332 | complete |
| Newer College | Dynamic Spinning | reference | 97.20 m · 81.6% | 77.10 | ATE | 0.0927 | included · 81.6% of GT |
| Newer College | Parkland Mound | reference | 695.7 m · 96.0% | 654.9 | ATE | 0.1609 | complete |

## Evaluated, not in the summary

These runs used the same alignment settings as the table above. They are not
part of the four headline means.

| Dataset | Sequence | Agent | GT coverage | ATE RMSE (m) | Why it is outside the mean |
|---|---|---|---|---:|---|
| HILTI 2023 | Site 2 · Handheld 4 | reference | 6.68 m, 2 control points | 0.034 | Only two control points |
| HILTI 2023 | Site 2 · Handheld 5 | reference | 53.06 m, 4/4 | 14.251 | Diverged on this recording |
| HILTI 2023 | Site 2 · Handheld 6 | reference | — | — | Umeyama alignment failed: degenerate covariance |
| HILTI 2023 | Site 2 · Robot 2 | reference | 56.62 m, 3 control points | 0.058 | Robot platform, separate extrinsic |
| S3E | Tunnel 1 | Alpha | 522.0 m | 40.780 | Withheld: full time coverage, but the estimate is 78 m shorter than GT and the pairing is not confirmed |
| S3E | Tunnel 1 | Bob | 502.4 m | 47.018 | Same withheld check as Alpha |
| S3E | Tunnel 1 | Carol | 501.1 m | 3.309 | Same withheld check; path is 25 m shorter than GT |

Teaching Building 1 was scored as well (Alpha 0.142 m, Bob 0.840 m, Carol
0.577 m). Its control path is 36–68 m while the estimates are about 600–740 m,
with gaps longer than 5 s, so those scores are not mixed with the continuous
runs above. Laboratory 1–4 only provide endpoint ground truth and are not
reported as ATE.

Campus Road 3 Bob is inside the S3E mean. A trajectory plot was not produced
for that run.

## Trajectory overlays

Blue is the full Sapphire estimate, orange dashed connects GT control points,
and crosses mark the controls.

### HILTI 2022

| Trajectory result | Trajectory result |
|---|---|
| ![HILTI 2022 · exp01 construction ground level](trajectories/hilti_2022_exp01_construction_ground_level.png) | ![HILTI 2022 · exp02 construction multilevel](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| ![HILTI 2022 · exp03 construction stairs](trajectories/hilti_2022_exp03_construction_stairs.png) | ![HILTI 2022 · exp04 construction upper level](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| ![HILTI 2022 · exp05 construction upper level 2](trajectories/hilti_2022_exp05_construction_upper_level_2.png) | ![HILTI 2022 · exp06 construction upper level 3](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| ![HILTI 2022 · exp07 long corridor](trajectories/hilti_2022_exp07_long_corridor.png) | ![HILTI 2022 · exp09 cupola](trajectories/hilti_2022_exp09_cupola.png) |
| ![HILTI 2022 · exp10 cupola 2 recovered full coverage](trajectories/hilti_2022_exp10_cupola_2.png) | ![HILTI 2022 · exp11 lower gallery](trajectories/hilti_2022_exp11_lower_gallery.png) |
| ![HILTI 2022 · exp14 basement 2](trajectories/hilti_2022_exp14_basement_2.png) | ![HILTI 2022 · exp16 attic to upper gallery 2 partial](trajectories/hilti_2022_exp16_attic_to_upper_gallery_2.png) |

### HILTI 2023

| Trajectory result | Trajectory result |
|---|---|
| ![HILTI · Site 1 · Handheld 1](trajectories/hilti_site1_handheld_1.png) | ![HILTI · Site 1 · Handheld 2](trajectories/hilti_site1_handheld_2.png) |
| ![HILTI · Site 1 · Handheld 3](trajectories/hilti_site1_handheld_3.png) | ![HILTI · Site 1 · Handheld 4](trajectories/hilti_site1_handheld_4.png) |
| ![HILTI · Site 1 · Handheld 5](trajectories/hilti_site1_handheld_5.png) | |

### S3E

| Trajectory result | Trajectory result |
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

| Trajectory result | Trajectory result |
|---|---|
| ![Newer College · Short Experiment](trajectories/newer_college_01_short_experiment.png) | ![Newer College · Long Experiment](trajectories/newer_college_02_long_experiment.png) |
| ![Newer College · Quad with Dynamics](trajectories/newer_college_05_quad_with_dynamics.png) | ![Newer College · Dynamic Spinning](trajectories/newer_college_06_dynamic_spinning.png) |
| ![Newer College · Parkland Mound](trajectories/newer_college_07_parkland_mound.png) | |
