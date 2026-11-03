# Benchmark results

Translation ATE is computed after timestamp association and SE(3) Umeyama
alignment, without scale correction. Lower is better.

HILTI 2022 and HILTI 2023 use sparse surveyed control points and a maximum
association interval of 2.0 s. S3E and Newer College use continuous ground
truth with a maximum association interval of 0.1 s. The sparse and continuous
results should therefore not be treated as one identical evaluation protocol.

The aggregate uses each dataset's documented inclusion set. Partial results
are retained and marked as `partial`; unavailable recordings are marked as
`not evaluated`. All five valid Newer College runs are included regardless of
their ATE.

## Aggregate results

| Dataset | Included runs | Included GT coverage | Mean ATE (m) | Median ATE (m) |
|---|---:|---:|---:|---:|
| HILTI 2022 | 10 | 103 control points | 0.260 | 0.082 |
| HILTI 2023 | 5 | 346.4 m control path | 0.058 | 0.052 |
| S3E | 17 | 10.42 km | 1.17 | 1.05 |
| Newer College 2020 | 5 | 5.94 km | 0.2639 | 0.1609 |

## Per-sequence ATE

The HILTI 2022 results use a dataset-specific handheld profile and sparse
control-point GT. Estimates are transformed from the IMU frame to the
measurement tip before alignment. Replays used a 0.5x sequence rate for stable
frame processing, so these results characterize trajectory accuracy rather
than real-time throughput.

| Dataset | Sequence | Agent | GT coverage | Estimated travel (m) | Metric | ATE RMSE (m) | Status |
|---|---|---:|---:|---:|---|---:|---|
| HILTI 2022 | exp01_construction_ground_level | reference | 13/13 | — | sparse ATE | 0.016 | complete |
| HILTI 2022 | exp02_construction_multilevel | reference | 22/22 | — | sparse ATE | 0.098 | complete |
| HILTI 2022 | exp03_construction_stairs | reference | 17/17 | — | sparse ATE | 0.113 | complete |
| HILTI 2022 | exp04_construction_upper_level | reference | 7/7 | — | sparse ATE | 0.031 | complete |
| HILTI 2022 | exp05_construction_upper_level_2 | reference | 6/6 | — | sparse ATE | 0.019 | complete |
| HILTI 2022 | exp06_construction_upper_level_3 | reference | 7/7 | — | sparse ATE | 0.033 | complete |
| HILTI 2022 | exp07_long_corridor | reference | 6/6 | — | sparse ATE | 0.114 | complete |
| HILTI 2022 | exp09_cupola | reference | 16/16 | — | sparse ATE | 1.810 | complete |
| HILTI 2022 | exp10_cupola_2 | reference | 3/14 | — | sparse ATE | 0.095 | partial |
| HILTI 2022 | exp11_lower_gallery | reference | 5/5 | — | sparse ATE | 0.296 | complete |
| HILTI 2022 | exp14_basement_2 | reference | 4/4 | — | sparse ATE | 0.066 | complete |
| HILTI 2022 | exp15_attic_to_upper_gallery | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp16_attic_to_upper_gallery_2 | reference | 7/9 | — | sparse ATE | 0.320 | partial |
| HILTI 2022 | exp18_corridor_lower_gallery_2 | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp21_outside_building | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2022 | exp23_the_sheldonian_slam | reference | — | — | sparse ATE | — | not evaluated |
| HILTI 2023 | Site 1 · Handheld 1 | reference | 81.32 m | 205.5 | sparse ATE | 0.030 | complete |
| HILTI 2023 | Site 1 · Handheld 2 | reference | 84.39 m | 163.9 | sparse ATE | 0.052 | complete |
| HILTI 2023 | Site 1 · Handheld 3 | reference | 81.89 m | 165.2 | sparse ATE | 0.108 | complete |
| HILTI 2023 | Site 1 · Handheld 4 | reference | 52.88 m | 300.0 | sparse ATE | 0.053 | complete |
| HILTI 2023 | Site 1 · Handheld 5 | reference | 45.94 m | 150.1 | sparse ATE | 0.045 | complete |
| S3E | Campus Road 3 | Alpha | 983.3 m | 940.0 | ATE | 0.876 | complete |
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
| Newer College | Short Experiment | reference | 1609.4 m · 98.1% | 1421.9 | ATE | 0.473 | complete |
| Newer College | Long Experiment | reference | 3063.4 m · 97.5% | 2599.8 | ATE | 0.460 | complete |
| Newer College | Quad with Dynamics | reference | 479.0 m · 98.0% | 474.9 | ATE | 0.133 | complete |
| Newer College | Dynamic Spinning | reference | 97.20 m · 81.6% | 77.10 | ATE | 0.093 | latest rerun · partial |
| Newer College | Parkland Mound | reference | 695.7 m · 96.0% | 654.9 | ATE | 0.161 | complete |

The `exp10` result covers only the first 3 of 14 control points before tracking
is lost after approximately 88 s. The `exp16` result covers 7 of 9 control
points before tracking is lost after approximately 156 s. Their RMSE values
describe the matched prefix only and are not used for complete-sequence
statistics. The four `not evaluated` recordings do not currently have complete
ROS bags in the local dataset.

For sparse-control HILTI runs, GT length is the surveyed control-point
polyline while estimated travel covers the full replay. The two length columns
are therefore not directly comparable.

Newer College estimates are written in the LiDAR frame, while its ground truth
uses the original dataset's base frame at the left-camera centre. Evaluation
therefore applies `T_world_base = T_world_lidar * T_lidar_base` before SE(3)
alignment. The original Rooster layout and static sensor transforms give a
0.1012 m lever arm:

```text
T_base_lidar =
[-0.70711  -0.70711  0  -0.084250]
[ 0.70711  -0.70711  0  -0.025000]
[ 0         0        1   0.050188]
[ 0         0        0   1       ]
```

The `Dynamic Spinning` rerun covers 81.6% of its GT because the first
approximately 4.7 s are consumed by IMU initialization; the other sequences
cover 96.0-98.1%.

The latest reruns are reported in the table with their coverage status. For
reproducibility, two
additional `exp03` full runs produced 0.119 m and 0.114 m RMSE, while an earlier
batch replay produced 2.021 m. An earlier `exp11` run produced 0.437 m, while
the latest complete run produced 0.296 m. These variations indicate that
run-to-run stability should be considered alongside a single best result.

## Trajectory overlays

Blue is Sapphire, orange dashed is GT, and crosses are HILTI controls.

### HILTI 2022

| Trajectory result | Trajectory result |
|---|---|
| ![HILTI 2022 · exp01 construction ground level](trajectories/hilti_2022_exp01_construction_ground_level.png) | ![HILTI 2022 · exp02 construction multilevel](trajectories/hilti_2022_exp02_construction_multilevel.png) |
| ![HILTI 2022 · exp03 construction stairs](trajectories/hilti_2022_exp03_construction_stairs.png) | ![HILTI 2022 · exp04 construction upper level](trajectories/hilti_2022_exp04_construction_upper_level.png) |
| ![HILTI 2022 · exp05 construction upper level 2](trajectories/hilti_2022_exp05_construction_upper_level_2.png) | ![HILTI 2022 · exp06 construction upper level 3](trajectories/hilti_2022_exp06_construction_upper_level_3.png) |
| ![HILTI 2022 · exp07 long corridor](trajectories/hilti_2022_exp07_long_corridor.png) | ![HILTI 2022 · exp09 cupola](trajectories/hilti_2022_exp09_cupola.png) |
| ![HILTI 2022 · exp10 cupola 2 partial](trajectories/hilti_2022_exp10_cupola_2.png) | ![HILTI 2022 · exp11 lower gallery](trajectories/hilti_2022_exp11_lower_gallery.png) |
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
