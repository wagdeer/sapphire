# Gate 0: Baseline and Observability

## Status

Accepted. This gate adds diagnostics only; it does not change registration
acceptance, ESKF correction, PGO, or trajectory output behavior.

- Branch: `feat/robust-frontend`
- Source commit before Gate 0 code: `5bbb9df`
- Baseline worktree: dirty, including pre-existing `gal3_right` changes
- Run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-0-observability_20261107T130434Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  sequential replay

The worktree fingerprint at run start was:

```text
status_sha256=6e729d08501a800e41c7fd22fb4a373e0e6548c5c91ca68cccc481b3cc844e88
diff_sha256=36f092cc1c3d5c7dd8ae4c380c631adf920bff76f30e5e75515a0d2b14bd58a0
```

## Changes

- `RegistrationResult` now records source point count, solver iterations,
  inlier ratio, solver error, combined Hessian eigenvalues, and separate
  rotation/translation block condition numbers.
- Every registration emits a warning-level `[registration-quality]` record so
  the standard benchmark log level retains it.
- The pipeline tracks current and maximum consecutive rejection counts,
  acceptance run length, and map updates.
- A fixed five-sequence runner and evaluator were added under
  `docs/robust-frontend/scripts/`.

Code:

- [Registration result fields](/home/bindeer/git/sapphire_ws/src/sapphire/include/sapphire/types.hpp:112)
- [Registration diagnostics](/home/bindeer/git/sapphire_ws/src/sapphire/src/odometry/registration.cpp:132)
- [Pipeline counters](/home/bindeer/git/sapphire_ws/src/sapphire/include/sapphire/odometry/pipeline.hpp:240)
- [Fixed runner](/home/bindeer/git/sapphire_ws/src/sapphire/docs/robust-frontend/scripts/run_hilti_gate.py)
- [Gate evaluator](/home/bindeer/git/sapphire_ws/src/sapphire/docs/robust-frontend/scripts/evaluate_hilti_gate.py)

## Validation

- ROS 1 rebuild: passed, no compiler warnings reported in the final build.
- ROS 1 tests: `18/18` passed.
- ROS 2 wrapper build: passed. LTO emitted serial-compilation warnings only.
- Gate replay: `5/5` runs produced valid, non-empty trajectories.

Two preliminary replays were discarded:

1. The quality lines were initially info-level and were filtered from benchmark
   logs.
2. The first parallel scheduler launched five jobs with three worker slots and
   reused ROS master ports, causing premature shutdown.

Neither discarded run is used for a numerical conclusion.

## ATE Results

Metric: HILTI sparse-control-point translation RMSE, IMU-to-pole-tip
transform, SE(3) Umeyama alignment, no scale correction, 2 s association.

| Case | Status | Coverage | RMSE | Median | P95 | Max | Estimated / GT path |
|---|---|---:|---:|---:|---:|---:|---:|
| HILTI22 `exp03` | ok | 17/17 | 0.130519 m | 0.087079 m | 0.216704 m | 0.223705 m | 240.70 / 186.76 m |
| HILTI22 `exp07` | ok | 6/6 | 0.133626 m | 0.103783 m | 0.204103 m | 0.227146 m | 114.74 / 102.82 m |
| HILTI22 `exp09` | ok | 16/16 | 1.349571 m | 0.663182 m | 2.704542 m | 3.441977 m | 202.57 / 96.96 m |
| HILTI22 `exp10` | incomplete | 3/14 | 0.053004 m | 0.055504 m | 0.064724 m | 0.065748 m | 57.33 / 92.84 m |
| HILTI23 `site2_handheld_5` | ok | 4/4 | 14.244510 m | 11.650821 m | 20.504442 m | 21.892457 m | 125.92 / 53.06 m |

`exp10`'s low RMSE applies only to the 3/14 matched prefix and is not a valid
full-sequence result.

Compared with the previous full-run reference:

| Case | Previous RMSE | Gate 0 RMSE | Relative |
|---|---:|---:|---:|
| `exp03` | 0.128513 | 0.130519 | +1.56% |
| `exp07` | 0.126219 | 0.133626 | +5.87% |
| `exp09` | 1.303564 | 1.349571 | +3.53% |
| `exp10` | 0.073697, 3/14 | 0.053004, 3/14 | not rankable |
| `site2_handheld_5` | 14.283640 | 14.244510 | -0.27% |

The small differences are consistent with a diagnostic-only change and normal
run-to-run variation.

## Registration and Filter Health

| Case | Registrations | Rejected | Zero inliers | Longest zero-inlier run | Inlier ratio median | Rotation p95 | Translation p95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `exp03` | 3083 | 0 | 0 | 0 | 0.903 | 83.5 | 25.9 |
| `exp07` | 1315 | 3 | 0 | 0 | 0.935 | 35783 | 198 |
| `exp09` | 4459 | 14 | 0 | 0 | 0.960 | 1334 | 38.7 |
| `exp10` | 3697 | 2829 | 2818 | **2818** | 0.000 | 1295 | 51.8 |
| `site2_handheld_5` | 2049 | 0 | 0 | 0 | 0.936 | 359 | 14.8 |

These results establish two distinct failure classes:

1. `exp10` loses all correspondences for a long contiguous period. The current
   frontend continues IMU propagation and cannot reacquire the map.
2. `site2_handheld_5` accepts every registration with high inlier ratio, yet
   the trajectory is grossly wrong. Inlier count alone cannot establish
   correctness.

The long corridor also shows a large rotational condition number even while
inlier ratio remains high. A combined 6x6 condition threshold is therefore not
sufficient to describe localizability.

## Decision

Accept Gate 0 as the reproducible baseline. Proceed to Gate 1, quality-aware
soft fusion, using the measured distributions rather than arbitrary thresholds.

Gate 1 must not treat a single accepted registration as a fully trusted pose.
It should distinguish high-quality, partial, degenerate, and invalid
observations, and must guarantee that a zero-inlier run cannot repeatedly enter
the local map.

## Artifacts

- [Run manifest](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-0-observability_20261107T130434Z/run_manifest.json)
- [Run status](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-0-observability_20261107T130434Z/status.json)
- [Evaluation summary](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-0-observability_20261107T130434Z/evaluation/summary.md)
- [Log metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-0-observability_20261107T130434Z/evaluation/log_metrics.json)
