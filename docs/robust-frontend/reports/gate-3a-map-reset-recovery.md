# Gate 3A: Bounded Local-map Recovery

## Status

Accepted as a coverage-recovery gate. The global accuracy of the recovered
`exp10` segment is not yet acceptable, but the local-map deadlock is broken and
the other four sequences retain complete coverage.

- Branch: `feat/robust-frontend`
- Source commit: `583051f`
- Run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3a-map-reset-recovery_20261107T170506Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  one sequential worker

## Mechanism

After five consecutive registration rejections, the pipeline:

1. keeps the current IMU/deskew pose;
2. replaces the local registration submap with the current scan;
3. starts a five-frame warm-up in which the next accepted scans are forced into
   the new local map;
4. resumes the normal strict registration gate;
5. allows at most three recovery attempts per run.

The normal registration thresholds, ESKF gains, scan cadence, and global PGO
configuration are unchanged.

## Validation

- ROS 1 build: passed.
- ROS 1 tests: `18/18` passed.
- ROS 2 wrapper build: passed with the existing serial-LTRANS LTO warnings.
- Full HILTI replay: `5/5` bags reached EOF.

## ATE Results

| Case | Status | Coverage G0 -> G3A | RMSE G0 -> G3A | P95 G0 -> G3A |
|---|---|---:|---:|---:|
| HILTI22 `exp03` | ok | 17/17 -> 17/17 | 0.130519 -> 0.136790 m | 0.216704 -> 0.234644 m |
| HILTI22 `exp07` | ok | 6/6 -> 6/6 | 0.133626 -> 0.132115 m | 0.204103 -> 0.203825 m |
| HILTI22 `exp09` | ok | 16/16 -> 16/16 | **1.349571 -> 1.140626 m** | **2.704542 -> 1.955036 m** |
| HILTI22 `exp10` | incomplete | **3/14 -> 10/14** | 0.053004 -> 3.156892 m | 0.064724 -> 4.718521 m |
| HILTI23 `site2_handheld_5` | ok | 4/4 -> 4/4 | 14.244510 -> 14.316365 m | 20.504442 -> 20.945751 m |

The `exp10` Gate 0 RMSE covered only the initial 3/14 prefix. It cannot be
compared directly with the recovered 10/14 trajectory.

## Recovery Behavior

| Case | Recoveries | Trajectory poses G0 -> G3A | Longest zero-inlier run |
|---|---:|---:|---:|
| `exp03` | 0 | 3084 -> 3084 | 0 |
| `exp07` | 0 | 1313 -> 1313 | 0 |
| `exp09` | 2 | 4446 -> 4441 | 1 |
| `exp10` | 3 | **869 -> 2752** | **919** |
| `site2_handheld_5` | 0 | 2050 -> 2050 | 0 |

`exp09` recovered near the end after two consecutive-rejection runs. Its
estimated path remained close to the control-point envelope and its P95 error
improved by about 28%.

`exp10` was initially trapped after frame 869. The first reset extended the
trajectory, but two subsequent resets occurred within 20 seconds. The resulting
trajectory covers 10/14 control points but has a several-metre local offset and
obvious jumps between recovered segments.

## Decision

Accept for coverage and deadlock recovery, with a major unresolved limitation:
the recovered segment is locally continuous but not globally re-associated
with the pre-reset map.

The next work item should connect reset segments through:

- a bounded scan-to-scan bootstrap while the new local map is too small;
- segment metadata in PGO so a reset is not represented as an ordinary
  odometry edge;
- loop/global-map reacquisition to correct the post-reset transform.

This gate must not be described as a general accuracy improvement until those
steps are implemented.

## Artifacts

- [Evaluation table](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3a-map-reset-recovery_20261107T170506Z/evaluation/summary.md)
- [Registration metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3a-map-reset-recovery_20261107T170506Z/evaluation/log_metrics.json)
- [Run manifest](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3a-map-reset-recovery_20261107T170506Z/run_manifest.json)
- [Trajectory plots](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3a-map-reset-recovery_20261107T170506Z/evaluation/plots)
