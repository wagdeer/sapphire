# Gate 3B: Bounded Scan-to-map Bootstrap

## Status

Accepted for completion and recovery continuity. All five HILTI cases reached
full control-point coverage, including `exp10_cupola_2` at 14/14. The recovered
trajectory still has a large global offset and is not a global relocalization
solution.

- Branch: `feat/robust-frontend`
- Source commit: `4117654`
- Full-run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-bounded-bootstrap_20261107T181800Z`
- Isolated `exp10` screen:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-screen-exp10_20261107T180345Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  one sequential worker

## Mechanism

Gate 3A reset the local registration map after five consecutive rejections but
could still fail during the first few frames of the new map. Gate 3B adds a
five-frame bootstrap:

- a rejected frame during bootstrap is inserted at the IMU/deskew prior pose;
- the local target is rebuilt from those scans;
- registration remains disabled as a correction source for those bootstrap
  frames;
- after five scans, the normal hard registration gate resumes.

Normal registration acceptance, map thresholds, ESKF gains, and PGO settings
are unchanged. Each run remains bounded to three reset attempts.

## Validation

- ROS 1 build: passed.
- ROS 1 tests: `18/18` passed.
- Full HILTI replay: `5/5` bags reached EOF and all five reached full
  control-point coverage.

## Full-run ATE

| Case | Coverage | RMSE | Median | P95 | Max |
|---|---:|---:|---:|---:|---:|
| HILTI22 `exp03` | 17/17 | 0.141931 m | 0.090340 m | 0.228729 m | 0.235964 m |
| HILTI22 `exp07` | 6/6 | 0.135650 m | 0.107631 m | 0.205374 m | 0.228650 m |
| HILTI22 `exp09` | 16/16 | 1.578837 m | 0.949381 m | 3.356628 m | 4.409046 m |
| HILTI22 `exp10` | **14/14** | 2.458338 m | 1.897382 m | 4.456982 m | 5.213422 m |
| HILTI23 `site2_handheld_5` | 4/4 | 14.242283 m | 11.647415 m | 20.502501 m | 21.891991 m |

Gate 3A -> Gate 3B:

| Case | Coverage | RMSE | P95 |
|---|---:|---:|---:|
| `exp03` | 17/17 -> 17/17 | 0.136790 -> 0.141931 m | 0.234644 -> 0.228729 m |
| `exp07` | 6/6 -> 6/6 | 0.132115 -> 0.135650 m | 0.203825 -> 0.205374 m |
| `exp09` | 16/16 -> 16/16 | 1.140626 -> 1.578837 m | 1.955036 -> 3.356628 m |
| `exp10` | **10/14 -> 14/14** | **3.156892 -> 2.458338 m** | **4.718521 -> 4.456982 m** |
| `site2_handheld_5` | 4/4 -> 4/4 | 14.316365 -> 14.242283 m | 20.945751 -> 20.502501 m |

The `exp09` change is not caused by the bootstrap: that run did not trigger a
reset. It confirms significant run-to-run variation in the existing threaded
PGO/frontend path.

## Recovery Evidence

| Case | Resets | Rejected scans | Longest zero-inlier run | Poses |
|---|---:|---:|---:|---:|
| `exp03` | 0 | 0 | 0 | 3084 |
| `exp07` | 0 | 0 | 0 | 1316 |
| `exp09` | 0 | 9 | 0 | 4451 |
| `exp10` | 2 | 19 | 3 | 3681 |
| `site2_handheld_5` | 0 | 0 | 0 | 2050 |

The isolated `exp10` screen also reached 14/14 coverage and 3688 poses without
needing a reset, with RMSE 2.208114 m and P95 4.601562 m.

`exp10` therefore has two complete runs under this gate. The resetting run used
the bootstrap and improved every aggregate error statistic relative to Gate
3A, while removing the remaining four uncovered control points.

## Limitations

The recovered `exp10` path is 235.33 m long versus 92.84 m of sparse GT. It
covers the control-point sequence but contains a large post-reset offset and
excessive path length. PGO does not yet reconnect reset segments to their
pre-reset global frame.

This gate improves completion and continuity, not absolute global accuracy.
The next step should preserve reset-segment metadata and attempt global-map or
loop reacquisition before inserting cross-segment PGO constraints.

## Artifacts

- [Full evaluation](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-bounded-bootstrap_20261107T181800Z/evaluation/summary.md)
- [Full registration metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-bounded-bootstrap_20261107T181800Z/evaluation/log_metrics.json)
- [Full trajectory plots](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-bounded-bootstrap_20261107T181800Z/evaluation/plots)
- [Isolated exp10 screen](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-3b-screen-exp10_20261107T180345Z/evaluation/summary.md)
