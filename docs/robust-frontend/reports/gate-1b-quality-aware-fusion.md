# Gate 1B: Quality-aware Map Veto

## Status

Rejected and removed. Per-scan overlap ratio and solver residual were not
stationary enough to decide whether a scan should extend the local map.
`exp03`, `exp09`, and `exp10` all stopped mapping while the robot continued
moving, then dead-reckoned out of the map.

- Branch: `feat/robust-frontend`
- Source commit: `ec19af1`
- Run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1b-quality-aware-fusion_20261107T150837Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  one sequential worker

## Hypothesis

Converged GICP corrections can still have a large residual, and
non-converged corrections can be nearly correct. The candidate therefore
classified each registration as:

- healthy: converged, adequately conditioned, and low residual;
- degraded: bounded motion and enough absolute inliers, but not healthy;
- invalid: malformed, too few inliers, or beyond the motion limits.

Degraded corrections were intended to update the filter with a nine-times
larger measurement covariance and not enter the submap or PGO.

## Validation

- ROS 1 build: passed.
- ROS 1 tests: `18/18` passed.
- Full HILTI replay: `5/5` bags reached EOF.
- The evaluator was made robust to a degenerate two-point estimate so all
  remaining cases are still reported.
- The log analyzer was extended to read the optional quality counters while
  remaining compatible with the Gate 0/1A logs.

## ATE Results

| Case | Status | Coverage | Trajectory poses | RMSE |
|---|---|---:|---:|---:|
| HILTI22 `exp03` | incomplete | **6/17** | 879 | 0.157982 m |
| HILTI22 `exp07` | ok | 6/6 | 1316 | 0.133474 m |
| HILTI22 `exp09` | alignment failed | **2/16** | 400 | not estimable |
| HILTI22 `exp10` | insufficient matches | **1/14** | 340 | not estimable |
| HILTI23 `site2_handheld_5` | ok | 4/4 | 2050 | 14.239367 m |

Only the untouched `exp07` result remained comparable with the control. The
large `site2_handheld_5` error was also unchanged, confirming that this gate
does not address consistent high-inlier misregistration.

## Registration Health

| Case | Samples | Healthy | Degraded | Map-allowed | Rejected | Zero inliers | Longest zero run |
|---|---:|---:|---:|---:|---:|---:|---:|
| `exp03` | 3083 | 861 | 17 | 861 | 2205 | 2150 | 2147 |
| `exp07` | 1315 | 1315 | 0 | 1315 | 0 | 0 | 0 |
| `exp09` | 4459 | 344 | 55 | 344 | 4060 | 4009 | 4009 |
| `exp10` | 3697 | 336 | 3 | 336 | 3358 | 3264 | 3069 |
| `site2_handheld_5` | 2049 | 2049 | 0 | 2049 | 0 | 0 | 0 |

## Failure Analysis

The source cloud voxel count is not stationary. In stair/cupola transitions it
changed from roughly 4k to 10k points while the absolute number of inliers
remained high. The source-normalized inlier ratio consequently fell below the
configured degraded threshold even though thousands of correspondences were
still valid.

The local map then stopped at:

- `exp03`: frame 861;
- `exp09`: frame 344;
- `exp10`: frame 336.

These are precisely the transitions where map growth was needed. Once the
map target became stale, the moving scan lost overlap entirely and the
remaining registrations returned zero inliers. The map veto therefore created
the long zero-inlier failure it was intended to prevent.

Solver error divided by inlier count was also geometry-dependent. It rose
sharply while entering a larger space even when the correction displacement
was only a few centimeters. Neither ratio nor residual can be used as a
standalone map-insertion decision.

One implementation issue independently prevented the intended soft fusion:
with `inject_directional_pose=true`, the directional pose gain was installed
directly and was not attenuated by the measurement covariance multiplier.
Thus the covariance scale affected NIS but not the injected pose. Any future
soft-fusion gate must scale the effective directional gain and apply the same
gain in the Joseph covariance update.

## Decision

Reject Gate 1B and restore the hard registration gate and unconditional map
insertion for accepted frames.

Do not derive map eligibility from scan-wide overlap ratio or raw solver
residual without a scene-independent normalization. The next attempt should:

- keep accepted observations feeding the local map;
- treat loss of correspondences as a tracking-state event;
- use bounded reacquisition with a wider search only after repeated rejects;
- return to normal strict registration immediately after reacquisition.

## Artifacts

- [Evaluation table](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1b-quality-aware-fusion_20261107T150837Z/evaluation/summary.md)
- [Registration metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1b-quality-aware-fusion_20261107T150837Z/evaluation/log_metrics.json)
- [Run manifest](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1b-quality-aware-fusion_20261107T150837Z/run_manifest.json)
- [Trajectory plots](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1b-quality-aware-fusion_20261107T150837Z/evaluation/plots)
