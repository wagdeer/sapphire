# Gate 1A: Blockwise Localizability Projection

## Status

Rejected and removed. The candidate did not demonstrate a reproducible
robustness gain, and `exp09_cupola` lost half of its control-point coverage.

- Branch: `feat/robust-frontend`
- Source commit before the experiment: `5b0faca`
- Candidate worktree: dirty, including the pre-existing `gal3_right` changes
- Run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  one sequential worker

The candidate changes were removed after this report was written. The next
commit should contain only this report and the corresponding status update in
the robustness plan.

## Hypothesis

The Euclidean ESKF computed one six-dimensional eigenbasis for the Hessian and
used one condition threshold for both rotation and translation. Rotation and
translation have different physical units and observed scales, so a strong
rotation mode could incorrectly make translation look degenerate, or hide a
weak rotation mode.

The tested change decomposed the observable projection into separate rotation
and translation 3x3 blocks. It did not change measurement covariance `R`,
registration acceptance, map insertion, PGO, or estimator gains.

## Validation

- ROS 1 build: passed without compiler warnings.
- ROS 1 test suite: `18/18` passed.
- ROS 2 wrapper build: passed. Only the existing serial-LTRANS LTO warnings
  were emitted.
- Full HILTI replay: `5/5` bags reached EOF and produced non-empty
  trajectories.

## Gate 0 vs Gate 1A

Metric: HILTI sparse-control-point translation RMSE, IMU-to-pole-tip
transform, SE(3) Umeyama alignment, no scale correction, 2 s association.

| Case | Coverage G0 -> G1A | RMSE G0 -> G1A | P95 G0 -> G1A | Max G0 -> G1A |
|---|---:|---:|---:|---:|
| HILTI22 `exp03` | 17/17 -> 17/17 | 0.130519 -> 0.124955 m | 0.216704 -> 0.205990 m | 0.223705 -> 0.211882 m |
| HILTI22 `exp07` | 6/6 -> 6/6 | 0.133626 -> 0.130713 m | 0.204103 -> 0.200521 m | 0.227146 -> 0.223479 m |
| HILTI22 `exp09` | **16/16 -> 8/16** | 1.349571 -> 1.150937 m | 2.704542 -> 1.623999 m | 3.441977 -> 1.633878 m |
| HILTI22 `exp10` | 3/14 -> 3/14 | 0.053004 -> 0.055886 m | 0.064724 -> 0.072183 m | 0.065748 -> 0.073570 m |
| HILTI23 `site2_handheld_5` | 4/4 -> 4/4 | 14.244510 -> 14.235599 m | 20.504442 -> 20.487741 m | 21.892457 -> 21.871332 m |

`exp09` Gate 1A metrics are calculated only on the surviving prefix and are
not comparable with the full Gate 0 trajectory.

## Registration and Filter Health

| Case | Accepted / samples | Zero inliers | Longest rejection run | Rotation condition p95 | Translation condition p95 |
|---|---:|---:|---:|---:|---:|
| `exp03` | 3083/3083 | 0 | 0 | 83.9 | 25.4 |
| `exp07` | 1313/1315 | 0 | 1 | 36015 | 198 |
| `exp09` | 2104/4459 | 2338 | 2348 | 1179 | 41.0 |
| `exp10` | 881/3697 | 2805 | 2812 | 1339 | 59.4 |
| `site2_handheld_5` | 2049/2049 | 0 | 0 | 394 | 14.7 |

For `exp09`, registration was healthy through frame 2110. It then entered a
cascade of rejected corrections and never recovered:

1. near the transition, `nu_pos` had already reached 0.244 m while
   `P(position)` had grown to 0.106;
2. GICP stopped converging, then exceeded the correction limits;
3. registration returned zero inliers for the final 2338 frames;
4. map insertion stopped, so the trajectory ended at 8/16 control points.

The same failure was not present in Gate 0. It is a hard coverage regression
for this candidate run, even though the mechanism was mostly inactive before
the failure.

## Why the Candidate Was Rejected

The experiment did not validate the localizability hypothesis:

- In `exp07`, the principal corridor case, all six sampled pose gains remained
  exactly `1.00`; no weak-axis suppression occurred.
- Across `exp03`, `exp07`, `exp09`, `exp10`, and `site2_handheld_5`, the
  median minimum pose gain was `1.0`.
- The small `exp03`/`exp07` ATE changes are therefore attributable to run
  variation, not to a triggered projection change.
- `site2_handheld_5` remained unchanged at roughly 14.24 m RMSE and still
  accepted every high-inlier registration.
- `exp10` did not improve and retained a 2805-frame zero-inlier run.
- `exp09` failed the primary completion/coverage requirement.

The raw per-block Hessian conditions and the transformed innovation-coordinate
conditions used by the projector are not the same quantity. In particular,
the large `exp07` raw rotation condition did not cross the configured
`hessian_max_condition=10000` in the transformed coordinate system often enough
to affect an update. Retuning that threshold after observing the test results
would be post-hoc fitting, so it was not done.

## Decision

Reject Gate 1A and restore the Gate 0 estimator behavior. Do not add another
gain or lower the condition threshold to force this mechanism to activate.

The next experiment should operate on an input that has already failed,
instead of another projection of a registration that looks satisfactory:

- classify zero-inlier and non-converged runs explicitly;
- stop map insertion while tracking is lost;
- grow covariance during the rejection streak;
- implement bounded reacquisition before attempting another global frontend
  change.

The unresolved high-inlier/high-error case (`site2_handheld_5`) still requires
a correctness or degeneracy signal that is independent of the inlier count.

## Artifacts

- [Evaluation table](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z/evaluation/summary.md)
- [Evaluation data](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z/evaluation/summary.json)
- [Registration metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z/evaluation/log_metrics.json)
- [Run manifest](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z/run_manifest.json)
- [Trajectory plots](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-1-blockwise-observability_20261107T140524Z/evaluation/plots)
