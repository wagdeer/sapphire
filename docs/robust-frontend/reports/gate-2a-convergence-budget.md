# Gate 2A: Extended GICP Convergence Budget

## Status

Rejected and reverted. Increasing the HILTI GICP iteration cap from 32 to 64
allowed `exp10_cupola_2` to accumulate one additional control point, but the
alignment error increased by more than 4x. It also materially degraded
`exp09_cupola`.

- Branch: `feat/robust-frontend`
- Source commit: `aa1e7a1`
- Run root:
  `/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-2a-convergence-budget_20261107T160708Z`
- Protocol: ROS 1 Noetic, RViz off, HILTI 2022 at 0.5x, HILTI 2023 at 1.0x,
  one sequential worker

## Hypothesis

Several hard-frame rejections occurred after exactly 31 iterations, which is
the last numbered iteration under a cap of 32. Normal well-conditioned frames
converged in 1-7 iterations. Raising only the cap to 64 therefore had little
expected cost for healthy frames and could give difficult frames enough time to
reach the configured transformation and rotation epsilons.

No source code, acceptance rule, covariance, map update, or PGO behavior was
changed. Only `registration.gicp.max_iterations` changed for the HILTI 2022
and HILTI 2023 handheld profiles.

## ATE Results

| Case | Status | Coverage G0 -> G2A | RMSE G0 -> G2A | P95 G0 -> G2A |
|---|---|---:|---:|---:|
| HILTI22 `exp03` | ok | 17/17 -> 17/17 | 0.130519 -> 0.121075 m | 0.216704 -> 0.205930 m |
| HILTI22 `exp07` | ok | 6/6 -> 6/6 | 0.133626 -> 0.122500 m | 0.204103 -> 0.188045 m |
| HILTI22 `exp09` | ok | 16/16 -> 16/16 | **1.349571 -> 1.608208 m** | **2.704542 -> 3.436552 m** |
| HILTI22 `exp10` | incomplete | **3/14 -> 4/14** | **0.053004 -> 0.245063 m** | **0.064724 -> 0.371700 m** |
| HILTI23 `site2_handheld_5` | ok | 4/4 -> 4/4 | 14.244510 -> 14.238380 m | 20.504442 -> 20.497826 m |

The apparent `exp03`/`exp07` improvements are within normal whole-run
variation. They do not compensate for the two hard-case regressions.

## Convergence Usage

| Case | Registrations | Median iterations | Frames >= 32 | Frames at 63 |
|---|---:|---:|---:|---:|
| `exp03` | 3083 | 1 | 0 | 0 |
| `exp07` | 1315 | 1 | 4 | 4 |
| `exp09` | 4459 | 2 | 14 | 14 |
| `exp10` | 3697 | 0 | 9 | 8 |
| `site2_handheld_5` | 2049 | 1 | 0 | 0 |

The extra budget was exercised only on difficult frames, as intended. The
result does not support the hypothesis that non-convergence was primarily a
budget problem.

## Failure Analysis

`exp10` reached 962 map-frame samples instead of 869 and matched 4/14 control
points instead of 3/14. However, its RMSE rose from 0.053 to 0.245 m and its
P95 rose from 0.065 to 0.372 m.

The sequence was therefore extending a wrong registration basin for longer,
not recovering the correct trajectory. The same mechanism caused a 19%
RMSE regression on the fully covered `exp09`.

Increasing the iteration cap is unsafe without a correctness criterion:
additional iterations can improve the optimizer objective while remaining on
the wrong geometric solution.

## Decision

Reject Gate 2A and restore `max_iterations=32` in both HILTI handheld profiles.

The next attempt should not search longer inside the same residual minimum.
It must change either:

- the association or residual model used by registration; or
- the target/map used for registration after tracking loss.

## Artifacts

- [Evaluation table](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-2a-convergence-budget_20261107T160708Z/evaluation/summary.md)
- [Registration metrics](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-2a-convergence-budget_20261107T160708Z/evaluation/log_metrics.json)
- [Run manifest](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-2a-convergence-budget_20261107T160708Z/run_manifest.json)
- [Trajectory plots](/home/bindeer/git/sapphire_ws/results/robust_frontend/2026/11/07/gate-2a-convergence-budget_20261107T160708Z/evaluation/plots)
