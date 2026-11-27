# Full ATE evaluation — 2026-11-26

UTC start: `2026-11-26T15:43:11Z`

UTC completion: `2026-11-26T21:01:33Z`

This directory preserves the consolidated output of the full Sapphire replay
and ATE evaluation. Datasets were mounted read-only and were not copied into
the workspace.

## Results

- 50 evaluated records
- 39 `ok`, 10 `warning`, 1 `invalid`
- 49 records have an ATE RMSE

| Dataset | Runs with RMSE | RMSE mean (m) | RMSE median (m) |
|---|---:|---:|---:|
| HILTI 2022 | 12 | 0.805217 | 0.129892 |
| HILTI 2023 | 8 | 2.074279 | 0.111665 |
| Newer College | 5 | 0.307356 | 0.408919 |
| S3E | 24 | 5.318318 | 0.994213 |

The invalid record is `hilti_2023/site2_handheld_6`. Its prediction and ground
truth associate at two poses, so SE(3) Umeyama alignment is degenerate and ATE
cannot be computed. The warning records have ground-truth gaps that were not
interpolated.

See [summary.md](summary.md) for the per-sequence table,
[summary.json](summary.json) for all metrics and warnings, and
[summary.csv](summary.csv) for the compact tabular form.

## Anomalies

The largest reported errors are:

| Dataset / sequence / agent | ATE RMSE (m) |
|---|---:|
| S3E / `S3E_Tunnel_1` / bob | 61.892560 |
| S3E / `S3E_Tunnel_1` / alpha | 37.856942 |
| HILTI 2023 / `site2_handheld_5` | 14.633085 |
| S3E / `S3E_Tunnel_1` / carol | 3.552856 |
| HILTI 2022 / `exp09_cupola` | 3.010162 |
| HILTI 2022 / `exp10_cupola_2` | 2.963495 |

These are evaluation findings, not automatically algorithm regressions.
Sparse ground truth, ground-truth gaps, and path divergence need to be checked
per sequence.

## Verification

- ROS 1 core/interface CTest: 17/17 passed.
- ROS 1 conversion test: 9/9 passed.
- ROS 2 conversion test: 9/9 passed.

The ROS 1 CTest wrapper entry is excluded because its environment cannot
import the catkin Python package. The conversion test binary itself passes.

## Provenance

- Core commit: `4ca7e3307a0a161d79722ab8e8cadb7a76a51d95`
- Core working-tree diff SHA-256: `038c03e3f3f613ac4328c7dcb064de953f38e5eddd99c8ec811e4d168b667c98`
- ROS 1 commit: `4880ed379e8ec5de11a1821ffe4009cc81c34d2c`
- ROS 2 commit: `d6373826c90709a0f37d9bd373bf4a976e3949bd`
- ROS 1 image: `sapphire:noetic-foxglove` (`sha256:a8182a2908cf803b68c0ebd5f27599dcc94ef68fc87cc8428809f2333661c513`)
- ROS 2 image: `sapphire:humble` (`sha256:b7c634736e1efa3eeaaf0e2c9ede653411018b8f273822f931b6f16a774ce1c6`)

Run configuration, status tables, replay logs, and evaluation logs are
included beside this README. The 156 MB of per-sequence trajectories remain in
the local run directory and are intentionally not included in this repository.
