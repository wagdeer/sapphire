# RICP observation component

RICP owns the verified shared-map GICP observation and full-state fusion. It is
called optionally by the ESKF tracking path after point-plane rejection:

```toml
[odometry]
frontend = "eskf"

[odometry.gicp_fallback]
enabled = true
```

The switch defaults to false. Source resolution, correspondence distance, geometry
variance floor and iteration budget remain under `odometry.gicp_fallback`; defaults
are 0.25m, 0.5m, 0.0005m² and20 iterations. The shipped
`config/hilti2022_exp09_ricp.toml` now selects ESKF with this component enabled.
The retired standalone `frontend="ricp"` and `[odometry.ricp]` settings report
migration errors. Old window-based VGICP/direct-pose/velocity-gain code is archived
under `audit/2026-10-08-ricp-component/before`, outside production source.

## Ownership and update

Pipeline owns the configured Ricp instance on the odometry thread. Point-plane
ESKF always runs first. Only `no_matches` or `normal_support` rejection triggers
Ricp; invalid input/numerical rejection does not. The propagated state remains
unchanged between attempts. Ricp reads the existing VoxelMap, including nonplane
leaf moments; all source statistics and lazy target lookup caches are call-local.
There is no second persistent map, window cache or reset state.

Input points are in scan-end IMU coordinates, and the state pose is T_world_imu.
Unmodified small_gicp supplies GICP factors, Jacobians, LM solve and convergence.
The IMU prediction seeds LiDAR-only registration. Library factors are relinearized
at the returned pose, and GTSAM chart Jacobians support one full 15-state ESKF
fusion. Velocity/bias corrections follow the prior cross-covariance. Geometry
information is approximate and is not calibrated independent pose uncertainty.
Finite/support/correction/objective checks remain local acceptance criteria.

Failure leaves state/covariance unchanged. Pipeline then preserves IMU continuity
and skips map/window insertion. Acceptance commits once through the existing
pipeline. Gal3 propagation, deskew, initialization, local BA and reset policy are
unchanged. This component adds no reverse solve, ZUPT or RKO IMU model.

## Verification

`ricp_test` retains the verified library/fusion, asymmetric pose, frame-equivariance,
velocity/bias, covariance and late-rollback checks. `odometry_rejection_test` covers
both switch states, map/window rejection and IMU continuity. Full Hilti migration
replays are compared with the preceding library implementation for exact trajectory
equivalence. See [current engineering state](../../../../docs/CURRENT_STATE.md)
and repository-root `audit/2026-10-08-ricp-component` for results.
