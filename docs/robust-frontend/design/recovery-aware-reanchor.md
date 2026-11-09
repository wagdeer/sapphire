# Recovery-aware Global Re-anchor

## Status

Implementation contract. Follow this document as written. The decisions below
replace the earlier sketch where they differ.

- Base branch: `feat/robust-frontend`
- Base commit: `82a1165` (`feat(frontend): bootstrap scans after local map reset`)
- Work branch: `feat/recovery-aware-reanchor`, cut from that commit

The current worktree is `exp/hilti-crop-box-0p5` and also contains unrelated
`gal3_right` edits. Implement this gate on a clean branch from `82a1165`.
Leave the crop-box branch and the `gal3_right` files untouched. Copy this
document onto the work branch; it already lives only as an untracked file on
the dirty worktree.

Gate 3B is the behavior to preserve when re-anchor is disabled. Its numbers
are in
[the Gate 3B report](../reports/gate-3b-bounded-bootstrap.md).

## Problem

After five consecutive registration rejections the frontend calls
`SubmapManager::resetKeyframes()` and then inserts five IMU/deskew bootstrap
scans. That path is `OdometryPipeline::processLidarScan`. Gate 3B reached
full coverage on all five HILTI stress cases, including 14/14 on
`exp10_cupola_2`.

The recovered trajectory is only locally continuous:

- odom-frame poses keep integrating through the reset;
- `buildOdometryGraph()` then ties the keyframe before the reset to the
  keyframe after it with `odom_noise_` (rotation variance `1e-6`, translation
  variance `1e-4`);
- the post-reset segment is therefore glued to the drifted odom pose;
- the resetting `exp10` run has RMSE 2.458 m, P95 4.457 m, max 5.213 m, and
  estimated length 235.33 m against 92.84 m of sparse GT.

One map-frame prior can rigidly place a recovered epoch. It cannot reshape
the extra path inside that epoch. This gate does not try to.

## Goals

1. Give every local-map recovery a monotonically increasing epoch.
2. When re-anchor is enabled, insert the reset scan as a PGO keyframe and
   connect it with a weak recovery edge.
3. After bootstrap, and only after a locally accepted registration, ask the
   PGO worker to register one new-epoch keyframe against an anchor map.
4. On success, add one `PriorFactor<Pose3>` in the map frame.
5. Leave the live ESKF / observer state in the odom frame.
6. Keep an unanchored epoch in the graph through the weak edge.
7. Publish per-epoch and re-anchor diagnostics.

## Non-goals

- ESKF gains, Hessian thresholds, and the frontend registration acceptance
  gate stay as they are.
- `kRecoveryBuildFrames` (5) and `kMaxRecoveryAttempts` (3) stay as they are.
- `odometry.submap.recovery_after_rejections` stays 5 in the HILTI configs.
- The frontend pose is not rewritten when a global match is found.
  `imu_state_` stays on the continuous odom trajectory.
- Loop-closure thresholds, loop search, keyframe distance, and keyframe
  rotation stay as they are.
- Occupancy insertion is unchanged.
- S3E is not part of this gate.
- The removed Gal(3) experiment profile is not restored by this gate.
- This gate does not add a second anchor to the same epoch, and it does not
  deform a recovered segment to chase path length.

## Locked decisions

These are requirements. They come from the current backend, not from a
parameter search.

1. Re-anchor registration runs on the PGO worker, using the same GICP solver
   family as `registerLoop` (`small_gicp::Registration<GICPFactor,
   SerialReduction>`). The frontend `Registration` object stays on the lidar
   thread and keeps the HILTI VGICP settings (`max_correction_trans = 1`,
   `max_correction_rot_deg = 20`). Its Hessian is not converted into a PGO
   factor.
2. The source cloud is the query keyframe's lidar-body cloud,
   `frames_[i].cloud_lidar`. The solver initial guess is `T_map_lidar`.
   The source is not pre-transformed by `T_map_odom`. Doing both applies the
   pose twice.
3. The query cloud is that one keyframe only, range-gated by the existing
   `source_max_range_m` and voxelized by the existing `source_voxel_size`.
   `buildQueryCloud()` merges a travel window and can pull pre-reset points
   into the source, so the anchor path does not call it.
4. The anchor target is built from keyframe body clouds placed at
   `optimizedPose(i)`, then voxelized at `pgo.reanchor.target_voxel_size`.
   It is not `latestGlobalMap()`. The visualization map uses
   `map_frame_stride` and `map_voxel_size` and may already contain the new
   epoch.
5. The anchor target contains only keyframes whose epoch is already anchored.
   Epoch 0 is anchored by the existing root prior. An unanchored intermediate
   epoch is left out, even though its epoch id is smaller. Matching against
   that drifted copy can confirm the wrong pose.
6. A local registration acceptance means the new submap is consistent. It
   does not mean `T_map_odom * T_odom_lidar` is the right global pose.
   Agreement is between two global registration seeds. An agreement test
   against the drifted prior within 1 m would reject the multi-metre fix
   this gate exists to make.
7. The map prior is a `PriorFactor<Pose3>` on the query keyframe. It is not
   a `BetweenFactor`, and it is not passed through `tryCommitLoops` or
   `max_constraint_error_sigma`. That gate measures disagreement with
   existing edges. The weak recovery edge is expected to disagree.
8. The prior noise goes through the existing `loopInformationNoise()` helper.
   Covariance floors stay the loop floors already added inside
   `registerLoop` (`lidar_rotation_sigma_rad`, `lidar_translation_sigma_m`).
   There is no second, tighter floor.
9. `pgo.reanchor.enabled = false` keeps Gate 3B factors and keyframe
   selection. The reset scan is force-inserted only when the flag is true.
   Epoch ids may still be stored while the flag is false.
10. `max_correction_trans_m` is only the accept/reject cap on the solver
    output. Association range is `max_correspondence_m`. With the loop value
    of 2 m, a multi-metre offset cannot converge. This gate uses 5 m because
    the Gate 3B `exp10` maximum ATE is 5.213 m. That value belongs only to
    re-anchor. Loop closure keeps `lidar_max_correspondence_m = 2`.

## Current code the change has to fit

`OdometryPipeline` owns recovery. The constants are file-local in
`src/odometry/pipeline.cpp`:

- reset when `registration_reject_streak_ >= recovery_after_rejections` and
  `recovery_count_ < 3`;
- the reset branch calls `resetKeyframes`, sets
  `recovery_build_frames_remaining_ = 5`, and `addFrame`s the IMU prior;
- while that counter is positive, a rejected scan is inserted at the deskew
  prior and returns before ESKF;
- an accepted scan during the same window force-adds a local keyframe.

`PoseGraphBackend::addFrame` drops a frame whose motion is inside
`keyframe_distance` / `keyframe_rotation`. A stationary reset can therefore
miss the graph entirely. The worker copies `input_frames_` into `frames_`,
converts the cloud with `T_odom_lidar.inverse()`, and appends a
`BetweenFactor` per new keyframe inside `buildOdometryGraph()`.

`T_map_odom()` is `optimized_pose_last * T_odom_last.inverse()`.
`latestResult()` multiplies it onto the odom pose in `applyGlobalCorrection`.
After a prior is committed, the published map pose moves. The ESKF state
does not. That map-frame motion is the re-anchor result.

`MapTrajectory` already replays each sample with its owning keyframe:
`T_map_i * T_odom_i.inverse() * T_odom`. Samples whose keyframe is not
optimized yet fall back to `T_map_odom()`. Keep that fallback. Dropping those
samples truncates the live tail.

`recordOdometrySample` ignores `converged == false`. Bootstrap and reset
currently pass `converged = true` into that helper. Leave that behavior.

PGO `update_period_sec` is 1.0 in the HILTI configs. The anchor attempt has
to tolerate the map and the new keyframe arriving on a later worker cycle.

## Terminology

- **Odom frame**: continuous frontend frame. Local-map recovery does not
  reset it.
- **Map frame**: PGO frame. The root keyframe is fixed by the existing
  tight prior.
- **Recovery epoch**: `uint64_t`. Epoch 0 starts with the first scan. It
  increases by one in the pipeline reset branch, immediately before
  `resetKeyframes()`.
- **Anchored epoch**: epoch 0, or any later epoch that owns one accepted
  map prior.
- **Anchor map**: sparse cloud of anchored-epoch keyframes, expressed in the
  map frame, excluding the query epoch.
- **Re-anchor prior**: one `PriorFactor<Pose3>` whose measurement is
  `T_map_lidar` of the query keyframe.

## Invariants

1. Odom-frame ESKF / observer state stays continuous across epochs.
2. The epoch increases only in the reset branch of `processLidarScan`, and
   only when `resetKeyframes()` runs.
3. `SubmapManager` does not store epochs. Direct `resetKeyframes()` tests
   keep their current meaning.
4. With `pgo.reanchor.enabled = false`, factor types and keyframe admission
   match Gate 3B. Epoch metadata may be stored.
5. With the flag enabled and no reset, the factor graph has the same
   between-edges and the same keyframe set as before. The only addition is
   epoch metadata.
6. A between-edge uses `odom_noise_` when both keyframes share an epoch.
7. The one between-edge that steps from epoch `E` to epoch `E + 1` uses the
   recovery noise model when the flag is enabled.
8. Each epoch accepts at most one map prior.
9. The anchor cloud for epoch `N` contains no point from a keyframe with
   epoch `N` or from an unanchored epoch.
10. A rejected attempt does not erase frames, reset the local map, change
    `T_map_odom`, or stop the run.
11. Anchor failure does not drop trajectory samples.
12. The worker reads epoch and anchor commands from the queued inputs. It
    does not read a pipeline global.

## Pipeline state

```text
HEALTHY(epoch E)
    |
    | 5 consecutive rejects, recovery_count < 3
    v
LOCAL_RESET
    - epoch = E + 1
    - resetKeyframes()
    - if reanchor enabled: addFrame(..., epoch, force_keyframe=true)
    - else: existing addFrame(...)
    - recovery_build_frames_remaining = 5
    - reanchor_ready = false
    |
    v
BOOTSTRAP
    - existing IMU/deskew insertion
    - addFrame uses the new epoch and force_keyframe=false
    - no requestReanchor
    |
    | counter reaches 0
    v
READY
    - reanchor_ready = enabled
    - normal local registration
    |
    | locally accepted scan and reanchor_ready
    v
requestReanchor(epoch)
    |
    +-- worker accepts prior --> reanchor_ready = false
    |
    +-- worker gives up --------> reanchor_ready = false
```

`requestReanchor` is issued only from the accepted-registration path, after
bootstrap has finished for this epoch. Rejected scans, bootstrap insertions,
and epoch 0 do not request it.

The pipeline may call `requestReanchor` again on later accepted scans. The
worker coalesces a queued attempt, ignores an anchored epoch, and stops
after the attempt budget. The pipeline clears `reanchor_ready_` when
`reanchorState()` reports `anchored` or `gave_up` for this epoch.

Every write to `latest_result_` sets `recovery_epoch` to the pipeline epoch.
`applyGlobalCorrection` copies the result through, so the field survives.

## Data model

In `types.hpp`:

```cpp
using RecoveryEpoch = std::uint64_t;
```

`OdometryResult` gains:

```cpp
RecoveryEpoch recovery_epoch = 0;
```

It does not gain a `map_anchored` flag. The frontend cannot know that until
the worker commits.

PGO `InputFrame`, `GraphFrame`, and `OdometrySample` store
`RecoveryEpoch epoch`.

`MapTrajectorySample` gains:

```cpp
RecoveryEpoch recovery_epoch = 0;
bool epoch_anchored = false;
```

`epoch_anchored` is filled at replay time from the worker's anchored-epoch
set. It is a report of graph state, not an input to optimization.

## Public PGO API

```cpp
struct MapAnchorMeasurement {
    RecoveryEpoch epoch = 0;
    size_t keyframe = 0;
    double stamp = 0.0;
    Isometry3d T_map_lidar = Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    size_t source_points = 0;
    size_t target_points = 0;
    size_t inliers = 0;
    double inlier_ratio = 0.0;
};

struct ReanchorEpochState {
    RecoveryEpoch epoch = 0;
    size_t attempts = 0;
    size_t deferred = 0;
    bool anchored = false;
    bool gave_up = false;
    const char* last_reason = "idle";
};

std::optional<size_t> addFrame(
    const PointCloudConstPtr& cloud_odom,
    const Isometry3d& T_odom_lidar,
    double stamp,
    RecoveryEpoch epoch = 0,
    bool force_keyframe = false);

void requestReanchor(RecoveryEpoch epoch);
void submitMapAnchor(const MapAnchorMeasurement& anchor);
void requestAnchorMap(RecoveryEpoch target_epoch);
PointCloudConstPtr latestAnchorMap() const;
RecoveryEpoch latestAnchorMapEpoch() const;
ReanchorEpochState reanchorState() const;
```

`addFrame` returns the stored keyframe index, or `nullopt` when the frame is
dropped or PGO is disabled. `force_keyframe` is consulted before the
distance/rotation gate. Existing callers stay valid because the new
arguments have defaults and the old `void` return can be ignored.

Production split:

- the pipeline calls `addFrame` and `requestReanchor`;
- the worker, after a successful two-seed registration, calls
  `submitMapAnchor`;
- tests may call `submitMapAnchor` directly so the factor math does not
  depend on GICP.

`submitMapAnchor` queues the measurement. The worker applies it only when:

- re-anchor is enabled;
- the keyframe exists, its epoch matches, and its stamp matches;
- that epoch has no prior yet;
- `T_map_lidar` is finite;
- `loopInformationNoise(covariance)` succeeds.

Any other measurement is ignored. `T_map_odom` and the optimized poses stay
on the previous revision. A duplicate for an anchored epoch is ignored and
does not add a second prior.

`requestAnchorMap` is for tests and diagnostics. `requestReanchor` builds
the cloud it needs internally and does not depend on a previous
`requestAnchorMap`.

## Worker cycle

Inside `processPending`, after the odometry ISAM update and before loop
search:

1. Service `requestAnchorMap` if one is pending.
2. Service at most one `requestReanchor`.
3. Apply at most one queued `submitMapAnchor`, then `updateCorrection()`.
4. Run the existing loop search against the corrected poses.

A re-anchor request that arrives before its keyframe is in `optimized_`
stays pending. That deferral is not an attempt.

Attempt accounting lives in the worker:

- a registration that actually runs consumes one attempt;
- `anchor_map_not_ready` and `query_not_ready` do not;
- calls inside `cooldown_sec` of the last counted attempt are coalesced and
  do not increment attempts or rejections;
- `gave_up` becomes true when `attempts >= max_attempts_per_epoch` and the
  epoch is still unanchored;
- after `gave_up` or `anchored`, further requests for that epoch are
  ignored.

## Anchor cloud

For target epoch `N`, include keyframe `i` when:

- `i` is already in `optimized_`;
- `frames_[i].epoch` is anchored;
- `frames_[i].epoch < N`;
- the optimized translation is within `search_radius_m` of the query prior.

Place `frames_[i].cloud_lidar` at `optimizedPose(i)`, merge, and voxelize
with `reanchor.target_voxel_size`. Store the cloud, the target epoch, and
the included keyframe count for `latestAnchorMap()`.

The prior used for the crop and for seed A is the correction of the latest
optimized keyframe whose epoch is anchored, times this query's
`T_odom_lidar`:

```text
T_map_lidar_prior = T_map_anchor * T_odom_anchor.inverse() * T_odom_query
```

That is the pre-reset map alignment applied to the continuous odom pose.
It is not "whatever `T_map_odom()` is after the weak edge."

## Registration

Add `registerAnchor` beside `registerLoop`. Leave `registerLoop`'s
thresholds, correspondence distance, and call pattern alone. Shared private
math is fine only when the existing loop tests still pass unchanged.

`registerAnchor` uses:

- solver: GICP, serial reduction, same covariance-neighbor count and
  convergence epsilons as the loop solver;
- `max_iterations`: existing `lidar_max_iterations`;
- correspondence: `reanchor.max_correspondence_m`;
- source voxel: existing `source_voxel_size`;
- target voxel: `reanchor.target_voxel_size`;
- covariance floors: existing loop sigmas, applied the same way as
  `registerLoop`.

Quality gate, all required:

- finite pose and finite Hessian;
- source points `>= min_source_points`;
- target points `>= min_anchor_map_points`;
- inliers `>= min_inliers`;
- `inliers / source_points >= min_inlier_ratio`;
- fitness finite and `<= lidar_max_fitness_m2`;
- minimum information eigenvalue `> 0` after the same normalization
  `registerLoop` uses;
- correction from the seed, measured as `seed.inverse() * T_result`,
  within `max_correction_trans_m` and `max_correction_rot_deg`;
- `loopInformationNoise` accepts the covariance.

Seed A is `T_map_lidar_prior`.

Seed B is seed A translated by `agreement_trans_m` along map-frame +X and
yawed by `agreement_rot_deg` about map-frame +Z.

Both seeds run the full gate. Both must pass. The two results must then lie
within `agreement_trans_m` and `agreement_rot_deg` of each other. The
committed measurement is seed A's pose and covariance. A seed that fails the
gate, or a pair that disagrees, is one rejection and does not call
`submitMapAnchor`.

Log at most one rejection line per cooldown interval. The reason string is
one of:

- `query_not_ready`
- `anchor_map_not_ready`
- `too_few_source_points`
- `too_few_target_points`
- `not_converged`
- `non_finite`
- `too_few_inliers`
- `inlier_ratio`
- `fitness`
- `singular_hessian`
- `correction_limit`
- `indefinite_covariance`
- `seed_disagreement`
- `duplicate_epoch`
- `disabled`

## PGO factors

Same-epoch between-factor: existing `odom_noise_`.

Cross-epoch between-factor, only when `reanchor.enabled`:

```text
rotation variance    = recovery_odometry_rotation_sigma_rad^2   on rx, ry, rz
translation variance = recovery_odometry_translation_sigma_m^2  on tx, ty, tz
```

GTSAM `Pose3` order is rotation then translation, matching the existing
`odom_variances` vector. The measurement is still the odom relative pose.
Weak noise has the same mean as the tight edge, so an epoch with no prior
stays on the odom trajectory. The weak variance matters only once a prior
disagrees with it.

Map prior:

```text
PriorFactor<Pose3>(poseKey(query), T_map_lidar, loopInformationNoise(...))
```

The root prior on keyframe 0 stays as it is.

## Map trajectory

Replay stays `sampleCorrection()`.

- optimized keyframe: `T_map_i * T_odom_i.inverse()`;
- keyframe not optimized yet: current `T_map_odom()`;
- samples are not dropped because an epoch is unanchored;
- `recovery_epoch` is copied from the owning keyframe;
- `epoch_anchored` follows the anchored set at replay time.

A later epoch's anchor moves `T_map_odom()`. It must not be applied over an
older sample whose keyframe is already optimized. The existing per-keyframe
formula already does this. Keep it.

## Configuration

Struct default is disabled. Add the fields under `Config::Pgo` and register
them in `src/config.cpp` (`kNumbers`, `kInts`, `kBools`, and
`validateConfig`). Unknown TOML keys are rejected, so every key below has to
be in the schema before either HILTI file sets it.

```toml
[pgo.reanchor]
enabled = false
search_radius_m = 10.0
target_voxel_size = 0.4
max_correspondence_m = 5.0
min_anchor_map_points = 500
min_source_points = 500
min_inliers = 256
min_inlier_ratio = 0.65
max_correction_trans_m = 10.0
max_correction_rot_deg = 30.0
max_attempts_per_epoch = 5
cooldown_sec = 1.0
agreement_trans_m = 1.0
agreement_rot_deg = 5.0
recovery_odometry_translation_sigma_m = 2.0
recovery_odometry_rotation_sigma_rad = 0.17453292519943295
```

Validation:

- distances, voxel size, correspondence, sigmas, and `min_inlier_ratio` are
  finite;
- `min_inlier_ratio` is in `(0, 1]`;
- integer counts are `>= 1`;
- `cooldown_sec >= 0`;
- `search_radius_m >= max_correction_trans_m >= agreement_trans_m`;
- `max_correspondence_m >= agreement_trans_m`.

These are the gate values. A failing HILTI run is reported with the rejection
reason. The same run does not retune them.

Enable the block only in:

- `cfg/hilti_2022_handheld.toml`
- `cfg/hilti_2023_handheld.toml`

Set `enabled = true` there. Other datasets keep the struct default.

## Logging and stats

Pipeline, warning, extending the existing reset line:

```text
[pipeline] recovery epoch=E reset_attempt=N
```

Worker, warning:

```text
[reanchor] attempt epoch=E keyframe=K map_points=P prior=(x,y,z)
[reanchor] rejected epoch=E keyframe=K reason=...
[reanchor] accepted epoch=E keyframe=K inliers=I/J ratio=R correction=...
[pgo] recovery edge A->B weak sigma_r=... sigma_t=...
[pgo] map anchor epoch=E keyframe=K info_diag=[...]
```

`PoseGraphStats` gains:

- `recovery_epochs`
- `map_anchor_attempts`
- `map_anchor_successes`
- `map_anchor_rejections`
- `unanchored_epochs`

`reanchorState()` reports the latest epoch the worker has seen.
`unanchored_epochs` counts epochs above 0 that are not anchored.
Epoch 0 is not an unanchored epoch.

## Files

Edit:

- `include/sapphire/types.hpp`
- `include/sapphire/backend/pose_graph.hpp`
- `src/backend/pose_graph.cpp`
- `src/config.cpp`
- `include/sapphire/odometry/pipeline.hpp`
- `src/odometry/pipeline.cpp`
- `tests/config_test.cpp`
- `tests/pose_graph_test.cpp`
- `cfg/hilti_2022_handheld.toml`
- `cfg/hilti_2023_handheld.toml`

Leave alone:

- `SubmapManager` behavior
- `src/odometry/registration.cpp`
- ESKF sources
- occupancy sources
- loop thresholds

A pipeline test is required only for the epoch increment and for the absence
of a re-anchor request before bootstrap ends. The factor tests belong in
`pose_graph_test.cpp`.

## Tests

Follow the existing `expect()` style in `tests/pose_graph_test.cpp`. Wait on
the worker the same way the current tests wait, with
`update_period_sec` short enough for the test. Reuse `makeCornerCloud()` and
`transformCloud()`.

Thresholds in tests may be lowered so the corner cloud clears the point
counts. Production defaults stay as specified above.

### Factor tests, no GICP

1. **Epoch boundary noise.** Three keyframes at x = 0, 1, 2. Epochs
   `0, 0, 1`. Enabled. No prior. Optimized poses stay on the odom positions
   (a weak edge has the same mean). Then `submitMapAnchor` on the third
   keyframe at x = 0 with diagonal covariance `0.05^2` on every tangent.
   The third pose moves to about x = 0 (assert `x < 0.3`). The second pose
   stays near x = 1 (assert `|x - 1| < 0.05`), because its incoming edge is
   still `odom_noise_`.
2. **Tight edge rejects the same prior.** Same geometry, but
   `enabled = false`, so the cross-epoch edge stays at translation sigma
   0.01 m. The same prior must leave the third pose near x = 2
   (assert `x > 1.5`). This is the regression that proves the recovery
   variance is actually installed.
3. **Duplicate prior.** A second `submitMapAnchor` on that epoch does not
   move the pose further and does not increment `map_anchor_successes`
   past 1.
4. **Invalid prior.** A zero covariance, or a non-finite pose, leaves
   `T_map_odom` and the trajectory on the previous revision.
5. **Disabled flag.** `enabled = false` with two epochs produces the same
   keyframe count and the same optimized poses as an all-epoch-0 graph.
   `force_keyframe` is what the pipeline uses; the backend test calls
   `addFrame` without force and checks the distance gate still drops a
   stationary frame.

### Anchor-cloud tests

6. **Epoch filter.** Epoch 0 is a corner at the origin. Epoch 1 is the same
   corner plus one point that lands near x = 50 after the odom pose is
   applied. `requestAnchorMap(1)` returns a cloud with the corner and with
   no point near x = 50.
7. **Unanchored gap.** Epoch 0 anchored, epoch 1 present but not anchored,
   `requestAnchorMap(2)` excludes the epoch-1 unique point as well.

### Registration tests

8. **Two seeds agree.** Epoch 0 corner at x = 0. Epoch 1 body cloud is the
   same corner, odom pose x = 0.4. Lower the test point-count gates to the
   corner cloud. `requestReanchor(1)` commits a prior and the optimized
   epoch-1 translation ends closer to 0 than to 0.4.
9. **Out of range.** The same cloud with the epoch-1 prior 20 m away, and
   the test `max_correction_trans_m` left at 10, is rejected. Pose stays on
   odometry. `map_anchor_rejections` increases. `map_anchor_successes`
   stays 0.
10. **Not ready is free.** `requestReanchor` before the epoch has an
    optimized keyframe increments `deferred` and leaves `attempts` at 0.

### Trajectory

11. Samples exist for both epochs after a rejected anchor. An accepted
    anchor changes the new epoch's samples and leaves an already-optimized
    older sample on its own keyframe correction.

### Pipeline

12. A unit-level check that a reset increments the epoch once and that
    `requestReanchor` is not implied by the reset frame itself. If the
    existing pipeline test harness cannot see worker requests without a long
    spin, cover the request timing with a focused pipeline test that enables
    PGO, feeds the rejection streak, and asserts `reanchorState().attempts`
    is still 0 until a post-bootstrap scan is accepted. Do not weaken
    `testOdometryPipeline` in `pipeline_test.cpp`.

`tests/submap_test.cpp` recovery assertions stay valid without modification.

## HILTI gate

Fixed cases, no S3E:

- HILTI22 `exp03_construction_stairs`
- HILTI22 `exp07_long_corridor`
- HILTI22 `exp09_cupola`
- HILTI22 `exp10_cupola_2`
- HILTI23 `site2_handheld_5`

Protocol: HILTI22 at 0.5x, HILTI23 at 1.0x, sequential playback, RViz off
for metric runs.

Screen `exp09` and `exp10` first, then the other three. `exp09` had zero
resets in Gate 3B. It is a non-regression control, not a recovery case.
`exp10` reset on the full Gate 3B run (2 resets) and did not reset on the
isolated screen. A run that never resets does not fail this gate.

Gate 3B reference:

| Case | Coverage | RMSE | P95 |
|---|---:|---:|---:|
| `exp03` | 17/17 | 0.141931 m | 0.228729 m |
| `exp07` | 6/6 | 0.135650 m | 0.205374 m |
| `exp09` | 16/16 | 1.578837 m | 3.356628 m |
| `exp10` | 14/14 | 2.458338 m | 4.456982 m |
| `site2_handheld_5` | 4/4 | 14.242283 m | 20.502501 m |

## Acceptance

Code acceptance, all required before any bag is played:

1. Existing pose-graph, pipeline, submap, and config tests pass.
2. The new tests above pass.
3. `enabled = false` is the struct default, and loading either HILTI file
   yields `enabled = true`.

Bag acceptance:

1. `5/5` bags reach EOF.
2. A case that does not reset logs zero re-anchor attempts. Gate 3B did not
   reset `exp03`, `exp07`, `exp09`, or `site2_handheld_5`. A new reset on
   one of those is recorded as a regression to inspect, and thresholds are
   not loosened to hide it.
3. Cases with zero resets stay within 5% RMSE and 5% P95 of the Gate 3B
   table.
4. `exp10` keeps 14/14 when the run completes. If it resets, the log shows
   the weak edge and at least one attempt. A successful anchor must reduce
   that epoch's map-frame offset relative to the pre-anchor odom pose, and
   the report records RMSE, P95, and path length. Path length is not
   required to fall to the sparse-GT length in this gate.
5. If `exp10` does not reset, say so and rely on tests 1 and 8 for the
   mechanism. That outcome is not a failed gate.
6. A rejected attempt does not move `T_map_odom` or delete trajectory
   samples. Pose count stays on the same order as Gate 3B (about 3084,
   1316, 4451, 3681, 2050 for the five cases).
7. No rejection streak exceeds the Gate 3B maximums on a case that
   previously completed cleanly.

If no anchor is accepted on a resetting `exp10`, report the reason counters.
Do not change the configured limits in that same run.

## Implementation order

1. Add the epoch argument, `force_keyframe`, and config schema, default off.
2. Add the weak cross-epoch edge and tests 1–5.
3. Add `submitMapAnchor` and the trajectory fields. Tests 3, 4, and 11.
4. Add the filtered anchor cloud. Tests 6–7.
5. Add `requestReanchor` and `registerAnchor`. Tests 8–10.
6. Wire the pipeline reset, bootstrap guard, and ready-state request.
   Test 12.
7. Enable the two HILTI configs.
8. Run the unit tests.
9. Screen `exp09` and `exp10`.
10. Run the five-case gate.
11. Write `docs/robust-frontend/reports/gate-4-recovery-aware-reanchor.md`
    using the existing gate-report template. Include per-epoch attempt,
    rejection reason, and whether a prior was committed.

## Rollback

`pgo.reanchor.enabled = false` restores Gate 3B factors and keyframe
admission.

If the bag gate fails:

- turn the flag off in the two HILTI configs;
- keep epoch fields only when tests still show identical poses with the
  flag off;
- leave the Gate 3B reset and bootstrap path in place;
- record the rejection reasons in the gate report.
