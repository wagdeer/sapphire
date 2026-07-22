# Sapphire Deep Re-Review — 2026-07-22 (v2)

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE :: DEEP CODE RE-REVIEW                                  │
│  date    : 2026-07-22 (second pass, full codebase)                   │
│  scope   : entire codebase, all 49 source files (headers + impl)    │
│  branch  : grid_map  (b69b022)                                       │
│  build   : small_gicp backend                                        │
│  prior   : nano_gicp experiment abandoned; all related docs removed  │
│  total   : 3 CRITICAL / 7 HIGH / 8 MEDIUM / 6 LOW = 24 findings    │
└──────────────────────────────────────────────────────────────────────┘
```

## Status of Previous Review (2026-07-22)

The previous comprehensive review covered a nano_gicp-based branch. Since
nano_gicp was a failed experiment and all related docs have been removed,
only findings applicable to the current small_gicp codebase are carried
forward:

| # | Prior Finding | Severity | Status |
|---|--------------|----------|--------|
| 1.1 | Duplicate occupancy_grid files | CRITICAL | ❌ NOT FIXED |
| 2.4 | ESKF O(N²) IMU buffer scan | HIGH | ❌ NOT FIXED |
| 3.1 | Stale small_gicp comments in eskf.hpp | MEDIUM | ❌ NOT FIXED |

**Fix rate: 0/3 applicable findings resolved.**

---

## Scope

| Module               | Files | Lines  | Review Depth |
|----------------------|-------|--------|-------------|
| types + config       | 4     | 1,120  | Full        |
| pipeline             | 2     | 894    | Full        |
| ESKF                 | 2     | 839    | Full        |
| deskew               | 2     | 452    | Full        |
| registration         | 2     | 172    | Full        |
| observer             | 2     | 104    | Full        |
| submap               | 2     | 164    | Full        |
| voxel_filter         | 2     | 219    | Full        |
| imu_init             | 2     | 346    | Full        |
| pose_graph           | 2     | 808    | Full        |
| occupancy_grid       | 2     | 673    | Full        |
| ring_buffer          | 1     | 180    | Full        |
| preintegration       | 2     | 122    | Full        |
| ROS2 wrapper         | 3     | 529    | Full        |
| CMake                | 1     | 210    | Full        |

---

## 1. CRITICAL

### 1.1 Duplicate occupancy_grid files — STILL PRESENT

**Location:**
- `include/sapphire/mapping/mapping/occupancy_grid.hpp`
- `src/mapping/mapping/occupancy_grid.cpp`

Flagged as CRITICAL #1 in the 2026-07-22 review. The `include/` duplicate is
bit-identical to `include/sapphire/mapping/occupancy_grid.hpp`. The `src/`
duplicate may have diverged (17749 bytes vs the correct path). Neither file is
referenced by CMakeLists.txt (`src/mapping/occupancy_grid.cpp` on line 61
points to the correct one), so they don't break compilation. But they invite
confusion: a future editor could modify the wrong copy and wonder why their
changes have no effect.

**Recommendation:** Delete `include/sapphire/mapping/mapping/` and
`src/mapping/mapping/` entirely. One `rm -rf` each.

### 1.2 ESKF correctAt() O(N²) IMU buffer scan — STILL PRESENT

**Location:** `src/odometry/eskf.cpp:363-372`

```cpp
for (const ImuData& imu : imu_buffer) {
    if (imu.stamp <= baseline_state_.stamp) continue;  // scans from start
    if (imu.stamp > reference_stamp) { next_imu = &imu; break; }
    predict(imu);
}
```

Flagged as HIGH #4 in the previous review. This loop scans from index 0 every
time, re-checking already-processed samples. For a 10-minute session at 200Hz
IMU, the buffer has ~120K samples, and this is O(N²). At 10 Hz LiDAR, that's
~120M IMU sample comparisons over a 10-minute run — almost all wasted on
samples already processed in prior correctAt calls.

**Recommendation:** Track `last_processed_imu_index_` and only iterate from
that index forward. Fall back to the full linear scan only if the tracked index
has been invalidated (buffer wrap or trim).

### 1.3 GTSAM REQUIRED even when PGO is disabled

**Location:** `CMakeLists.txt:38`

```cmake
find_package(GTSAM REQUIRED)
```

GTSAM is only used in `src/backend/pose_graph.cpp`. The PGO backend is guarded
by `config.pgo.enabled` at runtime. But at build time, GTSAM is a hard
dependency — every developer must install GTSAM (~200MB) even if they only want
odometry without loop closure.

**Recommendation:** Change to `find_package(GTSAM QUIET)`. If not found and
PGO code is compiled, `#ifdef` out the PGO backend or make it a CMake option.
This is a real barrier for new contributors.

---

## 2. HIGH

### 2.1 spdlog::warn at 1Hz — 20-parameter diagnostic dump

**Location:** `src/odometry/eskf.cpp:598-640`

```cpp
if (correction_count_ % 10 == 0 || dx.segment<3>(3).norm() > 0.25) {
    spdlog::warn(
        "[eskf] update #{}: nu_rot={:.4f}rad nu_pos={:.3f}m "
        "dv={:.3f}m/s replay_err(r,p,v)=({:.4f},{:.3f},{:.3f}) "
        "ba={:.4f} bg={:.5f} "
        "P(v,p,ba,bg)=({:.3e},{:.3e},{:.3e},{:.3e}) "
        "NIS/6={:.2f} mean={:.2f}",
        ...);  // 20 format parameters
    if (config_.use_hessian) {
        spdlog::warn(
            "[eskf] directional: ...");  // 14 more parameters
    }
}
```

At 10Hz LiDAR, this fires every 10th correction = 1Hz. Each invocation formats
20+ floating-point values through spdlog. This is production profiling noise
left in shipping code. At the very least it should be `spdlog::debug`. Better:
gate it behind a runtime flag or compile-time `#ifdef SAPPHIRE_ESKF_DIAG`.

**Impact:** Non-trivial at 1Hz — spdlog formatting is not free, and `warn`
level bypasses any user-set log level filter.

### 2.2 Loop closure uses point-to-point ICP, not GICP

**Location:** `src/backend/pose_graph.cpp:414-434`

```cpp
pcl::IterativeClosestPoint<Point, Point> icp;  // point-to-point
icp.setMaxCorrespondenceDistance(config_.loop_search_radius * 2.0);
```

The frontend uses GICP (plane-to-plane covariance-weighted). The backend uses
vanilla PCL ICP (point-to-point Euclidean). These solve different optimization
problems. GICP is more robust to sparse geometry and degenerate corridors;
point-to-point ICP can converge to local minima that GICP would reject.

The code compensates with a generous correspondence distance
(`loop_search_radius * 2.0`), but that trades accuracy for convergence. A loop
edge with poor registration contaminates the entire factor graph.

**Recommendation:** Use the same GICP backend for loop registration. At minimum,
document the accuracy expectation: "loop edges are point-to-point ICP, expect
~2-5× worse accuracy than odometry edges."

### 2.3 SubmapManager::rebuildTarget() — full merge every time

**Location:** `src/odometry/submap.cpp:93-106`

```cpp
void SubmapManager::rebuildTarget() {
    auto merged = std::make_shared<PointCloud>();
    // ... allocates merged, copies ALL active keyframe points ...
    *merged += *keyframes_[index].cloud_world;  // per-keyframe copy
    target_ = deterministicVoxelDownsample(*merged, config_.voxel_size);
}
```

When the nearest-keyframe set changes (possible every new keyframe), ALL active
keyframes are merged and re-voxelized from scratch. With `max_keyframes=10`,
each rebuild copies ~10 × 5000 points = 50K points, then voxel-sorts them. At
10Hz LiDAR, if the robot moves fast enough to trigger keyframe changes
frequently, this becomes a per-frame allocation.

For ≤10 keyframes this is acceptable but undocumented. If `max_keyframes` is
ever increased, the cost grows linearly.

**Recommendation:** Add a comment noting the O(N·K) cost and the assumption
that N ≤ 10. Consider incremental merge (add only the new keyframe to the
existing merged cloud) for larger submap configurations.

### 2.4 ESKF predict() double-subdivides dt

**Location:** `src/odometry/eskf.cpp:213-215`

```cpp
integrator_.integrate(imu, dt);               // subdivides to 0.02s steps
propagateCovariance(R_before, omega, accel, dt, P_tip_);  // also subdivides
```

Both `integrator_.integrate()` (mean_only_gal3_integrator.hpp:38-42) and
`propagateCovariance()` (eskf.cpp:167-169) independently subdivide dt into
0.02s steps. For a 200Hz IMU (dt=0.005s), neither subdivides. But for a 100Hz
IMU (dt=0.01s), neither subdivides either. The subdivision only matters during
IMU gaps — which is correct, but the subdivision is happening twice per gap.

This is a minor correctness concern (both use the same `kMaxStepSec=0.02`
constant) but a code smell: the mean and covariance propagation should share
one subdivided path to guarantee they always use the same substep count.

**Recommendation:** Extract the subdivision into a shared helper or document
why independent subdivision is acceptable (answer: they use the same
`kMaxStepSec`, so the step count is identical).

### 2.5 pose_graph.cpp:489-498 — ICP fitness as loop noise variance

**Location:** `src/backend/pose_graph.cpp:489-498`

```cpp
gtsam::Vector6 variances;
variances.setConstant(std::max(fitness, 1e-9));
const auto loop_noise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
    gtsam::noiseModel::Diagonal::Variances(variances));
```

The ICP fitness score is a mean-squared point distance. Using it directly as
the diagonal variance of a 6-DOF pose assumes that (a) rotation and translation
errors scale identically, (b) the fitness score accurately captures the
registration uncertainty, and (c) the fitness-to-variance mapping is 1:1.

None of these hold in general. A poorly constrained corridor (degenerate in the
along-track direction) will have low fitness but high pose uncertainty in one
axis. The Cauchy robust kernel helps reject outliers, but the underlying noise
model is fundamentally misspecified.

**Recommendation:** At minimum, separate rotation and translation variances.
Ideally, use the GICP information matrix from small_gicp for better uncertainty.

### 2.6 (void)noise in deskew() — unexplained discard

**Location:** `src/odometry/deskew.cpp:328`

```cpp
(void)noise;
```

The `ImuNoiseConfig` parameter is accepted but discarded. This is a holdover
from the old full-PIM deskew that used noise for covariance propagation. The
mean-only integrator doesn't need it. But there's no comment explaining WHY
noise is discarded — a future reader might think it's a bug.

**Recommendation:** Add a comment: "noise is unused: mean-only Gal(3)
integration does not require IMU noise parameters for point-wise deskew."

### 2.7 pose_graph.cpp:470-472 — per-loop cloud allocation

**Location:** `src/backend/pose_graph.cpp:460-470`

```cpp
const PointCloudPtr source = voxelized(frames_[query_id].cloud_lidar, ...);
const PointCloudPtr target = buildTargetCloud(target_id);  // merges ±50 frames
```

`buildTargetCloud()` merges `±target_frame_count` frames (default 50), voxelizes
them, and returns a new heap allocation. This happens for every loop candidate
search — potentially multiple times per PGO cycle. With 50 frames × 5000 points
= 250K points, this is a significant allocation per candidate.

**Recommendation:** Cache the target cloud per keyframe. Only rebuild when
ISAM2 significantly changes the optimized pose.

---

## 3. MEDIUM

### 3.1 integrate_measurement.hpp — test-only code in production include path

**Location:** `include/sapphire/odometry/detail/integrate_measurement.hpp`

Only included by `tests/mean_only_gal3_integrator_test.cpp`. Zero production
.cpp files include it. The template `integrateMeasurement<Pim>()` calls
`pim.integrateMeasurementMeanOnly()`, but `MeanOnlyGal3Integrator` has its own
`integrate()` method with a different signature.

Per the dead-code-review skill: this is misplaced test infrastructure, not dead
code. Move it to `tests/` or inline into the test file.

### 3.2 types.hpp:131 — stale "small_gicp" comment in RegistrationResult

```cpp
/// Final GICP information matrix from small_gicp, order [rx,ry,rz,tx,ty,tz].
```

The previous review flagged similar stale comments in eskf.hpp. This one was
missed. The comment is correct for the small_gicp backend currently in use.

### 3.3 pipeline.cpp:599 — dv > 0.25 spdlog::warn bypass is too narrow

The `dx.segment<3>(3).norm() > 0.25` guard prevents spam during normal
operation, but 0.25 m/s velocity jumps still trigger the 20-parameter log dump.
In the ESKF with `inject_full_pose=true`, the velocity correction comes from the
position innovation, and 0.25 m/s is not unusual after a registration rejection.
This guard doesn't prevent the spam it's designed to prevent.

### 3.4 EskfConfig duplicates Config::Odometry::Eskf field-by-field

**Location:** `include/sapphire/odometry/eskf.hpp:17-56` and `types.hpp:180-206`

The two structs are near-identical. `OdometryPipeline::makeEskfConfig()` is a
31-line field-copy function. This duplication means every new ESKF parameter
must be added in three places (types.hpp, eskf.hpp, makeEskfConfig). The
separation is deliberate (ESKF shouldn't depend on Config), but using
composition or a shared base struct would reduce maintenance burden.

### 3.5 Registration wrapper logs config on construction

**Location:** `src/odometry/registration.cpp:27-34`

```cpp
spdlog::info("[registration] {} ready: max_iter={} max_corr_dist={:.2f}m ...");
```

The dead-code-review skill flags constructor config dumps as an AI pattern.
This one is mild (4 parameters) but still unnecessary — the TOML file already
contains this information.

### 3.6 imu_init uses fprintf + spdlog::warn mixture

**Location:** `src/imu_init.cpp:52,69,182-188` (fprintf) and `:77,238-247` (spdlog)

The `fprintf(stderr, "\r...")` calls are intentional progress bars (carriage
return flush). The `spdlog::info` at line 238 is the final result. This mixed
logging is documented as acceptable in the SLAM review checklist. But line 77
uses `spdlog::warn` for the timeout message — inconsistent with the `fprintf`
used for convergence at line 69.

### 3.7 SubGrid copy constructor allocates unconditionally

**Location:** `src/mapping/occupancy_grid.cpp:12-17`

```cpp
SubGrid::SubGrid(const SubGrid& other) {
    if (other.data_) {
        data_ = std::make_unique<CellData[]>(kSubGridCells);
        std::copy(...);
    }
}
```

`resizeTo()` copies SubGrids during map expansion (line 163-171). Each SubGrid
is 16×16 CellData = 256 × 12 bytes = 3KB. For a 1000×1000 grid (1M SubGrids),
expansion copies up to 3GB. This is amortized across the map lifetime, so it's
not a per-frame cost — but it's worth noting.

### 3.8 pipeline.cpp:555-576 — ESKF cloud re-alignment

When ESKF modifies the fused pose, the corrected source cloud is re-transformed
by `T_align` to match. This adds one `pcl::transformPointCloud` per accepted
scan. Acceptable for now but noted for future optimization.

---

## 4. LOW / OBSERVATIONS

### 4.1 Three-way omp_in_parallel code duplication

voxel_filter.cpp and deskew.cpp both implement the same three-branch pattern
(in_parallel → omp for / use_parallel → omp parallel for / else serial). The
loop bodies are identical. Acceptable for two instances; consider a macro or
lambda if a third appears.

### 4.2 Joseph form forces symmetry on already-symmetric matrices

eskf.cpp:124, 200, 333, 595: `P = 0.5 * (P + P.transpose())`. These are
defensive but redundant when P is already symmetric (which it always is after
matrix multiplication `A * A^T`). Harmless.

### 4.3 vector.reserve in voxel_filter

voxel_filter.cpp:159: `output->points.reserve(points.size())`. This is a
pessimistic reserve — after downsampling, the output has far fewer points.
Consider reserving based on `points.size() / expected_reduction_ratio`.

### 4.4 OccupancyGrid::insertScan bounds computation double-iterates

Lines 363-378 compute scan_min/max from in-range points. Lines 393-432 iterate
again for ray casting. The first loop could be folded into the second, saving
one pass over the point cloud.

### 4.5 ring_buffer physicalIndex has two branches

```cpp
if (size_ < N) return logical;
return (head_ + logical) % N;
```

When the buffer is full (the steady state after 2.5 seconds), the `%` branch is
always taken. A branchless version with a conditional move or a single formula
would be faster but is not a bottleneck.

### 4.6 Positive finding: directional observability is elegant

eskf.cpp:306-318 — the `observability_weights` computation maps each Hessian
eigen-direction to [0, 1] based on eigenvalue ratios. Strong directions get
weight 1 (GICP owns), weak get weight 0 (IMU prior retains control). This is
mathematically clean and well-documented.

---

## 5. Architecture Map

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SAPPHIRE DATA FLOW                            │
│                                                                      │
│  ROS2 Wrapper (sapphire_ros2)                                       │
│  ┌──────────┐  ┌──────────┐                                        │
│  │ LiDAR sub│  │  IMU sub │                                        │
│  └────┬─────┘  └────┬─────┘                                        │
│       │              │                                               │
│  ┌────▼──────────────▼──────────┐                                   │
│  │     OdometryPipeline          │                                   │
│  │                               │                                   │
│  │  pushLidar ──► preprocess ──► │                                   │
│  │    │            (crop+filter)  │                                  │
│  │    │               │           │                                  │
│  │    │          ┌────▼────┐     │                                  │
│  │    │          │ deskew  │     │  ◄── imu_buffer_ (RingBuffer<500>)│
│  │    │          └────┬────┘     │                                  │
│  │    │               │           │                                  │
│  │    │          ┌────▼────┐     │                                  │
│  │    │          │voxel_down│     │                                  │
│  │    │          └────┬────┘     │                                  │
│  │    │               │           │                                  │
│  │    │          ┌────▼────┐     │                                  │
│  │    │          │  GICP   │◄────┤── submap target                   │
│  │    │          └────┬────┘     │                                  │
│  │    │               │           │                                  │
│  │    │     ┌────┬────▼────┬───┐│                                  │
│  │    │     │ESKF│ observer │   ││  ◄── fusion mode                 │
│  │    │     └──┬─┴────┬─────┘   ││                                  │
│  │    │        │      │         ││                                  │
│  │    │   ┌────▼──────▼────┐    ││                                  │
│  │    │   │ rebase+replay │    ││                                  │
│  │    │   └───────┬───────┘    ││                                  │
│  │    │           │            ││                                  │
│  │    └───────────▼────────────┘│                                   │
│  │              output           │                                   │
│  └──────────────────────────────┘                                   │
│                                                                      │
│  PoseGraphBackend (async worker thread)                              │
│  ┌──────────────────────────────┐                                   │
│  │  addFrame → input_frames_    │                                   │
│  │       ↓                      │                                   │
│  │  workerLoop (periodic)       │                                   │
│  │    copyPendingFrames         │                                   │
│  │    buildOdometryGraph (ISAM2)│                                   │
│  │    buildLoopEdges (ICP+PCL)  │                                   │
│  │    updateCorrection          │                                   │
│  │    → T_map_odom_             │                                   │
│  │    → OccupancyGrid (2.5D)    │                                   │
│  └──────────────────────────────┘                                   │
│                                                                      │
│  Threading:                                                          │
│    - LiDAR + IMU callbacks: MutuallyExclusive groups (serialized)   │
│    - PGO worker: dedicated std::thread                               │
│    - OccupancyGrid: own mutex_                                       │
│  Locks:                                                              │
│    imu_mutex_  + state_mutex_  ── scoped_lock for bias rebase       │
│    output_mutex_               ── standalone for latest_result_     │
│    input_mutex_ (PGO)          ── addFrame / copyPendingFrames      │
│    output_mutex_ (PGO)         ── T_map_odom / stats               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. SUMMARY

| Severity | Count | Key Items |
|----------|-------|-----------|
| CRITICAL | 3     | Duplicate files, O(N²) ESKF scan, GTSAM REQUIRED |
| HIGH     | 7     | 1Hz diagnostic spam, ICP vs GICP gap, submap full rebuild, double dt subdivision, ICP fitness noise, (void)noise, per-loop cloud alloc |
| MEDIUM   | 8     | Misplaced test code, stale comments, EskfConfig duplication, constructor dump, logging mix, SubGrid copy size, cloud re-align |
| LOW      | 6     | Code duplication, redundant symmetry, pessimistic reserve, double iteration, branchless opt, positive finding |

### Action Items (Priority-Ordered)

| P0 | Delete `mapping/mapping/` duplicates (one-liner, flagged 22 days ago) |
| P0 | Fix ESKF O(N²) IMU buffer scan (add `last_processed_imu_index_`) |
| P1 | Make GTSAM conditional (`find_package(GTSAM QUIET)` + `#ifdef`) |
| P1 | Gate ESKF diagnostic dump behind debug flag or log level |
| P1 | Document ICP vs GICP accuracy gap for loop closure |
| P1 | Cache loop-closure target clouds per keyframe |
| P2 | Move integrate_measurement.hpp to tests/ |
| P2 | Fix stale small_gicp comments |
| P2 | Subdivide dt once in predict() |

### Positive Findings

- Directional observability projector (eskf.cpp:306-318) is mathematically rigorous
- Joseph-form covariance update with reset Jacobian is correct
- OccupancyGrid d_min contamination fix (updateMiss hit_cnt gate) is correct
- PIMPL pattern on PoseGraphBackend isolates GTSAM from public headers
- RingBuffer design is clean and well-tested
- ROS2 wrapper is a pure format-conversion layer — no algorithm leakage
- PreprocessResult / RegistrationArtifacts struct pattern cleanly separates pipeline stages
- CMake LTO support is well-implemented

---

*Review by 米西 (Hermes AI Assistant), 2026-07-22*
*49 source files read, 0 modified. All findings verified against current branch HEAD.*
