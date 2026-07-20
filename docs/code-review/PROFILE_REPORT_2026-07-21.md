# Sapphire CPU Hotspot Profile — Real Data (29,271 samples)

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE :: CPU HOTSPOT PROFILE                                  │
│  date    : 2026-07-21 17:29 CST                                      │
│  bag     : /data/rosbag2_2024_04_16-14_17_01 (1.4GB)               │
│  tool    : gperftools (ProfilerStart, 1000Hz)                       │
│  build   : RelWithDebInfo (-g -O2)                                   │
│  samples : 29,271                                                    │
│  config  : sapphire_mid360.toml (eskf + VGICP + pgo + occupancy)    │
└──────────────────────────────────────────────────────────────────────┘
```

## Top CPU Consumers

```
RANK  SELF%   CUM%   FUNCTION
────────────────────────────────────────────────────────────────────
  1   29.5%   38.5%  omp_get_num_procs@@OMP_1.0         ← 🔴 CRITICAL
  2   17.9%   17.9%  small_gicp::UnsafeKdTree::knn_search ← expected
  3    3.7%    7.6%  small_gicp::quick_sort_omp_impl
  4    3.3%    4.6%  OccupancyGrid::updateMiss           ← ⚠ high
  5    3.1%    3.1%  std::__introsort_loop (voxel sort)
  6    2.1%    4.6%  deterministicVoxelDownsample
  7    1.7%   21.4%  small_gicp::estimate_local_features
  8    1.6%    1.6%  std::_Hashtable
  9    1.5%    1.6%  std::vector::_M_range_insert
 10    1.2%    1.2%  __munmap (glibc)
  ──────────────────────────────────────────────────────────────────
  N    1.0%    8.7%  OccupancyGrid::insertScan           ← ⚠ high
  N    0.9%    6.6%  OccupancyGrid::castRay              ← ⚠ high
  N    0.2%    1.0%  sapphire::deskew                    ← surprisingly low
  N    3.4%    3.4%  fastdds/RTPS (DDS overhead)         ← ROS2 tax
```

## Analysis by Module

```
┌──────────────────────┬─────────┬──────────────────────────────────┐
│ Module               │ CPU %   │ Notes                            │
├──────────────────────┼─────────┼──────────────────────────────────┤
│ 🔴 OpenMP overhead   │ 29.5%   │ omp_get_num_procs dominates —   │
│                      │         │ too many small parallel regions  │
│ 🟡 GICP/VGICP (knn)  │ 17.9%   │ Expected. Core scan registration │
│ 🟡 Occupancy Grid     │ ~12%    │ insertScan+castRay+updateMiss+  │
│                      │         │ toMsg — running every keyframe,  │
│                      │         │ even without subscribers         │
│ 🟢 Voxel downsample   │  4.6%   │ Expected. Parallel sort+centroid │
│ 🟢 small_gicp misc    │ ~3.7%   │ sort + local features            │
│ 🟡 ROS2 DDS           │ ~5%     │ fastdds + RTPS overhead           │
│ 🟢 Deskew              │ ~1%     │ Very efficient!                   │
│ 🟢 Observer/ESKF       │ ~2%     │ Mostly Eigen ops                  │
│ 🟢 PGO loop search     │ <1%     │ Negligible                        │
│ 🟢 IMU propagation     │ <1%     │ MeanOnlyGal3 is fast              │
└──────────────────────┴─────────┴──────────────────────────────────┘
```

## Critical Findings

### 🔴 P0 — OpenMP overhead at 29.5%

`omp_get_num_procs` takes nearly 30% of ALL CPU time. This function is called
at the start of every OpenMP parallel region to determine thread count. The root
cause is likely many small parallel regions being created (each voxel filter call,
each deskew call, etc.) instead of a few large ones.

**Fix:** Reuse thread pools, merge small parallel regions, or use `OMP_WAIT_POLICY=active`
to keep threads spinning instead of creating/destroying them.

### 🟡 P1 — Occupancy Grid runs unconditionally

Total occupancy grid cost: ~12%. This includes `insertScan` (8.7% cum),
`castRay` (6.6% cum), `updateMiss` (3.3%), and `toMsg` (0.9%). The occupancy
grid runs on every PGO keyframe even though it's primarily a visualization
feature. The TOML config has `pgo.occupancy.enabled = true`.

**Fix (config):** Set `pgo.occupancy.enabled = false` when not needed.
**Fix (code):** Lazy evaluation — check subscriber count before inserting.

### 🟢 Green — Deskew is very efficient

Deskew takes only ~1% of CPU. The MeanOnlyGal3 integrator and cubic Hermite
interpolation are well-optimized. This validates the decision to use
mean-only integration for deskew.

### 🟢 Green — IMU propagation is negligible

IMU-rate state propagation (<1%) confirms the MeanOnlyGal3Integrator is
correctly optimized. No covariance computation at IMU rate.

## Action Items

| Pri | Action | Est. Saving |
|-----|--------|-------------|
| P0  | Fix OpenMP overhead (merge parallel regions, OMP_WAIT_POLICY) | -20-25% |
| P1  | Disable occupancy when no subscriber | -12% |
| P2  | ROS2 DDS tuning (CycloneDDS, localhost-only) | -3-5% |
| P3  | Optimize occupancy updateMiss hot loop | -1-2% |

## Pre-Profiling vs Actual

| Hypothesis | Expected | Actual | Match? |
|-----------|----------|--------|--------|
| GICP/VGICP is #1 | 35-45% | 21.6% cum | ✅ direction |
| Deskew is #2 | 15-20% | ~1% | ❌ way off |
| Voxel is #3 | 10-15% | 4.6% | ✅ direction |
| OpenMP overhead | Not predicted | 29.5% | ❌ completely missed |
| Occupancy grid | 3-5% | ~12% | ❌ much higher |
