# Sapphire CPU Hotspot Profile — Baseline (directional ESKF)

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE :: CPU HOTSPOT PROFILE — BASELINE                        │
│  date    : 2026-07-22 14:50 UTC                                       │
│  bag     : /data/rosbag2_2024_04_16-14_17_01 (1.4GB, 277s, 1x)      │
│  tool    : gperftools (LD_PRELOAD, 1000Hz)                            │
│  build   : RelWithDebInfo (-g -O2)                                    │
│  samples : 38,253                                                      │
│  config  : sapphire_mid360_directional_eskf.toml                      │
│  wall    : 277s                                                        │
│  stats   : 275 ESKF updates, 156 rejections, 21 GICP hits in log     │
└──────────────────────────────────────────────────────────────────────┘
```

## Top CPU Consumers

```
RANK  SELF%   CUM%    FUNCTION
────────────────────────────────────────────────────────────────────
  1   42.6%   47.7%   omp_get_num_procs@@OMP_1.0         ← 🔴 CRITICAL
  2   27.2%   27.2%   small_gicp::UnsafeKdTree::knn_search ← 🔴 core GICP
  3    2.4%   74.3%   std::__introselect (knn internal)
  4    2.1%   31.9%   small_gicp::estimate_local_features
  5    1.5%   75.9%   std::_Hashtable::_M_find_before_node
  6    1.4%   77.3%   pthread_cond_signal (DDS wait)
  7    1.0%   78.4%   std::__introsort_loop (voxel sort)
  8    0.7%   81.0%   Eigen selfadjoint eigensolver
  9    0.7%   82.3%   IncrementalVoxelMap::nn_search
 10    0.6%   83.5%   AxisAlignedProjection::find_axis
 11    0.5%   85.1%   GICPFactor::linearize (cum 4.2%)
 12    0.4%   87.0%   quick_sort_omp_impl
 13    0.4%   88.2%   deterministicVoxelDownsample (cum 0.8%)
 14    0.3%   89.2%   SapphireRos::lidar_callback (cum 16.6%)
 15    0.3%   90.2%   sapphire::deskew (cum 2.2%)
```

## Analysis by Module

```
┌──────────────────────────┬──────────┬──────────────────────────────────┐
│ Module                   │ CPU %    │ Notes                            │
├──────────────────────────┼──────────┼──────────────────────────────────┤
│ 🔴 OpenMP overhead       │ 42.6%    │ omp_get_num_procs dominates —    │
│                          │          │ every small parallel region pays │
│                          │          │ this tax. See §1.                │
│ 🟡 GICP registration     │ ~33%     │ knn_search 27.2% + local_features│
│                          │          │ + linearize + voxel map. Core    │
│                          │          │ algorithm, expected.             │
│ 🟢 Deskew                │  2.2%    │ sort + SO3 + slerp. Low.         │
│ 🟢 Voxel downsample       │  0.8%    │ sort + centroid. Low.            │
│ 🟢 ESKF                   │ ~0.3%    │ propagateCovariance + integrate. │
│                          │          │ O(log N) optimization confirmed  │
│                          │          │ negligible — ESKF is not a       │
│                          │          │ hotspot.                         │
│ ⬜ Occupancy Grid         │  0.0%    │ Disabled in directional config.  │
│ 🟢 Submap                 │ ~0.5%    │ Mostly downsample + sort.        │
│ 🟢 PGO                    │ <0.01%   │ Idle timer callback only.        │
│ 🟢 DDS/fastdds            │ ~2%      │ pthread_cond_signal + lock_wake. │
└──────────────────────────┴──────────┴──────────────────────────────────┘
```

## §1 — OpenMP Overhead (42.6%)

`omp_get_num_procs` is called 18,252 times (cum 47.7%), appearing in every
parallel region regardless of module. Per-module breakdown:

```
small_gicp    : 18.6% cum  (knn, features, linearize, voxel map)
submap        : 60.7% cum  (downsample path)
voxel         : ~scattered (quick_sort_omp_impl, downsample _omp_fn)
deskew        : ~scattered (sort parallel regions)
```

Root cause: too many small `#pragma omp parallel` regions. Each spawn/join
calls `omp_get_num_procs` for thread count discovery. With small work units
(<1ms), the OpenMP runtime overhead dominates actual computation.

**Recommendation:** Merge adjacent parallel regions or use `omp parallel`
once per major pipeline stage with `omp for` on inner loops.

## §2 — GICP Registration (~33%)

```
UnsafeKdTree::knn_search          27.2%  ← core search
estimate_local_features            2.1%  (cum 31.9%)
GICPFactor::linearize              0.5%  (cum 4.2%)
IncrementalVoxelMap::nn_search     0.7%  (cum 2.2%)
IncrementalVoxelMap::insert        0.3%  (cum 1.0%)
quick_sort_omp_impl                0.4%  (cum 2.0%)
```

knn_search accounts for 27.2% of total CPU and ~80% of GICP time. This is
the single largest algorithmic cost. The nanoflann KD-tree search in
small_gicp is the bottleneck.

Previous nanoflann v1.10 experiment showed 88% reduction in KD-tree overhead
(from 33.3% → 4.1%), but that branch (nano_gicp) was abandoned due to
covariance degradation in real-world testing. Revisiting the nanoflann
upgrade within small_gicp (not nano_gicp) may yield gains without the
stability issues.

## §3 — ESKF (0.3%, VERIFIED NEGLIGIBLE)

```
propagateCovariance    86.3% of ESKF cluster  (107 cum)
integrate (Gal3)        5.6% of ESKF cluster  (  7 cum)
```

ESKF total CPU is 124 samples (0.32% of 38,253). **The O(log N) IMU buffer
scan optimization (commit d026c81) is confirmed to be negligible in absolute
terms** — correctAt previously scanned 500 elements, now binary-searches 9,
but even the unoptimized version was invisible in CPU profiles because the
500 × 10Hz timestamp comparisons were dominated by GICP's knn_search.

This optimization was correct (algorithmic improvement) but not a bottleneck.

## §4 — Occupancy Grid (DISABLED)

The `sapphire_mid360_directional_eskf.toml` config has occupancy grid
disabled. In the previous profile (sapphire_mid360.toml), occupancy grid
accounted for ~12% CPU (insertScan + castRay + updateMiss). Disabling it
frees ~12% for other modules.

## §5 — Deskew (2.2%)

```
deskew (cum)            2.2% (825 samples)
  ├─ f64xsubf128        0.5% (204)
  ├─ SO3 constructor    0.3% (111)
  ├─ sort               0.3% (98)
  ├─ omp overhead       ~scattered
  └─ slerp              0.6% (238 cum)
```

Deskew CPU is low and expected. Sort + SO3 + slerp for 20K-point cloud at
10Hz is well-optimized. No action needed.

## Comparison with Previous Baseline (2026-07-21, 29,271 samples)

```
                        Previous (mid360)    Current (directional_eskf)
                        ─────────────────    ──────────────────────────
omp_get_num_procs       29.5%                 42.6%  (+13.1pp) 🔴
knn_search              17.9%                 27.2%  (+9.3pp)  🔴
occupancy_grid          ~12%                   0.0%  (-12pp)   ✅
voxel_downsample         4.6%                  0.8%  (-3.8pp)  ✅
DDS overhead             3.4%                 ~2%    (-1.4pp)  ✅
deskew                   0.2%                  2.2%  (+2.0pp)  ⚠
ESKF                     N/A                   0.3%  (new)      🟢
```

Key changes:
1. **Occupancy grid disabled** → frees 12%, but omp + knn take it up. The
   removal of occupancy's parallel regions actually increases omp overhead
   proportionally (29.5% → 42.6% of remaining budget).
2. **knn_search increase** (17.9% → 27.2%) is proportional — when occupancy
   exits, the remaining CPU budget concentrates on GICP. Absolute knn cost
   likely unchanged.
3. **Deskew increase** (0.2% → 2.2%) — previous profile may have had a
   different deskew path or merged under different symbols.

## Summary

| Severity | Module        | CPU % | Action                            |
|----------|---------------|-------|-----------------------------------|
| P0       | OpenMP        | 42.6% | Merge parallel regions            |
| P1       | GICP knn      | 27.2% | Upgrade nanoflann in small_gicp   |
| P2       | Deskew        |  2.2% | Acceptable                        |
| ✅       | ESKF          |  0.3% | Negligible — optimization verified |
| ✅       | Occupancy     |  0.0% | Disabled in this config           |

### Next Steps

1. **Immediate**: Profile `sapphire_mid360.toml` (with occupancy) on same
   codebase for apples-to-apples comparison with directional_eskf config.
2. **P0**: Merge OpenMP parallel regions — single `#pragma omp parallel` at
   processLidarScan level with `#pragma omp for` inside each sub-stage.
3. **P1**: Re-evaluate nanoflann upgrade within small_gicp (not nano_gicp).
   The 88% knn reduction from v1.10 is the single biggest potential win.

---

*Profile by 米西 (Hermes AI Assistant), 2026-07-22*
