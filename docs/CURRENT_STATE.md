# Sapphire current engineering state

## Visual default disabled — 2026-10-08

User requested all shipped profiles default to visual_loop.enabled=false. Hilti,
Hilti with RICP fallback and mid360_visual templates retain their calibration and
opt-in image-attribute settings, but camera subscriptions and visual processing
are disabled until explicitly enabled. Core already defaults visual.enabled=false;
no estimator, GICP fallback, LiDAR backend or other configuration changes. Earlier
visual-enabled measurements below are historical opt-in results.

## Optional shared-map Gaussian ESKF fallback — 2026-10-08

User accepted retaining this feature in production source as a configurable ESKF
option, and then moved its implementation to frontend/ricp as an optional Ricp
observation component. The ESKF path still runs point-plane first and calls Ricp
only on eligible geometric rejection; this is a code-ownership change. `odometry.gicp_fallback.enabled=true` enables observation fallback
after ESKF rejects for no matches or normal support. Default is false. All five
shipped algorithm profiles expose the switch and advanced parameters; the ROS
adapter reads them from algorithm_config at startup (not a dynamic ROS parameter).
The old standalone RICP frontend has been retired: frontend=ricp and the old
odometry.ricp table now produce migration errors. Keep frontend=eskf; the shipped
hilti2022_exp09_ricp profile now enables gicp_fallback within ESKF. [Usage](../sapphire_ros2/README.md#eskf-高斯观测兜底).
The normal threshold remains 14. Invalid inputs/numerical failures do not trigger
fallback. Source voxel statistics are computed lazily; target means/covariances
come from existing voxel-map leaf moments, including nonplane leaves, with bounded
neighbor-root search and per-call caching. No second persistent map or KD-tree is
built. This is a raw Gaussian voxel metric, not standard VGICP preprocessing or a
calibrated measurement covariance.

The initial custom registration solver has been replaced by unmodified installed
small_gicp 1.0.1: GICPFactor, SerialReduction, DistanceRejector, LM iteration and
convergence. A call-local traits adapter exposes the existing map; serial reduction
is required by its mutable lazy cache. The library solves only LiDAR registration,
seeded with the IMU prediction, then the result is fused once with the full 15-state
ESKF prior. GTSAM supplies the pose-chart and rotation-reset Jacobians; library
factors are relinearized at the returned pose to obtain geometric information.
Velocity and bias updates follow prior cross-covariances. Library convergence is
necessary but not sufficient: finite/support/correction/objective checks remain
ours. Rejection is atomic and acceptance has one map/window commit. Gal3, deskew,
initialization, local BA and reset policy are unchanged. No IMU prior is counted
inside the library registration objective or fused twice.

Full Hilti exp09, 4,466 scans, root1m/minimum leaf0.5m:

| Fallback | Resets | Rejected observations | Fallback accepted/attempts | Sparse position RMSE |
|---|---:|---:|---:|---:|
| Off | 30 | 51 | 0/0 | invalid after resets |
| Archived custom solver | 0 | 0 | 37/37 | 0.69009m |
| small_gicp, 6 iterations | 0 | 12 | 46/58 | 0.53426m |
| small_gicp, 20 iterations | 0 | 0 | 21/21 | 0.42086m |

The production default budget is now20, matching the library default; the six-step
trial's12 rejections were library nonconvergence (at most2 consecutive). The20-step
trial's additional fallback time was mean0.562ms,p950.823ms,max0.923ms,21 calls.
These are single offline timing runs, not a statistical speedup or latency claim.
Minimum leaf0.25m with fallback enabled makes0 calls,0 resets and RMSE0.21546m;
its entire trajectory is exactly identical to the prior ESKF baseline. Coarse off
also reproduces its archived trajectory exactly. Accuracy uses16 sparse positions
with independent rigid no-scale alignment, not dense trajectory/orientation truth.
Both map geometry and local BA change with depth. No general robustness claim or
accuracy superiority over the fine-map baseline follows from this one sequence.

After relocation, core/replay and ROS builds and all three targeted test targets
pass. Full 4,466-scan coarse on/off replays reproduce the preceding library
implementation exactly: poses, velocities and all observation outcomes (excluding
compute time). Enabled remains21 successful fallback calls,0 rejected frames and
0 resets; disabled remains51 rejected observations and30 resets. Retired selector/
parameter tables report migration errors; the updated shipped profile loads.
[Relocation results](../../audit/2026-10-08-ricp-component/results.json) ·
[Configuration migration](../../audit/2026-10-08-ricp-component/config-migration.json).

The targeted test targets cover library solve,
nonidentity rotation/translation, world-frame equivariance, velocity/bias
cross-covariance, positive covariance, nonplane leaves, late-rejection rollback,
pipeline rejection/IMU continuity with fallback off/on, and module boundaries.
Engineering support/correction gates remain uncalibrated across devices;
mixed-surface voxels and correlated geometry can still give misleading confidence.
No new degeneracy-direction controller.
[Design](../../audit/2026-10-07-voxel-reset/PLAN.md) ·
[Current library results](../../audit/2026-10-08-gicp-library/results.json) ·
[Selected trial configuration](../../audit/2026-10-08-gicp-library/layer1_library_default.toml) ·
[Library provenance](../../audit/2026-10-08-gicp-library/library-provenance.json) ·
[Archived custom solver results](../../audit/2026-10-08-gaussian-fallback/results.json).

## Experimental RICP and native frontend comparison — 2026-10-07

Historical record: this standalone RICP implementation was replaced on 2026-10-08
by the optional ESKF observation component described above. Its old source and
configuration are archived in audit/2026-10-08-ricp-component/before. The following
results describe the retired algorithm, not the current Ricp component.

`odometry.frontend="ricp"` selects the new frontend/ricp correction path; ESKF
remains the default. RICP uses small_gicp VGICP against the accepted local window,
direct pose replacement and velocity feedback. Existing Gal(3) propagation/deskew,
initialization and local BA remain shared with ESKF. Cached source geometry is
bounded by that window and rebuilt at its current poses after BA. Rejection leaves
the propagated state/covariance unchanged and does not insert the scan into the
map/window. Optional reverse registration is off by default and does not prove
uniqueness. No RKO IMU model, gravity regularizer or ZUPT is integrated into Sapphire.

Full Hilti2022 exp09 replay: 4,466 scans, same 16 sparse position references,
independent rigid no-scale alignment. ESKF RMSE 0.21546m; RICP 1.95202m, 191 rejected
updates and zero resets. An initial target-cell-count gate incorrectly applied the
source correspondence minimum to aggregate voxels; its 34-reset failure is retained
and the gate corrected/tested. RICP remains experimental: successful completion is
not evidence of an accuracy improvement.

The user also requested native historical/official baselines. Unmodified pre-ESKF
Sapphire 320951b loses LiDAR corrections after 87.417s with the historical profile
and Hilti extrinsics. Official RKO-LIO v0.4.0 dba08e6 defaults stop around 320s after
too few keypoints and then no correspondences. RKO voxel_size=.5 completes but has
9.205m RMSE without initialization; enabling native gravity/bias initialization
reduces this to 4.308m, still worse than current ESKF. Initial IMU Z is opposed to
world-up. These are whole-frontend comparisons, not isolated ICP/IMU ablations.
RKO's native map/IMU processing and historical Sapphire's geometric observer are
not production dependencies or changes to current Gal3. No universal performance
claim follows from this one sparse-truth sequence.

Core/ROS builds and all 40 registered tests pass across the initial run plus
host-permission and ROS-environment retries; historical observer/deskew tests pass.
No changes to A1–B3/A2/F1 settlement, persistence schema or visual mission.
[Design](../../audit/2026-10-07-ricp/PLAN.md) ·
[Results, provenance and reproduction](../../experiments/ricp/README.md).

## Visual subdirectories — 2026-10-07

backend/visual now contains feature / faiss / utils / solver. Feature detection,
tracking, keyframes and scene feature records live in feature; binary Faiss indexing
in faiss; image helpers in utils; PnP/stereo geometry in solver. visual_loop remains
the integration entry above them. No FBoW implementation exists, so no placeholder
fbow directory is retained.21 visual files moved;120 owned C++ files are byte-identical
after reversing include-path changes. Camera implementations are also consolidated
inside common/camera: camera selects the model, calibration supplies shared geometry,
and pinhole/kb4 have their own implementation files. Each camera still owns its
calibration by value; callers need no separate calibration object. All20 original camera function
bodies are preserved; float behavior is unchanged. Core/ROS builds,36/36 core tests,3/3 ROS tests
and fresh-installed-header compilation pass. No algorithm/schema/schedule change
or new bag/performance run. [Evidence](../../audit/2026-10-07-visual-layout/README.md).

## Final module layout, utilities and float camera models — 2026-10-07

This refinement supersedes the directory layout in the module-split entry below.
Core source has frontend / backend / common / tools. parameters.h/.cpp sits beside
pipeline; tools is an independent library for SIMD, timer, parallel execution and
LRU cache. Image helpers and feature/descriptor storage belong to backend/visual;
the single-use keypoint grid and timestamp formatter are private to their callers.
Unused require/IO helpers are removed. Both full SIMD wrappers are preserved
byte-for-byte against the pre-cleanup snapshot, including unused operations.

Camera geometry is split into common/camera/{ucm,kb4,pinhole}.hpp. Float calibration,
projection/bearings and extrinsics belong to models; calibrated optical streams
use a value dispatcher over pinhole/radtan and KB4/equidistant. UCM remains explicit
legacy scene geometry, not a newly enabled configuration mode. Model ownership and
cached inverse focal design were compared with the local VSLAM Perspective code.
K and distortion are float, including production OpenCV stereo calls. Existing
LIO/stereo solver and persistence interfaces retain their types; frozen image
projection metadata records the float intrinsics actually used for the pixels.

Final complete core build and36/36 tests pass; ROS build and3/3 tests pass.
Fresh-installed-header compilation, independent module linking and include-boundary
checks pass. Independent double OpenCV projection oracle: maximum observed float
pixel error0.0000763602px; asymmetric extrinsics, raw bearing roundtrips, temporal
tracking, stereo and image persistence regressions pass. This is scoped synthetic/
regression evidence, not a new full-bag/performance acceptance run. The queued
independent odometry/mapping split and closed A1–B3/A2/F1 semantics are unchanged.
[Module layout](MODULE_LAYOUT.md) · [Evidence](../../audit/2026-10-07-parameters-placement/README.md).

## Module split and shared point observation — 2026-10-07

Core source is now frontend / backend / common. Frontend contains initialize,
eskf and its own common: reusable IMU propagation/deskew, preintegration, sensor
synchronization and native LiDAR preprocessing. Backend owns visual image selection
and attributes alongside graph/registration/storage/grid/retrieval. Top-level common
owns shared types/configuration/camera geometry/tools. Three static implementation
targets independently depend on common; the existing sapphire::core pipeline is
the composition entry point. Qualified includes replace old flat/mapping paths.

PointUncertainty is removed. pointVar::projectVariance computes the same projected
point/pose uncertainty and outputs the caller-owned Jacobian; no per-point fields
are added. Existing full-covariance, finite-difference, coordinate-invariance and
recursive matching checks pass, with a storage-size assertion. ROS wire decoding
calls the shared native preprocessing policy without another full-cloud buffer.
Dual-LiDAR ROS message-queue fusion remains in the adapter.

Final core build and36/36 tests pass; ROS build and3/3 tests pass. Independent
frontend/backend link checks, module dependency guard and fresh-installed-header
compilation pass. No new bag/performance run was performed for this restructuring;
previous Hilti measurements remain historical evidence, not new measurements.
Existing pipeline worker/state ownership and sliding-window/ESKF coupling remain;
this is not the queued independent odometry/mapping algorithm split. A1–B3/A2/F1
settlement, graph/persistence formats and user-selected visual mission are unchanged.
[Module layout](MODULE_LAYOUT.md) · [Verification](../../audit/2026-10-07-module-split/README.md).

## Current product direction: submap image attributes — 2026-10-07

This section supersedes the earlier visual-loop acceptance goals below. The user
now wants representative image attributes, not visual retrieval/PnP or persisted
visual features. Enabled Hilti and mid360_visual profiles set attributes_only=true;
legacy readers/tools remain available for old maps and explicit legacy profiles.
Runtime per-camera FAST/LK coverage/renewal plus original LIO motion selects images;
LiDAR independently switches submaps. Selected views are undistorted grayscale PNGs
with frozen projection, camera/time/original pose; at most8 per camera/submap.
New VOM4 records contain zero feature/descriptor rows and no triangulated scene.
SQL schema, graph settlement and closed A1–B3/A2/F1 contracts are unchanged.

Full Hilti1x:4464odom,7132 evaluated images,303selected,243retained images across
18submaps; PNG payload10,210,305B. Every saved image decoded and passed CRC/identity
checks; feature/descriptor fields and VisualScene rows are zero. Gallery exports
contain the exact stored PNG bytes. Existing33core tests plus the new cross-process
image persistence test pass; ROS3/3 pass. Checked finish, database integrity and
independent current-reader reopen with unchanged file hash pass.

Peak RSS790,269,952B; mean CPU54.05% of one core; maximum30threads. No observed
Reset/refusal/image eviction. Three LiDAR constraints persisted (11→7,12→8,13→7),
revision21. Sparse16-position online/committed RMSE0.215/0.231m; no demonstrated
accuracy gain. Single-run resources include a small user-requested read-only PNG
preview export and do not establish a performance gain or global memory bound.
Selector-only ordinary p50/p95 is1.076/2.923ms; PNG processing is excluded from
that local timer.16 tail images beyond the last marginalized pose remain unprocessed.

**Image-attribute path verified on this dataset; full mainline acceptance remains
open.** Images are grayscale views, not semantic labels or guaranteed representative
coverage. Old executables cannot read VOM4; old VOM0–3 remain readable by this build.
Long-duration memory convergence, automatic cross-session continuation and the
queued independent odometry/mapping split remain unqualified or unimplemented.
PnP=0 is intentional in the current product direction, not a remaining blocker.
[Evidence and complete image gallery](../../audit/2026-10-07-submap-image-attributes/README.md).

## Current-version LiDAR-only loop check — 2026-10-07

Complete Hilti1x with vision disabled, same packed-double executable:4464odom,
no Reset/refusal, peak RSS732,708,864B, CPU55.92% of one core, max29threads.
Three online BBS→GICP constraints persisted (DB11→7,12→8,13→7);18nodes,
revision21, checked finish/integrity and unchanged-hash independent reopen pass.
The tail/start constraint from the older4-loop run was not reproduced:9candidates,
BBS rejected, no GICP. Current result is **3**, not4. Sparse16-position RMSE
online0.215m versus committed0.231m: loop operation demonstrated, accuracy gain
not demonstrated. This does not qualify the visual branch or full mainline.
[Evidence](../../audit/2026-10-07-lidar-loop-check/README.md).

## Memory attribution and OpenCV baseline — 2026-10-07

ORB detection/descriptors remain on OpenCV; experimental custom ORB was withdrawn
per user preference. The executable sets OpenCV threads2 before workers start.
Retained reference-equivalent SIMD FAST replenishment and2x gray downsampling;
symmetric9x9 moment covariance uses45 doubles, expanded for plane propagation.
No float precision reduction, residency policy, estimator schedule or A2/F1 change.
Root1m/minimum0.25m retained; the0.5m trial reset13times and was rejected.

Final core33/33, ROS3/3, complete Hilti1x and independent unchanged-hash reopen pass.
4464odom,3566paired observations,148keyframes/10,902depths; no Reset/refusal/image
eviction or marginal wait>1ms. Conservative RSS907,485,184B versus995,368,960B;
CPU61.29% versus64.74% of one core; max39 versus69threads. Same-binary visual-off
peak771,813,376B is an end-to-end attribution, not isolated visual heap size.

**Documented tradeoffs; mainline NOT ready for final acceptance.** Ordinary visual
p50/p95 rises0.705/1.099→1.304/3.156ms; keyframe6.017/9.844→9.016/15.183ms.
Online sparse16-position RMSE0.179→0.199m; committed0.311→0.190m. Single-run
trajectory differences do not establish unchanged precision or an accuracy gain.
PnP attempts0;41active scenes/6loops. Long-run RSS convergence, useful real visual
loops and automatic cross-session continuation remain unqualified.16tail images
still fall beyond the final marginalized pose. This is not an enforced1G budget.
[Evidence and limitations](../../audit/2026-10-07-local-residency/README.md).

User-requested [independent odometry / mapping split](ODOMETRY_MAPPING_SPLIT.md)
is queued for a subsequent architecture trial, **not implemented**. Existing ESKF
and sliding-window optimization still share map/state/reset; ICE-BA was reviewed,
not integrated. Prior local-residency draft is likewise unimplemented.

## Memory budget follow-up — 2026-10-07

LIO point slots now allocate on actual insertion; the existing empty-window reuse
pool has a64MiB total capacity budget, enforced after each marginalized frame, and
is emptied when odometry stops. Active observations,10-frame window, voxel sizes
and700m cumulative-travel aging remain unchanged. Owner capacity diagnostics are
opt-in through SAPPHIRE_PROFILE_BACKEND. No graph/schema/A2/F1 changes.

Final core31/31 and ROS3/3 pass. Complete Hilti1x replay:4464odom,148 visual
keyframes/10,902 stereo depths, no observed Reset/refusal/eviction or marginal
wait>1ms; checked close/reopen and unchanged database hash pass. Final CPU64.74%
of one core. Conservative observed peak RSS995,368,960bytes (kernel989,171,712;
sampled995,368,960), down from the preceding sampled1,104,437,248bytes.

**Partial qualification only; mainline NOT ready for final acceptance.** The
1,000,000,000byte test threshold passes this run by only4,631,040bytes. This is
not an enforced global budget or long-run guarantee. LIO historical tree capacity
still reaches~373MB; locality/history residency policy remains unresolved.
PnP attempts0; sparse16-position online/committed RMSE0.179/0.311m;41 active
scenes,6 accepted constraints, no observed retirement. Global memory convergence
and useful real visual loops remain unqualified.
[Full measurements, failed intermediate run and limits](../../audit/2026-10-07-memory-budget/README.md).


## Flow/LIO keyframes and final ORB stereo — 2026-10-07

The current Hilti profile selects visual keyframes using bounded cam0 FAST/LK
track renewal/coverage and original LIO translation/rotation. Ordinary frames
compute no ORB descriptors or stereo depth. Selected pairs run the established
multiscale ORB stereo matcher (360x270, <=500 per camera), and attach to the
current independently switched submap. Experimental LK-seeded stereo was rejected
after comparison; no visual odometry or LiDAR-assigned feature depth.

Final core31/31 and ROS3/3 pass. Full1x replay:4464odom,148 keyframe pairs,
10,902 stereo depths, no Reset/refusal/image eviction or marginal wait>1ms.
Ordinary p50/p95 0.694/1.091ms, keyframe6.093/10.138ms; CPU61.37% of one core.
Checked close/reopen and unchanged database hash pass.16 tail images beyond the
last marginalized pose are not processed at finish.41nodes/7loops/all scenes active.

**Mainline acceptance is not passed.** Sampled peak RSS1,104,437,248bytes exceeds
the user's newly specified1G ceiling (conservatively1,000,000,000bytes). PnP attempts
remain0; sparse16-position online/committed RMSE0.179/0.249m. Long-duration memory
convergence, useful real visual metric loops and real automatic cross-session
continuation remain unqualified. Memory diagnosis is the next authorized work;
A2/F1 remains closed. [Evidence](../../audit/2026-10-07-flow-lio-keyframes/README.md).


## Direct stereo indexing profile — 2026-10-07

Hilti profile `sapphire/config/hilti2022_exp09.toml` now processes each 720×540
camera at 360×270 with <=500 features. Explicit `stereo_depth=true` disables the
old temporal tracking path for this profile. Calibrated timestamp pairing, sparse
shared-feature descriptor matching, fisheye/pinhole rectified patch subpixel
refinement and disparity-domain gating generate camera-local depth; no fixed
metric-distance ceiling or LiDAR-assigned depth. LIO poses only anchor observations.
Visual submap changes use bounded descriptor references, not global covisibility.

Stereo evidence is explicitly two-camera, VOM3; temporal evidence still requires
>=3 observations, and legacy/v1/v2 remain supported. Older readers reject VOM3.
Core31/31 and ROS entry3/3 pass after stereo optimization. Controlled real-image
module A/B reduces p50/p95 about13% with identical output hashes. LIO and mapping
already run in separate threads; the bounded marginal queue can still backpressure.
The final full1x Hilti replay completes4464odom/849stereo pairs with no observed
Reset/refusal/image eviction or marginal wait>1ms. Stereo p50/p95/max4.17/9.02/98.37ms,
process CPU57.9%, sampled RSS1135.5MiB; checked finish and independent reopen pass.
All838 archived image feature/geometry/descriptor payloads match the pre-optimization
run when excluding odometry poses/CRC. This is not a whole-system speedup claim.

**Visual backend qualification still fails:** online PnP attempts0; sparse16-position
RMSE0.180m online versus0.313m committed.87scenes remain active; growth convergence
and useful real visual metric loops are unverified. Runtime odometry/loop results
vary across the two full runs, so their accuracy differences are not attributed to
stereo efficiency. [Full evidence](../../audit/2026-10-07-stereo-depth/README.md).
October1 below describes the older temporal configuration and earlier odometry.

## Odometry observation rejection isolation — 2026-10-04

User-requested frontend fix: `observe_voxelmap` commits state/covariance only on
acceptance. Rejected updates retain the current-time IMU prediction and skip map,
accepted trajectory/window and BA insertion. A pending ImuFactor spans rejected
intervals until the next accepted frame; reset clears it. Existing reset/resume
policy remains. Core and ROS production builds and four targeted tests pass.
The existing normal-diversity threshold is now a conservative admission gate,
not a complete registration-quality test; useful planar updates can be rejected.
The October7 combined stereo-profile runs above exercise this frontend, but are
not an isolated A/B validation of the rejection change. October1 acceptance applies
to the earlier frontend. [Contract and evidence](../../audit/2026-10-04-odometry-rejection/README.md).

## Hilti exp09 real-data acceptance — 2026-10-01

Two full 1x replays (446.64s) complete with 4,464 identical online odom poses,
zero observed Reset/refusal logs, checked finish and independent database reopen.
LIO versus cam0/1 tracking/retrieval: CPU 50.6% / 150.8% (one core=100%), sampled
RSS 972.8 / 1065.0 MiB, nodes 18 / 79, accepted loop constraints 4 / 23.
**Real-data backend geometry fails this acceptance:** sparse 16-position RMSE is
0.179m online in both runs, 0.209m for LIO committed layout and 1.108m for the
cam0/1 committed layout. Raw archived odom is also identical. Accepted constraints
on small query scenes show large disagreements with original relative motion.

Visual appearance recall occurs, but online PnP attempts remain zero. A read-only
prefix diagnostic has at most 17 metric matches among appearance-qualified pairs,
below the current 20-inlier prerequisite; it is not an exact asynchronous replay.
Two camera slots use temporal triangulation, not left/right disparity fusion.
No scene retires in either run; long-duration memory convergence, automatic real
cross-session continuation and useful visual metric loops remain unverified.
Prior synthetic successes do not override these real-data failures. Next work is
small-scene geometric acceptance and usable visual metric support, not A2/F1.
[Full report, limits and evidence](../../audit/2026-10-01-hilti-exp09/README.md).

## Active spatial targets after scene retirement — 2026-09-30

Retired c4 scene identities now leave the R-tree while retaining dense graph/bounds/
pose metadata. Pose deltas preserve inactive membership, and reconstruction restores
it from SceneState. Eligibility uses this materialized membership instead of a
SQLite read per spatial candidate. Default all-active formats remain unchanged.
Core29/29(58.78s), ROS3/3(1.72s) pass, including warm/cold scene-refresh and independent
process continuation. A 65,536-identity check has only one active R-tree target.
This removes retired overlap enumeration; graph/dense metadata and reconstruction
still scale with history. [Evidence](../../audit/2026-09-30-active-spatial/README.md).

## Controlled covered-scene refresh — 2026-09-30

Opt-in `map.scene_refresh=true` now selects an explicit geometry-only c4/schema3
format. Graph Node identities, original anchors, committed poses, pose witnesses
and factors remain. SceneState records whether a node still owns searchable heavy
payload or which newer associated observation replaced it. Schema1/c2 and schema2/c3
remain unchanged; no automatic migration of existing maps occurs.

After the new observation's normal commit, a scene-only transaction may retire a
fully covered old target's Gaussian/pyramid/image/visual-scene payload and advance
the committed revision. It requires a persisted type1 association, complete old
Gaussian support and distinct metric/descriptor support for every old indexed
visual landmark. Missing metric evidence, novel appearance, partial geometry or
budget exhaustion retains both scenes. New measured geometry is retained as-is in
its own original anchor. No averaged descriptors, LiDAR-derived visual depths,
static-scene prerequisite or graph contraction. This conservative policy does not
infer free space or remove arbitrary disappeared objects.
Retirement also requires measured and authoritative committed placements to agree
within 0.02m at new Gaussian means and old metric visual landmarks. A finite
disagreement retains both; it does not change loop acceptance or A2 diagnostics.

Retirement metadata/deletion/revision commit atomically. Memory cache invalidation
and descriptor retirement follow commit; spatial candidate eligibility excludes
retired payloads while graph/spatial summaries keep their identities. B1/B2/new-map
ready publication includes any successful scene-only revision. A failure after the
new observation commit still reports that committed outcome and fences reuse.
An uncertain later scene COMMIT takes precedence over a known earlier commit:
the last known revision remains available, but latest outcome stays Unknown.
Existing explicit recovery handles a failed writer; no sidecar bypass is added.

Controlled repeated tests preserve 13 graph nodes with one active payload after
12 independent-process attachments; main-file size remains 294,912 bytes between
the fourth and twelfth revisits. New-map and B2 full-repeat tests retain four active
payloads under existing temporal exclusion. These are finite synthetic scenarios,
not map-size convergence for arbitrary environments or Hilti performance. Graph
and spatial metadata still grow; novel/uncertain scenes remain retained.
Final core **29/29,58.29s**, ROS signal/publication/sync **3/3,3.66s** and
active-scene Gaussian exporter checks pass. These are controlled regressions.
[Design, limits and verification](../../audit/2026-09-30-scene-refresh/README.md).

## Revisit support and storage attribution — 2026-09-30

Accepted GICP profiling now measures bounded, bidirectional Gaussian support in
the target's original anchor. Unique pairs prevent duplicate density from inflating
coverage; covariance is rotated with the means. Incomplete work has no support
score. This increment itself was diagnostic evidence; the later opt-in retirement
policy above consumes its bounded kernel. Default runtime behavior is unchanged.

Read-only inspection of the frozen old0519 four-cycle workload finds 94 nodes,
355.16 MiB database size and 353.42 MiB of Gaussian/pyramid/image/visual-scene
BLOBs (99.86% of BLOB bytes). The historical file hash is unchanged. This is storage
attribution, not geometric validity or evidence of map-size convergence.
The subsequent scene-refresh increment above separates graph identity from scene
payload membership and checks visual preservation independently of LiDAR coverage.
Core **28/28,55.88s**, final ROS build and the accepted-GICP automatic continuation
fixture pass. Synthetic coverage/budget tests include 32,768-point input and dense
comparison exhaustion; no real-data accuracy or speed claim.
[Design, tests and results](../../audit/2026-09-30-revisit-coverage/README.md).

## Restricted automatic frontend continuation — 2026-09-30

Opt-in `map.automatic_attachment=true` connects the existing automatic historical
association API to SlamPipeline and the ROS configuration path. A separate worker
owns a bounded ordered startup queue. Local submaps can accumulate while an earlier
view is being verified. Later same-generation images may initialize the original
root, but that root's own Gaussian geometry must pass independent overlap/GICP;
its archived images are never replaced by the later view. A committed root enters
existing B2 and retained submaps transfer in original producer order.

Default startup retention is **8 submaps / 128 MiB**, with the existing 64 MiB
per-item limit. Unrecognized overflow fails explicitly before admitting the
extra item; after attachment the queue applies backpressure. Pending work is
**memory-only before association**, not a durable unattached-session journal.
Checked shutdown drains mapping/association/backend without another input; an
unresolved nonempty queue reports failure to finish, not successful persistence.
ROS status exposes pending bytes/count, high-water count, attempts and attachment,
and includes producer errors. Pending counters describe ownership, not commit
outcome after an exceptional write. Existing backend CommitOutcome stays authoritative.

**Scope:** the first frozen submap must geometrically overlap the historical map.
An arbitrary new prefix which reaches history only later is not supported by this
entry. Defaults/manual entry, storage schema, solver schedule and A1→B3/A2-F1
closure are unchanged. Controlled revisit updates/redundant-growth control and
real Hilti accuracy, end-to-end latency/CPU/RSS validation remain open.

Full core **27/27, 56.89s**, dedicated frontend/cross-process tests and ROS
signal/publication/synchronization **3/3, 3.60s** pass. These are synthetic ownership,
geometry and lifecycle checks, not a sensor-to-map Hilti run. Results, initial
corrections and reproducible commands are recorded in the
[frontend packet](../../audit/2026-09-30-auto-frontend/README.md).

## Restricted automatic association backend — 2026-09-30

An explicit backend entry now selects its own historical target/metric seed from
visual retrieval, checks independent LiDAR overlap/GICP and rejects conflicting
map placements among accepted returned candidates. Fresh-session retrieval includes
all history rather than applying the ordinary same-chain exclusion. An agreeing
selection reuses B1's existing solver/W/materialization and can enter B2 without
repeating BBS/GICP. Query producer sequence and persistent identity remain distinct.

Controlled tests include a one-node map, translated/rotated cloned-place ambiguity,
agreeing historical views, missing support, geometric rejection/retry, numeric
error, immediate readiness and independent-process automatic B1→B2→reopen.
Full core **26/26,56.63s** and final ROS build pass. Evidence and scope: [packet](../../audit/2026-09-30-auto-association/README.md).

The subsequently implemented opt-in frontend entry is described above. This
backend packet remains the evidence for the decision/commit layer alone.
Agreement gates (0.5m/5degrees) and bounded top_k recall are initial restricted-scene
rules, not universal distinctiveness or Hilti accuracy evidence. Controlled revisit
updates and redundant-growth control remain open. A1→B3/A2-F1 closure remains unchanged.

## Online visual PnP loop routing — 2026-09-30

Ordinary loop candidates now carry a metric visual seed when exact descriptor
matches pass the existing bounded PnP gates. That seed takes one fixed-pose LiDAR
overlap check and existing GICP acceptance; it triggers no BBS search, including
after geometric rejection. Missing support/no consensus retains the existing BBS
route. Fused spatial/visual targets keep one initializer, with metric vision first.

Observation archives now freeze the rectified-pixel projection in VOM-family v2;
legacy/v1 records remain readable. Historical PnP never substitutes current camera
intrinsics for missing archived projection. Original image/submap anchors define
the seed, not optimized poses. No SQL schema/identity or solver schedule change.

Full core regression **25/25, 62.67s**, ROS executable build and controlled live
PnP→score→GICP→persisted-factor tests pass. Cases cover asymmetric transforms,
visual-only recall despite odometry drift, fused candidates, rejected geometry,
missing evidence, changed runtime calibration and historical/transient queries.
Three independent processes preserve projection/metric evidence through real
B1/B2 continuation and reopen. [Design/results](../../audit/2026-09-30-live-pnp/README.md).

These are synthetic checks, not Hilti accuracy/latency or long-run memory evidence.
Manual B1 attachment retains its prior registration path. Automatic session
association, repeated-scene updates and redundant-growth control remain open;
A1→B3/A2-F1 closure is unchanged.

## Recoverable visual metric scene evidence — 2026-09-30

Visual triangulation evidence now survives ImageRecord storage through an explicit
VOM1/v1 BLOB encoding: original camera pose, local track ID, camera-frame point,
parallax, reprojection error and observation count. Legacy appearance-only images
keep their original encoding and remain readable. SQL schema/identity are unchanged;
old readers reject extended image lengths rather than silently dropping geometry.
No existing database is rewritten on open.

The scene builder transforms eligible visual points into the original submap anchor.
A2 reads and W preparation check metric presence/coordinates against the same recipe
rebuilt from image evidence. Descriptor order/deduplication/capacity remain unchanged;
a later duplicate does not overwrite the first retained row. This is persistence,
not geometric scene fusion or a dynamic-environment accuracy claim.

Full core regression: **24/24, 58.25s**. The independently executed write → destroy →
reopen → real B1/B2 continuation → reopen case also passes, covering eight metric
nodes, restored index fingerprints, graph links, unchanged read-only reopen and no
navigation tables. CRC-valid scene/image geometry disagreement is rejected. This
case is now permanently registered in CTest (separate final **1/1, 2.24s**), making
25 registered cases. ROS executable build passes. [Design/evidence](../../audit/2026-09-30-metric-archive/README.md).

At this earlier archive milestone, live PnP was absent; the integration above
supersedes that status.
The cross-process case supplies its attachment target/seed explicitly, so it does
not establish automatic association. Hilti accuracy/performance, controlled scene
updates and redundant-growth control remain open. Accepted A1→B3/A2-F1 solver and
lifecycle closure remains unchanged.

## Explicit Hesai point-time input — 2026-09-30

TOML/ROS now select single `hesai` (numeric 2) explicitly. FLOAT64 absolute-second
point timestamps supply min/max over all raw points before filtering, and the
full end is passed to the existing core synchronizer. The stateless all-point
inspection is shared with dual Airy; no new queue or persistence owner. Wrong or
constant timestamps/oversized spans are rejected. Launch requires explicit
algorithm and sensor files rather than a Mid360 calibration default.

ROS build, single/dual decoding and moving-rig tests, TOML selector checks,
launch checks and live ROS mode/callback/exit smoke pass. New fixtures cover
unsorted epoch times, both endians, PCL-like field padding and both bounds removed
by decimation/blind filtering. These are synthetic input checks, not actual
Hilti compatibility, odometry or performance evidence. See [the packet](../../audit/2026-09-30-hesai-input/README.md).

Real message layout/epochs, calibrated profile, full replay and supplied-truth
evaluation remain pending user data. Existing Livox/Airy single timing remains.
Metric archive/PnP, scene updates and automatic association remain open;
A1→B3/A2-F1 closure is unchanged.

## Equidistant visual input and ROS1 conversion preflight — 2026-09-30

Explicit `visual_loop.left/right.distortion_model` now supports `radtan` (default,
empty/five coefficients) and `equidistant` (four coefficients). Shared stateless
conversions drive tracker rays, raw anchor projection and the legacy extractor.
Invalid inverse results cannot become geometry; PnP consumes already rectified
pixels without applying lens distortion again. No odometry or scene format change.
Full core **23/23 (58.35s)** and ROS executable build pass.

New synthetic checks cover the official Hilti cam0 numeric calibration, off-axis
projection/inversion, invalid models/coefficients, both TOML slots, asymmetric
triangulation, raw fisheye image flow and descriptor row alignment. The4mplane
fixture produces447metric rows and414retained points from30images/2descriptions;
depth-error median/p95/max=.012/.130/.257m. Exact-ray tests have separate tighter
bounds. These are not real-data accuracy results. Existing positive-z and
rectified-image bounds remain, so full wide-angle field-of-view retention is not
established. [Implementation/evidence](../../audit/2026-09-30-equidistant/README.md).

A six-message ROS1→ROS2 synthetic conversion preserves timestamps, image/point
padding, IMU values and both-endian FLOAT64 point timestamps, except unsupported
ROS1 Header.seq. ROS2 Humble reads the converted bag. Actual Hilti message layout,
scan epochs, large-bag memory cost, replay and truth evaluation remain unverified.
The user will supply data/truth when ready; old0519remains engineering-only.
Metric archive/PnP, controlled repeated-scene updates and automatic cross-session
association remain open. A1→B3/A2-F1 closure stays unchanged.

## Dataset transition and intermediate visual geometry — 2026-09-30

**User decision:** the current 0519 sensor layout is unsuitable for algorithm
assessment. Preserve old runs as engineering workload evidence only; stop tuning
geometry/appearance against them. Hilti 2022 and its truth are being downloaded
by the user. No further old-calibration investigation is required. Continue
independent implementation, synthetic tests and input preparation. See
[Hilti acceptance transition](HILTI2022_ACCEPTANCE.md).

A descriptor-cadence bug is fixed: retained anchors now receive valid same-track
triangulation on every posed observation, even when no descriptors are computed.
The new synthetic test fails on the old code and passes after the fix, with depth
and projection verified in the retained camera frame. All existing geometry,
identity, gap and budget gates remain. Core22/22(52.87s) and ROS build pass.

The already-running old-bag engineering regression completes with2,008odom/grids,
0Reset/refusal,43/43submaps, checked finish and unchanged independent DB reopen.
Mean CPU129.95%, peak RSS1,304.06MiB. Visual candidates change1→0 compared with the
previous density run; improved runtime depth row retention does **not** establish
better retrieval or geometric quality. Anchor affiliation/selection can change.
Fixed-boundary diagnostic candidates have5/2metric pairs, still insufficient for
PnP. Tracking remains opt-in; production metric persistence/PnP remain absent.
[Fix, negative results and raw evidence](../../audit/2026-09-30-visual-depth/README.md).

Official Hilti cam0/1 calibration uses equidistant distortion, currently unsupported
by native visual processing. Two small official YAMLs and a transform inspection
are saved; no runnable Hilti profile, bag replay or truth result exists yet. Camera
model, timestamp and ROS1 input adaptation are the next independent work. Scene
updates, bounded redundant growth and automatic cross-session association remain
open. Accepted A1→B3/A2-F1 closure is unchanged.

## Coverage-first visual recall increment — 2026-09-30

The opt-in live tracker now spends unused cell capacity in textured regions after
its ordinary spatial coverage pass, with5px detection spacing. Total600points,
threeanchors,16,384association visits and all match/acceptance gates are unchanged.
Single-pool identity, mature coverage and independent LiDAR verification remain.
Full core22/22(52.66s), ROS build and same-input1×real-bag acceptance pass.

Compared with the previous tracker: visual candidates0→1, completed submaps43→42,
geometric constraints5→5;2,008odom/grids each,0Reset/refusal. Mean CPU126.53→128.15%
(one logical core=100%); peak RSS1,274.66→1,307.63MiB, database81.57→83.25MiB.
Tracking p50/p95/p99=4.81/9.67/12.26ms. These are single runs; no speedup or memory
improvement claim. Legacy independent ORB also hadonevisual candidate. New boundaries
are38visual/3resource/1tail;10images still lie beyond the final pose watermark.
Checked finish, unchanged independent DB reopen and all local-grid coordinates pass.

A read-only real-record counterfactual shifts query translation1km, retains recorded
orientation and removes all spatial candidates. Visual query40→target1 survives;
BBS/GICP accept (overlap.6505,16,530inliers). This isonecandidate and doesnot establish
arbitrary relocalization, physical truth or automatic attachment; no graph write.

Controlled fixed-boundary offline matching improves to28/23pairs, but only2/0have
eligible target visual geometry. PnP doesnot solve. **Production is still appearance
retrieval with BBS/GICP, not metric visual initialization.** Usable visual PnP seeds
are intended to bypass BBS; BBS remains for missing usable visual initial poses.
Geometric rejection must not be bypassed by a second initializer. Metric archive/
PnP, controlled repeated-scene updates and automatic cross-session association
remain open. A2/F1 stays closed. [Evidence and reproduction](../../audit/2026-09-30-visual-recall/README.md).

## Live visual submap producer — 2026-09-30

This supersedes the earlier utility-only tracker status below. The explicit
`visual_loop.tracking_enabled=true` path now runs the single-pool tracker in the
mapping worker, interpolates original LIO image-time camera poses and uses visual
novelty to close actual submaps. Descriptor cadence is separate from tracking;
local anchors, tracks and pending image count/pixel bytes are bounded. Resource
cuts remain and distance is used after a visual outage. Pose-less frames remain
2D-only. The final input-copy path also releases borrowed raw-image owners so
unaccounted original messages cannot evade the pending-pixel budget.

On the same201s recorded input, same binary,1× and local rolling grid enabled:

| Measurement | Legacy image path | Live tracker path |
|---|---:|---:|
| Odometry / local grids received | 2,008 / 2,008 | 2,008 / 2,008 |
| Reset / refusal logs | 0 / 0 | 0 / 0 |
| Accepted and completed submaps | 21 | 43 |
| Mean CPU (100%=one logical core) | 81.16% | 126.53% |
| Sampled peak RSS | 1,340.63MiB | 1,274.66MiB |
| Database size | 79.61MiB | 81.57MiB |
| Accepted geometric loop constraints | 1 | 5 |
| Visual retrieval candidates | 1 | 0 |

New boundaries:40visual,2resource,1tail. The live tracker processes2,000images,
1,997with pose brackets,301descriptions; tracking p50/p95/p99=4.84/9.60/11.97ms.
Image queue eviction and flow-gap resets are0. Ten remaining images lie beyond
the final marginalized-pose watermark; this is not full frontend-tail completion.
127exported representatives carry7,436runtime metric rows, **not persisted 3D**.
Both runs finish successfully, all2,008grid origins/timestamps match odometry,
and independent schema2reopen preserves the DB bytes with no navigation tables.
No independent pose/loop truth or repeated-route growth bound is established.
The legacy candidate count above corrects the initial manually transcribed0;
[raw-log correction](../../audit/2026-09-30-live-visual/ERRATUM.md).

The integration passes22/22core tests and ROS build. A final raw-image owner
release, which does not affect this compressed-image input, separately passes
3/3targeted final regressions and ROS rebuild; the packet preserves both binary
versions. The new path remains opt-in: CPU rises, node count doubles, and visual
retrieval still finds no candidates on this recording. Online visual PnP/metric
storage, reliable revisit matching, controlled historical scene updates and
automatic cross-session association remain unfinished. A2/F1 remains closed.
[Design, raw evidence, source snapshots and reproduction](../../audit/2026-09-30-live-visual/README.md).

## New continuous-mapping product work — 2026-09-29

This is user-requested work after the accepted lifecycle closure below; it does
not reopen A2/F1. Design and evidence are in [CONTINUOUS_MAPPING](CONTINUOUS_MAPPING.md)
and the [implementation/acceptance packet](../../audit/2026-09-29-continuous-mapping/README.md).

- Canonical application configs now use geometry-only schema 2/c3, with no
  FlatGrid/NaviTrajectory tables. Explicit legacy schema 1/c2 support remains;
  historical files are not migrated. Online navigation spline construction is
  removed; original odometry is retained.
- A transient, bounded local rolling occupancy window is implemented on the
  existing ROS output worker, in odom coordinates. Default 20 m square / 0.1 m
  cells / 5 s expiry; flat-ground height classification, dual-sensor endpoint-only
  limitation. It has no database or committed map-revision membership.
- New-map admission no longer takes the consumer's lifecycle lock; one waiting
  submap plus active processing preserves bounded backpressure. Core 20/20 and
  ROS 3/3 regressions passed; the final grid-only immediate-hit rule additionally
  passed its targeted regression.
- Same-input four-cycle 2× comparison: 7,268 → 8,036 received odometry; 1 → 0 Reset;
  maximum output interval 4.695 → 0.201 s. Refusal-log lower bound 700 → 1 (throttled
  logs, not exact final counts). Peak RSS 3,088.64 → 3,216.04 MiB; **no memory
  improvement claim**. Single-run evidence, artificial route seams, no truth pose.
  All 94 accepted submaps completed, 71 geometric loop constraints persisted;
  checked exit and independent reopen of the geometry DB passed byte-preserving.
  This is not evidence that all those loop constraints are physically correct.

Local-window real-bag 1× acceptance passed: 2,008 odometry and 2,008 grids,
no refusal/Reset logs; every grid matched the same-time odometry and expected
sensor-centered window origin. Fixed 200×200 cells, peak RSS 1,332.30 MiB,
mean CPU 80.72% (100%=one logical core). All 21 submaps completed; the schema-2
database reopened unchanged with no navigation tables. This verifies publication
and coordinates, not general obstacle accuracy or terrain handling.
Whole-process bounded memory, repeated-scene
replacement and automatic cross-session association are not delivered by these
changes. Visual retrieval is implemented; live metric visual depth/PnP is **not**.
The user-defined next visual direction is lightweight pyramidal optical flow and
visual triangulation for submap triggering and coarse loop initialization only;
no visual odometry and no LiDAR-feature depth association. See
[the corrected design](VISUAL_METRIC_INITIALIZATION.md). Independent LiDAR
verification still decides final loop acceptance.

A bounded single-pool optical-flow / three-view triangulation core utility is now
implemented and exercised independently of the live pipeline. On an offline
2010-image recording, 181 of 245 selected observations contain some accepted
triangulation; no depth truth or real PnP/loop accuracy is established. Single-thread
per-input p50/p95 is 10.92/15.80 ms on the recorded probe. The raw selector still
proposes 159 boundaries and is **not enabled** in production. The user's next
requirement is bounded local keyframe affiliation and visual-landmark projection,
with grid replenishment sharing coverage counters and no historical covisibility
traversal. Metric persistence, live integration and PnP remain absent. See the
[utility/probe evidence](../../audit/2026-09-29-visual-metric/README.md).

The subsequent [bounded-affiliation utility](../../audit/2026-09-29-visual-affiliation/README.md)
retains up to three local visual anchors, recovers associations by visual-landmark
projection and descriptors, and separates internal representative observations
from boundary requests. Synthetic blackout recovery reattaches 478 feature
identities; wrong-pose and exhausted-budget cases reject. On the same offline
recording, requests fall from159 to43, p50/p95 is11.38/16.61 ms, and anchors remain
bounded. Only26 frames contain actual reacquisition, so the request reduction is
not evidence of broadly validated reattachment quality. Targeted CTest and access/
UB sanitizer checks pass. This remains outside the live pipeline; metric storage,
PnP, actual submap/loop behavior and the full continuous-mapping goal remain open.

The next [visual PnP utility/probe](../../audit/2026-09-29-visual-pnp/README.md)
adds detached export of those enriched local anchors and a bounded candidate-pair
PnP seed function. It reuses exact FeatureMap/FeatureBlock match indices, validates
image-time camera/submap transform direction and gates refined support. Targeted
tracker/PnP tests pass 2/2; the PnP ASan/UBSan access check passes with leak detection
disabled. There is **no live PnP or metric persistence integration**. In the same
2010-image probe, 43 exported representative scenes contain 27,944 points, 6,063
with visual geometry. Candidates at least 15 seconds after target closure have
at most 16 descriptor matches and at most 6 metric matches; none reaches the
20-match entry gate, so there are zero PnP attempts, not real-data PnP success.
These are probe camera anchors, not production submaps. Revisit truth and LiDAR
verification remain absent. The existing global descriptor index also retains
history-dependent work; bounded local affiliation does not solve that cost.

The [follow-up image diagnosis](../../audit/2026-09-29-visual-revisit/README.md)
corrects an audit-driver export omission: a missing pose on the first retained
image no longer discards later pose-bearing representatives. Corrected44scenes
contain28,281points/6,092metric points. Across1,129top-five historical candidate
pairs, exhaustive diagnostic matching reaches at most17matches/8metric matches,
so no PnP solve occurs. Global index loss is not established as the main cause.

Following the user's narrow-FOV/foreground concern, current live loop candidate
fusion now preserves an existing spatial seed when appearance retrieval finds
the same target. Empty visual results already preserve spatial candidates.
There remains exactly one BBS attempt per target; the intermediate dual-search
proposal was withdrawn. Future usable PnP seeds should proceed directly to LiDAR
verification, with BBS supplying unavailable visual initial poses. No live PnP
integration, global pose-independent geometric retrieval or automatic association
is claimed. Final single-search core CTest22/22passes, including real CUDA
registration and serialized factor checks; BBS/GICP rejection has exactly one
BBS attempt and no same-target retry. The ROS `sapphire` executable builds
successfully; live metric visual initialization remains unverified.
[Counterexample and final validation packet](../../audit/2026-09-30-loop-candidate-independence/README.md).

## Accepted lifecycle closure and research handoff — 2026-09-29

**A1→A2→W→B1→B2→B3 is a completed, independently accepted engineering stage.**
Source and accepted closure audits govern this status; acceptance remains bounded
by their documented support domains. **B3 implementation accepted with documented
limitations.** Its R1/R2/R3 findings are closed.

| Phase | Completed responsibility | Status / closure evidence |
|---|---|---|
| A1 | Safe historical open | **ACCEPTED**; [accepted A1 contract](PHASE1_DESIGN.md), retained by [A2 closure](../../audit/2026-09-27-a21-independent/README.md). |
| A2 | Historical runtime reconstruction | **ACCEPTED**; [A2.1 closure](../../audit/2026-09-27-a21-independent/README.md). |
| W | Writable atomic finalized commit | **ACCEPTED**; [W.1 closure](../../audit/2026-09-27-w1-acceptance/README.md). |
| B1 | Fresh-session attachment | **ACCEPTED**; [B1.1 closure](../../audit/2026-09-27-b11-acceptance/README.md), [teardown closure](../../audit/2026-09-28-teardown-acceptance/README.md). |
| B2 | Same-session continuation | **ACCEPTED**; [B2-R1 closure](../../audit/2026-09-28-b2-r1-closure-review/README.md). |
| B3 | Publication / checked drain | **ACCEPTED**; [final independent B3 closure](../../audit/2026-09-28-b3-independent-closure-review/README.md). |

The completed boundary is:

```text
persistent map authority
    ↓
runtime reconstruction / readiness
    ↓
live continuation
    ↓
attributed publication
    ↓
checked drain / close
```

These boundaries remain valid independently of future registration, retrieval,
candidate policy, robust optimization or descriptor-selection improvements.
No B4 follows automatically. Future work is driven by concrete research/product
questions, not continuation of migration phase numbering.

### System invariants retained

The [independent algorithm/system audit](../../audit/2026-09-29-algorithm-system-invariant-audit/README.md)
supports preserving the underlying invariants, without freezing every current
implementation, capacity or algorithm:

- Safe historical open and file identity; writable ownership through complete close.
- Atomic persistent outcomes and explicit `NotCommitted / Committed / Unknown`;
  uncertain acknowledgement cannot justify blind retry.
- DB-authoritative / runtime-disposable separation; committed C is distinct from
  runtime-ready Y, and a runtime failure cannot undo a committed fact.
- Sticky failure and fenced service after uncertain mutation; local preparation
  before mutation remains retryable.
- Bounded admission/backpressure and accounting for accepted work.
- Fresh-domain attachment semantics; original odometry anchor is distinct from
  committed pose, and restart does not restore a live producer domain.
- Publication attribution and separate correction/navigation frontiers Pc/Pg;
  local publication completion does not establish subscriber delivery.
- Checked drain/shutdown ordering: join upstream work, drain backend, settle
  required final publication with transport alive, join output, checked-close storage.

### F1 — numerical-support amendment accepted with documented limitations; closed

The [causal investigation](../../audit/2026-09-29-f1-numerical-reopen-research/README.md)
reproduced the retained discrepancy as further nonlinear movement of the same
inconsistent cycle under A2's current two-update rebuild; float loss and changed
model interpretation are not necessary explanations for this witness. The
[beyond-envelope experiment](../../audit/2026-09-29-a2-beyond-envelope-continuation-experiment/README.md)
opened it at approximately **0.073646 m pose / 0.094173 m residual translation**
discrepancy, preserved R5 committed presentation, and completed real B1/R6,
B2/R7, checked close and independent reopen without writer settlement or extra
optimization. The experiment used an audit-local diagnostic rule and one ordinary
control; it is not a production amendment or a universal continuation guarantee.

The [documentation-only A2 amendment](../../audit/2026-09-29-a2-contract-amendment-design/README.md)
reclassifies the unchanged four similarity references as reproduction diagnostics.
All storage, representation, topology, key/type, exact model/prior, finite
solver/residual/objective, product and identity failures remain hard failures.
Committed X^R remains authoritative for R; runtime X* is disposable. B1/B2
validate actual operations, reconcile every serialized historical pose change
independently of affected hints, and use W plus committed-owner materialization
before readiness/correction. No revision-jump bound or A1→B3 algorithm change.

**Final status: ACCEPTED WITH DOCUMENTED LIMITATIONS / CLOSED.** The
[bounded implementation and evidence](../../audit/2026-09-29-a2-contract-amendment-implementation/README.md)
passed [fresh independent implementation review](../../audit/2026-09-29-a2-contract-amendment-independent-implementation-review/README.md).
The [final closure](../../audit/2026-09-29-a2-contract-amendment-closure/README.md)
updates status, the stale verification heading and warning units only; it creates
no new contract, algorithm, research phase or further review requirement.
Production now retains finite similarity exceedance as an attributed reproduction
diagnostic. Explicit finite arithmetic and six-dimensional residual checks remain
hard failures. A2 additionally compares factor type and robust Scalar/Block scheme
(the vendored Huber equality compares k only). The fixed original F1 and control
pass production open, real B1/B2 continuation and new-process reopen; all-key and
committed-product oracles and the regression scope are recorded in the packet.

The [settlement characterization](../../audit/2026-09-29-f1-settlement-characterization/README.md)
and [independent remedy review](../../audit/2026-09-29-f1-independent-design-review/README.md)
remain correct evidence for the previous reader policy. Under the amended
semantics the full-graph probe is useful for research/diagnosis, not a mandatory
managed B1/B2 production gate. Its product necessity is superseded; implement no
settlement or extra updates from the prior approval. Keep all old evidence and
numerical references. Historical rejection reports remain period-correct.

This change does not guarantee that arbitrary finite graphs continue, arbitrary
divergence is safe, all B1/B2 operations succeed, accepted loops are physically
correct, large revision jumps are controller-safe, or beyond-envelope B2 new-loop
behavior is universally supported. Indefinite continuation is not guaranteed.
These are documented limitations, not unfinished tasks. Operation-specific failure,
aliasing/Policy-C quality limitations and all other retained limits remain.

Final disposition: **A1→B3 lifecycle ACCEPTED / CLOSED**; **A2 numerical-support
contract amendment ACCEPTED WITH DOCUMENTED LIMITATIONS / CLOSED**; **F1 mandatory
writer settlement NOT ADOPTED**. The full-graph settlement probe remains research /
diagnostic evidence only. F2/F3 remain future compatibility debt only; no next
engineering task starts automatically.

### F2 / F3 — material future compatibility debt only

**F2: current ordinary-loop temporal eligibility/cardinality policy is partly
promoted into persisted graph admissibility.** Fixed same-chain K=3, one ordinary
type-1 per query and shared retrieval/W/A2 validation semantics are accepted and
internally consistent native writer/reader behavior. **Future compatibility
cleanup only when a justified temporal/cardinality algorithm change actually
exists.** This is a trigger condition, not a current task: no configurable K,
schema migration, weakened validators or multiple-loop implementation now.

**F3: current VisualScene historical validity includes the accepted producer's
exact scene-construction recipe, not only factual descriptor/provenance validity.**
Current scenes remain valid and current validation is not declared defective.
**Future compatibility cleanup only when a justified scene/descriptor-selection
policy changes.** No visual geometry, descriptor selection, Faiss or scene-schema
redesign follows from closure. Only F2/F3 are marked material future compatibility
debt from this audit; neither warrants immediate production cleanup.

### Accepted current couplings and other retained limitations

Keep accepted current scope: SQL Node ID / backend key / append order coupling;
type-0 roots; source sequence; B2 source-sequence/revision arithmetic for its
successful contiguous prefix; graph revision; volatile generation g; map UUID U
and publication incarnation E; visual prefix representation; native BBS pyramid
representation/preflight; and deterministic occupancy fusion replay. These create
no cleanup tasks merely because a more abstract alternative is imaginable.

Other retained known limitations remain separate from F1 and F2/F3: aliased
geometry evidence and unestablished Policy-C real-data quality; unresolved
third-party/OpenMP race-tool diagnostics and incomplete whole-program coverage;
transport's eventual-return assumption and local completion rather than remote
delivery; LocalGrid coordinate/terrain limitations; intentional optional
visualization loss; and bounded synthetic performance evidence rather than a
deployment throughput guarantee. Closure does not solve or reclassify these.

### Negative guidance and evidence scope

This closure does **not** recommend immediate F2/F3 production cleanup,
configurable K, Place/Region infrastructure, persistent session IDs, a schema/version
framework without a real format change, optimizer checkpoints, per-head full
reconstruction, wider A2 tolerance without evidence, rebuilding historical
scenes/LocalGrids using current policies, or another middleware qualification
campaign. Accepted phase decisions remain closed.

This pass changes documentation only. Prior B3 independent results (core 18/18,
CUDA-hidden A2/W 2/2, ROS 3/3) remain prior evidence, not newly executed tests.
[Closure artifact, exact patch and preservation](../../audit/2026-09-29-a1-b3-phase-closure/README.md).

## Historical implementation entries and pre-migration snapshot — superseded status

Everything below retains period-specific evidence. Earlier review-pending,
correction-waiting, unimplemented and stop-before-phase statements, including
the old next-phase gates and broken-resume/publication rows, are **superseded by
the accepted closure above** and are not current blockers or work authorization.
Historical F1–F5 labels belong to the 2026-09-26 audit; they must not be confused
with the 2026-09-29 algorithm/system audit's F1/F2/F3 used above. Retained grid,
terrain and verification limits continue to apply.

## B3 — revision-consistent publication and checked drain, 2026-09-28

B3 is implemented against the independently accepted design. Existing backend,
pipeline and ROS output owners now separate committed C, runtime-ready Y and
volatile correction/navigation Pc/Pg. Coherent capture attributes original
anchors, correction and grid to one ready source. Legacy mutation/visual owner
completion is fenced before readiness. Four fixed optional slots and bounded
required demand replace the closure queue; dense/producer-copy limits precede
allocation. Output errors do not change map truth.

Checked shutdown joins upstream work, drains accepted backend work, freezes F/S,
settles required local publication while ROS is alive, joins output and then
checked-closes storage. SIGINT/SIGTERM only request this path through the existing
executor. Reset waits for active immutable output without holding owner locks.
Wire identities and Pc/Pg are not persisted; schema/user_version, Policy C K=3,
registration, optimizer, W and attachment/continuation algorithms are preserved.

Frozen Release core CTest **18/18 (46.98 s)** and CUDA-hidden A2/W **2/2
(17.61 s)** pass. The B3 barrier/process test passes **10 consecutive runs**.
Installed `rmw_zenoh_cpp`, using an isolated owned router/domain, passes ROS
**3/3 (3.35 s)**, including actual SIGINT/SIGTERM/finish and explicit-resume
A→B1→B2→final-F→independent CUDA-hidden A2. F=4/S=42/Pc=Pg=4 and checked
close are retained with serialized wire witnesses. Earlier Fast DDS checks are
supplementary. Payload maximum encoding is 16,000,448 bytes for a revisioned
16-million-cell grid. Timings characterize synthetic fixtures, not deployed
throughput or a transport deadline.

[Full A–R report, phase-only patch, hashes, process witnesses and performance](/home/user/code/sapphire_git/audit/2026-09-28-b3-implementation/README.md).
**B3 is ready for independent implementation review, not self-approved.** This
entry supersedes older stop-before-B3/state statements below while preserving
their historical evidence. The A2 aliased-geometry limitation, possible external
transport non-return, intentional optional visualization loss, and absence of
whole-program race/real-data quality guarantees remain explicit.

## B2 — resumed same-session continuation, 2026-09-28

B2 is implemented against the independently accepted design with native fixed
Policy C K=3 and schema `user_version=1`. Explicit same-producer B1→B2 handoff
now feeds a bounded worker, transient retrieval, one finalized W commit per node,
exact all-key float pose delta, incremental pose/spatial/visual/occupancy owners,
and coherent newest-node correction/readiness. Ordinary full runtime rebuild
count is zero. Sticky mutation failures retain W outcomes and original causes;
pre-mutation heads support explicit retry. Finish/reset/failure wake waiters and
account the accepted tail. New-map shutdown retains its final short-builder flush.

Final Release CTest **17/17 (46.46 s)** passes, including the expanded B2
cross-process A2→fresh B1→B2→independent A2 oracle and new-map tail regression.
CUDA-hidden W/A2 **2/2 (17.73 s)** and B2 structural fixtures passed; ROS executable
build and synchronization test **1/1** passed. Synthetic median ordinary head
service at histories6/100/512/1024 is 2.725/3.011/9.464/17.989 ms. Flat→IVF training
adds a ~1.6 s spike; a 512-node evidence-heavy fixture is ~253 ms/head, dominated
by unchanged W pre-BEGIN validation. These are not deployed throughput claims.

Retained limit: an aliased repeated-geometry restart fixture accepted an extra
loop and then exceeded A2's existing numerical support envelope on reopening.
Its failure log and reproduction patch are retained; no threshold or optimizer
was changed. Supported fixtures pass the semantic oracle; universal reopenability
and real-data Policy-C quality are not claimed.

[Full A–N implementation report, phase patch, preservation proof and logs](/home/user/code/sapphire_git/audit/2026-09-28-b2-implementation/README.md).
**Ready for independent B2 implementation review within the retained support
envelope. B3 has not started.** This supersedes earlier stop-before-B2 gates;
the historical evidence below remains unchanged.

## Narrow teardown correction after accepted B1, 2026-09-28

A1/A2/W/B1 are independently accepted. E20 fixes the inherited accidental
termination when checked `finish()` rethrows a stored non-standard exception
through `PoseGraphBackend::Impl::~Impl()`. The sole production change adds a
final catch-all at that destructor boundary. Explicit finish, sticky failure,
B1 preparation/ISAM admission, worker entry code and W ownership remain unchanged.
SQLite must still close completely before the owner fd and descriptor admission
are released; the existing deliberate invariant fail-stop remains.

The retained reviewer probe and new entry-boundary regression both reproduced
SIGABRT (exit 134) before the production fix. Seven isolated cases now cover the
original integer failure, post-solver and post-COMMIT failures, direct destruction,
runtime_error, bad_alloc, and checked storage-close failure with both owned
workers joined. They verify checked failure reporting, fenced service, safe
close order, same-process admission reacquisition and independent DB reopening.
Existing B1 retryable/sticky and W BUSY/admission assertions remain intact.

Release build and full CTest **16/16 (33.18 s)** pass; device-hidden W/A2
**2/2 (17.43 s)** pass. The final test-only capture-lifetime adjustment is checked
by a separate B1 rerun recorded in the report. Production changes are five added
lines in one destructor; the focused test, long-term AGENTS invariant and these
state documents are the only other changed repository files in this pass.
No new lifecycle architecture or persistence-format behavior is introduced.
Synthetic tests do not constitute real-data validation.

[Full 14-item correction report and red/green evidence](/home/user/code/sapphire_git/audit/2026-09-28-teardown-correction/README.md).
**Ready for independent review of this local correction. Stop before B2/B3.**
This supersedes the earlier readiness gates while retaining their evidence.

## B1.1 — pre-mutation retry boundary correction, 2026-09-27

Independent B1 review accepted the architecture and major contracts but found
B1-R1: local BetweenFactor/key-vector and initial Values allocations were caught
by unconditional sticky-failure handlers before any ISAM mutation. The narrow
fix keeps an attempt-local flag false throughout preparation and sets it
immediately before the first `updateIsam()` call. Earlier standard errors retain
the existing retryable result with error detail; failures inside that call and
afterward preserve accepted sticky failure and W commit outcomes. No solver
clone/rollback/checkpoint or geometry/storage/session redesign was added.

E19: Both new regressions first failed against the old handler: a 16-byte factor
key-vector allocation and a 112-byte Values Pose3 clone, each with zero ISAM
updates and an incorrectly failed backend. After correction, both prove exact
solver factors/estimates and DB/directory preservation, healthy historical
queries, unavailable correction and successful explicit retry on the same owner.
A third allocation failure inside `ISAM2::addVariables` during its first update
remains sticky and requires reconstruction. All existing B1 failure/geometry/
reopen assertions are retained.

Release build and full CTest **16/16 (31.10 s)** pass, including B1 **10.54 s**;
device-hidden W/A2 **2/2 (17.38 s)** pass. Only pose_graph.cpp, its API comment,
the existing fresh-attachment test and these state documents changed. Other
source/tests and CMake remain byte-identical; no accepted B1 algorithms or
persistent semantics changed. Synthetic verification is not real-data validation.

[Full 14-item correction report, red/green evidence and phase diff](/home/user/code/sapphire_git/audit/2026-09-27-b11-correction/README.md).
**B1.1 is ready for narrow independent re-review of B1-R1. Stop before B2/B3.**
This supersedes the previous readiness gate without removing historical evidence.

## B1 — explicit fresh-session attachment, 2026-09-27

A1/A2/W are independently accepted. B1 adds one explicit backend operation:
frozen SubmapFrame plus finalized LocalGrid, selected historical SQL target and
metric T_H_Q seed → unchanged BBS/GICP → bounded tentative ISAM update → W
promotion/atomic root commit → reconstruction of existing committed runtime
owners → active correction from serialized Q pose and its fresh odometry anchor.
All proposed poses are compared with persisted committed poses, including A2
solver differences. SQL root N+1 has one type-1 Q→H edge and no type-0 predecessor
or extra prior. Pre-solver rejection is nonmutating/retryable; later errors are
sticky and retain W commit-outcome semantics. Reset invalidates correction,
fences/finishes the owner and requires reconstruction plus another explicit B1.

E18: Release build and full CTest **16/16 (31.12 s)** pass, with real GPU
registration, exact rejected file/row evidence, 11 speculative/commit failure
boundaries, lifecycle races, and independent A2/recovery processes. B1 takes
**10.39 s** including all cases and a measured larger fixture. Device-hidden
historical/W tests also pass **2/2 (17.69 s)** inside the sandbox. Committed runtime
rebuilding measured **0.604 ms at 7 nodes** and **1568.36 ms at 129 nodes**,
including existing visual training; these are synthetic single-run timings,
not a real-data or large-map performance claim. MapDatabase, registration/grid
algorithms, and preexisting tests remain byte-identical to the accepted baseline.

The API consumes already finalized LocalGrid evidence; no ground/CAPE/raycast
pass runs in B1. A2's borrowed occupancy diagnostic becomes a lifecycle-locked
copy to avoid dangling across attachment. Normal NavigationGrid remains shared
and immutable. No publication callback, automatic frontend resume transport,
repeated continuation, schema change, migration, completion marker, persistent
session or correction was added. `user_version == 1` remains a format guard;
loop-decision status remains Unavailable. The new-map/pipeline admission and
ordinary addFrame continuation gate remain unchanged.

Full requested 20-item report, exact phase diff, test logs and preserved baseline:
[B1 implementation report](/home/user/code/sapphire_git/audit/2026-09-27-b1-implementation/README.md).
**B1 is ready for independent implementation review. Stop before B2/B3.**
This supersedes the earlier phase gates below without removing their evidence.

## W.1 — finalized-input domain correction, 2026-09-27

Independent W review accepted the architecture/lifecycle/transaction contracts
but reproduced W-R1 (unrepresentable committed occupancy geometry) and W-R2
(NaN first navigation knot). The narrow correction changes only MapDatabase
production validation: A2 and W share the same cell/aggregate representability
helpers; W checks persisted grids under proposed historical poses, unchanged
historical extents and the new grid/pose before BEGIN. Runtime occupancy
resolution is retained from the existing configuration. Every finalized spline
knot must explicitly be finite; the trajectory helper remains unchanged.

E17: Release build and full CTest **15/15 (22.10 s)** passed; final device-hidden
W/historical tests **2/2 (17.21 s)** passed. Sixteen invalid-input cases prove zero
BEGIN/writes and exact prior rows/blobs/revision/files, with independent retained
A2/recovery/backend reconstruction. Successful populated, empty-spline,
multi-chain and near-int-boundary cases also pass independent reopening. The
reviewer's rebuilt original probe now rejects all three counterexamples while
both retained-state opening modes succeed. Details and exact changes are in the
[W.1 correction report](/home/user/code/sapphire_git/audit/2026-09-27-w1-correction/README.md).

**W.1 is ready for narrow independent re-review of W-R1/W-R2.** This supersedes
the previous W readiness statement without changing the accepted W architecture,
Route-S SQL transaction, ownership, graph topology or current format.
**Stop before B1/B2/B3.**

## W — exclusive storage transition and finalized append, 2026-09-27

A1/A2 are accepted. The settled post-A2 roadmap authorizes W only, before B1
attachment, B2 continuation and B3 publication/drain. W strengthens the existing
MapDatabase, Memory, PoseGraphBackend and pipeline owners with process-wide raw
descriptor admission, checked historical-to-writable handoff, actual SQLite inode
proof, explicit storage recovery/finish, atomic finalized append, multi-chain
validation, sticky failure and old-owner-before-replacement sequencing.

The DB remains authoritative. The current format guard remains SQLite
`user_version == 1`, unrelated to software version; no schema change, migration,
Link type 2, session identity, completion marker or optimizer serialization was
introduced. Loop-decision status remains Unavailable. Historical-only mmap /
deserialize opening still rejects sidecars; explicit recovery delegates journal /
WAL handling to SQLite after proving the actual main inode. Cold rollback
journals that SQLite ignores are preserved, so historical-only opening can still
reject those maps after recovery inspection. No manual sidecar deletion occurs.

E16: Release core build and full CTest **15/15 (11.61 s)** passed, including both
GPU tests. Final device-hidden W/historical tests also passed inside the sandbox
**2/2 (6.16 s)**. Detailed ownership, API/transaction contents,
900 denied-open lock regressions, deterministic A–E races, identity substitutions,
17 atomic fault boundaries, seven SIGKILL recovery cases, backend/pipeline failure
checks, source audit, exact changes and limitations are in the
[W implementation report](/home/user/code/sapphire_git/audit/2026-09-27-w-implementation/README.md).
W is a prerequisite, **not full resumed mapping**.
**W is ready for independent implementation review.**
**Stop before B1/B2/B3.** This entry supersedes the older phase/readiness gates
below without erasing their historical evidence.

## A2.1 — narrow storage-boundary correction

Independent A2 review accepted the runtime architecture but reproduced two
storage-boundary defects: the pyramid allocation check incorrectly used surviving
sparse entries, and several SQLite readers coerced types/narrowed counts before
validation. The user authorized only these corrections, with no A3/A4/A5 work.

Production edits are confined to MapDatabase's header/implementation. Pyramid
preflight now derives its per-level capacity bound from the checked persisted
source Gaussian count: at most eight input voxels per Gaussian, and at most
16 times that input population in the current builder. It validates complete
payload lengths, indices/order, representation and allocation arithmetic before
native decode, without changing the producer or BBS. Historical INTEGER, REAL
and BLOB readers check actual SQLite storage classes before conversion. Matrix
dimensions stay full-width through product/byte checks; the writer's empty
ImageRecord `descriptors_type=-1` sentinel is preserved.

The real SubmapFrameBuffer regression reproduces 2048 buckets / 80 stored entries,
persists through MapDatabase and requires successful historical reopening.
Targeted tests also cover 40 SQL type/range mutations and 12 unsafe pyramid
payloads, unchanged bytes and released ownership; the existing 15 invalid-map
cases and all A2 behavior checks remain. Final execution status and the complete
field-by-field audit are maintained in the
[A2.1 report](/home/user/code/sapphire_git/audit/2026-09-27-a21-correction/README.md).
Independent re-review remains required. A2 review F3 is informational: no new
cleanup defect was demonstrated, and allocation/exception injection at every
construction stage is not claimed. This entry supersedes E14's incorrect
pyramid bound and readiness statements pending the corrective re-review.

E15 (2026-09-27): final Release build and full CTest passed **13/13 (8.06 s)**.
The final historical/visual CPU tests independently passed inside the sandbox
**2/2 (2.82 s)**. All 52 new malformed cases preserve files and release ownership;
the 2048/80 producer map and empty ImageRecord reopen successfully. The initial
full run caught a missed empty-matrix `-1` sentinel in the corrective code; it
was fixed and explicitly covered before the final passing run. The actual GPU
writer/readers and normal GPU regressions also pass. A2.1 is ready for narrow
independent re-review of the two findings; **stop before A3/A4/A5**.

## A2 — historical read-only reconstruction implemented, independent review pending

The user accepted A1 and authorized A2 after the design review, numerical
characterization and GTSAM persistence investigation. This entry supersedes the
older authorization/status statements below. A2 is implemented through the
existing MapDatabase, Memory, PoseGraphBackend, VisualLoopClosure and
OccupancyGrid owners. It validates persisted facts, reconstructs the original
prior and stored factors, builds a fresh configured ISAM2, and restores committed
pose/spatial lookup, appearance retrieval and occupancy from persisted LocalGrid.
Committed database poses remain authoritative; rebuilt estimates are diagnostic
runtime state. No optimizer serialization, g2o production path, migration,
precision change or additional runtime aggregate was introduced.

Backend `resume` now exposes historical queries only. Input is rejected before
mutation, correction is unavailable, and loop-decision status is always
`Unavailable`. The pipeline frontend attachment/continuation gate remains closed.
There is exactly one supported persistent format: SQLite `user_version == 1`
is a format-identity guard, independent of the product version. Other formats
fail closed. The accepted A1 mmap/deserialize opening contract is unchanged.

E14 (2026-09-27): Release build succeeded; full CTest passed **13/13 (5.95 s)**.
Independent writer → reader B → reader C cases cover 0, 1, 6, 12 and 32 Nodes,
including asymmetric transforms, a loop, committed/original pose differences,
visual history and coordinates near 20 km. A separate actual CUDA backend writer
produced a six-Node map, then both historical readers ran with CUDA hidden and
wrapped device entry points forbidden (zero calls). The CPU historical test also
passed inside the sandbox. Fifteen targeted invalid-map cases cover required
records, payloads, graph semantics and numerical discrepancy; failed opens
preserve bytes and release ownership. Successful readers check whole-file and
directory identity through queries, rejected input, destruction and reopen.

Successful reconstruction maxima: translation **0.000937506565 m**, rotation
**3.37747613e-8 rad**, per-factor unwhitened residual-change translation
**0.00132813097 m**, rotation **4.86068810e-8 rad**. These remain inside the
approved 0.035 m / 0.003 rad and 0.026 m / 0.003 rad envelopes. Objective delta
is recorded, never used as a monotonicity gate. B/C diagnostics were identical
in these fixtures; private solver/index layout equality is not a requirement.

Verification is bounded to these controlled maps and the existing regressions,
not deployed long-run maps, general real-data quality or large-history IVF
training. Existing CUDA-linked build dependencies remain; the historical path
does not initialize or require a CUDA device. F2/F3/F4 and later phases are not
implemented. A2 is ready for independent review; **stop before A3/A4/A5**.
The complete 17-item report, exact file manifest and retained logs are in
[E14 completion report](/home/user/code/sapphire_git/audit/2026-09-27-a2-implementation/README.md).
The approved contract is [A2_DESIGN.md](/home/user/code/sapphire_git/src/docs/A2_DESIGN.md).

## A1.2 — SQLite eligibility consumes the validated inode

The A1.1 re-review closed path semantics and compatibility identity. It also
reproduced a second-open FIFO race inside SQLite, outside the cooperative
namespace contract. The accepted A1.2 fix is implemented only in resume
eligibility: privately map the already-validated/locked descriptor, validate and
privately adapt a clean WAL header, then inspect it with read-only deserialize.
SQLite never reopens the original resume pathname. Existing sidecar rejection,
final identity checks, exclusive new creation, `c2:` fields and all A2 exclusions
remain unchanged. Resource order is SQLite close → unmap → descriptor close.
The exact mapping/header/ownership contract and remaining in-place mutation /
truncation limitations are in PHASE1_DESIGN.md. Final independent re-review is
still required; A2 remains unauthorized.

A1.2 evidence (E13, 2026-09-27): Release build and the linked SQLite 3.37.2
deserialize capability check passed. A separate configure experiment with a
library missing that symbol failed at the explicit capability gate with the
intended diagnostic (`/tmp/sapphire-a12-no-deserialize/configure.log`).
`sapphire_map_eligibility_test` passed all four groups: normal DELETE/clean-WAL
headers and actual SQL write rejection; 12 deterministic replacements (six
forms, before mmap and before SQLite open); six failure/cleanup paths; and eight
short-size/magic/header rejection cases. Replacement cases inspect the original
UUID/config/revision/user_version and return `StaleMap` without reopening the
pathname. Full canonical bytes, including header bytes 18/19, stay identical;
the private view differs only by the permitted WAL-header adaptation. Cleanup
checks observe SQLite close → unmap → descriptor close, no outstanding SQLite
statements, no descriptor leaks, and released advisory ownership.

Full CTest outside the sandbox passed **11/11 (4.94 s)**, including map_open,
map_storage, visual_mapping and core regressions. CUDA-dependent
visual_laser_loop, retrieval_loop and loop_pipeline_verification passed **3/3**.
Logs: `/tmp/sapphire-a12-build.log`, `/tmp/sapphire-a12-build-final.log`,
`/tmp/sapphire-a12-eligibility-build.log`, `/tmp/sapphire-a12-eligibility.log`,
`/tmp/sapphire-a12-ctest.log`. Scope remains A1 eligibility only.

## A1.1 corrective pass — independent re-review required

The independent A1 review reproduced lexical path rewriting, an overly broad
configuration fingerprint, and a blocking FIFO open. The corrective pass is
limited to those findings: filesystem resolution of the existing parent,
nonblocking descriptor/type validation, and compatibility encoding `c2:` with
the six occupancy reconstruction parameters only. The complete field audit and
path contract are in [PHASE1_DESIGN.md](/home/user/code/sapphire_git/src/docs/PHASE1_DESIGN.md).
Old `a1:` identities are not silently reinterpreted or migrated. Journal policy,
cooperative inode ownership, failed-new residual behavior and the A1 stop gate
are unchanged. A2 remains unauthorized; resume is eligibility only.

A1.1 evidence (E12): Release build completed successfully. The expanded direct
`sapphire_map_open_test` passed all eleven groups outside the sandbox, including
actual-inode symlink/`..` resolution, nonexistent intermediate components,
relative/absolute paths, bounded FIFO/directory/socket rejection, excluded-field
eligibility, all six critical occupancy mismatches, old encoding rejection, and
a fixed-vector/process/locale determinism check. Full unrestricted CTest passed
**10/10 (4.78 s)**. Its three CUDA-dependent cases (visual_laser_loop,
retrieval_loop, loop_pipeline_verification) all passed. The initial sandbox run
passed 6/10; map_open could not bind its Unix-socket fixture and the same three
CUDA tests could not detect a device. These environment failures passed on the
unrestricted rerun without weakening tests. Logs: `/tmp/sapphire-a11-build.log`,
`/tmp/sapphire-a11-map-open-unrestricted.log`, `/tmp/sapphire-a11-ctest.log`,
`/tmp/sapphire-a11-ctest-unrestricted.log`. Independent re-review remains the
gate; these tests do not claim full restoration or continuation.

## A1 update — 2026-09-26, after the audit snapshot

Only **A1 safe opening / mode protection / insert-only Node identity** has been
implemented. The approved boundary and later-stage constraints are recorded in
[PHASE1_DESIGN.md](/home/user/code/sapphire_git/src/docs/PHASE1_DESIGN.md).

- `new` exclusively creates the requested database and rejects any existing
  destination. Pipeline directory creation no longer recursively clears an
  existing timestamp directory before opening its map.
- A storage-level `resume` handle performs non-mutating schema/identity/config
  eligibility checks only. It requires a closed/checkpointed file and rejects
  journals instead of recovering them. Read-only paths are supported.
- Backend/pipeline resume validates and then explicitly reports
  `ResumeUnavailable`, before runtime/index reconstruction or worker startup.
  `localize` is explicitly unsupported. Memory no longer implicitly adopts an
  existing map through its ordinary constructor.
- Schema 1 adds map UUID and a configuration compatibility fingerprint. It does
  not certify historical graph completeness or implement the later T1/T2
  revision contract. Existing graph_revision behavior is not upgraded to an
  output-completion guarantee.
- New Node insertion rejects duplicate identities transactionally. Writer
  identity/revision and inode checks reject stale writes; cooperative file locks
  and SQLite transaction locks expose conflicting opens/writes.

F1's audited unsafe-open/overwrite path is addressed by A1. Actual historical
graph restoration and continuation remain unavailable; this is **not verified
resume**. F2–F5 remain unchanged. The earlier snapshot and E1–E10 below describe
the pre-A1 audit, not the implementation status of these now-protected paths.

A1 evidence (E11): `sapphire_map_open_test` covers typed errors and byte/state
non-mutation, including independent-process exclusive-create collision. Storage
and core tests use explicit read-only record inspection; cache invalidation is
tested independently rather than by overwriting a canonical Node.

E11 execution (2026-09-26): the Release build in `/tmp/sapphire-backend-build`
completed successfully. The direct A1 test passed all eight groups. Full CTest
passed 7/10 in the sandbox; the three CUDA-dependent tests failed only with
`cudaMalloc: no CUDA-capable device is detected`. Rerunning those three outside
the sandbox passed 3/3: visual_laser_loop, retrieval_loop and
loop_pipeline_verification. All ten registered tests therefore have passing
results. No registration algorithms or registration tests were changed for A1.
Logs: `/tmp/sapphire-a1-build-final.log`, `/tmp/sapphire-a1-map-open.log`,
`/tmp/sapphire-a1-ctest.log`, `/tmp/sapphire-a1-gpu-ctest.log`. These verify A1
safe opening and existing regressions, not historical runtime restoration.

## Snapshot identity and scope

| Field | Snapshot |
|---|---|
| Snapshot date | 2026-09-26，Asia/Shanghai |
| Canonical project / Git root | Sapphire；`/home/user/code/sapphire_git/src`，父目录不是 Git repo |
| HEAD | `59954c8723e43d5d1efbd64769b8d3cab84fe61c`，简称 `59954c8` |
| Working tree | **dirty**；本次文档固化前43个tracked路径有修改/删除，30个untracked status条目（包含目录，不是文件总数）；业务差异均已存在 |
| Consolidation changes | 仅新增 Git 根 `AGENTS.md`、本文件、`docs/MIGRATION_LEDGER.md`；不修业务代码，不开始Phase 1 |
| Primary evidence | 2026-09-26独立源码审计、当前增量build/CTest、独立CPU/GPU审计程序、跨进程记录读取、真实PoseGraph及录制子图重放 |
| Code identity check | 固化开始时，审计source manifest全部匹配，Git status与审计结束时相同；F1/F2关键控制流另行核对 |
| Original Sapphire comparator | 本仓HEAD作为可复核基线；不能保证它恰好是上个coding session的起点，不把所有差异归因于本次迁移 |
| Pandora comparator | `/home/user/code/pandora_slam` HEAD `9cd57f21`；直接检查 `backend/exterior` 源码，不采用作者自述作验证依据 |

**这是当前工作区的 authoritative snapshot，不是永久事实，也不是“完成”声明。** 后续session须核对HEAD、dirty diff和实际源码后使用；若有变化，更新事实、证据范围及ledger。长期规则在 [AGENTS.md](/home/user/code/sapphire_git/src/AGENTS.md)，逐能力决策在 [MIGRATION_LEDGER.md](/home/user/code/sapphire_git/src/docs/MIGRATION_LEDGER.md)。

Sapphire是canonical project；Pandora-SLAM是同源分支的后端演进，自研能力回迁不作为第三方系统整体接入。吸收单位是能力/算法/架构/数据管理设计。

## Current canonical architecture

```text
LiDAR / IMU
→ Synchronizer → 初始化、去畸变、ESKF、滑窗优化
→ Gaussian MargiFrame（odom系均值/协方差）
→ SubmapFrameBuffer（首帧body/IMU为anchor，默认约15m行程）
→ SubmapFrame（局部Gaussian、BBS pyramid、odom poses、导航、VisualFrame）
→ PoseGraph worker → Memory / SQLite子图事务
→ spatial R-tree / visual Faiss retrieval → candidate set
→ BBS → GICP → loop factor → ISAM2
→ optimized submap anchors、pose/link事务、空间索引更新
→ LocalGrid重新融合 / grid output
```

优化更新子图anchor，不原地重写全部Gaussian。离线全局地图按 `Node.submap_pose × LaserRecord` 构造；frontend的local scan/map/trajectory仍处于odom系。最后的map→odom对外发布存在F2，以上数据流不意味着发布契约已经闭合。

```text
Image → 有界图像队列 → ORB → VisualFrame
→ appearance scene（descriptor精确去重/截断，最多8192）
→ Faiss → historical submap ID → LiDAR geometric verification
```

视觉仅做appearance候选提议。单目无需depth；`mode=stereo`只是两个独立图像流，没有stereo depth、triangulation、PnP metric initialization或geometric scene fusion。scene中的 `has_position=false`，不能将存在3D字段当成已实现metric视觉能力。

Faiss实际在线调用：BinaryFlat+IDMap2起步，达到8192 descriptors后后台训练128-list BinaryIVF，nprobe=4。查询前排除当前/未来和最近3个子图；默认top-K=5、min_matches=20。历史scene延迟插入，query features暂存于pending，finishQuery后释放。archive底层save/restore cache存在，但生产VisualSubmapIndex未接入。受控查询有效不意味着真实图像召回达标。

`T_A_B`把B系坐标转到A系。BBS/GICP输出 `T_target_query`；代码添加 `BetweenFactor(query,target,inverse(T_target_query))`，SQL Link同方向、Node ID为内部submap ID+1。空间初值为 `inverse(T_map_target)*T_map_query`；视觉候选清零平移、保留相对姿态，再由BBS搜索。BBS不搜索任意roll/pitch；GICP最终接受检查converged、至少64内点、finite pose、fitness<0.5。每query最多接受首个通过的候选，不保证全候选最优。

地面路径：子图Gaussian → 80×80、1m XY lattice里的真实centroid → CAPE近水平patch；不足20则回退到Gaussian平面/协方差 → scalar horizontal floor → LocalGrid ground/obstacle/free rays → occupancy tile融合。没有DepthFrame适配层。ground不校正pose graph的Z，完整地形支持存在F3/F4。

### Ownership and state boundaries

| Owner / state | Current responsibility / boundary |
|---|---|
| SlamPipeline odometry thread | frontend状态、滑窗、边缘化；mutex/CV向mapping传递MargiFrame |
| Mapping thread | ORB、子图累积和冻结；异步addFrame；随后立即读correction并输出，产生F2时序缺口 |
| PoseGraph worker | 独占graph frames、ISAM2、VisualSubmapIndex、ground和occupancy；Memory存取 |
| Memory | DB、spatial/pose索引、cloud/grid LRU、scene prefetch；const cloud共享，数据库和索引分别加锁 |
| Scene prefetch / Faiss training | prefetch在Memory销毁前join；训练持有不可变sample；正常路径owner明确，未完成TSAN/长期压力验收 |
| Persistent state | Node的odom序列和submap pose；SpatialRecord、MetaTag、NaviTrajectory、LaserRecord、ImageRecord、VisualScene、PyramidVoxel、FlatGrid、Link、MapState |
| Runtime graph / map | frames、ID计数、ISAM2、optimized values、搜索进度、correction、ground reference和OccupancyGrid；当前没有完整恢复链 |
| Derived indexes / caches | Memory启动重建spatial/pose索引；Faiss历史scene按query推进重建；缓存不等同于持久事实来源 |
| Existing revisions | DB `MapState.graph_revision`随pose/link事务增加；occupancy有独立进程内revision；二者尚未形成一致发布revision |

依赖：ROS2 adapter → sapphire_core → Eigen/Boost/OpenCV/SQLite/Zlib/toml++/spdlog/OpenMP/small_gicp；仓内外部依赖为GTSAM、Faiss、CAPE、CUDA BBS。没有生产FBoW依赖。core CMake强制CUDA；独立审计用Release、BUILD_TESTING=ON、arch52已有build，非默认arch89全新clean build。本次文档固化未重新执行build/CTest。

## Verification vocabulary

| Status | Interpretation |
|---|---|
| implemented | 代码及调用路径存在，不暗含通过验收 |
| verified | 指定实验范围内有可重复证据 |
| partially verified | 部分层级/条件成立，尚有未验证部分 |
| unverified | 缺少足够证据，不据此判错 |
| broken | 有反例或明确契约违例，需保留触发条件 |

禁止把一项的verified扩展到整套系统；禁止把未接线的方法称为已完成能力。

## Verified capabilities and limits

| Evidence | Verified result | Scope / limit |
|---|---|---|
| E1 独立审计build / CTest | GPU可访问后9/9通过，4.83s | 沙箱内3例仅因CUDA不可见失败；build/test不等于feature整体验收，未重跑完整ROS sensor bag |
| E2 控制图像重访 | 真实ORB：query4→target0 top1，1195匹配；0–3时间排除；随机5–7无候选；query8命中0/4 | 固定seed纹理；不是跨视角/光照真实recall@K |
| E3 两进程记录write/read | cloud数值、pose、导航、pyramid、LocalGrid、VisualFrame恢复；重新提交query后top1=0且1195匹配 | 未重新提交pending query时查询为空；仅恢复/重建记录和查询，不是resume continuation |
| E4 非对称SE(3) BBS→GICP | t=(1.3,-.7,.4)m，R=Rz(.37)Ry(-.11)Rx(.08)；平移误差5.94e-7m，旋转4.67e-9rad；不重叠query拒绝 | 1500个合成Gaussian，roll/pitch初值正确；非任意6DoF冷启动 |
| E5 因子及SQL方向 | 真实BetweenFactor正确inverse残差5.58e-7，故意反向3.17466；真实worker落库Link(5,1,type1)与期望inverse矩阵误差5.83e-7 | 验证方向和接线；不证明全局轨迹真值精度 |
| E5 合成完整backend链 | query4视觉候选跨120m人为odom平移偏差进入BBS/GICP并落loop | 不是120m轨迹漂移被正确修复的证明 |
| E6 录制子图重放 | 14子图/106 VisualFrame；5次BBS、1次GICP；query13→0空间回环，8794内点，fitness=.0582；13 odom+1 loop；DB integrity ok | 源为录制地图的子图，非原始sensor bag；无外部真值。视觉候选总0，CAPE patches全部0 |
| E6 地图重放一致性 | 重放pose矩阵与输入历史结果最大差约9.54e-7；14个FlatGrid落库，发布3个grid revision | 是同实现重放一致性，不是地图几何/TF一致性的真值验收 |
| E7 平地合成ground | 3721点，64个CAPE patch，floor=-1，ground3721/obstacle0 | CAPE仅平地合成正例；不推广到真实地形 |
| E8 事务回滚 | 注入scene写入失败时整个子图回滚；非法link使pose/link/revision事务回滚且live pose不更新 | 没有证明ISAM/DB跨层回滚或crash/power-loss恢复 |
| E8 缓存 | const cloud命中、LRU eviction保留reader快照、prefetch失效/过期in-flight丢弃 | 非全系统RSS或无限运行内存上限 |
| E9 索引底层 | archive cache恢复一致、erase/reinsert成功；Flat/IVF受控匹配成立 | 生产restore未接线。28,800 descriptors查询约189.5→10.65ms，仅局部微基准；非整体A/B |

E7另覆盖10°坡、垂直墙、20cm台阶、25点稀疏地面和3.5cm噪声地面。墙不产ground；margin=.15的台阶可区分；稀疏25点超过回退最小20支持，不是稀疏拒绝测试。坡面反例见F4。

## Partially verified / unverified / broken boundaries

| Capability / claim | State | What remains |
|---|---|---|
| Real-data visual recall | partially verified：合成有效；真实效果unverified | 本次录制重放visual candidates=0；需有重访标签/重复场景/视角光照变化的recall与误检证据，不能把空间loop当视觉贡献 |
| CAPE real-data effectiveness | partially verified：合成有效；真实效果unverified | 14个子图CAPE patches=0，全部靠Gaussian回退；不证明CAPE对所有真实数据无效 |
| Complete map resume | broken | 有非空DB覆盖/编号拒绝反例；尚无两进程continue通过证据 |
| Index production restore | implemented（底层）/ unverified（产品路径未接线） | archive缓存API已测，VisualSubmapIndex没有生产save/restore调用 |
| End-to-end throughput | unverified | 3.91s子图重放不含sensor frontend；没有同输入/参数/硬件整套A/B |
| Full process RSS scaling | unverified | 单次replay peak301,324KiB包含reader缓存及GPU；图、索引、栅格和队列长期增长未验收 |
| Crash / power-loss recovery | unverified | 已测普通事务异常，不含kill、掉电、磁盘满、ISAM/DB跨层故障 |
| General terrain navigation | broken / unverified | F3/F4有明确反例；实际可通行性与真实地图准确性未验证 |
| Arbitrary 6DoF cold-start registration | unverified | BBS沿用roll/pitch初值，不能凭SE(3)结果类型声称6DoF全局搜索 |
| Real ground-truth trajectory accuracy | unverified | 无外部真值ATE/RPE；回环存在与重放一致性不是精度真值 |

## Known correctness issues and engineering debt

ID沿用audit便于追踪，下面以触发条件/证据和影响定性，不把severity标签代替推理。

### F1 — Map lifecycle：已有DB不是可恢复的backend

Memory可加载历史记录/索引，但PGO重新创建空ISAM2，frames/optimized/ID计数未恢复。已有5节点地图重开后提交5被拒绝：`expected=0, received=5`。在副本提交0、x=999，Node1被覆盖成x=999，历史5条边仍在。常规pipeline另建时间戳目录，所以风险触发条件是backend复用旧DB路径；常规启动则没有resume能力。

这属于数据完整性风险，应先定义mode并禁止不匹配写入，再恢复graph和runtime。不能只把next ID改为max+1。定位：`PoseGraphBackend::Impl`、`addFrame`、`MapDatabase::saveNode`；E10记录反例。它至少部分继承HEAD，不能将其全部归因于新迁移。

### F2 — Optimization/publication：最终revision没有发布保证

mapping线程异步addFrame后立即取T_map_odom并发布。PGO worker之后更新内部correction，没有确保在无下一输入时发布它；grid另经callback发布。最终loop的grid与外部TF/correction可能属于不同优化版本。

这是源码控制流证据，不是已经通过的ROS时序测试。已有DB `graph_revision`与occupancy revision不是统一发布协议。定位：`SlamPipeline::thd_mapping`、`PoseGraphBackend::Impl::updateCorrection/processPending`。Phase 1要先设计版本/提交/发布/退出契约，不能只补一个callback。

### F3 — LocalGrid coordinate：降维后的SE(3)不成立

LocalGrid仅存局部XY，OccupancyGrid却用完整pose乘(x,y,0)。局部点(2,0,-1)、pitch30°：真实map x=1.23205，契约实际投影x=1.73205，差0.5m。yaw-only特例不暴露问题。一般roll/pitch下丢失的信息无法凭设z=0恢复。

定位：`LocalGridMaker::createLocalMap`、`OccupancyGrid::transformedCell`。**Phase 1不修此项**；Phase 2先定义frame、信息损失与合法transform，再改代码。

### F4 — Ground：scalar horizontal floor不代表一般地形

10°坡的3721点用例中，CAPE patches=0；回退估计floor=-2.222，305 ground cells、2623 obstacle cells，很多坡面被当障碍。当前表示是一个水平高度，不是plane/height field。默认mid360/dual ground_margin=.5m，visual profile=.15m；20cm台阶在后者可分，在前者按现有阈值会归地面。配置意义须与traversability分开确认。

定位：GroundEstimator、FlatGroundReference、LocalGridMaker及profiles。**Phase 1不调CAPE/阈值、不更换ground表示**；这与F3是后续坐标/地形设计问题。

### F5 — 性能债务有事实，整体性能改善无A/B结论

空间候选无统一数量预算，先加载全部候选pyramid；target cloud在BBS前加载；每候选BBS采样/上传、GICP重建KD-tree；视觉在min_matches过滤前加载scene；无fingerprint的loadScene绕过scene cache；生产index restore未接线。`cold_loads`计数不是实际cache miss/磁盘读数。全局Faiss/graph/spatial/occupancy及无背压队列仍增长。

受控IVF查询更快只证明局部收益；本次没有全系统同条件A/B。此项是需观测和后续优化的债务，不得把它混同数据损坏，也不在Phase 1顺手优化。

### Other retained audit facts

`create_raw(..., empty points, ...)`填NaN points，`FeatureMap::from_frame`跳过它们，导致辅助链1200 descriptors→0；生产buildVisualScene走另一构造路径，故不是当前在线视觉全空的解释。此项留档，不在Phase 1删除FeatureBlock。frontend尾滑窗/reset边界、单子图origin对多视点free rays、异常后ISAM/DB一致性仍有审计限制；不要把backend drain范围悄悄扩为frontend重构。

## Important non-problems / confirmed facts

- **没有已证实的“GICP再次从DB读取同一target cloud”问题。** 当前GICP消费传入的共享const cloud；更早的加载时机/缓存策略债务另见F5。
- **BBS target pyramid不是每次由cloud重建。** 当前读取保存的pyramid；每次GPU准备/上传不等于CPU pyramid重建。
- **loop inverse / BetweenFactor / SQL Link方向已有强验证。** 没有新反例不得凭习惯再次翻转query/target。
- 当前core没有错误的`pandora_visual`/`exterior` ownership层，也未搬入重复logging系统；`PoseGraphBackend`的backend命名语义正常。
- CAPE adapter直接用Gaussian lattice，没有为了迁移新造DepthFrame。
- 视觉生产路径确实调用Faiss，不是只有unused implementation；真实效果不足与“从未调用”不同。

## Current next phase — Phase 1 only

**审计固化时 Phase 1尚未实施；当前只实现上方 A1，下一步 A2 必须另获授权。完整 Phase 1 仍仅处理 A（map lifecycle/persistence）和 B（optimization revision/publication）。** F1先保护持久事实，A恢复可依赖的状态，B使其对外一致；这些是后续地图/检索验收的基础。问题严重性并不等于必须在同一session同时修复F3/F4/F5。

### Entry gate：先设计，再实现

重新核对Git、这三个文档和源码。先记录mode矩阵、persistent/runtime/index/graph恢复清单、revision/发布状态流程、owner和测试验收条件；检查现有等价抽象，不先创建新Frame/通用backend层，不先加callback。以下是下一阶段的要求，**不是当前API或schema已支持的行为**。

### A. Map lifecycle / persistence contract

**Map/backend resume != frontend estimator state resume。** Phase 1不包含完整ESKF、IMU bias history、active sliding window等frontend estimator runtime state的序列化/恢复。新frontend session可以使用新的odom frame，但必须与persistent map建立显式attachment/alignment contract，不能默认坐标连续或默认恢复整个frontend。

先消除silent corruption，再完成map/backend resume，顺序如下：

1. 定义互斥入口：`new`只创建新地图；`resume`恢复既有地图并延续写入；`localize`使用历史地图定位、不得隐式续建或覆盖canonical map。明确空DB、非空DB、未知schema、只读路径和失败返回行为。
2. 在resume完整可用前，对mode不允许的非空DB **hard reject**；拒绝不能顺带修改旧Node/schema/graph，不允许fallback到new、重编号、清空或覆盖。localize完整定位功能不属于本phase扩展目标；如尚不支持必须明确拒绝，不能伪装resume。
3. 明确持久事实与可重建runtime。现有表并不自动足以恢复所有语义；尤其prior/noise/配置/地图身份/版本可能缺失，须设计受控兼容策略或拒绝不支持的旧图，不能默默用不同默认值恢复。
4. 按下表设计恢复，再实现continuation；不要求二进制序列化ISAM2，允许由持久图重建，但不能重放原始sensor数据冒充resume。

| State | Required recovery/consistency decision |
|---|---|
| Map/session identity and node IDs | 内部submap ID与SQL Node ID映射、next ID、顺序/空洞校验；保留旧节点身份，不覆盖 |
| Submap/odometry/optimized poses | 区分原odom anchor与优化后map anchor，恢复odom序列和最新committed poses；不要混用Node字段 |
| Odom / loop edges | 类型、端点、方向、重复插入规则、noise与历史优化语义；重建图后验证 |
| Prior / graph state | 固定锚、prior/noise/robust model及优化配置来源，ISAM初始化与搜索进度；说明缺失信息如何处理 |
| New process odom frame | 定义新frontend odom frame与persistent map的attachment/alignment关系、验证条件及未对齐时的行为；不能直接复用旧correction或凭跨session相邻ID构造odom边，也不以恢复整个frontend代替此契约 |
| Spatial / pose indexes | 从committed bounds/poses重建并与graph一致；检索历史节点可达 |
| Visual history / index | 恢复持久appearance身份、历史范围和时间排除；cache可拒绝并重建，但不能把cache当事实来源，不做Faiss调参 |
| LocalGrid / occupancy | 加载既有LocalGrid并按恢复poses重建现有occupancy；明确revision和一致性。只恢复现有语义，不借机修F3/F4 |
| Remaining runtime state | 明确ground reference等哪些需恢复/重置及其后果，避免第二进程隐式改变既有契约；不扩大为ground算法重做 |

必须通过独立进程continuation验收：process A创建地图、加入N个子图并触发loops、正常close；进程销毁；process B以resume打开，验证恢复的nodes/edges/poses/index/grid，随后加入N+1…、优化、关闭。此处N指已存子图数，实际0-based next ID按已定义映射断言。历史payload和ID不得变化；历史pose只允许有可解释的合法优化变化；links无丢失/重复，历史检索与grid/graph一致。包含非空DB错误mode拒绝、旧ID覆盖拒绝、缺失/不兼容恢复信息失败、非对称loop方向和不需要原始sensor文件的测试。

### B. Optimization revision / publication contract

目标是一个optimization revision对外表现为一致状态。必须先回答下表，再选实现形式：

| Contract item | Required decision / acceptance |
|---|---|
| Revision number | map身份+单调revision的定义、生成owner、跨resume延续及commit点；现有graph_revision不能直接冒充完整发布revision |
| Optimized poses | 哪个完整/增量pose集合属于revision，消费者如何应用/验证它 |
| map→odom correction | 对应哪个odom frame、参考pose与revision；与poses同源，末次优化后无新输入仍交付 |
| Grid revision | 标出来源pose revision；grid未就绪/禁用/失败的显式状态，不能把旧grid冒称与新correction一致 |
| Publish ownership | 哪个线程/组件交付一致状态，ROS与非ROS消费者如何观察revision，避免mapping先读后端旧值的时序；不预定新增callback/class |
| Commit/failure boundary | DB、ISAM runtime、derived indexes、grid、输出之间的顺序及部分失败策略；不能对未完成状态报告成功 |
| Flush / drain | 停止接收后的accepted input边界；等待持久化、最终优化、必要grid构造及输出交付/确认的语义；shutdown幂等和生命周期界限 |
| Backend failure visibility | 写入/优化/grid/output失败如何到达调用者；可重试/不可恢复状态和drain结果，不以日志代替结果 |

必须新增末次输入用例：**最后一个submap触发loop closure，之后没有任何sensor/submap输入。** 等待已设计的drain/交付条件，最终correction、map state、grid必须标明一致的最后revision并对外可见；不靠额外dummy frame、sleep猜测、轮询下一输入或仅析构后的DB检查。覆盖grid禁用/失败的显式语义、backend失败可见性，以及A恢复后revision继续工作的情况。F3仍存在时，仅验证既有grid与该pose revision的构造/标记一致性，不把它称为一般SE(3)栅格几何已正确。

### Explicit exclusions and exit criteria

Phase 1不处理F3坐标修复、ground表示重做、CAPE调参、视觉扩展、geometric scene fusion、Faiss调参、BBS/GICP性能优化、FeatureBlock删除或全量architecture debt清理。A/B若必须触及相关文件，先记录必要依赖、最小改动与回归证据，保持算法/坐标语义范围；不能据此扩大phase。

Phase 1完成必须同时有：错误mode非空DB保护、跨进程真实continuation、一致最终revision发布、flush/drain与失败可见性测试，并更新本文件和ledger。只完成保护可记录阶段进展，不能将resume标为verified。此文档不授权本次固化session执行这些实现。

## Phase 2 — design questions only

主题：**LocalGrid coordinate contract + ground representation contract**。开始Phase 2时先写数学/坐标设计，回答全部问题，然后才实现；现在不patch F3/F4。

1. LocalGrid究竟在哪个frame，anchor如何定义？
2. 是否gravity-aligned，重力估计和anchor更新如何处理？
3. submap 3D evidence降到2D时丢弃哪些信息，保留什么足以支持重建？
4. 降维后允许哪些transform，有什么数学条件？
5. PGO更新带roll/pitch时，grid由哪种证据、在哪个frame重新融合？
6. 多视点观测的ray origin如何表示，如何避免从单一anchor错误清空遮挡后区域？
7. ground选择scalar floor、plane、piecewise plane还是height field；适用域、未知区域和失效行为是什么？
8. slope/step/curb的traversability与occupancy分别如何定义，阈值如何进入各自语义？

## Evidence registry and re-entry

证据原目录在Git根之外：`/home/user/code/sapphire_git/audit/2026-09-26`；真实源录制地图在 `/tmp/sapphire-0519-visual-v2/2026-09-26_20-03-27/map.db`。这些本机路径可能随环境清理失效；本文已记录关键输入、结果和限制，不能因文件缺失擅自将结论升级。复现前确认文件与source manifest，缺失时报告证据不可用。

| ID | Local artifacts / source anchors |
|---|---|
| Audit | [独立报告](/home/user/code/sapphire_git/audit/2026-09-26/REPORT.md)、[复现说明](/home/user/code/sapphire_git/audit/2026-09-26/README.md)、source-sha256.txt、git-status-before.txt |
| E1 | build.log、ctest.log、ctest-gpu-access.log；build目录 `/tmp/sapphire-backend-build` |
| E2/E3 | probe.cpp的write/read；persistence-write.log、persistence-read.log、persistence.db |
| E4/E5 | gpu-probe.log、cpu.log、backend-write.log、backend-sql-verification.json |
| E6 | real-replay.log、real-replay-summary.json、real-replay-time.txt、real-input-copy.db、real-replayed.db；重放配置mid360_visual.toml |
| E7 | cpu.log及probe.cpp ground；flat/slope/wall/step/sparse/noisy诊断输出，不能将诊断程序exit0等同六类全部正确 |
| E8 | sapphire/tests/map_storage_test.cpp；CTest对应结果 |
| E9 | bench.log、bench-time.txt、index.cache；底层测试，不是生产restore |
| E10 | backend-reopen.log、backend-overwrite-result.json；只在审计DB副本执行覆盖反例 |
| Code | [pipeline](/home/user/code/sapphire_git/src/sapphire/src/pipeline.cpp)、[graph](/home/user/code/sapphire_git/src/sapphire/src/mapping/graph/pose_graph.cpp)、[storage](/home/user/code/sapphire_git/src/sapphire/src/mapping/storage/map_database.hpp)、[visual](/home/user/code/sapphire_git/src/sapphire/src/mapping/visual/visual_loop.cpp)、[grid](/home/user/code/sapphire_git/src/sapphire/src/mapping/grid/local_gridmap.hpp) |

## Consolidation self-review

2026-09-26以不依赖聊天的新agent视角自审：project identity与长期约束见AGENTS；真实architecture、视觉范围、verified/implemented边界、F1–F5及非问题见本文；能力来源和semantic gap见ledger；下一步只做lifecycle + revision consistency及明确禁区见Phase 1；Phase 2保留为设计问题。未发现audit与当前源码的新事实冲突。文档固化不修业务实现，也不将任一问题标为已修复。
