# Pandora → Sapphire capability migration ledger

## Shared-map Gaussian observation experiment — 2026-10-08

Native Sapphire work, not a Pandora capability import. The verified GICP observer
now lives in frontend/ricp and is optionally called by the ESKF tracking path.
The old standalone RICP observer/configuration has been retired and archived;
frontend remains eskf. Optional
`odometry.gicp_fallback.enabled` defaults off; normal ESKF/Gal3 path and
threshold are retained. User accepted this as a formal configurable frontend
capability, then requested library-owned registration solve. Unmodified small_gicp
1.0.1 now owns GICP factors, LM and convergence, using a call-local adapter over
existing voxel moments. No second persistent map. LiDAR-only registration is
followed by one full-state ESKF fusion; acceptance/rollback remain ours.
Default iteration budget20 matches the library default. Full Hilti coarse leaf0.5m:
30→0 resets,21 successful fallback calls,0.42086m sparse-position RMSE (archived
custom solver0.69009m; library6-step trial0.53426m with12 rejected frames). Fine
leaf0.25m remains exactly at its0.21546m baseline with0 calls. Core/ROS builds and
three targeted test targets pass. After moving the capability into Ricp, full
coarse on/off replays preserve every trajectory and observation result exactly
([migration evidence](../../audit/2026-10-08-ricp-component/results.json)). Geometric information is not calibrated pose
uncertainty; no universal robustness/accuracy claim or odometry/mapping split.
[Current scope and evidence](CURRENT_STATE.md#optional-shared-map-gaussian-eskf-fallback--2026-10-08).

## Experimental RICP and native frontend comparison — 2026-10-07

Historical record of the retired standalone frontend. Current Ricp is the optional
ESKF observation component above, with unchanged verified GICP/fusion behavior.

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


## Direct stereo depth and efficiency — 2026-10-07

The new explicit Hilti profile uses360x270, <=500 retained features per camera,
calibrated synchronous stereo and no temporal optical flow. Same descriptor rows
serve stereo, submap-change detection and historical retrieval; no LiDAR depth
association. VSLAM-inspired rectified patch refinement gates disparity without a
fixed metric range. Two-camera evidence persists as explicit VOM3; old formats
remain supported. LIO and mapping already have separate owners/threads, with a
bounded downstream handoff that can apply backpressure.

Core31/31 and ROS3/3 pass after efficiency changes. Controlled24-pair image A/B
reduces stereo p50/p95 about13% with byte-identical output hashes. First full direct
stereo run yields849 pairs,34,719 depth observations, zero Reset/refusal and a clean
reopen; online PnP attempts remain0 and committed sparse-position RMSE0.683m is
worse than online0.157m. Stereo depth is verified within these inputs; useful metric
visual loops, correct backend geometry and long-run growth remain unqualified.
Final optimized replay also completes/reopens: stereo p50/p95/max4.17/9.02/98.37ms,
CPU57.9%, no marginal wait>1ms. PnP remains0; online/committed RMSE0.180/0.313m.
Different full-run odometry/loop outcomes preclude attributing accuracy or whole-system
CPU changes to the stereo optimization.
[Full replay and optimization evidence](../../audit/2026-10-07-stereo-depth/README.md).


## Hilti exp09 qualification result — 2026-10-01

Full 1x Hesai/IMU and cam0/1 runs finish and reopen, but real visual-backend
qualification is **not passed**. Identical online odometry has 0.179m sparse-position
RMSE; committed layouts are 0.209m (LIO) and 1.108m (cam0/1). More accepted loops
(4→23) do not mean correct loops. Online PnP has zero attempts; offline diagnosis
finds insufficient metric correspondence support. No real scene replacement or
automatic cross-session continuation was demonstrated. Engineering/store successes
and earlier synthetic results remain valid within their scope; do not promote them
to usable real-world visual localization or growth convergence.
[Evidence and next priorities](../../audit/2026-10-01-hilti-exp09/README.md).

## Active spatial retrieval membership — 2026-09-30

Scene retirement now removes only searchable R-tree membership while retaining
graph identity and historical committed poses. Pose updates cannot reactivate
retired targets; cold reconstruction follows authoritative SceneState. Hot spatial
eligibility no longer issues one SQLite query per candidate. Existing c2/c3 remain
all active. Core29/29 and ROS3/3 pass; graph metadata contraction remains out of scope.
[Evidence and isolated lookup comparison](../../audit/2026-09-30-active-spatial/README.md).

## Controlled persistent scene refresh — 2026-09-30

Covered revisits now replace heavy searchable payloads in opt-in c4/schema3 maps
while preserving contiguous graph identities and existing factor/solver semantics.
A separate post-observation transaction records retirement and its revision;
materialization invalidates caches and removes descriptor membership. Full Gaussian
coverage and preservation of all old indexed metric visual appearances are required.
Measured placement must also agree with the authoritative committed layout within
the retirement-only 0.02m gate; finite disagreement retains both payloads.
Uncertain/new content stays retained. Default formats and old databases are unchanged.

Production new-map, B1 and B2 paths are connected. Controlled process cycles keep
one active payload across 13 graph nodes and reuse SQLite pages; same-session tests
retain the existing temporal window. Novel/missing visual support and transactional
fault recovery have explicit tests. This does not contract graph/spatial summaries,
prove long-run CPU/RSS or validate Hilti/dynamic-scene generalization.
Final core29/29(58.29s), ROS3/3(3.66s) and active-scene exporter checks pass;
unknown later-COMMIT recovery and committed-layout preservation are included.
[Packet](../../audit/2026-09-30-scene-refresh/README.md).

## Revisit refresh prerequisite — 2026-09-30

Pandora's scene contraction was inspected; its graph-node removal/edge redirection
is not transplanted into Sapphire's graph contract. New accepted-GICP diagnostics
report bounded bidirectional Gaussian support, with unique pairing and explicit
incomplete outcomes. Stored payload attribution shows heavy scene records dominate
one frozen repeated-route database. This advances the evidence for controlled
payload refresh; persistent replacement, visual-view preservation, active index
membership and map-size convergence remain unfinished.
[Evidence](../../audit/2026-09-30-revisit-coverage/README.md).

## Restricted automatic frontend handoff — 2026-09-30

Opt-in resume now connects the existing association backend to a Pipeline-owned
worker and bounded ordered pending queue. Later image evidence can initialize Q0;
Q0's own geometry must still pass independent verification. Successful B1 retires
the root and hands retained source order to B2. Normal shutdown handles the mapping
tail and drains without another sensor input; cancellation, numerical errors,
unresolved association and budget exhaustion have visible failure semantics.
ROS exposes queue/attachment state and producer errors.

This is a restricted initial-overlap capability: default 8 submaps/128 MiB,
64 MiB/item, memory-only startup retention. It does not merge an arbitrary new
prefix or make unattached observations crash-durable. No schema/solver closure
changes. Full evidence and configuration:
[frontend packet](../../audit/2026-09-30-auto-frontend/README.md).
Full core27/27(56.89s), ROS checks3/3(3.60s), controlled Pipeline association and
independent-process continuation/reopen pass.
Repeated-scene updates, redundant growth and physical Hilti validation remain open.

## Automatic association decision and commit backend — 2026-09-30

The backend can now retrieve fresh-session visual targets across all history,
validate metric seeds independently and reject contradictory accepted placements
before using the existing B1 commit and B2 continuation. No externally supplied
target/seed is needed by this API; caller still supplies the frozen observation.
Controlled ambiguity/no-write, retry and independent-process continuation evidence
is in the [packet](../../audit/2026-09-30-auto-association/README.md).
Full core26/26(56.63s) and ROS build pass.

Frontend automatic invocation and bounded pending observations were added in the
subsequent increment above. This backend packet alone does not establish that
integration. Controlled scene updates and growth control remain unfinished.
No Hilti accuracy/performance claim.

## Online metric visual initializer — 2026-09-30

Existing visual retrieval now feeds bounded PnP with frozen observation projection
and original image-time relative pose. A usable seed gets one LiDAR overlap score
and GICP, with zero BBS searches and no alternate initializer after rejection.
Appearance-only/no-consensus cases preserve BBS. Version2 archives freeze projection;
legacy/v1 reads remain compatible and never borrow current calibration.

Controlled production-path tests, full core25/25(62.67s), ROS build and independent
process continuation/reopen pass. Real Hilti quality/performance, automatic session
association and controlled scene updates remain unfinished.
[Evidence](../../audit/2026-09-30-live-pnp/README.md).

## Metric visual archive prerequisite — 2026-09-30

Versioned image evidence preserves visual triangulation and original camera poses;
scene geometry is derived in the original submap anchor and checked against the
persisted observations on reopen/commit preparation. Existing scene/DB owners,
descriptor identities and legacy image encoding remain. Synthetic codec checks,
24 core regressions and independent-process B1/B2 continuation/reopen pass; the
process case is additionally registered and passes as a permanent CTest regression.
This advances persistence beyond appearance-only. Online PnP routing, automatic
association, dynamic scene updates and physical data accuracy remain unfinished.
[Evidence](../../audit/2026-09-30-metric-archive/README.md).

## Hesai input increment — 2026-09-30

Explicit single Hesai input shares all-point absolute-time inspection with the
existing dual adapter and delivers complete scan bounds to the core. TOML/ROS/
launch selection, synthetic decoding/synchronization and ROS callback/exit checks
pass. Actual Hilti clocks, calibration, trajectory and performance await the
user's recording. This is input preparation, not a completed continuous-mapping
capability. [Evidence](../../audit/2026-09-30-hesai-input/README.md).

## Equidistant input increment — 2026-09-30

The explicit camera model now supports Hilti-style equidistant distortion across
tracking, anchor projection and legacy extraction, with synthetic geometry/flow
checks. A small ROS1→ROS2 conversion fixture preserves sensor payloads and times.
Actual dataset compatibility/accuracy and full fisheye FOV retention remain
unverified. Metric archive/PnP, scene updates and automatic association remain
open. [Evidence](../../audit/2026-09-30-equidistant/README.md).

## Dataset transition / geometry delivery — 2026-09-30

User-confirmed unsuitable old sensor layout retires0519as an algorithm tuning or
accuracy benchmark. It remains workload/lifecycle evidence. Hilti2022input and
truth acceptance follow [the transition plan](HILTI2022_ACCEPTANCE.md).
Descriptor-independent delivery of valid same-track geometry is now implemented;
a before/after regression and22/22core checks pass. Live old-bag visual proposals
change1→0, so no net algorithm improvement is claimed. Metric archive/PnP and
continuous scene-update/association goals remain open. [Evidence](../../audit/2026-09-30-visual-depth/README.md).

## Visual recall follow-up — 2026-09-30

Coverage-first replenishment restoresonevisual candidate on the opt-in tracker
recording while keeping600points/threeanchors and existing acceptance gates.
A real-record translation-prior counterfactual verifiesonevisual-only proposal
throughBBS/GICP. This doesnot deliverPnP: the controlled matching probe still has
only0–2eligible metric correspondences. See [current evidence](../../audit/2026-09-30-visual-recall/README.md)
for22/22regressions, real-bag CPU/RSS tradeoffs and unchanged reopen verification.
Historical lifecycle closure and remaining continuous-mapping goals are unchanged.

## Live visual production increment — 2026-09-30

Single-pool tracking, image-time pose interpolation and visual submap boundaries
are now wired under an explicit tracking switch. Same-input live validation,
CPU/memory/database tradeoffs and remaining gaps are recorded in
[CURRENT_STATE](CURRENT_STATE.md#live-visual-submap-producer--2026-09-30) and the
[integration packet](../../audit/2026-09-30-live-visual/README.md). This moves the
tracker beyond utility-only status; metric archive/PnP, successful real visual
retrieval, bounded repeated-scene updates and automatic association remain open.
No historical accepted lifecycle decision is reopened.

## New product work after closure — 2026-09-29

The user's continuous-mapping requirements now drive geometry-only persistence,
removal of online navigation splines, bounded local rolling occupancy and measured
producer isolation. These are not migration-phase continuation or A2/F1 repairs.
See [current product status](CURRENT_STATE.md#new-continuous-mapping-product-work--2026-09-29)
and [design/evidence](CONTINUOUS_MAPPING.md). Metric visual localization/PnP,
controlled repeated-scene replacement and automatic cross-session association
remain unimplemented in the live pipeline; optional 3D fields do not establish them.
A separate bounded visual tracker/affiliation and PnP seed utility is now tested;
the real-image probe has insufficient eligible historical correspondences to
enter PnP. This does not establish metric loop initialization or change the
accepted lifecycle closure. See current product status and its probe packet.

## A1→B3 engineering stage closed — 2026-09-29

**A1 safe historical open; A2 historical runtime reconstruction; W writable
atomic finalized commit; B1 fresh-session attachment; B2 same-session
continuation; B3 publication / checked drain: all ACCEPTED.**
**B3 implementation accepted with documented limitations.** Final independent
closure evidence and the completed authority → readiness → continuation →
attributed publication → checked drain/close boundary are recorded in
[CURRENT_STATE](CURRENT_STATE.md#accepted-lifecycle-closure-and-research-handoff--2026-09-29).

The [algorithm/system invariant audit](../../audit/2026-09-29-algorithm-system-invariant-audit/README.md)
established the following dispositions. F1 is updated by the later accepted
[A2 numerical-support amendment design](../../audit/2026-09-29-a2-contract-amendment-design/README.md);
F2/F3 and native format/model/operation contracts remain unchanged:

| Finding | Category | Disposition |
|---|---|---|
| F1: committed B2 map can exceed A2 reproduction reference | ACCEPTED WITH DOCUMENTED LIMITATIONS / CLOSED | The [bounded implementation](../../audit/2026-09-29-a2-contract-amendment-implementation/README.md) passed [fresh independent implementation review](../../audit/2026-09-29-a2-contract-amendment-independent-implementation-review/README.md). Unchanged similarity references are reproduction diagnostics; hard validity, committed X^R authority, disposable X* and later movement through all-key reconciliation/W/committed materialization remain mandatory. Bounded R5→B1/R6→B2/R7→reopen evidence is not universal divergence or continuation support. F1 mandatory writer settlement is NOT ADOPTED; its full-graph probe is research/diagnostic evidence only. |
| F2: loop temporal/cardinality policy partly defines persisted admissibility | Future compatibility debt | Cleanup **only when a justified temporal/cardinality algorithm change exists**. |
| F3: VisualScene construction recipe participates in historical validity | Future compatibility debt | Cleanup **only when a justified scene/descriptor-selection policy changes**. |

Only F2/F3 are material future compatibility debt from this audit. Other accepted
couplings create no cleanup tasks; operational and other retained limitations
remain separately classified in CURRENT_STATE. The authority/representation/
algorithmic-support/delivery distinction is now explicit in
[MAP_MODELING_PRINCIPLES](MAP_MODELING_PRINCIPLES.md#authority-and-access--established-sapphire-invariants).
No B4 or automatic migration-phase continuation is created. The
[A2 amendment closure](../../audit/2026-09-29-a2-contract-amendment-closure/README.md)
records completed acceptance and final wording/status corrections. A1→B3 remains
ACCEPTED / CLOSED; the amendment is ACCEPTED WITH DOCUMENTED LIMITATIONS / CLOSED.
No further implementation/review phase or mandatory writer settlement follows.
The earlier reproduction characterization, rejection reports and remedy approval
remain period-correct evidence; no historical phase entry is rewritten.
[Closure patch, hashes and preservation](../../audit/2026-09-29-a1-b3-phase-closure/README.md).

## Historical entries and capability snapshot — superseded status

The entries below retain their original evidence scope. All earlier pending-review,
correction gates, unimplemented/broken lifecycle statements and next-phase
instructions are superseded by the accepted closure above. The 2026-09-26
capability table is a historical comparison, not today's lifecycle status;
its F1–F5 labels are distinct from this closure's F1/F2/F3. It creates no new work.

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

## A2.1 — storage validation correction

The independent A2 review found two concrete storage-boundary defects while
accepting the historical runtime architecture. The narrow corrective pass changes
only MapDatabase production readers: producer-derived pyramid allocation bounds
replace the invalid sparse-occupancy ratio; actual SQLite storage classes and
full-width count/matrix bounds are checked before conversion or allocation.
The current writer's empty-image matrix sentinel remains valid. There is no
producer, graph, visual, occupancy, registration, format or A1 opening change.

The real SubmapFrameBuffer 2048-bucket/80-entry fixture, 40 SQLite type/range
mutations and 12 malformed pyramid cases extend the existing A2 tests. Current
execution evidence, field audit, limitations and narrow re-review status are in
the [A2.1 report](/home/user/code/sapphire_git/audit/2026-09-27-a21-correction/README.md).
The broader map-loading capability remains partially absorbed; no continuation,
attachment, T1/T2, localization or A3/A4/A5 work is included. The prior E14
readiness claim below is superseded by this corrective re-review gate.

E15: final Release build and full CTest **13/13 (8.06 s)** passed; corrected
historical/visual tests also passed **2/2** in the sandbox. The real sparse
producer map opens, all 52 new malformed inputs are rejected without file changes
or retained ownership, and the existing 15 failure cases remain passing.
A2.1 is ready for narrow independent re-review; no later phase has started.

## A2 — historical reconstruction, 2026-09-27

The user accepted A1 and authorized this bounded A2 implementation. This update
supersedes historical claims below that A2 is unavailable or unauthorized; it
does not claim full resume/continuation. Existing owners now validate authoritative
DB facts and reconstruct disposable runtime machinery. E14: Release build and
full CTest **13/13** passed, including independent writer/B/C reconstruction and
an actual GPU writer followed by device-independent historical readers.

| Capability | A2 behavior | Verification / remaining boundary |
|---|---|---|
| Historical map loading | Read-only validated Node/Link/payload enumeration; A1 opening preserved | Required-data failures close without mutation; one supported format only; no repair/migration |
| Historical graph and optimizer | Original-odometry prior, persisted directed factors, current noises/Huber Block, fresh ISAM2 | Controlled asymmetric/loop/near-20-km maps within approved numerical bounds; committed poses remain authoritative; no serialization or g2o |
| Pose/spatial retrieval | Rebuild existing indexes from committed poses and stored bounds | Exact committed poses and policy-equivalent candidate membership checked |
| Appearance retrieval | Restore scenes/descriptors and persisted ImageRecord queries; exclude temporal neighbors before search | Fingerprints, membership, ranking and tie policy checked; large IVF training and real-data recall remain outside this evidence |
| Occupancy | Replay persisted LocalGrid with committed poses and compatible occupancy parameters | Exported grid, evidence/owner and active membership checked against canonical replay/refusion; F3/F4 remain unchanged |
| Historical-mode lifecycle | New input rejected, correction unavailable, loop-decision status Unavailable; no mapping worker/device initialization/publication | Repeated independent-process byte/state identity; no new frontend attachment or continuation |

Map loading remains **partially absorbed** as a broader capability: A2 historical
representation/query is implemented and verified within the stated fixtures,
while A3/A4/A5, T1/T2, localization and F2 publication/drain remain excluded.
SQLite `user_version == 1` is only the current format-identity guard and has no
relationship to product version. A2 is ready for independent review; no next
phase is authorized by this completion. See
[CURRENT_STATE.md](/home/user/code/sapphire_git/src/docs/CURRENT_STATE.md) and the
[E14 report](/home/user/code/sapphire_git/audit/2026-09-27-a2-implementation/README.md)
for scope, measurements, test evidence and limitations.

## A1.2 — eligibility pathname-reopen race

Map storage remains **partially absorbed**, and full map restoration is still
unavailable. A1.1 path semantics and `c2:` identity were closed by independent
re-review. The accepted A1.2 robustness fix makes read-only SQLite eligibility
consume a private mapping of the validated/locked inode instead of reopening
the mutable pathname. It preserves sidecar rejection and cooperative content
stability, adds no VFS/manager, and implements no A2 state reconstruction.
PHASE1_DESIGN.md records buffer ownership and header rules; CURRENT_STATE.md
records verification. Final A1.2 independent re-review is the next gate.
E13: Release build and deserialize API check passed; missing-symbol configure
probe failed clearly as intended. Actual production eligibility regressions
passed 12 namespace replacement cases, six failure/cleanup paths, read-only SQL
and full-byte preservation, and malformed header/size rejection. Full CTest
passed 11/11, including all three CUDA tests. No A2 capability is thereby verified.

## A1.1 correction — independent re-review required

Map database / map loading remain **partially absorbed**. The independent A1
review found path-resolution, fingerprint-scope and FIFO-blocking defects. The
corrective pass resolves existing parents through the filesystem, checks a
nonblocking-opened descriptor's regular-file type, and versions the narrowed
occupancy compatibility encoding as `c2:`. No restoration, continuation,
ownership redesign, WAL recovery or algorithm work is included. Full A2 remains
unavailable and unverified. See PHASE1_DESIGN.md for every fingerprint field's
classification and CURRENT_STATE.md for corrective verification evidence.
E12: Release build succeeded; expanded A1/map-open test passed all eleven groups;
unrestricted full CTest passed 10/10, including all three CUDA regressions.
The initial sandbox run could not bind the socket fixture or access CUDA.
No test or algorithm was weakened to obtain passing results.

## A1 update — 2026-09-26

This update supersedes only the safe-open/overwrite parts of the audit-era rows
below. Full map resume remains unavailable and is not verified.

| Capability | Current A1 behavior | Decision / verification boundary | Remaining gap |
|---|---|---|---|
| Map database | Exclusive new-file creation, schema/UUID/config eligibility, insert-only Node identity, stale/conflicting writer rejection | partially absorbed; E11 tests safe opening and storage non-mutation | T1/T2 and completion marker are not implemented; current graph_revision is not yet the full Phase-1 map-state contract |
| Map loading/reloading | Storage resume validates read-only; backend/pipeline report ResumeUnavailable before reconstruction | partially absorbed; eligibility is distinct from restoration | A2 graph/index/occupancy reconstruction, A3 equivalence, A4 attachment, A5 continuation require approval |
| Async / threading | No output or drain redesign in A1 | Existing audit limitations retained | F2 final revision/local output completion remains unresolved |

Long-term abstraction constraints are in AGENTS.md; the approved short-transaction
model, no-new-Snapshot requirement, and A1 stop gate are recorded in
[PHASE1_DESIGN.md](/home/user/code/sapphire_git/src/docs/PHASE1_DESIGN.md).
E11 verification: Release build succeeded; seven CTest cases passed in the
sandbox, and all three CUDA-dependent cases passed outside it after sandbox
device-access failures. The direct A1 test also passed. This does not verify A2
restoration or later continuation/publication contracts.

Snapshot：2026-09-26。Canonical project是Sapphire；Pandora-SLAM为同源后端演进，自研回迁不是third-party集成。基准是Sapphire HEAD `59954c8` 加独立审计时dirty工作区，Pandora对照 `9cd57f21`；本轮仅固化文档。完整状态、证据ID E1–E10和下一phase边界见 [CURRENT_STATE.md](/home/user/code/sapphire_git/src/docs/CURRENT_STATE.md)，长期原则见 [AGENTS.md](/home/user/code/sapphire_git/src/AGENTS.md)。

“Sapphire original”指可核对的HEAD基线，不能保证它是上次session起点。“Pandora behavior”是对源码职责的比较，不表示本轮另行复现其全部运行效果。Decision描述当前吸收方式，不是完成等级，也不是授权下一轮重构。Verification中的verified只覆盖所列实验域。

## Decision vocabulary

| Decision | Meaning |
|---|---|
| retained | 保留Sapphire已有能力，不能算新增迁移成果 |
| merged | 有用能力进入既有职责/数据流 |
| redesigned | 为Sapphire输入/ownership重新设计，不暗含算法效果等价 |
| replaced | 现有能力被另一实现取代，须明确替代语义 |
| partially absorbed | 只吸收部分能力，仍有明确semantic gap |
| rejected | 明确不采用某能力/方案，须有决策理由；不能把尚未迁移冒称已拒绝 |
| not migrated | 当前未吸收或未接线，不暗含将来必须实现 |

## Capability ledger

目录ownership指Sapphire core的语义归属。保留既有文件位置不等于认可其全部抽象，也不授权Phase 1顺手移动文件。

| Capability | Pandora behavior | Sapphire original behavior | Current Sapphire behavior | Decision | Ownership | Verification | Remaining gap |
|---|---|---|---|---|---|---|---|
| Visual submap retrieval | Memory/scene/descriptor检索历史候选并进入验证 | VisualFrame与descriptor召回接口存在，主backend无完整图像检索链 | ORB→appearance scene→Faiss→历史submap ID→既有LiDAR验证 | merged | mapping/visual；ImageMeas/VisualFrame在common | partially verified：E2/E5受控retrieval及候选验证链verified；real-data effectiveness unverified，E6真实候选总0 | 真实recall@K/误检unverified，不把空间loop算视觉收益 |
| Feature aggregation | 几何验证后更新/合并scene的geometry与appearance | 无完整geometric scene fusion | descriptor精确去重、顺序截断≤8192；无3D landmark关联，has_position=false | partially absorbed | mapping/visual | 实际聚合implemented；E2证明其可服务受控检索 | **不是完整geometric scene fusion**；覆盖率/视角多样性未验收，空3D/ray字段有债务 |
| Descriptor/index management | archive upsert/erase、身份、训练、持久cache与重建 | RecallIndexAdapters插槽、MetaTag | DescriptorArchive实际在线使用；旧descriptor adapters默认空；两条接口共存 | partially absorbed | mapping/visual archive；mapping/storage持久scene与索引边界 | E9底层upsert/erase/reinsert/cache verified；生产partially verified | 失效/恢复产品契约未闭合；不在Phase 1清理所有接口 |
| Faiss | Binary descriptor archive及训练/查询/恢复 | 主生产descriptor backend未接入 | BinaryFlat→异步训练BinaryIVF，在线调用真实存在 | merged | mapping/visual；外部库thirdparty/faiss | E2/E9受控查询verified；局部189.5→10.65ms微基准 | 真实召回和整体A/B unverified；Phase 1不调Faiss参数 |
| FBoW | vocabulary/postings可配置检索路线 | 无生产FBoW依赖 | 不使用FBoW；FeatureBlock/scene仍有词汇相关字段 | not migrated | 若未来采用须在mapping/visual明确设计；当前无runtime owner | 生产无接线的源码事实 | 未采用不等于已决定永久拒绝；不能因残留字段宣称支持 |
| Scene persistence | 带身份的scene记录及数据库事务 | 可保存VisualFrame特征 | VisualScene与Node/cloud/grid等同事务保存，fingerprint校验/预取 | merged | mapping/visual编码；mapping/storage提交 | E3记录恢复、E8失败回滚verified | appearance-only scene，无几何融合事务；ImageRecord/scene有descriptor重复 |
| Map database | 分层driver/reader/recovery与持久地图状态 | SQLite Node/cloud/pyramid/grid/link等 | 延用Sapphire DB，增加scene/revision、事务和缓存配合 | partially absorbed | mapping/storage | E3/E8记录write/read及有限回滚verified | **record persistence/read不等于PoseGraph resume**；缺完整恢复/版本/旧图兼容契约，F1 |
| Submap lifecycle | scene/node sealing、merge、retire及重锚定 | Gaussian子图冻结、导航与anchor | 保留Sapphire MargiFrame→SubmapFrameBuffer→SubmapFrame；无Pandora scene merge/retire生命周期 | retained | common跨模块冻结数据；mapping及pipeline负责构造/消费 | E5/E6子图消费及落库partially verified | 停机frontend尾滑窗/reset、跨进程继续未验收；不可机械搬入另一套Frame |
| Loop candidate generation | 图像历史检索与几何验证 | 空间AABB历史候选 | 空间R-tree与视觉候选去重合并，时间排除，visual优先 | merged | mapping/storage检索；mapping/registration候选组织 | E2/E5受控路径、E6空间路径verified | 空间candidate无统一预算；真实视觉贡献unverified |
| BBS | 审计所比较的Pandora视觉回环主线不是这套GPU BBS | Sapphire已有CUDA BBS | 保留原算法；visual扩大窗口，使用保存的pyramid和Gaussian均值 | retained | mapping/registration适配；thirdparty/3d_bbs算法 | E4非对称变换、E6录制空间回环verified | 非任意6DoF冷启动；roll/pitch沿用初值；不是新增迁移收益 |
| GICP | 几何配准能力 | Sapphire已有small_gicp链 | 冻结Gaussian均值/协方差供给，target KD-tree准备和收敛门限 | retained | mapping/registration；外部small_gicp依赖 | E4恢复/拒绝、E5落边、E6真实子图配准verified于各自条件 | 无重复DB读target的已证实bug；preparation成本及一般失败分布尚未完整测量 |
| Pose graph | 图恢复、优化、scene处理与持久状态 | ISAM2、odom/loop factors | 保留ISAM2，接入LiDAR验证后的视觉候选loop，pose/link事务 | partially absorbed | mapping/graph；外部GTSAM在thirdparty | E5 inverse与SQL方向强验证；E6优化重放一致 | runtime/graph恢复broken(F1)，revision publication契约缺口(F2) |
| CAPE | organized DepthFrame上的平面分割 | 无CAPE | Gaussian centroid→内部XY lattice→CAPE；没有新DepthFrame | redesigned | mapping/grid adapter；thirdparty/cape外部算法 | E7平地synthetic verified；E6 **14子图patch全部0** | 接入算法不等于输入语义等价或真实有效；**real-data effectiveness尚未证明** |
| Ground estimation | 水平floor reference及筛选 | 固定高度分类 | CAPE不足则Gaussian平面回退，scalar horizontal floor供给LocalGrid | partially absorbed | mapping/grid | E7平地/墙等有限验证；10°坡反例；E6全部走回退 | 一般坡面broken(F4)，profile margin语义待确认；Phase 2先设计，不调参掩盖 |
| Gridmap | 多层局部/导航地图及地面流程 | LocalGrid、occupancy tile融合 | 保留Sapphire occupancy及LocalGrid持久化，接入新的floor输入 | partially absorbed | mapping/grid；mapping/storage保存LocalGrid | E3 LocalGrid读回、E6重融合/发布存在；partially verified | 一般roll/pitch投影broken(F3)；多视点ray origin、traversability未定义完整 |
| Map loading/reloading | 历史nodes/poses/links、graph与索引恢复 | Memory可读部分记录，PGO从0初始化 | 改善Memory读/索引重建；PGO仍不恢复 | partially absorbed | mapping/storage + mapping/graph + pipeline入口 | E3记录读取verified；E10实际resume/覆盖反例broken | new/resume/localize入口和完整continuation未实现；Phase 1 A |
| Index persistence | Memory启动/关闭接线archive save/restore，验证身份后恢复/回退重建 | 无完整descriptor缓存恢复 | archive方法移入并底层可用；生产VisualSubmapIndex未调用save/restore，按历史ID推进重建 | partially absorbed | mapping/visual archive；storage历史scene事实来源 | E9底层cache consistency verified；生产restore未接线 | Phase 1需正确恢复视觉历史/身份，可重建；缓存提速与调参不属于必做扩展 |
| Cold-load / cache | 分离冷scene/feature/graph数据并预取 | 多次直接DB访问 | cloud/grid按字节LRU、scene按身份预取，reader共享const快照 | merged | mapping/storage；通用LRU在tools | E8命中/淘汰/失效/in-flight测试verified | 提前target加载、先scene读取后min_matches过滤、无fingerprint读绕过缓存(F5) |
| Memory management | 工作集和scene生命周期管理 | 轻量全局索引+DB | payload cache有预算，global Faiss/graph/spatial/occupancy/队列仍增长 | partially absorbed | 各mapping owner负责预算与生命周期；Memory管理payload caches | E8局部预算verified；E6单次RSS仅样本 | full-process RSS scaling与背压unverified；局部LRU不是总内存上限 |
| Async / threading | graph/grid/scene等异步owner | odometry/mapping/PGO线程 | 保留现有线程，增加scene prefetch与Faiss sample训练 | merged | pipeline / mapping/graph / storage / visual各自owner | implemented；E1/E6正常运行、E8部分生命周期测试 | F2输出版本，异常/drain可见性；TSAN、长期并发和完整吞吐未验收 |

## Scope discipline for the next implementation

Phase 1仅为map lifecycle/persistence与optimization revision/publication建立正确契约并验证。恢复历史视觉索引、LocalGrid/occupancy属于所需状态恢复；更换视觉/ground算法、修F3、调CAPE/Faiss/BBS/GICP、删FeatureBlock和大规模结构清理不因此获得授权。

后续每次变更按capability更新Current behavior、Verification和Remaining gap；Decision仅使用上表定义，不用“done”覆盖semantic gap。若新增metric视觉能力或改变LocalGrid/ground表示，先进入独立architecture/coordinate设计及对应phase，不从现有scene字段推断能力已经具备。
