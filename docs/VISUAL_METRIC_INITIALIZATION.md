# Visual submap triggering and metric loop initialization

## Online PnP integration — 2026-09-30

VisualSubmapIndex now retains exact query descriptors, original image-time camera
pose relative to the submap, and frozen rectified-pixel projection. For each final
top_k target it selects the view with most metric support and makes at most one
bounded PnP attempt. The existing support/coverage/reprojection gates remain.
A usable T_target_query supersedes the spatial initial guess for that same target;
missing support or unsuccessful PnP preserves the existing BBS route.

The PnP route scores LiDAR overlap once at the proposed pose (existing CUDA
score_pose, no BBS search), then uses existing GICP acceptance. Either geometric
rejection discards that target without trying another initializer. The persisted
factor remains query→target with inverse(T_target_query). This applies to ordinary
new-map/continuation loop verification, not automatic fresh-session attachment.
Manual B1 attachment still follows its existing registration path.

VOM family version2 freezes width,height,fx,fy,cx,cy alongside image evidence;
version1 and legacy bytes remain readable. No current calibration is substituted
for absent historical projection. Old metric target points remain usable, while
old query pixels without projection remain appearance-only. SQL schema, c2/c3
identity, solver/writer contracts and navigation omission are unchanged.

Build and controlled verification results are recorded in the
[current packet](../../audit/2026-09-30-live-pnp/README.md). Real Hilti performance,
accuracy, automatic association and controlled repeated-scene updates remain open.

## Metric archive integration — 2026-09-30

This supersedes the appearance-only storage status in the historical sections below.
VOM1/v1 image BLOBs now preserve original camera poses, local track IDs and visual
triangulation summaries. Legacy images remain byte-compatible. The deterministic
scene recipe transforms camera-frame points by inverse(T_odom_submap) * T_odom_camera;
only posed observations can produce metric target landmarks. A2 and W validate this
relationship using original anchors, never optimized map poses. The descriptor-first
identity/order/capacity rule is retained, so later duplicate descriptors do not
borrow/replace the first row's geometry. This does not implement scene fusion.

Codec/coordinate/corruption tests and independent-process write/reopen/B1/B2/reopen
verify retained metric evidence, index identity and absence of archived navigation.
Parallax/reprojection/count are quality summaries, not covariance or physical-truth
validation. [Implementation and evidence](../../audit/2026-09-30-metric-archive/README.md).

At the archive milestone, PnP routing was the next integration gap (now implemented above): preserve image-time query camera pose
relative to its original anchor with the exact descriptor snapshot; select bounded
support for each target; a usable seed must enter independent LiDAR verification
without BBS. Missing/insufficient metric support may retain BBS initialization;
independent geometric rejection must not trigger a second initializer. None of this
is automatically delivered by storing 3D coordinates. Real acceptance waits for the
user's Hilti recording/truth; old0519 remains engineering-only.

## Binding product decision — 2026-09-29

The user explicitly rejected image-feature/LiDAR-point association after prior
experiments: do not project Gaussian/LiDAR geometry to assign visual depths.
The preceding association proposal was documentation only and was not implemented.
Vision has exactly two roles: decide when to open a submap and provide a coarse
metric pose for loop registration. It does not estimate/update Sapphire odometry.

Use a lightweight bounded pyramidal optical-flow tracker and visual triangulation.
Read existing LIO odometry as a one-way metric motion input to triangulation;
never feed visual residuals back to odometry. This avoids pixel-to-LiDAR association
but still requires image timing, camera intrinsics and camera-to-body calibration.
A monocular visual baseline without a metric motion source cannot supply metres.

## Reference inspected; adapt capabilities, not structure

- `/home/user/code/tmp/vslam/src/core/module/keyframe_inserter.cpp`: tracked and
  reliable reference support, view-change ratio, minimum insertion interval,
  and refusal to insert from unstable tracking. Its BA scheduling gates and
  mandatory timeout insertion do not define Sapphire's submap policy.
- `/home/user/code/tmp/vslam/src/core/solve/two_view_triangulator.cpp`: parallax,
  cheirality, reprojection and scale-consistency checks.
- `/home/user/code/pandora_slam/backend/exterior/src/solver/motion_triangulation.cpp`:
  known camera poses, metric baseline, ray triangulation and two-view verification.
  Adapt these mathematical contracts; use lightweight flow for temporal matching.
- Sapphire currently samples images at 2 Hz for independent ORB retrieval and
  closes submaps by travelled distance, with frame/point capacity caps. That is
  not yet the requested visually driven submap policy.

## Ownership and bounded execution

One small VisualTracker per enabled camera belongs to the existing mapping
worker. It owns only the previous grayscale image and capped feature tracks,
including first/previous observation rays/poses. No new thread or map owner.
It consumes timestamped images and optional image-time original odometry poses;
its outputs use existing VisualFrame/VisualPoint with optional geometric evidence
and explicit tracking/trigger diagnostics. Reset discards the tracker domain.
Track cap, pyramid levels, image dimensions and maximum tracking gap are explicit.
Forward/backward flow rejects inconsistent tracks; reseeding is spatially spread.
Descriptors are computed for selected visual keyframes, not every tracked frame.

There is one feature population per camera: detection/replenishment, optical flow,
triangulation and descriptor rows share positions and track identities. Replenish
under-covered image cells in this population; do not independently detect a second
backend population. A track ID denotes temporal correspondence within the tracker
domain, not a persistent scene identity or a cross-session match. Descriptor
filtering/reordering must preserve the explicit row-to-track association.
Tracked points need refreshed descriptor orientation and an explicit scale policy;
passing provided keypoints to OpenCV ORB does not run its detection/orientation
stage. LK pyramid levels are search levels, not automatically descriptor octaves.

A new visual keyframe/submap request requires adequate current feature count and
image coverage plus loss of reference overlap or substantial view displacement,
with a minimum interval. Missing/black images do not repeatedly create empty
submaps. Pure rotation can change the field of view and justify a submap, but
cannot fabricate triangulated depth. Existing point/frame caps remain explicit
resource fallbacks; distance must cease being the primary trigger when the visual
policy is enabled. All actual flush/ID/commit handling stays with SubmapFrameBuffer
and existing backend ownership.

Visual keyframe selection and actual submap boundaries remain distinct decisions:
an observation may help triangulation or appearance coverage without closing the
submap. Compare submap overlap against its accepted reference, not automatically
against every intermediate descriptor frame. A tracking-quality gate assesses
whether this observation is usable; it does not require a stable environment.

## Grid / feature-change investigation — 2026-09-29

Inspected local Pandora implementation:

- `frontend/ov_msckf/src/utils/FrontendKeyframeSelector.cpp:109-118` computes
  `renewal = 1 - shared_reference_IDs / current_unique_IDs`.
- The same selector uses `grid_occupancy >= min_grid_occupancy` as an eligibility
  gate, followed by visual renewal or motion fallback and a minimum interval.
  Its reference IDs change only on accepted keyframes.
- `frontend/ov_msckf/src/utils/TextureScore.cpp:52` defines occupancy as cells
  containing any feature divided by all image cells. It does not measure temporal
  per-cell appearance change. The additional texture-health score combines
  occupancy, feature count and displacement; it is not a scene-change estimator.
- `backend/exterior/src/registration/scene_update.cpp:102` separately measures
  spatial coverage of checked original PnP seeds on a 4-by-3 image grid. That
  coverage belongs to association support, not keyframe novelty.

Equal grid occupancy can hide completely different features. Conversely, blur,
occlusion, dropped images or detector churn can produce high ID renewal without
new scene content. A moving foreground can dominate feature counts. These are
failure cases to measure, not evidence that grid coverage is useless.

The proposed Sapphire baseline keeps separate, interpretable measurements:

1. Current usable track count and occupied-cell fraction.
2. Reference retention `shared / reference_count` and current renewal
   `1 - shared / current_count`; zero denominators mean unavailable evidence.
3. Spatial distribution of surviving reference support, with bounded per-cell
   contribution so one textured patch does not dominate. Group reference support
   by reference-image cells; feature movement across cells is not itself ID loss.
4. Track age, forward/backward consistency, view displacement and geometric
   support. Newly seeded one-frame points do not immediately count as reliable
   novelty. Recovery after blackout is attributed as recovery, not proven novelty.

This is a candidate decision policy, not an already validated new algorithm.
Thresholds, persistence/cooldown and tracking cadence require bag evaluation.
The existing 2 Hz descriptor sampling interval must not silently become the LK
tracking cadence: test bounded higher-rate tracking separately from sparse
descriptor production, including overload and image-gap recovery.

Primary online references and selection implications:

| Reference | Relevant capability | Decision for Sapphire |
|---|---|---|
| [FastORB-SLAM](https://arxiv.org/abs/2008.09870) | Pyramidal sparse flow between frames; descriptors only on keyframes | Supports the split in computation. Published RGB-D speedups are not Sapphire measurements. |
| [VINS-Mono tracker source](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/feature_tracker/src/feature_tracker.cpp) | Retains feature IDs/ages, favors long tracks, replenishes spatial gaps | Adapt feature ownership and replenishment; do not introduce visual odometry. |
| [GMS, CVPR 2017](https://openaccess.thecvf.com/content_cvpr_2017/html/Bian_GMS_Grid-based_Motion_CVPR_2017_paper.html) | Uses local motion statistics to reject false correspondences | Optional loop-match filtering experiment. It is not a grid novelty/submap selector and does not replace geometric verification. |
| [XFeat official implementation](https://github.com/verlab/accelerated_features) | Lightweight learned detection/description; supports XFeat with LighterGlue | Candidate comparison for difficult revisit matching. Evaluate descriptors at the shared tracked points; do not add a permanent parallel feature population. Include index/storage and inference costs. |
| [LightGlue official implementation](https://github.com/cvg/LightGlue) | Adaptive sparse correspondence matching | Potential candidate-pair matcher with compatible features, not a per-frame LK replacement. CPU cost and candidate multiplicity matter. |
| [OpenCV optical-flow API](https://docs.opencv.org/4.13.0/dc/d6b/group__video__track.html) and [ORB source](https://raw.githubusercontent.com/opencv/opencv/4.x/modules/features2d/src/orb.cpp) | Reusable flow pyramids; supplied-keypoint descriptor computation | Reuse flow pyramids and explicitly calculate descriptor orientation/row identity; do not assume LK and ORB pyramids are interchangeable. |

Provisional implementation baseline: spatially balanced single-pool corners,
pyramidal LK, sparse oriented binary descriptions on those same points, visual
triangulation with read-only metric motion, then PnP and independent LiDAR
verification. Detector choice (FAST versus Shi–Tomasi) is evaluated jointly with
descriptor repeatability, not on tracking lifetime alone. GMS and learned matching
are challengers if the baseline's measured failures justify them, not mandatory
stages stacked into every frame.

Selection evidence must compare the same inputs, hardware and workload, separate
parameter tuning from held-out route segments, and report tracking/descriptor/
retrieval/verification p50/p95/p99, CPU, peak memory and queue delay. Quality measures
include spurious/missed submap boundaries, useful triangulated support, verified
revisit recall at controlled false acceptance, PnP initialization success, BBS
fallback frequency, and database growth per repeated route. Match counts or
extractor FPS alone cannot select the winner. Include rotation, rapid motion,
blur, low texture, repeated structures, illumination changes and moving foreground.
No comparative benchmark or universal-best claim is established by this research.

## Lightweight keyframe affiliation — user refinement, 2026-09-29

The first real-bag tracker probe produced 161 submap candidates in 200.9 s.
Preserving live tracks across grid-cell quotas reduced that only to 159 on the
same initial export. This is evidence against enabling raw track-renewal cuts,
not a reason to tune a threshold until the output count resembles the old mapper.
The corrected direct-grayscale export also produces 159 candidates. They are
diagnostic requests in an offline evaluator; no production submaps were cut.

Further inspected vslam paths:

- `src/core/module/frame_tracker.cpp:20`: match previous-frame visual landmarks
  by projection before checking available support.
- `src/core/pipeline/tracking_module.cpp:747`: exclude already observed landmarks,
  project local candidates, check visibility/view/scale, then recover associations.
- `src/core/matcher/projection.cpp:1045`: query nearby image-grid points, compare
  descriptors, apply ambiguity checks and associate landmarks.
- `src/core/module/keyframe_inserter.cpp`: reference reliable-landmark support
  participates in insertion after association. Its pose optimizer, full local
  covisibility map and BA scheduling are outside Sapphire's visual role.

The next implementation must distinguish a short optical-flow segment from its
visual anchor association. A new segment may reacquire a known feature; a new ID
is not automatically new scene evidence. Reacquisition must not splice a gap into
a supposedly uninterrupted triangulation track: retain new-segment observations
and validate their geometry independently. Use the same current feature pool,
with explicit anchor association metadata rather than a second detector stream.

Proposed bounded ownership: extend the mapping-worker-owned VisualTracker with a
small fixed number of recent representative visual keyframe records (initial
experiment budget 3, capped total landmark/descriptor count). They retain poses,
visual points/descriptions and observation provenance, not a global covisibility
graph. Use bounded candidate scans and the existing image grid for local lookup.
No unbounded pairwise keyframe links, all-history projection or visual pose BA.
Actual image/point budgets and eviction must be measured before enabling this.

The user's scaling constraint is binding: preserve repeatable association and
reference selection without a visual covisibility graph. Local image processing
must have work bounded by image dimensions, live-feature count, active-anchor
count, projection candidates and local search visits, independently of total
historical keyframe count. Never recursively expand keyframe neighbors or retain
an unbounded per-landmark observation list. Historical revisit discovery remains
index retrieval with a bounded candidate shortlist; it is separate from local
frame affiliation. The existing LiDAR pose graph is not a visual covisibility
graph. Benchmarks must grow archive size while holding local workload fixed and
show that local affiliation costs stay bounded; index/pose-graph growth is measured
separately and is not declared solved by this design.

Processing order and failure semantics:

1. Continue surviving tracks; carry valid existing anchor associations. Grid
   counters used in selection/replenishment directly yield total and mature
   coverage. Cell quotas restrict new detections, not moving live tracks.
2. Project candidate **visual-triangulated** landmarks using read-only image-time
   odometry. Discard behind-camera, outside-image, unsuitable view/scale and
   excessive-uncertainty predictions. Prediction is a search prior, not evidence
   that a point was observed. No LiDAR/Gaussian points participate.
3. Search existing unassociated points near predictions. Replenish deficient
   areas of the same point pool, prioritizing bounded projection neighborhoods,
   then uncovered grid cells. Compute descriptions only for selected frames or
   the capped subset needing reacquisition. Enforce one-to-one assignment,
   descriptor distance/ambiguity and reprojection consistency; do not bind a point
   based only on proximity. Confirm tentative reacquisitions over time before
   counting them as reliable support or merging triangulation evidence.
4. Failed predictions do not consume observed coverage or permanently mask cells.
   New points without an anchor remain pending until temporally supported;
   they can then represent new content. Missing points alone do not establish
   environment removal. Dynamic/occluded content must not pin stale masks.
5. Choose the best-supported bounded local anchor after association, considering
   spatially distributed support. Separate 2D temporal affiliation from 3D metric
   support. Count currently observed unexplained content and visible-anchor
   support separately; out-of-view landmarks are not tracking failures.
6. Decide independently whether to attach this observation, retain another
   representative visual keyframe inside the current submap, or request a new
   submap. Require persistent, usable unexplained content for a visual boundary;
   low-quality tracking enters recovery. A bounded anchor replacement policy must
   preserve submap coverage accounting: rotating reference anchors forever cannot
   silently expand the submap forever. Existing frame/point capacities remain
   explicit resource flushes, not disguised visual novelty.

Verification before production: blackout/reappearance restores association without
claiming continuous flow; repeated texture cannot bind by proximity alone; moving
occluders cannot permanently reserve cells; returning to a retained viewpoint can
reattach; view expansion can add an internal keyframe without cutting a submap;
true new content eventually cuts; anchor/point/lookup budgets remain bounded.
Report attribution of boundaries (novelty/recovery/resource), false affiliation,
retained representative coverage, triangulation support and incremental CPU.
The raw-tracker probe did not implement this affiliation design. The subsequent
[bounded-affiliation increment](../../audit/2026-09-29-visual-affiliation/README.md)
implements local anchor retention, projection matching, temporal confirmation and
separate internal-keyframe/boundary decisions in the core utility. Production
integration and general association-quality validation remain pending.

Implementation experiment: keep at most three owned VisualFrame anchor
records in VisualTracker, each no larger than its live feature budget. Do not
evict an anchor to continually move an active submap's coverage. A new
representative observation can fill a free slot; after those slots are occupied,
persistent unexplained coverage may request a boundary. Explicit boundary
acknowledgement clears that local coverage set and seeds the new one. This is
separate from later archival representative selection and global scene updates.
Flow gaps preserve these local anchors in the same odometry domain; explicit
reset discards them. New segment IDs remain distinct from confirmed anchor-point
associations. Projection, descriptor and one-to-one checks establish a tentative
link; at least one later consistent observation is required for confirmation.
No inherited anchor depth is presented as newly triangulated segment geometry.
Anchor image observations/descriptions/poses stay fixed. Geometry absent when an
anchor was selected may be filled later from the same continuous feature segment,
transformed into that anchor's camera coordinates and checked against its pixel.
This records at least three original triangulation observations; its maximum
reprojection error also includes the extra anchor check. It never splices a newly
reacquired segment into old triangulation history.

## Coordinates and triangulation

`T_A_B` maps B into A. Interpolate original body odometry at the image timestamp;
reject extrapolation and large bracket gaps. Compose
`T_odom_camera = T_odom_body(t) * T_body_camera`. Optical pixels are undistorted
before constructing rays. Triangulate temporal rays using their known relative
camera poses, require metric baseline, angular parallax, positive depth and
reprojection agreement in both reference/current and an intermediate observation.
Keep only finite, bounded depths with explicit observation count and quality.
Camera-frame feature geometry converts to the submap's original body anchor at
finalization, never through a later committed/global correction.

For loop initialization, match historical 3D visual landmarks to current 2D image
features and solve PnP. Convert its optical-camera result explicitly to
`T_target_query`; independent LiDAR geometric verification still decides the loop.
BBS is fallback when reliable visual initialization is absent. No visual odometry,
visual BA frontend or LiDAR-derived visual depths are part of this design.

## Persistence and verification sequence

First test tracking/triggering/triangulation with the production utility, before
wiring it into submap production. Extend existing VisualPoint/VisualFrame for
optional geometry and observation pose; existing appearance-only extraction keeps
those absent. This first step does not claim the geometry is already persisted.
Before enabling end-to-end, define versioned, checked storage of real visual
landmarks/provenance and preserve legacy appearance-only files. Do not silently
mark legacy zero positions as measured geometry or weaken historical validation.

Tests: image translation, pure rotation, low texture/blackout/recovery, reference
support and cooldown, forward/backward mismatch, insufficient baseline/parallax,
three-view reprojection, nonidentity camera extrinsics, domain reset, capped tracks
and images. Then real bag measurements of tracking cost, usable triangulated
points, submap trigger reasons and memory. Finally PnP/GICP on real revisit pairs,
wrong candidates, asymmetric factor direction, and independent-process persist →
reopen → associate → continue → reopen. Self-pair success is not revisit evidence.

Status: the bounded single-pool tracker, three-view triangulation, supplied-point
descriptors and raw overlap diagnostics are implemented as a tested core utility.
The diagnostic selector is not enabled in the live pipeline. Bounded anchor
affiliation is implemented and partially verified by the later packet above;
metric persistence, PnP and live submap integration remain pending.
See the [probe packet](../../audit/2026-09-29-visual-metric/README.md).
A1–B3/A2-F1 closure remains intact.

## Bounded visual PnP seed — implementation contract, 2026-09-29

The next utility consumes the existing immutable scene FeatureMap, the exact
query FeatureBlock and its scene::Match row indices. Reusing these avoids a second
matcher or reordered VisualFrame rows. It owns no archive, graph, thread or global
landmark list. The caller supplies one retrieved candidate at a time; pair and
RANSAC/refinement budgets are explicit. A bounded candidate shortlist is still
the caller's responsibility. No local covisibility expansion is introduced.

Query keypoints must be rectified pinhole pixels with the supplied processing K;
distortion is not applied again. Target positions are visual-triangulated metres
in the target submap anchor. T_query_camera maps the query image-time optical
camera into the query submap anchor, including motion since its first body pose.
OpenCV estimates T_camera_target; the returned seed is
T_target_query = inverse(T_camera_target) * inverse(T_query_camera).
Image-time original odometry supplies this local motion; committed map poses do
not substitute for it. No LiDAR point provides visual depth.

PnP RANSAC and bounded refinement are followed by rechecking positive depth,
residual, inlier fraction, image coverage and noncollinear metric support. Planar
support is allowed but these gates do not resolve all planar/repeated-structure
ambiguities. Success means a registration seed only, never loop acceptance or
permission to mutate a historical scene. Independent LiDAR acceptance remains
mandatory; missing geometry or rejected seeds leave BBS fallback available.
Malformed inputs fail explicitly; exhausted pair budgets yield no partial seed.

VisualTracker may export detached copies of its bounded retained anchors after
later same-track geometry enrichment. Copies retain each original camera pose
and optical-frame geometry; conversion into a submap anchor belongs to the caller.
Export precedes boundary acknowledgement. This adds no persistence format and
does not make legacy VisualFrame encoders preserve geometry. No live pipeline,
database validator, solver schedule or closed lifecycle contract changes here.

Verification: asymmetric camera/submap transforms and inverse, noisy/outlier
correspondences, wrong candidates, appearance-only scenes, collinear/narrow image
support, invalid indices/duplicate assignments, quantized row ordering, pair
budget exhaustion and detached export ownership. Real revisit plus LiDAR checks
and live/persistent integration remain separate required evidence.

Implemented as an independent core utility; [the probe packet](../../audit/2026-09-29-visual-pnp/README.md)
records 2/2 targeted tests and a PnP access/UB sanitizer pass. The recorded-image
probe does not establish PnP success: eligible historical candidates never reach
20 appearance matches (maximum16; maximum metric matches6), so no solve occurs.
This localizes the current evidence gap before the solver. Investigate repeatable
descriptions, representative-view selection and bounded candidate matching using
known-overlap revisit pairs; neither relaxed thresholds nor larger graph walks
are established remedies. Production integration and metric persistence remain
pending. The existing archive's global reverse/work-array passes and index search
also require separate scaling measurements; the local tracker bound does not
make global retrieval or the LiDAR pose graph constant-cost.

## Primary visual initialization, BBS for unavailable visual seeds — 2026-09-30

The user clarified that future visual index observations support the primary
initialization path. A usable visual PnP seed proceeds directly to independent
LiDAR/GICP verification; do not also run BBS for the same usable initialization.
BBS supplies a coarse initial pose when visual initialization is unavailable.
Keep insufficient/unavailable visual evidence distinct from an independently
geometrically rejected candidate; changing solvers must not bypass rejection.
No visual residual feeds odometry, and no LiDAR point supplies visual depth.

Narrow FOV, foreground change and occlusion can remove image correspondence while
LiDAR still observes useful background geometry. Empty visual retrieval must not
veto independently discovered geometric candidates. Current spatial retrieval
depends on pose/bounds overlap; it is not global LiDAR place recognition under
arbitrary drift. The live metric visual path remains unimplemented.

The [follow-up probe](../../audit/2026-09-29-visual-revisit/README.md) fixes an
audit-only omission of the first scene when its first image lacked a pose, while
later images had poses. Corrected archive:44scenes,28,281points,6,092metric points.
Exhaustive diagnostic matching within the top5eligible index candidates yields
at most17correspondences and8metric correspondences, still below the unchanged
PnP gate. This is insufficient evidence to blame the index or select another
detector. Visually overlapping early/late images exist; representative selection,
visual depth availability and descriptor repeatability require separate checks.


## Live tracker / submap producer integration — 2026-09-30

`visual_loop.tracking_enabled` now selects the existing VisualTracker inside the
mapping worker. `tracking_interval` (default0.05s) is separate from legacy
`image_interval`; default-off retains the legacy appearance extractor. Enabled
tracking uses at most600points and up to3anchors per camera, further bounded by
max_features/max_frames. No second detector, worker or pose estimator is added.
The pending image queue is capped at64packets/32MiB pixels; the active extracted
batch is separately bounded by the same limit. Flow gaps remain explicit.

Consecutive marginalized original body poses bracket image time, with linear
translation/SLERP rotation and a maximum0.25s gap, then optical-camera extrinsics
are composed. Outside that bracket, tracking remains2D-only. No extrapolation or
LiDAR point association. A domain reset clears both pose bracket and all trackers.

Visual novelty flushes the actual SubmapFrameBuffer. Resource limits remain256
marginal frames/24,000Gaussian entries; distance is a fallback only after1s without
usable visual observations. Before any flush, bounded enriched anchor observations
are copied into the existing submap. `beginSubmap()` clears local anchor affiliation
and forces a fresh representative on the next usable image while preserving
continuous flow and original triangulation observations. It requires no current
described frame, unlike the utility's explicit visual-reference acknowledgement.
IDs, backend admission, publication and checked finish keep their existing owners.

**Storage is still appearance-only.** Runtime metric evidence is measured but not
silently reinterpreted as persisted geometry. No metric PnP/GICP path is enabled
by this switch. The deterministic appearance scene recipe and legacy archive
validation remain unchanged. This is an intermediate delivery toward the required
metric archive/initialization, not an alternate final design.

Design and live validation results are recorded in the
[integration packet](../../audit/2026-09-30-live-visual/README.md).

## Coverage-first replenishment / recall evidence — 2026-09-30

Replenishment retains the ordinary equal-cell quota as its first pass, then spends
spare capacity up tothreequotas in textured cells, under the same600point limit.
GFTT spacing is5px. New detections are admitted once; existing tracks are not pruned.
Mature-coverage gates and equal-cell affiliation prevent dense foreground texture
from replacing image-wide support. No scene recipe or persistence contract changes.

Same-input live replay restoresonevisual candidate (previous trackerzero; legacy
independent ORBone). A translation-only1kmprior shift in a read-only real-record
probe eliminates spatial candidates yet visualquery40→target1passes BBS/GICP.
This is not metricvisualPnP or arbitrary-orientation/cross-session localization.

The fixed-boundary controlled probe reaches28/23descriptor pairs but only2/0metric
pairs; both PnP entries refuse insufficient geometry before solving. More views,
doubled point budgets and the tested multiscale variant did not improve enough and
were not adopted. Next diagnosis must inspect matched-feature geometry availability,
not infer readiness from aggregate runtime depth rows. Anchor enrichment currently
occurs only on described frames; investigate its effect before changing the policy.
A usable visual pose should skip BBS and enter independent LiDAR verification;
BBS supplies initialization when no usable visual pose exists. Never use a second
initializer to bypass geometric rejection. [Design, tests and measured scope](../../audit/2026-09-30-visual-recall/README.md).

## Intermediate geometry delivery and benchmark scope — 2026-09-30

Retained anchor geometry is now enriched from the bounded live track pool on each
posed observation, independently of descriptor scheduling. Same uninterrupted ID,
original anchor-camera coordinates, positive depth and anchor reprojection checks
remain mandatory. No cross-gap borrowing, extra descriptions or LiDAR-associated
depth. A static initial anchor followed by translation between descriptions proves
the omission in old code and verifies the fix. Full core22/22and ROS build pass.

The real fixed-boundary diagnostic still has only5/2metric matches; no PnP solve.
The live old-bag candidate count changes1→0 as richer anchor geometry changes local
association/selection. Preserve this negative result. The user has confirmed that
this recording's layout does not satisfy the algorithm's requirements, so further
geometry/appearance tuning on it stops. It supplies engineering load evidence only.
Continue input/model support for [Hilti2022](HILTI2022_ACCEPTANCE.md); metric archive,
PnP quality and continuous-map behavior still require implementation/acceptance.
