# Continuous mapping: product capability work

Started 2026-09-29 following explicit user authorization to autonomously implement
a usable core of long-running, revisitable, changing-environment mapping (roughly
"60%", not a measurable completion percentage). This is new product work, not a
reopening of A1–B3 or A2/F1. Existing accepted invariants remain requirements.

## Automatic association backend increment — 2026-09-30

An explicit backend API now searches all historical visual targets for an unrelated
new producer domain, verifies bounded PnP candidates using LiDAR/GICP, rejects
conflicting map placements and uses the existing B1 commit/B2 continuation owners.
Its controlled process/rejection evidence is in the
[association packet](../../audit/2026-09-30-auto-association/README.md).

The subsequent [frontend increment](../../audit/2026-09-30-auto-frontend/README.md)
connects this API to opt-in automatic resume with a bounded ordered startup queue.
Later images can initialize the original root while its own geometry independently
verifies the constraint. Normal shutdown/cancellation and ordered B2 handoff are
covered by controlled tests. The first root must overlap history; arbitrary new
prefix merging and durable pre-association retention remain unsupported. The
8-submap/128-MiB default envelope fails explicitly if exhausted before attachment.
Loaded history is still not a live correction. Controlled revisit updates and
persistent redundancy remain separate unfinished objectives.

## Revisit payload refresh preparation — 2026-09-30

The frozen repeated-route storage inventory identifies scene payloads as the main
durable cost: 353.42 MiB of 355.16 MiB file size across 94 nodes. New profiling of
accepted GICP measures query and target Gaussian support separately. High query
inliers alone cannot justify replacing an old wide-coverage scene with a narrow
current observation. Shape-aware unique pairs, spatial-support counts and explicit
point/comparison budgets supply diagnostic evidence without changing loop acceptance.
[Packet](../../audit/2026-09-30-revisit-coverage/README.md).

The intended next implementation must make active scene payloads distinct from
retained graph/producer identity, replace covered content transactionally and refresh
retrieval/cache membership after commit. Geometry coverage is insufficient to retire
visual views, and endpoint clusters do not prove free space or disappeared objects.
Do not invent deletion visibility, require static-scene stability or transplant
Pandora's graph contraction. Existing per-node geometry is mandatory under current
storage validation, so actual retirement requires explicit format/reader/commit
support. This diagnostic increment does not claim persistent growth control.

## Controlled persistent refresh implementation — 2026-09-30

The [scene-refresh increment](../../audit/2026-09-30-scene-refresh/README.md) implements
explicit scene membership in opt-in c4/schema3, atomic retirement after the new
observation commit, cache/descriptor materialization, historical validation and
B1/B2/new-map invocation. Complete old Gaussian coverage and metric/descriptor
preservation of the old indexed visual scene guard replacement. Uncertain or novel
views remain; small observed geometry changes can replace the old representative.
Retirement also checks measured versus committed placement (0.02m at the retained
support points); a finite mismatch simply keeps both observations. A second COMMIT
with unknown outcome cannot be hidden by the known first observation commit.

This delivers a conservative update/growth-control path for covered scenes, not
universal fusion or object-disappearance inference. Synthetic repeated revisits
reuse storage pages and keep active payloads bounded in the tested sequences.
Graph/trajectory/spatial summaries still grow, and scanning historical spatial
candidates still has historical-scale cost before active eligibility filtering.
The subsequent [active-spatial increment](../../audit/2026-09-30-active-spatial/README.md)
removes retired identities from R-tree lookup while preserving the dense metadata;
thus retired overlap enumeration is no longer part of hot spatial recall. Cold
reconstruction, graph state and all-key reconciliation still scale with history.
End-to-end CPU/RSS and actual Hilti effectiveness of this new format remain to be
measured. Earlier workload isolation evidence remains valid only for its tested
configuration; do not transfer its performance numbers to scene refresh.

## Outcomes and evidence

1. Sustained single-session operation: measure and remove demonstrated backend
   stalls reaching odometry. Repeat the existing real-bag and artificial repeated
   route runs with identical input/configuration/hardware. Report admission loss,
   odometry gaps, queue latency, CPU, RSS, finish, and independent reopen. Do not
   replace loss with unlimited queue growth. Artificial route seams are not
   natural motion, truth, or proof of physical loop correctness.
2. Controlled historical state: distinguish bounded resident payloads from
   persistent redundancy. Establish ownership and byte accounting for retained
   graph/index/map data. Repeated observations should update a useful existing
   representation when association is verified, rather than blindly adding full
   copies. Retain uncertain/new observations without corrupting known history.
3. Limited automatic cross-session association: use existing reconstruction,
   attachment, geometric verification, continuation and commit owners. Propose
   historical candidates, reject ambiguity, and demonstrate independent-process
   reopen → associate → continue → reopen on retained data. No claim of arbitrary
   cold-start localization or universal dynamic-scene correctness.

Pandora is a reference for producer isolation, working-memory eviction/reactivation
and scene update. Its visual/depth representations and
performance claims are not interchangeable with Sapphire's LiDAR/Gaussian input.
Environment or submap "stability" is not a prerequisite for retaining updates.

### User scope correction: local online navigation only

The user explicitly excluded archived navigation maps, global raster output and
3D occupancy products. Keep LiDAR/Gaussian submaps and search pyramids for loop
retrieval/verification, and keep original/optimized poses and observation identity.
Existing local point-cloud output remains online in the odometry frame. It is not
to be represented as a global navigation map. A local navigation product must
remain transient and spatially bounded; the user subsequently explicitly requested
the rolling window described below.

Remove global occupancy processing/persistence/publication from the canonical
disabled-navigation configuration. Introduce a deliberately distinct geometry-only
map format (schema 2, c3 identity) with no FlatGrid or NaviTrajectory table; schema 1/c2
remains explicitly supported for legacy callers with navigation enabled. Do not
rewrite old databases. Per the user's clarification, omit navigation splines too;
retain the Node odometry observations needed to interpret the scene/pose graph.
The online SubmapFrameBuffer no longer accumulates duplicate navigation samples
or fits a spline. Raw ordered odometry stays available for future on-demand use;
legacy path structures/decoding and legacy test fixtures are retained.
Retain all graph/geometry validity checks.
Navigation rendering parameters no longer affect c3 map identity. Historical
materialization and B1/B2 commits skip occupancy work for c3, while correction
readiness and checked shutdown remain mandatory. ROS canonical configurations
advertise correction only and create no global navigation publishers when disabled.

Ownership: the DB determines whether archived grid evidence exists from the
explicit configuration identity and matching schema; backend owners construct no
occupancy accumulator when disabled. No new map owner or coordinate conversion.
Tests must cover geometry-only write, reopen, attachment/continuation and reopen,
absence of FlatGrid, unchanged loop geometry, and legacy-schema regression. This
format change follows the new product requirement and does not reopen A2/F1.

## First implementation: performance attribution

Inspection: PoseGraphBackend::processPending holds lifecycle_mutex_ throughout
new-map graph, retrieval/registration, storage and occupancy processing; addFrame
also takes that lock. Pipeline's two-entry marginal queue can propagate a blocked
submission to odometry, whose synchronizer also admits only two LiDAR frames.
Existing logs bracket multiple operations, so they cannot isolate the observed
6.9-second gap or assign it to a particular algorithm.

Reuse existing PoseGraphBackend, VisualSubmapIndex and Pipeline owners. Add opt-in
`SAPPHIRE_PROFILE_BACKEND` stage and wait timing through the existing logger; no
new runtime manager or persistent fields. Clocks are monotonic wall time, durations
are milliseconds. Worker-local timers are consumed on their owning thread; no
cross-thread mutable profiling state. No coordinate/model changes. No changed
factor schedule, candidate policy, commit, identity, readiness or shutdown order.
Profile output records elapsed intervals, not CPU time or exclusive nested time.

Validation: build both existing Release trees, run the relevant regression suite,
then repeat the clean four-cycle 2× stress configuration with the profile enabled.
Preserve pre-change sources and executable under
`audit/2026-09-29-continuous-mapping/before`. Use measured stages and waiting to
choose the smallest effective execution change; document ownership, bounded
admission, failure/wakeup/drain and regression criteria before changing it.

## Status

The broader product goal remains in progress. Geometry-only persistence, online
spline removal, bounded local occupancy and new-map admission isolation are
implemented and verified in the scope below. The same-input stress comparison
demonstrates removal of the previous long submission-lock stalls and no Reset in
that run. Map-size convergence and whole-process bounded memory remain unproved.
Restricted automatic association now has the controlled evidence above; unrestricted
cold starts and arbitrary prefix merging remain open. Prior acceptance packets stay intact.

## Local rolling occupancy (explicit user request)

Use a transient, fixed-size, gravity-aligned 2D window in the current odometry
domain. The window follows the live sensor origin, discards cells outside it, and
expires unobserved evidence to unknown. Free rays clear old obstacle evidence;
same-scan obstacle endpoints override free rays. It is not an archived map,
descriptor, loop input, global product or revisioned navigation frontier.

Reuse the existing bounded/coalescing scan-output slot and output worker. Carry
the scan's own origin, timestamp and session generation with its immutable cloud;
never pair it with a separately published "latest pose". Grid update and ROS
serialization execute outside the frontend and outside output-owner locks. Reset
the grid with the producer domain. No additional work queue or global map worker.

The new RollingGrid owns two fixed cell buffers and per-scan marks. It has a
distinct transient spatial lifetime, unlike the archived OccupancyGrid owner.
The default window is 20 m × 20 m at 0.1 m; expose local-only ROS parameters
for extent, resolution, observation lifetime and obstacle height band. Flat-ground
height classification initially uses an explicit sensor-height parameter (0.8 m
for the retained MID360 data), not a claim of general terrain estimation. Single
sensor ray origins are supported; merged multiple-origin scans require provenance
before free-space clearing can be claimed equivalent.

## Measured producer isolation change

The geometry-only four-cycle 2× run completed but had 700+ LiDAR refusals and one
Reset. New-map batch wall time totaled 133.47 s: BBS 86.97 s, visual retrieval
36.07 s, graph odometry update 0.253 s, loop optimization 0.047 s. Submission lock
wait totaled 42.17 s; logged frontend marginal-queue waits totaled 39.72 s, max
4.661 s. The largest odometry output gap was 4.695 s. These nested timers are not
additive CPU samples, and artificial route seams still limit tracking claims.

Change new-map admission to use existing input_mutex_/capacity_cv_ instead of
acquiring lifecycle_mutex_ held by the consumer. Keep exactly one waiting submap
plus the consumer's active submap, with the existing 64 MiB evidence envelope.
Configuration and historical mode are immutable; failure/fence flags are atomic;
submap_count_, admission stop, queue, bytes and progress remain input-mutex owned
on this path. The worker owns graph/index/storage state under lifecycle_mutex_.
Taking queued input wakes capacity waiters; stop/failure already wake that same
condition. No unbounded buffering, candidate/order/optimizer/commit changes or
new thread. Check queue identity again after a capacity wait. Test admission while
the worker is deliberately stalled under its lifecycle lock, capacity blocking,
stop wakeup and drain of every accepted frame. Re-run the same input after change.

Validation: negative-coordinate and large-shift window movement, origin-aligned
rays, hit/free conflicts, obstacle clearing, unknown expiry, producer reset,
timestamp ordering and constant allocations over prolonged movement. ROS wiring
must publish the local grid in odom with the source timestamp; the DB must still
contain no navigation tables. Measure the enabled local-grid run separately from
the geometry-only profiling baseline.

### Implementation verification — rolling window and producer isolation

Release core regression: 20/20 passed (52.87 s); ROS lifecycle/publication/input
regression: 3/3 passed (1.62 s). The new admission barrier case verifies producer
independence from the consumer lifecycle lock, a single waiting submap, stop
wakeup and checked drain. Rolling-grid cases cover clearing/expiry/movement and
constant resident buffers over 20,000 updates. These are regression/unit results;
real-bag local-grid publication and same-input admission performance were measured
separately. Evidence: `core-final-ctest.log`, `ros-final-tests.log`,
`isolated-admission-source.json` in the continuous-mapping audit directory.

Completed four-cycle 2× comparison: 8,036 odometry, zero Reset, maximum odometry
gap 0.201 s; admission wait sum 0.487 ms versus baseline 42.17 s. Peak RSS rose
from 3,088.64 to 3,216.04 MiB; no memory improvement claim. All 94 accepted submaps
drained, and the geometry DB reopened unchanged. This is one artificial stress
run, not a universal realtime or physical loop-accuracy result.

Final grid-only immediate-hit regression passed. Enabled-grid real-bag 1× run:
2,008 odometry and 2,008 valid 200×200 local grids, no refusal/Reset logs, all
timestamps and snapped window origins matching source odometry; peak RSS
1,332.30 MiB and mean CPU 80.72%. All 21 submaps completed and independent reopen
passed without navigation tables. Full measurements and limitations are in the
[audit packet](../../audit/2026-09-29-continuous-mapping/README.md).

## Visual metric initialization — integration in progress

Update2026-09-30: versioned visual observation persistence and deterministic metric
scene construction now pass codec and independent-process continuation/reopen tests.
Online PnP routing is now implemented with frozen query projection, one-pose LiDAR
verification and GICP; no BBS search follows a usable seed or geometric rejection.
Controlled verification is recorded in the live-PnP packet; real-data acceptance
and automatic fresh-session attachment remain open. See [the current visual design](VISUAL_METRIC_INITIALIZATION.md)
and [archive evidence](../../audit/2026-09-30-metric-archive/README.md).
The following paragraphs record the earlier appearance-only baseline.

The user identified costly BBS and asked whether existing 2D image points can
estimate pose. Current online `buildVisualScene` explicitly sets
`Landmark::has_position=false`; FeatureBlock construction supplies no depth, and
VisualSubmapMatch carries a candidate ID/score/match count, no metric transform.
The production path remains appearance retrieval → BBS → GICP. Existing PnP
comments and optional 3D fields are not an implementation of metric localization.

Calibrated monocular 2D–2D correspondences can constrain relative rotation and
translation direction with sufficient parallax, not translation scale. They can
reject some incorrect candidates or constrain a search, but are not a metric
replacement for BBS. Pure rotation, low parallax, planar/ambiguous geometry and
dynamic features need explicit rejection/alternative handling.

The user subsequently rejected LiDAR-feature association and specified lightweight
pyramidal optical flow with visual triangulation. Vision drives submap opening
and coarse loop initialization only, never odometry. The superseding design and
reference inspection are in [VISUAL_METRIC_INITIALIZATION](VISUAL_METRIC_INITIALIZATION.md).
No LiDAR-feature association code was implemented. Existing LIO motion may provide
metric camera baselines as a read-only input; camera timing/calibration and
triangulation quality still require explicit verification.

## Independent visual/spatial loop hypotheses — 2026-09-30

The user requires visual failure under narrow FOV, changed foreground or occlusion
not to veto a valid LiDAR loop. Inspection confirms that an empty visual shortlist
already preserves spatial candidates. However a target found by both paths loses
its spatial initial transform: prepareCandidates replaces its translation with
zero and switches it to the appearance-centered search. This can exclude a valid
local hypothesis when the visual search window does not cover that transform.

The user clarified that visual metric initialization and BBS are alternative
initialization paths, with vision the primary future path and BBS for unavailable
visual initial poses. Do not redundantly run BBS after a usable visual PnP seed.
Insufficient visual evidence is distinct from an independently geometry-rejected
candidate; do not bypass the latter rejection by changing solvers.

Today the live visual path supplies appearance only. When a target is already in
the spatial batch, retain that existing spatial initial transform and its local
BBS window. Only appearance-only targets use the existing larger origin-centered
BBS search. There is exactly one BBS attempt per target and no serial second
search after BBS/GICP rejection. The initially proposed dual-search fallback was
withdrawn following the user's clarification. No optional seed field remains.
Geometry/pyramid ownership stays in the existing batch; do not duplicate clouds,
expand the candidate list or create a graph/manager. Existing BBS/GICP acceptance
is unchanged. Transforms map query anchor into the historical target anchor, metres.
No new factor semantics, temporal/cardinality policy, persistence fields,
optimizer schedule, writer behavior or lifecycle contract is introduced.

Verification must witness a real geometric loop with no visual matches, a fused
candidate whose valid spatial seed is outside the visual window, and retained
visual-only discovery under large odometry drift. Check the serialized directed
factor, exactly one attempt after injected BBS/GICP rejection, and ordinary
lifecycle regressions. This does not provide pose-independent
LiDAR place recognition: when drift defeats spatial overlap and vision has no
candidate, a separate bounded global geometric retrieval capability is still
needed. Missing visual support is unavailable evidence, not proof of no revisit.
