# A2 historical runtime reconstruction — approved contract

Status: implementation authorized by the consolidated user instruction on 2026-09-27,
after numerical characterization and GTSAM persistence investigation. Current
implementation/verification evidence is recorded in CURRENT_STATE.md.
The user has accepted A1, including its corrective passes. Existing documents'
older statements that A1 still awaits review are historical status, not a reason
to reopen A1. This document does not change the accepted A1 implementation.

A2 opens an A1-eligible committed map, validates its historical contents, and
reconstructs existing backend owners in memory. It accepts no new submaps,
frontend session, attachment, localization input, or writable continuation.

Three distinctions are essential to reviewing this design:

* A1 eligibility is necessary, not sufficient: it intentionally does not certify
  graph or payload completeness.
* Schema 1 stores graph Link measurements and committed poses as float matrices.
  Exact recovery of the former process's double measurements is impossible.
  This design restores the persisted mathematical model, with the same factors,
  directions and noise semantics, and explicitly measures numerical discrepancy.
* The current format has no loop-decision marker. Its completion state is always
  **Unavailable**; this does not disqualify an otherwise valid historical map.
  There are no optional-marker variants.

Sapphire supports exactly one database format. SQLite `user_version == 1` is
only its cheap native format-identity guard; every other value fails closed.
This is independent of the software/product version, which may remain `0.0.0`.
This design includes no persistence evolution, compatibility negotiation,
legacy-format support, migration, or speculative future format contract.

## Numerical-support amendment — accepted with documented limitations; closed

The independently accepted design [2026-09-29 amendment](../../audit/2026-09-29-a2-contract-amendment-design/README.md)
changes fresh-versus-committed similarity from historical-open rejection to a
reproduction diagnostic. The [bounded implementation](../../audit/2026-09-29-a2-contract-amendment-implementation/README.md)
has passed [fresh independent implementation review](../../audit/2026-09-29-a2-contract-amendment-independent-implementation-review/README.md):
**A2 implementation accepted with documented limitations**. The amendment is
**CLOSED**; [final closure](../../audit/2026-09-29-a2-contract-amendment-closure/README.md)
records status, stale-heading and warning-unit corrections only. It splits
numerical validity from finite similarity,
retains the normal solver schedule and records attributed discrepancy warnings.
The older empirical reports remain period-correct.

This amendment supersedes numerical rejection wording only. Accepted B1/B2
extensions govern current multi-chain topology, root attachments and explicit
continuation; earlier single-chain/no-attachment phase wording below does not
undo them. A2 opening itself is still read-only and creates no active session.
No A1→B3 architecture, reconstruction schedule, threshold value, model, storage
format or failure-lifecycle change is authorized here.

## 1. Persistent authoritative state

| Fact | Current persistent representation and interpretation |
|---|---|
| Map identity and compatibility | `MapState.map_uuid`, `config_identity`, SQLite `user_version=1`; retain the accepted six-field `c2:` identity exactly. |
| Current stored revision | `MapState.graph_revision`; read its existing meaning, without promoting it to the later T1/T2 or output-completion contract. |
| Node identity | `Node.id=d`, with backend/submap/GTSAM key `k=d-1`. No renumbering. |
| Historical committed map pose | `Node.submap_pose`, a column-major 4x4 float matrix, is `T_map_submap[k]`. |
| Original odometry | `Node.odom_pose_count`, double timestamps, double `(tx,ty,tz,qx,qy,qz,qw)` tuples, and `Node.stamp`. |
| Pose tag | Pose `MetaTag` (`tag_type=2`) initially stores `lio.T_odom_base` but **is updated on every optimized-pose commit**. It witnesses the committed pose, not an immutable original anchor. Decode its row-major matrix separately from the Node blob layout. This corrects the initial design's source-reading error. |
| Graph edges | `Link(from_id,to_id,type,transform)`, including its stored float measurement. No factor is reconstructed from optimized pose differences. |
| Original prior | For the supported production producer, the first odometry sample of SQL Node 1 defines the original anchor. The timestamp and Node 1 committed-pose/tag consistency support validation; the tag is not independent proof of the original measurement. Section 4 defines the restrictions. There is no prior row. |
| Local geometry and bounds | `LaserRecord`, `PyramidVoxel`, `SpatialRecord`; preserve their metric data and payload-format identities. |
| Navigation records | `NaviTrajectory` samples/spline remain persisted historical data, even though not factors. |
| Appearance | `VisualScene` bytes, scene vocabulary/landmark/descriptor identities, and `ImageRecord` camera IDs, frame order, timestamps, points and descriptors. Other MetaTags remain records, not the production Faiss archive. |
| Occupancy evidence | `FlatGrid` ground/obstacle/empty coordinates, cell size and viewpoint, decoded into existing `LocalGrid`. These labels are authoritative even with F3/F4 limitations. |
| Loop-decision marker | Does not exist in the current format. Runtime status is Unavailable; no boolean value is inferred or persisted. |

The prior/noise interpretation is a fixed schema-1 compatibility rule backed by
the current producer, not an additional fact secretly stored in `c2:`. Do not
write a new model, pose, index, or occupancy copy to simplify reopening.

## 2. Derived runtime state, ownership and lifetime

Reuse `PoseGraphBackend::Impl` for `frames_`, GTSAM factors/ISAM2, committed pose
lookups, counts, historical-status fields, visual index, occupancy and their
existing mutexes. Reuse `Memory` for its single `MapDatabase`, spatial/pose
indexes, record readers and payload caches. Reuse `VisualSubmapIndex`,
`DescriptorArchive`, `OccupancyGrid` and `NavigationGrid`.

Derived state comprises:

* Decoded lookups and ID mappings; original anchor entries in `GraphFrame`.
* GTSAM factors created from records, Bayes tree, linearizations and estimates.
* Pose KD-tree, transformed AABB R-tree and their ID membership.
* Faiss slots, training state, index and calculated descriptor fingerprints.
* Cloud/grid LRU contents, decoded-scene cache and prefetch state.
* Occupancy cell evidence, active-frame/tile indexes and exported grid.
* Runtime capability/status flags and numerical reconstruction diagnostics.

Construction is synchronous. The object is not usable until validation and all
required reconstruction succeed. Failure destroys the incomplete owners through
RAII; no partial map is returned. There is no backend processing worker in
historical mode, no GPU initialization, and no `processPending()` call.
Memory's existing prefetch worker may start after validation; Faiss's existing
training worker is allowed, with training settled before declaring the relevant
index ready. Neither performs map writes. Destruction joins these workers before
their Memory/database dependencies disappear.

Use small flags/enums and ordinary accessors on existing owners. No recovery
manager, context, session manager, graph snapshot, or aggregate restored-world
object. Historical pose/revision/status/retrieval accessors return values or
existing immutable result types, not a mutable Memory handle.

The direct `PoseGraphBackend` resume entry becomes historical read-only opening.
The input-oriented `SlamPipeline` resume gate remains closed in A2: opening a
historical backend is not authorization to start its frontend threads. Its error
text should accurately say that session attachment/continuation is unavailable.

## 3. Historical graph reconstruction algorithm

1. Open exactly one read-only `MapDatabase` through the accepted A1 path. Memory
   must keep that same handle for validation, reconstruction and later reads;
   do not validate one file and reopen another pathname.
2. Read UUID, revision and compatibility. Run read-only SQLite integrity and
   foreign-key checks, then explicit checks for Link endpoints, payload ownership
   and required rows; Link currently declares no foreign keys.
3. Enumerate **Node first**, `ORDER BY id`. Validate every ID, blob and required
   payload before building indexes. Inner joins must not conceal missing rows.
4. Require SQL IDs `1..N`, within the existing signed-int mapping range. Empty
   initialized map is valid only with no dependent rows/links and revision 0:
   empty ISAM2, no prior, empty indexes/grid. For a nonempty map require revision
   at least 1 and the complete supported graph structure. This detects a saved
   base node that never reached its initial graph commit where observable; it
   does not prove loop-decision completion.
5. Decode original anchors and committed poses separately. Populate `frames_`
   from validated original anchors and `optimized_` from committed map poses.
6. Enumerate all Link rows explicitly and validate the table below. Build factors
   in the specified order and validate the resulting component/count invariants.
7. Build ISAM2, validate its semantics/numerics, then rebuild spatial, visual and
   occupancy owners. Expose the backend only after every required step succeeds.

Throughout, `T_A_B` transforms B into A; `X_k=T_map_submap[k]`.

| Factor | Persistent source and endpoints | Measurement and model | Deterministic order and validation |
|---|---|---|---|
| Initial prior | Original anchor from Node 1; GTSAM key 0 | `PriorFactor<Pose3>(0,A_0)`, six variances `1e-12`, no robust wrapper | First and exactly once for nonempty maps. Section 4 validates its provenance. No fabricated Link prior. |
| Odometry | Each `Link.type=0`, SQL `(d,d+1)` to keys `(d-1,d)` | Stored `Z=T_submap[d-1]_submap[d]`; `BetweenFactor` predicts `X[d-1]^-1 X[d]`. Variances `(1e-6,1e-6,1e-6,1e-4,1e-4,1e-4)` in GTSAM rotation/translation order; no robust wrapper | After prior, increasing `from_id`; exactly `N-1`, one for every adjacent pair. Cross-check against original anchor differences at float-serialization tolerance. Never substitute that cross-check for the stored Link. |
| Loop | Each `Link.type=1`; SQL `(query+1,target+1)` to `(query,target)` | Use stored `T_query_target` directly. It is already the inverse of registration's `T_target_query`. Same diagonal variances as odometry, Huber `k=10`, **Block** reweighting | After odometry, increasing `(from_id,to_id)`. Current producer requires `query>target`, `query-target>3`, at most one loop per query. Validate endpoints, transform and uniqueness. Do not invert again or rerun registration. |
| Attachment | No reserved attachment enum/type or measurement/noise contract exists in current source; an arbitrary integer column is not a reservation | Unsupported | Reject all types other than 0 and 1. No speculative interpretation or future format design. |

For `N>0`, factor count is `N + L`, one connected component containing all N
keys, and exactly one anchored component. Duplicate SQL identities, reversed
duplicates, duplicate loop queries and conflicting factors fail; do not coalesce
or pick a winner. A prior is implicit in this format and is not counted as a Link.
Historical insertion batching/order is not stored; the order above is the A2
canonical rebuild order, not a claim to replay the old optimizer's event history.

## 4. Prior and gauge

The production `SubmapFrameBuffer` chooses its first marginal frame as reference,
appends that frame as the first odometry sample, and uses the same reference for
the frozen `LioFrame`. `buildOdometryGraph()` anchors key 0 to that original
`LioFrame.T_odom_base`. Thus Node 1's first original odometry tuple is the
recoverable original prior measurement; the latest `Node.submap_pose` is not.

For every supported historical node, require a nonempty, correctly sized odometry
array, finite nondecreasing timestamps, `Node.stamp` matching its first timestamp,
and valid unit quaternions. Require the pose tag's existing matrix type/shape and
equality to its committed Node pose after decoding their different layouts.
Node 1's first original odometry tuple supplies the prior; first tuples of later
nodes provide original anchors for odometry cross-checks.

The characterization exposed a concrete mistake in the original design:
`saveSubmapPoses()` calls `saveMetaTag(entry.first, MetaTag(entry.second))`.
Recorded pose tags match committed poses exactly and differ from original
anchors by up to 0.549488 in a matrix coefficient. Do not validate every tag
against original odometry or treat it as an immutable original-anchor witness.
This is a correction to the design, not a regression in accepted A1.

The general `SubmapFrame` constructor does **not** enforce equality between its
LioFrame anchor and its first odometry sample. Consequently A1-eligible arbitrary
low-level writer fixtures are not automatically supported production graphs.
Missing required records or inconsistent original odometry/Link data fail A2;
there is no fallback to identity,
the optimized pose, the last odometry sample, or fitting an anchor from links.

Pin the current-format graph model to the existing six prior variances, odometry
variances, Huber block loss and Pose3 convention. Do not extend `c2:` with
unrelated search or frontend settings. Incompatible format/model fails closed;
no evolution or compatibility strategy is designed here. The format guard cannot
identify an external producer that silently changed constants while retaining
its identity; it is not authenticated provenance. Original quaternion encoding
and float Links prevent bit-for-bit recovery of every pre-serialization double.
Such recovery is not promised. The supported production-buffer convention,
rather than a mutable tag, establishes the prior's original-anchor provenance;
arbitrary low-level producers with ambiguous provenance are not certified by A2.

## 5. ISAM2 reconstruction procedure

Use the repository's supported GTSAM build/model. Pin the settings used by the
current backend: Gauss-Newton (wildfire threshold `0.001`), relinearization
threshold `0.01`, skip `1`, relinearization enabled, Cholesky, linear-factor
caching enabled, partial relinearization check disabled, unused-factor-slot
reuse disabled and adaptive reorder disabled. Retain the current Pose3/Rot3
exponential-map build settings. Diagnostics flags do not define historical data.

Insert initial Values in ascending key order, using **stored committed poses**,
not unoptimized odometry. Insert the full canonical factor vector in one update.
Use the existing update sequence: `update(graph, initial)`, `calculateEstimate()`,
one empty `update()`, then `calculateEstimateWithAffectedKeys()`.

This performs numerical optimization; it is not a passive factor deserialization
or a convergence loop. It reconstructs a usable solver, not its private old caches.
Compare its result to committed poses under section 6. Finite similarity-only
exceedance records a reconstruction reproduction discrepancy and does not reject
historical open. Every hard validity and materialization check still applies.
It never triggers extra optimization or a change to the map.

Keep `optimized_` as the decoded **committed** pose view for A2's historical
readers; ISAM2's own estimate remains a distinct derived solver estimate. Use a
temporary Values for comparisons, rather than a second permanent graph owner.
The reconstructed solver X* may differ materially from committed X^R. Report
differences through existing diagnostics; keep X* internal to the solver.
Revision R's pose/spatial lookup, occupancy and global presentation use X^R
exactly; visual history uses persisted evidence. X* cannot be published as R,
change UUID/revision attribution or create active correction before fresh
attachment. Never call
`saveSubmapPoses`, `saveLink`, `updateCorrection`, or publication-commit code from
this path. No new revision, no overwrite, no silently adopted solver estimate.

Set historical node/odometry/grid counts from validated contents. Input queues,
pending links and pending changed-key sets are empty. Do not use
`searched_loop_id_=N` to assert completion when schema 1 cannot establish it.
It is inactive in historical mode and must not cause processing on destruction.

## 6. Persistent equality, semantic equivalence and numerical reproduction

The design-time probe and complete results are recorded in
[the characterization report](/home/user/code/sapphire_git/audit/2026-09-27-a2-characterization/README.md).
It uses the current GTSAM build, the fixed proposed reconstruction sequence,
93 controlled synthetic graphs and three existing audited graph databases,
repeated in five independent processes per input. There are no extra solver
iterations, tuning, production changes or database writes. The audited databases
have `user_version=0`: they provide numerical graph facts only, and remain
rejected by the current format guard. They are not A2 acceptance fixtures.

### A. Persistent-fact equality

UUID, stored revision, Node IDs, Link identities/types/endpoints, persisted
transform/committed-pose/payload bytes, database and sidecar bytes, user_version
and config identity remain exact, with zero tolerance. No derived numerical
result may overwrite or revise them. An absent sidecar remains absent.

### B. Derived semantic equivalence

Require the same graph topology, connected components, factor endpoints,
measurement decoding, prior provenance and fixed noise/robust semantics. Require
the committed Node float pose to be used unchanged by spatial/pose indexes and
occupancy. Compare spatial candidate sets under the same policy, visual descriptor
membership/fingerprints/eligible target IDs, and occupancy evidence meaning and
output from the same persisted inputs. Discrete identities and preserved numeric
representations compare exactly. Internal tree, container, cache, Faiss slot or
ISAM layout does not enter acceptance.

### C. Derived numerical validity and reproduction diagnostics

The rebuilt ISAM estimate, residual arithmetic, nonlinear objective and any
floating-point occupancy evidence are numerical runtime results. Former solver
batching/linearization and asynchronous Faiss history were not persisted. Exact
same-build repeatability is useful evidence, not a new persistent-fact contract.

Measured maxima over the characterized inputs:

* Rebuilt versus committed: translation `0.0113141644 m`, rotation
  `0.000861710519 rad`. At ordinary origins the translation maximum was
  `0.00747276573 m`; the larger bound includes 100 km coordinate offsets.
* Factor residual changes between rebuilt and committed estimates: translation
  norm `0.00853721819 m`, rotation norm `0.000913888370 rad`.
* Isolated float-Link effect versus the synthetic double-Link rebuild with the
  **same committed initial Values**: `2.64729186e-7 m`, `2.97560784e-8 rad`.
* Repeated-process pose differences and reported scalar differences: zero in
  all five runs. This removes no need for semantic comparison and establishes
  no universal `1e-9 m/rad` requirement.
* Objective decreased in every characterized case. The largest decrease was
  about `6.408704e6`, from quantization of the strongly anchored prior at a large
  absolute origin. A tight absolute or symmetric relative objective-equality
  requirement would reject legitimate reconstruction.

Amended numerical contract:

1. Every committed/reconstructed pose, factor residual, objective and objective
   delta must be finite and valid. Require exact complete typed Pose3 key coverage,
   expected factor cardinality/endpoints/type and original-prior/model semantics.
   The same persisted graph/model interpretation is required; the same numerical
   stopping point is not. Retain inserted-factor endpoint equality, the current
   noise equality comparator (`1e-15`) and exact component equality of unwhitened
   residuals evaluated at identical committed Values. That semantic comparison
   is independent of the movement comparison between X* and X^R.
2. Keep all four unchanged reference limits: maximum all-key translation
   discrepancy **0.035 m**, relative rotation **0.003 rad**, and maximum all-factor
   unwhitened residual-change norms **0.026 m** translation / **0.003 rad** rotation.
   Residual differences include the original prior, using tail/head three
   coordinates respectively. Preserve the existing Euclidean translation and
   normalized diagnostic quaternion angle calculation; do not repair stored
   transforms. Strict `>` denotes exceedance; equality is within reference.
3. These values characterize reproduction of the current rebuild procedure,
   not a universal semantic validity boundary for revision R. Their original
   margins (3.09x/3.48x pose, 3.05x/3.28x residual maxima) and the replacement of
   the unmeasured 1 mm / 1e-4 rad proposal remain historical characterization.
   They remain regression/reference and diagnostic thresholds. They are not
   navigation tolerances, a guaranteed continuation basin, safety limits or
   mathematical convergence thresholds. Future retuning needs separate evidence.
4. Retain `ReconstructionDiagnostics` fields and `reconstructionDiagnostics()`
   with existing availability guards. Derive `within_reference` or
   `exceeds_reference` from those fields; add no persisted bit, public enum or
   status subsystem. They remain initial-opening evidence, not a current-revision
   snapshot or later-operation certification. For empty valid history retain
   zero diagnostics/no prior/no solve. Nonfinite/invalid results fail before
   classification; reductions must not hide NaN or overflow.
5. Keep the existing metric log. On exceedance, emit one warning/detail through
   existing spdlog: “reconstruction reproduction discrepancy”, opening UUID/R,
   all four values/units/references, exceeded components, node/factor counts and
   objective delta. State that similarity alone does not reject opening; do not
   imply corruption, invalid persisted map, required repair or completion of the
   remaining product/identity checks. No schema field, warning service, manager,
   new revision, transport message or extra optimization is introduced.
6. Always record both objectives and `E_rebuilt - E_committed`. Finite objective
   change, of either sign or magnitude, is diagnostic; nonfinite objectives/delta
   remain a hard failure. No monotonicity or whitened-residual-change gate is
   added. The strong original prior can amplify small legitimate pose movement.
7. Independent reconstructions each satisfy the hard semantic, representation,
   numerical-validity and product contracts. Record reproduction differences;
   neither bit equality nor a separate 1e-9 m/rad gate is required. The former
   0.070 m / 0.006 rad pairwise consequence holds only when both reconstructions
   are inside reference, not for all successful opens. Preserve reference-bound
   assertions on characterized controls and investigate unexpected changes;
   expected beyond-envelope fixtures assert successful open plus diagnostics.

The probe spans 5–512 nodes including recorded inputs, noncommuting rotations,
robust-tail loops, two live insertion schedules and positions up to 118327 m in
norm. Those earlier results remain valid characterization, but reproduction
similarity is no longer the sole criterion for semantic historical-open support.
The [retained beyond-envelope experiment](../../audit/2026-09-29-a2-beyond-envelope-continuation-experiment/README.md)
opened F1 at approximately 0.073646 m pose / 0.094173 m residual translation
discrepancy and completed R5→B1/R6→B2/R7→checked close→independent reopen using
an audit-local diagnostic rule, without settlement or extra optimization. It
supports this semantic change for that history and one control; it is not an
already-amended production reader or proof about arbitrary divergent graphs.
No node-count, coordinate cutoff or revision-jump limit is added.

Keep representation validation separate: 64-byte finite float transforms,
homogeneous row `(0,0,0,1)`, positive determinant and existing proposed float
rigidity checks (`||R^T R-I||_F <= 1e-6`, `|det R-1| <= 1e-6`); finite unit double
quaternions (`|norm(q)-1| <= 1e-12`); timestamp agreement within `1e-9 s`.
Original-odometry-to-Link cross-check remains a float-representation comparison:
per coefficient `4*eps32*max(1,abs(a),abs(b))+1e-12`, `eps32=2^-23`.
Pose-tag versus committed Node pose comparison is exact after decoding their
row-major/column-major layouts. Do not compare later optimized tags to original
odometry. Matrix conversion and rotation-distance diagnostics remain as in the
initial design; no transform repair is introduced.

### D. Later revisions and operation-specific support

A2 reconstructs a valid runtime representation of the accepted persisted graph
while preserving revision-R committed presentation. X^R is authoritative for R;
X* is disposable optimization state. Divergence alone rejects neither A2 open
nor later B1/B2 continuation. It does not guarantee every finite graph can
continue indefinitely, every attachment/optimization succeeds, arbitrary
divergence is safe, or a map/loop is physically correct.

B1 preserves historical target selection, local BBS/GICP, explicit T_H_Q seed,
Q→H attachment factor, current two-update tentative solve, W promotion and
correction equation. Historical runtime X_H* may differ from committed X_H^R
before B1. A valid solve may reconcile that movement into R+1; failed solver
mutation remains sticky under [failure semantics](BACKEND_FAILURE_SEMANTICS.md).

B2 preserves original-anchor odometry measurements, committed-geometry seeding
and retrieval, loop verification, Policy C/K=3, optimizer schedule and W. Runtime
movement remains speculative until the next committed all-key delta. Follow
[B2](B2_DESIGN.md)'s committed-owner and readiness rules; do not expose X* under
the previous committed revision. No R→R+1 pose-jump limit is added.

Every serialized historical pose change in the proposed revision must be
compared and included in the committed delta, independent of affected-key hints.
Retain the existing exact all-key comparisons, including keys outside hints.
B1 uses serialized float coefficient equality (a signed-zero-only `+0.0f/-0.0f`
difference need not be written); B2 compares all 64 serialized bytes. Preserve
these distinct comparators and include the new node. The F1 experiment's nine changed historical keys happened to
appear in hints; that does not replace the outside-hint regression or all-key
scan. Successful W commits and required owner materialization may make the
tentative result authoritative for R+1/R+2, never retrospectively for R.

W outcomes, DB authority, postcommit materialization, C/Y, correction, B3
revision/source attribution, Pc/Pg and checked drain/shutdown are unchanged.
A large coherent transition is permitted; a partial/misattributed transition
is not. Old immutable products retain their own revision attribution.

The previously characterized full-graph F1 writer settlement probe remains
useful diagnostic/research evidence, but is not part of the mandatory managed
B1/B2 production contract under these amended semantics. It correctly predicted
the previous reader rule; its product necessity is superseded, not its technical
evidence refuted. Preserve its audits; implement no settlement or extra updates.
No solver-free historical mode, new runtime owner or new map mode is introduced.

## 7. Spatial and pose index reconstruction

Use `SpatialRecord` local bounds and `Node.submap_pose`, in ascending Node ID,
to populate the existing `SubmapSpatialIndex` and `PoseRecallIndex`. Transform
all local bounds using the committed float pose and the current AABB algorithm.
Require exactly one spatial row per Node, finite ordered bounds, valid transformed
bounds and exact index membership/counts. Validate bounds against the persisted
cloud's local bounds using the float coefficient tolerance above; never rebuild
missing bounds from raw sensor input or silently discard an orphan row.

The current Memory constructor first inserts committed poses and then loads all
MetaTags. `indexFor(kPose)` returns the same pose index. Current optimized-pose
transactions also update the tag, so the original design's claim of an existing
original-odometry overwrite was incorrect. In historical reconstruction, validate
the tag against its committed Node pose and **do not insert it into the pose index**:
Node remains the single authoritative source for pose-index population.
Other supported descriptor tags retain their identity; production appearance
retrieval still belongs to VisualSubmapIndex, not these default-empty adapters.

Keep overlap threshold `0.5`, pose distance/tie behavior and existing candidate
temporal filtering unchanged. Do not run BBS/GICP during opening. Test historical
spatial queries with asymmetric, optimized poses and known overlapping bounds;
compare candidate sets and committed candidate poses, not R-tree node layout or
an undocumented traversal order.

## 8. Visual historical-state reconstruction

Read and validate all historical `VisualScene` and `ImageRecord` rows, including
rows not currently eligible as targets. Rebuild only appearance retrieval.
Never recompute ORB, undistortion, depth, rays, 3D geometry or scene fusion from
raw inputs or from the current camera calibration.

The scene is the target appearance source. Decode the existing scene format and
CRC, validate finite fields, unique landmark identities, capacities, vocabulary
and descriptor identities. For current appearance-only production scenes,
`has_position=false`, zero vocabulary and the single-descriptor representation
are the supported semantics. Valid old scene encodings are accepted only when
the existing decoder preserves those supported semantics; metric scenes are not
silently reinterpreted as this producer's appearance history.

Recalculate `DescriptorArchive::fingerprint(scene)` using its existing algorithm;
restore archive metadata `SQL node ID -> (fingerprint, descriptor count)` exactly.
Fingerprint is derived from the persistent scene, not an independently persisted
authentication checksum. CRC detects ordinary payload damage; no mechanism
detects all coordinated modifications that recompute a valid checksum. A scene
identity mismatch is an error, not a cache miss that discards historical data.

ImageRecord supplies query feature blocks for **existing historical IDs**. Reuse
the same VisualFrame-to-FeatureBlock conversion used by `addSubmap`, without
manufacturing a SubmapFrame or accepting new input. Preserve frame order,
descriptors, camera IDs and feature identities. Validate row counts, descriptor
shape/type, finite points/timestamps and frame-index sequence. Consistency-check
present production scenes against the existing stable descriptor deduplication
and capacity rule applied to persisted ImageRecords; this is validation only,
never regeneration to replace missing or corrupt scenes.

Missing scene rows are legal in schema 1: `visual.enabled=false` saves ImageRecords
but no scene. An absent scene is explicitly inventoried as no persisted target
appearance; an empty valid scene contributes zero descriptors. Neither is grounds
to invent target state. A missing scene cannot be distinguished from deletion
without a persisted manifest, so do not claim full deletion detection. A present
malformed scene always fails. No images/scenes is a valid LiDAR-only history,
with empty visual retrieval capability reported explicitly.

For a historical query `q`, eligible targets are exactly `0 <= t < q-3`; exclusion
must occur **before** nearest-neighbor/NNDR search. Retain current top-k,
min-matches, scoring, Hamming/ratio defaults and final tie rules. Exclusion uses
historical submap IDs, not process age, wall-clock time or a new session ID.

At opening, rebuild the prefix for the last existing ID `q=N-1` (empty for fewer
than five nodes). All other valid scenes remain available through Memory; do not
load recent excluded descriptors into the active archive. To inspect another
historical q, adjust the eligible prefix: extend in ascending ID order, or clear
and rebuild the same existing archive for a smaller prefix. Historical query
calls load their persisted query frames transiently; they never call loop
registration or mark a loop decision complete.

For repeatable rebuilding, insert each eligible scene in ascending ID order and
settle `poll_training(true)` at each newly eligible prefix. The first prefix that
crosses the existing training threshold supplies the training population, as in
sequential historical query progression. Use the existing Flat-to-IVF transition,
seed and all parameters unchanged. This settles worker timing; it does not replay
sensor data or loop decisions. No archive `save_cache` or `restore_cache` calls.
Ignore existing Faiss cache files without modifying/deleting them. Payload caches
start empty and remain disposable.

Exact descriptor membership/fingerprints and temporal eligibility are mandatory.
Exact query rankings are compared under equal configuration, eligible prefix and
settled archive mode. A former process's transient Flat-versus-IVF activation
timing is not persisted and cannot be reproduced exactly. Report a ranking
discrepancy under unlike modes; do not call it restored identical behavior, tune
Faiss, or persist a cache to hide it. Changed search policy is allowed by A1 but
is not an equal-policy retrieval-equivalence experiment.

## 9. Schema-1 loop-decision contract

The current supported format has no `Node.loop_decision_complete`. Therefore:

`LoopDecisionStatus = Unavailable`

Expose that fact on the historical backend. Do not infer completion or pending
work from loop Links, their absence, graph_revision, clean shutdown,
searched_loop_id_, or node count. Do not reject an otherwise valid historical
map merely because definite completion state is unavailable. No loop is processed
or scheduled, and continuation remains prohibited.

There is no optional-column reader, alternate schema-1 marker interpretation,
marker writer or marker-state test branch in A2. An added column cannot confer
recognized loop-decision semantics. Unsupported persistence semantics fail
closed, with no migration or speculative future format/version design. Decisions
about changing persistence semantics belong to the actual future implementation,
not this design. SQLite user_version is a current-format guard only, independent
of Sapphire's product version.

## 10. Occupancy reconstruction

Always validate LocalGrid inputs and reconstruct historical occupancy in A2,
even when `navi_map.enabled=false`; that flag controls ordinary production/output
selection and is intentionally excluded from A1 compatibility. Construct an
existing `OccupancyGrid` using exactly the six accepted occupancy parameters.
Do not construct GroundEstimator or LocalGridMaker for historical reconstruction.

Require one FlatGrid row per Node, safe nonnegative counts and exact blob lengths,
finite coordinates/viewpoint, positive finite cell size, and representable
transformed cell coordinates, grid dimensions and allocation sizes. Grid labels
are metric evidence: use the current float transform of `(x,y,0)` followed by
floor at the occupancy resolution, without adding a tilt fix. The recorded
LocalGrid cell size/viewpoint are validated facts; current OccupancyGrid fusion
uses its configured resolution and stored label coordinates, not a new ray cast.
Do not require LocalGrid cell size to equal occupancy resolution: disabled-grid
production can persist an empty default-size grid, and current fusion does not
use that field to rescale labels.

Append `GridFrame(SQL_ID, committed_float_pose, persisted_LocalGrid)` in ascending
SQL ID. Preserve the current per-node cell deduplication, miss-before-hit order,
log-odds clamps and owner-node semantics. Stream grids through existing caches;
no raw clouds, CAPE or ground estimation. Missing/corrupt input or an append
failure aborts reconstruction; legitimate empty evidence gives an empty grid.

The rebuilt grid's **source** is `(map_uuid, graph_revision)` read at open. Record
this using simple backend fields/accessors; do not relabel the internal
`OccupancyGrid::revision()` or `NavigationGrid::revision` as a database revision.
Those existing counters describe local operations and need not equal the former
process's counter. A2's NavigationGrid can retain the local build revision while
the backend exposes the explicit source UUID/revision. No F2 callback/publication
protocol redesign and no automatic navigation callback during construction.

Occupancy acceptance compares exact resolution, origin, dimensions, exported
cells, cell-key membership, observed/occupied meaning, owner IDs and active node
membership under equal input facts and the supported arithmetic environment.
Compare source UUID/revision exactly, not local operation counts. Internal
container/tree/tile-set order is not part of equality.

Source reasoning explains why same-build log-odds bit equality is expected:
`append()` requires ascending node IDs; each frame first reduces contributions
for a cell to boolean miss/hit flags; `fuseFrame()` applies at most one miss then
one hit, with fixed clamps; `GridMapUpdate::commit()` clears dirty tiles and
replays their contributors in sorted node order. Unordered iteration interleaves
**different** cells, without changing the arithmetic recurrence within a cell.
Thus, for identical active poses/evidence, binary/libm, floating environment and
parameters, a given cell follows the same ordered recurrence from zero.

This is conditional determinism of current implementation, not a persisted
floating-state contract. Retain bit comparisons against canonical replay and
uninterrupted runtime as **diagnostics**, not unconditional A2 acceptance gates.
A discrepancy requires checking that the uninterrupted grid actually used the
same committed poses, membership and parameters; former unpublished/private
runtime history is not persisted. Require exact exported output and evidence
semantics under that equal-input condition. If internal arithmetic differs but
output and evidence meaning agree, report the numerical discrepancy without
calling persistent facts unequal. If exported cells, geometry or evidence meaning
differ, the semantic-equivalence test fails; do not invent an unmeasured log-odds
tolerance, add persistent occupancy state, change fusion, or fix F3/F4.

Test the already planned canonical persisted-fact replay and uninterrupted
pose/tile updates. No expansion of the corruption-test matrix is authorized by
this review. Without an old-process/test witness, the database alone cannot
prove what the former in-memory grid contained.

## 11. Correction and active-session semantics

Historical map poses and historical odometry anchors are available, but there is
no active newly attached odometry session. Do not call `updateCorrection()` and
do not publish/use a stored or recomputed historical correction as a new-session
`T_map_odom`.

Add a small correction-availability query to the existing backend; the existing
`T_map_odom()` accessor must throw an explicit unavailable error in historical
mode rather than returning its identity-initialized member as if valid. Ordinary
new-map behavior stays unchanged. No frontend serialization or session object.

`addFrame()` rejects with the existing `ReadOnly` error **before** checking its
ID, payload, enabled flag or queueing. Reject old and prospective IDs alike.
No attachment API is introduced. `localize` remains unsupported. The historical
backend is readable, not enabled for ingestion; accessors/capability checks must
make this distinction explicit. The pipeline's independent input gate remains.

## 12. Strict read-only guarantees

Keep A1's regular-file validation, cooperative shared inode lock, sidecar
rejection, private mapping, read-only deserialize, private WAL-header adaptation,
identity validation and cleanup order unchanged. No writable promotion, new-file
fallback, schema creation/migration, journal recovery, checkpoint, VACUUM,
persistent PRAGMA, cache sidecar generation or manifest writing.

Extend data readers within that read-only handle. Validate the held path/inode
again before making the completed backend available, using the existing identity
check logic; keep its ownership for the runtime lifetime. SQLite statements remain
RAII-local. A normal close and every failure release workers/statements, then
SQLite, mapping and descriptor in dependency order. No destructor processing or
flush of historical facts.

Read-only database methods still enforce `ReadOnly`; backend ingress gates add a
second guard. Derived index/grid building must not reuse persistence-facing
`Memory::saveSubmapPoses()` merely to update runtime indexes.

Acceptance compares complete database bytes and directory entries/sidecar bytes
before open, after construction, after explicit queries/rejected writes, after
close, after a second process repeats, and after injected failures. UUID, config,
revision and row/blob contents also compare exactly for diagnostics. Ordinary
filesystem access-time changes are not database-content mutation.

The accepted cooperative-content-stability limitation remains: this is not a
defense against an external program ignoring locks and truncating/mutating the
mapped inode. No new hostile-filesystem recovery machinery is part of A2.

## 13. Corruption and failure rules

All required-state failures abort construction. Reuse existing `MapError` and
error codes. `ReconstructionDiscrepancy` remains the compatibility code for hard
reconstruction/model failures, wrapped standard exceptions and its existing
occupancy/tentative B1/B2 uses; it is not a corruption diagnosis. Stop throwing it
for finite similarity-only exceedance. Split the combined objective/similarity
condition so nonfinite objectives or delta still throw with precise cause.
Preserve other MapError mappings and bad_alloc-to-Storage handling at the current
boundary. No broad catch/suppression or new error framework/enum is permitted.
Include available table/node/factor, stage and failed invariant in error detail;
use separate reproduction-warning wording for similarity-only diagnostics.

| Condition | Required result |
|---|---|
| Required Node, anchor, bound, cloud, pyramid, navigation or LocalGrid record missing | Historical-data error, no partial ready backend; no regeneration. |
| SQL ID hole/zero/negative/out-of-range; orphan required payload | Historical-data error. |
| Dangling factor, duplicate/conflicting edge, missing adjacent odometry, self-edge, invalid loop direction/gap/multiplicity | Historical-data error. Do not invert, merge or drop it. |
| Transform blob wrong shape, non-finite pose, non-rigid matrix, invalid quaternion | Historical-data error; no projection/repair. |
| Unknown factor type, model or schema; unsupported native serialization ABI | Unsupported-format/model error. Native float/double/size_t encodings are not portable promises. |
| Invalid scene CRC/identity/type, mismatched scene/ImageRecord content, invalid image payload | Historical-data error even with visual output disabled. Legitimately absent/empty scenes are inventoried as in section 8. |
| Non-finite or unrepresentable occupancy coordinates/dimensions, malformed counts, failed append | Historical-data/resource error; never skip a node. |
| Wrong/missing/extra graph key, wrong factor cardinality/endpoints/type, original prior mismatch or wrong noise/robust/model semantics | Hard reconstruction/model failure, regardless of similarity. Preserve canonical typed construction, complete expected-key access and same-Values model checks. |
| GTSAM construction/update/estimate failure or singularity/exception; invalid Pose3 type/dimension; nonfinite committed/rebuilt pose, factor residual, objective or objective delta | Hard failure through the existing storage/reconstruction boundary; no partial ready backend and no finite-only shortcut. Invalid dimensions or NaN hidden by a reduction cannot be classified as reproduction similarity. |
| Finite fresh-versus-committed discrepancy exceeding a section 6 reference, with every hard check passing | Successful historical open plus reproduction diagnostic; no mutation, automatic extra optimization or corruption claim. |
| Training/allocation/I/O failure | Construction failure with cause; no successful partial capability hidden behind a warning. |
| Namespace identity change, ownership conflict or sidecar appearance | Existing A1 failure semantics; no fallback/recovery. |

Decode counts against remaining bytes before allocation; check integer products,
casts and row SQLite types. Existing permissive reads are not sufficient for this
boundary. In particular, validate PyramidVoxel sparse lengths/counts against the
blob and supported allocation limits before invoking its existing CPU decoder;
do this at the storage boundary, without changing BBS algorithms. Also reject
trailing bytes and unsafe expansion/resource requirements explicitly.

## 14. Bounded production-file scope

All paths below are relative to `/home/user/code/sapphire_git/src/`.

| File | Bounded reason |
|---|---|
| `sapphire/src/mapping/storage/map_database.hpp` | Checked Node/Link enumeration, payload/relationship validation, read-only identity recheck and narrow error codes on the existing A1 handle. No change to A1 open/lock/identity encoding, writer transactions or schema creation. |
| `sapphire/src/mapping/storage/memory.hpp` | Explicit historical read-only construction using its owned database; validate before index/cache readiness; checked reader forwarding; populate pose indexes only from committed Node poses and validate redundant pose tags. |
| `sapphire/src/mapping/graph/pose_graph.hpp` | Minimal historical read/status/revision/pose/retrieval and correction-availability accessors on the existing backend. No snapshot/session/context API. |
| `sapphire/src/mapping/graph/pose_graph.cpp` | Synchronous restore branch, shared fixed factor-model setup, graph/ISAM reconstruction and checks, committed-pose index/grid population, input/correction guards, no worker/GPU/processing in historical mode. |
| `sapphire/src/mapping/visual/visual_loop.hpp` | Existing-class methods for loading/querying persisted historical IDs and rebuilding eligible prefixes. No new-input query/session API. |
| `sapphire/src/mapping/visual/visual_loop.cpp` | Share existing feature-block conversion; scene identity validation, prefix reconstruction and fail-closed historical retrieval. Preserve scoring, eligibility and Faiss settings. |
| `sapphire/src/pipeline.cpp` | Only update the existing resume rejection diagnostic to distinguish available backend reconstruction from unavailable input/session continuation; leave the gate closed. |

No expected algorithm changes to `retrieval_index.hpp`, `descriptor_archive.*`,
`scene_features.*`, `scene_archive.cpp`, `occ_layer.hpp`, LocalGridMaker, ground,
registration, GTSAM or Faiss. A test-only friend/access seam in existing graph/grid
headers may be needed to inspect factors/evidence; prefer current public data and
existing test techniques, and do not add production state solely for assertions.
If such a seam is required, `sapphire/src/mapping/grid/occ_layer.hpp` is the only
additional expected header, for read-only evidence/membership inspection through the existing owner, with fusion code unchanged.

Build/test registration changes: `sapphire/CMakeLists.txt`. After approved
implementation/verification, update `docs/CURRENT_STATE.md` and
`docs/MIGRATION_LEDGER.md` with actual evidence; this design itself updates no
capability to implemented/verified. Historical record-validation methods are implemented out of line in
`sapphire/src/mapping/storage/map_database.cpp`, owned by the existing MapDatabase.
This file split introduces no recovery owner or aggregate. `parameters.h` also
receives a mode-comment correction; it introduces no configuration field.

## 15. Exact required tests and evidence

Add `sapphire/tests/historical_reconstruction_test.cpp`, registered as
`sapphire_historical_reconstruction_test`. Use separate executable child modes for
writer, reader and repeat-reader, with explicit process completion/error
communication and timeouts, not sleeps guessing worker readiness. Test artifacts
and numerical oracles belong in a test directory, never map-owned sidecars.

Required cases:

1. **Independent-process success:** process A uses the production backend to
   build/commit/cleanly close a nontrivial map; process B has no raw sensor files,
   opens direct backend historical mode, inspects every required owner and closes;
   process C repeats. Cover empty, single-node, odometry chain and accepted loop.
   Include a recorded loop fixture for integration and small analytical fixtures
   for exact factor inspection. Stop before all continuation/attachment work.
2. **Prior witness:** nonidentity, non-axis-aligned initial anchor; committed node
   0 pose deliberately distinguishable from the original at representable
   precision. Assert prior comes from original Node odometry, not optimized pose.
   Missing/malformed or Node-inconsistent tag, mismatched first timestamp and
   invalid quaternion fail. The optimized tag is not tested as an immutable
   original anchor.
3. **Factor semantics:** noncommuting asymmetric SE(3) odometry and loop transforms;
   inspect actual GTSAM endpoint order/measurement, no extra loop inverse, precise
   diagonal variances and Huber Block model. Assert `N+L` factors and one component.
4. **ISAM reconstruction:** stored committed initial Values, deterministic factor
   order and fixed settings; section 6 hard validity checks and diagnostic
   reference limits. Exercise sub-reference float loss and deliberate finite
   exceedance (successful open plus diagnostic), singular/nonfinite failure and
   repeated-process reproduction. Preserve same-Values factor checks and all
   corruption/ownership assertions. No threshold changes to make a fixture pass.
5. **Pose-index source:** original odometry far from committed map pose after
   optimization; both pose/spatial retrieval return committed Node poses, and
   persisted pose tags match those committed poses. Missing last
   SpatialRecord must fail, including the case an inner join would omit silently.
   Compare spatial candidates to independently computed current-policy bounds.
6. **Visual reopen:** historical query ID reads ImageRecord without `addSubmap`;
   recover descriptors, fingerprints/counts, cameras and scene identities. Test
   IDs 0..4 boundary, last three adjacent exclusions before NNDR, backward query
   prefix rebuild and repeated queries. Include no-scene/empty-scene LiDAR-only
   histories; malformed present scene is fatal, not silently skipped.
7. **Faiss modes:** fixtures below and across the unchanged training threshold;
   compare settled equal-prefix/equal-policy outputs, node metadata and matching
   identities across processes. Test no disk cache, an irrelevant stale cache
   unchanged, training failure, and explicit distinction from transient warm-index
   ranking. No Faiss tuning or benchmark deliverable.
8. **Loop status:** every valid current-format history reports `Unavailable`,
   whether or not it contains loop Links. Opening succeeds despite unavailable
   completion state; assert no registration, completion update, pending work or
   continuation. Remove the optional-marker test branch entirely.
9. **Occupancy:** fresh persisted-fact replay versus A2 versus uninterrupted
   runtime, including historical pose/tile updates, negative cells, overlapping
   hit/miss, clamping, empty grids, disabled output and nonzero roll/pitch under
   current F3 semantics. Compare every evidence field and exported grid as in
   section 10; report discrepancies without changing the grid algorithm.
10. **Source revision:** reconstruction reports original UUID/R and occupancy
    source UUID/R while local counters retain their distinct meanings. No node-
    count-to-revision inference, no claim that schema-1 R certifies loop completion.
11. **Input/correction gates:** valid new-ID and old-ID addFrame calls both throw
    ReadOnly before mutation; disabled flags cannot bypass. Correction unavailable;
    historical pose lookup succeeds. Pipeline resume/localize still starts no
    frontend, backend processing or GPU worker. Historical construction never
    calls publication callback or `processPending`, including on destruction.
12. **Read-only repeatability:** full file-byte and directory/sidecar manifest
    comparisons around construction, historical spatial/visual/grid queries,
    failed writes, close and second reopen. Use read-only file/directory fixtures
    and accepted clean WAL-header representation. Assert no persistent changes
    including header bytes 18/19, UUID/config/poses/revision.
13. **Corruption matrix:** mutate separate baseline copies for each section 13
    failure: all required missing rows, dangling endpoint, malformed duplicate
    tables, reversed/conflicting factors, ID holes, unsupported types/version,
    bad transforms, oversized/truncated/overflowing payload counts, invalid
    pyramid, corrupt visual identity and unreconstructable grid coordinates.
    Every failure must preserve that fixture's bytes and expose no partial backend.
14. **Cleanup/failure injection:** fail after database open, validation, factor
    build, ISAM update, spatial build, visual training and grid append. Verify
    no processing on teardown, no descriptor/worker leak, released locks, and
    unchanged persistent files. Reopening a known-good map afterward succeeds.

### Numerical-support amendment verification

The detailed deterministic oracles are in the [amendment §L](../../audit/2026-09-29-a2-contract-amendment-design/README.md#l-required-implementation-tests--future-only).
They supplement accepted A1/A2/W/B1/B2/B3 regressions. The
[independent implementation review](../../audit/2026-09-29-a2-contract-amendment-independent-implementation-review/README.md)
records completed verification and its documented limitations; older phase-only
stop instructions above do not exclude that continuation oracle.

| Case | Required result |
|---|---|
| A. Retained F1 R5 | Original frozen DB; all hard checks pass; open succeeds with measured exceedance, structured metrics/warning, exact committed products and read-only byte/identity preservation; correction unavailable. |
| B. Real R5→B1/R6→B2/R7→checked close→independent reopen | Fixed displaced target and real BBS/GICP; all-key proposed/delta/DB equality, original facts preserved, C/Y and product/correction attribution coherent, old products retain attribution. Count normal updates; no settlement or extra optimization. |
| C. Inside-envelope control | Existing bounds and behavior retained, no exceedance warning; same process/product checks, plus empty and single-node cases. |
| D. Structural/model failures | Existing invalid maps/error codes still fail with bytes/ownership preserved; exercise complete keys, wrong typed/dimensioned Values, prior/direction/type/noise/Huber errors independent of movement. Inspect constructed factors; test-only injection where the invalid model is not persistable. |
| E. Nonfinite/solver/product failure | Fail closed for poses/residuals/objectives/delta, overflow and solver exceptions; no partial ready owner. Retain B1/B2 mutation-entry, W outcome, materialization and checked-failure regressions. |
| F. Product authority | Distinguish grid/world-bounds products at X* from X^R; actual A2 uses committed products and persisted visual evidence at opening, later revisions and final reopen. No new public solver-output API. |
| G. All-key reconciliation | Retain existing outside-affected/append-hint historical movement regression, exact serialized comparisons, unchanged-float exclusion and independent product/reopen oracle. |

Test all four diagnostic component boundaries with deterministic classification
fixtures: below/equal/above reference, equality within; invalid/nonfinite values
fail before classification. Keep previous control reference assertions. The
coherent 0.2 m Node-plus-pose-tag fixture changes only its similarity-rejection
expectation to success/diagnostics when hard checks pass; retain its preservation
checks and all true malformed/mismatched-data failures. It does not prove complete
corruption detection without independent provenance. Run the required existing
core and relevant ROS regressions; retained evidence is not a new test run.

Extend `sapphire/tests/map_open_test.cpp` only where the direct-backend A1 stop
gate is intentionally replaced by A2 readiness/data-validation behavior. Keep
storage eligibility-only and pipeline input-gate expectations, and every A1
byte/path/config/ownership/duplicate-node protection regression.

Extend `sapphire/tests/map_storage_test.cpp` for strict readers and the original
odometry-versus-committed-pose distinction, including committed pose-tag consistency. Extend
`sapphire/tests/visual_mapping_test.cpp` for the persisted historical-query and
prefix behavior as appropriate; do not weaken its existing tests.

Run the new test plus existing map_eligibility, map_open, map_storage,
visual_mapping and full CTest regressions in the supported build. GPU-dependent
loop fixture/regressions retain their normal environment requirements; the
**reader** must demonstrate no CUDA dependency even when the writer used CUDA.
Record build/configuration, commands and numerical maxima. The standalone
design-time numerical probe was executed; no production tests were added or run.
A3/A5 continuation acceptance and
F2 drain/publication validation remain separate stages.

## 16. A1 assumptions checked against actual source

No inspected evidence invalidates the accepted A1 safe-open, namespace, read-only
eligibility, insert-only Node or six-field compatibility contract. No A1 fix is
proposed. The source instead rules out stronger assumptions that A1 explicitly
did not promise:

| Stronger assumption | Source evidence and design consequence |
|---|---|
| Eligibility certifies a complete graph | `MapDatabase::saveSubmap` and pose/Link commits are separate; PHASE1_DESIGN explicitly excludes completeness. A2 validates historical contents. |
| Every map records its loop-decision state | Node schema has no marker; runtime-only `searched_loop_id_` is lost. Return Unavailable; no optional-marker support. |
| The old double graph can be recovered exactly | `toIsometry3f` writes both odometry/loop Links and committed poses. Preserve persisted measurements and model semantics; quantify precision loss. |
| Any SubmapFrame's first odometry tuple is guaranteed to be its anchor | Production buffer establishes this, but public constructor validates only count/order/finiteness. Original odometry supplies the supported-producer anchor; the mutable pose tag cannot independently prove it. |
| Existing Memory index initialization is ready for reopening | Correction to the initial design: saveSubmapPoses also updates kPose tags to committed poses. Keep Node authoritative and validate redundant tags; the claimed original-anchor overwrite is not established. |
| Calling visual query after reopen restores queries automatically | VisualSubmapIndex queries require runtime `pending` feature blocks; absent pending returns empty despite indexing scenes. Load persisted ImageRecords for historical read-only queries. |
| An absent VisualScene necessarily means corruption | Scene creation is conditional on visual_index_. No persisted manifest distinguishes disabled target appearance from deletion. Explicitly represent absence; never invent a scene. |
| Local occupancy counter is database revision | `occupancy_revision_`, OccupancyGrid's revision and graph_revision advance separately. Expose source UUID/R explicitly; do not redesign F2. |
| Scope requires recovery of ground/front-end runtime | Historical grids are already persisted. A2 creates neither ground-estimation state nor an active frontend correction/session. |

Primary source anchors inspected:

* [A1 accepted boundary and future short-transaction model](/home/user/code/sapphire_git/src/docs/PHASE1_DESIGN.md).
* [Backend opening and fixed noise models](/home/user/code/sapphire_git/src/sapphire/src/mapping/graph/pose_graph.cpp).
* [Prior, odometry and loop construction](/home/user/code/sapphire_git/src/sapphire/src/mapping/graph/pose_graph.cpp).
* [Solver, correction and occupancy update behavior](/home/user/code/sapphire_git/src/sapphire/src/mapping/graph/pose_graph.cpp).
* [Persisted base payloads](/home/user/code/sapphire_git/src/sapphire/src/mapping/storage/map_database.hpp).
* [Schema and Link encoding](/home/user/code/sapphire_git/src/sapphire/src/mapping/storage/map_database.hpp).
* [Memory's index-loading sequence](/home/user/code/sapphire_git/src/sapphire/src/mapping/storage/memory.hpp).
* [Production original-anchor selection](/home/user/code/sapphire_git/src/sapphire/src/common/key_frame.hpp).
* [Visual temporal eligibility and pending-query dependency](/home/user/code/sapphire_git/src/sapphire/src/mapping/visual/visual_loop.cpp).
* [Faiss fingerprint and index training](/home/user/code/sapphire_git/src/sapphire/src/mapping/visual/descriptor_archive.cpp).
* [Persisted-grid fusion semantics](/home/user/code/sapphire_git/src/sapphire/src/mapping/grid/occ_layer.hpp).
* [Pipeline input-mode gate](/home/user/code/sapphire_git/src/sapphire/src/pipeline.cpp).

The consolidated implementation instruction settles this review. A2 retains DB
fact reconstruction, float persistent poses/Links and double GTSAM computation;
no native optimizer serialization, Boost.Serialization dependency or g2o
production functionality is added. Current implementation evidence and remaining
limitations belong to CURRENT_STATE.md. A3/A4/A5 remain outside authorization.
