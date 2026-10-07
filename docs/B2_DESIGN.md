# B2 bounded continuation design

Status: **B2 design accepted with required clarifications — ready for bounded
implementation**, 2026-09-28. The fixed-K clarification is incorporated below.
This documentation-only pass authorizes no production or test implementation.
**Policy C is the accepted B2 baseline:**
hard recent-neighbour exclusion only within the same type-0 odometry chain,
with fixed native K=3 as specified in section 5. Existing spatial/visual proposals,
deterministic ranking, BBS→GICP and one accepted ordinary type-1 per query remain.
Full-graph soft ranking is experimental and is not part of this baseline.

The [completed bounded research](B2_LOOP_ELIGIBILITY_RESEARCH.md) is evidence,
not implementation authorization or proof of empirical superiority. Real-data
evaluation may falsify Policy C. This revision incorporates the review's interim
throughput conclusion: no prerequisite W optimization is justified; retain its
correctness scans and evaluate sustainable representative arrivals with bounded
backlog, not strict O(1) service time (section 13). Narrow W/A2 admissibility
changes and the same-format software compatibility decision are explicit below.
**The independent B2 design review is complete; B2/B3 implementation is not begun
or authorized in this documentation-only pass.**

## 1. Foundation, scope and source evidence

B2 continues Q1, Q2, Q3… after B1 has attached Q0, all from the **same live fresh
odometry domain**. It is ordinary mapping, not repeated attachment. DB facts are
authoritative; use the existing backend, Memory, visual and occupancy owners.
Follow [BACKEND_FAILURE_SEMANTICS](BACKEND_FAILURE_SEMANTICS.md).

Reviewed foundation: [AGENTS](../AGENTS.md), [CURRENT_STATE](CURRENT_STATE.md),
[MIGRATION_LEDGER](MIGRATION_LEDGER.md), [A1](PHASE1_DESIGN.md),
[A2](A2_DESIGN.md), [W](../../audit/2026-09-27-w-design/W_DESIGN.md),
[W-F1](../../audit/2026-09-27-w-f1-correction/W_F1_CORRECTION.md),
[W.1](../../audit/2026-09-27-w1-correction/README.md),
[B1 design](../../audit/2026-09-27-b1-implementation/PLAN.md),
[B1 implementation](../../audit/2026-09-27-b1-implementation/README.md), and
[B1.1 acceptance](../../audit/2026-09-27-b11-acceptance/README.md).
The [independent teardown acceptance](../../audit/2026-09-28-teardown-acceptance/README.md)
explicitly says “Lifecycle correction accepted — proceed to B2 design”. This
satisfies the prerequisite despite the older state documents' review-pending
headings. Old A3/A4/A5 and T1/T2 proposals are superseded by accepted A2/W/B1.

Implementation truth inspected in the current dirty tree:

| Existing source | Finding relevant to B2 |
|---|---|
| `mapping/graph/pose_graph.cpp`: attachment, materialization, worker | B1 is synchronous, retains speculative ISAM, rebuilds committed owners once, sets `attached_`; resume starts no worker and `addFrame` rejects. Old `copyPendingFrames/processPending` persist before loop completion and cannot be reused as B2 orchestration. |
| `mapping/storage/memory.hpp` | W forwarding does not materialize indexes. `saveSubmapPoses` also writes SQL; it is unsuitable after a W commit. Existing pose/spatial owners already support updates. |
| `mapping/storage/retrieval_index.hpp`, `common/rtree.hpp` | Single-pose upsert is incremental; multi-pose upsert rebuilds the whole pose KD-tree. AABB upsert removes the exact old stored entry before inserting the replacement. Spatial query currently requires a stored query ID. |
| `mapping/registration/loop_closure.cpp` | Preparation loads query cloud from DB and requires its indexed bounds. Eligibility is global ID gap >3; visual candidates are prioritized; first accepted BBS/GICP result wins. |
| `mapping/visual/visual_loop.cpp`, `descriptor_archive.cpp` | B1 fills all committed targets. Historical queries can clear/rebuild prefixes. New query currently dispatches to persisted ImageRecords in historical mode. Archive upsert, erase and existing Flat→IVF transition are available. |
| `mapping/grid/occ_layer.hpp` | Active-frame/tile ownership supports remove/reinsert and ordered dirty-tile refusion. Full dense export still scans/copies map extent. |
| `mapping/storage/map_database.cpp`: finalized commit | One accepted W transaction is sufficient. W.1 preflight reads **all** historical LocalGrids; inside BEGIN it rereads anchors/Links and validates full topology. |
| `pipeline.hpp/.cpp`, ROS lidar/input adapters | Backend and marginal deques, core IMU/LiDAR queues are unbounded. Images cap at 16 by dropping oldest. Dual-lidar synchronization has bounded queues and drop counters. Pipeline resume transport is still gated. |

Paths in this table are relative to `sapphire/src`, except ROS sources under
`sapphire_ros2`. The design-time source manifest covers 2,511 existing repository
files; evidence and isolated probes are outside the Git root in
[the B2 audit directory](../../audit/2026-09-28-b2-design/README.md).

## 2. Exact starting state and retained runtime fields

At successful B1 return: SQL Q0=N, key N−1, is durably committed at revision R;
Q0 has exactly one `(Q0,H,type=1)` attachment to earlier history, no predecessor
type-0, and no new prior. Node 1's original prior remains the sole gauge anchor.
All existing runtime owners describe R. `T_map_odomNew` is available and derived
from Q0's serialized committed pose and immutable original anchor. W owns the
writable DB exclusively. Frontend odometry is live and has not reset.

B2 adds only ordinary fields to existing owners: active root SQL ID, last
committed active-session key, next expected **frontend-local** submap sequence,
volatile producer generation, runtime-ready revision, and ordered admission/head
status. Preserve Q0's original frontend ID before B1 calls `withPersistentId()`;
its successor is the first expected producer sequence. Do not confuse that
sequence with global SQL identity. The existing pipeline `session_id_` may supply
the generation; none of this is persisted, inferred from timestamps alone, or
restored as an active session on reopen. Historical type-0 chain roots are
independently derivable from persisted topology as section 5 specifies; that
reconstruction does not establish a live producer or correction. No session
object/table/counter/UUID is added to the DB.

At the one-time B1→B2 handoff, validate the live producer binding, canonicalize
visual eligibility as in section 7, and start the existing backend worker.
Separate ingestion readiness from `historical_` (which currently means opened
from history): attached + healthy + current + matching generation permits B2.
Do not simply change the current `historical_` branch to run the legacy worker.
Worker startup/canonicalization failure fences the owner; Q0 remains committed.

`frames_` keeps immutable original anchors. Memory's pose index is the committed
lookup. Existing `optimized_` retains A2/B1's committed-values role; update its
new/changed entries after COMMIT, rather than retaining another permanent full
pose map. ISAM's internal estimate remains speculative solver state. Temporary
proposed Values and a changed-pose vector live only for the processing head.

## 3. Persistent identity, odometry and revision rules

If maximum committed SQL Node ID is N, the processing head proposes SQL N+1 and
backend/GTSAM key N. Reserve neither ID nor revision at upstream enqueue. Assign
the global key with existing `withPersistentId` only to the head's frozen input;
preserve all original odometry timestamps/poses and payload evidence.

For committed predecessor Qprev and current Q, both from the active domain:

```text
A_prev = T_odomNew_Qprev
A_curr = T_odomNew_Q
Z_prev_curr = inverse(A_prev) * A_curr       // maps Q coordinates into Qprev
SQL Link = (N, N+1, type=0, float(Z_prev_curr))
GTSAM BetweenFactor = (N-1, N, double(float(Z_prev_curr)))
```

Use the serialized measurement in the tentative factor as B1 does, so the live
factor and A2 reconstruction consume the same stored model. Keep the existing
rotation/translation variances `[1e-6,1e-6,1e-6,1e-4,1e-4,1e-4]`.
Do not derive Z from optimized/map pose differences or correction. Do not alter
original anchors when optimized poses move. W validates anchor/Link consistency
with its existing float-representation tolerance.

Require predecessor to equal the last committed node of this live generation,
whose chain root is Q0. Persistent adjacency alone is insufficient to establish
same-frame provenance. No type-0 crosses B1, reset, restart or another odometry
domain. Range-check SQL signed-int and revision exhaustion before mutation.

One finalized node gives exactly one R→R+1. No revision for search/rejection,
optimization alone, materialization, correction, export or publication. Precommit
rejection leaves N available to the same ordered head's retry. After a durable
commit, never reuse N+1. On Unknown, issue no further identity; recovery decides
which complete state exists. A restarted frontend then needs a fresh B1 root,
even if an operator retained the old uncommitted payload for diagnostics.

## 4. One head's Route-S processing sequence

1. Acquire the head from a bounded FIFO without draining it into an unbounded
   local batch. Check health/generation/order, expected UUID/R/next ID/root and
   immutable anchor agreement with the first odometry sample. Read previous
   committed active anchor and pose. Input/refusal remains visible.
2. Consume the head's already finalized LocalGrid supplied with the frozen input,
   extending the existing InputFrame fields rather than adding a new owner.
   Build the existing appearance scene and transient query FeatureBlocks. No
   historical grid/cloud regeneration, committed membership insertion or SQL
   write occurs here. The pipeline producer creates new-grid evidence once,
   while holding its single not-yet-admitted frozen item, using the existing
   ground/grid maker initialized for this fresh session. It must not infer an
   old estimator from A2. GroundEstimator::update is itself stateful: failure
   there cannot automatically be called retryable. Fence that producer/domain
   if its state is uncertain; retain successful finalized evidence across backend
   capacity/preparation retries. No ground algorithm or F3/F4 change is proposed.
3. Place the transient query for retrieval at
   `X_prev_committed * inverse(A_prev) * A_curr`. This seed uses current committed
   map placement; it is not the odometry measurement. Retrieve and verify ordinary
   loops as section 5 describes. No accepted loop is a successful result.
4. Prepare the type-0 factor, optional single type-1, initial Values and all
   local allocations feasible before ISAM. Initial Q pose uses the same
   committed-derived seed. Verify finite/rigid/representable inputs locally;
   W retains its authoritative final checks.
5. Set the speculative-mutation boundary **before** entering the first
   `updateIsam` call. Submit the base and optional loop in one local factor graph
   and reuse the existing bounded update sequence. Combining them avoids an
   early speculative odometry update solely for candidate retrieval. This is an
   orchestration change, not extra iterations or a new noise/registration model;
   test numerical/reopen compatibility explicitly.
6. Validate estimate keys, finite objective and rigid/float-representable poses.
   Serialize **every** proposed pose to current float encoding and compare every
   key with Memory's committed float pose. Build exact changed set D, always
   including Q. Do not filter by `affected_keys` or double epsilon. This catches
   residual A2 solver-versus-committed differences outside reported affected keys.
7. Call W exactly once with section 6's inputs. Until it succeeds, no committed
   pose/index/grid/correction is updated. W preparation failure after ISAM is
   sticky even if SQL BEGIN was never reached.
8. After known COMMIT, record DB revision, mark runtime unavailable and apply D
   to existing owners in sections 7–9. Use the exact float matrices passed to and
   committed by W, not a new ISAM estimate. W writes those bytes unchanged; this
   is a temporary commit delta, not a second source of committed truth. Unexpected
   row/membership discrepancies fail closed. No full history readback is needed.
9. Prepare correction and, if requested, immutable grid output. After **all**
   required owners succeed, advance runtime-ready revision to R+1 and expose
   correction/read services. Mark this input completed and notify waiters; only
   now may the next head proceed. Discard transient Q query blocks/pose delta.

Hold existing lifecycle ownership around head processing/materialization so
readers cannot observe partial owner updates. Read-only queries can block; they
must recheck Failed/current after acquiring the lock. Admission waits use a
different mutex and never hold lifecycle/output/DB locks while waiting.

## 5. Ordinary loop processing on a transient query

### Shared temporal contract — accepted Policy C baseline

Define hard temporal eligibility **once**, independently of retrieval source,
ranking or storage owner. IDs in the following pseudocode use one common domain
(SQL IDs are one-based; backend keys are SQL−1).

**K=3 is the fixed same-chain temporal-exclusion constant for the current
Policy-C/schema-1 semantic contract.** The same fixed native constant is used by
production retrieval eligibility, visual pre-NNDR prefix calculation, W ordinary-
loop preflight, shared A2/W topology validation, and corresponding production/
reopen tests. K is not runtime configurable, not independently configurable
between writer and reader, and not inferred from the map. It is not a world-
modeling principle and is not claimed physically optimal. Chain awareness fixes
the cross-chain identity error; it does not solve submap/keyframe-density
sensitivity.

The K argument below and K in subsequent formulas are explanatory/test notation
for this fixed native constant, not a production configuration parameter.

```text
temporallyEligible(query, target, validatedType0Topology, K):
    if target is not committed or target.id >= query.id:
        return false
    if root(query) != root(target):
        return true
    return type0_chain_distance(query, target) > K
```

`type0_chain_distance` counts edges on the type-0 chain; it is undefined/infinite
across chains. Neither type-1 edges, full-graph shortest paths, spatial distance,
attachment proximity nor global ID gaps across roots define temporal adjacency.
Later ordinary type-1 insertion cannot alter this hard predicate for existing
query/target identities and type-0 structure. B1 root attachment retains its
separate accepted contract; this is not a generalized B1 adjacent-target exception.

Derive roots using the existing validated topology: SQL Node1 starts root1;
a valid `(d−1,d,type=0)` inherits its predecessor's root; a noninitial node with
no type-0 predecessor must satisfy the accepted type-1 root-attachment contract
and starts root d. Preserve endpoint, uniqueness, original-anchor and contiguous
ID validation. Type-1 connectivity never merges type-0 domains. For transient Q,
validate its planned predecessor as the last committed node of the active live
domain and inherit that node's root; a caller-supplied root alone is not proof.

Under current contiguous, non-interleaved append-only chains, same-chain distance
is the ID difference **only after chain equality is established**. For the active
tail, targets before its root are other-chain; targets between root and query
are same-chain. An arbitrary historical query uses its own derived root interval,
not the newest/active root. Root derivation costs O(N+E) during existing validation;
a derived root vector gives O(1) lookup, or sorted roots O(log S) for S chains.
The active tail's root/ID predicate is O(1). No persistent session identifier or
second topology owner is needed. Interleaved chains would require a separately
reviewed invariant, not reuse of this interval optimization.

One small dependency-light pure helper should express this semantic predicate;
existing owners supply validated identities/roots and translate SQL/key domains
at their boundaries. No new manager/class is required. The following consumers
must share it or a proved equivalent prefix derived from it:

| Consumer | Required use |
|---|---|
| Transient spatial candidates | Query the committed index, then filter before payload preparation/BBS/GICP. Spatial exclusions do not affect an NNDR denominator. |
| Visual searchable membership | Exclude recent same-chain descriptors **before** nearest-neighbor/NNDR computation; previous-chain nearby IDs remain searchable. Section 7 derives the equivalent prefix. |
| Merged proposals | Deduplicate spatial/visual proposals and defensively recheck the same predicate. Preserve existing deterministic visual-first ranking and tie rules. |
| W optional-loop preflight | Check the planned ordinary query/predecessor/root using this predicate, with authoritative topology revalidation before application writes (section 6). |
| A2/W persisted graph validation | Apply the predicate to ordinary type-1s using independently validated type-0 roots; preserve the distinct B1 root contract (section 6). |
| Independent tests/oracles | Use identical chain-aware membership and revision-aligned inputs (section 15). |

Eligibility means a target may be proposed/verified, not that every historical
target must be searched or that a loop is accepted. Keep the union of existing
spatial and visual evidence; do not require a visual proposal to also pass the
spatial gate. After Policy C, use existing candidate preparation → BBS → GICP →
first accepted ordinary type-1 → optimization.

### Full-graph distance and optional future ordering

Full type-0/type-1 shortest-path distance is **not a baseline persistent
ordinary-loop validity invariant**, nor an additional retrieval gate. Its metric
changes when a loop is accepted: a stored edge makes its own endpoints one hop
apart, later edges can shorten old paths, and wrong historical type-1s can distort
connectivity. Geometric connectivity is not temporal adjacency. Do not impose a
full-graph threshold in retrieval, W or A2; reopening must not retroactively
invalidate an admissible edge because its own or a later type-1 exists.

A future optional experiment may order the Policy-C eligible set using full-graph
or metric novelty before unchanged BBS/GICP. **It is disabled/not implemented in
the B2 baseline.** Preserving membership does not guarantee equivalent attempted
or accepted loops: finite budgets and first-success stopping can starve candidates.
Compare any experiment at equal attempt/time budgets. No persistent ranking state,
no ranking criterion in W/A2 validity, no new verification scheduler and no new
multi-loop-per-query support are proposed.

Two small API seams are necessary because Q is not yet in SQL:

- Add an existing spatial-owner query overload taking explicit local AABB and
  map pose. Use the identical AABB transform, intersections and >0.5 XY overlap
  rule against committed targets; do not temporarily insert Q into an index.
- Add a candidate-preparation overload consuming Q's transient cloud/bounds and
  seed. Retain the existing DB-backed overload for historical callers/tests.
  Factor shared preparation so ranking, deduplication, eligibility, target
  pyramid loading and visual-first order do not diverge. Transient visual query
  takes FeatureBlocks from Q instead of loading nonexistent ImageRecords.

Spatial seeds are `inverse(X_target_committed)*X_query_seed`; visual candidates
retain existing translation-zero seed and configured wider search window.
Reuse BBS sampling/search and GICP gates without tuning. Source eagerly prepares
candidate pyramids and has no global spatial candidate budget; section 13 records
its cost. Legitimate BBS/GICP rejection tries the next candidate; exhausted
candidates gives no-loop. Storage/decoder/training/registration exceptions are
not silently converted to no-loop or hidden behind the old warning-and-continue
visual catches.

Registration returns `T_H_Q`. The ordinary loop is SQL `(Q,H,1)` and GTSAM
`(q,h)` with measurement `float(inverse(T_H_Q))`; its prediction is
`inverse(X_Q)*X_H`. Keep diagonal noise and Huber 10 / Block. Stop at the first
accepted result, preserving one type-1 per query. Independently verify this
direction with asymmetric noncommuting SE(3), actual factor residual and SQL.

## 6. Exact W finalized commit inputs

Call the existing Memory→MapDatabase `commitFinalizedSubmap` with:

| Argument | B2 value |
|---|---|
| Frozen submap | Q with backend key N, immutable full original odometry, cloud/pyramid, local bounds, navigation and ImageRecords |
| LocalGrid | Q's finalized local ground/obstacle/empty evidence, cell size and viewpoint |
| Scene | Existing encoded appearance scene (valid empty/absent follows producer policy) |
| Poses | Q's final serialized map pose and every changed historical serialized pose in D, unique SQL IDs |
| Base | Exactly one `(N,N+1,0,Z_prev_curr)` |
| Optional loop | Zero or one geometrically accepted `(N+1,H,1,inverse(T_H_Q))`, satisfying the shared Policy-C temporal predicate in section 5 |
| Expectations | Captured map UUID, committed R, next SQL N+1, existing `c2:` config identity, active Q0 SQL chain root |

W inserts Node, SpatialRecord, LaserRecord, PyramidVoxel, NaviTrajectory,
FlatGrid, ImageRecords, optional VisualScene and required tags; persists the
base/optional loop; updates all changed Node poses and corresponding pose tags;
increments revision once, all in **one** transaction. Original odometry remains
unchanged. No independent `saveSubmap`, `saveLink` or `saveSubmapPoses` transaction.

No-loop is a complete finalized node. No `loop_decision_complete`, pending-loop
table/work, T1/T2 or retrospective old-node loop replay. Generic historical
schema-1 completion provenance remains `Unavailable`.

### Narrow W preflight and A2/W validator amendment

Replace the existing finalized ordinary-loop preflight's global `id-target>3`
check with section 5's shared temporal predicate. Validate the planned type-0
base `(N,N+1,0)`, predecessor provenance and inherited root. Preflight may use
prepared validated root facts, but its expected root is not authoritative until
W checks it against persisted topology under existing ownership. Keep the current
in-transaction identity/revision/topology recheck before the first application
write: validate old roots, add the planned type-0 base, derive Q's root, apply the
same predicate, then validate the complete proposed graph and expected root.
No new validation cache, parallel root authority or locking path is needed.

In shared `validateTopology`, derive all type-0 roots after validating base
structure. For each type-1, a noninitial root uses the accepted B1 attachment
contract; an ordinary query with its type-0 predecessor must satisfy section 5's
predicate. This same amended validator serves independent A2 reconstruction and
W old/proposed-graph checks. Later type-1 edges cannot change this admissibility.
Do not reconstruct historical candidate ranks or replay geometric verification
on reopen: A2 validates persistent structural admissibility, not whether a loop
was the best proposal at its creation time. W likewise validates admissibility,
not ranking optimality.

Retain backward type-1/forward type-0 endpoints, unique predecessor and type-1
slots, at most one ordinary loop per query, root attachment semantics, original
anchors, factor measurements/noise, W.1 representability scans and rejection,
atomic transaction/revision rules and CommitOutcome handling. W locking, inode
identity, descriptor admission and durability architecture are unchanged. This
is a narrow graph-validity amendment, not W redesign or throughput optimization.

### Accepted compatibility decision — retain native format `user_version == 1`

The proposed amendment **broadens software-supported graph admissibility under
the same storage format**. Tables/columns, Node/Link IDs and directions, Link
types, measurements/noise/prior interpretation, payload encodings, map UUID and
six-field `c2:` identity remain unchanged. A nearby cross-chain ordinary type-1
still denotes exactly the same measured relative-pose factor. It does not need
a session field, new Link type or alternative byte interpretation. Existing
valid global-gap maps remain valid under C; the converse need not hold.

Therefore retaining `user_version == 1` is appropriate as the existing native
format-identity guard under the accepted design. Format identity
is not a promise that every older binary supports every admissible graph.
Older A2/W binaries may reject newly valid edges such as SQL101→99; their
existing global-gap check fails closed rather than reinterpreting stored bytes.
Do not promise old-reader/writer compatibility for such maps, silently downgrade
them, omit their edges or add migration/version-negotiation infrastructure.
Include explicit old-reader rejection and new-reader acceptance fixtures in
section 15. No persistent-format change is unavoidable for this proposal.

## 7. Incremental pose, bounds and visual materialization

### Pose and spatial owners

Add a DB-free committed-delta operation to Memory, callable under backend
lifecycle ownership only after known COMMIT. Precheck expected existing IDs and
new contiguous ID. For each D entry call **single-entry** `pose_index_.upsert`;
do not use its vector overload, which rebuilds the KD-tree whenever size >1.
For existing spatial entries use `updatePoses`, preserving their validated
persisted local bounds. Insert Q with its just-persisted local bounds/pose.
Require existing membership explicitly: current `updatePoses` silently ignores
unknown IDs and is not by itself a sufficient consistency check.

`DenseAABBIndex::upsert` removes `(old_world_AABB,id)` using stored old bounds,
then inserts the new transformed AABB. Both old-only and new-only query regions
must be tested. No cloud decoding/rebuilding/registration belongs in this work.
Any allocation/index exception after COMMIT is sticky; partial KD/R-tree updates
remain inaccessible. Original `GraphFrame` anchor is appended once; historical
anchors do not change. Update only D in existing committed `optimized_` Values.
Cloud/grid caches may retain immutable payloads because map poses are separate.

### Visual owner

Use section 5's Policy-C predicate to define searchable descriptors. For active
append-only continuation, let r be the **zero-based** key of the active root and
q the zero-based transient query key (q>r). SQL identities are r+1 and q+1.
The equivalent exclusive boundary is:

```text
b(q) = max(r, q-K)
eligible committed target keys = [0, b(q))
```

Compute q−K in a checked signed domain (or clamp it to zero before the maximum)
to avoid unsigned underflow. With K≥0 and q≥r, b(q)≤q; no query/future target can
enter. For ordinary historical A2 queries use that query's derived root and
committed target range with the same predicate; reconstruction creates no active
session. Tests must prove the prefix/helper equivalence and SQL↔key conversion.

Example: SQL root100/query101 means r=99/q=100; K=3 gives b=99, hence SQL1..99
searchable and SQL100 excluded. Query102 keeps b=99; query103 has b=99; query104
has b=100 and admits root100 once its same-chain distance exceeds K. Previous
chains' descriptors remain searchable throughout, irrespective of nearby SQL IDs.
Recent excluded descriptors cannot compete in nearest neighbors or the NNDR ratio.

B1's `materializeCommittedRoot(count)` includes every committed scene including
Q0. Before opening B2 admission, perform **one explicit prefix canonicalization**
to b(first Q1), using the existing ascending insertion/settled-training behavior.
For Policy C at Q1 this removes Q0 from the searchable population, retaining all
prior chains. Merely erasing Q0 from an index trained on a different population
does not prove the equal-population/equal-training oracle. One historical pass
is allowed for the handoff, never per ordinary Qn or as failure fallback. Amend
the existing visual preparation path to consume the reviewed boundary rather
than falling through to the old global-gap population and rebuilding again.

Thereafter maintain committed `node_count` and exclusive eligible `next_history`.
Search Q from transient FeatureBlocks; never call persisted `queryHistory()` for
a row that does not yet exist. Scratch stays local and is discarded on retry.
After known commit, advance committed count once and prepare b(next query):
insert only newly eligible persisted scenes, in ascending SQL ID, using existing
`DescriptorArchive::upsert`. The boundary advances by zero or one in ordinary
continuation; early heads may add no scenes. A committed active-chain node enters
only after leaving the same-chain recent K window. Committed history and active
searchable descriptors are therefore distinct populations. There is one monotone
archive, no second Faiss archive, no per-query full erase/rebuild and no Faiss
persistence/cache. Root changes require the already-defined reset/reopen/B1
boundary, not an unnoticed reinterpretation of an active prefix.

Keep scene landmark IDs, descriptor bytes, SQL identity and fingerprint exactly.
Absent/empty scenes advance the prefix without fabricated descriptors. Pose
changes do not alter appearance identity and need no archive reinsert.
`upsert` already does nothing for identical ID/fingerprint. In healthy operation
the committed count/prefix prevents duplicate insertion; after a partial archive
failure stop and reconstruct, never retry that partially materialized revision.

Retain 8,192-descriptor Flat→IVF threshold, 128 lists, sample cap, seed, training
and query parameters. For reproducible equal-prefix behavior, settle existing
`poll_training(true)` when advancing an eligible prefix before declaring it
ready, as A2 does. Its one transition may stall a head; bounded upstream admission
handles this. Training activation reindexes live slots once, not every node.
No Faiss cache/persistence or new archive owner.

For B2 active mode, prevent arbitrary historical query calls from silently
rewinding this archive: forward/current-prefix diagnostics may run serialized;
a smaller-prefix query returns explicit unsupported-while-continuing status.
Unattached A2 retains arbitrary backward-query capability, with searchable
membership amended to the same Policy-C predicate for each query's own root.
An explicit quiescent diagnostic/reconstruction can support them outside active continuation, but B2
does not maintain a second archive or restore a full prefix after every diagnostic.
This bounded capability restriction must be tested and documented in the API.

## 8. Incremental occupancy and changed-history handling

The existing occupancy owner suffices; no new occupancy implementation is needed.
It retains `activeFrames_`, `nodeTiles_`, `tileNodes_`, cell evidence and monotone
append IDs. Its update API already supports removal by SQL Node owner.

For all changed historical nodes in D, load **persisted** LocalGrid via Memory,
make one `GridMapUpdate`, do `update -= nodeId` and
`update += GridFrame(nodeId, committed_pose, grid)`, then checked `commit()`.
This unions old and new dirty tiles, loads all surviving overlapping owners,
removes/replaces active ownership, clears affected tiles and refuses them in
ascending SQL Node order. Order matters: clamped hit/miss evidence is not safely
invertible by subtracting the last owner's log odds. Unchanged overlapping nodes
must participate in refusion. `GridMapUpdate` copies pending grids, so local
loader temporaries need not survive its preparation calls.

After historical replacements, append Q under its committed serialized pose and
next SQL ID. This preserves canonical replay order. Do not use `updateSingle`
for historical nodes: it forwards to append and rejects old IDs. Remove the old
backend helper's `local_grid_maker_` prerequisite for *maintenance*: historical
reconstruction has occupancy but no maker. Maintenance uses only persisted grids,
with no ground estimation, CAPE, raycasting or LocalGrid generation. This applies
even when navigation publication is disabled, as accepted A2/B1 reconstruct the
evidence owner independently of output enablement.

For each changed pose: update Memory pose/KD lookup, world AABB, committed Values,
occupancy transformed evidence and any exported grid's source revision. Keep
original anchors, local bounds, local payloads and appearance unchanged. Recompute
active correction using the newest committed active node, including when historical
optimization moves it. Do not only update the newly appended node.

Grid update failure can occur after its own internal mutation and has no rollback
promise. Post-W failure leaves DB R authoritative and the backend Failed.
Exceptional reconstruction is explicit; do not silently reconstruct and continue.
Canonical persisted-evidence replay remains the oracle, including negative cells,
overlaps, clamping, empty evidence and retained F3 transform semantics.

Dense `getMap()` is O(observed cells + bounding rectangle area). Avoid eagerly
exporting the entire map merely to maintain occupancy every B2 commit: invalidate
the cached `occupancy_msg_` on a successful new runtime revision and lazily build
the existing immutable NavigationGrid under lifecycle ownership on a read request.
Never return an old cached message labelled as current. A caller retaining an old
shared message may still read its explicitly old UUID/source revision. Export
allocation failure is visible and conservatively sticky; DB/ready revision facts
stay explicit. If implementation instead elects eager export, count its whole-map
cost and do not claim bounded per-node materialization. B3's delivery policy is
deliberately undecided; it must be able to request output for a specific ready R.

## 9. Correction and commit-before-exposure

After each successful B2 materialization choose the newest committed active
session node Qn, not an arbitrary historical optimized key:

```text
T_map_odomNew = double(X_Qn_committed_float) * inverse(A_Qn_original_double)
```

Use the exact Node float serialization and immutable same-domain original anchor.
No speculative ISAM estimate, registration seed or recursive accumulation of old
correction. Validate finiteness; install correction and ready revision together
under existing lifecycle/output locks. All accessors check healthy/current.

Retain simple scalar observability on the existing backend: DB committed revision
(known last value plus W outcome for Unknown), runtime-ready revision, correction
availability/revision, and sticky failure. During commit/materialization, readers
block; on failure ready stays at the prior R and correction becomes unavailable.
If W reports Committed through an exception, retrieve its recorded committed
revision for diagnostics without claiming runtime readiness. For Unknown do not
describe the old expected scalar as a freshly verified DB result.

```text
tentative work → W COMMIT → DB R authoritative
→ incremental materialization → runtime-ready R → readable correction/state
```

B2 does not invoke a publication-completion protocol or claim delivery of ROS
output. The current pipeline addFrame-then-read-correction pattern cannot certify
that the just-enqueued input completed. Reads expose only their actual ready R;
do not label them with the enqueued head's expected revision. B3 can later observe
the separate scalar states without adding DB revisions or a Snapshot type.

## 10. Admission and backpressure through existing owners

Use the existing single backend worker. One processing head plus **one** waiting
frozen submap is a provisional implementation capacity candidate. Count bytes as
well as objects; include head scratch and any producer-held frozen item in the memory accounting.
The one pipeline mapping producer may hold at most one not-yet-admitted frozen
item, and must stop consuming marginal frames while it waits. No general queue,
scheduler, promise-per-node completion store or new pool is needed.

Admission reports accepted, capacity refusal, invalid/order/generation refusal,
stopped or failed using a small existing-owner API result/error. Acceptance is
ownership transfer into the bounded FIFO, not persistence success. A nonblocking
try form transfers nothing on refusal; the blocking form waits interruptibly on
the same capacity condition. Do not retain the old void API's silent returns.
Only one registered producer is supported for B2; concurrent submitters cannot
manufacture an unbounded collection of caller-owned waiting payloads inside the
backend. At most one sequence is the head; later inputs cannot overtake retry.

Concrete provisional implementation limits make the proposal reviewable; their
numeric values require representative workload validation and may be revised
within this finite-admission design. Queue sizes 1/2, 16/64 MiB and 256 frames are
not fundamental SLAM constants. Boundedness is structural; exact capacities are
not acceptance facts or measured latency targets. Initial candidates: backend waiting
capacity 1, marginal capacity 2, core LiDAR waiting capacity 2 plus synchronizer
pending 1, images 16, IMU capacity `max(64, ceil(2 * configured_imu_rate_hz))`.
Require a finite positive rate and checked conversion. Add finite byte/element
guards to variable packets: proposed 16 MiB per input LiDAR cloud and 64 MiB per
frozen submap including grid/images/pyramid/original trajectory. Size overflow is
an explicit refusal before transfer. These candidates require workload validation,
not a claim about all deployed scans. Persisted map capacity is a separate issue.
Keep changes local to existing admission/backpressure ownership; no general
sensor-ingestion redesign is included.

| Producer/owner | Required bounded behavior |
|---|---|
| Backend FIFO | Full → visible try-refusal or interruptible capacity wait; worker notifies on dequeue/completion/failure/reset/finish. Head remains owned through final status. |
| Pipeline marginal producer | Reserve a slot before advancing/marginalizing the next frontend unit, or hold at most one completed MargiFrame. Wait on `marginal_cv_`; mapping pop wakes it. No unbounded batch swap. |
| SubmapFrameBuffer | Existing travel-distance threshold alone can retain unlimited stationary original samples. Pipeline uses `buffered_frame_count()` and existing `flush()` at a finite cap (proposed 256 marginal frames), plus checked payload accounting; no LocalGrid/ground algorithm change. Oversize evidence is reported, never silently truncated. |
| Core Synchronizer | Enforce LiDAR/IMU count and byte limits before updating timestamp watermarks or taking ownership; refusal must allow retry of the same input. Its pending scan is included in the bound. |
| Replay/offline caller | Pace on explicit capacity signals; do not block a single multiplexed sensor callback while the missing IMU needed by the pending LiDAR cannot be delivered. Reserve enough IMU lookahead or return WouldBlock and let the feeder service the required stream. |
| Live callbacks | Cannot guarantee sensor slowdown. Nonblocking refusal/drop policy is explicit, counted and logged/status-visible. An IMU/LiDAR continuity loss invalidates the active domain before further type-0 continuation; trigger the reset fence. Raw input acceptance is not a guarantee every sample becomes a frozen node. |
| Images / dual LiDAR | Images currently evict oldest silently and rate-sample by design; report those policies/counters and distinguish input sampling from frozen-work acceptance. Dual-lidar already counts overflow/coverage drops; feed continuity-loss status to the same fence when it invalidates frontend odometry. No ROS output redesign. |

Accepted **frozen mapping work** is never silently dropped in ordinary operation.
Provide bounded scalar completion accounting: last accepted/last completed source
sequence, current head status and the contiguous canceled tail range plus reason.
On reset/failure report unfinished work explicitly; do not store an ever-growing
list of per-node statuses. A caller must retain input if it wants to retry after
a capacity refusal; the C++ move happens only after admission checks succeed.

Backend failure notification must reach pipeline input/marginal waiters even
when no further sensor arrives. Use one narrow failure/stop notification on the
existing owner, installed by its pipeline owner; copy notification state then
invoke outside backend locks. It only sets pipeline flags and notifies existing
CVs, never calls join/reset/finish or writes SQL. Do not turn it into an event
framework. Shutdown directly requests backend admission stop before joining a
mapping producer that may be blocked in enqueue.

All waits test capacity **or** failure/stop/reset-generation change under their
queue mutex. Never hold `pose_graph_mutex_` across a blocking admission operation;
owner replacement must wait for the existing mapping thread to acknowledge its
use has ended. A separate stop request can wake it without destroying the owner.
No worker joins while holding a lock needed by that worker. This addresses the
otherwise concrete shutdown/reset lock inversion in current submit ownership.

## 11. Worker, retry and failure boundaries

Adapt `workerLoop` to dispatch the attached-continuation path one head at a time;
do not batch through legacy `copyPendingFrames/buildOdometryGraph` or persist a
query for the benefit of candidate retrieval. Keep failure, admission-stop and
normal finish-drain distinguishable: a normal checked backend finish fences new
admission and completes already accepted frozen heads if healthy; failure/reset
must not drain later heads. This is backend persistence completion only, not B3
frontend-tail/ROS publication completion.

For retryable local preparation failure, retain the same head with a visible
retryable status and wait for explicit retry of that sequence, or explicit session
abort. Do not busy-loop, silently skip or process later heads. Input validation
before acceptance returns the input to the producer. Candidate/registration
**rejection** is ordinary no-loop progress, not a retained failed head. If finish
encounters a retryable head awaiting action, report incomplete work, fence and
cancel the unfinished range explicitly instead of waiting forever for a caller
that is itself waiting in finish.

| Boundary | DB | Backend/action |
|---|---|---|
| Invalid input before transfer | R unchanged | Visible refusal; producer owns input; no persistent ID consumed |
| No candidates/BBS or GICP rejects | R unchanged so far | Healthy; complete no-loop processing and commit base-only node |
| Local query/factor/Values preparation exception, all owners proven unchanged | R unchanged | Retryable head; later work held; standard or non-standard cause retained locally |
| Unexpected committed index/archive mutation failure during preparation | R unchanged | Sticky, even before ISAM; “pre-solver” alone does not prove retryability |
| First ISAM entry throws, including before it returns | R unchanged | Sticky Failed; no Qn+1 |
| W `NotCommitted` after ISAM | R authoritative | Sticky Failed despite rollback |
| W `Unknown` | Must resolve by reopen | Sticky Failed; no ID/factor issuance or blind retry |
| W `Committed`, then index/visual/grid/correction failure | R+1 authoritative | Sticky Failed, runtime-ready remains R |
| Checked close failure | Last known W outcome retained | Ownership/admission retained until complete close; finish reports failure |

Mutation flags are local to the head and set before calls, as accepted B1.1.
Use existing `exception_ptr` and MapError/CommitOutcome. Existing W sometimes
marks its own owner Failed during packing before BEGIN; B2 does not weaken W
or claim such a call is retryable just because SQL is unchanged.

### Worker exception boundary decision

Concrete paths: containers/Values/scene preparation can throw `bad_alloc`,
invalid_argument, runtime_error; OpenCV/Faiss/GTSAM use standard exceptions;
W converts many SQL/packing exceptions to MapError but its final rethrow can
preserve a non-standard exception. `future::get` also rethrows its stored cause.
Most decisively, the existing `writableStorageTestPoint("isam-update-entry")`
can throw the integer used by the independently accepted B1/teardown probes.
Executing that **same B2 ISAM call inside the current std::exception-only worker**
would allow the retained non-standard fault to terminate the process if it were
not contained. No evidence claims ordinary GTSAM routinely throws integers.

Design a local catch-all at the B2 head/worker entry, preserving the boundary
flag and stored cause; pre-mutation proven-clean faults become visible retryable
head results, everything uncertain calls existing recordFailure and wakes all
waiters. Add a final worker guard for exceptions outside the classified head
body, conservatively sticky. Logging must not be the only effect. Do not broadly
rewrite unrelated algorithm catches, add terminate handlers, or alter deliberate
SQLite complete-close fail-stop. This is a proposed future local fix; **none is
implemented in this design pass**.

Explicit `finish()` joins backend/visual work before Memory/database close,
reports original retained failure and remains repeatably checked. Destructor
cleanup retains the accepted standard plus catch-all behavior and W close order.

## 12. Reset, restart, pipeline handoff and recovery

Reset linearizes at an admission fence under the input mutex: invalidate the
generation immediately, wake blocked producers, reject new/queued old-domain
heads and make correction unavailable. A head that already crossed its processing
admission point may complete as old-domain work; reset waits for it before closing
the owner. It cannot become an edge in the new domain. Explicitly report canceled
accepted tail sequences; never replay them into a new session. Do not clear a
queue before accounting for its contents. Reset racing before head admission
cancels that head; racing after COMMIT cannot roll it back.
The reset/admission-stop flag must also participate in correction availability
and the head's final readiness check, even while reset is waiting for lifecycle
ownership. A completing old head cannot re-enable correction after that fence.

Retain B1's conservative reset policy: checked finish/close and destroy this
backend before reopening the same DB. On successful reopen all old chains are
historical, correction is unavailable, fresh B1 is required. Global IDs continue;
new odometry topology starts another attached root. Do not use current new-map
pipeline reset filename behavior as resumed-domain continuation.

The current pipeline resume gate must not be opened by constructor alone.
Bounded integration needs an explicit handoff of B1 success to the **same live
producer**: pipeline-owned backend, retained Q0 source sequence/generation and
the following frozen frames. Historical/awaiting-attachment mode admits no B2
work. The caller continues to provide B1's selected target and metric seed;
no automatic localization or new attachment algorithm is specified. Before B1
success, freeze/pause that producer with the same queue guards; do not build a
hidden backlog. A mere externally supplied correction cannot unlock admission.
Implement transport through existing pipeline/backend ownership, without a
second backend or adoption of a simultaneously open writer. The end-to-end
handoff test is required before claiming pipeline resumed mapping is available.

Process restart always does A2 reconstruction (or explicit W recovery followed
by reconstruction where needed), with no active correction. Do not infer a live
domain from the last chain/root, restore a generation from SQL IDs, or reuse the
previous process's final anchor for a new frontend type-0.

Full authoritative rebuild is allowed on explicit startup/recovery, and the
single canonical visual-prefix handoff in section 7. It is also the test oracle.
It is never the normal Q1/Q2/Q3 materialization path and never an automatic
“incremental failed → rebuild → continue” branch. Safe closure/reconstruction
does not make a failed owner's state healthy again.

## 13. Cost, bounded characterization and throughput acceptance

Let N be history nodes, D changed poses, G_i stored LocalGrid evidence size,
V eligible descriptors, C candidates, T dirty occupancy tiles and U owners
overlapping those tiles (distinct from temporal K=3). Incremental materialization
avoids historical geometry and scene replay in the usual unchanged-history case, but **the complete current
W-based step is not independent of history size**.

| Work | Expected scaling / existing caveat |
|---|---|
| Pose reconciliation | O(N) serialization/comparison required by accepted B1 semantics; full estimate extraction/objective evaluation also traverses graph/Values. Cannot replace by affected-key list without proof. |
| Pose/spatial maintenance | Typically O((D+1) log N), using scalar pose upsert and R-tree replacement; tree degeneracy/rebalancing and vector capacity growth can cost more. No cloud rebuild. |
| Visual maintenance | New eligible descriptors/fingerprint plus insertion; one Flat→IVF transition trains bounded sample and reindexes O(V) live slots. Not repeated per node. |
| Visual query | Flat distance search scales with eligible descriptors; current search also clears per-slot/per-node scratch even with IVF. No promise of O(1) retrieval. |
| Occupancy append | O(G_Q) evidence transform/fusion. |
| Occupancy pose changes | Tile discovery and clearing O(1024T), candidate sorting, and full LocalGrid traversal of U overlapping owners; dense revisits/large loops can touch all history. No second occupancy map. |
| Dense grid export | O(cells + rectangle area), only on demand in this proposal; frequent readers/publication can still make it a per-node cost. |
| Candidate/registration | Existing C-dependent preparation, target pyramids/clouds, BBS and per-target GICP tree/registration. No spatial candidate cap; worst-case C=N, inherited F5 remains. |
| W before BEGIN | Packs Q/D; **reads and transforms all historical LocalGrids** to check coordinate and aggregate extent representability, O(sum G_i), even D=0. |
| W BEGIN→COMMIT | Rereads historical Nodes/original odometry and Links; validates old and proposed topology, then writes Q, D and one R increment. Thus transaction duration itself contains history-dependent validation, not just O(payload + D) SQL writes. |

**Interim independent-review conclusion incorporated: no prerequisite W
optimization is currently justified.** Retain the accepted W/W.1 correctness
scans for initial bounded B2 implementation after design acceptance. Section 6's
narrow admissibility amendment changes the predicate, not those scans, their
ownership, W.1 guards or transaction architecture. Do not add validation caches,
a bypass assertion, a second pose store or storage architecture in this phase.

The acceptance target is **no ordinary full-runtime rebuild plus sustainable
representative arrival workload with bounded backlog**, not strict O(1) service
time or map-size-independent throughput. Mandatory O(history) W validation and
all-pose reconciliation remain explicit costs, not prerequisite design blockers.
Bounded queues alone do not demonstrate sustainable service: measure offered,
admitted and completed rates, head-service latency and queue high-water/backlog
trends; account for refusals, continuity-loss fences and time blocked. Do not hide
an overloaded representative workload by pacing away its offered load or silently
dropping accepted work. Live overload behavior remains governed by section 10.

Actual operational throughput is still unverified and must pass workload
acceptance. If representative measurements fail, report the bottleneck and seek
a bounded follow-up decision; do not preemptively optimize W, weaken correctness
checks or invent a convenient latency target. Broad F5 work remains excluded.

### Measured existing-owner evidence

An isolated CPU probe compiles against current owners and the configured core
library; no production modifications or GPU calls. Three runs per size, 512
LocalGrid points per node, overlapping translated grids, one moved historical
node plus one append; 96 seeded-random descriptors per node, settled training.
Times below are medians in milliseconds. The probe asserts exact occupancy
export/evidence against fresh ordered replay and exact visual identity metadata.

| Existing nodes | Occupancy move | Append | Occupancy replay N+1 | Visual insert | Visual replay N+1 | Dense export |
|---|---:|---:|---:|---:|---:|---:|
| 8 | 0.112 | 0.022 | 0.246 | 0.0069 | 0.068 | 0.020 |
| 100 | 0.121 | 0.024 | 2.721 | 0.071 | 1554.73 | 0.319 |
| 512 | 0.075 | 0.024 | 14.589 | 0.076 | 1501.84 | 3.069 |

Moved-node refusion loaded 4/3/2 overlapping *other* grids, respectively, because
fixture location relative to tile boundaries differs. Those numbers explain
why move time need not grow monotonically. Two scalar pose/spatial upserts cost
0.0013/0.0030/0.0117 ms. These timings demonstrate viable existing owner APIs,
not integrated B2 latency. The index timing case does not substitute for changed-
membership tests. Visual probe verifies metadata, not query rank equivalence.
O3 probe/header code uses the existing configured library; allocation/cache state,
synthetic input and shared host limit generalization. No RSS or end-to-end claim.

Retained B1 full materialization timings are 0.604 ms at 7 nodes and 1568.36 ms
at 129 nodes (different fixture, one run, includes archive training). The W design
SQL-only 1.74 MB payload probe measured 3.80 ms median (3.31–6.74 ms); it excluded
production W validation and is **not** B2 transaction timing. Do not compute an
overall speedup by comparing these unlike fixtures.

During eventual implementation characterize small/~100/512-node no-loop and
history-moving cases under identical payload/configuration/build: split W packing,
occupancy preflight, BEGIN→COMMIT, reconciliation, index, visual insertion/training,
occupancy maintenance, optional export, candidate/preparation and BBS/GICP.
Measure complete A2/B1 reconstruction separately on the same DB revision, record
median/range and cold/warm conditions, changed counts, tile overlaps, descriptor
counts and candidate counts. Also characterize end-to-end head service and
backlog under sustainable representative arrival workloads: W pre-BEGIN work,
BEGIN→COMMIT, all-pose reconciliation, incremental pose/spatial maintenance,
visual insertion/training, occupancy maintenance and candidate/registration cost.
Count full rebuild calls: zero per ordinary B2 commit. Record cold/warm and loop/
no-loop behavior, Flat→IVF stalls and bounded-queue recovery after finite bursts.
These remain required acceptance measurements, not results claimed here.

## 14. Required production scope after design approval

| File/owner | Necessary bounded change |
|---|---|
| `sapphire/src/mapping/graph/pose_graph.hpp/.cpp` | Live-domain binding, bounded admission/results, single-head Route-S dispatch, exact pose delta, materialization/readiness/correction, local worker catch-all, finish/reset/wakeup semantics; preserve B1 and legacy new-map boundaries. |
| `sapphire/src/mapping/storage/memory.hpp` | DB-free committed pose/spatial delta and transient spatial-query forwarding; use scalar pose upserts, preserve cache identity. |
| `sapphire/src/mapping/storage/retrieval_index.hpp` | Spatial query with explicit transient local bounds; apply the shared Policy-C predicate after retrieval and before preparation. Existing trees need no algorithm change. |
| `sapphire/src/mapping/registration/loop_closure.hpp/.cpp` | Transient Q cloud/bounds preparation; delegate temporal eligibility to the shared helper and defensively filter merged candidates. Preserve deterministic ranking, BBS/GICP/scoring/noise. |
| `sapphire/src/mapping/visual/visual_loop.hpp/.cpp` | Policy-C membership before NNDR for transient and historical queries, one-time B1 handoff, monotone prefix advancement and active backward-query guard; retain DescriptorArchive algorithm/settings. |
| `sapphire/src/mapping/loop_eligibility.hpp` (proposed small pure header) | One dependency-light semantic predicate over validated query/target/root facts; no manager, graph owner, mutable cache or persistent state. Existing registration-only helper delegates to it. |
| `sapphire/src/mapping/storage/map_database.cpp` (header only if narrow helper plumbing needs it) | Replace ordinary-loop preflight and shared A2/W graph validation with Policy C, derive/revalidate planned root, retain scans and every other W/A2 invariant; no locking, inode, atomic transaction, descriptor-admission or CommitOutcome changes. |
| `sapphire/src/pipeline.hpp/.cpp` | Explicit already-attached producer handoff, finite sensor/marginal queues and builder cap, refusal/failure signaling, reset-generation fencing and safe blocked-producer lifetime. No output redesign. |
| `sapphire/src/parameters.h`, `parameters.cpp` | Only validated finite admission/payload limits if configurable; no algorithm or persisted compatibility-field change. |
| `sapphire_ros2` input callbacks/status | Only if needed to expose ignored image admission/continuity-loss counters to the existing pipeline fence; no output queue/publication work. |
| Focused tests and `sapphire/CMakeLists.txt` | Continuation, owner-oracle, fault, queue and process tests. |

No required change to OccupancyGrid, DescriptorArchive, SubmapFrameBuffer or
third-party code was identified for the functional incremental path. Existing
buffer APIs allow the finite frame cap. The MapDatabase scope is only section 6's
narrow admissibility amendment, not W redesign. No persistent field, table, Link
type or encoding change is required; retain user_version1 with the explicit older-
software limitation proposed there. Section 13 retains W correctness scans with
no prerequisite optimization. Existing source directories are dirty/untracked;
implementation must isolate its changes from the accepted baseline. **This is
future implementation scope, not authorization to edit production now.**

## 15. Acceptance scenarios and independent oracles

Tests described here are **required future tests**, not implemented/passing B2
claims. Preserve all A1/A2/W/B1/teardown correctness/failure regressions. Update
only explicitly superseded cross-chain global-gap expectations to the reviewed
Policy-C contract; do not weaken unrelated assertions to accommodate continuation.
Reuse existing weak fault seam and child-process testing patterns; no exhaustive instruction-level injection.

| Scenario | Assertions |
|---|---|
| History → B1 Q0 → Q1/Q2/Q3 | IDs contiguous; one revision per node; Q0 one attachment/no predecessor odom; each successor one type-0 from previous active node; sole Node-1 prior; exact original-anchor bytes and root expectations. |
| Base-only sequence | Zero ordinary type-1s, complete payload/tag/pose commits; no completion marker or extra revision. Real registration rejection is healthy progress. |
| Ordinary accepted loop | Shared section 5 Policy C; existing deterministic first-success ranking and one loop per query; independent asymmetric T_H_Q inverse, actual factor endpoints/noise/residual and stored Link. |
| Immediate post-B1 boundary | Old97/98/99, root100, transient101: exclude100; retain99/98 if retrieved; geometrically verified ordinary101→99 passes W and independent A2 reopen; no cross-session type-0. Distinguish B1 root100 attachment from ordinary101 loop. |
| Same-chain K and prefix boundary | Same-chain distances1/2/3 excluded, distance4 eligible; query102/103 still exclude root100, query104 admits it; older-chain99/98 stay eligible. Check zero-/one-based conversions, near-zero unsigned underflow and helper/prefix equality. |
| Multiple historical chains | Derive all roots independently; active query excludes only its own recent chain; historical arbitrary queries use their own root interval. No temporal continuity inferred from type-1 links. |
| Endogenous topology | Hold type-0 facts/query/proposals fixed; insert verified or deliberately wrong historical type-1 shortcut and assert unchanged hard C eligibility. No full-graph-hop validity test, including after an edge shortens its own endpoint distance. These are semantic-policy fixtures, not accuracy claims. |
| Consecutive corroboration | root100→H40, ordinary101→H41 and102→H42 remain eligible when retrieved and geometrically valid; one loop per query, no automatic neighborhood suppression after preceding acceptance. |
| Same-format software compatibility | Existing valid schema1 histories remain accepted; new nearby-ID cross-chain ordinary edges pass amended W/A2 but a retained older reader rejects before exposure without reinterpreting bytes. Same-chain invalid/forward/duplicate/dangling/root-invalid edges remain rejected. |
| History moves | Include a changed key outside optimizer affected list and float-serialized equality/no-change case; compare every D owner, old-only/new-only spatial region membership and newest-session correction. |
| Visual continuation | B1 full archive → one C-prefix handoff → monotone zero/one-node advancement; same-chain recent descriptors excluded before NNDR, previous-chain close IDs included; adversarial excluded nearest descriptor cannot contaminate NNDR; identical fingerprints/no retry duplicates; below/at/across8192 with identical training history/mode and ties; backward active-query refusal, arbitrary unattached A2 queries use C. |
| Paired continuous/split replay | Same physical trajectory/IDs with and without a root split: record C's bounded extra old-chain targets, not a false restart-invariance claim; evaluate load, false accepts, first-success target, trajectory consistency and backlog/latency at equal budgets. |
| Occupancy | Overlapping hits/misses, saturation/clamping, multiple moved old owners, negative tiles, vacated tiles, new-node overlap, empty grids and disabled publication. Compare active owners, exported origin/shape/cells and evidence fields to canonical persisted replay. No regeneration calls. |
| Readiness fence | Barrier after COMMIT and after each owner update; readers never observe partial R, retained old message remains explicitly old, correction and ready scalar advance together only after success. No publication-completion claim. |
| Cross-process continuation/restart | Writer history → destroy → process B A2/B1/Q1… → close → independent C A2 (CUDA hidden). Graph/poses/spatial/visual/occupancy reconstruct, correction unavailable, addFrame denied until fresh B1. No raw sensor replay. |
| Reset/rebind | Reset before/after head admission and COMMIT, queued/waiting work accounted, no new-domain type-0; next B1 appends a root using global next ID. End-to-end pipeline handoff to the same live producer. |

Core oracle at selected committed revisions:

```text
incremental runtime(R) ≈ independent A2 reconstruction(DB at R)
```

Compare exact committed float poses, semantic spatial candidate membership,
visual membership and equal-prefix/equal-mode query behavior, occupancy exported
cells/evidence; not private container/tree/slot layout. A2 deliberately has no
active correction: compare B2 correction to independently decoded committed
newest pose times inverse of its original anchor, while separately asserting
A2 correction unavailable. Do not enable a fictitious session on the reference.
The independent visual oracle requires **the same committed revision, the same
transient query, the same Policy-C eligible target population, the same Flat/IVF
training history/mode and deterministic tie rules**. Current unamended A2's
global-gap population is not a valid reference. B2 preloads the next query's
prefix, while an A2 opening may prepare a different persisted query; normalize
through a test-only transient query on the independently reconstructed owner,
without inserting a Node or creating a fictitious active session/correction.
Canonical ascending insertion alone is not a substitute for matching training
history: settle the same threshold transition and compare supported query output,
not private index layout. A2 graph/root reconstruction remains independent of
any active producer and must expose correction as unavailable.

W-F1 forbids opening a second MapDatabase in the writer process. For selected
revision oracles, stop/close the writer before an independent reader child, or
use separate fixture runs ending at each selected R. Do not copy only a live WAL
main file or bypass owner admission to get an oracle. For within-owner occupancy
unit tests a fresh in-memory evidence owner is a test oracle only.

Failure matrix for a representative Qn: local candidate/preparation errors and
factor/Values allocations before mutation (same head retry, identical solver/DB/
committed owners); first ISAM-entry failure including integer/bad_alloc; W
NotCommitted after solver change; COMMIT error before/after actual SQL commit
(Unknown resolves both old/new); known Committed plus each materializer failure.
For every sticky case assert Qn+1 never enters processing/ISAM/W, waiting producer
wakes with failure, reads/correction fence, explicit finish reports cause/outcome,
safe ownership closes, independent recovery/A2 succeeds at the authoritative
revision. Repeated finish/destructor tests retain accepted no-accidental-abort
and deliberate unclosable-SQLite fail-stop distinctions.

Backpressure tests use barriers/CVs and recorded sequence/accounting, not sleeps:
hold head before solver, fill the configured finite queue (one slot for the
provisional candidate), block/refuse next input; release and
verify ordered exactly-once commits. Fail head while producer waits; stop/finish
while it waits; reset before/after dequeue; retryable head plus finish; block
mapping and fill marginal/IMU/LiDAR limits; replay needs IMU lookahead while LiDAR
full; live overload invalidates generation; stationary builder reaches cap;
oversize input refused before transfer. Verify all wakes, bounded high-water
marks, explicit cancellation ranges, zero silent accepted-work loss and no
join/lock deadlock. Run these on actual pipeline/backend ownership as well as
unit queues. Publication backlog remains outside the B2 memory guarantee.

### Restart limitation and real-data falsification

Policy C is **not restart invariant**. With an otherwise physically continuous
trajectory, unchanged IDs and a split at root r, it may re-admit previous-chain
targets in `[q-K, r-1]` that the unsplit chain excluded (using one consistent ID
domain and clipping to existing targets). The extra set has at most K members
under current contiguous append semantics and disappears when q≥r+K; for the
ordinary Q1/q=r+1 case it is at most K−1. With root100, q101 admits99/98; by q103
the split/unsplit eligibility difference is gone. Unrelated older targets outside
that narrow window do not change. A subsequent fresh restart creates a new case.

This difference is intentional: persisted geometry/root attachment does not
prove temporal continuity across independent odometry domains. It can still be
harmful: overlapping old scans can increase registration work or ambiguous
first-success matches. Neither C nor K=3 is claimed empirically superior.

Require paired continuous-vs-split real-data replay plus genuine independent-
session revisits at equal retrieval/registration attempt/time budgets, unchanged
BBS/GICP thresholds and comparable hardware. Measure useful loop recall,
registration workload, ambiguity around K across submap/keyframe densities,
false accepted loops, first-success target selection, trajectory consistency,
head service and backlog/latency. Native K remains fixed at 3. Future controlled
sensitivity experiments may study other temporal policies, but changing the
native persisted admissibility contract requires a separate compatibility
decision. Such experiments do not make native K configurable or permit writer/
reader disagreement. No migration/version machinery is added; `user_version == 1`
remains unchanged.
Reconsider C if restored cross-chain near-boundary proposals provide no useful
recall, reproducibly increase false accepted loops, degrade trajectory consistency,
or prevent required continuation rate at equal budgets. Report which condition
failed; do not protect the proposal by adding new hard graph thresholds after
seeing results. The research's synthetic fixtures test semantics, not these
empirical acceptance claims.

## 16. Limitations, exclusions and review decision

Retained limits: schema-1 float precision and A2 numerical support envelope;
W's Linux/cooperative ownership and WAL/NORMAL durability scope; F3/F4 occupancy/
ground limitations; no real-data continuation validation; map-resident graph,
spatial, visual and occupancy memory grows with map size. Bounded work admission
is not a bounded total-map RSS claim. Registration candidate count, exact pose
scan and W validation costs remain history-dependent. Dense output and the
existing ROS output queue are outside the B2 admission guarantee; no end-to-end
ROS memory/publication claim is made. Admission limits are proposed defaults
requiring representative input validation. Historical backward visual queries
are restricted while active continuation owns the archive.

No B3 publication/drain design, ROS output redesign, global localization,
attachment algorithm redesign, registration/Faiss tuning, multiple loops per
query, optimizer persistence/checkpoint, rollback, Snapshot, session manager,
FailureManager, RecoveryManager, scheduler, storage abstraction, migrations or
unrelated F3/F4/F5 work. No new tables/session columns/completion markers/Link
type 2. Section 6 retains `user_version == 1` under the accepted design: this is a software
admissibility extension with explicit fail-closed older-reader limitations, not
a different persistent byte/factor interpretation. No persistent-format change
has become unavoidable. Changes to the fixed native admissibility contract require
a separate compatibility decision as section 15 specifies.

**B2 design accepted with required clarifications — ready for bounded
implementation.** The fixed native K=3 clarification is incorporated in section 5.
Policy C is the accepted baseline; full-graph soft ranking
remains experimental/non-baseline, and real-data evaluation can falsify C.
The review's interim throughput conclusion is incorporated: no prerequisite W
optimization; initial implementation retains correctness scans and must later
prove no ordinary full-runtime rebuild plus sustainable representative arrivals
with bounded backlog. Numeric backpressure limits remain provisional workload-
validated implementation candidates.

The incremental owner evidence and policy fixtures are review evidence only,
not integrated B2 validation. Accepted retryable/sticky failure semantics remain
unchanged, including the concrete local B2 worker catch-all proposal; no global
worker exception architecture is introduced. **This documentation-only pass does
not authorize implementation.** Production remains
unchanged; B2/B3 implementation has not begun in this pass.
