# Map modeling principles

An engineering compass for distinguishing observations, world identity, geometry,
retrieval structures and geometric measurements. This records established
invariants, externally supported principles and explicitly conditional hypotheses;
it prescribes no future architecture. **Path 1 — B2 unchanged.**

## Distinct contracts — externally supported principles

**What something is, where it is currently estimated to be, how it is retrieved,
and how it is physically stored are different contracts.**

| Concept | Contract to preserve |
|---|---|
| Observation identity | One retained acquisition/submap and its evidence provenance; unchanged by pose optimization or reindexing. |
| World / Place identity | Declared continuity of a spatial referent across observations; any future contract must allow correction. |
| Estimated geometry | Revision-dependent pose, shape or extent, distinct from the identity being estimated. |
| Spatial index identity | Cell address or index entry within a frame, scheme and resolution; not canonical world identity. |
| Vector-index identity | Index-local slot/label mapped to application evidence; not the observation or world entity itself. |
| Pose-graph measurement identity | A particular accepted measurement with observation endpoints, transform, uncertainty and provenance; distinct from solver-variable keys. |
| Persistent storage identity | Logical record key in a namespace; distinct from physical row/page/offset or storage version. |
| Derived grouping / Region identity | Recomputable grouping label; persistent semantic Region identity would require a separately justified continuity contract. |

Semantic separation does not require separate classes, tables, services or graphs.
One observation ID can also serve as its logical database key.

**Persistent identity should not encode mutable spatial location, partition,
hierarchy or index layout unless that locator itself is the intended referent.**
GERS separates maintained identity from geometry, bbox and source records; H3/S2
identify spatial addresses; ANN systems separate application IDs from internal
slots; storage engines separate logical keys from physical layout. These lessons
do not mandate UUIDs: opaque monotonic IDs remain valid where appropriate in
Sapphire's single-writer map, with explicit namespace and non-reuse contracts.
Persistent identity is maintained under a policy, not guaranteed perfect or
immutable forever. Industrial entity resolution does not solve Sapphire's data
association. Evidence: [research §§1, 6–10, 19](../../audit/2026-09-28-spatial-identity-research/REPORT.md),
[comparison matrices and their sources](../../audit/2026-09-28-spatial-identity-research/MATRICES.md).

## Relations and heuristics — durable modeling rules

**The existence of a heuristic does not prove that a missing persistent entity
is its cause.** K is an observation-count proxy for recent temporal proximity and
likely redundancy—not physical distance, Place identity, measurement independence,
information gain, geometric overlap or semantic locality. Place is no proven
replacement for K and does not eliminate temporal guards.

| Relation | Meaning |
|---|---|
| T(q,h) | Temporal adjacency / same acquisition-chain relationship. |
| I(q,h) | Evidence of a common spatial referent / identity. |
| C(q,h) | Dependence or redundancy of measurements/evidence. |
| G(q,h) | Independently verified geometric registration. |

**These relations are not interchangeable.** Same-Place revisits can provide
nonredundant measurements; different Places can share correlated evidence.
Temporal adjacency does not establish identity. Registration can verify partial
overlap between distinct Places. Membership must never automatically create a
pose-graph factor.

Prefer state variables and relationships with explicit domain meaning. Heuristics
and thresholds are acceptable when their surrogate meaning and scope are explicit;
thresholds, partitions and groupings must not silently become persistent world
semantics. **Parameters are not inherently a defect.** Distinguish a parameter
tuning a meaningful relation from one standing in for a relationship the model
has never represented. Neither implies a parameter-free model is achievable or
desirable. Evidence: [research §§2, 20–21](../../audit/2026-09-28-spatial-identity-research/REPORT.md).

## Authority and access — established Sapphire invariants

**Authority, representation validity, algorithmic support, and delivery are
separate claims.**

```text
authoritative ≠ representation-valid ≠ supported by every runtime algorithm
              ≠ successfully externally delivered
```

A DB-authoritative map can exceed A2's characterized reproduction envelope.
Under the [amended A2 design](A2_DESIGN.md#numerical-support-amendment--accepted-with-documented-limitations-closed),
finite similarity-only exceedance is diagnostic; exact representation, graph/model,
typed-key, numerical-validity, materialization and identity requirements still
govern successful opening. The implementation has passed
[fresh independent review](../../audit/2026-09-29-a2-contract-amendment-independent-implementation-review/README.md)
and is **accepted with documented limitations / closed**; final disposition is
recorded in CURRENT_STATE.
Revision R's committed X^R is its authoritative presentation layout; fresh X* is
disposable runtime optimization state. Products cannot use X* while attributed to
R. Valid tentative B1/B2 movement may become a later revision only through all-key
reconciliation, W and coherent committed-owner materialization. No promise of
arbitrary-graph or indefinite continuation, safe arbitrary divergence, universal
B1/B2 success, physical loop correctness, controller-safe revision jumps or
universal beyond-envelope B2 new-loop support follows. These are retained
limitations, not unfinished work. Mandatory F1 writer settlement is not adopted;
the full-graph probe remains research/diagnostic evidence only.

A valid ready map can exceed an output capability envelope; runtime-ready state
does not imply subscriber delivery. These distinctions describe what each claim
proves, without introducing entities, layers or managers. Underlying ownership,
atomicity, readiness, attribution and checked-close invariants survive algorithm
improvements; their exact current implementations need not be eternal. Evidence:
[independent algorithm/system audit](../../audit/2026-09-29-algorithm-system-invariant-audit/README.md)
and [bounded continuation experiment](../../audit/2026-09-29-a2-beyond-envelope-continuation-experiment/README.md).

**Indexes are derived access structures, not world truth.** Faiss slots, KD-tree
entries and AABB/R-tree organization follow this rule; so would future spatial
partitions, representative caches and search communities. Rebuilding, retraining,
compacting or reorganizing them must not silently redefine represented entities.
Preserve **DB authoritative; runtime derived/disposable**, within the accepted
persistence contracts. No Faiss or SQLite replacement follows.

Retrieval, association and identity hypotheses supply candidate evidence only.
Currently, a BBS/GICP-verified observation pair supplies the geometric measurement
under the existing acceptance and uncertainty contracts. Place matches, semantic
labels, ANN neighbors, spatial adjacency, Region membership and graph communities
cannot automatically create factors. Conversely, an accepted factor need not
force common Place membership. Measurement endpoints and provenance must survive
grouping changes. BBS/GICP is the current verified implementation; future changes
require independent review and geometric verification, not permanent algorithm
lock-in. Established authority: [AGENTS](../AGENTS.md) and
[accepted A2 contracts](A2_DESIGN.md).

## Conditional future hypothesis — Place, not a commitment

A persistent Place could help consumers needing durable cross-session spatial
references, identity correction, annotations, change tracking or retrieval
consolidation. Its construction algorithm is unsettled; generic local support may
have no natural invariant boundary and may merely introduce new thresholds.
Observation-only diversity/representative selection may solve retrieval scaling
more cheaply.

**O** (observation-only) remains valid for B2. **OP** (Observation + persistent
Place) is a bounded future experiment only if continuity has a demonstrated
consumer. **OPR** (Observation + Place + Region hierarchy) remains deferred.
None is a new roadmap milestone.

If persistent membership is ever justified, authoritative records would comprise
observations, committed geometric measurements, committed poses and explicitly
accepted identity/membership decisions. Spatial/ANN indexes, representative
selection, search graphs and recomputable Regions/communities remain derived.
An accepted association is a recorded decision, not infallible ground truth;
retain enough evidence and version information to explain and revise it. This
defines no schema. Evidence: [research §§10, 13–16, 23–26](../../audit/2026-09-28-spatial-identity-research/REPORT.md).

## Do not infer

- Same H3/S2/grid cell ⇒ same Place; same descriptor cluster ⇒ same world entity.
- Same graph community ⇒ persistent Region; same Place ⇒ independent measurement.
- Successful registration ⇒ necessarily same Place; spatial adjacency ⇒ temporal adjacency.
- Pose-graph connectivity ⇒ world identity; ANN slot ⇒ observation identity.
- Observation ID ⇒ automatically Place ID. A stable name or integer alone does not create a stable semantic contract.

## B2 and scope

This document **does not modify accepted B2**. [Policy C](B2_DESIGN.md) remains the
accepted bounded observation-level rule; K=3 is not a world-modeling principle.
The accepted A1→B3 lifecycle remains closed. Observation identity, geometry/index
separation, independent geometric verification and DB authority
already permit later, separately reviewed membership facts. Add no Place seam
for future elegance. Current loop-policy/cardinality and scene-construction
persistence couplings remain accepted native contracts; their F2/F3 compatibility
cleanup is conditional on a justified algorithm change, as recorded in
[CURRENT_STATE](CURRENT_STATE.md). This document creates no implementation phase.

**Global invariant, local implementation.** Nothing here justifies PlaceManager,
RegionManager, WorldModelManager, a graph or entity-resolution service, another
database, event bus, distributed infrastructure, semantic scene hierarchy or
migration/version framework. Any concrete future need requires separate
justification within existing ownership boundaries first.
