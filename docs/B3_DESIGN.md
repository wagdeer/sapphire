# B3 — revision-consistent publication and drain

Design only, 2026-09-28. **No B3 implementation is authorized by this document.**

Readiness: **B3 design ready for independent review**.
The completed transport probe found no contradiction with the minimal adapter
contract. B3 bounds Sapphire-owned publication state, not the duration or internal
memory of third-party transport calls. Graceful drain assumes an in-flight call
eventually returns; it has no hard wall-clock deadline. Transport uncertainty is
a platform limitation, not a map-correctness or design-review gate (section 22).

## 1. Contract and scope

After B2 makes revision R runtime-ready, B3 may externalize immutable, explicitly
attributed correction/pose and grid state from R. A successful local publication
means the configured output actions completed in this process; it never means a
subscriber received or consumed them. State outputs may skip intermediate ready
revisions. Normal checked shutdown fences admission, settles accepted healthy
mapping work, selects the final ready revision, performs the required local
output actions for that revision while publishers and runtime remain alive,
then checked-closes storage and destroys dependencies. Processing, publication,
cancellation and close failures remain separately visible. Persistent map facts
never change merely because output or drain advances.

B2 still owns admission, loop decisions, solver, W, incremental materialization,
ready revision and correction. B3 owns the subsequent output contract and the
ordering of existing owners at shutdown. No persistence/solver operation moves
into a ROS callback. B3 finishes this lifecycle; no invented B4 is needed.

The authority boundary is:

```text
map persistence / algorithmic truth
    → runtime-ready state
    → attributed local publication contract
    → replaceable communication adapter (ROS / Zenoh / DDS / future backend)
```

**Transport may affect delivery and graceful-shutdown completion, but must not
determine map correctness or runtime readiness.** Only existing map owners decide
DB revision, solver state, loop validity, committed poses, occupancy and backend
health. C/Y record those facts; Pc/Pg record Sapphire-observed publication actions.
Transport choice is not map identity. Adapter replacement requires adapter and
deployment validation, not changes to these frontiers, B2/W/A2, map lifecycle,
solver or schema. This is a behavior boundary on existing owners, not a new
adapter manager, abstract class hierarchy or publication service.

Foundations: [A1](PHASE1_DESIGN.md), [A2](A2_DESIGN.md),
[W](../../audit/2026-09-27-w-design/W_DESIGN.md),
[B1](../../audit/2026-09-27-b1-implementation/PLAN.md),
[B1.1 acceptance](../../audit/2026-09-27-b11-acceptance/README.md),
[B2](B2_DESIGN.md), [failure semantics](BACKEND_FAILURE_SEMANTICS.md),
[map modeling](MAP_MODELING_PRINCIPLES.md), [engineering invariants](../AGENTS.md),
[current state](CURRENT_STATE.md), and [migration ledger](MIGRATION_LEDGER.md).
The [B2 implementation audit](../../audit/2026-09-28-b2-implementation/README.md),
[independent review](../../audit/2026-09-28-b2-independent-implementation-review/README.md)
and [R1 closure review](../../audit/2026-09-28-b2-r1-closure-review/README.md)
were inspected. The closure review's acceptance supersedes the older pending
review wording in the state documents. Historical evidence is not rewritten.

## 2. Source audit and starting state

Source paths below are relative to the Git root `src/`. Function names are the
stable anchors; line numbers describe the inspected working tree, not a clean
HEAD. The preexisting dirty tree is the implementation baseline.

| Source anchor | Current behavior and B3 consequence |
|---|---|
| `sapphire/src/mapping/graph/pose_graph.hpp`: NavigationGrid, ContinuationProgress | Grid already has map UUID and source graph revision, separately from its local occupancy counter. Progress has ready revision, accepted/completed counts and sequences, head state, W outcome and canceled range. No publication frontier exists. |
| `pose_graph.cpp:778–906`, `processContinuationHead` | W commits, owners materialize, cached grid is invalidated, then correction/ready/completed advance together. No B2 navigation callback is invoked. |
| `pose_graph.cpp:234–389`, B1 attachment | Q0 commits/materializes and exposes correction. B1's constructor/attachment path deliberately does not publish. B3 must notify after the explicit successful return, not publish during A2 reconstruction. |
| `pose_graph.cpp:430–453`, correction/grid access | Correction has no attribution in its return type. Lazy grid capture owns lifecycle then occupancy lock. Separate calls to correction, graphRevision and grid cannot establish one coherent read across commits. |
| `pose_graph.cpp:173–192`, finish | Stops admission, requests stop, joins worker, fences reads, destroys visual owner and finishes Memory/storage. No opportunity for a final lazy read after finish. Split drain from close locally. |
| `pipeline.cpp:249–266`, shutdown | Stops sensor admission, wakes producers, fences resume backend admission before joining mapping/odometry. Does not itself checked-finish backend; destructor destroys it later. |
| `pipeline.cpp:607–645`, submit_submap | Reads correction immediately after asynchronous admission and multiplies it by the submitted anchor. This does not identify completion of that submission; next input can accidentally drive output of earlier work. |
| `pipeline.cpp:35–48`, invoke_output | Callback exceptions are logged and swallowed. Returning from the current callback can mean only enqueue, not publication. Required B3 output must bypass this best-effort success convention. |
| `pipeline.cpp:648–737`, mapping producer | Drains marginal queue, builds/finalizes, and attempts a final short flush. Resume admission has already stopped, so its unaccepted tail is refused. New-map final flush is preserved and tested by B2. |
| `common/key_frame.hpp:252–255,302–338` | flush builds a real shorter submap only with a reference and nonempty valid cloud; otherwise returns no submap. It does not manufacture evidence from an image-only tail. |
| `sapphire_ros2/src/sapphire_node.cpp:165–190,340–385` | Six standard publishers plus TF; callbacks append arbitrary closures to an unbounded output deque; one existing output thread executes and swallows task errors. No output acknowledgment or revision status topic/service. |
| `sapphire_node.cpp:450–485` | map→odom TF, map-frame Odometry and OccupancyGrid lose revision identity. Grid dedup uses only local `revision`, not UUID/source revision; this can suppress a different map or reset output. |
| `sapphire_node.cpp:237–251,328–337`, `src/main.cpp` | finish parameter calls rclcpp shutdown before pipeline/backend destruction and output drain. Main returns 0 after ordinary destruction; destructor logs are not checked success. |
| `mapping/grid/occ_layer.hpp:82–113`, getMap | Const export builds local dense storage; cost is observed cells plus bounding rectangle area. It does not mutate authoritative occupancy. |
| `pose_graph.cpp:929–1040,1139–1240`, legacy new-map path | Separate legacy transactions, no B2 ready watermark, correction can use unrounded optimizer values, grid callback lacks source identity and eagerly exports. Cannot pretend this is Route-S or infer ready from DB alone. |

After successful B1→B2 handoff, DB and ready revision agree, correction belongs
to the live generation, and every successful B2 head increments the DB once.
During post-COMMIT materialization DB may lead ready; after failure, that gap is
real and retained. A2 alone has history and ready state but no active correction.

Consumer audit: production ROS subscriptions consume IMU/LiDAR/images, not map
revision events. Launch/RViz/configuration and map-export scripts provide no
per-revision event consumer or acknowledgement protocol. The map pose and grid
are current-state representations. This supports coalescing; it does not prove
unknown external consumers already understand the new contract. Standard TF,
Odometry and OccupancyGrid consumers need compatibility topics (section 5).

Local-map visualization is a separate **incremental best-effort stream**:
`pipeline.cpp:313–328`, `emit_local_map`, emits newly marginalized points; the
supplied RViz display accumulates them. A newer increment does not cover an older
dropped increment. Latest-slot replacement can leave visualization gaps; these
optional losses must be counted/observable and do not justify the unbounded deque.
Correction/navigation and a full trajectory representation are state-like outputs
whose latest-state coalescing can provide state coverage; the local-map increment
has no such coverage meaning.

Actual output QoS uses reliable KeepLast with volatile durability unless noted:

| Existing channel | Depth | Evidence / consumer |
|---|---|---|
| Scan | 10 | `sapphire_node.cpp:165` |
| Local map / trajectory | 2 each | `sapphire_node.cpp:166–167` |
| Odometry | 100 | `sapphire_node.cpp:168` |
| Map pose | 10 | `sapphire_node.cpp:169` |
| Navigation | 1, transient-local | `sapphire_node.cpp:170`; supplied RViz Map display also requests reliable, transient-local, KeepLast(1), `/sapphire/gridmap`, with `Use Timestamp: false`. It displays full state, not every DB revision. |
| Dynamic TF | 100 | Installed `tf2_ros/qos.hpp`, DynamicBroadcasterQoS default. |

Reliability/durability defaults are verified in the installed Humble
`rmw/qos_profiles.h` and `rclcpp` QoS construction. There is no existing map
status publisher QoS to inherit. The RViz configuration names a grid update topic,
but Sapphire currently publishes full OccupancyGrid messages, not an update log.

Two source/document distinctions are explicit:

- B2 design section 8 calls export allocation failure conservatively sticky;
  accepted current `latestOccupancyGrid()` instead propagates failure from a
  const export/local allocation without `recordFailure`. Keep source behavior:
  proven read-only generation failure is an output failure. Partial occupancy
  **materialization** failure remains sticky. This narrow reconciliation follows
  the failure invariant; it does not weaken any uncertain-mutation boundary.
- Existing new-map writes are not W's finalized-node route. B3 adds a coherent
  output/readiness seam at successful legacy processing completion, without
  converting its transactions or claiming B2 source→revision arithmetic there.

## 3. Frontiers: retain two map scalars, add two output scalars

Let U be map UUID, E a volatile publisher-process incarnation, g the existing
producer generation, C last known committed DB revision, Y last runtime-ready
revision, Pc last locally published correction group revision, and Pg last
locally published navigation group revision. Revisions are unsigned 64-bit at
interfaces. Unavailable is distinct from revision zero.

| Value | Advancing event / owner | Monotonicity, lag and lifetime | Exposure |
|---|---|---|---|
| C plus outcome-known flag | Known successful W commit; MapDatabase owns authority, Memory forwards it. Existing progress gains an observational copy immediately after commit. | Monotonic for U. Survives restart in MapState. Unknown preserves last-known C but makes actual current DB outcome unresolved. | graphRevision and attributed progress/status/final report; never expose last-known as resolved after Unknown. |
| Y | All required B2 owners and correction install succeed; backend lifecycle owner. A2 establishes initial historical Y. | Monotonic within owner. May lag C after materialization failure; a retained Y does not make failed mutable runtime readable. Runtime-only; rebuilt from C on A2. | Existing progress, attributed ready reads, status/final report. |
| Pc, optional | All fixed correction-group actions for a captured U,E,g,R return checked success; output executor owner (SapphireNode in ROS). | Monotonic within U,E,g. May lag Y due to coalescing, generation/callback/publish failure. Runtime-only; reset unavailable on new domain. | Output status, exact checked publication result, final report. Never backend graphRevision. |
| Pg, optional | All fixed navigation-group actions for captured U,E,R succeed; same output owner. | Monotonic within U,E; same lag reasons. Runtime-only; disabled is N/A, not 0 or Y. | Same output interfaces; NavigationGrid's source field is provenance, not proof of publication. |
| F and S, optional targets | Once producers and backend are settled, latch Y and last accepted source sequence for this drain; shutdown caller. | Fixed target, not advancing frontier. Runtime-only. | Checked finish report/status. |

Do **not** store another `drained_revision`. Drained is a predicate over admission,
head/tail accounting, F, required outputs and close outcome. Do not store a generic
`published_revision`: Pc and Pg can differ. If both are enabled,
`common_current_revision` is available only when Pc == Pg; it is a derived value,
not a monotonic frontier. `min(Pc,Pg)` does not prove both channels ever published
that exact revision when channels can skip revisions independently.

At a healthy quiescent point, known C == Y; every available Pc/Pg <= Y. With a
failed backend, these are historical watermarks, not a claim of current readable
runtime. U changes reset comparisons. E distinguishes a new process publishing
the same U,R (notably status and unavailable active correction); g distinguishes
frontend resets within a process. E is one random runtime identifier, not a DB
session or a new owner. No frontier other than C is persisted.

For live observability without waiting on a long lifecycle operation, extend
ContinuationProgress with last-known C, known/outcome, current head sequence and
first-cause diagnostics. Update C under the existing input mutex immediately
after W returns, and in W's Committed/Unknown exception paths before fencing.
This is a mirror of the DB owner's recorded result, never another authority or
SQL query under the input mutex. Updating Y/completion remains B2's existing
linearization point. Status may report an as-of observation; revision-sensitive
payload capture uses the stronger locking rule in section 7.

## 4. What counts as local publication

The transport-facing contract requires only an immutable payload with explicit
map/revision attribution, a live owned transport/publisher context throughout the
call, a local success/failure result where the API provides one, and completion
of the call before its transport owner is destroyed. Standard compatibility
views derive from that attributed attempt; they do not become revision sources.
No remote ACK, guaranteed delivery/enqueue, network-health certificate, every-
revision observation, middleware-internal resource accounting or hard publish
deadline is required.

A payload being constructed, placed in a Sapphire slot, or accepted by an
asynchronous Sapphire callback does not advance Pc/Pg. The configured local
consumer must complete its actual action. For a transport adapter that action is
local submission, not draining a private middleware queue. A completed submission
may succeed even if the transport does not retain or deliver the sample. This
contract covers configured sinks, not every possible remote reader.

For the current ROS adapter, required typed publication counts only when the
checked `rcl_publish` returns `RCL_RET_OK` **and** the owned ROS context remained
valid for the publication interval. This means only that the local adapter
completed the requested submission action. The API choice belongs to the ROS
adapter, not the backend/pipeline revision semantics. Check context validity
before and after the action; ownership excludes normal context shutdown during
it, and unexpected external invalidation conservatively fails the attempt even
if the call returned success. Do not retry with an invalid context or advance a
guessed frontier. The current rclcpp void return alone is insufficient: installed
Humble `rclcpp/publisher.hpp:453–470` can silently return for a shutdown context.
Sapphire currently uses the ordinary inter-process path; enabling intra-process
publication must preserve this same adapter contract.

TF and legacy Odometry are compatibility outputs. Revision-aware payloads carry
authoritative publication attribution; TF/Odometry are never authoritative for
C/Y or source revision. Track `sendTransform` completion conservatively in the
same owned, valid-context interval, with no reported error; any visible failure
or context loss leaves the group incomplete. Its void API supplies no extra
success/delivery certificate. No separate middleware theorem is required for TF.

No subscribers is success if the local action succeeds; never wait for a
subscriber count or `wait_for_all_acked`. Even a transport ACK would not prove
application consumption. Transient-local history is a living-publisher service,
not a durable output log. Required local publication may be clean even when no
external subscriber sees the final sample.

Normal graceful publication/drain assumes that an in-flight transport call
eventually returns or reports failure. This is an external platform/liveness
assumption, not a map invariant, persistent fact or deadline guarantee. If it
does not return, publication/drain and owner replacement may stall while C/Y
remain unchanged by transport. Later externally forced termination follows the
existing crash/forced-shutdown semantics; it does not manufacture clean drain.

## 5. Fixed output set and wire attribution

Required-output selection is fixed for one owner lifetime, recorded in status,
and independent of subscriber presence. Missing a configured required sink is
an explicit output-configuration failure, not silent success. A ROS-free backend
with no sinks supports backend drain only; it reports publication N/A and cannot
claim ROS drain. For an active ROS mapping session:

| Output | Classification and action |
|---|---|
| Revisioned map→odom correction, existing map→odom TF, revisioned anchor map pose and existing map-pose Odometry | One required **correction group**. Derive all from one ready capture. Pc advances only when every group action succeeds. Individual calls are sequential, not transactional; retry may duplicate a successful action. |
| Revisioned NavigationGrid and existing OccupancyGrid | One required **navigation group** when navigation output is enabled. Pg advances only after both actions. Disabled navigation is explicitly N/A even though A2/B2 still maintain occupancy evidence. |
| Status, logs, diagnostics and counters | Best effort on ROS; no status-publication frontier and no recursive requirement to publish publication success. When a checked operation returns, its caller/final local report receives failure/outcome information independently of this topic. |
| Local scan and full trajectory visualization | Best effort. Finite replaceable slots and payload caps; may drop, stop selection before final required output. A full trajectory is state-like; replacement may provide latest-state coverage. These channels cannot affect map correctness; an active call can delay drain under section 4's liveness assumption. |
| Local-map incremental visualization | Incremental best-effort stream of newly marginalized points. Latest-slot replacement deliberately loses distinct increments: newer does not cover older, and visualization gaps are permitted. Count/expose dropped increments; retain a finite slot, not the unbounded deque. Stop selection before final required output. |
| Raw odometry and odom→base TF, sensor traffic, feature diagnostics | B3-external measurement semantics. Share only bounded queue/lifetime handling. No DB-revision claim. |

For the current ROS adapter, preserve the existing topic QoS listed in section 2.
Proposed revision-aware correction and map-pose endpoints use reliable KeepLast(1),
volatile; revisioned grid and status use reliable KeepLast(1), transient-local.
These are ROS adapter choices, not map/backend invariants or timing guarantees.

Map pose here is the newest committed active submap anchor at its original
anchor timestamp, not the pose of a just-submitted Qn or a continuous tracking
pose. Export X_latest_committed directly. Correction remains
`double(X_latest_committed_float) * inverse(A_latest_original_double)`.
Do not relabel the original sensor time as a revision. TF broadcast time may be
current publication time; correction payload provenance remains U,E,g,R.

Existing `NavigationGrid.map_uuid/source_graph_revision` suffice in core; local
`revision` stays an occupancy counter. Extend existing correction/pose sink
parameters and the coherent read seam with U,R,g, source sequence and timestamp.
No MapSnapshot/RevisionSnapshot return object or stored aggregate is needed.

ROS Header has only time and frame name; OccupancyGrid, Odometry and
TransformStamped have no map UUID/revision field. Do not misuse frame_id,
map_load_time, covariance, or timestamps. Propose three small fixed wire messages
under the existing ROS package, each carrying the identity with its full payload:

- `RevisionedMapCorrection`: U, E, g, source_graph_revision, active source
  sequence, and TransformStamped for map→odom.
- `RevisionedMapPose`: the same identity and sequence, plus Odometry for the
  committed anchor (the contained header has the anchor timestamp).
- `RevisionedNavigationGrid`: U, E, source_graph_revision and OccupancyGrid.

Use companion names derived from the existing map-pose/navigation topic settings
and one configured correction topic; preserve standard topic names/types for
TF/RViz/navigation compatibility. These wire envelopes are necessary because
existing fields cannot carry provenance. They have serialization responsibility
only, no independent lifetime manager, combined map-state payload or second grid
owner. Generate them in the existing package; do not add a generic interface
registry or a separate package just for organization.

A separate metadata sidecar keyed only by timestamp is insufficient: messages
can be dropped/reordered independently. Revision-aware clients use the full
attributed messages, compare U/E/g where relevant and compare source revision
to observed ready status. Retained immutable old messages always remain old;
status is an as-of observation, not a guarantee of global instantaneous freshness.
Legacy standard topics remain explicitly unattributed compatibility views and
must not be advertised as a way to identify the current DB revision. A cache of
those alone cannot satisfy B3's revision-aware consumer contract.

A small `MapPublicationStatus` ROS message mirrors existing progress plus the
fixed fields in sections 3 and 14. There is no existing map status topic/service
to extend. This is the minimal observability seam, not a monitoring subsystem.
No ROS attachment-selection service is included; B2 currently exposes explicit
attachment through the core pipeline API, and the ROS constructor supplies none.
ROS integration tests may inject that existing API through a test seam.

## 6. Coalescing, coherence and output failure

These are latest-state outputs, not events. It is legal to publish R then R+3;
there is no obligation to materialize dense output or send R+1/R+2. A successful
newer state covers an older state request in the **at-least revision** sense,
not as evidence that the old exact state was ever sent. Exact intermediate
revision replay is not supported.

Correction can publish promptly while navigation lags. Each message tells the
truth about its own source. When navigation is selected, capture correction,
anchor pose and grid at the same R; send correction group then navigation group.
Prepare both messages before calls when possible, but grid allocation failure
must not prevent a separately captured ready correction from being published.
A single output executor serializes attempts, so no old attempt can overwrite a
newer published value. Never send correction R behind an already published R+1;
if preparing a fresh grid, recapture the newest available revision instead.

Partial success is real: Pc=R, Pg<R is allowed online. If a call fails within a
group, that group frontier remains at its previous value even though some
individual topics may contain R. Do not roll back another publisher or the DB.
Consumers needing a coherent pair match actual attributed payloads at one R;
status saying Pc=Pg does not prove that particular consumer received both.

Output failures retain a cause/failed target per fixed group and remain separate
from backend Failed. Retry read-only export or a failed publication from retained
immutable values; same-revision content (geometry, grid, source identity and
anchor time) stays identical. Transport send time may change on TF rebroadcast.
Duplicates are legal. A newer ready revision can supersede the failed online
target. Missing/invalid ready invariants are backend failures, not retryable
transport errors. Any unexpected mutation discovered while generating output
uses existing sticky failure semantics.

For each group/identity/revision, allow at most one automatic online retry after
the initial failure, no sooner than the configured backend update period. Retain
one bounded attempt counter and cause; after exhaustion wait for a newer target
or explicit retry. One explicit request permits one attempt and coalesces in the
same fixed slot; requests never form a queue or replenish automatic retries for
that target. A newer target resets that target's counter. Final drain uses a
frozen target and one checked final attempt per missing group, even if its online
retry budget was exhausted. Failure returns an incomplete-output result after
safe cleanup; there is no automatic unbounded retry and no wait for operator
input in finish.
A caller may explicitly retry a quiescent owner's output before choosing close.
An output-only failure never writes MapState, increments C, invalidates correct
occupancy or calls backend recordFailure. Slow transport, lost subscribers and
publish failures cannot roll back C/Y, create a revision, mutate solver/poses or
occupancy, reject a loop, or fabricate backend failure. Only a genuine internal
map/runtime invariant or mutation failure uses existing sticky backend semantics.

## 7. Execution model and lazy generation

Reuse **SapphireNode's existing output thread**; no new publication worker.
Add a narrow ready-change notification to the existing pipeline/output boundary.
B1 successful handoff and B2 ready completion notify after releasing backend
locks. The notification stores only a dirty/latest-ready scalar and wakes the
output CV. It does not generate grids, publish, query the backend or wait.
A failed/lost notification cannot lose final publication: final drain explicitly
requests F, and a low-rate output-thread timed predicate checks progress while
running. Callback allocation failure cannot turn an already completed W head
into a fictitious processing failure.

The output thread pulls attributed ready data through the existing pipeline and
backend owners. One lifecycle acquisition checks healthy/current/generation and
copies correction, latest committed anchor pose, timestamp, source sequence,
U and Y into caller-local variables. On a grid turn it also acquires the existing
immutable NavigationGrid at **that same Y**, validating its source fields. The
backend does not invoke user callbacks inside this read. Release every backend,
DB, pipeline-owner and output-queue lock before conversion, publishing or invoking
local consumers. Separate accessor calls followed by checking revision are not
an adequate replacement for coherent capture. No transport call holds a DB
transaction, lifecycle mutation lock, solver lock, occupancy mutation lock or
admission lock. Slow transport therefore cannot directly hold map mutation back
through algorithm lock ownership; mapping can continue and coalesce independently.

For legacy `processPending()`, backend lifecycle ownership must start **before
`copyPendingFrames()`** and cover all solver, storage, index and occupancy
mutations through final ready installation. Locking only the final Y update is
insufficient: no attributed/current capture may observe partially mutated owners.
Retain failure and fence current capture as required before another current
capture can proceed. Readiness/output notification occurs only after releasing
backend locks; conversion and transport calls remain outside lifecycle ownership.
Section 11 specifies the configured visual-owner completion/failure accounting.

Online correction wakes on readiness. Dense grid is selected at most once per
`pose_graph.update_period_sec` while there is a newer ready revision; the output
thread's existing CV can time this without a timer/new thread. Retry uses the
same minimum interval. Explicit local grid reads remain lazy; final drain forces
one export for F if no matching cached grid exists, irrespective of that cadence
or subscriber count. No eager export in B2's commit/materialization path.

The actual `getMap()` reads mutable occupancy: hold lifecycle ownership for the
export to avoid copying all evidence or adding immutable occupancy generations.
It can temporarily delay the next head; this unavoidable cost is measured.
Use a nonblocking/try capture online if lifecycle is busy, retaining one dirty
request and retrying after the interval. Final capture occurs after worker join.
All ROS serialization and calls occur outside lifecycle. Do not claim export
is O(1) or that mapping never waits for a selected dense read.

A grid that was captured before a later failure is immutable and still correctly
attributed. Current mutable access becomes unavailable on sticky failure.
The failure rule in section 12 controls whether an already selected attempt can
complete; do not reconstruct old occupancy just to finish a partial publication.

## 8. Bounded Sapphire-owned publication state

Replace the arbitrary output closure deque with fixed slots on the existing
node: one map-output dirty/request scalar; one pending latest value for each of
the existing odom, scan, trajectory and local-map callbacks; one status dirty
flag. Values replace old values under output_mutex. One active task owns its
immutable payload outside the mutex. No per-revision FIFO, queued retry closures
or unbounded publication promises/results. Group errors and retry counters have
fixed storage and section 6's finite attempt policy. Local-map incremental
visualization uses a finite replaceable slot too, but replacement is deliberate
loss of a distinct increment, not state coverage. A newer increment cannot cover
one dropped earlier; visualization gaps are allowed only because this output is
optional/best-effort. Count/expose discarded increments, including replacement,
oversize/error drops and drain discard. Correction/grid and full-trajectory state
coverage remain separate; incremental loss does not justify an unbounded deque.

Required map actions have selection priority; optional work cannot starve them
through an application-created queue. When drain is requested stop selecting
optional visualization/telemetry, including status and shared-thread measurement
outputs, and drop their pending slots before selecting final required output.
An optional call already executing must return before this single output thread
can proceed; it shares the external liveness assumption, not a hard duration
bound. Optional failure/drop never changes map correctness.

There is at most one active map capture, one current backend cached grid and one
pending scalar demand. Retained failed-group values occupy fixed group slots and
are released on supersession/close. Conversion buffers and the two ROS grid
representations add a fixed number of capped payload copies, not one per revision;
account for those copies explicitly. Check sizes before retaining/copying pending
payloads and before dense output allocation. External consumers holding old
immutable values are responsible for their own retention. The full trajectory
and existing graph/index/occupancy residency are map/run-dependent; publication
caps bound retained output copies, not authoritative map storage or total RSS.

Enforce optional caps at the **producer-side publication-copy boundary**, not
only when the node receives a callback. In `SlamPipeline::emit_local_trajectory`
and `emit_local_map`, check before both current copies of `trajectory_` into a
publication vector (`pipeline.cpp:310,328`), and before any other optional
publication copy/assembly. An oversize output is dropped/counted before creating
that copy. Wrap output-only copy/conversion allocation failure at its actual
creation site, including failures before `invoke_output`: required output retains
an output error, optional output retains drop/error accounting. Neither changes
C/Y/map state or fabricates backend failure; authoritative owner mutation still
uses its existing failure semantics.

**Application payload envelope.** The supported navigation requirement is up to
200 m×200 m at 0.05 m/cell: width and height each at most 4,000, with at most
16,000,000 cells. A complete required grid message, whether standard or attributed,
has a Sapphire boundary cap of **16 MiB (16,777,216 bytes)** in its encoded
representation. All non-cell fields together, including revision metadata,
strings, lengths, alignment and serialization overhead, must fit **64 KiB
(65,536 bytes)**. Thus the maximum cells plus this allowance fit below 16 MiB.
The cap is per message, not a combined limit for both required grid actions.

Bound every frame identifier to 255 UTF-8 bytes, UUID/incarnation text to its
canonical maximum 36 ASCII bytes, and each diagnostic text field to 1,024 UTF-8
bytes. Attribution scalars retain their fixed widths. Correction, pose, TF and
status messages each fit within 64 KiB encoded, with fixed field/list counts for
these B3 actions. Each optional scan/local-map/trajectory output is capped at
16 MiB including metadata; reject/drop an oversize output before a pending copy,
rather than retaining a growing full-trajectory copy in a fixed-count slot.
These are Sapphire interface budgets, independent of any transport's allocator,
network framing or total internal memory. The adapter must account for its chosen
encoding within the boundary budget; deployment message limits belong in adapter
validation. No middleware-internal memory theorem is required.

Before selected dense grid allocation, derive extents under the same coherent
lifecycle/occupancy capture used for the export. Widen signed coordinates **before**
subtraction; use checked subtraction/addition for `max - min + 1`, validate
representability before narrowing, and checked width × height multiplication.
Enforce configured dimension/cell/payload limits within the envelope above using
a conservative encoded-size estimate, and check allocation-byte arithmetic before
allocating the dense representation.
A post-`getMap()` check is too late; current `occ_layer.hpp:82–113` allocates inside
that call. Only after these checks may dense allocation/conversion proceed;
occupancy cell semantics remain unchanged. After conversion/serialization, verify
the actual encoded size at the adapter boundary as well.

Account separately for authoritative/native occupancy state, the dense native
grid buffer, each converted ROS/message buffer, and encoded/serialized storage.
Use checked element-count × native-element-size calculations and the fixed copy
counts for the native/conversion allocations; the 16 MiB encoded cap is **not** a
16 MiB total-native-memory guarantee. Authoritative occupancy retains its existing
map-dependent residency contract.

Oversize required payload or invalid required identifier is an
explicit output failure: never truncate geometry, strings or attribution, change
a revision, or invalidate healthy C/Y. Optional oversize visualization/status may
be dropped with a bounded diagnostic/counter. The completed probe's standard
grid encoded to 16,000,104 bytes; future revision-message implementation must
verify these budgets in its serialization tests.

No subscriber ACK wait is introduced. Bounded here guarantees Sapphire-owned
slots, payloads, retry state and result retention, **not** a hard wall-clock bound
for a third-party call or total middleware memory. While a call runs, only bounded
pending state coalesces. A CV timeout cannot safely cancel that call, detach it or
permit transport-owner destruction. Eventual return is section 4's external
liveness assumption; its failure can stall graceful drain without altering C/Y.

## 9. Submission completion and revision attribution

Admission Accepted means ownership transfer only. Existing last_accepted,
last_completed, counts and canceled tail remain the bounded accounting. Add
current head sequence (the existing head status alone does not name it), and a
completed_revision field coupled with last_completed; this explicitly pairs the
last completed source and ready revision under the input mutex. B1 exposes its
already-known root source sequence S0 and committed root revision R0.

For this accepted B2 contract only, successful ordered continuations have
`R(S) = R0 + (S - S0)`: one frozen node per W increment, contiguous single
producer, no intervening independent writer/commit API. Check overflow; bind the
formula to U,g and the B1 root. All admitted sources <= last_completed have
completed at their derived revision. A canceled source must be reported through
its outcome, not by extrapolating arithmetic. If diagnostic low-level commits
are permitted concurrently, this mapping is invalid: disallow them while active
B2 just as its single writer contract requires. Legacy new-map callers use
observed completion pair only; no equivalent arithmetic is claimed.

A state output at R with last completed source S says which committed prefix it
contains, not that it is the original immutable pose of Qs; later optimization
can change earlier poses. Waiting for publication through S means Pc>=R(S) and,
if enabled, Pg>=R(S), at the same U/E/g. It does not promise an exact message for
S or common intermediate revision. Exact final coherence is stronger (section
10). A caller wanting current read state gets the actual captured R. No
unbounded per-submission result store is needed.

The existing canceled range can include the failed head even when its W outcome
is Committed or Unknown. Interpret this as canceled **completion**, not proof
of absent rows. Preserve failed-head sequence and W outcome separately; later
unprocessed tail sources were never committed. Never label the entire canceled
range as rolled back.

## 10. Ordered normal shutdown

Use three explicit operations on existing owners: upstream stop/join, backend
accepted-work drain, and storage close. Preserve `PoseGraphBackend::finish()` as
a checked composition of backend drain plus storage close for direct callers;
it still does not assert output completion. Introduce a narrow drain-only entry
that leaves healthy runtime readable until explicit close. Pipeline's checked
stop/drain and close entries expose these two boundaries to its caller. The ROS
node/main caller sequences its own output executor between them; the pipeline
does not join or own the ROS thread. A ROS-free caller can run its synchronous
configured sinks in the same interval and retain their checked results.
Destructors perform safe cleanup and contain exceptions, never supply the
success verdict.

Lifecycle states are descriptive fields on existing owners:
Running → InputFenced → ProducersJoined → BackendDrained → OutputsSettled → Closed.
A returned failure is retained while safe cleanup continues; there is no new
manager. An active transport call must finish before cleanup can destroy its
owners. This sequence defines ordering and checked completion, not a fixed
wall-clock deadline; a nonreturning call can stall graceful shutdown.

1. **Request stop while ROS is alive.** The finish callback only records the
   request and cancels executor spinning/wakes the control path; it must not call
   rclcpp shutdown. Main stops scheduling sensor callbacks and waits for active
   callbacks to return before destroying subscriptions or pipeline. All input
   callbacks, including both right-image lambdas and LiDAR timer, obey the same
   input fence. Keep node/publishers/context alive.
2. **Fence raw input and resume admission.** Pipeline stops sensor admission and
   Synchronizer intake, sets stopping, wakes input/marginal/attachment waiters.
   Resume calls stopAdmission(false) before joining any producer blocked in
   submitContinuation. The backend input lock defines S=last accepted. An
   in-flight B1 attachment is joined and its actual commit result retained;
   no unstarted B1 is launched just to flush shutdown tail.
3. **Join upstream workers.** Healthy processing of already available finite
   synchronized inputs may stop at existing safe boundaries; raw buffers are
   not a durability promise. Resume builder/unaccepted frozen tail is discarded
   with counts/reason. New-map follows its existing finite marginal drain and
   final valid short-builder flush, then fences backend admission. No fresh
   sensor input or indefinite attachment/capacity wait is allowed.
4. **Drain accepted backend work.** Request backend stop; join worker without
   holding lifecycle/input/output/owner mutexes. Healthy accepted B2 heads
   complete in order through W and runtime-ready. Retryable waiting head is
   canceled with its tail and retained cause, exactly as B2 finish specifies;
   no automatic solver retry. Sticky failure cancels later heads. Do not close
   Memory or destroy runtime yet. Settle owned visual work as necessary.
5. **Select final target.** In the healthy attached case latch known C==Y==F and
   the completed source prefix through S. Require every accepted source through
   S complete, no head/waiting payload and no canceled accepted work. Hold no
   lock while waiting for output. Correction remains valid throughout healthy
   drain; normal admission stop is not reset/Failed.
6. **Settle publication.** Stop selecting optional work and drop pending
   telemetry/status. The output thread completes any older in-flight call under
   the external liveness assumption, then captures F. No deadline is promised.
   Required correction actions run before required navigation actions. If only
   one group already equals F,
   execute the missing group using the coherent F capture. Force lazy export
   only when needed. Require Pc=F and, if enabled, Pg=F; all pending/active
   required work is gone. One final attempt per missing group; report failure
   rather than silently treating a queued task as done. Join the output thread
   before its backend or publishers can be destroyed.
7. **Checked storage close and report.** Finish backend visual/prefetch lifetimes,
   Memory and SQLite in W order; release external descriptors/admission only
   after complete close. Preserve original processing, output and close errors
   separately. Report final scalar values/cancellation and return failure for
   incomplete mapping, required output or storage finish. The final local caller
   report remains available without a status-topic send. Do not select a new
   optional transport call after beginning final required publication.
8. **Destroy safely.** Destroy pipeline/backend, publisher state/node, then call
   rclcpp shutdown. Checked main returns nonzero for incomplete required work;
   the existing always-success-after-destructor convention must change.

A healthy active mapping shutdown is clean exactly when admission/producers are
fenced and joined, accepted work through S completed without cancellation, known
C==Y==F, Pc==F, Pg==F if enabled, no required task remains, and storage close
succeeded. No subscribers, dropped best-effort output and declared unaccepted
frontend tail do not violate that predicate. Navigation disabled is N/A, never
an invented Pg=F. No active B1 yet means correction N/A and explicit
`not_attached`; historical-only A2 retains no automatic output. An empty new map
has no active correction and reports no mapping output target rather than
publishing identity as a real correction. A configured historical grid request
is allowed independently but is not active-session completion.

SIGINT/SIGTERM must request this same normal control path **before** context
shutdown if they are to count as checked normal shutdown: disable the default
rclcpp context-shutdown handler for the owned context and use a signal-safe
request bridge to wake the main control path. Do not run joins, logging or ROS
calls in a raw signal handler. No extra worker is required. A forced context
shutdown or second forced termination follows failure/crash semantics and cannot
be called clean B3 drain. This is a bounded local main/node change, not a generic
signal/lifecycle framework.

## 11. Frontend tail, new-map and reset decisions

| Tail at normal stop | Decision |
|---|---|
| B2 already Accepted frozen Qn | Drain through W/readiness unless retained failure explicitly cancels completion. |
| Resume producer-held frozen Qn waiting for capacity | Stop wakes wait; refusal retains ownership then discard/report unaccepted. It is not in accepted S. |
| Partly filled resume SubmapFrameBuffer; queued marginalized evidence | May discard after admission fence. Do not delay fencing to admit an opportunistic last Qn. This preserves accepted B2 shutdown behavior. |
| New-map marginalized evidence and short builder | Preserve existing final flush. It must pass the normal frozen-submap, LocalGrid and scene contracts. Empty/invalid evidence does not become a fake node. Failure reports incomplete mapping. |
| Frontend active sliding window, initializer and current unmarginalized points | Not finalized evidence. Do not invent marginalization/optimization or serialize ESKF to empty the window. May be lost. |
| Pending LiDAR synchronization, missing future IMU coverage, dual-lidar timeout tail | No wait for new input at shutdown. May discard/count; no fabricated IMU or interpolation across loss. |
| Image-only queue/features | Sampling evidence without a valid geometric submap; discard/count, no node and no mandatory last ORB extraction. |

For resume, skip expensive builder/grid work once the input fence makes transfer
impossible. If a partial Qn was finalized and accepted **before** that fence, it
must have the identical LocalGrid/scene/evidence contract as ordinary B2; “short”
is not an exemption. Accepted mapping work is durable only after known W COMMIT,
not merely because it entered a marginal queue.

Legacy `processPending()` must hold backend lifecycle ownership from before
`copyPendingFrames()` through all solver, storage, index and occupancy mutations
and final ready installation. Holding lifecycle only for the final Y update is
insufficient. Attributed/current capture cannot observe partially mutated owners;
retain failure before permitting another current capture, applying the existing
read fence for uncertain mutation. Readiness/output notification happens only
after backend locks are released, and transport calls stay outside lifecycle.

At successful legacy completion, verify final committed pose/index/occupancy
and all configured visual-owner work, use serialized committed newest-node pose
for public correction, tag/invalidate the cached grid, and record Y at the actual
final DB revision. Disable the legacy eager callback path in favor of selected
export. Intermediate legacy DB commits are not published as ready; idle processing
alone must not advance C or Y. This adds no W transaction or optimizer iteration,
retains existing final-builder persistence and does not redesign legacy storage.

Visual readiness accounts for insertion, query preparation, descriptor/archive
mutation associated with query preparation, and owned asynchronous/training
completion where applicable to required owner state. Caught visual exceptions
remain visible to readiness/failure accounting: they cannot be silently converted
into successful no-match processing or cleared by a later batch. A normal return
from `processPending()` alone does not demonstrate owner completion.

- Valid non-failures: empty feature input, a legitimate no-match result, or a
  normally completed query with no accepted candidate.
- Failures: insertion exception, query/preparation exception, uncertain descriptor/
  archive mutation, or owned training failure/incomplete required owner state.
  Retain the cause and do not promote Y without demonstrated required completion.
  Uncertain owner mutation uses existing sticky backend failure semantics and
  forbids Y/ready promotion; a later batch cannot erase that failure.

This is completion/failure visibility on existing owners, not a change to the
visual algorithm, retrieval policy or training policy. No completion manager or
new owner is introduced.

Reset differs from normal drain: invalidate generation/correction immediately,
stopAdmission(true), account queued old-domain work, and wait for any admitted
processing head per B2. Drop old pending output; already executing immutable
old-domain publication may complete under its original identity, but no new
old-domain action starts after the output executor acknowledges reset. Wait for
that acknowledgment/join before replacing the backend or reusing TF frame names.
No lock is held across that wait. Clear output frontiers/dedup for the new owner;
never compare local occupancy counters across maps. Old shared messages remain
readable with their old attribution, not current correction.

## 12. Failure matrix and permitted output

Use R for the attempted new revision and p for the previous ready revision.
Publication values denote successful group watermarks, not remote receipt.

| Situation | DB | Runtime-ready / reads | Publication | Required action |
|---|---|---|---|---|
| Normal R commit/materialize/publish | Known R | Y=R, current | Required groups reach R | Continue; final close can be clean. |
| W NotCommitted after solver mutation | Known previous C | Y=p retained; mutable reads fenced | Existing Pc/Pg remain | Sticky backend Failed; cancel tail, checked cleanup, reconstruct before reuse. |
| W Unknown | Last-known C only; actual unresolved | Y=p retained, unavailable current | Freeze new attempts; no guessed R | Failed, preserve Unknown/failed sequence, safe close/recovery then A2; no blind retry or inferred rollback. |
| W Committed then materialization failure | Known R | Y=p<R, reads fenced | Never publish R | Failed; cancel later work; preserve durable failed-head outcome. |
| Y=R, required publish failure | Known R | Y=R remains healthy | Failing group's watermark unchanged; other may be R | Output error; bounded retry/supersession online; final incomplete if not repaired. |
| R published, R+1 processing fails | R or R+1, or unresolved according to W | R retained; current reads fenced | Published R remains a valid historical representation | No relabel/retraction promise; failure status and checked failure report. |
| Shutdown with healthy accepted pending heads | Advances in order | Advances in order to F | Older attempt then final F | Fence first, drain, force selected final export, close last. |
| Shutdown with retryable head | Prior known C | Prior Y; finish retains failure | No new required retry demanded after backend failure | Cancel incomplete head/tail with retained cause; non-clean finish, no wait for another solver retry. |
| Shutdown after sticky failure | Preserve W classification | No fabricated readiness | Previously sent state remains; pending work dropped | Join/close/report; no silent recovery in owner. |
| No subscribers | Unchanged by publishing | Unchanged | Checked local action may reach R | No connection/ACK wait. |
| Slow subscriber/full middleware queue | Unchanged by publishing | Mapping can continue/coalesce | Active attempt pending or fails; no false advance | Fixed Sapphire slots/payload caps; eventual return is external. Graceful drain may stall, never mutate map state or fake completion. |
| Lazy grid allocation/conversion failure | Unchanged | Y unchanged, occupancy proven unchanged | Pg stays old; Pc may advance | Output failure; retry same/current R, never relabel old cells. |
| Context already shut down | Unchanged | Current readiness remains a fact | No local success inferred from void return | Output incomplete; safe close, nonzero checked outcome. |
| Storage finish failure after final publish | Last known C/F, preserve actual outcome | Previously ready F | Pc/Pg may equal F | Shutdown still failed; retain ownership if complete close fails. |
| Reset while output is active | Head may finish per B2 | Old generation invalid for future reads | In-flight old identity may complete | Await executor acknowledgment, clear pending old work, then replace. |

Minimal backend-failure rule: stop selecting new map output when failure is
observed; discard pending requests and do not retry from mutable runtime. One
attempt already selected from healthy immutable state may finish its own
captured actions; it never names an unready revision. Serialize the failure
notification and selection under the short output mutex to define their order.
Previously captured objects can be read as historical values but are not
re-published as “current” by cleanup. Publish failure/status best effort, including
current=false and the actual W outcome. No final Pc=Pg promise after failure.
This avoids maintaining an old mutable runtime or retaining every old grid.

Processing failure, output failure, and close failure each retain their own
first cause. Do not let a later callback/logging error erase Unknown or the
original ISAM exception. All worker/destructor boundaries contain non-standard
exceptions in the same local manner as accepted teardown; explicit checked
operations still report them. W's deliberate complete-close fail-stop is intact.

## 13. Locks, CVs, callbacks and object lifetime

Current graph order is lifecycle → output → input at the B2 ready install;
lazy grid uses lifecycle → occupancy; committed indexes/DB are reached beneath
lifecycle. Submission owns its submitter mutex and input/capacity wait, never
lifecycle. stopAdmission uses input only. Preserve these relations; no reverse
input→lifecycle acquisition is introduced. Failure mutex is held only to copy
or retain cause, never across a join or acquisition of another wait lock.

| Boundary | Rule |
|---|---|
| Pipeline pose_graph_mutex | Protect owner acquisition/replacement and short status access. Do not hold during admission waits, backend/output joins or callbacks. Runtime borrow for output is pinned by owner lifetime: stop new captures, acknowledge current capture, then replace. |
| Backend lifecycle / algorithm ownership | Serializes mutation and coherent output read. May contain dense export; never middleware/consumer callback. No transport call holds a DB transaction, lifecycle, solver, occupancy or admission lock. Finish/drain serialization may use existing finish_mutex, but worker does not need it. |
| Legacy processPending lifecycle interval | Acquire backend lifecycle before copyPendingFrames; hold through all solver/storage/index/occupancy mutation and final ready installation, not only the Y update. No attributed/current capture may see partial owners. Retain failure before another current capture proceeds; notify only after backend locks release. Transport remains outside lifecycle. |
| Backend input/capacity CV | Predicates include stop, failure, reset and retryable state. Publish C/Y/completion as short scalar updates. No waits while holding lifecycle/output locks. |
| Node output_mutex/CV | Own fixed slots, selected-attempt identity, Pc/Pg, error fields and stop/final-target flags. Extract local work then unlock before backend capture, conversion or publish. No call back into backend under it. |
| Completion notification | Invoked after backend locks release, only updates node/pipeline signal state. Callback capture outlives worker. No join/reset/reentry; test-hook installation/acquisition remains synchronized as R1 established. |
| Join/close | Main holds none of the preceding locks. No worker joins itself. Output thread finishes before pipeline/backend and publishers are destroyed. No publish/fini concurrency. |

A stop flag store is followed by the existing associated queue-mutex handshake
and notify_all so no waiter misses a transition. B3 adds the same predicate
handling for pending/final output, failure and generation changes. Reading
progress during drain remains allowed and does not itself make drain fail.
Attributed correction/grid reads during healthy backend drain block behind a
head or use online try-capture, then recheck health/current/generation. After
backend drain and before close they remain available for F. At close entry
fence new reads, wait for active capture borrowers, then close. Owned immutable
NavigationGrid messages already returned remain readable after close with their
original U,R. Raw runtime references never escape.

Repeated checked finish returns/rethrows retained outcome after idempotent safe
cleanup; it does not create a second final target or run another solver pass.
Owner replacement requires successful old close as in W. A caller bypassing
checked finish gets safe cleanup, not a clean-publication guarantee.

## 14. Status, final reporting and restart

Extend existing progress/caller reporting, not a new owner: U, known C/Y,
backend health, current-read/correction availability, g, B1 root S0/R0,
last accepted/completed and completed_revision, head sequence/state, canceled
completion range/reason, failed-head W outcome, output enabled mask, Pc/Pg,
last output errors, pending/in-flight indicator, coalescing counters, drain S/F
and close result. ROS adds E and serializes this fixed status best effort.
A bounded error summary accompanies retained exception_ptr locally. Updating
status creates neither map revision nor publication obligation.

During processing, node status combines labeled as-of progress and output
watermarks; it is not an atomic view of arbitrary runtime owners. Capture output
watermarks first, then backend scalar progress, and reject a changed U/g before
reporting combined comparisons. Live status remains advisory; terminal report
is taken after workers stop. Logs alone cannot satisfy the checked API. Main's
exit code distinguishes successful required work from incomplete/failure.

Restart restores C from MapState and reconstructs Y using A2. Pc/Pg are
unavailable, E changes, and correction/source-generation binding is unavailable.
Do not publish an old live correction from DB odometry anchors. Fresh B1 is
required even if numeric g starts at zero again. Historical grid may be read
with U,C but no active-session implication. Prior publication state is neither
needed for A2 nor recoverable from the DB; no output ledger is proposed.

Abrupt process loss keeps only accepted W durability/recovery guarantees.
WAL/NORMAL power-loss limits, sidecar recovery and A2 numerical support remain.
No crash-safe final delivery, replay of canceled tails, session resurrection or
publication journal is added.

## 15. Exact answers to the twelve design questions

| Question | Decision |
|---|---|
| One published revision sufficient? | No: Pc and Pg, with enabled/N/A state; equality is only a derived common-current value. |
| Correction/grid both required? | Correction group for an attached mapping output owner; grid group when navigation enabled. Fixed selection, never subscriber-dependent. |
| Latest-state coalescing legal? | Yes; current representations have no in-repository revision-event consumer. |
| Every ready revision externally published? | No; at-least state coverage, exact final F only. |
| Dense export every revision? | No; selected periodic/latest demand and final fence only. |
| Exact local success? | Completed local adapter submission; ROS checked success with context valid throughout the owned interval. Group completes only after all actions; no enqueue/delivery/health certificate. |
| No subscriber successful? | Yes, if the local action succeeds; no ACK or subscriber wait. |
| Output failure fails backend? | No for proven read-only generation/transport failure; uncertain map mutation still fails backend. |
| Clean shutdown DB==ready==published? | Known C==Y==F and every enabled required group==F, plus accepted-work and close success; no fictitious scalar for disabled outputs. |
| Which frontend tail finalized? | Preserve valid new-map short flush; resume unaccepted tail may be discarded; no synthetic frontend marginalization. |
| New worker necessary? | No; reuse ROS output thread and caller/main shutdown control. |
| Minimum additional state? | Pc/Pg availability, bounded pending/active fields and output errors on existing node, C/completion/head attribution in existing progress, volatile E, fixed drain targets and close outcome. No persistent state or generalized owner. |

## 16. Persistent format and architecture restraint

No SQLite schema, row, table, user_version, config identity, Link encoding,
completion marker, durable session, optimizer serialization or revision rule
changes. No revision for materialization, correction, grid export, local send,
subscriber action or drain. New ROS wire types are a communication interface
addition, **not a persistent-format migration**.

MapDatabase/Memory own persistence; backend owns runtime/ready; pipeline owns
producer/backend lifetime; SapphireNode owns ROS execution and publication
watermarks. Wire messages and local variables are data transport, not map-state
owners. No PublicationManager, RevisionCoordinator, DrainManager, Snapshot,
OutputTransaction, RevisionContext, DeliverySession or generic registry/scheduler.
Persistent facts remain distinct from derived runtime/index/publication state.

## 17. Implementation-scope forecast (not edits)

| Likely file/owner | Bounded purpose |
|---|---|
| `sapphire/src/mapping/graph/pose_graph.hpp/.cpp` | Coherent attributed read into existing payload/scalar outputs, scalar commit/completion observations, readiness notification, drain-before-close entry, legacy completion readiness/output gate. Keep B1/B2 algorithms and W calls unchanged. |
| `sapphire/src/pipeline.hpp/.cpp` | Stop/join/drain/close sequencing, checked outcome path, resume tail accounting, wake-only ready bridge, remove admission-time completion inference, owner-borrow lifetime. Check optional payloads before producer-side output copies, including both trajectory-copy emission sites; contain output-copy failures locally. Preserve new-map flush. |
| `sapphire_ros2/include/sapphire_ros2/sapphire_node.hpp`, `src/sapphire_node.cpp` | Replace closure backlog with fixed slots, existing output-thread capture/scheduling, checked required actions and frontiers, final target, status and error retention. |
| `sapphire_ros2/src/main.cpp` | Checked normal shutdown before context loss; signal request path; correct exit outcome. |
| `sapphire_ros2/msg/*`, package.xml/CMakeLists.txt | Fixed attributed wire messages and status generation in existing package. No speculative service or separate interface framework. |
| Existing ROS configuration/README | Topic attribution, application payload budgets, enabled outputs, adapter/deployment validation and normal finish/liveness assumptions; no broad QoS retuning. |
| Existing continuation/attachment/history and ROS tests; small focused publication/drain test | Deterministic seams and real process witnesses listed below. No production fake solver/storage. |

No change is forecast to MapDatabase/Memory persistence, loop_policy,
registration, descriptor algorithms, LocalGrid representation or occupancy
fusion. `getMap` already provides const export; the selected dense export must
check widened signed extent subtraction/addition, representability, width × height
and allocation-byte multiplication, and configured dimension/cell/payload limits
under the same coherent occupancy capture **before** dense allocation. Retain the
post-conversion/serialization encoded-size check at the adapter. Account for
native occupancy, dense native grid, converted message and encoded buffers
separately; preserve occupancy cell semantics. Producer-side checks cover all
optional publication-copy sites, including both full-trajectory copies, with
output-only allocation failures contained as section 8 specifies. Large-scale
file movement is not justified.

## 18. Required deterministic core tests

Future implementation tests use barriers/CVs at actual boundaries, never sleeps
to claim ordering. Retain all accepted B2 assertions and synchronized hook access.

1. Barrier immediately after W commit: C=R, Y/Pc/Pg=p; attempted capture cannot
   publish R until all owners pass ready. Release then verify exact progression.
2. Block a required sink after ready: Y can advance under subsequent B2 heads,
   publication remains pending; one dirty request replaces many without payload
   FIFO growth. Capture and publish newest after release with exact source data.
   Assert no algorithm locks span the blocked sink, no early Pc/Pg advance or
   owner destruction, and no clean-drain claim before release. The barrier tests
   Sapphire ownership, not a third-party deadline.
3. Qn admission followed by read before completion returns previous attributed
   R, never labels it Qn. Verify completed source→revision formula, B1 root,
   overflow checks, current-head identity and failed-head Committed/Unknown range.
4. Correction success/grid failure, grid success/correction subaction failure,
   synchronous/nonstandard sink exceptions and allocation failures: preserve
   truthful partial frontiers, DB bytes and healthy ready state; retries duplicate
   exact content or explicitly supersede. Verify the finite per-target retry
   budget, oversize required failure and optional drop before retained copies;
   C/Y and map data stay unchanged. No false min/common frontier. At every
   producer-side optional copy site, including both trajectory emissions, verify
   oversize drop/count happens before allocation of the publication copy. Inject
   output-only copy/conversion allocation failure before callback invocation;
   required output reports error and optional output records drop/error without
   changing C/Y, map data or backend health.
5. Dense generation only on selected demand/final F; verify no per-head getMap,
   no stale-grid relabeling, including equal occupancy counter/different graph
   revision and different UUID. Hold an old shared grid across commit/close.
   Exercise extreme signed extents and envelope boundaries: widened checked
   subtraction/addition, narrowing and width × height/allocation-byte checks must
   reject invalid/oversize export before dense allocation under the same coherent
   occupancy capture. Verify actual encoded size after conversion/serialization;
   account native occupancy, dense native grid, converted and encoded buffers
   separately rather than equating their total to the encoded cap.
6. Healthy accepted heads drain in order; blocked submitter wakes before producer
   join; no-input final completion still wakes publication. Assert final F and
   matching correction/grid without a next sensor event.
7. Retryable head at finish cancels/accounted tail and reports cause. Sticky
   NotCommitted, Unknown and post-COMMIT failures cancel later heads, freeze new
   output selection and never publish unready R. Barrier selected-old output
   against failure/reset; prove allowed in-flight old identity and later fence.
8. Read during healthy drain versus after close/failure; concurrent reset/owner
   replacement; callback lifetime and reentry prohibition; join without needed
   locks. Repeated checked finish and std/nonstd destructor cleanup.
9. New-map one-marginal short flush remains durable, receives coherent final
   committed attribution and publication. Resume equivalent unaccepted partial
   tail is counted/discarded; already accepted short Q drains. Empty, image-only,
   unsynchronized IMU/LiDAR and unmarginalized frontend tails create no fake node.
10. Core no-output, navigation-disabled, empty-new-map and unattached historical
    modes report explicit N/A; they do not fabricate correction or publish during
    A2 construction. Legacy caught visual failure cannot promote false readiness.

## 19. ROS integration and process oracle

Use isolated transport domains and actual publishers on the deployed adapter/RMW.
A test sink must acknowledge observed local-call boundaries independently of
subscriber observations. No subscriber is required for clean local completion.
Test no subscribers, late transient-local subscriber, stalled subscriber,
finite-history saturation, invalid context before/during an attempt, partial
multi-topic failure, checked close error, finish parameter, normal signal,
forced context loss, and publisher destruction after output-thread join. Verify
standard compatibility topics and attributed equivalents contain identical
geometry/cells for the same selected source. End-to-end arrival checks can prove
those test subscribers received selected data; do not promote that observation
to the production contract. New-map and explicit-resume core-injected ROS paths
both need coverage; no auto-attachment UI is implied.

Process oracle:

1. Process A writes supported history and checked-closes it.
2. Process B A2-opens, performs actual LiDAR-verified B1 then B2 Q1/Q2, with at
   least one controlled publication lag/coalescing interval. Stop without another
   input. Record local successful-action witness (U,E,g,R,payload hash), scalar
   final report and the existing semantic oracle values. Normal B3 drain must
   end at exact F and successful close before process exit.
3. Process C independently reopens A2, with CUDA hidden where supported. Compare
   UUID/final DB revision, Nodes/Links/serialized poses, original anchors,
   occupancy and existing spatial/visual semantic oracles. Assert active
   correction unavailable, no old generation/E/publication watermark restored,
   and no mutation caused by read-only reopen. Fresh continuation still needs B1.
4. In failure variants, compare known/Unknown W outcome to independent recovery;
   a published witness may be older than DB, but can never attest to unready
   content. Keep the separate aliased-geometry expected numerical limitation.

A core-only fake sink suffices for revision/coalescing/failure/lock logic; actual
ROS is required to validate observed local results, context lifetime, application
payload budgets, compatibility serialization and process exit. These are future
B3 implementation tests, not a new middleware-internals qualification campaign.
Test watchdogs contain failures and do not promise production deadlines; use event
predicates for correctness. Section 22 retains the completed adapter probe. No
B3 implementation tests or new runtime experiments were run in this correction.

## 20. Performance characterization

Compare identical input, configuration, build and hardware: accepted B2 output
disabled, B3 correction-only, and B3 correction+selected-grid. Vary map extent and
sparsity separately from node count, include historical pose changes, Flat→IVF
transition and unchanged W history-scan costs. Use representative recorded loads
as well as synthetic stress; preserve arrival schedules and cold/warm cache
conditions. No new arbitrary throughput/latency acceptance threshold.

Measure ready→selection, selection→local success and ready→success latency per
group; capture/lifecycle wait, dense getMap, conversion/serialization and publish
call times separately; pending/active slot and byte high-water; coalesced request
count, attempts/failures/retries; mapping service/queue wait slowdown; frontend
join, backend drain, final export, final publish, storage close and whole shutdown
tail latency. Report distributions and maxima with RMW/profile/subscriber setup.
Differentiate work prevented by coalescing from skipped required work (none).

Check structural bounds exactly: one active attempt, one pending map demand,
fixed optional slots, zero queued required work at clean exit. Report area-sized
payload/RMW memory, retained consumer buffers and total RSS separately. Finite
backlog tests are not sustained-overload throughput claims. Transport timings and
middleware RSS are deployment observations, not map-correctness conditions or
hard deadline/total-memory qualifications. Validate Sapphire's own slot, payload
and retry limits without requiring proof of middleware internals.

## 21. Retained limits and explicit exclusions

No Policy-C real-data validation, fixed-K change, aliased-geometry/A2 numerical
support fix, parallel-GICP/OpenMP/TSan investigation, W scan optimization,
Flat→IVF redesign, keyframe/submap selection redesign, robust loop optimizer,
F3/F4 grid-coordinate/ground correction or unrelated retrieval cleanup.
No Place/Region, cloud sync, map merging, distributed replication, multiprocess
server, HA/failover, remote acknowledgment protocol, event log or output journal.
No guarantee every DB revision becomes a message, every consumer is current,
all raw inputs become durable, or final output survives process loss.

The legacy new-map output seam must be verified independently; B2's atomicity
and single-increment proof do not automatically apply there. Existing external
standard-topic consumers cannot extract revision metadata without adopting the
attributed topics. No universal numerical reopenability or total memory/real-time
claim follows. Dense selected reads can stall lifecycle for map-area-dependent
time; uninterruptible solver/storage calls retain their accepted limitations.

## 22. Completed adapter evidence and review disposition

The [completed transport audit](/home/user/code/sapphire_git/audit/2026-09-28-b3-transport-qualification/README.md)
checked whether the current communication layer obviously contradicts Sapphire's
minimal interface contract. It found no such contradiction for the exercised
workloads. Runtime was confirmed as `rmw_zenoh_cpp`; reliable KeepLast semantics
were exercised. No-reader, stalled-reader and disappearing-reader cases returned;
the intended approximately 16 MB grid was exercised, invalid context was detectable
through the checked path, and no indefinite publish block was reproduced.

This is evidence that the current adapter/profile is usable for the intended
workload, not hard real-time qualification or a total middleware-memory bound.
Exact experimental configurations and implementation-specific observations remain
in the historical adapter audit, not in backend/pipeline correctness semantics.
The audit's earlier “not qualified” verdict used the former, stronger full-call/
resource proof criterion; it remains historical evidence. That criterion and its
review gate are superseded by sections 1, 4 and 8 here. No further transport-
internal proof or broad ROS/RMW campaign is required for B3 design review.

Fast DDS, Cyclone DDS and future RMWs are **not currently validated against the B3
transport adapter contract in this phase**. They are not architecturally excluded.
Changing RMW requires adapter/deployment validation of attribution, local result
interpretation, context/lifetime and the Sapphire payload envelope. It does not
redesign C/Y/Pc/Pg, B2/W/A2, map lifecycle, solver, database schema or map identity.
No implementation-specific queue policy, session topology, cache machinery or
resource limit is a map/runtime-ready invariant.

Third-party call liveness and internal resource uncertainty remain documented
platform limitations. A call that does not return may stall graceful publication,
drain or owner replacement. External forced termination retains existing crash
semantics; it cannot be advertised as clean drain. It does not justify unsafe
cancellation, concurrent destruction, fabricated Pc/Pg or changes to C/Y.
Sapphire still guarantees fixed output slots, latest-state coalescing, finite
retry policy, explicit payload budgets and no ACK wait on its side of the boundary.
Existing output-thread reuse, independent correction/navigation frontiers, lazy
grid generation, final-before-context ordering and no publication persistence
remain unchanged.

**B3 design ready for independent review.** This is design readiness, not B3
implementation or validation of every deployment. This correction changes only
transport-boundary wording and directly related application limits in this
document, with a new preservation audit. Production code, message definitions,
accepted foundations and all prior evidence remain unchanged.
