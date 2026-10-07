# Backend failure semantics

Engineering contract, 2026-09-28. Applies to bounded changes in the existing
Sapphire owners. It does not claim every older path already satisfies it.
Source anchors: `PoseGraphBackend::Impl::{recordFailure,finish,attachFreshSession}`,
`MapDatabase::{commitFinalizedSubmap,finish,closeHandles}`, and pipeline owner
replacement. A1/A2/W/B1 and the independently accepted narrow teardown correction
are the foundation; see [CURRENT_STATE](CURRENT_STATE.md) and the
[teardown acceptance](../../audit/2026-09-28-teardown-acceptance/README.md).

## Invariant

**Global invariant, local implementation.**

**If state is proven unchanged, failure may remain retryable.**

**Once uncertain or non-rollbackable runtime/persistent mutation begins, failure
becomes sticky and the backend must fail-stop rather than continue from uncertain
state.**

Committed DB facts are authoritative. Solver state, original-anchor lookups,
pose/spatial indexes, visual archive, occupancy, correction and readiness are
derived/disposable runtime machinery. Reconstruct them from the authoritative DB
before reuse after uncertain failure. A successful SQL rollback does not undo an
ISAM mutation. A failed runtime update does not undo a successful SQL COMMIT.

## Local response sequence

```text
Detect
→ Classify
→ Fence
→ Cleanup
→ Report
→ Reconstruct before reuse
```

1. **Detect:** check results and catch exceptions at the actual owner boundary,
   including entry into the first potentially mutating call. A flag set only
   after that call returns is too late.
2. **Classify:** establish what remains proven correct: previous DB revision,
   newly committed revision, unchanged solver, or uncertain state. Preserve the
   first failure and known commit outcome. Rejected registration is a normal
   decision; an exception is not evidence of a completed no-loop decision.
3. **Fence:** retain Failed; reject admission and inconsistent read/correction
   services. Never process Qn+1 after Qn fails with uncertain state. Wake all
   blocked producers/consumers on failure and shutdown. Wait predicates must
   include these conditions; notification alone is insufficient.
4. **Cleanup:** stop/join owned work, settle owned asynchronous lifetimes, then
   close dependencies in order. Release SQLite completely before its external
   mapping/owner descriptor; release descriptor admission last. A close failure
   retains ownership and prevents replacement. Do not repair uncertainty by
   silently rebuilding and continuing inside the failed owner.
5. **Report:** explicit checked `finish()` reports retained processing and storage
   failure after safe cleanup; quiet logs or thread exit are not success. Final
   destructor cleanup must be noexcept-safe for standard and non-standard
   exceptions and must not erase failure or falsely report rollback.
6. **Reconstruct before reuse:** close the old owner, use W's explicit recovery
   entry where sidecars require it, then reconstruct DB facts through A2. A new
   frontend odometry domain requires B1; reconstruction does not revive it.

Pre-mutation input checks, registration rejection and local graph/Values
allocation may remain retryable when solver, DB and committed runtime are proven
unchanged. Disposable cache reads are not committed facts, but a partial index
mutation must not be dismissed as a harmless cache miss. Retry the same ordered
head explicitly; do not skip it or consume a persistent identity.

## W outcome is separate from runtime health

| Outcome | Meaning and required handling |
|---|---|
| `NotCommitted` | Previous DB revision remains authoritative. Runtime can still be sticky Failed because tentative solver mutation already began. |
| `Committed` | New Node/revision is authoritative, including when later materialization or storage finalization fails. Runtime-ready remains behind until every required owner is coherent. |
| `Unknown` | Stop assigning IDs/factors. No blind retry or inferred rollback. Close/recover/reopen and resolve from authoritative DB facts before any new mapping. |

Runtime-ready and correction must never claim a partially materialized revision.
These invariants require no extra DB graph revision for cleanup, index work,
correction or publication. WAL/NORMAL retains W's existing durability limits;
this contract does not add a power-loss guarantee.

## Exception and termination boundaries

Accidental termination from an arbitrary exception escaping a thread entry or
destructor is a defect when a concrete path demonstrates it. The accepted
teardown correction adds the local destructor catch-all; it does not prescribe
mechanical catch-all edits throughout algorithms. Each new worker path must
contain its own escaping exceptions and preserve retryable/sticky classification.

Deliberate invariant-failure termination is distinct: MapDatabase's noexcept
cleanup may deliberately terminate if SQLite cannot completely close and safe
ownership release is impossible. Do not mask that invariant failure, release
admission early, or describe it as the repaired accidental destructor escape.

## Scope discipline

These semantics do **not** justify a FailureManager, LifecycleManager,
RecoveryManager, ShutdownCoordinator, RuntimeSnapshot, TransactionContext,
generalized rollback infrastructure, or generalized exception/event framework.
Use existing owners, their locks, flags, `exception_ptr`, error codes and checked
operations. Do not refactor current code merely to make it aesthetically conform
to this document. This records invariants for future bounded implementations;
B3 publication/drain remains a separate phase.
