# Robust Frontend Iteration Plan

Status: active  
Branch: `feat/robust-frontend`  
Baseline: current worktree on top of `49c2110`, including the pre-existing
`gal3_right` worktree changes

## Objective

Improve frontend completion rate and worst-case robustness before optimizing
mean ATE. The intended end state is a quality-aware, localizability-aware
LiDAR-inertial frontend that can:

1. distinguish a correct registration from a converged but wrong local minimum;
2. preserve useful measurements in partially observable directions;
3. avoid inserting low-confidence scans into the local map;
4. detect sustained tracking degradation and attempt recovery;
5. remain competitive in well-conditioned scenes while reducing divergence and
   partial runs in difficult scenes.

The iteration order deliberately avoids adding more ESKF gain parameters.

## Fixed Stress Set

Every optimization gate uses the same five sequences. They cover distinct
failure modes without turning each gate into a full nightly benchmark.

| ID | Dataset | Sequence | Required coverage | Why it is included |
|---|---|---|---|---|
| S1 | HILTI 2022 | `exp03_construction_stairs` | 17/17 control points | Stairs, close geometry, vertical motion |
| S2 | HILTI 2022 | `exp07_long_corridor` | 6/6 control points | Directional degeneracy and velocity correction |
| S3 | HILTI 2022 | `exp09_cupola` | 16/16 control points | Full coverage but historically high ATE |
| S4 | HILTI 2023 | `site2_handheld_5` | 4/4 control points | Registration was accepted throughout, yet ATE was about 14 m |
| S5 | HILTI 2022 | `exp10_cupola_2` | 14/14 control points | Large cupola; historically partial coverage and a high-error/failure case |

S3E and Newer College are intentionally excluded from the gate set. Their GT
preparation can mask an algorithm regression with time-association or sparse-GT
problems. They remain part of the later full-suite validation.

The five-case set is a smoke gate, not a replacement for the full benchmark.
A gate that passes here must later be checked on the complete suite before it is
advertised as robust across all datasets.

## Invariants

- One optimization gate per commit.
- A gate may contain multiple small code changes only when they implement one
  testable mechanism.
- Every gate starts from the previous committed gate.
- No gate may mix unrelated benchmark-document edits or pre-existing worktree
  changes into its commit.
- Every gate runs all five stress cases unless a gate cannot be built or started.
  Infrastructure failures may be retried once; numerical failures remain in the
  report.
- Full bag playback is required. A partial prefix may be reported as a failure
  but cannot be used to rank a gate.
- Reports preserve both full-run metrics and coverage. Sparse GT must not reduce
  the estimated trajectory used for plots.
- The default production profile is not changed until the gate passes and the
  new behavior is available as an explicit configuration mode.

## Metrics

For each case record:

- completion status and whether the bag reached EOF;
- trajectory pose count and strictly increasing timestamps;
- control-point coverage or GT association coverage;
- ATE RMSE, mean, median, p95, and maximum;
- estimated and reference path lengths;
- registration inlier ratio;
- nearest-neighbor fitness / mean residual;
- rotational and translational Hessian condition indicators;
- number and longest run of rejected corrections;
- count of zero-inlier scans;
- map-update count;
- final position/velocity/acceleration sanity checks.

Ordering for regression decisions:

1. Completion and full input consumption.
2. Coverage and zero-inlier/rejection behavior.
3. p95 and maximum error.
4. Mean and median ATE.

A gate that improves mean ATE while losing a sequence, reducing coverage, or
creating a long rejection deadlock is rejected.

## Gate Sequence

### Gate 0: Baseline and observability

Purpose: establish the five-case baseline and add diagnostics without changing
the estimator decision.

Changes:

- extend registration results with inlier ratio, fitness, overlap where
  available, and separate rotation/translation Hessian indicators;
- log a structured per-scan quality record;
- count consecutive registration failures and map-update attempts;
- keep the acceptance decision and state update behavior unchanged.

Pass condition: builds cleanly, diagnostics are finite, and no stress case
changes algorithmic behavior relative to the baseline fingerprint.

### Gate 1: Quality-aware soft fusion

Purpose: stop treating registration as a binary event.

Changes:

- classify registrations as excellent, partial, degenerate, or invalid;
- inflate measurement covariance according to quality;
- prohibit map insertion for invalid updates;
- preserve finite state and covariance through rejected scans;
- introduce consecutive-failure tracking without changing normal high-quality
  updates.

Pass condition: no case loses coverage, rejection deadlocks do not grow, and at
least one hard sequence improves p95 or maximum error without a material
regression in the other four.

### Gate 2: Adaptive data association and robust residual weighting

Purpose: make correspondence gating depend on motion uncertainty, map
discretization, and range instead of one fixed distance.

Changes:

- compute an adaptive correspondence threshold from predicted pose uncertainty,
  voxel resolution, and LiDAR range noise;
- add conservative robust weighting if it is supported by the registration
  backend;
- keep hard correction limits as a final safety check, not as the sole quality
  model.

Pass condition: full completion on all five cases, fewer rejected runs, and no
material ATE regression in well-constrained cases.

### Gate 3: Hybrid or localizability-aware registration backend

Purpose: address the failure mode where one residual metric is unsuitable for
the current local geometry.

Implementation candidates, evaluated behind the existing `Registration`
interface:

- a point-to-plane plus point-to-point hybrid backend;
- an MIT-licensed GenZ-ICP integration;
- a custom localizability-aware residual model.

GPL-2.0 GenZ-LIO source is not copied into the MIT-licensed Sapphire tree. Its
paper and architecture may inform an independent implementation.

Pass condition: better completion or error on at least two structurally
different hard cases without weakening the other three.

### Gate 4: Scale-aware preprocessing and map resolution

Purpose: remove fixed voxel-size dependence across narrow and open scenes.

Changes:

- estimate scene scale from robust range statistics;
- regulate estimation and map-insertion resolutions independently;
- record selected resolution and point counts per scan.

Pass condition: improves the weak-geometry/scale-transition cases while keeping
runtime within the input frame budget.

### Gate 5: Recovery and degraded-input handling

Purpose: recover from sustained tracking loss instead of dead-reckoning
indefinitely.

Changes:

- explicit healthy/degraded/lost state;
- bounded dead-reckoning with covariance growth;
- wider-window or multi-hypothesis reacquisition;
- no map insertion until reacquisition is confirmed;
- initialization fallback for a moving start or insufficient static samples.

Pass condition: every stress case either completes without entering `lost`, or
enters `lost` and recovers to a valid trajectory; silent unbounded drift is not
acceptable.

## Git Workflow

For each gate:

1. implement only that gate;
2. run focused unit/integration tests;
3. build both ROS wrappers;
4. run the five stress cases;
5. evaluate with dataset-specific metric contracts;
6. write `reports/gate-N-<name>.md`;
7. inspect the diff and commit only files belonging to that gate.

Suggested commits:

```text
docs(robust-frontend): define iterative robustness gates
feat(registration): add quality diagnostics
feat(registration): rank corrections by confidence
feat(registration): adapt correspondence gating
feat(registration): add hybrid residual backend
feat(frontend): adapt scan and map resolution
feat(frontend): recover from sustained tracking loss
```

## Report Template

Each gate report contains:

- commit and dirty-worktree fingerprint;
- changed behavior and non-goals;
- build/test commands and results;
- one row per stress sequence with status and coverage;
- an ATE comparison table against the previous gate;
- failure and log evidence for every rejected gate;
- decision: accept, revise, or revert;
- limitations and next gate.

## Baseline

The pre-change baseline for this branch is the current worktree on top of
`49c2110`. That worktree already contains uncommitted `gal3_right` and benchmark
changes. Gate reports record its dirty fingerprint rather than silently treating
it as clean `main`.

