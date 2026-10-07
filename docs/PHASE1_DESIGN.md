# Phase 1 approved constraints and A1 implementation boundary

The design review authorizes **A1 only**. A2 and later require separate approval.
Strengthen existing invariants and ownership before adding types. No new Snapshot
type, aggregate publication state-holder, generic backend layer, or public ROS message.

## A1: safe open and canonical Node identity

Core configuration accepts `map.mode` (`new` by default, `resume`, `localize`) and
`map.database_path`. An explicit path is used exactly; its parent must already
exist. An empty path in `new` retains the timestamp path. `resume` requires a path.
Pipeline output directory creation never clears an existing directory.

A1.1 path contract: make the request absolute without lexical simplification,
then use filesystem `canonical()` resolution of its **existing parent** only.
This follows directory symlinks before processing `..` and fails on a missing
intermediate component even when followed by `..`. The final component is kept
unresolved: `new` uses exclusive creation and `resume` rejects final-component
symlinks with `O_NOFOLLOW`. `path()` reports the resolved parent plus that final
component. `open`, the new-writer SQLite filename, sidecar checks and later inode
checks all use this same effective path; `flock` owns the opened inode. A1.2
resume SQL inspection consumes that descriptor's mapped bytes, never a second
filesystem pathname open. Relative
requests are anchored to the working directory at open time. Parent resolution
is not a lease on the directory tree; the cooperative ownership contract below
still applies to external namespace mutation.

`MapDatabase(path, "new", config_identity)` exclusively creates a regular file
using POSIX `O_CREAT|O_EXCL`. It never adopts, truncates, migrates, or reopens an
existing destination. SQLite receives READWRITE, without CREATE, after exclusive
file creation. A failed initialization may leave its newly created destination;
it never deletes an existing file to make retry succeed.

`MapDatabase(path, "resume", config_identity)` is **eligibility validation only**:
it validates schema version, required tables/columns, map identity, nonnegative
revision, and the required configuration identity. It constructs no Memory,
ISAM2, spatial/Faiss index, cache worker, occupancy or frontend session. Its write
methods reject with `ReadOnly`. Record inspection through existing database
read methods remains available. This is not complete graph/payload validation.

Backend/pipeline `resume` validates, then throws `ResumeUnavailable` explicitly;
neither restoration nor submap continuation starts. `localize` throws
`UnsupportedMode` without touching a destination, including when PGO is disabled.

Preflight takes a shared nonblocking advisory lock on the existing regular file.
Its descriptor open uses `O_RDONLY|O_NONBLOCK|O_NOFOLLOW|O_CLOEXEC`, followed by
`fstat` of that descriptor before locking or SQLite inspection. A FIFO with no
writer therefore cannot block, and replacing a target between a preliminary
type check and open cannot bypass a check: there is no preliminary stat/open
sequence. Directories and other non-regular descriptors return `Storage`;
unopenable special targets also return `Storage`. The existing subsequent
path/inode checks remain in force. This is the cooperative local-file contract,
not a hostile-filesystem or arbitrary-device-driver availability guarantee.
It requires a closed/checkpointed map: existing `-wal`, `-shm`, or `-journal`
files are rejected without checkpoint/recovery. A1.2 maps the held regular-file
descriptor with `MAP_PRIVATE`, after validating a size of at least 100 bytes and
representability in both `size_t` and `sqlite3_int64`. Mapping/protection failures
are explicit storage errors. This reserves file-sized virtual address space; it
does not duplicate the entire file in resident heap memory.

The view must have the exact SQLite magic and either header version pair (1,1)
or (2,2) at bytes 18/19. Any other pair is rejected as unsupported/malformed.
For a clean WAL-header file only, the pair (2,2) becomes (1,1) **in the private
view**, following SQLite's documented deserialize requirement. Canonical bytes
remain unchanged. `mprotect(PROT_READ)` then protects the view.

SQLite opens `:memory:` and consumes this view with exactly
`SQLITE_DESERIALIZE_READONLY`; neither FREEONCLOSE nor RESIZEABLE is used.
MapDatabase owns the mapping. Its existing writable flag still rejects public
writes; tests also execute SQL mutations against the actual eligibility
connection and require SQLITE_READONLY. `sqlite3_db_readonly()` is not the
enforcement test for a deserialized memory database. Eligibility cannot create
database sidecars, recover journals, or persist settings. Read-only files and
directories remain supported.

All statements are local RAII objects and none escape MapDatabase. On normal
destruction and every constructor failure they finalize before cleanup closes
SQLite, unmaps its external buffer, and finally closes the locked descriptor.
The configure step explicitly checks that the linked SQLite exposes
`sqlite3_deserialize` (verified with 3.37.2). This path uses the existing Linux
filesystem/mapping contract and adds no custom VFS or new storage owner.

This fixes the independently reproduced **resume** pathname-reopen race: after
validation, rename/unlink/FIFO/regular-file/symlink replacement cannot redirect
SQLite away from the held inode. Final path validation still rejects such a
replacement with `StaleMap`. `/proc/self/fd` is not used as a SQLite filename:
the current Unix VFS can resolve it back to the mutable pathname. Writable new
creation is unchanged; this is not a general adversarial-filesystem guarantee.
This intentionally does not implement inspection/recovery of live or interrupted
WAL maps; sidecar presence is an unsupported state here, not evidence of corruption.
External programs must respect Sapphire's exclusive writer ownership;
the advisory lock is not a security boundary against arbitrary file mutation.
In-place modification or truncation of a held inode remains unsupported;
external truncation can invalidate mapped access (including Linux SIGBUS).
There are no signal handlers, monitoring, leases or recovery mechanisms here.

New writers hold a nonblocking exclusive `flock` on the created inode. Before a
write, and again inside its short SQLite transaction, they validate path/inode,
format, UUID, configuration identity and expected revision. SQLite write-lock
contention is a visible conflict. No writable resume/promotion exists in A1.
Stale writers cannot persist changes or change journal settings on destruction.

Canonical Node insertion is plain INSERT. An existing ID yields `DuplicateNode`
and rolls back before changing historical payloads, links, poses or revision.
This does not make every database record immutable or redesign Link updates.

`MapError` extends runtime_error only to expose the small set of testable storage
failure codes: DestinationExists, MissingMap, UnsupportedFormat,
IncompatibleConfig, MalformedMetadata, UnsupportedMode, DuplicateNode,
WriterConflict, StaleMap, ReadOnly, ResumeUnavailable and Storage.

## Metadata and stage distinction

A1 writes `user_version=1` and adds `MapState.map_uuid` (UUID v4) and
`MapState.config_identity`. A1.1 replaces the overly broad `a1:` compatibility
encoding with `c2:` plus 16 lowercase FNV-1a-64 hex digits. Schema layout remains
version 1. The input starts with `sapphire-map-config-2`, then exactly these
`NaviMapParameters` fields, in order, even when grid output is disabled:

1. `resolution`: occupancy cell discretization of stored metric grid evidence.
2. `occ_threshold`: interpretation of reconstructed log odds as occupied/free.
3. `hit_probability`: log-odds contribution of stored occupied evidence.
4. `miss_probability`: log-odds contribution of stored free/ground evidence.
5. `clamp_min_probability`: lower bound during historical evidence fusion.
6. `clamp_max_probability`: upper bound during historical evidence fusion.

All six are historical occupancy reconstruction semantics (category 4), used
by `OccupancyGrid` in `mapping/grid/occ_layer.hpp`. They are not recoverable from
the stored LocalGrid labels alone; changing them can change the reconstruction
of the *same* committed evidence. Disabled output does not waive this contract.
The encoding uses semicolon delimiters, classic locale, default floating-point
format with double `max_digits10`, fixed ordering and no environment values.
Default test vector: `c2:9477d0eb7e4b7acd`. An `a1:` identity is recognized as an
old encoding but mismatches the current identity (`IncompatibleConfig`); no
migration or metadata rewrite is attempted. It is not an authentication hash.

The complete audit of **previously included** fields is:

| Fields | Current source role / rationale | A1.1 decision |
|---|---|---|
| `submap_voxel_size` | Future Gaussian aggregation in SubmapFrameBuffer; committed Gaussians store metric means/covariances, and saved PyramidVoxel includes its own minimum resolution, levels and buckets | Exclude; no historical decoding or coordinate dependency |
| `submap_travel_distance`, `submap_max_point_range` | Future freezing and input selection; historical poses, geometry and bounds are already stored | Exclude; future construction policy |
| `navi_map.enabled` | Selects runtime grid processing/output; stored LocalGrid remains interpretable independently | Exclude; feature switch |
| `adaptive_ground`, `ground_plane_tolerance`, `ground_max_correction`, `h_clearance`, `ground_margin`, `usable_range`, `min_range` | Ground estimation and generation/classification of future LocalGrid; stored ground/obstacle/free labels are loaded, not regenerated under these thresholds | Exclude; future observation policy, not stored-grid fusion semantics |
| `clear_height_eps`, `margin` | No current production consumer | Exclude; inert |
| `d_max` | Logging only in PoseGraph initialization | Exclude; logging |
| `resolution`, `occ_threshold`, `hit_probability`, `miss_probability`, `clamp_min_probability`, `clamp_max_probability` | Occupancy reconstruction described above | Include, unconditionally |
| `visual.enabled`, `visual.mode` | Future image-stream/retrieval selection; persistent camera IDs, descriptors and scenes decode independently | Exclude; feature/stream switches |
| `visual.global_xy_window`, `visual.global_z_window`, `visual.top_k`, `visual.min_matches` | Future candidate ranking, acceptance and BBS search extent | Exclude; search policy |
| `visual.image_interval`, `visual.max_features`, `visual.max_frames` | Future image sampling/ORB count/submap retention; committed descriptor matrices and scenes contain their own sizes/content | Exclude; future sampling |
| Both cameras' `width`, `height`, `input_width`, `input_height`, `intrinsics`, `distortion` | Future input scaling, feature extraction, undistortion and bounds filtering; persisted pixels/descriptors load directly, and historical retrieval uses appearance-only scenes | Exclude; no current metric reprojection of historical image records |
| Both cameras' `time_offset`, `camera_to_imu_rotation`, `camera_to_imu_translation` | Time offset selects future image timing; extrinsics do not participate in current historical appearance-only retrieval or metric LiDAR factors | Exclude; no historical frame reinterpretation in the supported implementation |

None of the removed fields supplies serialization layout, committed metric
frame interpretation or graph-noise/prior semantics (categories 1–3), nor another
current historical dependency (category 5). Those contracts depend on stored
records and the supported implementation/schema, not these policies. This is
not permission to recompute stored LocalGrid from clouds under new thresholds,
or reinterpret historical pixels as metric camera rays using new calibration.
A2 must load stored evidence; ground-reference recovery still needs its separate
approved decision. A later metric visual capability or graph-format change must
revisit compatibility explicitly. Path/mode, cache budgets and scheduling remain
excluded. No complete configuration or duplicate pose/factor storage is added.

Some existing low-level SQLite record read/write failures still throw plain
`std::runtime_error`. A1.1 does not broaden the error framework; constructor
path/type failures continue to use the existing typed `MapError` contract.

Schema 1 establishes A1 storage eligibility, **not a guarantee of graph recovery**.
Ordinary mapping still uses the existing submap and pose/link transactions.
`graph_revision` still advances on pose/link transactions; A1 does not claim the
later T1/T2 contract is implemented. No completion marker or pending recovery is
implemented yet. Future schema changes and compatibility must be explicit.

## Approved later work, not implemented here

Sequence: A2 historical reconstruction without new mapping; A3 equivalence and
no-persistent-mutation verification; A4 explicit LiDAR-verified fresh-odom-session
attachment; A5 independent-process continuation. Map resume never means ESKF,
IMU bias or active sliding-window restoration. No cross-session odometry edge.

Use short commits only. T1 commits base submap, prior/odom/attachment state,
poses, `Node.loop_decision_complete=0` if needed, and increments the existing
graph_revision once. Retrieval/prefetch waits/BBS/GICP/optimization happen outside
the database transaction and mutex. T2 commits the optional loop and poses,
completion=1 (also for a successful no-loop result), and one revision increment.
Errors are not no-loop results. No global cursor or pending-work table.

At most one pending node, at the highest ID, is a Phase-1 processing constraint,
not a permanent restriction of the map format. An unsupported processing state
must be reported as such. Completed decisions are not rerun; pending decisions
are completed only during explicit writable continuation, not read-only A2.

(map_uuid, graph_revision) will identify committed map state, not node count,
fully processed input count, optimization calls, or publication count. A pending
T1 is coherent committed state. Reconstruction/index/cache/grid/output work and
idempotent retries do not advance revision. Correction, poses and grid for R
must derive from R through existing objects/output paths, with safe lifetimes.
Later drain covers accepted frozen submaps, completion of their loop decisions,
and final local output execution without another input. No remote acknowledgement.

F2, F3, F4, F5, registration/visual/ground/grid algorithms, Faiss tuning,
FeatureBlock cleanup and frontend state serialization remain outside A1.
