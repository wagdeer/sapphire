# Hilti 2022 acceptance — updated 2026-10-07

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


## Historical direct stereo profile — 2026-10-07

Use `sapphire/config/hilti2022_exp09.toml` with
`sapphire_ros2/config/hilti2022_exp09_ros.yaml`: 720×540 raw input, 360×270
processing, <=500 features per camera, direct sparse stereo and no temporal optical
flow. Depth is gated by calibrated disparity and match quality, not a fixed metric
range. [Implementation, verification and current replay](../../audit/2026-10-07-stereo-depth/README.md).
Final optimized full1x run:4464odom,849stereo pairs, no Reset/refusal or >1ms
marginal wait; stereo p50/p95/max4.17/9.02/98.37ms, CPU57.9%, RSS1135.5MiB.
Checked close/reopen pass. PnP attempts0 and committed RMSE0.313m versus online0.180m:
real visual-loop/backend geometry qualification remains failed. Controlled module
A/B improves about13%; full-system CPU does not show a matching improvement.
The October1 results below describe a different visual path and earlier frontend.

## Historical exp09 result — 2026-10-01

Data arrived at `/home/user/data/hilti2022_exp09/`. Two full 1x runs with official
Hesai/Alphasense/cam0-1 calibration are complete. Engineering completion, storage
integrity and independent reopen pass. That cam0/1 backend geometry **fails**:
identical online odometry scores 0.179m sparse-position RMSE, versus committed
layouts 0.209m (LIO) and 1.108m (cam0/1). PnP attempts are zero despite some appearance
recall. No scene retires; no real automatic cross-session claim is established.
[Report, runnable configurations, raw evidence and limitations](../../audit/2026-10-01-hilti-exp09/README.md).

The remainder is the dated **2026-09-30 preparation record**, retained for rationale;
its pending-download/input statements are superseded by the actual result above.

## Preparation context (historical)

The user is downloading Hilti 2022 and will provide its path and ground truth.
They confirmed that the current 0519 recording's sensor layout is unsuitable for
algorithm assessment. Stop optimizing geometry/appearance settings against that
recording. Keep prior measurements as engineering workload diagnostics only:
queue pressure, CPU/RSS, bounded local state, checked finish and storage integrity.
They do not qualify geometric accuracy, depth quality, tuning superiority or
cross-session effectiveness. No more old-calibration investigation is required.

Work that does not depend on downloaded measurements continues. Explicit camera
model math is now implemented and synthetically checked. A small
ROS1→ROS2 conversion fixture passes; actual recording compatibility still needs
the downloaded messages. No real Hilti accuracy result is available.

## Verified source material

The [official dataset page](https://hilti-challenge.com/dataset-2022) identifies
five camera streams, a Hesai PointCloud2 stream and IMU; reference trajectories
use the IMU frame. Most truth files are sparse positions; selected sequences also
have dense six-degree-of-freedom trajectories. Evaluate according to each truth
file's actual support, without treating sparse positions as continuous pose truth.

The official [cam0/cam1 calibration](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/blob/main/calibration/calibration_files/calib_3_cam0-1-camchain-imucam.yaml)
specifies 720×540 pinhole cameras with equidistant distortion. T_cam_imu must be
inverted for Sapphire's camera-to-IMU convention. The [Kalibr convention](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) is
`t_imu = t_cam + timeshift_cam_imu`; Sapphire adds `time_offset` with that sign.
Actual recording stamp semantics still need verification. Do not reinterpret four
fisheye coefficients as ordinary radial/tangential distortion.

The [LiDAR calibration](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/blob/main/calibration/calibration_files/lidar_calibration.yaml)
provides sensor-parent transforms, with IMU as the base. Verify quaternion ordering
and transform direction against the official frame model before emitting a runnable
profile. No hardcoded adaptation from another dataset is accepted.

Downloaded YAMLs, exact source URLs/SHA256s and a read-only transform inspection are
in [the preparation packet](../../audit/2026-09-30-hilti-preparation/README.md).

## Current code gaps and next implementation

- CameraParameters now explicitly selects `radtan` or `equidistant`. Shared native
  normalization/projection is wired into tracker and legacy extractor. Synthetic
  roundtrip, off-axis, invalid-parameter, triangulation and actual image-flow tests
  pass. PnP uses rectified pinhole coordinates without double undistortion. Existing
  positive-z/rectified-image bounds can exclude wide-angle boundary pixels; full
  FOV retention and real feature quality remain unverified.
- ROS/TOML now explicitly select single `hesai` with FLOAT64 absolute-second
  `timestamp`. All raw point times determine the full scan bounds before filtering;
  malformed or constant times reject. Shared dual checks, synthetic decoding/core
  synchronization, launch configuration and live ROS callback/exit smoke pass.
  Inspect the actual recording's field layout, scan times and IMU clock before
  declaring Hilti compatibility. A calibrated runnable profile remains pending.
  [Implementation and evidence](../../audit/2026-09-30-hesai-input/README.md).
- The source recordings are ROS1. The installed `rosbags-convert` tool passes a
  six-message synthetic preservation
  check, and ROS2 Humble reads its metadata version 8 output. Keep original stamps
  and point payloads, use a new destination, and verify actual scan layout/epochs.
  Large-bag memory behavior is not yet measured; source recordings stay unchanged.
- First establish single-camera plus LiDAR/IMU operation, then the existing two
  camera slots if useful. Five-camera support is not currently implemented and
  must not be inferred from the dataset's sensor count. Vision still does not
  update odometry or obtain depths from LiDAR-feature association.

## Acceptance once data arrives

Inspect one actual recording and its calibration before selecting a runnable
configuration. Verify input counts/timestamp conventions, then baseline LIO and
visual tracking/depth/initialization independently. Compare against supplied truth
in the correct IMU frame, with declared alignment and timestamp association.
Measure completion, CPU/RSS/queues and candidate/initialization/verification stages
separately from trajectory accuracy. Revisit and restricted cross-session runs must
add evidence for scene updates, duplicate growth and automatic association.

Metric visual observations now persist with original camera poses and pass synthetic
cross-process B1/B2 continuation/reopen checks; [archive evidence](../../audit/2026-09-30-metric-archive/README.md).
Online ordinary-loop PnP routing now uses frozen query projection and independent
one-pose LiDAR scoring/GICP. Real-data geometry/loop quality, repeated-scene
replacement and automatic cross-session association remain open. None of this reopens A2/F1.

Implementation, synthetic results and conversion reproduction: [equidistant packet](../../audit/2026-09-30-equidistant/README.md).
