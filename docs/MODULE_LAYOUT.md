# Sapphire modules

The core is organized into four implementation modules. Existing class names,
namespace identities and runtime owners are preserved.

| Module | Responsibilities | CMake target |
|---|---|---|
| frontend | initialize / eskf / ricp / common: initialization, ESKF, optional GICP observation and reusable sensor processing | sapphire::frontend |
| backend | Visual keyframe/image preparation, submap registration, graph optimization, scene storage, retrieval, PnP, ground/grid outputs | sapphire::backend |
| common | Shared measurements/state/submap observations and camera geometry | sapphire::common |
| tools | Reusable low-level utilities | sapphire::tools |

```
sapphire/src/
  frontend/
    initialize/
      initialize.hpp
    eskf/
      ... current estimator dependencies ...
    ricp/
      ricp.{hpp,cpp}  # experimental VGICP correction, selected by configuration
    common/
      imu_estimator.hpp
      imu_factor.hpp
      lidar_preprocess.hpp
      synchronizer.{hpp,cpp}
  backend/
    graph/
    registration/
    storage/
    grid/
    visual/
      feature/   # feature blocks, detection/tracking, keyframes and scene features
      faiss/     # binary descriptor index and Faiss thread policy
      utils/     # image helpers
      solver/    # stereo geometry and PnP
      visual_loop.{hpp,cpp}  # integration/orchestration
  common/
    camera/
      calibration.{hpp,cpp}
      camera.{hpp,cpp}
      pinhole.{hpp,cpp}
      kb4.{hpp,cpp}
      ucm.hpp
    ... shared data types ...
  tools/
  pipeline.{hpp,cpp}
  parameters.{h,cpp}
```

Frontend and backend independently depend on common/tools. Common can use tools;
tools depends on neither business modules nor system configuration. The `sapphire::core` target
is the existing pipeline composition entry point and links the implementation modules. It is not another
algorithm module. No module includes pipeline.hpp. Project headers use qualified
paths relative to src; old mapping/odometry and flat include aliases are removed.
The root configuration header is included as parameters.h.
ROS and tests are updated to use these paths. External dependencies stay in
thirdparty; the existing common submap representation still refers to BBS payload
types, without depending on the backend implementation.

Synchronizer method bodies move unchanged out of pipeline. The legacy
extractLoopFeatures routine moves unchanged out of backend visual_loop into
backend visual_features, separating image preparation from retrieval.
Visual selection remains executed on the existing mapping worker;
code ownership and thread placement are separate decisions.

The existing SlamPipeline still owns estimation state, worker startup, marginal
queues, reset/continuation coordination and submap submission. This reorganization
does not implement the proposed independent robust-odometry/mapping estimator split.
Frontend/common includes reusable IMU propagation/deskew, IMU preintegration,
point preprocessing and IMU/LiDAR synchronization. It may not include initialize
or eskf implementations. Top-level common holds shared observations/types. System configuration parameters.h/.cpp is beside pipeline at the source
root; the parameter parser is compiled by sapphire_common without depending on
the pipeline implementation. PointUncertainty has been merged into pointVar::projectVariance();
rotation/covariance and output Jacobian remain caller-owned. pointVar has no new
storage fields, and the right-attitude/world-position error convention is explicit.

The ROS LiDAR adapter validates and decodes PointCloud2 layout/fields/endianness;
frontend/common/lidar_preprocess.hpp applies the shared numeric timing, decimation,
finite-point and blind-distance policy without storing or copying another cloud.
Dual-LiDAR message-queue fusion remains in the ROS adapter because it owns ROS
messages; the core Synchronizer performs estimator-independent IMU/LiDAR pairing.
No new queue policy, lock ordering, coordinate definition, persisted schema or
closed A1-B3/A2/F1 policy is introduced.

Build the whole system through sapphire::core, or link only sapphire::frontend /
sapphire::backend for module consumers. Tests include independently linked frontend
and backend algorithms and an include-boundary guard. Headers and all five static
archives are installed; this project does not yet export a standalone CMake package
with all bundled third-party dependencies.

Verification evidence: [module split audit](../../audit/2026-10-07-module-split/DESIGN.md) and
[layout/tools/camera refinement](../../audit/2026-10-07-parameters-placement/README.md).

Generic tools are limited to SIMD wrappers (kept complete, including currently
unused operations), the parallel executor, LRU cache and timer.hpp. Profiling
uses a steady clock and no backend types; SAPPHIRE_PROFILE=1 enables its callers,
with SAPPHIRE_PROFILE_BACKEND retained as a fallback for existing scripts.
Explicit SAPPHIRE_PROFILE takes precedence. Timing labels remain at call sites.
Image processing and feature/descriptor storage belong to backend/visual; the
single-user keypoint grid is private to scene_features.cpp. Unused require macros
and IO helpers are removed; timestamp filename formatting is private to pipeline.

Camera geometry lives in common/camera/{ucm,kb4,pinhole}.hpp with shared float
calibration storage in calibration.hpp. common/camera/camera.hpp dispatches configured
radtan/equidistant streams to their concrete models; UCM remains an explicit model.
Projection, inverse projection, bearings, intrinsic matrices and extrinsics belong
to the models. The local VSLAM Perspective design informed cached inverse focal
lengths and per-model geometry ownership; its camera database is not imported.
Internal model storage/math uses float; existing double LIO/stereo interfaces use
explicit conversions. Frozen image projection records use the float intrinsics
actually used to produce the pixels. Config/schema representations are unchanged.
The old common/vision directory and standalone visual-camera helpers are removed.
Tracking retains one camera model, avoiding construction in per-feature loops.
Stereo pair rectification/triangulation remains in backend/visual.

The visual subdirectory split changes paths only. Feature blocks remain independent
of a specific retrieval engine: raw ORB and legacy vocabulary-compatible records
share feature/. No FBoW implementation exists, so no empty fbow directory is
retained. External Faiss remains thirdparty. Verification is recorded
in [visual layout audit](../../audit/2026-10-07-visual-layout/README.md).

CameraCalibration is an implementation base owned as part of each concrete camera.
It is not an external calibration manager. Constructing CameraModel(params) copies
the float calibration into the selected model; callers keep using the camera's
projection, matrix, distortion and extrinsic methods. Subsequent changes to the
input config do not alter that camera (covered by the camera ownership test).
