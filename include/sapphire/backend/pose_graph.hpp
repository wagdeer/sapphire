#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <sapphire/mapping/occupancy_grid.hpp>
#include <sapphire/types.hpp>
#include <vector>

namespace sapphire {

/// Loop GICP translation (m) and rotation (rad) convergence.
/// Not registration.gicp.transformation_epsilon or rotation_epsilon.
inline constexpr double kLoopConvergenceEpsilon = 0.01;

/// Numbers the loop registrar already computed. This is not the frontend
/// Registration::align gate.
struct LoopRegistrationGate {
  bool converged = false;
  bool finite = false;
  size_t inliers = 0;
  size_t min_inliers = 0;
  double overlap = 0.0;
  double min_overlap = 0.0;
  double fitness = 0.0;
  double max_fitness = 0.0;
  bool curvature_ok = false;
  double min_eigenvalue = 0.0;
};

/// Nullptr when the loop registration is accepted. The covariance floor is
/// applied by the caller after this returns.
inline const char* rejectLoopRegistration(const LoopRegistrationGate& gate) {
  if (!gate.converged) {
    return "not converged";
  }
  if (!gate.finite) {
    return "non-finite result";
  }
  if (gate.inliers < gate.min_inliers) {
    return "too few inliers";
  }
  if (gate.overlap < gate.min_overlap) {
    return "overlap below minimum";
  }
  if (!std::isfinite(gate.fitness) || gate.fitness > gate.max_fitness) {
    return "fitness above maximum";
  }
  if (!gate.curvature_ok || gate.min_eigenvalue <= 0.0) {
    return "singular Hessian";
  }
  return nullptr;
}

class Pipeline2;
class RevisitExchange;
class LocalPlaneExchange;
struct InertialFrame;

struct PoseGraphStats {
  size_t keyframes = 0;
  size_t optimized_poses = 0;
  size_t loop_candidates = 0;
  size_t loop_registration_failures = 0;
  /// Loop factors dropped because a trial optimization tore a binary edge
  /// past max_constraint_error_sigma, or the trial itself diverged.
  size_t loop_residual_rejections = 0;
  size_t loop_edges = 0;
  double last_loop_fitness = -1.0;
  /// Source points of the latest GICP attempt, after the query window,
  /// body-frame range gate, and source voxel. Zero before any attempt.
  size_t last_loop_source_points = 0;
  double correction_translation = 0.0;
  double correction_rotation = 0.0;
  size_t revision = 0;
  size_t dropped_frames = 0;
  size_t solver_poses = 0;
  size_t solver_factors = 0;
  size_t glc_summaries = 0;
  double isam_odometry_ms = 0.;
};

/// Loop endpoints for visualization. `source` is the query keyframe and
/// `target` is the historical keyframe. The factor itself runs the other
/// way, so its noise stays in the registration tangent.
struct PoseGraphEdge {
  size_t source = 0;
  size_t target = 0;
};

struct PoseGraphSnapshot {
  PoseGraphStats stats;
  std::vector<Isometry3d> optimized_poses;
  std::vector<PoseGraphEdge> loop_edges;
};

/// One odom-frame lidar pose anchored to the keyframe that owned it.
/// After a graph update the map pose is
/// `T_map_i * T_odom_i^{-1} * T_odom`, not the latest rigid correction.
struct MapTrajectorySample {
  double stamp = 0.0;
  /// Index into the keyframe sequence. Samples between keyframes keep
  /// the previous keyframe, so their relative odometry shape is unchanged.
  size_t keyframe = 0;
  Isometry3d T_map_lidar = Isometry3d::Identity();
  Eigen::Vector3d v_map = Eigen::Vector3d::Zero();
};

/// Full replay of recorded lidar poses. `revision` matches PoseGraphStats
/// and advances only when optimized keyframe poses are published.
struct MapTrajectory {
  size_t revision = 0;
  std::vector<MapTrajectorySample> samples;
};

/// Asynchronous ROS-free pose-graph and loop-closure backend.
///
/// The frontend remains entirely in the odom frame. The live pose uses the
/// latest keyframe's rigid map-from-odom correction. Historical lidar poses
/// are replayed from the keyframe that owned each stamp.
class PoseGraphBackend {
 public:
  explicit PoseGraphBackend(const Config::Pgo& config);
  ~PoseGraphBackend();

  PoseGraphBackend(const PoseGraphBackend&) = delete;
  PoseGraphBackend& operator=(const PoseGraphBackend&) = delete;

  bool enabled() const;
  /// After the last sensor callback, seal the open submap and join the worker.
  void finish();

  /// Submit an accepted frontend frame. Keyframe selection is independent
  /// from the frontend submap policy.
  void addFrame(const PointCloudConstPtr& cloud_odom, const Isometry3d& T_odom_lidar, double stamp);

  /// Remember an accepted lidar pose in the odom frame. Call after
  /// addFrame() so the sample anchors to the keyframe that owns it,
  /// including a keyframe this pose just created. Non-keyframes stay on
  /// the previous keyframe. IMU propagation is not recorded.
  void recordOdometrySample(const OdomFrameSample& odom_frame_sample);

  /// Replay recorded samples with each owning keyframe's optimized pose.
  /// A sample whose keyframe is not optimized yet uses T_map_odom(), which
  /// is the same formula with the latest optimized keyframe.
  MapTrajectory mapTrajectory() const;

  /// Thread-safe snapshot of the latest optimized map-from-odom transform.
  Isometry3d T_map_odom() const;

  PoseGraphStats stats() const;

  /// Latest completed visualization snapshot. Pose arrays remain empty
  /// until requestSnapshot() has been serviced by the backend worker.
  PoseGraphSnapshot snapshot() const;

  /// Request optimized poses and loop edges on the backend worker.
  void requestSnapshot();

  /// Request a sparse global map rebuild on the backend worker.
  void requestGlobalMap();

  /// Latest completed sparse map. Empty until one has been requested.
  PointCloudConstPtr latestGlobalMap() const;

  /// Request an occupancy-grid export snapshot on the backend worker.
  void requestOccupancyGrid();

  /// Latest completed occupancy-grid snapshot. Empty until the worker has
  /// serviced a request. Cheap to call: returns a shared_ptr, not a deep copy.
  std::shared_ptr<const OccupancyGridMsg> latestOccupancyGrid() const;

 private:
  friend class OdometryPipeline;
  friend class Pipeline2;
  explicit PoseGraphBackend(const Config& config);
  bool addInertialFrame(InertialFrame frame);
  void setLiveCorrection(const Isometry3d& transform, double stamp, const Eigen::Vector3d& gravity_odom);
  RevisitExchange& revisitExchange();
  LocalPlaneExchange& localPlanes();

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
