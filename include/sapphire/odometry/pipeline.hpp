#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/types.hpp>

namespace sapphire {

/// LiDAR-inertial odometry with an asynchronous mapping backend.
class OdometryPipeline {
 public:
  explicit OdometryPipeline(const Config& config);
  ~OdometryPipeline();

  /// Whether IMU initialization has completed.
  bool initialized() const;

  /// Access the IMU initialization result (valid after initialized()).
  /// Throws std::logic_error if called before initialization completes.
  const ImuInitResult& imuInitResult() const;

  /// Wall-clock budget for a later scan to wait until buffered IMU covers
  /// its newest remaining point time. Sensor time and ROS time do not
  /// extend this wait.
  static constexpr std::chrono::milliseconds kImuCoverageWait{500};

  /// Feed a scan with point timestamps on the IMU clock. Wait up to
  /// kImuCoverageWait for the IMU to reach the scan end.
  void pushLidar(double stamp, const PointCloudConstPtr& points);

  /// Feed raw IMU data and update the IMU-rate propagated estimate.
  void pushImu(const ImuData& imu);

  /// Retrieve the most recent LiDAR result in the map frame.
  OdometryResult latestResult() const;

  /// Retrieve the IMU-rate state propagated from the latest LiDAR correction.
  /// Empty until the first LiDAR target has initialized the propagation base.
  std::optional<OdometryResult> latestPropagatedResult() const;

  /// Current IMU bias estimates.
  Eigen::Vector3d accelBias() const;
  Eigen::Vector3d gyroBias() const;

  /// Retrieve the most recent deskewed point cloud in the world frame.
  PointCloudConstPtr latestDeskewed() const;

  /// Submap diagnostics, primarily useful for deterministic verification.
  size_t keyframeCount() const;
  size_t submapTargetRevision() const;
  /// LiDAR-rate trajectory in the map frame. History follows each sample's keyframe.
  MapTrajectory mapTrajectory() const;

  /// Finish the open submap before exporting the final map or trajectory.
  void finishMapping();

  PoseGraphSnapshot poseGraphSnapshot() const;
  void requestPoseGraphSnapshot();
  void requestPoseGraphMap();
  PointCloudConstPtr latestPoseGraphMap() const;
  void requestOccupancyGrid();
  std::shared_ptr<const OccupancyGridMsg> latestOccupancyGrid() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
