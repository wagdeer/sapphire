#pragma once

#include <sapphire/mapping/occupancy_grid.hpp>
#include <sapphire/types.hpp>

#include <cstddef>
#include <memory>
#include <vector>

namespace sapphire {

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
    double correction_translation = 0.0;
    double correction_rotation = 0.0;
    size_t revision = 0;
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

    /// Submit an accepted frontend frame. Keyframe selection is independent
    /// from the frontend submap policy.
    void addFrame(
        const PointCloudConstPtr& cloud_odom,
        const Isometry3d& T_odom_lidar,
        double stamp);

    /// Remember an accepted lidar pose in the odom frame. Call after
    /// addFrame() so the sample anchors to the keyframe that owns it,
    /// including a keyframe this pose just created. Non-keyframes stay on
    /// the previous keyframe. IMU propagation is not recorded.
    void recordOdometrySample(const OdometryResult& odom_frame_sample);

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
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
