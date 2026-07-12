#pragma once

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
    size_t loop_edges = 0;
    double last_loop_fitness = -1.0;
    double correction_translation = 0.0;
    double correction_rotation = 0.0;
    size_t revision = 0;
};

struct PoseGraphEdge {
    size_t source = 0;
    size_t target = 0;
};

struct PoseGraphSnapshot {
    PoseGraphStats stats;
    std::vector<Isometry3d> optimized_poses;
    std::vector<PoseGraphEdge> loop_edges;
};

/// Asynchronous ROS-free pose-graph and loop-closure backend.
///
/// The frontend remains entirely in the odom frame. This backend consumes
/// immutable frontend frame snapshots and publishes only the latest
/// map-from-odom correction.
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

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
