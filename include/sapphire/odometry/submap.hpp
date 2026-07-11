#pragma once

#include <sapphire/types.hpp>

#include <cstddef>
#include <vector>

namespace sapphire {

/// An accepted LiDAR frame retained for local-map construction.
struct Keyframe {
    Isometry3d T_world_lidar = Isometry3d::Identity();
    PointCloudConstPtr cloud_world;
    double stamp = 0.0;
};

/// Deterministic, synchronous scan-to-submap manager.
///
/// Keyframe clouds are already expressed in the world frame. The active target
/// is rebuilt only when the nearest-keyframe index set changes.
class SubmapManager {
public:
    explicit SubmapManager(const Config::Odometry::Submap& config);

    /// Whether an accepted pose should become a keyframe.
    bool shouldAddKeyframe(const Isometry3d& T_world_lidar) const;

    /// Add an accepted world-frame scan. Returns true when the GICP target was
    /// rebuilt because the active nearest-keyframe set changed.
    bool addKeyframe(
        const Isometry3d& T_world_lidar,
        const PointCloudConstPtr& cloud_world,
        double stamp);

    size_t keyframeCount() const { return keyframes_.size(); }
    const std::vector<size_t>& activeKeyframeIndices() const {
        return active_indices_;
    }
    PointCloudConstPtr target() const { return target_; }
    size_t targetRevision() const { return target_revision_; }
    size_t storedPointCount() const { return stored_point_count_; }

private:
    std::vector<size_t> selectNearest(
        const Eigen::Vector3d& position) const;
    void rebuildTarget();

    Config::Odometry::Submap config_;
    std::vector<Keyframe> keyframes_;
    std::vector<size_t> active_indices_;
    PointCloudConstPtr target_;
    size_t target_revision_ = 0;
    size_t stored_point_count_ = 0;
};

}  // namespace sapphire
