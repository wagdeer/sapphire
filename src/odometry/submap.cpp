#include <sapphire/odometry/submap.hpp>

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace sapphire {

SubmapManager::SubmapManager(const Config::Odometry::Submap& config)
    : config_(config)
{
}

bool SubmapManager::shouldAddKeyframe(
    const Isometry3d& T_world_lidar) const
{
    if (keyframes_.empty()) {
        return true;
    }

    const Isometry3d delta =
        keyframes_.back().T_world_lidar.inverse() * T_world_lidar;
    Eigen::Quaterniond rotation(delta.rotation());
    rotation.normalize();
    const double angle =
        2.0 * std::atan2(rotation.vec().norm(), std::abs(rotation.w()));

    constexpr double kThresholdTolerance = 1e-12;
    return delta.translation().norm()
            > config_.splitting_distance + kThresholdTolerance
        || angle > config_.splitting_rotation + kThresholdTolerance;
}

bool SubmapManager::addKeyframe(
    const Isometry3d& T_world_lidar,
    const PointCloudConstPtr& cloud_world,
    double stamp)
{
    if (!cloud_world || cloud_world->empty()) {
        throw std::invalid_argument(
            "SubmapManager keyframe cloud must be non-empty");
    }

    keyframes_.push_back({T_world_lidar, cloud_world, stamp});
    const std::vector<size_t> selected =
        selectNearest(T_world_lidar.translation());
    if (selected == active_indices_) {
        return false;
    }

    active_indices_ = selected;
    rebuildTarget();
    return true;
}

std::vector<size_t> SubmapManager::selectNearest(
    const Eigen::Vector3d& position) const
{
    std::vector<std::pair<double, size_t>> distances;
    distances.reserve(keyframes_.size());
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        const double squared_distance =
            (keyframes_[i].T_world_lidar.translation() - position).squaredNorm();
        distances.emplace_back(squared_distance, i);
    }

    const size_t count = std::min(
        keyframes_.size(), static_cast<size_t>(config_.max_keyframes));
    std::partial_sort(
        distances.begin(),
        distances.begin() + static_cast<std::ptrdiff_t>(count),
        distances.end(),
        [](const auto& lhs, const auto& rhs) {
            if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
            }
            return lhs.second < rhs.second;
        });

    std::vector<size_t> selected;
    selected.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        selected.push_back(distances[i].second);
    }
    std::sort(selected.begin(), selected.end());
    return selected;
}

void SubmapManager::rebuildTarget() {
    auto merged = std::make_shared<PointCloud>();
    size_t point_count = 0;
    for (const size_t index : active_indices_) {
        point_count += keyframes_[index].cloud_world->size();
    }
    merged->reserve(point_count);
    for (const size_t index : active_indices_) {
        *merged += *keyframes_[index].cloud_world;
    }

    auto filtered = std::make_shared<PointCloud>();
    pcl::VoxelGrid<Point> voxel_filter;
    const float leaf_size = static_cast<float>(config_.voxel_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(merged);
    voxel_filter.filter(*filtered);
    // PCL's generic VoxelGrid centroid path does not restore the padding
    // component from PCL_ADD_POINT4D. small_gicp consumes homogeneous point
    // vectors, so every filtered point must explicitly keep w=1.
    for (Point& point : filtered->points) {
        point.data[3] = 1.0f;
    }
    target_ = filtered;
    ++target_revision_;
}

}  // namespace sapphire
