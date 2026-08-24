#include <sapphire/odometry/submap.hpp>
#include <sapphire/odometry/voxel_filter.hpp>

#include <SO3.hpp>

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
    const double angle =
        lie::SO3d::log(lie::SO3d(delta.rotation())).norm();

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
    stored_point_count_ += cloud_world->size();
    const std::vector<size_t> selected =
        selectNearest(T_world_lidar.translation());

    bool target_changed = false;
    if (selected != active_indices_) {
        active_indices_ = selected;
        rebuildTarget();
        target_changed = true;
    }

    pruneStaleKeyframes();

    return target_changed;
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

    target_ = deterministicVoxelDownsample(*merged, config_.voxel_size);
    ++target_revision_;
}

void SubmapManager::pruneStaleKeyframes() {
    // Keep a buffer of 3× the active window to avoid thrashing under
    // back-and-forth motion while still bounding memory growth.
    constexpr size_t kBufferFactor = 3;
    const size_t limit =
        static_cast<size_t>(config_.max_keyframes) * kBufferFactor;
    if (keyframes_.size() <= limit) {
        return;
    }

    // Mark every keyframe that participates in the current active window.
    std::vector<bool> keep(keyframes_.size(), false);
    for (size_t idx : active_indices_) {
        keep[idx] = true;
    }

    // Compact: move kept elements to the front, track old→new mapping.
    std::vector<size_t> new_index(keyframes_.size());
    size_t write = 0;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (keep[i]) {
            new_index[i] = write;
            if (write != i) {
                keyframes_[write] = std::move(keyframes_[i]);
            }
            ++write;
        } else {
            stored_point_count_ -= keyframes_[i].cloud_world->size();
        }
    }
    keyframes_.resize(write);

    // Remap active_indices_ to the compacted vector.
    for (size_t& idx : active_indices_) {
        idx = new_index[idx];
    }
}

}  // namespace sapphire
