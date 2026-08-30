#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "map_database.hpp"
#include "retrieval_index.hpp"

namespace sapphire {

class Memory final {
 public:
  explicit Memory(const std::string &database_path, RecallIndexAdapters adapters = {}) : database_(database_path) {
    descriptor_indexes_[static_cast<std::size_t>(MetaTagType::kFloat)] = std::move(adapters.float_index);
    descriptor_indexes_[static_cast<std::size_t>(MetaTagType::kBinary)] = std::move(adapters.binary_index);
    database_.visitSpatialRecords(
        [this](std::uint64_t submap_id, const AABB &local_bounds, const Eigen::Isometry3f &map_T_submap) {
          spatial_index_.upsert(submap_id, local_bounds, map_T_submap);
        });
    for (auto &[submap_id, tag] : database_.loadMetaTags()) {
      RecallIndex *index = indexFor(tag.type());
      if (index != nullptr && index->available()) {
        index->upsert(submap_id, tag);
      }
    }
  }

  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;

  void saveSubmap(const SubmapFrame &submap, const LocalGrid &grid) {
    database_.saveSubmap(submap, grid);
    std::lock_guard<std::mutex> lock(index_mutex_);
    spatial_index_.upsert(submap.id(), submap.bounds(), submap.lio().T_odom_base.cast<float>());
    pose_index_.erase(submap.id());
    for (std::unique_ptr<RecallIndex> &index : descriptor_indexes_) {
      if (index) {
        index->erase(submap.id());
      }
    }
    for (const std::optional<MetaTag> &tag : submap.tags()) {
      if (tag) {
        RecallIndex *index = indexFor(tag->type());
        if (index != nullptr && index->available()) {
          index->upsert(submap.id(), *tag);
        }
      }
    }
  }

  std::vector<RecallMatch> recall(std::uint64_t query_id, const Eigen::Isometry3f &query_pose, std::size_t top_k, float max_distance) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    std::vector<RecallMatch> descriptor_matches;
    bool used_descriptor_index = false;
    for (const std::unique_ptr<RecallIndex> &index : descriptor_indexes_) {
      if (!index || !index->available() || !index->contains(query_id)) {
        continue;
      }
      used_descriptor_index = true;
      std::vector<RecallMatch> matches = index->query(query_id, query_pose, top_k, std::numeric_limits<float>::infinity());
      descriptor_matches.insert(descriptor_matches.end(), matches.begin(), matches.end());
    }
    if (!used_descriptor_index) {
      const float radius_squared = max_distance * max_distance;
      return pose_index_.query(query_id, query_pose, top_k, radius_squared);
    }

    std::unordered_map<std::uint64_t, RecallMatch> unique;
    for (RecallMatch &match : descriptor_matches) {
      const Eigen::Isometry3f *pose = pose_index_.pose(match.submap_id);
      if (pose == nullptr) {
        continue;
      }
      match.pose = *pose;
      const auto existing = unique.find(match.submap_id);
      if (existing == unique.end() || match.squared_distance < existing->second.squared_distance) {
        unique[match.submap_id] = match;
      }
    }
    descriptor_matches.clear();
    descriptor_matches.reserve(unique.size());
    for (auto &[submap_id, match] : unique) {
      (void)submap_id;
      descriptor_matches.push_back(std::move(match));
    }
    std::sort(descriptor_matches.begin(), descriptor_matches.end(),
              [](const RecallMatch &left, const RecallMatch &right) { return left.squared_distance < right.squared_distance; });
    if (descriptor_matches.size() > top_k) {
      descriptor_matches.resize(top_k);
    }
    return descriptor_matches;
  }

  std::vector<SpatialMatch> recallSpatial(std::uint64_t query_id, const Eigen::Isometry3f &map_T_query) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    return spatial_index_.query(query_id, map_T_query);
  }

  std::shared_ptr<const vvec<float, 3>> loadCloud(std::uint64_t submap_id) { return database_.loadCloud(node_id(submap_id)); }

  std::vector<VisualFrame> loadVisualFrames(std::uint64_t submap_id) { return database_.loadVisualFrames(node_id(submap_id)); }

  NavigationPath loadNavigation(std::uint64_t submap_id) { return database_.loadNavigation(node_id(submap_id)); }

  cpu::VoxelMapsData loadPyramidVoxel(std::uint64_t submap_id) { return database_.loadPyramidVoxel(node_id(submap_id)); }

  bool loadLocalGrid(std::uint64_t submap_id, LocalGrid &grid) { return database_.loadLocalGrid(node_id(submap_id), grid); }

  void saveOptimizedPose(int id, const Eigen::Isometry3f &pose) { saveOptimizedPoses({{id, pose}}); }

  void saveOptimizedPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses) {
    database_.saveOptimizedPoses(poses);
    std::lock_guard<std::mutex> lock(index_mutex_);
    std::vector<std::pair<std::uint64_t, Eigen::Isometry3f>> index_updates;
    index_updates.reserve(poses.size());
    for (const auto &[node_id, pose] : poses) {
      if (node_id > 0) {
        index_updates.emplace_back(static_cast<std::uint64_t>(node_id - 1), pose);
      }
    }
    pose_index_.upsert(index_updates);
    spatial_index_.updatePoses(index_updates);
  }

  void saveLink(int from_id, int to_id, int type, const Eigen::Isometry3f &transform) { database_.saveLink(from_id, to_id, type, transform); }

 private:
  static int node_id(std::uint64_t submap_id) {
    if (submap_id >= static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("Submap ID exceeds map-database integer range");
    }
    return static_cast<int>(submap_id) + 1;
  }

  RecallIndex *indexFor(MetaTagType type) {
    if (type == MetaTagType::kPose) {
      return &pose_index_;
    }
    const std::size_t index = static_cast<std::size_t>(type);
    return index < descriptor_indexes_.size() ? descriptor_indexes_[index].get() : nullptr;
  }

  MapDatabase database_;
  mutable std::mutex index_mutex_;
  std::array<std::unique_ptr<RecallIndex>, 2> descriptor_indexes_;
  PoseRecallIndex pose_index_;
  SubmapSpatialIndex spatial_index_;
};

}  // namespace sapphire
