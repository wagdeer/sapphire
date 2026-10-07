#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "thirdparty/nanoflann.hpp"
#include "common/key_frame.hpp"
#include "common/rtree.hpp"

namespace sapphire {

struct RecallMatch {
  std::uint64_t submap_id = 0;
  float squared_distance = std::numeric_limits<float>::infinity();
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
};

struct SpatialMatch {
  std::uint64_t submap_id = 0;
  Eigen::Isometry3f map_T_submap = Eigen::Isometry3f::Identity();
};

/// Submap-aware spatial recall policy backed by a generic AABB R-tree.
class SubmapSpatialIndex final {
 private:
  struct Entry {
    AABB local_bounds;
    Eigen::Isometry3f map_T_submap = Eigen::Isometry3f::Identity();
  };

 public:
  void upsert(std::uint64_t submap_id, const AABB &local_bounds, const Eigen::Isometry3f &map_T_submap, bool active = true) {
    if (!local_bounds.valid() || !map_T_submap.matrix().allFinite()) {
      throw std::invalid_argument("Submap spatial index requires finite bounds and pose");
    }
    const AABB map_bounds = local_bounds.transform(map_T_submap);
    if (!map_bounds.valid()) {
      throw std::invalid_argument("Cannot transform submap bounds");
    }

    if (submap_id < entries_.size()) {
      entries_[submap_id] = {local_bounds, map_T_submap};
    } else {
      if (submap_id != entries_.size()) {
        throw std::invalid_argument("Submap spatial index requires contiguous IDs");
      }
      entries_.push_back({local_bounds, map_T_submap});
    }
    bounds_index_.upsert(submap_id, map_bounds, active);
  }

  void updatePoses(const std::vector<std::pair<std::uint64_t, Eigen::Isometry3f>> &updates) {
    for (const auto &[submap_id, map_T_submap] : updates) {
      if (submap_id < entries_.size()) {
        upsert(submap_id, entries_[submap_id].local_bounds, map_T_submap, bounds_index_.active(submap_id));
      }
    }
  }

  std::vector<SpatialMatch> query(std::uint64_t query_id, const Eigen::Isometry3f &map_T_query) const {
    if (query_id >= entries_.size() || !map_T_query.matrix().allFinite()) {
      return {};
    }
    return query(entries_[query_id].local_bounds, map_T_query);
  }

  std::vector<SpatialMatch> query(const AABB &local_bounds, const Eigen::Isometry3f &map_T_query) const {
    const AABB query_bounds = local_bounds.transform(map_T_query);
    if (!query_bounds.valid()) {
      return {};
    }

    std::vector<SpatialMatch> matches;
    bounds_index_.visitIntersections(query_bounds, [&](std::uint64_t submap_id, const AABB &map_bounds) {
      if (submap_id < entries_.size() && query_bounds.xyOverlapRatio(map_bounds) > kMinimumXyOverlapRatio) {
        matches.push_back({submap_id, entries_[submap_id].map_T_submap});
      }
    });
    return matches;
  }

  bool contains(std::uint64_t submap_id) const noexcept { return bounds_index_.contains(submap_id); }

  // Identity/pose metadata survives retirement; only target lookup membership changes.
  void setActive(std::uint64_t id, bool active) { bounds_index_.setActive(id, active); }
  bool active(std::uint64_t id) const noexcept { return bounds_index_.active(id); }
  std::size_t activeSize() const noexcept { return bounds_index_.activeSize(); }

  std::size_t size() const noexcept { return bounds_index_.size(); }

 private:
  static constexpr float kMinimumXyOverlapRatio = 0.5F;

  DenseAABBIndex bounds_index_;
  std::vector<Entry> entries_;
};

class RecallIndex {
 public:
  virtual ~RecallIndex() = default;

  virtual MetaTagType type() const noexcept = 0;
  virtual bool available() const noexcept = 0;
  virtual bool contains(std::uint64_t submap_id) const = 0;
  virtual void upsert(std::uint64_t submap_id, const MetaTag &tag) = 0;
  virtual void erase(std::uint64_t submap_id) = 0;
  virtual std::vector<RecallMatch> query(std::uint64_t query_id, const Eigen::Isometry3f &query_pose, std::size_t top_k,
                                         float max_squared_distance) const = 0;
};

/// Adapter boundary for a FAISS-backed global float descriptor index.
class FloatRecallIndex : public RecallIndex {
 public:
  MetaTagType type() const noexcept final { return MetaTagType::kFloat; }
};

/// Adapter boundary for an FBoW-backed binary descriptor index.
class BinaryRecallIndex : public RecallIndex {
 public:
  MetaTagType type() const noexcept final { return MetaTagType::kBinary; }
};

class PoseRecallIndex final : public RecallIndex {
 public:
  PoseRecallIndex() : tree_(3, dataset_) {}

  MetaTagType type() const noexcept override { return MetaTagType::kPose; }
  bool available() const noexcept override { return true; }
  bool contains(std::uint64_t submap_id) const override { return id_to_index_.count(submap_id) != 0; }

  void upsert(std::uint64_t submap_id, const MetaTag &tag) override {
    if (tag.type() != MetaTagType::kPose) {
      throw std::invalid_argument("PoseRecallIndex only accepts pose tags");
    }
    const Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> matrix(tag.descriptor().ptr<float>());
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.matrix() = matrix;
    upsert(submap_id, pose);
  }

  void upsert(std::uint64_t submap_id, const Eigen::Isometry3f &pose) {
    if (!pose.matrix().allFinite()) {
      throw std::invalid_argument("Pose recall entry must be finite");
    }
    const auto existing = id_to_index_.find(submap_id);
    if (existing != id_to_index_.end()) {
      const std::size_t index = existing->second;
      tree_.removePoint(index);
      dataset_.entries[index].pose = pose;
      tree_.addPoint(index);
      return;
    }

    const std::size_t index = dataset_.entries.size();
    dataset_.entries.push_back({submap_id, pose});
    id_to_index_.emplace(submap_id, index);
    tree_.addPoint(index);
  }

  void upsert(const std::vector<std::pair<std::uint64_t, Eigen::Isometry3f>> &updates) {
    if (updates.empty()) {
      return;
    }
    for (const auto &[submap_id, pose] : updates) {
      (void)submap_id;
      if (!pose.matrix().allFinite()) {
        throw std::invalid_argument("Pose recall entry must be finite");
      }
    }
    if (updates.size() == 1) {
      upsert(updates.front().first, updates.front().second);
      return;
    }

    dataset_.entries.reserve(dataset_.entries.size() + updates.size());
    id_to_index_.reserve(id_to_index_.size() + updates.size());
    try {
      for (const auto &[submap_id, pose] : updates) {
        const auto existing = id_to_index_.find(submap_id);
        if (existing != id_to_index_.end()) {
          dataset_.entries[existing->second].pose = pose;
          continue;
        }

        const std::size_t index = dataset_.entries.size();
        dataset_.entries.push_back({submap_id, pose});
        id_to_index_.emplace(submap_id, index);
      }
    } catch (...) {
      rebuildTree();
      throw;
    }
    rebuildTree();
  }

  void erase(std::uint64_t submap_id) override {
    const auto existing = id_to_index_.find(submap_id);
    if (existing == id_to_index_.end()) {
      return;
    }
    tree_.removePoint(existing->second);
    id_to_index_.erase(existing);
  }

  std::vector<RecallMatch> query(std::uint64_t, const Eigen::Isometry3f &query_pose, std::size_t top_k, float max_squared_distance) const override {
    if (top_k == 0 || id_to_index_.empty() || !query_pose.matrix().allFinite() || std::isnan(max_squared_distance) || max_squared_distance < 0.0F) {
      return {};
    }

    const std::size_t count = std::min(top_k, id_to_index_.size());
    std::vector<RecallMatch> matches;
    matches.resize(count);
    const std::size_t found = knnSearch(
        query_pose, count, max_squared_distance, [](const RecallMatch &) { return true; }, matches.data());
    matches.resize(found);
    return matches;
  }

  /// Returns up to k nearest accepted poses inside the inclusive squared radius.
  template <typename Predicate>
  std::size_t knnSearch(const Eigen::Isometry3f &query_pose, std::size_t k, float max_squared_distance, Predicate &&accept,
                        RecallMatch *output) const {
    if (k == 0 || output == nullptr || id_to_index_.empty() || !query_pose.matrix().allFinite() || std::isnan(max_squared_distance) ||
        max_squared_distance < 0.0F) {
      return 0;
    }

    const float search_radius =
        std::isfinite(max_squared_distance) ? std::nextafter(max_squared_distance, std::numeric_limits<float>::infinity()) : max_squared_distance;
    const Eigen::Vector3f translation = query_pose.translation();
    using Filter = typename std::remove_reference<Predicate>::type;
    FilteredKnnResultSet<Filter> result(k, search_radius, dataset_, accept, output);
    tree_.findNeighbors(result, translation.data(), nanoflann::SearchParameters(0.0F, false));
    return result.size();
  }

  const Eigen::Isometry3f *pose(std::uint64_t submap_id) const {
    const auto existing = id_to_index_.find(submap_id);
    return existing == id_to_index_.end() ? nullptr : &dataset_.entries[existing->second].pose;
  }

  std::size_t size() const noexcept { return id_to_index_.size(); }

 private:
  struct Entry {
    std::uint64_t submap_id;
    Eigen::Isometry3f pose;
  };

  struct Dataset {
    std::vector<Entry> entries;

    std::size_t kdtree_get_point_count() const { return entries.size(); }
    float kdtree_get_pt(std::size_t index, std::size_t dimension) const { return entries[index].pose.translation()[dimension]; }
    template <class BoundingBox>
    bool kdtree_get_bbox(BoundingBox &) const {
      return false;
    }
  };

  template <typename Predicate>
  class FilteredKnnResultSet {
   public:
    using DistanceType = float;
    using IndexType = std::size_t;

    FilteredKnnResultSet(std::size_t capacity, float radius, const Dataset &dataset, Predicate &accept, RecallMatch *output)
        : capacity_(capacity), radius_(radius), dataset_(dataset), accept_(accept), output_(output) {}

    std::size_t size() const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }
    bool full() const noexcept { return size_ == capacity_; }

    bool addPoint(float squared_distance, std::size_t index) {
      const Entry &entry = dataset_.entries[index];
      const RecallMatch candidate{entry.submap_id, squared_distance, entry.pose};
      if (!accept_(candidate)) {
        return true;
      }

      std::size_t position = 0;
      while (position < size_ && !better(candidate, output_[position])) {
        ++position;
      }
      if (position >= capacity_) {
        return true;
      }

      const std::size_t last = std::min(size_, capacity_ - 1);
      for (std::size_t i = last; i > position; --i) {
        output_[i] = std::move(output_[i - 1]);
      }
      output_[position] = candidate;
      if (size_ < capacity_) {
        ++size_;
      }
      return true;
    }

    float worstDist() const noexcept {
      if (!full()) {
        return radius_;
      }
      const float distance = output_[size_ - 1].squared_distance;
      return std::isfinite(distance) ? std::nextafter(distance, std::numeric_limits<float>::infinity()) : distance;
    }

    void sort() noexcept {}

   private:
    static bool better(const RecallMatch &left, const RecallMatch &right) noexcept {
      return left.squared_distance < right.squared_distance || (left.squared_distance == right.squared_distance && left.submap_id < right.submap_id);
    }

    std::size_t capacity_;
    float radius_;
    const Dataset &dataset_;
    Predicate &accept_;
    RecallMatch *output_;
    std::size_t size_ = 0;
  };

  using Distance = nanoflann::L2_Simple_Adaptor<float, Dataset>;
  using Tree = nanoflann::KDTreeSingleIndexIncrementalAdaptor<Distance, Dataset, 3, std::size_t>;

  void rebuildTree() {
    std::vector<std::size_t> live_indices;
    live_indices.reserve(id_to_index_.size());
    for (const auto &[submap_id, index] : id_to_index_) {
      (void)submap_id;
      live_indices.push_back(index);
    }
    std::sort(live_indices.begin(), live_indices.end());
    tree_.buildFromIndices(live_indices);
  }

  Dataset dataset_;
  Tree tree_;
  std::unordered_map<std::uint64_t, std::size_t> id_to_index_;
};

struct RecallIndexAdapters {
  std::unique_ptr<FloatRecallIndex> float_index;
  std::unique_ptr<BinaryRecallIndex> binary_index;
};

}  // namespace sapphire
