#pragma once

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <memory>
#include <sapphire/types.hpp>
#include <vector>

#include <sapphire/mapping/voxelmap.hpp>

namespace sapphire {
struct RevisitTarget {
  std::unique_ptr<const GaussianVoxelMap> map;
  double anchor_stamp;
  Eigen::Isometry3d global_from_source;
  Eigen::Isometry3d global_from_target;
  std::size_t source_id, target_id, bytes_read, blocks;
};

// One publisher and one reader. Destruction of retired targets stays on the publisher.
template <typename Target>
class MapExchange {
 public:
  void publish(std::unique_ptr<const Target> target) {
    if (owned_) retired_.push_back(std::move(owned_));
    owned_ = std::move(target);
    ready_.store(owned_.get());
    // Read in this order so promotion always protects a target in one slot.
    const auto* trial = candidate_.load();
    const auto* current = active_.load();
    retired_.erase(std::remove_if(retired_.begin(), retired_.end(), [=](const auto& map) { return map.get() != trial && map.get() != current; }),
                   retired_.end());
  }

  // Protect the latest read-only target until releaseCandidate().
  const Target* read_only() {
    const auto* target = ready_.load();
    candidate_.store(target);
    if (target == ready_.load()) return target;
    candidate_.store(nullptr);
    return nullptr;
  }

  void releaseCandidate() { candidate_.store(nullptr); }
  void setActive(const Target* target) { active_.store(target); }

 private:
  std::atomic<const Target*> ready_{nullptr}, candidate_{nullptr}, active_{nullptr};
  std::unique_ptr<const Target> owned_;
  std::vector<std::unique_ptr<const Target>> retired_;
};

class RevisitExchange : public MapExchange<RevisitTarget> {};
class LocalPlaneExchange : public MapExchange<GaussianVoxelMap> {};

struct RevisitBlock {
  std::size_t id;
  Isometry3d global_from_block;
};

struct PlaceProposal {
  std::size_t target;
  Isometry3d relative;
  float score;
};

// Stores sealed voxel maps and indexes places for historical queries.
// Correlation rasters are loaded only for shortlisted places.
class SubmapArxiv {
 public:
  explicit SubmapArxiv(std::filesystem::path directory);
  ~SubmapArxiv();
  void write(std::size_t id, const GaussianVoxelMap& map);
  std::unique_ptr<GaussianVoxelMap> read(std::size_t id, std::size_t* bytes = nullptr) const;
  std::vector<PlaceProposal> indexAndQuery(const GaussianVoxelMap& map, const Isometry3d& track_anchor);
  // The first block defines the target frame.
  void publishRevisit(std::size_t source, double stamp, const Isometry3d& source_pose, const std::vector<RevisitBlock>& blocks);
  RevisitExchange& revisitExchange() { return revisit_; }

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  RevisitExchange revisit_;
};
}  // namespace sapphire
