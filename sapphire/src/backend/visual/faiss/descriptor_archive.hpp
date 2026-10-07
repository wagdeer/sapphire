#pragma once
#include <map>
#include "backend/visual/feature/scene_features.hpp"
#include <memory>
#include <string>

namespace sapphire::mapping {
// Derived, owner-thread-only global index. No images, 3D banks or graph ownership.
class DescriptorArchive {
 public:
  struct Candidate {
    float score = 0;
    uint64_t fingerprint = 0;
    std::vector<scene::Match> matches;
  };
  using Results = std::map<int, Candidate>;
  struct Stats {
    size_t nodes, descriptors, slots, bytes;
    uint64_t searches, revision;
    bool ivf, restored;
  };
  DescriptorArchive();
  ~DescriptorArchive();
  void upsert(int id, const scene::FeatureMap& map);
  void erase(int id);
  void clear();
  // Training uses a bounded immutable sample; activation uses current live records.
  bool poll_training(bool wait = false);
  Results search(const features::FeatureBlock& query, int max_distance = 50, float ratio = .75f);
  Stats stats() const;
  uint64_t revision() const;
  // Exact committed feature identity: node -> (FeatureMap fingerprint, descriptor count).
  using IndexNodeMetadata = std::map<int, std::pair<uint64_t, size_t>>;
  IndexNodeMetadata node_metadata() const;
  // Optional derived cache. Rejection leaves the current archive untouched.
  bool restore_cache(const std::string& path, const IndexNodeMetadata& expected, std::string& reason);
  void save_cache(const std::string& path) const;
  static uint64_t fingerprint(const scene::FeatureMap& map);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire::mapping
