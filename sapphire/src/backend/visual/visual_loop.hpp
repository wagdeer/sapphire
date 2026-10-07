#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "common/key_frame.hpp"
#include "backend/visual/faiss/descriptor_archive.hpp"
#include "parameters.h"

namespace sapphire {
class Memory;
namespace mapping::scene {
class FeatureMap;
}
// Deterministic scene recipe, in the original submap anchor (metres).
// Geometry comes only from posed visual triangulation; descriptor identity/order
// remains first occurrence, including appearance-only duplicates.
std::shared_ptr<const mapping::scene::FeatureMap> buildVisualScene(const std::vector<VisualFrame> &frames,
    const Eigen::Isometry3d &T_odom_submap);
std::shared_ptr<const mapping::scene::FeatureMap> buildVisualScene(const SubmapFrame &submap);

struct VisualSubmapMatch {
  std::uint64_t target_id = 0;
  std::size_t matches = 0;
  float retrieval_score = 0;
  // A proposed metric initial pose only; independent LiDAR verification required.
  std::optional<Eigen::Isometry3d> T_target_query;
};

// Owned exclusively by the pose-graph worker. The visual index holds binary
// descriptors; immutable appearance records are loaded from Memory on demand.
class VisualSubmapIndex final {
 public:
  VisualSubmapIndex(const VisualLoopParameters &config, Memory &memory);
  ~VisualSubmapIndex();
  void addSubmap(const SubmapFrame &submap);
  void retireScene(std::uint64_t id); // postcommit searchable payload membership only
  // While continuation is active, historical queries requiring a different prefix throw logic_error.
  std::vector<VisualSubmapMatch> query(std::uint64_t query_id);
  void settle(); // Join/check owned training before readiness or checked close.
  void finishQuery(std::uint64_t query_id);
  void restoreHistory(std::size_t node_count);
  // Restore all committed targets, including the B1 root. Query-time temporal
  // filtering still applies to ordinary historical retrieval.
  void materializeCommittedRoot(std::size_t node_count);
  // One canonical ascending/settled archive at handoff, then zero/one insert per commit.
  void beginContinuation(std::uint64_t root_key, std::size_t committed_count);
  void advanceContinuation(std::size_t committed_count);
  std::vector<VisualSubmapMatch> queryTransient(const SubmapFrame &query);
  // Unattached producer: all historical targets, no same-chain exclusion or ID rewrite.
  std::vector<VisualSubmapMatch> queryFreshSession(const SubmapFrame &query, const std::vector<VisualFrame> *observations = nullptr);
  std::uint64_t eligiblePrefix() const;
  mapping::DescriptorArchive::Stats historyStats() const;
  mapping::DescriptorArchive::IndexNodeMetadata historyMetadata() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire
