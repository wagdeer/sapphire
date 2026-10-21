#pragma once
#include <backend/common/common_type.hpp>
#include <backend/common/odom_observation.hpp>
#include <backend/common/state_group.hpp>
#include <backend/database/sensor_archive.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/params/archive.hpp>
#include <map>
#include <memory>
#include <set>
#include <vector>
namespace sapphire::backend {
class Node;
class Link;
class DBDriver;
void persist_feature_content(DBDriver*, Node&, int source, const cv::Mat& expected_target = {}, const cv::Mat& expected_source = {});

uint64_t persist_fused_submap(DBDriver&, Node&, int source, const std::vector<Node*>& links, const std::map<int, StateGroupf>& poses,
                              const StateGroupf& last, const cv::Mat& expected_target = {}, const cv::Mat& expected_source = {}, int region = 0,
                              const cv::Mat& expected_region = {});
class Memory;
class LocalMapCollection;
struct LocalMap;
void persist_localmap(const Memory&, LocalMap&);
void append_localmap(LocalMapCollection&, double stamp, const StateGroupf& odom, std::shared_ptr<const LocalGrid>);
std::shared_ptr<const LocalMap> seal_localmap(LocalMapCollection&, const std::function<void(LocalMap&)>& persist = {});
void restore_localmaps(const Memory&, LocalMapCollection&);
class LocalMapMerge;
class Statistics;
// Persist the prepared contraction before owners publish their RAM changes.
void persist_submap_merge(DBDriver&, int retained, int removed, const std::vector<Node*>& writes, const LocalMapMerge&,
                          const std::map<int, StateGroupf>& poses, const StateGroupf& last_pose, const StateGroupf& transform);
// Reusable compression workspace and archive retention policy. No live map/index
// mutation; callers publish the completed SensorRecord through its storage owner.
class ArchiveWriter final {
 public:
  explicit ArchiveWriter(const ParametersMap& = {}, std::shared_ptr<tools::ParallelExecutor<>> = {});
  explicit ArchiveWriter(const ArchiveParams&, std::shared_ptr<tools::ParallelExecutor<>> = {});
  void prepare(SensorRecord&, const SensorData& input, const Node&, Statistics* = nullptr) const;
  tools::ParallelExecutor<>* executor() const { return config_.parallel_compression ? executor_.get() : nullptr; }

 private:
  ArchiveParams config_;
  std::shared_ptr<tools::ParallelExecutor<>> executor_;
};
// Owner-thread trajectory accumulation and keyframe batch submission. Rejected
// submissions retain observations here; accepted batches belong to DBDriver.
// Asynchronous failures are terminal and remain visible through reap()/Save.
class TrajectoryWriter final {
 public:
  void flush(const Memory*);
  void submit(Memory&, int node_id, const std::vector<OdomObservation>& current, std::shared_ptr<const LocalMap> sealed, const Statistics* = nullptr,
              bool save_wm = false);
  // Called only by the mapping owner. Worker threads never mutate LocalMap.
  void reap(const Memory&);
  void record(const LocalMapCollection&, const Memory*, bool persisted = false);
  void reset(int last_sequence = 0) {
    pending_.clear();
    navimap_.clear();
    recorded_sequence_ = last_sequence;
  }
  std::vector<OdomObservation> recent(const Memory&, int limit) const;

 private:
  std::vector<OdomObservation> pending_;
  int recorded_sequence_ = 0;
  struct NavimapWrite {
    std::shared_ptr<const LocalMap> map;
    cv::Mat archive;
    uint64_t sequence, epoch;
  };
  std::vector<NavimapWrite> navimap_;
};
}  // namespace sapphire::backend
