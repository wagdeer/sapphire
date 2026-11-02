#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>

#include "slam.hpp"
#include "parameters.hpp"
#include "maplayer/node_builder.hpp"
#include "module/mapping.hpp"
namespace sapphire {
// Pipeline-owned implementation. Parameters/output outlive this owner.
// process/finish/resetSession are serialized by the mapping worker.
class BackendRuntime {
 public:
  BackendRuntime(const SapphireParameters&, const SlamPipeline::Callbacks&);
  ~BackendRuntime();
  BackendRuntime(const BackendRuntime&) = delete;
  BackendRuntime& operator=(const BackendRuntime&) = delete;
  void process(LidarFrame&&);
  void finish();
  void requestRecovery();
  void requestSession(std::uint64_t generation);
  bool matchingPending() const;
  void pollMatching(bool wait = false);

  void resetSession(std::uint64_t generation);
  void invalidateOdometryDomain();
  SlamPipeline::ContinuationProgress continuationProgress() const;
  SlamPipeline::ContinuationProgress drain();
  void close();
  std::optional<SlamPipeline::ContinuationProgress> captureReady(bool, double&, Eigen::Isometry3d&, Eigen::Isometry3d&) const;

  bool failed() const { return failed_.load(std::memory_order_acquire); }

 private:
  friend struct SlamPipelineTestAccess;
  void submitNode(NodeBuilder::Result);
  void refreshReady();
  std::string backend_database_path() const;
  void create_pose_graph();
  bool resumeSession() const { return session_id_ == 0 && parameters_.pose_graph.map_mode == "resume"; }
  const SapphireParameters &parameters_;
  const SlamPipeline::Callbacks &output_;
  NodeBuilder builder_;
  std::uint64_t next_submission_sequence_ = 0;
  std::atomic_bool recovery_pending_{false};
  std::atomic_bool failed_{false}, output_domain_current_{true};
  std::uint64_t session_id_ = 0;
  std::atomic_uint64_t requested_generation_{0};
  std::string filename_;
  mutable std::mutex pose_graph_mutex_;
  std::unique_ptr<Mapping> pose_graph_;
  SlamPipeline::ContinuationProgress progress_;
  Eigen::Isometry3d ready_correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d ready_pose_ = Eigen::Isometry3d::Identity();
  double ready_timestamp_ = 0;
  bool closed_ = false;
};
}  // namespace sapphire
