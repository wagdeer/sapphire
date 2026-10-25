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
  BackendRuntime(const SapphireParameters&, const SlamPipeline::Callbacks&, std::optional<std::pair<int, Eigen::Isometry3d>> = std::nullopt);
  ~BackendRuntime();
  BackendRuntime(const BackendRuntime&) = delete;
  BackendRuntime& operator=(const BackendRuntime&) = delete;
  void process(LidarFrame&&);
  void finish();
  void resetSession();
  void stop();
  void invalidateOdometryDomain();
  void suspendOutput() { output_domain_current_.store(false); }
  void setAttachmentTarget(int, const Eigen::Isometry3d&);
  void retryContinuation(std::uint64_t);
  SlamPipeline::ContinuationProgress continuationProgress() const;
  SlamPipeline::ContinuationProgress drain();
  void close();
  std::optional<SlamPipeline::ContinuationProgress> captureReady(bool, double&, Eigen::Isometry3d&, Eigen::Isometry3d&) const;

  bool failed() const { return failed_.load(std::memory_order_acquire); }
  std::uint64_t unacceptedTailDrops() const { return unaccepted_tail_drops_.load(); }

 private:
  friend struct SlamPipelineTestAccess;
  void submitNode(NodeBuilder::Result);
  std::string backend_database_path() const;
  void create_pose_graph();
  const SapphireParameters &parameters_;
  const SlamPipeline::Callbacks &output_;
  NodeBuilder builder_;
  std::uint64_t next_submission_sequence_ = 0;
  std::string initial_filename_;
  std::mutex attachment_mutex_;
  std::condition_variable attachment_cv_;
  std::optional<std::pair<int, Eigen::Isometry3d>> attachment_;
  std::uint64_t attachment_attempt_ = 0;
  std::atomic_uint64_t unaccepted_tail_drops_{0};
  std::atomic_bool failed_{false}, stopping_{false}, output_domain_current_{true};
  int session_id_ = 0;
  std::string filename_;
  mutable std::mutex pose_graph_mutex_;
  std::unique_ptr<Mapping> pose_graph_;
  SlamPipeline::ContinuationProgress progress_;
  Eigen::Isometry3d ready_correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d ready_pose_ = Eigen::Isometry3d::Identity();
  double ready_timestamp_ = 0;
  bool attached_ = false, closed_ = false;
};
}  // namespace sapphire
