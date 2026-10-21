#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>

#include "backend/sensor/lidar/submap_builder.hpp"
#include "backend/system/action/mapping.hpp"
#include "app/output.hpp"
#include "app/parameters.h"
namespace sapphire {
enum class CommitOutcome { NotCommitted, Committed, Unknown };
enum class ContinuationHead { Idle, Processing, Retryable, Failed };
struct ContinuationProgress {
  std::string map_uuid;
  std::optional<std::uint64_t> generation;
  std::uint64_t ready_revision = 0, accepted = 0, completed = 0;
  std::optional<std::uint64_t> committed_revision, completed_revision, head_sequence;
  std::optional<std::uint64_t> root_source, root_revision;
  bool committed_outcome_known = true, ready_available = false;
  std::exception_ptr first_failure;
  std::optional<std::uint64_t> last_accepted, last_completed, canceled_first, canceled_last;
  std::size_t waiting = 0, high_water = 0, waiting_bytes = 0, head_bytes = 0;
  std::optional<CommitOutcome> last_commit_outcome;
  ContinuationHead head = ContinuationHead::Idle;
  std::string cancellation_reason;
  // Pipeline-owned automatic stage, separate from accepted B2 work above.
  std::size_t association_pending = 0, association_bytes = 0, association_high_water = 0;
  std::uint64_t association_attempts = 0;
  bool automatically_attached = false;
};

// Backend state owner; process/finish/resetSession are serialized by its caller.
class BackendRuntime {
 public:
  BackendRuntime(const SapphireParameters&, OutputSink, std::optional<std::pair<int, Eigen::Isometry3d>> = std::nullopt);
  ~BackendRuntime();
  BackendRuntime(const BackendRuntime&) = delete;
  BackendRuntime& operator=(const BackendRuntime&) = delete;
  void process(LidarFrame&&, std::deque<ImageMeas> = {});
  void finish();
  void resetSession();
  void stop();
  void invalidateOdometryDomain();
  void suspendOutput() { output_domain_current_.store(false); }
  void setAttachmentTarget(int, const Eigen::Isometry3d&);
  void retryContinuation(std::uint64_t);
  ContinuationProgress continuationProgress() const;
  ContinuationProgress drain();
  void close();
  bool captureReady(bool, std::string&, std::uint64_t&, std::uint64_t&, std::uint64_t&, double&, Eigen::Isometry3d&, Eigen::Isometry3d&) const;
  std::shared_ptr<const backend::NavimapWorker::Map> captureNavigation(const std::string&, std::uint64_t) const;
  bool failed() const { return failed_.load(std::memory_order_acquire); }
  std::uint64_t unacceptedTailDrops() const { return unaccepted_tail_drops_.load(); }

 private:
  friend struct SlamPipelineTestAccess;
  void submitSubmap(SubmapFrame);
  std::string backend_database_path() const;
  void create_pose_graph();
  const SapphireParameters parameters_;
  OutputSink output_;
  SubmapBuilder builder_;
  std::string initial_filename_;
  std::mutex attachment_mutex_;
  std::condition_variable attachment_cv_;
  std::optional<std::pair<int, Eigen::Isometry3d>> attachment_;
  std::uint64_t attachment_attempt_ = 0;
  std::atomic_uint64_t unaccepted_tail_drops_{0};
  std::atomic_bool failed_{false}, stopping_{false}, output_domain_current_{true};
  int session_id_ = 0;
  std::uint64_t owner_generation_ = 0;
  std::string filename_;
  std::string save_path_;
  int save_map_ = 0;
  mutable std::mutex pose_graph_mutex_;
  std::unique_ptr<backend::Mapping> pose_graph_;
  ContinuationProgress progress_;
  backend::LocalMapRasterState navigation_;
  std::shared_ptr<const backend::NavimapWorker::Map> ready_navigation_;
  Eigen::Isometry3d ready_correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d ready_pose_ = Eigen::Isometry3d::Identity();
  double ready_timestamp_ = 0;
  bool attached_ = false, closed_ = false;
};
}  // namespace sapphire
