#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <memory>
#include <mutex>
#include <thread>

#include "common/imu_state.hpp"
#include "common/common_type.hpp"
#include "parameters.hpp"
#include "module/synchronizer.hpp"
#include "common/lidar_frame.hpp"

namespace sapphire {

class FrontendRuntime;
class BackendRuntime;

class SlamPipeline {
 public:
  enum class CommitOutcome { NotCommitted, Committed, Unknown };
  enum class ContinuationHead { Idle, Processing, Retryable, Failed };
  // Observable admission/commit state. Completed does not imply map-frame readiness.
  struct ContinuationProgress {
    std::uint64_t ready_revision = 0, accepted = 0, completed = 0;
    std::string map_id;
    std::uint64_t generation = 0;
    std::optional<std::uint64_t> completed_revision, head_sequence, last_completed;
    bool ready_available = false;
    std::exception_ptr first_failure;
    CommitOutcome commit_outcome = CommitOutcome::NotCommitted;
    ContinuationHead head = ContinuationHead::Idle;
    std::string cancellation_reason;
    std::size_t association_pending = 0;
    std::uint64_t association_attempts = 0;
  };

  struct Callbacks {
    std::function<void()> ready_changed;
    std::function<void(bool)> owner_reset;    // Normal thread acknowledgement; never under backend/owner locks.
    std::function<void(bool)> optional_drop;  // true = distinct local-map increment lost.
    std::function<void(const StateGroup &)> odom_state;
    // Synchronous odometry-thread diagnostic, after LiDAR correction and before BA.
    std::function<void(const StateGroup &, bool, double)> lidar_update;
    // Completion of one synchronized scan, including initialization/rejection/BA.
    std::function<void(double, double)> scan_processed;
    // Current scan points and local-map increments are in the odometry frame.
    std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_scan;
    std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_map;
  };

  explicit SlamPipeline(SapphireParameters, Callbacks = {}, std::optional<std::pair<int, Eigen::Isometry3d>> = std::nullopt);
  ~SlamPipeline();
  SlamPipeline(const SlamPipeline&) = delete;
  SlamPipeline& operator=(const SlamPipeline&) = delete;
  bool push_imu(ImuMeas imu);
  bool push_lidar(double t, std::vector<LidarPoint>&& cloud, std::optional<double> end = std::nullopt);
  bool push_lidar(double t, const std::vector<LidarPoint>& cloud, std::optional<double> end = std::nullopt);
  void setAttachmentTarget(int target, const Eigen::Isometry3d& seed);
  void retryContinuation(std::uint64_t sequence);
  void invalidateOdometryDomain();
  void notifyInputLoss();
  std::exception_ptr producerFailure() const;
  ContinuationProgress continuationProgress() const;
  void shutdown();
  ContinuationProgress drain();
  void close();
  // Atomically read map transforms and their existing continuation metadata.
  std::optional<ContinuationProgress> captureReady(bool try_only, double& stamp,
                                                  Eigen::Isometry3d& correction, Eigen::Isometry3d& pose) const;

  bool failed() const { return failed_.load(std::memory_order_acquire); }
  std::uint64_t unacceptedTailDrops() const;

 private:
  friend struct SlamPipelineTestAccess;
  friend class FrontendRuntime;
  friend class BackendRuntime;
  static void reportCallbackFailure();
  template <class Callback, class... Args>
  static void invokeCallback(const Callback& callback, Args&&... args) {
    if (!callback) return;
    try { callback(std::forward<Args>(args)...); }
    catch (...) { reportCallbackFailure(); }
  }
  bool waitForCapacity();
  void transfer(LidarFrame&&);
  void requestReset();
  void thd_mapping();
  void recordFailure(std::exception_ptr);
  const SapphireParameters parameters_;
  const Callbacks output_;
  // Owners outlive both workers; no worker may call either owner after shutdown.
  std::unique_ptr<BackendRuntime> backend_;
  std::unique_ptr<FrontendRuntime> frontend_;
  mutable std::mutex failure_mutex_;
  std::exception_ptr producer_failure_;
  std::mutex marginal_mutex_;
  std::condition_variable marginal_cv_;
  std::deque<LidarFrame, Eigen::aligned_allocator<LidarFrame>> marginal_frames_, reset_tail_;
  int reset_flag_ = 0;
  std::atomic_bool failed_{false}, stopping_{false}, odometry_done_{false};
  std::thread odometry_thread_, mapping_thread_;
};

}  // namespace sapphire
