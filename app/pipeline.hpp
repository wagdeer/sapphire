#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>

#include "backend/runtime.hpp"
#include "app/output.hpp"
#include "frontend/runtime.hpp"
namespace sapphire {
// Composition and lifecycle only. The two runtimes own all algorithm state.
class SlamPipeline {
 public:
  explicit SlamPipeline(SapphireParameters, OutputSink = {}, std::optional<std::pair<int, Eigen::Isometry3d>> = std::nullopt);
  ~SlamPipeline();
  SlamPipeline(const SlamPipeline&) = delete;
  SlamPipeline& operator=(const SlamPipeline&) = delete;
  bool push_imu(ImuMeas imu) { return frontend_->push_imu(std::move(imu)); }
  bool push_lidar(double t, std::vector<LidarPoint>&& cloud, std::optional<double> end = std::nullopt) {
    return frontend_->push_lidar(t, std::move(cloud), end);
  }
  bool push_lidar(double t, const std::vector<LidarPoint>& cloud, std::optional<double> end = std::nullopt) {
    if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) {
      notifyInputLoss();
      return false;
    }
    auto owned = cloud;
    return push_lidar(t, std::move(owned), end);
  }
  bool push_image(ImageMeas image) { return frontend_->push_image(std::move(image)); }
  void setAttachmentTarget(int target, const Eigen::Isometry3d& seed) { backend_->setAttachmentTarget(target, seed); }
  void retryContinuation(std::uint64_t sequence) { backend_->retryContinuation(sequence); }
  void invalidateOdometryDomain();
  void notifyInputLoss();
  std::exception_ptr producerFailure() const;
  ContinuationProgress continuationProgress() const;
  void shutdown();
  ContinuationProgress drain();
  void close();
  bool captureReady(bool try_only, std::string& uuid, std::uint64_t& revision, std::uint64_t& generation, std::uint64_t& source, double& stamp,
                    Eigen::Isometry3d& correction, Eigen::Isometry3d& pose) const {
    return backend_->captureReady(try_only, uuid, revision, generation, source, stamp, correction, pose);
  }
  std::shared_ptr<const backend::NavimapWorker::Map> captureNavigation(const std::string& uuid, std::uint64_t revision) const {
    return backend_->captureNavigation(uuid, revision);
  }
  bool failed() const { return failed_.load(std::memory_order_acquire); }
  std::uint64_t unacceptedTailDrops() const { return backend_->unacceptedTailDrops(); }

 private:
  friend struct SlamPipelineTestAccess;
  bool waitForCapacity();
  void transfer(LidarFrame&&);
  void requestReset();
  void thd_mapping();
  void recordFailure(std::exception_ptr);
  const SapphireParameters parameters_;
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
using SapphirePipeline = SlamPipeline;
}  // namespace sapphire
