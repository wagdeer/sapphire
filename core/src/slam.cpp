#include "slam.hpp"
#include <spdlog/spdlog.h>
#include "pipeline/backend.hpp"
#include "pipeline/frontend.hpp"

#include <malloc.h>
#include <omp.h>

#include <stdexcept>
namespace sapphire {
namespace {
SapphireParameters validated(SapphireParameters parameters, bool attachment) {
  validate_parameters(parameters);
  if (parameters.pose_graph.map_mode != "new" && !(parameters.pose_graph.map_mode == "resume" && attachment)) {
    throw std::invalid_argument("Resume requires an explicit LiDAR target/seed ");
  }
  return parameters;
}

}  // namespace
SlamPipeline::SlamPipeline(SapphireParameters parameters, SlamPipeline::Callbacks output, std::optional<std::pair<int, Eigen::Isometry3d>> attachment)
    : parameters_(validated(std::move(parameters), bool(attachment))), output_(std::move(output)) {
  cv::setNumThreads(1);
  omp_set_num_threads(1);
  backend_ = std::make_unique<BackendRuntime>(parameters_, output_, std::move(attachment));
  FrontendRuntime::Handoff handoff;
  handoff.wait_for_capacity = [this] { return waitForCapacity(); };
  handoff.lidar = [this](LidarFrame&& frame) { transfer(std::move(frame)); };
  handoff.reset = [this] { requestReset(); };
  handoff.invalidate_domain = [this] { invalidateOdometryDomain(); };
  handoff.suspend_output = [this] { backend_->suspendOutput(); };
  frontend_ = std::make_unique<FrontendRuntime>(parameters_, output_, std::move(handoff));
  try {
    odometry_thread_ = std::thread([this] {
      try {
        frontend_->run();
      } catch (...) {
        recordFailure(std::current_exception());
      }
      {
        std::lock_guard<std::mutex> lock(marginal_mutex_);
        odometry_done_.store(true);
      }
      marginal_cv_.notify_all();
    });
    mapping_thread_ = std::thread([this] {
      try {
        thd_mapping();
      } catch (...) {
        recordFailure(std::current_exception());
      }
    });
  } catch (...) {
    shutdown();
    throw;
  }
}
SlamPipeline::~SlamPipeline() {
  shutdown();
  try {
    close();
  } catch (const std::exception& e) {
    spdlog::error("Backend close failed: {}", e.what());
  } catch (...) {
    spdlog::error("Backend close failed with unknown exception");
  }
}
bool SlamPipeline::push_imu(ImuMeas imu) { return frontend_->push_imu(std::move(imu)); }
bool SlamPipeline::push_lidar(double t, std::vector<LidarPoint>&& cloud, std::optional<double> end) {
  return frontend_->push_lidar(t, std::move(cloud), end);
}
bool SlamPipeline::push_lidar(double t, const std::vector<LidarPoint>& cloud, std::optional<double> end) {
  if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) {
    notifyInputLoss();
    return false;
  }
  auto owned = cloud;
  return push_lidar(t, std::move(owned), end);
}
void SlamPipeline::setAttachmentTarget(int target, const Eigen::Isometry3d& seed) { backend_->setAttachmentTarget(target, seed); }
void SlamPipeline::retryContinuation(std::uint64_t sequence) { backend_->retryContinuation(sequence); }
std::uint64_t SlamPipeline::unacceptedTailDrops() const { return backend_->unacceptedTailDrops(); }
std::optional<SlamPipeline::ContinuationProgress> SlamPipeline::captureReady(
    bool try_only, double& stamp, Eigen::Isometry3d& correction, Eigen::Isometry3d& pose) const {
  return backend_->captureReady(try_only, stamp, correction, pose);
}
void SlamPipeline::reportCallbackFailure() {
  try { throw; }
  catch (const std::exception& error) { spdlog::error("Output callback failed: {}", error.what()); }
  catch (...) { spdlog::error("Output callback failed with an unknown exception"); }
}

bool SlamPipeline::waitForCapacity() {
  std::unique_lock<std::mutex> lock(marginal_mutex_);
  marginal_cv_.wait(lock, [this] { return marginal_frames_.size() < 2 || stopping_.load() || failed(); });
  return !stopping_.load() && !failed();
}
void SlamPipeline::transfer(LidarFrame&& frame) {
  {
    std::lock_guard<std::mutex> lock(marginal_mutex_);
    // One producer owns the reservation from waitForCapacity through this move.
    if (marginal_frames_.size() >= 2) throw std::logic_error("Frozen handoff capacity exceeded");
    marginal_frames_.emplace_back(std::move(frame));
  }
  marginal_cv_.notify_one();
}
void SlamPipeline::requestReset() {
  {
    std::lock_guard<std::mutex> lock(marginal_mutex_);
    reset_tail_.swap(marginal_frames_);
    reset_flag_ = 1;
  }
  marginal_cv_.notify_one();
}
void SlamPipeline::shutdown() {
  if (frontend_) frontend_->stop();
  stopping_.store(true, std::memory_order_release);
  if (backend_) backend_->stop();
  { std::lock_guard<std::mutex> lock(marginal_mutex_); }
  marginal_cv_.notify_all();
  if (odometry_thread_.joinable()) odometry_thread_.join();
  if (mapping_thread_.joinable()) mapping_thread_.join();
}
void SlamPipeline::thd_mapping() {
  omp_set_num_threads(1);
  for (;;) {
    std::deque<LidarFrame, Eigen::aligned_allocator<LidarFrame>> reset_tail;
    std::optional<LidarFrame> frame;
    bool reset = false;
    {
      std::unique_lock<std::mutex> lock(marginal_mutex_);
      marginal_cv_.wait(lock, [this] { return reset_flag_ == 1 || !marginal_frames_.empty() || (stopping_.load() && odometry_done_.load()); });
      if (reset_flag_ == 1) {
        reset_flag_ = 0;
        reset_tail.swap(reset_tail_);
        reset = true;
      } else if (!marginal_frames_.empty()) {
        frame.emplace(std::move(marginal_frames_.front()));
        marginal_frames_.pop_front();
        marginal_cv_.notify_all();
      } else if (stopping_.load() && odometry_done_.load())
        break;
    }
    if (failed()) break;
    if (reset) {
      backend_->resetSession();
      continue;
    }
    if (!frame || (!parameters_.pose_graph.enabled && parameters_.pose_graph.map_mode == "new")) continue;
    backend_->process(std::move(*frame));
  }

  if (failed()) return;
  backend_->finish();
  malloc_trim(0);
}
void SlamPipeline::recordFailure(std::exception_ptr error) {
  {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    if (!producer_failure_) producer_failure_ = error;
  }
  invalidateOdometryDomain();
}
void SlamPipeline::invalidateOdometryDomain() {
  failed_.store(true);
  stopping_.store(true);
  if (frontend_) frontend_->stop();
  if (backend_) backend_->invalidateOdometryDomain();
  { std::lock_guard<std::mutex> lock(marginal_mutex_); }
  marginal_cv_.notify_all();
}
void SlamPipeline::notifyInputLoss() {
  if (parameters_.pose_graph.map_mode == "resume") invalidateOdometryDomain();
}
std::exception_ptr SlamPipeline::producerFailure() const {
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return producer_failure_;
}
SlamPipeline::ContinuationProgress SlamPipeline::continuationProgress() const {
  auto result = backend_->continuationProgress();
  if (auto error = producerFailure()) result.first_failure = error;
  return result;
}
SlamPipeline::ContinuationProgress SlamPipeline::drain() {
  shutdown();
  if (auto error = producerFailure()) std::rethrow_exception(error);
  if (failed()) throw std::runtime_error("Pipeline failed before drain");
  return backend_->drain();
}
void SlamPipeline::close() {
  shutdown();
  backend_->close();
}
}  // namespace sapphire
