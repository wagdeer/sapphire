#include "slam.hpp"
#include <spdlog/spdlog.h>
#include "pipeline/backend.hpp"
#include "pipeline/frontend.hpp"

#include <malloc.h>
#include <omp.h>

#include <stdexcept>
namespace sapphire {
namespace {
SapphireParameters validated(SapphireParameters parameters) {
  validate_parameters(parameters);
  if (parameters.pose_graph.map_mode != "new" && parameters.pose_graph.map_mode != "resume") {
    throw std::invalid_argument("Supported modes are new and automatic resume");
  }
  return parameters;
}

}  // namespace
SlamPipeline::SlamPipeline(SapphireParameters parameters, SlamPipeline::Callbacks output)
    : parameters_(validated(std::move(parameters))), output_(std::move(output)) {
  cv::setNumThreads(1);
  omp_set_num_threads(1);
  backend_ = std::make_unique<BackendRuntime>(parameters_, output_);
  FrontendRuntime::Handoff handoff;
  handoff.wait_for_capacity = [this] { return waitForCapacity(); };
  handoff.lidar = [this](LidarFrame&& frame) { transfer(std::move(frame)); };
  handoff.seal = [this](LidarFrame&& frame) {
    if (!waitForCapacity(true)) return false;
    transfer(std::move(frame));
    return true;
  };
  handoff.reset = [this](std::uint64_t generation) { return requestReset(generation); };
  handoff.suspend_output = [this] { backend_->requestRecovery(); marginal_cv_.notify_all(); };
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
ScanAdmission SlamPipeline::push_lidar(double t, std::vector<LidarPoint>&& cloud, std::optional<double> end) {
  return frontend_->push_lidar(t, std::move(cloud), end);
}
ScanAdmission SlamPipeline::push_lidar(double t, const std::vector<LidarPoint>& cloud, std::optional<double> end) {
  if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) {
    return rejectLidar(end.value_or(t), RecoveryReason::InvalidLidar);
  }
  auto owned = cloud;
  return push_lidar(t, std::move(owned), end);
}
ScanAdmission SlamPipeline::rejectLidar(double stamp, RecoveryReason reason) { return frontend_->rejectLidar(stamp, reason); }
RecoveryProgress SlamPipeline::recoveryProgress() const { return frontend_->recoveryProgress(); }
std::optional<SlamPipeline::ContinuationProgress> SlamPipeline::captureReady(
    bool try_only, double& stamp, Eigen::Isometry3d& correction, Eigen::Isometry3d& pose) const {
  return backend_->captureReady(try_only, stamp, correction, pose);
}
void SlamPipeline::reportCallbackFailure() {
  try { throw; }
  catch (const std::exception& error) { spdlog::error("Output callback failed: {}", error.what()); }
  catch (...) { spdlog::error("Output callback failed with an unknown exception"); }
}

bool SlamPipeline::waitForCapacity(bool sealing) {
  std::unique_lock<std::mutex> lock(marginal_mutex_);
  while (marginal_frames_.size() >= 2 && !failed() && (sealing || (!stopping_.load() && !frontend_->recoveryProgress().pending))) {
    marginal_cv_.wait_for(lock, std::chrono::milliseconds(100));
    lock.unlock();
    if (!sealing) frontend_->checkInputHealth();
    lock.lock();
  }
  return !failed() && (sealing || (!stopping_.load() && !frontend_->recoveryProgress().pending));
}
void SlamPipeline::transfer(LidarFrame&& frame) {
  {
    std::lock_guard<std::mutex> lock(marginal_mutex_);
    // One producer owns the reservation from waitForCapacity through this move.
    if (marginal_frames_.size() >= 2) throw std::logic_error("Frozen handoff capacity exceeded");
    frame.generation=reset_requested_;
    marginal_frames_.emplace_back(std::move(frame));
  }
  marginal_cv_.notify_one();
}
bool SlamPipeline::requestReset(std::uint64_t generation) {
  std::unique_lock<std::mutex> lock(marginal_mutex_);
  if (stopping_.load() || failed()) return false;
  if(generation<reset_requested_)throw std::logic_error("Odometry generation moved backwards");
  reset_requested_=generation;
  backend_->requestSession(generation);
  marginal_cv_.notify_all();
  return true; // Boundary admission, not a wait for registration or persistence.

}
void SlamPipeline::shutdown() {
  if (frontend_) frontend_->stop();
  stopping_.store(true, std::memory_order_release);
  { std::lock_guard<std::mutex> lock(marginal_mutex_); }
  marginal_cv_.notify_all();
  if (odometry_thread_.joinable()) odometry_thread_.join();
  if (mapping_thread_.joinable()) mapping_thread_.join();
}
void SlamPipeline::thd_mapping() {
  omp_set_num_threads(1);
  for (;;) {
    std::optional<LidarFrame> frame;
    std::uint64_t reset = 0, generation = 0;
    {
      std::unique_lock<std::mutex> lock(marginal_mutex_);
      const auto ready=[this] { return reset_requested_ > reset_completed_ || !marginal_frames_.empty() || (stopping_.load() && odometry_done_.load()); };
      if(backend_->matchingPending())marginal_cv_.wait_for(lock,std::chrono::milliseconds(20),ready);
      else marginal_cv_.wait(lock,ready);
      if(!marginal_frames_.empty() && marginal_frames_.front().generation!=reset_completed_) {
        reset=marginal_frames_.front().generation;generation=reset;
      } else if (!marginal_frames_.empty()) {
        frame.emplace(std::move(marginal_frames_.front()));
        marginal_frames_.pop_front();
        marginal_cv_.notify_all();
      } else if(reset_requested_>reset_completed_) {
        reset=reset_requested_;generation=reset;
      } else if (stopping_.load() && odometry_done_.load())break;
    }
    if (failed()) break;
    if (reset) {
      backend_->resetSession(generation);
      { std::lock_guard<std::mutex> lock(marginal_mutex_); reset_completed_ = reset; }
      marginal_cv_.notify_all();
      continue;
    }
    backend_->pollMatching();
    if (!frame || (!parameters_.pose_graph.enabled && parameters_.pose_graph.map_mode == "new")) continue;
    backend_->process(std::move(*frame));
  }

  if (failed()) return;
  backend_->finish();
  backend_->pollMatching(true);
  malloc_trim(0);
}
void SlamPipeline::recordFailure(std::exception_ptr error) {
  {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    if (!producer_failure_) producer_failure_ = error;
  }
  if (frontend_) frontend_->markFailed();
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
void SlamPipeline::notifyInputLoss(RecoveryReason reason) { frontend_->requestRecovery(reason); }
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
  auto result = backend_->drain();
  return result;
}
void SlamPipeline::close() {
  shutdown();
  backend_->close();
}
}  // namespace sapphire
