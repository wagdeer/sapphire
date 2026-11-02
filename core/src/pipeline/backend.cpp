#include <spdlog/spdlog.h>
#include "pipeline/backend.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "database/memory.hpp"
#include "solver/icp_solver.hpp"
#include "tools/logging.hpp"
namespace sapphire {
namespace {
std::string current_time_filename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
  localtime_r(&now_time, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S") << "_"
         << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  return stream.str();
}

}  // namespace
BackendRuntime::BackendRuntime(const SapphireParameters &parameters, const SlamPipeline::Callbacks &output)
    : parameters_(parameters), output_(output), builder_(parameters_.pose_graph.submap_travel_distance) {
  tools::configure({});
  filename_ = current_time_filename();
  create_pose_graph();
  spdlog::info("filename: {}", filename_);
}
BackendRuntime::~BackendRuntime() {
  try {
    close();
  } catch (const std::exception &e) {
    spdlog::error("Backend close failed: {}", e.what());
  } catch (...) {
    spdlog::error("Backend close failed with unknown exception");
  }
}
void BackendRuntime::process(LidarFrame &&frame) {
  if (failed()) return;
  if (!parameters_.pose_graph.enabled && !resumeSession()) return;
  if (auto built_node = builder_.push(std::move(frame))) submitNode(std::move(*built_node));
}
void BackendRuntime::finish() {
  if (failed()) return;
  if (auto tail = builder_.flush()) submitNode(std::move(*tail));
}
void BackendRuntime::requestRecovery() {
  output_domain_current_.store(false);
  recovery_pending_.store(true);
}
void BackendRuntime::resetSession(std::uint64_t generation) {
  // Accepted frozen evidence still belongs to the old segment. Seal its short
  // tail before resetting odometry continuity; keep the archive and node IDs.
  finish();
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  if (session_id_ != generation) {
    if (pose_graph_) {
      pose_graph_->checkpoint();
      pose_graph_->triggerNewMap();
    }
    session_id_ = generation;
    next_submission_sequence_ = 0;
    progress_ = {};
    progress_.generation = generation;

  }
}
void BackendRuntime::invalidateOdometryDomain() {
  output_domain_current_.store(false);
  failed_.store(true);
  SlamPipeline::invokeCallback(output_.ready_changed);
}
std::string BackendRuntime::backend_database_path() const {
  if (!parameters_.pose_graph.database_path.empty()) return parameters_.pose_graph.database_path;
  return (std::filesystem::path(parameters_.general.save_path) / filename_ / "map.db").string();
}

void BackendRuntime::create_pose_graph() {
  // Initial construction owns its fence; recovery is fenced by the frontend
  // until its ordered session boundary has been admitted.
  const bool initial = session_id_ == 0;
  if (initial && output_.owner_reset) output_.owner_reset(true);
  try {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) pose_graph_->close();
    pose_graph_.reset();
    progress_ = {};
    progress_.generation = static_cast<std::uint64_t>(session_id_);
    closed_ = false;

    if (parameters_.pose_graph.enabled) {
      const auto path = backend_database_path();
      if (!resumeSession() && std::filesystem::exists(path))
        throw std::runtime_error("New-map destination already exists: " + path);
      if (resumeSession() && !std::filesystem::is_regular_file(path))
        throw std::runtime_error("Resume database does not exist: " + path);
      if (!std::filesystem::path(path).parent_path().empty()) std::filesystem::create_directories(std::filesystem::path(path).parent_path());
      auto config = parameters_.pose_graph.mapping;
      // These lifecycle/data contracts cannot be relaxed by tuning files.
      config.topograph.persistent_root_frames = true;
      config.topograph.reduce_graph = false;
      pose_graph_ = std::make_unique<Mapping>();
      pose_graph_->init(config, path);
      pose_graph_->enable_async_relocalization();
      if (resumeSession()) pose_graph_->triggerNewMap();
    }
    output_domain_current_.store(true);
  } catch (...) {
    invalidateOdometryDomain();
    if (initial && output_.owner_reset) output_.owner_reset(false);
    throw;
  }
  if (initial && output_.owner_reset) output_.owner_reset(false);
}

void BackendRuntime::submitNode(NodeBuilder::Result built_node) {
  if (!built_node.node) throw std::invalid_argument("Missing built node");
  const auto sequence = next_submission_sequence_++;
  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  if (!pose_graph_ || failed()) return;
  progress_.head = SlamPipeline::ContinuationHead::Processing;
  progress_.head_sequence = sequence;
  ++progress_.accepted;
  progress_.commit_outcome = SlamPipeline::CommitOutcome::Unknown;
  try {
    sapphire::Mat66_t<double> covariance = sapphire::Mat66_t<double>::Identity();
    // Batch odometry edges use the configured noise model.
    const auto noise = pose_graph_->getParameters().registration;
    covariance.diagonal().head<3>().array() = noise.lidar_odometry_translation_sigma_m * noise.lidar_odometry_translation_sigma_m;
    covariance.diagonal().tail<3>().array() = noise.lidar_odometry_rotation_sigma_rad * noise.lidar_odometry_rotation_sigma_rad;
    if (!pose_graph_->process_node(std::move(built_node.node), covariance)) throw std::runtime_error("Backend rejected built LiDAR node");
    for (const auto &sample : built_node.trajectory) {
      pose_graph_->append_lidar_odometry(sample.timestamp, sapphire::SE3f(Eigen::Matrix4f(sample.T_odom_base.matrix().cast<float>())));
    }
    pose_graph_->checkpoint();
    progress_.ready_revision = std::max(progress_.ready_revision+1,std::uint64_t(pose_graph_->getLastLocationId()));
    progress_.completed_revision = pose_graph_->getLastLocationId();
    progress_.last_completed = sequence;
    ++progress_.completed;
    progress_.commit_outcome = SlamPipeline::CommitOutcome::Committed;
    progress_.head_sequence.reset();
    progress_.head = SlamPipeline::ContinuationHead::Idle;
    refreshReady();
  } catch (...) {
    failed_.store(true);
    output_domain_current_.store(false);
    progress_.first_failure = std::current_exception();
    progress_.head = SlamPipeline::ContinuationHead::Failed;
    progress_.ready_available = false;
    progress_.commit_outcome = SlamPipeline::CommitOutcome::Unknown;
    throw;
  }
  lock.unlock();
  SlamPipeline::invokeCallback(output_.ready_changed);
}
void BackendRuntime::requestSession(std::uint64_t generation) {
  requested_generation_.store(generation);
  recovery_pending_.store(false);
  output_domain_current_.store(true);
}
bool BackendRuntime::matchingPending() const { return pose_graph_ && pose_graph_->relocalization_pending(); }
void BackendRuntime::refreshReady() {
  if(!pose_graph_->isMapAligned() || !progress_.completed)return;
  const auto node=pose_graph_->getNodeCopy(pose_graph_->getLastLocationId());
  progress_.map_id=pose_graph_->getRootFrameId();progress_.ready_available=true;
  ready_correction_.matrix()=pose_graph_->getMapCorrection().matrix().cast<double>();
  ready_pose_.matrix()=pose_graph_->getMapCorrection().compose(node.getPose()).matrix().cast<double>();
  ready_timestamp_=node.getStamp();
}
void BackendRuntime::pollMatching(bool wait) {
  bool changed=false;
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if(pose_graph_ && pose_graph_->poll_relocalization(wait)) {
      ++progress_.ready_revision;refreshReady();changed=true;
    }
  }
  if(changed)SlamPipeline::invokeCallback(output_.ready_changed);
}
SlamPipeline::ContinuationProgress BackendRuntime::continuationProgress() const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  auto result = progress_;
  if(!output_domain_current_.load() || recovery_pending_.load() || result.generation!=requested_generation_.load())result.ready_available=false;
  return result;
}
SlamPipeline::ContinuationProgress BackendRuntime::drain() {
  if (failed()) throw std::runtime_error("Pipeline failed before drain");
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) {
      pose_graph_->checkpoint();
    }
  }
  const auto progress = continuationProgress();
  return progress;
}
void BackendRuntime::close() {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  if (!closed_ && pose_graph_) pose_graph_->close();
  closed_ = true;
  progress_.ready_available = false;
}

std::optional<SlamPipeline::ContinuationProgress> BackendRuntime::captureReady(
    bool try_only, double &timestamp, Eigen::Isometry3d &correction, Eigen::Isometry3d &pose) const {
  std::unique_lock<std::mutex> lock(pose_graph_mutex_, std::defer_lock);
  if (try_only) {
    if (!lock.try_lock()) return std::nullopt;
  } else lock.lock();
  if (failed() || closed_ || !output_domain_current_.load() || recovery_pending_.load() || progress_.generation!=requested_generation_.load() || !progress_.ready_available) return std::nullopt;
  timestamp = ready_timestamp_;
  correction = ready_correction_;
  pose = ready_pose_;
  return progress_;
}
}  // namespace sapphire
