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
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
  return stream.str();
}

}  // namespace
BackendRuntime::BackendRuntime(const SapphireParameters &parameters, const SlamPipeline::Callbacks &output, std::optional<std::pair<int, Eigen::Isometry3d>> attachment)
    : parameters_(parameters), output_(output), builder_(parameters_.pose_graph.submap_travel_distance, parameters_.pose_graph.submap_max_point_range), attachment_(std::move(attachment)) {
  tools::configure({});
  filename_ = initial_filename_ = current_time_filename();
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
  if (!parameters_.pose_graph.enabled && parameters_.pose_graph.map_mode == "new") return;
  if (auto built_node = builder_.push(std::move(frame))) submitNode(std::move(*built_node));
}
void BackendRuntime::finish() {
  if (failed()) return;
  if (auto tail = builder_.flush()) submitNode(std::move(*tail));
}
void BackendRuntime::resetSession() {
  builder_.clear();
  next_submission_sequence_ = 0;
  filename_ = initial_filename_ + std::to_string(++session_id_);
  create_pose_graph();
}
void BackendRuntime::stop() {
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    stopping_.store(true);
  }
  attachment_cv_.notify_all();
}
void BackendRuntime::invalidateOdometryDomain() {
  output_domain_current_.store(false);
  failed_.store(true);
  stop();
  SlamPipeline::invokeCallback(output_.ready_changed);
}
std::string BackendRuntime::backend_database_path() const {
  if (!parameters_.pose_graph.database_path.empty()) return parameters_.pose_graph.database_path;
  return (std::filesystem::path(parameters_.general.save_path) / filename_ / "map.db").string();
}

void BackendRuntime::create_pose_graph() {
  SlamPipeline::invokeCallback(output_.owner_reset, true);
  try {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) pose_graph_->close();
    pose_graph_.reset();
    progress_ = {};
    progress_.generation = static_cast<std::uint64_t>(session_id_);
    closed_ = false;
    attached_ = parameters_.pose_graph.map_mode == "new";
    if (parameters_.pose_graph.enabled) {
      const auto path = backend_database_path();
      if (parameters_.pose_graph.map_mode == "new" && std::filesystem::exists(path))
        throw std::runtime_error("New-map destination already exists: " + path);
      if (parameters_.pose_graph.map_mode != "new" && !std::filesystem::is_regular_file(path))
        throw std::runtime_error("Resume database does not exist: " + path);
      if (!std::filesystem::path(path).parent_path().empty()) std::filesystem::create_directories(std::filesystem::path(path).parent_path());
      auto config = parameters_.pose_graph.mapping;
      // These lifecycle/data contracts cannot be relaxed by tuning files.
      config.topograph.persistent_root_frames = true;
      config.topograph.reduce_graph = false;
      pose_graph_ = std::make_unique<Mapping>();
      pose_graph_->init(config, path);
      if (parameters_.pose_graph.map_mode == "resume") pose_graph_->triggerNewMap();
    }
    output_domain_current_.store(true);
  } catch (...) {
    invalidateOdometryDomain();
    SlamPipeline::invokeCallback(output_.owner_reset, false);
    throw;
  }
  SlamPipeline::invokeCallback(output_.owner_reset, false);
}

void BackendRuntime::setAttachmentTarget(int target, const Eigen::Isometry3d &seed) {
  if (target <= 0 || !seed.matrix().allFinite()) throw std::invalid_argument("Invalid attachment target/seed");
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    attachment_ = std::make_pair(target, seed);
    ++attachment_attempt_;
  }
  attachment_cv_.notify_all();
}
void BackendRuntime::submitNode(NodeBuilder::Result built_node) {
  if (!built_node.node) throw std::invalid_argument("Missing built node");
  const auto odom = built_node.node->getPose();
  const double stamp = built_node.node->getStamp();
  const auto sequence = next_submission_sequence_++;
  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  if (!pose_graph_ || failed()) return;
  std::optional<std::pair<int, lidar::Alignment>> attachment;
  while (!attached_) {
    std::pair<int, Eigen::Isometry3d> request;
    std::uint64_t attempt;
    {
      std::lock_guard<std::mutex> guard(attachment_mutex_);
      if (!attachment_) throw std::logic_error("Missing attachment");
      request = *attachment_;
      attempt = attachment_attempt_;
    }
    std::string rejection;
    try {
      const auto target = pose_graph_->getNodeCopy(request.first);
      auto aligned =
          lidar::align(target.lidar_cloud(), built_node.node->lidar_cloud(), sapphire::SE3f(Eigen::Matrix4f(request.second.matrix().cast<float>())),
                                pose_graph_->getParameters().registration);
      if (aligned.T_target_query.isNull())
        rejection = aligned.failure;
      else
        attachment = std::make_pair(request.first, std::move(aligned));
    } catch (const std::exception &error) {
      rejection = error.what();
    }
    ++progress_.association_attempts;
    if (attachment) {
      progress_.association_pending = 0;
      progress_.cancellation_reason.clear();
      break;
    }
    // No backend admission has occurred. Retain this exact frozen producer head
    // and wait for a changed target/seed; failed geometric preparation is retryable.
    progress_.head = SlamPipeline::ContinuationHead::Retryable;
    progress_.head_sequence = sequence;
    progress_.commit_outcome = SlamPipeline::CommitOutcome::NotCommitted;
    progress_.association_pending = 1;
    progress_.cancellation_reason = "Resume LiDAR preparation rejected: " + rejection;
    lock.unlock();
    SlamPipeline::invokeCallback(output_.ready_changed);
    {
      std::unique_lock<std::mutex> guard(attachment_mutex_);
      attachment_cv_.wait(guard, [&] { return stopping_.load() || failed() || attachment_attempt_ != attempt; });
    }
    lock.lock();
    if (stopping_.load() || failed()) {
      ++unaccepted_tail_drops_;
      progress_.association_pending = 0;
      progress_.cancellation_reason = "Stopped with an unaccepted frozen resume head";
      return;
    }
  }
  progress_.head = SlamPipeline::ContinuationHead::Processing;
  progress_.head_sequence = sequence;
  ++progress_.accepted;
  progress_.commit_outcome = SlamPipeline::CommitOutcome::Unknown;
  try {
    sapphire::Mat66_t<double> covariance = sapphire::Mat66_t<double>::Identity();
    // Preserve Sapphire's previous fixed odometry factor model. The frontend's
    // inverse Hessian cross-block diagonal is not a calibrated pose covariance.
    const auto noise = pose_graph_->getParameters().registration;
    covariance.diagonal().head<3>().array() = noise.lidar_odometry_translation_sigma_m * noise.lidar_odometry_translation_sigma_m;
    covariance.diagonal().tail<3>().array() = noise.lidar_odometry_rotation_sigma_rad * noise.lidar_odometry_rotation_sigma_rad;
    if (!pose_graph_->process_node(std::move(built_node.node), covariance)) throw std::runtime_error("Backend rejected built LiDAR node");
    if (attachment) {
      const int query = pose_graph_->getLastLocationId();
      const auto &alignment = attachment->second;
      Link edge(attachment->first, query, Link::kLoopClosure, alignment.T_target_query, alignment.covariance->inverse());
      std::map<int, sapphire::SE3f> poses;
      std::multimap<int, Link> links;
      pose_graph_->getCompleteGraph(poses, links);
      poses[query] = poses.at(edge.from()).compose(edge.transform());
      links.emplace(edge.from(), edge);
      pose_graph_->commit_loops({edge}, pose_graph_->solve_graph(poses, links), links);
      attached_ = true;
    }
    for (const auto &sample : built_node.trajectory) {
      pose_graph_->append_lidar_odometry(sample.timestamp, sapphire::SE3f(Eigen::Matrix4f(sample.T_odom_base.matrix().cast<float>())));
    }
    pose_graph_->checkpoint();
    progress_.ready_revision = pose_graph_->getLastLocationId();
    progress_.completed_revision = progress_.ready_revision;
    progress_.last_completed = sequence;
    ++progress_.completed;
    progress_.commit_outcome = SlamPipeline::CommitOutcome::Committed;
    progress_.head_sequence.reset();
    progress_.head = SlamPipeline::ContinuationHead::Idle;
    if (pose_graph_->isMapAligned()) {
      progress_.map_id = pose_graph_->getRootFrameId();
      progress_.ready_available = true;
      ready_correction_.matrix() = pose_graph_->getMapCorrection().matrix().cast<double>();
      ready_pose_.matrix() = pose_graph_->getMapCorrection().compose(odom).matrix().cast<double>();
      ready_timestamp_ = stamp;
      attached_ = true;
    }
  } catch (...) {
    failed_.store(true);
    output_domain_current_.store(false);
    stop();
    progress_.first_failure = std::current_exception();
    progress_.head = SlamPipeline::ContinuationHead::Failed;
    progress_.ready_available = false;
    progress_.commit_outcome = SlamPipeline::CommitOutcome::Unknown;
    throw;
  }
  lock.unlock();
  SlamPipeline::invokeCallback(output_.ready_changed);
}
SlamPipeline::ContinuationProgress BackendRuntime::continuationProgress() const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  return progress_;
}
void BackendRuntime::retryContinuation(std::uint64_t source_sequence) {
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (failed() || progress_.head != SlamPipeline::ContinuationHead::Retryable || progress_.head_sequence != source_sequence)
      throw std::logic_error("Only an uncommitted retained resume head can be retried");
  }
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    ++attachment_attempt_;
  }
  attachment_cv_.notify_all();
}
SlamPipeline::ContinuationProgress BackendRuntime::drain() {
  if (failed()) throw std::runtime_error("Pipeline failed before drain");
  if (unaccepted_tail_drops_.load()) throw std::runtime_error("Stopped with unaccepted frozen backend input");
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) {
      pose_graph_->checkpoint();
    }
  }
  const auto progress = continuationProgress();
  if (parameters_.pose_graph.map_mode == "resume" && progress.accepted && !progress.ready_available)
    throw std::runtime_error("Resume completed as an unlocalized component; no world-frame output is available");
  return progress;
}
void BackendRuntime::close() {
  stop();
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
  if (failed() || closed_ || !output_domain_current_.load() || !progress_.ready_available) return std::nullopt;
  timestamp = ready_timestamp_;
  correction = ready_correction_;
  pose = ready_pose_;
  return progress_;
}
}  // namespace sapphire
