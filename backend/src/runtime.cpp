#include "backend/runtime.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "backend/memory.hpp"
#include "backend/sensor/lidar/input.hpp"
#include "backend/sensor/lidar/registration.hpp"
#include "backend/tools/logging.hpp"
namespace sapphire {
using detail::invoke_output;
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
BackendRuntime::BackendRuntime(const SapphireParameters &parameters, OutputSink output, std::optional<std::pair<int, Eigen::Isometry3d>> attachment)
    : parameters_(parameters), output_(std::move(output)), builder_(parameters_.pose_graph), attachment_(std::move(attachment)) {
  backend::tools::configure({});
  filename_ = initial_filename_ = current_time_filename();
  save_path_ = parameters_.general.save_path;
  save_map_ = parameters_.general.save_map;
  if (!parameters_.pose_graph.database_path.empty()) create_pose_graph();
  if (save_map_ || (parameters_.pose_graph.enabled && parameters_.pose_graph.database_path.empty()))
    std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
  if (parameters_.pose_graph.database_path.empty()) create_pose_graph();
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
void BackendRuntime::process(LidarFrame &&frame, std::deque<ImageMeas> images) {
  if (failed()) return;
  if (!parameters_.pose_graph.enabled && parameters_.pose_graph.map_mode == "new") return;
  if (auto submap = builder_.push(std::move(frame), std::move(images))) submitSubmap(std::move(*submap));
}
void BackendRuntime::finish() {
  if (failed()) return;
  if (auto tail = builder_.flush()) submitSubmap(std::move(*tail));
}
void BackendRuntime::resetSession() {
  builder_.reset();
  filename_ = initial_filename_ + std::to_string(++session_id_);
  if (save_map_ || parameters_.pose_graph.enabled) std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
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
  invoke_output(output_.ready_changed);
}
std::string BackendRuntime::backend_database_path() const {
  if (!parameters_.pose_graph.database_path.empty()) return parameters_.pose_graph.database_path;
  return (std::filesystem::path(save_path_) / filename_ / "map.db").string();
}

void BackendRuntime::create_pose_graph() {
  invoke_output(output_.owner_reset, true);
  try {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) pose_graph_->close();
    pose_graph_.reset();
    navigation_.clear();
    ready_navigation_.reset();
    progress_ = {};
    closed_ = false;
    attached_ = parameters_.pose_graph.map_mode == "new";
    if (parameters_.pose_graph.enabled) {
      const auto path = backend_database_path();
      if (parameters_.pose_graph.map_mode == "new" && std::filesystem::exists(path))
        throw std::runtime_error("New-map destination already exists: " + path);
      if (parameters_.pose_graph.map_mode != "new" && !std::filesystem::is_regular_file(path))
        throw std::runtime_error("Resume database does not exist: " + path);
      if (!std::filesystem::path(path).parent_path().empty()) std::filesystem::create_directories(std::filesystem::path(path).parent_path());
      backend::ParametersMap config{
          {"registration.feature.enabled", parameters_.pose_graph.visual.loopEnabled() ? "true" : "false"},
          {"registration.feature.method", "0"},
          {"archive.parallel_compression", "false"},
          {"topograph.persistent_root_frames", "true"},
          {"memory.recent_nodes", "3"},
          {"memory.resident_budget_bytes",
           std::to_string((parameters_.pose_graph.storage.cloud_cache_mb + parameters_.pose_graph.storage.scene_cache_mb) * 1024 * 1024)},
          {"node.refresh_enabled", "false"},
          {"node.merge_previous", "false"},
          {"relocalizer.spatial.radius_m", std::to_string(parameters_.pose_graph.submap_travel_distance * 2)},
          {"relocalizer.spatial.max_graph_hops", "0"},
          {"node.navimap.frame_grid_enabled", "true"},
          {"node.navimap.max_depth_m", std::to_string(parameters_.pose_graph.submap_max_point_range)}};
      for (const auto &[key, value] : parameters_.pose_graph.backend_parameters) config[key] = value;
      // These lifecycle/data contracts cannot be relaxed by tuning files.
      config["registration.feature.enabled"] = parameters_.pose_graph.visual.loopEnabled() ? "true" : "false";
      config["archive.parallel_compression"] = "false";
      config["topograph.persistent_root_frames"] = "true";
      config["topograph.reduce_graph"] = "false";
      pose_graph_ = std::make_unique<backend::Mapping>();
      pose_graph_->init(config, path);
      pose_graph_->restore_localmaps(true, parameters_.pose_graph.map_mode != "new");
      navigation_.configure(config, {}, false);
      if (parameters_.pose_graph.map_mode == "resume") pose_graph_->triggerNewMap();
    }
    owner_generation_ = session_id_;
    progress_.generation = owner_generation_;
    output_domain_current_.store(true);
  } catch (...) {
    invalidateOdometryDomain();
    invoke_output(output_.owner_reset, false);
    throw;
  }
  invoke_output(output_.owner_reset, false);
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
void BackendRuntime::submitSubmap(SubmapFrame submap) {
  auto data = makeBackendInput(submap, parameters_.pose_graph.visual);
  const backend::StateGroupf odom(Eigen::Matrix4f(submap.T_odom_base().matrix().cast<float>()));
  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  if (!pose_graph_ || failed()) return;
  std::optional<std::pair<int, backend::lidar::Alignment>> attachment;
  while (!attached_ && !parameters_.pose_graph.automatic_attachment) {
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
      const auto target = pose_graph_->getNodeCopy(request.first, false, false);
      auto aligned =
          backend::lidar::align(target.lidar_cloud(), data.lidar_cloud, backend::StateGroupf(Eigen::Matrix4f(request.second.matrix().cast<float>())),
                                backend::RegistrationParams{}.withOverrides(pose_graph_->getParameters()));
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
    progress_.head = ContinuationHead::Retryable;
    progress_.head_sequence = submap.id();
    progress_.last_commit_outcome = CommitOutcome::NotCommitted;
    progress_.committed_outcome_known = true;
    progress_.association_pending = 1;
    progress_.cancellation_reason = "Resume LiDAR preparation rejected: " + rejection;
    lock.unlock();
    invoke_output(output_.ready_changed);
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
  progress_.head = ContinuationHead::Processing;
  progress_.head_sequence = submap.id();
  progress_.last_accepted = submap.id();
  ++progress_.accepted;
  progress_.committed_outcome_known = false;
  try {
    backend::Mat66_t<double> covariance = backend::Mat66_t<double>::Identity();
    // Preserve Sapphire's previous fixed odometry factor model. The frontend's
    // inverse Hessian cross-block diagonal is not a calibrated pose covariance.
    const auto noise = backend::RegistrationParams{}.withOverrides(pose_graph_->getParameters());
    covariance.diagonal().head<3>().array() = noise.lidar_odometry_translation_sigma_m * noise.lidar_odometry_translation_sigma_m;
    covariance.diagonal().tail<3>().array() = noise.lidar_odometry_rotation_sigma_rad * noise.lidar_odometry_rotation_sigma_rad;
    if (!pose_graph_->process_lidar(data, odom, covariance)) throw std::runtime_error("Backend rejected frozen LiDAR submap");
    if (attachment) {
      const int query = pose_graph_->getLastLocationId();
      const auto &alignment = attachment->second;
      backend::Link edge(attachment->first, query, backend::Link::kLoopClosure, alignment.T_target_query, alignment.covariance->inverse());
      std::map<int, backend::StateGroupf> poses;
      std::multimap<int, backend::Link> links;
      pose_graph_->getCompleteGraph(poses, links);
      poses[query] = poses.at(edge.from()).compose(edge.transform());
      links.emplace(edge.from(), edge);
      pose_graph_->commit_loops({edge}, pose_graph_->solve_graph(poses, links), links);
      attached_ = true;
    }
    for (size_t i = 1; i < submap.odom_poses().size(); ++i) {
      const auto &sample = submap.odom_poses()[i];
      pose_graph_->append_lidar_odometry(sample.timestamp, backend::StateGroupf(Eigen::Matrix4f(sample.T_odom_base.matrix().cast<float>())));
    }
    pose_graph_->checkpoint();
    if (pose_graph_->isMapAligned()) {
      navigation_.render(pose_graph_->getWorldOptimizedPoses(), pose_graph_->localmaps(), true, false, pose_graph_->swap_results(),
                         int64_t(submap.timestamp() * 1e9));
      ready_navigation_ = navigation_.navimap();
    }
    progress_.ready_revision = pose_graph_->getLastLocationId();
    progress_.committed_revision = progress_.ready_revision;
    progress_.completed_revision = progress_.ready_revision;
    progress_.last_completed = submap.id();
    ++progress_.completed;
    progress_.last_commit_outcome = CommitOutcome::Committed;
    progress_.committed_outcome_known = true;
    progress_.head_sequence.reset();
    progress_.head = ContinuationHead::Idle;
    if (pose_graph_->isMapAligned()) {
      progress_.map_uuid = pose_graph_->getRootFrameId();
      progress_.ready_available = true;
      ready_correction_.matrix() = pose_graph_->getMapCorrection().matrix().cast<double>();
      ready_pose_.matrix() = pose_graph_->getMapCorrection().compose(odom).matrix().cast<double>();
      ready_timestamp_ = submap.timestamp();
      if (!progress_.root_revision) {
        progress_.root_revision = progress_.ready_revision;
        progress_.root_source = submap.id();
      }
      progress_.automatically_attached = parameters_.pose_graph.automatic_attachment;
      attached_ = true;
    }
  } catch (...) {
    failed_.store(true);
    output_domain_current_.store(false);
    stop();
    progress_.first_failure = std::current_exception();
    progress_.head = ContinuationHead::Failed;
    progress_.ready_available = false;
    progress_.last_commit_outcome = CommitOutcome::Unknown;
    throw;
  }
  lock.unlock();
  invoke_output(output_.ready_changed);
}
ContinuationProgress BackendRuntime::continuationProgress() const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  auto result = progress_;
  return result;
}
void BackendRuntime::retryContinuation(std::uint64_t source_sequence) {
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (failed() || progress_.head != ContinuationHead::Retryable || progress_.head_sequence != source_sequence)
      throw std::logic_error("Only an uncommitted retained resume head can be retried");
  }
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    ++attachment_attempt_;
  }
  attachment_cv_.notify_all();
}
ContinuationProgress BackendRuntime::drain() {
  if (failed()) throw std::runtime_error("Pipeline failed before drain");
  if (unaccepted_tail_drops_.load()) throw std::runtime_error("Stopped with unaccepted frozen backend input");
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) {
      pose_graph_->seal_localmap();
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
std::shared_ptr<const backend::NavimapWorker::Map> BackendRuntime::captureNavigation(const std::string &uuid,
                                                                                                  std::uint64_t revision) const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  if (failed() || closed_ || !progress_.ready_available || progress_.map_uuid != uuid || progress_.ready_revision != revision) return {};
  return ready_navigation_;
}
bool BackendRuntime::captureReady(bool try_only, std::string &uuid, std::uint64_t &revision, std::uint64_t &generation, std::uint64_t &source,
                                  double &timestamp, Eigen::Isometry3d &correction, Eigen::Isometry3d &pose) const {
  std::unique_lock<std::mutex> lock(pose_graph_mutex_, std::defer_lock);
  if (try_only) {
    if (!lock.try_lock()) return false;
  } else
    lock.lock();
  if (failed() || closed_ || !output_domain_current_.load() || !progress_.ready_available) return false;
  uuid = progress_.map_uuid;
  revision = progress_.ready_revision;
  generation = owner_generation_;
  source = *progress_.last_completed;
  timestamp = ready_timestamp_;
  correction = ready_correction_;
  pose = ready_pose_;
  return true;
}
}  // namespace sapphire
