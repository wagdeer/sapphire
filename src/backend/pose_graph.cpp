#include <spdlog/spdlog.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/backend/submap.hpp>
#include <sapphire/backend/submap_arxiv.hpp>
#include <sapphire/config.hpp>
#include <sapphire/mapping/voxelmap.hpp>
#include <sapphire/odometry/voxel_filter.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

namespace sapphire {

class PoseGraphBackend::Impl {
 public:
  explicit Impl(const Config& config) : config_(config), arxiv_(checkedMapDirectory(config), config.pgo.loop_closure), submap_(arxiv_, config.odometry, config.submap, config.pgo) {
    if (config_.pgo.occupancy.enabled) {
      occupancy_directory_ = std::filesystem::path(config.io.map_directory) / "rays";
      if (!std::filesystem::create_directory(occupancy_directory_)) {
        throw std::runtime_error("map directory already contains ray scans");
      }
    }
    worker_ = std::thread([this] { workerLoop(); });
  }

  ~Impl() {
    try {
      finish();
    } catch (const std::exception& error) {
      spdlog::error("[pgo] submap processing failed: {}", error.what());
    }
  }

  void finish() {
    stop_.store(true, std::memory_order_release);
    wake_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (worker_error_) std::rethrow_exception(worker_error_);
  }

  bool addFrame(InertialFrame&& frame) {
    if (stop_.load(std::memory_order_acquire) || worker_failed_.load(std::memory_order_acquire)) return false;
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail - head_.load(std::memory_order_acquire) == queue_.size()) {
      ++dropped_;
      return false;
    }
    queue_[tail % queue_.size()] = std::move(frame);
    tail_.store(tail + 1, std::memory_order_release);
    wake_cv_.notify_one();
    return true;
  }

  void record(const OdomFrameSample& sample) {
    std::lock_guard lock(samples_mutex_);
    samples_.push_back(sample);
  }

  void setCorrection(const Isometry3d& correction, double stamp, const Eigen::Vector3d& gravity_odom) {
    std::lock_guard lock(correction_mutex_);
    correction_.graph_from_odom = correction;
    if (!gravity_odom.allFinite() || gravity_odom.squaredNorm() < 1.e-6) return;

    constexpr double smoothing_seconds = 1.0;
    if (gravity_stamp_) {
      const double weight = -std::expm1(-std::max(0., stamp - *gravity_stamp_) / smoothing_seconds);
      gravity_ += weight * (gravity_odom - gravity_);
    } else {
      gravity_ = gravity_odom;
    }
    gravity_stamp_ = stamp;

    // The solver and tracking maps keep their working frames. A single
    // rigid change of basis gives every public view a gravity-aligned Z.
    const Eigen::Vector3d gravity_graph = correction.linear() * gravity_;
    correction_.map_from_graph.linear() = Eigen::Quaterniond::FromTwoVectors(gravity_graph, -Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  Isometry3d correction() const {
    const auto frames = correctionFrames();
    return frames.map_from_graph * frames.graph_from_odom;
  }

  MapTrajectory mapTrajectory() const {
    MapTrajectory trajectory;
    const auto solution = std::atomic_load(&solution_);
    const auto frames = correctionFrames();
    std::vector<OdomFrameSample> samples;
    {
      std::lock_guard lock(samples_mutex_);
      samples = samples_;
    }
    trajectory.samples.reserve(samples.size());
    if (solution) trajectory.revision = solution->stats.revision;
    std::size_t next = 0;
    for (const auto& sample : samples) {
      auto transform = frames.graph_from_odom;
      if (solution && !solution->keyframes.empty()) {
        const auto& keyframes = solution->keyframes;
        while (next + 1 < keyframes.size() && keyframes[next + 1].stamp <= sample.stamp) ++next;
        if (keyframes[next].stamp <= sample.stamp && sample.stamp <= keyframes.back().stamp) {
          transform = keyframes[next].correction;
        }
      }
      transform = frames.map_from_graph * transform;
      trajectory.samples.push_back({sample.stamp, next, transform * sample.T_odom_lidar, transform.linear() * sample.v_odom});
    }
    return trajectory;
  }

  PoseGraphStats stats() const {
    const auto solution = std::atomic_load(&solution_);
    auto result = solution ? solution->stats : PoseGraphStats{};
    result.keyframes = tail_.load(std::memory_order_acquire);
    result.dropped_frames = dropped_.load(std::memory_order_relaxed);
    const auto alignment = correction();
    result.correction_translation = alignment.translation().norm();
    result.correction_rotation = Eigen::AngleAxisd(alignment.linear()).angle();
    return result;
  }

  PoseGraphSnapshot snapshot() const {
    const auto snapshot = std::atomic_load(&snapshot_);
    auto result = snapshot ? *snapshot : PoseGraphSnapshot{};
    result.stats = stats();
    return result;
  }

  RevisitExchange& revisitExchange() { return arxiv_.revisitExchange(); }
  LocalPlaneExchange& localPlanes() { return submap_.localPlanes(); }
  void requestSnapshot() { request(Snapshot); }
  void requestGlobalMap() { request(Map); }
  PointCloudConstPtr latestGlobalMap() const { return std::atomic_load(&map_); }
  void requestOccupancyGrid() { request(Occupancy); }
  std::shared_ptr<const OccupancyGridMsg> latestOccupancyGrid() const { return std::atomic_load(&occupancy_); }

 private:
  static const std::string& checkedMapDirectory(const Config& config) {
    validateConfig(config);
    return config.io.map_directory;
  }

  bool hasFrames() const { return head_.load(std::memory_order_relaxed) != tail_.load(std::memory_order_acquire); }
  bool hasRequests() const { return requests_.load(std::memory_order_acquire) != 0; }

  void processFrames() {
    auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    while (head != tail) {
      auto& frame = queue_[head % queue_.size()];
      if (!occupancy_directory_.empty()) recordRays(head, *frame.frame.cloud);
      odom_poses_.push_back(frame.frame.odom);
      if (submap_.update(std::move(frame))) {
        submap_.seal();
        publish();
      }
      frame = {};
      head_.store(++head, std::memory_order_release);
    }
  }

  void processRequests() { processRequests(requests_.exchange(0, std::memory_order_acq_rel)); }

  void drain() {
    submap_.seal();
    publish();
    processRequests(requested_views_.load(std::memory_order_acquire));
  }

  void workerLoop() {
    try {
      while (true) {
        std::unique_lock lock(wake_mutex_);
        wake_cv_.wait_for(lock, std::chrono::milliseconds(1), [&] { return stop_.load(std::memory_order_acquire) || hasFrames() || hasRequests(); });
        lock.unlock();
        processFrames();
        if (stop_.load(std::memory_order_acquire) && !hasFrames()) break;
        processRequests();
      }
      drain();
    } catch (...) {
      worker_error_ = std::current_exception();
      worker_failed_.store(true, std::memory_order_release);
    }
  }

  enum Request : unsigned { Snapshot = 1, Map = 2, Occupancy = 4 };

  void request(Request request) {
    if (stop_.load(std::memory_order_acquire)) return;
    requested_views_.fetch_or(request, std::memory_order_relaxed);
    if (!(requests_.fetch_or(request, std::memory_order_release) & request)) wake_cv_.notify_one();
  }

  struct Keyframe {
    double stamp;
    Isometry3d correction;
  };

  struct Solution {
    PoseGraphStats stats;
    std::vector<Keyframe> keyframes;
  };

  void publish() {
    const auto optimized = submap_.optimizedFrames();
    auto solution = std::make_shared<Solution>();
    solution->stats = submap_.stats();
    solution->keyframes.reserve(optimized.size());
    for (std::size_t i = 0; i < optimized.size(); ++i) {
      const auto& [stamp, pose] = optimized[i];
      solution->keyframes.push_back({stamp, pose * odom_poses_[i].inverse()});
    }
    std::atomic_store(&solution_, std::shared_ptr<const Solution>(std::move(solution)));
  }

  std::filesystem::path rayPath(std::size_t index) const { return occupancy_directory_ / (std::to_string(index) + ".bin"); }

  void recordRays(std::size_t index, const PointCloud& cloud) {
    const auto sampled = deterministicVoxelDownsample(cloud, config_.pgo.occupancy.cloud_voxel_size);
    std::ofstream file(rayPath(index), std::ios::binary);
    for (const auto& point : *sampled) {
      const std::array<float, 3> xyz{point.x, point.y, point.z};
      file.write(reinterpret_cast<const char*>(xyz.data()), sizeof(xyz));
    }
    file.close();
    if (!file) throw std::runtime_error("cannot write submap occupancy rays");
  }

  void updateOccupancy(const std::vector<std::pair<double, Isometry3d>>& poses, const Isometry3d& map_from_graph) {
    if (!config_.pgo.occupancy.enabled) return;
    OccupancyGrid grid(config_.pgo.occupancy);
    const auto T_lidar_imu = config_.extrinsics.T_imu_lidar.inverse();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      std::ifstream file(rayPath(i), std::ios::binary);
      if (!file) throw std::runtime_error("cannot read submap occupancy rays");
      auto hits = std::make_shared<PointCloud>();
      std::array<float, 3> xyz;
      while (file.read(reinterpret_cast<char*>(xyz.data()), sizeof(xyz))) {
        Point point{};
        point.getVector3fMap() = (T_lidar_imu * Eigen::Map<Eigen::Vector3f>(xyz.data()).cast<double>()).cast<float>();
        point.data[3] = 1.f;
        hits->push_back(point);
      }
      if (!file.eof() || file.gcount() != 0) throw std::runtime_error("invalid submap occupancy rays");
      grid.insertScan(hits, map_from_graph * poses[i].second * config_.extrinsics.T_imu_lidar);
    }
    auto result = std::make_shared<OccupancyGridMsg>(grid.toMsg());
    result->revision = submap_.stats().revision;
    std::atomic_store(&occupancy_, std::shared_ptr<const OccupancyGridMsg>(std::move(result)));
  }

  void processRequests(unsigned requests) {
    if (!requests) return;
    const auto map_from_graph = correctionFrames().map_from_graph;
    if (requests & Snapshot) {
      auto snapshot = std::make_shared<PoseGraphSnapshot>(submap_.snapshot());
      for (auto& pose : snapshot->optimized_poses) pose = map_from_graph * pose * config_.extrinsics.T_imu_lidar;
      std::atomic_store(&snapshot_, std::shared_ptr<const PoseGraphSnapshot>(std::move(snapshot)));
    }
    if (requests & Map) {
      spdlog::debug("[methodB] map_export_begin");
      auto map = submap_.globalMap(config_.io.global_map_voxel_size);
      const auto rotation = map_from_graph.linear().cast<float>().eval();
      for (auto& point : *map) point.getVector3fMap() = (rotation * point.getVector3fMap()).eval();
      std::atomic_store(&map_, PointCloudConstPtr(std::move(map)));
      spdlog::debug("[methodB] map_export_end");
    }
    if ((requests & Occupancy) && config_.pgo.occupancy.enabled) {
      const auto previous = std::atomic_load(&occupancy_);
      if (!previous || previous->revision != submap_.stats().revision || !occupancy_frame_.matrix().isApprox(map_from_graph.matrix())) {
        updateOccupancy(submap_.optimizedFrames(), map_from_graph);
        occupancy_frame_ = map_from_graph;
      }
    }
  }

  PointCloudConstPtr map_;
  std::shared_ptr<const OccupancyGridMsg> occupancy_;
  const Config config_;
  SubmapArxiv arxiv_;
  Submap submap_;
  std::filesystem::path occupancy_directory_;
  std::vector<Isometry3d> odom_poses_;
  std::vector<OdomFrameSample> samples_;
  mutable std::mutex samples_mutex_, correction_mutex_;
  struct CorrectionFrames {
    Isometry3d graph_from_odom = Isometry3d::Identity();
    Isometry3d map_from_graph = Isometry3d::Identity();
  };

  CorrectionFrames correctionFrames() const {
    std::lock_guard lock(correction_mutex_);
    return correction_;
  }

  CorrectionFrames correction_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
  std::optional<double> gravity_stamp_;
  Isometry3d occupancy_frame_ = Isometry3d::Identity();
  std::shared_ptr<const Solution> solution_;
  std::shared_ptr<const PoseGraphSnapshot> snapshot_;
  std::array<InertialFrame, 32> queue_;
  std::atomic<std::size_t> head_{0}, tail_{0}, dropped_{0};
  std::atomic<unsigned> requests_{0}, requested_views_{0};
  std::exception_ptr worker_error_;
  std::atomic<bool> worker_failed_{false};
  std::atomic<bool> stop_{false};
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::thread worker_;
};

PoseGraphBackend::PoseGraphBackend(const Config& config) : impl_(std::make_unique<Impl>(config)) {}

PoseGraphBackend::~PoseGraphBackend() = default;

bool PoseGraphBackend::enabled() const { return true; }

void PoseGraphBackend::finish() { impl_->finish(); }

bool PoseGraphBackend::addInertialFrame(InertialFrame frame) { return impl_->addFrame(std::move(frame)); }
void PoseGraphBackend::setLiveCorrection(const Isometry3d& transform, double stamp, const Eigen::Vector3d& gravity_odom) {
  impl_->setCorrection(transform, stamp, gravity_odom);
}
RevisitExchange& PoseGraphBackend::revisitExchange() { return impl_->revisitExchange(); }
LocalPlaneExchange& PoseGraphBackend::localPlanes() { return impl_->localPlanes(); }

void PoseGraphBackend::recordOdometrySample(const OdomFrameSample& sample) { impl_->record(sample); }

MapTrajectory PoseGraphBackend::mapTrajectory() const { return impl_->mapTrajectory(); }

Isometry3d PoseGraphBackend::T_map_odom() const { return impl_->correction(); }

PoseGraphStats PoseGraphBackend::stats() const { return impl_->stats(); }

PoseGraphSnapshot PoseGraphBackend::snapshot() const { return impl_->snapshot(); }

void PoseGraphBackend::requestSnapshot() { impl_->requestSnapshot(); }

void PoseGraphBackend::requestGlobalMap() { impl_->requestGlobalMap(); }

PointCloudConstPtr PoseGraphBackend::latestGlobalMap() const { return impl_->latestGlobalMap(); }

void PoseGraphBackend::requestOccupancyGrid() { impl_->requestOccupancyGrid(); }

std::shared_ptr<const OccupancyGridMsg> PoseGraphBackend::latestOccupancyGrid() const { return impl_->latestOccupancyGrid(); }

}  // namespace sapphire
