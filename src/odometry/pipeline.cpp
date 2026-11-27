#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/mapping/voxelmap.hpp>
#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/odometry/registration.hpp>
#include <sapphire/ring_buffer.hpp>
#include <stdexcept>

namespace sapphire {
namespace {

constexpr std::size_t kMappingImuCapacity = 1000;
constexpr std::size_t kImuBufferCapacity = 20000;
using ImuBuffer = RingBuffer<ImuData, kImuBufferCapacity>;

ImuBuffer::const_iterator firstImuAfter(const ImuBuffer& buffer, double stamp) {
  return std::upper_bound(buffer.begin(), buffer.end(), stamp, [](double time, const ImuData& sample) { return time < sample.stamp; });
}

FilterImuSample toFilterSample(const ImuData& imu) { return {imu.stamp, imu.accel.cast<float>(), imu.gyro.cast<float>()}; }

Isometry3d imuPose(const M3& rotation, const V3& position) {
  auto pose = Isometry3d::Identity();
  pose.linear() = Eigen::Quaterniond(rotation.cast<double>()).normalized().toRotationMatrix();
  pose.translation() = position.cast<double>();
  return pose;
}

OdometryResult applyGlobalCorrection(const OdomFrameSample& sample, const Isometry3d& T_map_odom) {
  OdometryResult result;
  result.T_world_lidar = T_map_odom * sample.T_odom_lidar;
  result.v_world = T_map_odom.rotation() * sample.v_odom;
  result.stamp = sample.stamp;
  result.converged = sample.converged;
  return result;
}

}  // namespace

class OdometryPipeline::Impl {
 public:
  explicit Impl(const Config& config) : config_(config), backend_(config), registration_(config, backend_.revisitExchange(), backend_.localPlanes()) {
    spdlog::info("[pipeline] ready: methodB");
  }

  void pushImu(const ImuData& imu) {
    if (!imu.isFinite()) return;
    {
      std::lock_guard lock(imu_mutex_);
      if (!imu_.empty() && imu.stamp <= imu_.back().stamp) return;
      imu_.push_back(imu);
      propagate(imu);
    }
    imu_ready_.notify_one();
  }

  void pushLidar(const PointCloudConstPtr& points) {
    if (!points || points->empty()) return;
    std::lock_guard scan_lock(scan_mutex_);
    MeasureGroup scan;
    scan.lidar = preprocessLidar(*points, config_.odometry.crop_box, config_.sensor.lidar.max_range_m);
    const double end = scan.lidar.end_time;
    if (scan.lidar.pc->empty() || (last_scan_stamp_ && end <= *last_scan_stamp_)) return;
    {
      std::unique_lock lock(imu_mutex_);
      if (!imu_ready_.wait_for(lock, OdometryPipeline::kImuCoverageWait, [&] { return !imu_.empty() && imu_.back().stamp >= end; })) return;
      // Select this scan's window without consuming the bracketing sample.
      const auto after_end = firstImuAfter(imu_, end);
      const auto first = last_scan_imu_ ? firstImuAfter(imu_, last_scan_imu_->stamp) : imu_.cbegin();
      for (auto imu = first; imu != after_end; ++imu) {
        last_scan_imu_ = *imu;
        scan.imu.push_back(toFilterSample(*imu));
        rememberMappingImu(*imu);
      }
      if (!last_scan_imu_) return;
      const auto& before = *last_scan_imu_;
      scan.end_imu = toFilterSample(before);
      if (after_end != imu_.cend()) {
        const auto& after = *after_end;
        rememberMappingImu(after);
        if (before.stamp < end) {
          // Split the bracketing IMU interval at the LiDAR reference time.
          // The future sample stays queued for the next scan.
          const double fraction = (end - before.stamp) / (after.stamp - before.stamp);
          scan.end_imu = {end, ((1. - fraction) * before.accel + fraction * after.accel).cast<float>(),
                          ((1. - fraction) * before.gyro + fraction * after.gyro).cast<float>()};
        }
      }
    }
    last_scan_stamp_ = end;
    const auto boundary = scan.end_imu;
    const bool tracked = registration_.process(std::move(scan));
    const auto& filter = registration_.filter();
    if (!initialized_.load(std::memory_order_relaxed) && registration_.initialized()) {
      const auto state = filter.GetSysState();
      init_result_ = {state.bg.cast<double>(), state.ba.cast<double>(), Eigen::Quaterniond(imuPose(state.R.R_, state.p).linear()),
                      registration_.accelerationStd(), registration_.accelerationScale()};
      initialized_.store(true, std::memory_order_release);
    }
    if (!tracked) return;

    const auto nav = filter.GetNavState();
    const auto state = filter.GetSysState();
    if (!nav.R.R_.allFinite() || !nav.p.allFinite() || !nav.v.allFinite()) {
      throw std::runtime_error("filter produced a non-finite state");
    }
    const Isometry3d pose = imuPose(nav.R.R_, nav.p);
    const OdomFrameSample sample{pose * config_.extrinsics.T_imu_lidar, nav.v.cast<double>(), nav.timestamp, true};
    if (registration_.matches() >= 64 && shouldSubmit(nav.timestamp, pose)) {
      if (!mapping_gravity_) mapping_gravity_ = filter.GetGravity().cast<double>();
      InertialFrame frame{
          {nav.timestamp, pose, registration_.deskewed()},
          {nav.v.cast<double>(), state.bg.cast<double>(), state.ba.cast<double>(), *mapping_gravity_, registration_.accelerationScale()},
          {mapping_imu_.begin(), mapping_imu_.end()}};
      if (backend_.addInertialFrame(std::move(frame))) {
        registration_.rememberKeyframe(nav.timestamp, pose);
        last_keyframe_stamp_ = nav.timestamp;
        last_keyframe_pose_ = pose;
        while (mapping_imu_.size() > 2 && mapping_imu_[1].stamp <= nav.timestamp) {
          mapping_imu_.pop_front();
        }
      }
    }
    backend_.setLiveCorrection(registration_.globalFromOdom(), nav.timestamp, filter.GetGravity().cast<double>());
    backend_.recordOdometrySample(sample);

    {
      std::lock_guard lock(output_mutex_);
      latest_ = sample;
      deskewed_ = registration_.deskewed();
      deskewed_pose_ = pose;
      accel_bias_ = state.ba.cast<double>();
      gyro_bias_ = state.bg.cast<double>();
      map_revision_ = registration_.revisitVersion();
    }
    {
      std::lock_guard lock(imu_mutex_);
      tip_ = filter;
      propagated_ = sample;
      tip_->SetTipMeasurement(boundary);
      for (auto imu = firstImuAfter(imu_, boundary.secs); imu != imu_.cend(); ++imu) propagate(*imu);
    }
  }

  const Config config_;
  PoseGraphBackend backend_;
  Registration registration_;
  std::atomic<bool> initialized_{false};
  ImuInitResult init_result_{};
  mutable std::mutex output_mutex_;
  OdomFrameSample latest_;
  PointCloudConstPtr deskewed_;
  Isometry3d deskewed_pose_ = Isometry3d::Identity();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  std::size_t map_revision_ = 0;
  mutable std::mutex imu_mutex_;
  std::optional<OdomFrameSample> propagated_;

 private:
  void rememberMappingImu(const ImuData& imu) {
    if (!mapping_imu_.empty() && imu.stamp <= mapping_imu_.back().stamp) return;
    mapping_imu_.push_back(imu);
    if (mapping_imu_.size() > kMappingImuCapacity) mapping_imu_.pop_front();
  }

  bool shouldSubmit(double stamp, const Isometry3d& pose) const {
    if (!last_keyframe_stamp_) return true;
    const double interval = stamp - *last_keyframe_stamp_;
    return interval >= .20 && (interval >= .8 || (pose.translation() - last_keyframe_pose_.translation()).norm() >= .65 ||
                               Eigen::AngleAxisd(last_keyframe_pose_.linear().transpose() * pose.linear()).angle() >= .15);
  }

  void propagate(const ImuData& imu) {
    if (!tip_) return;
    DynamicState state;
    if (tip_->PredictTip(toFilterSample(imu), state)) {
      propagated_ = OdomFrameSample{imuPose(state.R, state.p) * config_.extrinsics.T_imu_lidar, state.v.cast<double>(), state.time, true};
    }
  }

  std::mutex scan_mutex_;
  std::condition_variable imu_ready_;
  ImuBuffer imu_;
  std::deque<ImuData> mapping_imu_;
  std::optional<ImuData> last_scan_imu_;
  std::optional<double> last_scan_stamp_, last_keyframe_stamp_;
  Isometry3d last_keyframe_pose_ = Isometry3d::Identity();
  std::optional<iEskf> tip_;
  std::optional<Eigen::Vector3d> mapping_gravity_;
};

OdometryPipeline::OdometryPipeline(const Config& config) : impl_(std::make_unique<Impl>(config)) {}

OdometryPipeline::~OdometryPipeline() = default;

void OdometryPipeline::pushImu(const ImuData& imu) { impl_->pushImu(imu); }
void OdometryPipeline::pushLidar(double, const PointCloudConstPtr& points) { impl_->pushLidar(points); }

bool OdometryPipeline::initialized() const { return impl_->initialized_.load(std::memory_order_acquire); }

const ImuInitResult& OdometryPipeline::imuInitResult() const {
  if (!initialized()) throw std::logic_error("IMU initialization is incomplete");
  return impl_->init_result_;
}

OdometryResult OdometryPipeline::latestResult() const {
  OdomFrameSample sample;
  {
    std::lock_guard lock(impl_->output_mutex_);
    sample = impl_->latest_;
  }
  return applyGlobalCorrection(sample, impl_->backend_.T_map_odom());
}

std::optional<OdometryResult> OdometryPipeline::latestPropagatedResult() const {
  std::optional<OdomFrameSample> sample;
  {
    std::lock_guard lock(impl_->imu_mutex_);
    sample = impl_->propagated_;
  }
  if (!sample) return std::nullopt;
  return applyGlobalCorrection(*sample, impl_->backend_.T_map_odom());
}

PointCloudConstPtr OdometryPipeline::latestDeskewed() const {
  const auto T_map_odom = impl_->backend_.T_map_odom();
  PointCloudConstPtr body;
  Isometry3d pose;
  {
    std::lock_guard lock(impl_->output_mutex_);
    body = impl_->deskewed_;
    pose = impl_->deskewed_pose_;
  }
  if (!body) return {};
  auto cloud = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*body, *cloud, (T_map_odom * pose).matrix());
  return cloud;
}
Eigen::Vector3d OdometryPipeline::accelBias() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->accel_bias_;
}
Eigen::Vector3d OdometryPipeline::gyroBias() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->gyro_bias_;
}
std::size_t OdometryPipeline::submapTargetRevision() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->map_revision_;
}

std::size_t OdometryPipeline::keyframeCount() const { return impl_->backend_.stats().keyframes; }

MapTrajectory OdometryPipeline::mapTrajectory() const { return impl_->backend_.mapTrajectory(); }
void OdometryPipeline::finishMapping() { impl_->backend_.finish(); }
PoseGraphSnapshot OdometryPipeline::poseGraphSnapshot() const { return impl_->backend_.snapshot(); }
void OdometryPipeline::requestPoseGraphSnapshot() { impl_->backend_.requestSnapshot(); }
void OdometryPipeline::requestPoseGraphMap() { impl_->backend_.requestGlobalMap(); }
PointCloudConstPtr OdometryPipeline::latestPoseGraphMap() const { return impl_->backend_.latestGlobalMap(); }
void OdometryPipeline::requestOccupancyGrid() { impl_->backend_.requestOccupancyGrid(); }
std::shared_ptr<const OccupancyGridMsg> OdometryPipeline::latestOccupancyGrid() const { return impl_->backend_.latestOccupancyGrid(); }

}  // namespace sapphire
