#include <sapphire/odometry/pipeline2.hpp>

#include <pcl/common/transforms.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/odometry/pipeline.hpp>

#include <sapphire/mapping/voxelmap.hpp>
#include <sapphire/odometry/registration2.hpp>

namespace sapphire {
namespace {

constexpr std::size_t kMappingImuCapacity = 1000;

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

}  // namespace

class Pipeline2::Impl {
 public:
  Impl(const Config& config, PoseGraphBackend& backend, RevisitExchange& exchange, LocalPlaneExchange& local_planes)
      : config_(config), backend_(backend), registration_(config, exchange, local_planes) {}

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
    scan.lidar = preprocessLidar(*points, config_.odometry.crop_box, config_.methodB.max_range_m);
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
  PoseGraphBackend& backend_;
  Registration2 registration_;
  std::atomic<bool> initialized_{false};
  ImuInitializer::Result init_result_{};
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

Pipeline2::Pipeline2(const Config& config, PoseGraphBackend& backend)
    : impl_(std::make_unique<Impl>(config, backend, backend.revisitExchange(), backend.localPlanes())) {}
Pipeline2::~Pipeline2() = default;
void Pipeline2::pushImu(const ImuData& imu) { impl_->pushImu(imu); }
void Pipeline2::pushLidar(const PointCloudConstPtr& points) { impl_->pushLidar(points); }
bool Pipeline2::initialized() const { return impl_->initialized_.load(std::memory_order_acquire); }
const ImuInitializer::Result& Pipeline2::imuInitResult() const {
  if (!initialized()) throw std::logic_error("IMU initialization is incomplete");
  return impl_->init_result_;
}
OdomFrameSample Pipeline2::latest() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->latest_;
}
std::optional<OdomFrameSample> Pipeline2::propagated() const {
  std::lock_guard lock(impl_->imu_mutex_);
  return impl_->propagated_;
}
PointCloudConstPtr Pipeline2::deskewed(const Isometry3d& T_map_odom) const {
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
Eigen::Vector3d Pipeline2::accelBias() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->accel_bias_;
}
Eigen::Vector3d Pipeline2::gyroBias() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->gyro_bias_;
}
std::size_t Pipeline2::mapRevision() const {
  std::lock_guard lock(impl_->output_mutex_);
  return impl_->map_revision_;
}

}  // namespace sapphire
