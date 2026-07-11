#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sapphire {

namespace {

double computeScanEndStamp(
    double stamp,
    const PointCloud& points,
    bool time_offset)
{
    const auto [min_time_it, max_time_it] = std::minmax_element(
        points.points.begin(),
        points.points.end(),
        [](const Point& lhs, const Point& rhs) {
            return lhs.timestamp < rhs.timestamp;
        });
    return stamp + max_time_it->timestamp
        - (time_offset ? min_time_it->timestamp : 0.0);
}

}  // namespace

OdometryPipeline::OdometryPipeline(const Config& config)
    : config_(config)
    , imu_initializer_(config.imu.init)
    , propagation_pim_(std::make_unique<PropagationPim>(
          std::make_shared<PropagationPim::Params>(
              Eigen::Vector3d(0.0, 0.0, -config.imu.init.gravity_mag),
              config.imu.noise.gyro_noise_density,
              config.imu.noise.accel_noise_density,
              0.0,
              0.0,
              config.imu.noise.gyro_random_walk,
              config.imu.noise.accel_random_walk,
              0.0,
              0.0)))
    , submap_manager_(config.odometry.submap)
    , registration_(config.registration.gicp)
{
    latest_result_.T_world_lidar = Isometry3d::Identity();
    gravity_world_ =
        Eigen::Vector3d(0.0, 0.0, -config.imu.init.gravity_mag);

    const auto& box = config.odometry.crop_box;
    crop_filter_.setMin(Eigen::Vector4f(
        static_cast<float>(box.min_x),
        static_cast<float>(box.min_y),
        static_cast<float>(box.min_z), 1.0f));
    crop_filter_.setMax(Eigen::Vector4f(
        static_cast<float>(box.max_x),
        static_cast<float>(box.max_y),
        static_cast<float>(box.max_z), 1.0f));
    crop_filter_.setNegative(true);

    spdlog::info("[pipeline] OdometryPipeline created");
    spdlog::info("[pipeline]   scan_voxel_size={}m",
        config.odometry.voxel_size);
    spdlog::info("[pipeline]   crop_box: [{:.2f},{:.2f},{:.2f}] → [{:.2f},{:.2f},{:.2f}]",
        box.min_x, box.min_y, box.min_z,
        box.max_x, box.max_y, box.max_z);
    spdlog::info(
        "[pipeline]   submap: distance={}m, rotation={}rad, keyframes={}, "
        "voxel={}m",
        config.odometry.submap.splitting_distance,
        config.odometry.submap.splitting_rotation,
        config.odometry.submap.max_keyframes,
        config.odometry.submap.voxel_size);
    spdlog::info(
        "[pipeline]   observer: Kp={} Kv={} Kq={} Kab={} Kgb={}",
        config.odometry.observer.position_gain,
        config.odometry.observer.velocity_gain,
        config.odometry.observer.orientation_gain,
        config.odometry.observer.accel_bias_gain,
        config.odometry.observer.gyro_bias_gain);
    spdlog::info("[pipeline]   registration: {}", config.registration.type);
    spdlog::info("[pipeline]   cuda: {}", config.cuda.enabled ? "ON" : "OFF");
    spdlog::info("[pipeline]   imu_init: gyro_std<{:.4f}, accel_std<{:.3f}, "
        "timeout={:.1f}s",
        config.imu.init.convergence_gyro_std,
        config.imu.init.convergence_accel_std,
        config.imu.init.timeout_sec);
}

OdometryResult OdometryPipeline::latestResult() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return latest_result_;
}

Isometry3d OdometryPipeline::latestPose() const {
    return latestResult().T_world_lidar;
}

PointCloudConstPtr OdometryPipeline::latestDeskewed() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return latest_deskewed_;
}

std::optional<OdometryResult>
OdometryPipeline::latestPropagatedResult() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_first_scan_.load(std::memory_order_acquire)
        || !propagated_state_.valid) {
        return std::nullopt;
    }

    OdometryResult result;
    result.T_world_lidar =
        propagated_state_.T_world_imu * config_.extrinsics.T_imu_lidar;
    result.stamp = propagated_state_.stamp;
    result.converged = true;
    return result;
}

Eigen::Vector3d OdometryPipeline::accelBias() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return accel_bias_;
}

Eigen::Vector3d OdometryPipeline::gyroBias() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return gyro_bias_;
}

// ── LiDAR preprocessing ─────────────────────────────────────────────

PointCloudConstPtr OdometryPipeline::preprocessPoints(const PointCloudConstPtr& points) const {
    auto out = std::make_shared<PointCloud>();
    out->points.reserve(points->points.size());

    for (const auto& pt : points->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        out->points.push_back(pt);
    }

    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;

    // Crop Box Filter — remove robot body points inside the configured box
    crop_filter_.setInputCloud(out);
    crop_filter_.filter(*out);

    return out;
}

PointCloudConstPtr OdometryPipeline::downsamplePoints(
    const PointCloudConstPtr& points) const
{
    auto downsampled = std::make_shared<PointCloud>();
    pcl::VoxelGrid<Point> voxel_filter;
    const float leaf_size =
        static_cast<float>(config_.odometry.voxel_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(points);
    voxel_filter.filter(*downsampled);
    // PCL's centroid path does not guarantee the homogeneous padding member
    // for custom point types. small_gicp consumes getVector4fMap(), so every
    // point must retain w=1 for covariance estimation and transformations.
    for (auto& point : downsampled->points) {
        point.data[3] = 1.0f;
    }
    return downsampled;
}

DeskewResult OdometryPipeline::deskewPointcloud(
    double stamp, const PointCloudConstPtr& points)
{
    NavigationState baseline;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!imu_state_.valid) {
            throw std::logic_error(
                "OdometryPipeline::deskewPointcloud called without a valid IMU state");
        }
        baseline = imu_state_;
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);
    return deskew(
        points,
        stamp,
        imu_buffer_,
        baseline.stamp,
        baseline.T_world_imu,
        baseline.v_world,
        config_.extrinsics.T_imu_lidar,
        gravity_world_,
        config_.imu.noise,
        config_.deskew.time_offset);
}

void OdometryPipeline::recoverPropagatedStateLocked(double stamp) {
    using Gal3 = PropagationPim::Gal3;
    Gal3::IsometriesType initial_isometries{
        imu_state_.v_world,
        imu_state_.T_world_imu.translation(),
    };
    const Gal3 initial_state(
        imu_state_.T_world_imu.rotation(), initial_isometries, 0.0);
    const Gal3 propagated =
        propagation_pim_->Gamma_ij()
        * initial_state
        * propagation_pim_->Upsilon();

    propagated_state_.stamp = stamp;
    propagated_state_.T_world_imu = Isometry3d::Identity();
    propagated_state_.T_world_imu.linear() = propagated.R();
    propagated_state_.T_world_imu.translation() = propagated.p();
    propagated_state_.v_world = propagated.v();
    propagated_state_.valid = true;
}

void OdometryPipeline::propagateStateLocked(const ImuData& imu) {
    if (!has_first_scan_.load(std::memory_order_acquire)
        || !imu_state_.valid || !propagated_state_.valid
        || imu.stamp <= propagated_state_.stamp) {
        return;
    }

    const double dt = imu.stamp - propagated_state_.stamp;
    propagation_pim_->integrateMeasurement(imu.accel, imu.gyro, dt);
    recoverPropagatedStateLocked(imu.stamp);
}

void OdometryPipeline::rebasePropagation(
    const NavigationState& corrected_state,
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias)
{
    std::scoped_lock lock(imu_mutex_, state_mutex_);
    // Buffered samples were corrected with the previous bias. Shift them to
    // the new linearization before replaying from the LiDAR reference time.
    const Eigen::Vector3d accel_correction = accel_bias_ - accel_bias;
    const Eigen::Vector3d gyro_correction = gyro_bias_ - gyro_bias;
    for (ImuData& imu : imu_buffer_) {
        imu.accel += accel_correction;
        imu.gyro += gyro_correction;
    }
    accel_bias_ = accel_bias;
    gyro_bias_ = gyro_bias;

    imu_state_ = corrected_state;
    propagated_state_ = corrected_state;
    propagation_pim_->resetIntegrationAndSetBias(
        PropagationPim::Vec10::Zero());
    has_first_scan_.store(true, std::memory_order_release);

    for (const ImuData& imu : imu_buffer_) {
        propagateStateLocked(imu);
    }
}

bool OdometryPipeline::waitForImuCoverage(double end_stamp) {
    constexpr auto kImuWaitTimeout = std::chrono::milliseconds(500);
    std::unique_lock<std::mutex> lock(imu_mutex_);
    return imu_cv_.wait_for(lock, kImuWaitTimeout, [this, end_stamp]() {
        return !imu_buffer_.empty()
            && imu_buffer_.back().stamp >= end_stamp;
    });
}

// ── LiDAR callback ──────────────────────────────────────────────────

bool OdometryPipeline::initializeFirstLidarTarget(
    double stamp, const PointCloudConstPtr& preprocessed)
{
    // DLIO first-valid-scan path: do not integrate from the calibration
    // timestamp. Assume no motion, apply the gravity-aligned initial pose,
    // and establish this scan as the first registration target.
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_buffer_.empty() || imu_buffer_.front().stamp > stamp) {
            return false;
        }
    }

    NavigationState corrected_state;
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        corrected_state = imu_state_;
        accel_bias = accel_bias_;
        gyro_bias = gyro_bias_;
    }
    corrected_state.stamp = stamp;
    const Isometry3d T_world_lidar =
        corrected_state.T_world_imu * config_.extrinsics.T_imu_lidar;
    auto world_scan = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
        *preprocessed, *world_scan, T_world_lidar.matrix());
    const PointCloudConstPtr output_scan = downsamplePoints(world_scan);

    submap_manager_.addKeyframe(T_world_lidar, world_scan, stamp);
    registration_.setTarget(submap_manager_.target());
    spdlog::info(
        "[pipeline] submap target rebuilt: keyframes={}, points={}, "
        "revision={}",
        submap_manager_.keyframeCount(),
        submap_manager_.target()->size(),
        submap_manager_.targetRevision());

    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        latest_deskewed_ = output_scan;
        latest_result_.T_world_lidar = T_world_lidar;
        latest_result_.stamp = stamp;
        latest_result_.converged = true;
    }

    rebasePropagation(corrected_state, accel_bias, gyro_bias);

    spdlog::info(
        "[pipeline] first LiDAR target initialized: stamp={:.6f}, "
        "points={}",
        stamp,
        output_scan->size());
    return true;
}

std::optional<OdometryPipeline::RegistrationArtifacts>
OdometryPipeline::runScanRegistration(const DeskewResult& deskewed) {
    const PointCloudConstPtr registration_source =
        downsamplePoints(deskewed.cloud);
    if (registration_source->size()
        < static_cast<size_t>(config_.registration.gicp.min_num_points)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: {} points after voxel "
            "downsampling (minimum {}, input {})",
            registration_source->size(),
            config_.registration.gicp.min_num_points,
            deskewed.cloud->size());
        return std::nullopt;
    }
    spdlog::debug(
        "[pipeline] GICP source downsampled: {} -> {} points",
        deskewed.cloud->size(),
        registration_source->size());

    registration_.setSource(registration_source);
    RegistrationArtifacts artifacts;
    artifacts.result =
        registration_.align(deskewed.T_world_lidar_ref);
    artifacts.corrected_full = deskewed.cloud;
    artifacts.corrected_source = registration_source;

    if (artifacts.result.accepted) {
        auto transformed_full = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *deskewed.cloud,
            *transformed_full,
            artifacts.result.T_correction.matrix());
        artifacts.corrected_full = transformed_full;

        auto transformed_source = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *registration_source,
            *transformed_source,
            artifacts.result.T_correction.matrix());
        artifacts.corrected_source = transformed_source;
    }
    return artifacts;
}

void OdometryPipeline::commitLidarOutputs(
    const DeskewResult& deskewed,
    const RegistrationArtifacts& artifacts)
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    latest_deskewed_ = artifacts.corrected_source;
    latest_result_.T_world_lidar = artifacts.result.T_world_lidar;
    latest_result_.stamp = deskewed.reference_stamp;
    latest_result_.converged = artifacts.result.accepted;
}

void OdometryPipeline::maybeUpdateSubmapTarget(
    const DeskewResult& deskewed,
    const RegistrationArtifacts& artifacts)
{
    if (!artifacts.result.accepted
        || !submap_manager_.shouldAddKeyframe(
            artifacts.result.T_world_lidar)
        || !submap_manager_.addKeyframe(
            artifacts.result.T_world_lidar,
            artifacts.corrected_full,
            deskewed.reference_stamp)) {
        return;
    }

    registration_.setTarget(submap_manager_.target());
    spdlog::info(
        "[pipeline] submap target rebuilt: keyframes={}, points={}, "
        "revision={}",
        submap_manager_.keyframeCount(),
        submap_manager_.target()->size(),
        submap_manager_.targetRevision());
}

void OdometryPipeline::processLidarScan(
    double stamp, const PointCloudConstPtr& preprocessed)
{
    const double scan_end = computeScanEndStamp(
        stamp, *preprocessed, config_.deskew.time_offset);
    if (!waitForImuCoverage(scan_end)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: timed out waiting for IMU "
            "coverage through {:.6f}",
            scan_end);
        return;
    }

    DeskewResult deskewed = deskewPointcloud(stamp, preprocessed);
    if (deskewed.status != DeskewStatus::Success) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: deskew failed with status {}",
            static_cast<int>(deskewed.status));
        return;
    }

    const auto artifacts = runScanRegistration(deskewed);
    if (!artifacts.has_value()) {
        return;
    }
    commitLidarOutputs(deskewed, *artifacts);

    const Isometry3d T_world_imu_prior =
        deskewed.T_world_lidar_ref
        * config_.extrinsics.T_imu_lidar.inverse();
    const Isometry3d T_world_imu_corrected =
        artifacts->result.T_world_lidar
        * config_.extrinsics.T_imu_lidar.inverse();
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    double previous_state_stamp = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        previous_state_stamp = imu_state_.stamp;
        accel_bias = accel_bias_;
        gyro_bias = gyro_bias_;
    }

    NavigationState prior_state;
    prior_state.stamp = deskewed.reference_stamp;
    prior_state.T_world_imu = T_world_imu_prior;
    prior_state.v_world = deskewed.v_world_ref;
    prior_state.valid = true;
    const ObserverUpdate observer_update = applyGeometricObserver(
        prior_state,
        T_world_imu_corrected,
        previous_state_stamp,
        accel_bias,
        gyro_bias,
        config_.odometry.observer,
        artifacts->result.accepted);
    if (artifacts->result.accepted
        && deskewed.reference_stamp > previous_state_stamp) {
        spdlog::debug(
            "[pipeline] observer bias: accel=[{:.5f},{:.5f},{:.5f}] "
            "gyro=[{:.6f},{:.6f},{:.6f}]",
            observer_update.accel_bias.x(),
            observer_update.accel_bias.y(),
            observer_update.accel_bias.z(),
            observer_update.gyro_bias.x(),
            observer_update.gyro_bias.y(),
            observer_update.gyro_bias.z());
    }

    rebasePropagation(
        observer_update.state,
        observer_update.accel_bias,
        observer_update.gyro_bias);
    maybeUpdateSubmapTarget(deskewed, *artifacts);
}

void OdometryPipeline::pushLidar(
    double stamp, const PointCloudConstPtr& points)
{
    if (!initialized_.load(std::memory_order_acquire)
        || !points || points->empty()) {
        return;
    }

    const PointCloudConstPtr preprocessed = preprocessPoints(points);
    if (preprocessed->size()
        < static_cast<size_t>(config_.registration.gicp.min_num_points)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: {} points after preprocessing "
            "(minimum {})",
            preprocessed->size(),
            config_.registration.gicp.min_num_points);
        return;
    }

    if (!has_first_scan_.load(std::memory_order_acquire)) {
        initializeFirstLidarTarget(stamp, preprocessed);
        return;
    }
    processLidarScan(stamp, preprocessed);
}

// ── IMU callback ────────────────────────────────────────────────────

void OdometryPipeline::finalizeImuInitialization(
    ImuInitializer::Result&& result,
    const ImuData& trigger_sample)
{
    imu_init_result_ = std::move(result);

    ImuData corrected;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        accel_bias_ = imu_init_result_->accel_bias;
        gyro_bias_ = imu_init_result_->gyro_bias;
        imu_state_.stamp = trigger_sample.stamp;
        imu_state_.T_world_imu = Isometry3d::Identity();
        imu_state_.T_world_imu.linear() =
            imu_init_result_->q_gravity.toRotationMatrix();
        imu_state_.v_world.setZero();
        imu_state_.valid = true;
        corrected.stamp = trigger_sample.stamp;
        corrected.accel = trigger_sample.accel - accel_bias_;
        corrected.gyro = trigger_sample.gyro - gyro_bias_;
    }

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buffer_.clear();
        imu_buffer_.push_back(corrected);
    }
    initialized_.store(true, std::memory_order_release);
    imu_cv_.notify_all();
}

void OdometryPipeline::pushImu(const ImuData& imu) {
    if (!initialized_.load(std::memory_order_acquire)) {
        auto result = imu_initializer_.feedImu(imu.stamp, imu.accel, imu.gyro);
        if (result.has_value()) {
            finalizeImuInitialization(std::move(*result), imu);
        }
        return;
    }

    ImuData corrected;
    {
        // Correction, buffering, and propagation are atomic with respect to a
        // LiDAR observer update that changes bias and replays the buffer.
        std::scoped_lock lock(imu_mutex_, state_mutex_);
        if (!imu_buffer_.empty()
            && imu.stamp <= imu_buffer_.back().stamp) {
            return;
        }
        corrected.stamp = imu.stamp;
        corrected.accel = imu.accel - accel_bias_;
        corrected.gyro = imu.gyro - gyro_bias_;
        imu_buffer_.push_back(corrected);
        propagateStateLocked(corrected);
    }
    imu_cv_.notify_all();
}

}  // namespace sapphire
