#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/integrate_measurement.hpp>
#include <sapphire/odometry/voxel_filter.hpp>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sapphire {

namespace {

constexpr double kImuGapWarningSec = 0.1;

bool isFiniteImu(const ImuData& imu) {
    return std::isfinite(imu.stamp)
        && imu.accel.allFinite()
        && imu.gyro.allFinite();
}

OdometryResult applyGlobalCorrection(
    OdometryResult result,
    const Isometry3d& T_map_odom)
{
    result.T_world_lidar = T_map_odom * result.T_world_lidar;
    result.v_world = T_map_odom.rotation() * result.v_world;
    return result;
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
    , pgo_backend_(config.pgo)
    , registration_(
          config.registration.gicp,
          config.registration.type,
          config.registration.vgicp.voxel_resolution)
{
    latest_result_.T_world_lidar = Isometry3d::Identity();
    gravity_world_ =
        Eigen::Vector3d(0.0, 0.0, -config.imu.init.gravity_mag);

    spdlog::info(
        "[pipeline] ready: registration={}, deskew={}, submap_kf={}",
        config.registration.type,
        config.deskew.time_offset ? "offset" : "zero-ref",
        config.odometry.submap.max_keyframes);
}

OdometryResult OdometryPipeline::latestResult() const {
    OdometryResult result;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        result = latest_result_;
    }
    return applyGlobalCorrection(result, pgo_backend_.T_map_odom());
}

Isometry3d OdometryPipeline::latestPose() const {
    return latestResult().T_world_lidar;
}

PointCloudConstPtr OdometryPipeline::latestDeskewed() const {
    PointCloudConstPtr cloud;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        cloud = latest_deskewed_;
    }
    if (!pgo_backend_.enabled() || !cloud) {
        return cloud;
    }
    auto corrected = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
        *cloud,
        *corrected,
        pgo_backend_.T_map_odom().matrix());
    return corrected;
}

std::optional<OdometryResult>
OdometryPipeline::latestPropagatedResult() const {
    OdometryResult result;
    {
        std::scoped_lock lock(state_mutex_, output_mutex_);
        if (!has_first_scan_.load(std::memory_order_acquire)
            || !propagated_state_.valid) {
            return std::nullopt;
        }

        result = latest_result_;
        result.T_world_lidar =
            propagated_state_.T_world_imu * config_.extrinsics.T_imu_lidar;
        result.v_world = propagated_state_.v_world;
        result.stamp = propagated_state_.stamp;
    }
    return applyGlobalCorrection(result, pgo_backend_.T_map_odom());
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

OdometryPipeline::PreprocessResult OdometryPipeline::preprocessPoints(
    double stamp, const PointCloudConstPtr& points) const
{
    auto output = std::make_shared<PointCloud>();
    output->points.reserve(points->points.size());

    const auto& box = config_.odometry.crop_box;
    double min_time = 0.0;
    double max_time = 0.0;
    bool have_time = false;
    for (const Point& point : points->points) {
        if (!std::isfinite(point.x)
            || !std::isfinite(point.y)
            || !std::isfinite(point.z)) {
            continue;
        }

        // Remove points inside the crop box (robot body / mounting blind zone).
        const bool inside_crop_box =
            point.x >= box.min_x && point.x <= box.max_x
            && point.y >= box.min_y && point.y <= box.max_y
            && point.z >= box.min_z && point.z <= box.max_z;
        if (inside_crop_box) {
            continue;
        }

        output->points.push_back(point);
        if (!have_time) {
            min_time = point.timestamp;
            max_time = point.timestamp;
            have_time = true;
        } else {
            min_time = std::min(min_time, point.timestamp);
            max_time = std::max(max_time, point.timestamp);
        }
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;

    PreprocessResult result;
    result.cloud = output;
    result.scan_end_stamp = have_time
        ? stamp + max_time
            - (config_.deskew.time_offset ? min_time : 0.0)
        : stamp;
    return result;
}

PointCloudConstPtr OdometryPipeline::downsamplePoints(
    const PointCloudConstPtr& points) const
{
    return deterministicVoxelDownsample(
        *points, config_.odometry.voxel_size);
}

DeskewResult OdometryPipeline::deskewPointcloud(
    double stamp, const PointCloudConstPtr& points)
{
    NavigationState baseline;
    ImuBuffer imu_snapshot;
    {
        std::scoped_lock lock(imu_mutex_, state_mutex_);
        if (!imu_state_.valid) {
            throw std::logic_error(
                "OdometryPipeline::deskewPointcloud called without a valid IMU state");
        }
        baseline = imu_state_;
        imu_snapshot = imu_buffer_;
    }

    DeskewResult result = deskew(
        points,
        stamp,
        imu_snapshot,
        baseline.stamp,
        baseline.T_world_imu,
        baseline.v_world,
        config_.extrinsics.T_imu_lidar,
        gravity_world_,
        config_.imu.noise,
        config_.deskew.time_offset);
    return result;
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
    detail::integrateMeasurement(*propagation_pim_, imu, dt);
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
    double stamp, const PreprocessResult& preprocessed)
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
        *preprocessed.cloud, *world_scan, T_world_lidar.matrix());
    const PointCloudConstPtr downsampled = downsamplePoints(world_scan);

    submap_manager_.addKeyframe(T_world_lidar, downsampled, stamp);
    registration_.setTarget(submap_manager_.target());

    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        latest_deskewed_ = downsampled;
        latest_result_.T_world_lidar = T_world_lidar;
        latest_result_.v_world = corrected_state.v_world;
        latest_result_.stamp = stamp;
        latest_result_.converged = true;
    }

    rebasePropagation(corrected_state, accel_bias, gyro_bias);
    pgo_backend_.addFrame(downsampled, T_world_lidar, stamp);

    spdlog::info(
        "[pipeline] first LiDAR target initialized: stamp={:.6f}, "
        "points={}",
        stamp,
        downsampled->size());
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

    registration_.setSource(registration_source);
    RegistrationArtifacts artifacts;
    artifacts.result =
        registration_.align(deskewed.T_world_lidar_ref);
    artifacts.corrected_source = registration_source;

    if (artifacts.result.accepted) {
        auto transformed_source = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *registration_source,
            *transformed_source,
            artifacts.result.T_correction.matrix());
        artifacts.corrected_source = transformed_source;
    }
    return artifacts;
}

void OdometryPipeline::maybeUpdateSubmapTarget(
    const DeskewResult& deskewed,
    const RegistrationArtifacts& artifacts)
{
    if (!artifacts.result.accepted
        || !submap_manager_.shouldAddKeyframe(
            artifacts.result.T_world_lidar)) {
        return;
    }

    if (!submap_manager_.addKeyframe(
            artifacts.result.T_world_lidar,
            artifacts.corrected_source,
            deskewed.reference_stamp)) {
        return;
    }

    registration_.setTarget(submap_manager_.target());
}

void OdometryPipeline::processLidarScan(
    double stamp, const PreprocessResult& preprocessed)
{
    const double scan_end = preprocessed.scan_end_stamp;
    if (!waitForImuCoverage(scan_end)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: timed out waiting for IMU "
            "coverage through {:.6f}",
            scan_end);
        return;
    }

    DeskewResult deskewed = deskewPointcloud(stamp, preprocessed.cloud);
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

    rebasePropagation(
        observer_update.state,
        observer_update.accel_bias,
        observer_update.gyro_bias);
    maybeUpdateSubmapTarget(deskewed, *artifacts);
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        latest_deskewed_ = artifacts->corrected_source;
        latest_result_.T_world_lidar = artifacts->result.T_world_lidar;
        latest_result_.v_world = observer_update.state.v_world;
        latest_result_.stamp = deskewed.reference_stamp;
        latest_result_.converged = artifacts->result.accepted;
    }
    if (artifacts->result.accepted) {
        pgo_backend_.addFrame(
            artifacts->corrected_source,
            artifacts->result.T_world_lidar,
            deskewed.reference_stamp);
    }
}

void OdometryPipeline::pushLidar(
    double stamp, const PointCloudConstPtr& points)
{
    if (!initialized_.load(std::memory_order_acquire)
        || !points || points->empty()) {
        return;
    }

    const PreprocessResult preprocessed = preprocessPoints(stamp, points);
    if (preprocessed.cloud->size()
        < static_cast<size_t>(config_.registration.gicp.min_num_points)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: {} points after preprocessing "
            "(minimum {})",
            preprocessed.cloud->size(),
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
    if (!isFiniteImu(imu)) {
        spdlog::warn("[pipeline] dropping IMU sample with non-finite values");
        return;
    }

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
            spdlog::warn(
                "[pipeline] dropping non-monotonic IMU timestamp: "
                "current={:.9f}, previous={:.9f}",
                imu.stamp,
                imu_buffer_.back().stamp);
            return;
        }
        if (!imu_buffer_.empty()) {
            const double gap = imu.stamp - imu_buffer_.back().stamp;
            if (gap > kImuGapWarningSec) {
                spdlog::warn(
                    "[pipeline] large IMU gap detected: {:.3f}s", gap);
            }
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
