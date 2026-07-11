#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>

namespace sapphire {

OdometryPipeline::OdometryPipeline(const Config& config)
    : config_(config)
    , imu_initializer_(config.imu.init)
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
    spdlog::info("[pipeline]   voxel_size={}, max_points_per_voxel={}",
        config.odometry.voxel_size,
        config.odometry.max_points_per_voxel);
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
    spdlog::info("[pipeline]   observer: velocity_gain={}",
        config.odometry.observer.velocity_gain);
    spdlog::info("[pipeline]   registration: {}", config.registration.type);
    spdlog::info("[pipeline]   cuda: {}", config.cuda.enabled ? "ON" : "OFF");
    spdlog::info("[pipeline]   imu_init: gyro_std<{:.4f}, accel_std<{:.3f}, "
        "timeout={:.1f}s",
        config.imu.init.convergence_gyro_std,
        config.imu.init.convergence_accel_std,
        config.imu.init.timeout_sec);
}

// ── IMU bias correction ─────────────────────────────────────────────

ImuData OdometryPipeline::correctImu(const ImuData& raw) const {
    // correctImu is only called after IMU init completes (caller ensures initialized_)
    assert(imu_init_result_.has_value() && "correctImu called before IMU initialization");
    ImuData corrected;
    corrected.stamp = raw.stamp;
    corrected.gyro  = raw.gyro  - imu_init_result_->gyro_bias;
    corrected.accel = raw.accel - imu_init_result_->accel_bias;
    return corrected;
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

DeskewResult OdometryPipeline::deskewPointcloud(
    double stamp, const PointCloudConstPtr& points)
{
    assert(imu_state_.valid && "deskewPointcloud called without an IMU state");
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return deskew(
        points,
        stamp,
        imu_buffer_,
        imu_state_.stamp,
        imu_state_.T_world_imu,
        imu_state_.v_world,
        config_.extrinsics.T_imu_lidar,
        gravity_world_,
        config_.imu.noise,
        config_.deskew.time_offset);
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

void OdometryPipeline::pushLidar(double stamp, const PointCloudConstPtr& points) {
    if (!initialized_.load(std::memory_order_acquire)
        || !points || points->empty()) {
        return;
    }

    auto preprocessed = preprocessPoints(points);
    if (preprocessed->size()
        < static_cast<size_t>(config_.registration.gicp.min_num_points)) {
        spdlog::warn(
            "[pipeline] skipping LiDAR scan: {} points after preprocessing "
            "(minimum {})",
            preprocessed->size(),
            config_.registration.gicp.min_num_points);
        return;
    }

    // DLIO first-valid-scan path: do not integrate from the calibration
    // timestamp. Assume no motion, apply the gravity-aligned initial pose,
    // and establish this scan as the first registration target.
    if (!has_first_scan_) {
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (imu_buffer_.empty() || imu_buffer_.front().stamp > stamp) {
                return;
            }
        }

        const Isometry3d T_world_lidar =
            imu_state_.T_world_imu * config_.extrinsics.T_imu_lidar;
        auto world_scan = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *preprocessed, *world_scan, T_world_lidar.matrix());

        latest_deskewed_ = world_scan;
        submap_manager_.addKeyframe(T_world_lidar, world_scan, stamp);
        registration_.setTarget(submap_manager_.target());
        spdlog::info(
            "[pipeline] submap target rebuilt: keyframes={}, points={}, "
            "revision={}",
            submap_manager_.keyframeCount(),
            submap_manager_.target()->size(),
            submap_manager_.targetRevision());

        latest_result_.T_world_lidar = T_world_lidar;
        latest_result_.stamp = stamp;
        latest_result_.converged = true;

        imu_state_.stamp = stamp;
        has_first_scan_ = true;

        spdlog::info(
            "[pipeline] first LiDAR target initialized: stamp={:.6f}, "
            "points={}",
            latest_result_.stamp,
            latest_deskewed_->size());
        return;
    }

    const auto [min_time_it, max_time_it] = std::minmax_element(
        preprocessed->points.begin(),
        preprocessed->points.end(),
        [](const Point& lhs, const Point& rhs) {
            return lhs.timestamp < rhs.timestamp;
        });
    const double scan_end =
        stamp + max_time_it->timestamp
        - (config_.deskew.time_offset ? min_time_it->timestamp : 0.0);

    // DLIO waits until IMU data reaches the end of the sweep. The ROS wrapper
    // runs IMU and LiDAR in separate callback groups so this wait cannot block
    // incoming IMU messages.
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

    // DLIO correction flow: the source already contains the IMU world prior,
    // so GICP estimates a global left-multiplicative correction.
    registration_.setSource(deskewed.cloud);
    const RegistrationResult registration =
        registration_.align(deskewed.T_world_lidar_ref);

    PointCloudConstPtr corrected_scan = deskewed.cloud;
    if (registration.accepted) {
        auto transformed = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *deskewed.cloud,
            *transformed,
            registration.T_correction.matrix());
        corrected_scan = transformed;
    }

    latest_deskewed_ = corrected_scan;
    latest_result_.T_world_lidar = registration.T_world_lidar;
    latest_result_.stamp = deskewed.reference_stamp;
    latest_result_.converged = registration.accepted;

    const Isometry3d T_world_imu_prior =
        deskewed.T_world_lidar_ref
        * config_.extrinsics.T_imu_lidar.inverse();
    const Isometry3d T_world_imu_corrected =
        registration.T_world_lidar
        * config_.extrinsics.T_imu_lidar.inverse();
    Eigen::Vector3d corrected_velocity = deskewed.v_world_ref;
    const double state_dt =
        deskewed.reference_stamp - imu_state_.stamp;
    if (registration.accepted && state_dt > 0.0) {
        // DLIO geometric observer: feed the GICP position innovation back
        // into velocity so the next IMU prior does not retain a drift mode.
        const Eigen::Vector3d position_error =
            T_world_imu_corrected.translation()
            - T_world_imu_prior.translation();
        corrected_velocity +=
            state_dt
            * config_.odometry.observer.velocity_gain
            * position_error;
    }

    imu_state_.stamp = deskewed.reference_stamp;
    imu_state_.T_world_imu = T_world_imu_corrected;
    imu_state_.v_world = corrected_velocity;

    // Rejected scans never enter the keyframe history. Accepted scans only
    // update GICP's target when they cross a keyframe threshold and change
    // the active nearest-keyframe set.
    if (registration.accepted
        && submap_manager_.shouldAddKeyframe(
            registration.T_world_lidar)
        && submap_manager_.addKeyframe(
            registration.T_world_lidar,
            corrected_scan,
            deskewed.reference_stamp)) {
        registration_.setTarget(submap_manager_.target());
        spdlog::info(
            "[pipeline] submap target rebuilt: keyframes={}, points={}, "
            "revision={}",
            submap_manager_.keyframeCount(),
            submap_manager_.target()->size(),
            submap_manager_.targetRevision());
    }
}

// ── IMU callback ────────────────────────────────────────────────────

void OdometryPipeline::pushImu(const ImuData& imu) {
    if (!initialized_.load(std::memory_order_acquire)) {
        auto result = imu_initializer_.feedImu(imu.stamp, imu.accel, imu.gyro);
        if (result.has_value()) {
            imu_init_result_ = std::move(result);

            // DLIO propagation starts from a gravity-aligned, stationary state.
            // q_gravity maps vectors from the IMU frame into the world frame.
            imu_state_.stamp = imu.stamp;
            imu_state_.T_world_imu = Isometry3d::Identity();
            imu_state_.T_world_imu.linear() =
                imu_init_result_->q_gravity.toRotationMatrix();
            imu_state_.v_world.setZero();
            imu_state_.valid = true;

            // Calibration samples are raw. Start the deskew buffer at the
            // corrected-state timestamp with bias-corrected measurements only.
            ImuData corrected = correctImu(imu);
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                imu_buffer_.clear();
                imu_buffer_.push_back(corrected);
            }
            initialized_.store(true, std::memory_order_release);
            imu_cv_.notify_all();
        }
        return;
    }

    ImuData corrected = correctImu(imu);
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!imu_buffer_.empty()
            && corrected.stamp <= imu_buffer_.back().stamp) {
            return;
        }
        imu_buffer_.push_back(corrected);
    }
    imu_cv_.notify_all();
}

}  // namespace sapphire
