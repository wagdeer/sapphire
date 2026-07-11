#pragma once

#include <sapphire/types.hpp>
#include <sapphire/ring_buffer.hpp>
#include <sapphire/imu_init.hpp>
#include <sapphire/odometry/registration.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <pcl/filters/crop_box.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>

namespace sapphire {

/// OdometryPipeline — LiDAR-inertial front-end odometry
///
/// Lifecycle:
///   1. UNINITIALIZED: pushImu feeds raw IMU to ImuInitializer.
///      pushLidar is silently skipped until initialization converges.
///   2. INITIALIZED: IMU bias-corrected and buffered. pushLidar
///      runs full pipeline (deskew → downsample → registration → …).
///
/// Zero ROS dependency, pure C++17 + Eigen.
class OdometryPipeline {
public:
    explicit OdometryPipeline(const Config& config);

    /// Whether IMU initialization has completed.
    bool initialized() const {
        return initialized_.load(std::memory_order_acquire);
    }

    /// Access the IMU initialization result (valid after initialized()).
    const ImuInitializer::Result& imuInitResult() const { return *imu_init_result_; }

    /// Feed a LiDAR scan. Silently skipped until IMU is initialized.
    void pushLidar(double stamp, const PointCloudConstPtr& points);

    /// Feed raw IMU data. During initialization, routed to ImuInitializer.
    /// After initialization, bias-corrected and buffered for deskew.
    void pushImu(const ImuData& imu);

    /// Retrieve the most recent odometry result
    const OdometryResult& latestResult() const { return latest_result_; }

    /// Shorthand for latest pose
    Isometry3d latestPose() const { return latest_result_.T_world_lidar; }

    /// Retrieve the most recent deskewed point cloud in the world frame.
    PointCloudConstPtr latestDeskewed() const { return latest_deskewed_; }

private:
    /// IMU navigation state at the most recent LiDAR reference timestamp.
    /// This is the corrected baseline used by the next deskew integration,
    /// matching DLIO's previous pose/velocity propagation state.
    struct ImuState {
        double stamp = 0.0;
        Isometry3d T_world_imu = Isometry3d::Identity();
        Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
        bool valid = false;
    };

    /// Apply bias correction.  Requires imu_init_result_ to be set.
    /// Caller must check initialized_ before calling.
    ImuData correctImu(const ImuData& raw) const;

    /// Filter and normalize raw LiDAR points before deskew/registration.
    PointCloudConstPtr preprocessPoints(const PointCloudConstPtr& points) const;

    /// Motion-compensate a preprocessed scan using buffered IMU data.
    DeskewResult deskewPointcloud(double stamp, const PointCloudConstPtr& points);

    /// Wait until buffered IMU measurements cover the requested scan end.
    bool waitForImuCoverage(double end_stamp);

    Config config_;

    // ── Initialization ──────────────────────────────────────────
    std::atomic<bool> initialized_{false};
    ImuInitializer imu_initializer_;
    std::optional<ImuInitializer::Result> imu_init_result_;

    // ── Latest output ───────────────────────────────────────────
    OdometryResult latest_result_;
    PointCloudConstPtr latest_deskewed_;

    // ── IMU buffer: ring buffer for deskew timestamp interpolation
    static constexpr size_t kMaxImuBuffer = 500;  // 200Hz * 2.5s
    RingBuffer<ImuData, kMaxImuBuffer> imu_buffer_;
    std::mutex imu_mutex_;
    std::condition_variable imu_cv_;

    // ── Corrected IMU state used as the next integration baseline ──
    ImuState imu_state_;
    Eigen::Vector3d gravity_world_ = Eigen::Vector3d::Zero();

    // ── Previous scan state ───────────────────────────────────────
    bool has_prev_scan_ = false;

    // ── Previous scan for scan-to-scan registration ────────────────
    PointCloudConstPtr prev_scan_;

    // ── GICP Registration ─────────────────────────────────────────
    Registration registration_;

    // ── Crop Box: remove robot body points in LiDAR frame ────────
    mutable pcl::CropBox<Point> crop_filter_;

    // TODO v0.1:
    //   std::unique_ptr<VoxelMap> voxel_map_;
    //   std::unique_ptr<SubmapManager> submap_manager_;
    //   std::unique_ptr<RegistrationBackend> registration_;
};

}  // namespace sapphire
