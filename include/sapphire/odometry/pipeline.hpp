#pragma once

#include <sapphire/types.hpp>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/imu_init.hpp>
#include <sapphire/odometry/registration.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/observer.hpp>
#include <sapphire/odometry/submap.hpp>
#include <preintegration.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>

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
    /// Throws std::logic_error if called before initialization completes.
    const ImuInitializer::Result& imuInitResult() const {
        if (!imu_init_result_.has_value()) {
            throw std::logic_error(
                "imuInitResult() called before IMU initialization completes");
        }
        return *imu_init_result_;
    }

    /// Feed a LiDAR scan. Silently skipped until IMU is initialized.
    void pushLidar(double stamp, const PointCloudConstPtr& points);

    /// Feed raw IMU data. During initialization, routed to ImuInitializer.
    /// After initialization, bias-corrected and buffered for deskew.
    void pushImu(const ImuData& imu);

    /// Retrieve the most recent LiDAR result in the map frame. When PGO is
    /// disabled, map and odom are identical.
    OdometryResult latestResult() const;

    /// Retrieve the IMU-rate state propagated from the latest LiDAR correction.
    /// Empty until the first LiDAR target has initialized the propagation base.
    std::optional<OdometryResult> latestPropagatedResult() const;

    /// Current total IMU biases, including stationary initialization and
    /// online geometric-observer corrections.
    Eigen::Vector3d accelBias() const;
    Eigen::Vector3d gyroBias() const;

    /// Shorthand for latest pose
    Isometry3d latestPose() const;

    /// Retrieve the most recent deskewed point cloud in the world frame.
    PointCloudConstPtr latestDeskewed() const;

    /// Submap diagnostics, primarily useful for deterministic verification.
    size_t keyframeCount() const { return submap_manager_.keyframeCount(); }
    size_t submapTargetRevision() const {
        return submap_manager_.targetRevision();
    }
    PoseGraphStats poseGraphStats() const { return pgo_backend_.stats(); }
    PoseGraphSnapshot poseGraphSnapshot() const {
        return pgo_backend_.snapshot();
    }
    void requestPoseGraphSnapshot() { pgo_backend_.requestSnapshot(); }
    void requestPoseGraphMap() { pgo_backend_.requestGlobalMap(); }
    PointCloudConstPtr latestPoseGraphMap() const {
        return pgo_backend_.latestGlobalMap();
    }

private:
    struct PreprocessResult {
        PointCloudConstPtr cloud;
        double scan_end_stamp = 0.0;
        double elapsed_ms = 0.0;
    };

    struct DownsampleResult {
        PointCloudConstPtr cloud;
        double elapsed_ms = 0.0;
    };

    struct RegistrationArtifacts {
        RegistrationResult result;
        PointCloudConstPtr corrected_source;
        size_t source_points = 0;
        size_t target_points = 0;
        double downsample_ms = 0.0;
    };

    bool initializeFirstLidarTarget(
        double stamp, const PreprocessResult& preprocessed);
    void processLidarScan(double stamp, const PreprocessResult& preprocessed);
    std::optional<RegistrationArtifacts> runScanRegistration(
        const DeskewResult& deskewed);
    void commitLidarOutputs(
        const DeskewResult& deskewed,
        const RegistrationArtifacts& artifacts,
        const ObserverUpdate& observer_update,
        double preprocess_ms,
        double submap_rebuild_ms);
    double maybeUpdateSubmapTarget(
        const DeskewResult& deskewed,
        const RegistrationArtifacts& artifacts);
    void finalizeImuInitialization(
        ImuInitializer::Result&& result,
        const ImuData& trigger_sample);

    /// Filter and normalize raw LiDAR points before deskew/registration.
    PreprocessResult preprocessPoints(
        double stamp, const PointCloudConstPtr& points) const;

    /// Uniformly downsample a world-frame deskewed scan for registration.
    DownsampleResult downsamplePoints(
        const PointCloudConstPtr& points) const;

    /// Motion-compensate a preprocessed scan using buffered IMU data.
    DeskewResult deskewPointcloud(double stamp, const PointCloudConstPtr& points);

    /// Wait until buffered IMU measurements cover the requested scan end.
    bool waitForImuCoverage(double end_stamp);

    /// Integrate one new IMU sample from the current corrected LiDAR baseline.
    /// state_mutex_ must be held by the caller.
    void propagateStateLocked(const ImuData& imu);

    /// Replace the corrected LiDAR baseline and replay newer buffered IMU data.
    void rebasePropagation(
        const NavigationState& corrected_state,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias);

    /// Recover propagated_state_ from propagation_pim_ and imu_state_.
    /// state_mutex_ must be held by the caller.
    void recoverPropagatedStateLocked(double stamp);

    Config config_;

    // ── Initialization ──────────────────────────────────────────
    std::atomic<bool> initialized_{false};
    ImuInitializer imu_initializer_;
    std::optional<ImuInitializer::Result> imu_init_result_;

    // ── Latest output ───────────────────────────────────────────
    OdometryResult latest_result_;
    PointCloudConstPtr latest_deskewed_;
    mutable std::mutex output_mutex_;

    // ── IMU buffer: ring buffer for deskew timestamp interpolation
    ImuBuffer imu_buffer_;
    std::mutex imu_mutex_;
    std::condition_variable imu_cv_;

    // ── Corrected IMU state used as the next integration baseline ──
    using PropagationPim =
        preintegration::EquivariantPreintegration<double>;
    NavigationState imu_state_;
    NavigationState propagated_state_;
    std::unique_ptr<PropagationPim> propagation_pim_;
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    mutable std::mutex state_mutex_;
    Eigen::Vector3d gravity_world_ = Eigen::Vector3d::Zero();

    // ── First scan / synchronous local submap state ───────────────
    std::atomic<bool> has_first_scan_{false};
    SubmapManager submap_manager_;
    PoseGraphBackend pgo_backend_;

    // ── GICP Registration ─────────────────────────────────────────
    Registration registration_;
    size_t deskew_log_count_ = 0;

    // TODO v0.1:
    //   std::unique_ptr<VoxelMap> voxel_map_;
    //   std::unique_ptr<RegistrationBackend> registration_;
};

}  // namespace sapphire
