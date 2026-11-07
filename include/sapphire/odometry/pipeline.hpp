#pragma once

#include <sapphire/types.hpp>
#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/imu_init.hpp>
#include <sapphire/odometry/registration.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>
#include <sapphire/odometry/eskf.hpp>
#include <sapphire/odometry/observer.hpp>
#include <sapphire/odometry/submap.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
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
///      pushLidar returns until initialization converges.
///   2. INITIALIZED: IMU is bias-corrected and buffered. The first
///      scan sets the registration target. Later scans run the full
///      pipeline and may block for IMU coverage; see pushLidar().
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

    /// Wall-clock budget for a later scan to wait until buffered IMU covers
    /// its newest remaining point time. Sensor time and ROS time do not
    /// extend this wait.
    static constexpr std::chrono::milliseconds kImuCoverageWait{500};

    /// Feed one LiDAR scan.
    ///
    /// Returns immediately, leaving latestResult() unchanged, until IMU
    /// initialization has completed, or when `points` is null or empty.
    /// The first accepted scan does not wait.
    ///
    /// Each later scan waits until the newest buffered IMU sample reaches
    /// the latest finite point timestamp kept by preprocessing, for at most
    /// kImuCoverageWait. The IMU mutex is released while waiting, so
    /// pushImu() on another thread can satisfy the wait. If it is still
    /// uncovered at the deadline, the scan is dropped and latestResult()
    /// stays unchanged. A single-threaded caller must push IMU through
    /// that scan end before calling pushLidar().
    void pushLidar(double stamp, const PointCloudConstPtr& points);

    /// Feed raw IMU data. During initialization, routed to ImuInitializer.
    /// After initialization, scaled to m/s², bias-corrected, and buffered.
    /// Buffering a sample wakes a pushLidar() waiting for coverage.
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

    /// Retrieve the most recent deskewed point cloud in the world frame.
    PointCloudConstPtr latestDeskewed() const;

    /// Submap diagnostics, primarily useful for deterministic verification.
    size_t keyframeCount() const { return submap_manager_.keyframeCount(); }
    size_t submapTargetRevision() const {
        return submap_manager_.targetRevision();
    }
    /// Lidar-rate trajectory in the map frame. Empty when pose-graph
    /// optimization is disabled. History follows each sample's keyframe.
    MapTrajectory mapTrajectory() const { return pgo_backend_.mapTrajectory(); }

    PoseGraphSnapshot poseGraphSnapshot() const {
        return pgo_backend_.snapshot();
    }
    void requestPoseGraphSnapshot() { pgo_backend_.requestSnapshot(); }
    void requestPoseGraphMap() { pgo_backend_.requestGlobalMap(); }
    PointCloudConstPtr latestPoseGraphMap() const {
        return pgo_backend_.latestGlobalMap();
    }
    void requestOccupancyGrid() { pgo_backend_.requestOccupancyGrid(); }
    std::shared_ptr<const OccupancyGridMsg> latestOccupancyGrid() const {
        return pgo_backend_.latestOccupancyGrid();
    }

private:
    struct PreprocessResult {
        PointCloudConstPtr cloud;
        double scan_end_stamp = 0.0;
    };

    struct RegistrationArtifacts {
        RegistrationResult result;
        PointCloudConstPtr corrected_source;
    };

    bool initializeFirstLidarTarget(
        double stamp, const PreprocessResult& preprocessed);
    void processLidarScan(const PreprocessResult& preprocessed);
    void maybeUpdateSubmapTarget(
        const Isometry3d& T_world_lidar,
        const PointCloudConstPtr& cloud,
        bool force_keyframe = false);
    void finalizeImuInitialization(
        ImuInitializer::Result&& result,
        const ImuData& trigger_sample);

    /// Anchor an accepted odom-frame lidar pose to the latest keyframe.
    /// addFrame() for this scan must already have run.
    void rememberPoseGraphSample(
        const Isometry3d& T_odom_lidar,
        const Eigen::Vector3d& v_odom,
        double stamp);

    /// Drop non-finite xyz and timestamps, then apply the crop box.
    /// scan_end_stamp is the maximum remaining absolute timestamp.
    PreprocessResult preprocessPoints(const PointCloudConstPtr& points) const;

    /// Uniformly downsample a world-frame deskewed scan for registration.
    PointCloudConstPtr downsamplePoints(
        const PointCloudConstPtr& points) const;

    /// Motion-compensate a preprocessed scan using buffered IMU data.
    DeskewResult deskewPointcloud(const PointCloudConstPtr& points);

    /// Wait until buffered IMU covers end_stamp, for at most
    /// kImuCoverageWait. The IMU mutex is released while waiting.
    /// Timeout returns false and the caller drops the scan.
    bool waitForImuCoverage(double end_stamp);

    /// Integrate one new IMU sample from the current corrected LiDAR baseline.
    /// state_mutex_ must be held by the caller.
    void propagateStateLocked(const ImuData& imu);

    /// Shift buffered IMU samples from the previous bias onto `accel_bias`
    /// and `gyro_bias`. imu_mutex_ and state_mutex_ must be held.
    void shiftImuBiasLocked(
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias);

    /// Shift the IMU buffer onto the new bias, commit `state` as the ESKF
    /// baseline with the caller-supplied covariance, and replay to the tip.
    /// imu_mutex_ and state_mutex_ must be held.
    ///
    /// `correctAt` leaves the reference posterior in the tip and does not
    /// commit the baseline. The LiDAR update therefore passes tipCovariance()
    /// after that call. This is the only ESKF baseline commit after the
    /// first scan.
    void commitEskfBaselineLocked(
        const NavigationState& state,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias,
        const Eskf::Mat15& covariance);

    /// Install a corrected baseline and replay buffered IMU.
    ///
    /// Used by the first LiDAR scan and by later observer updates. An
    /// already-initialized ESKF is not committed here; that posterior goes
    /// through commitEskfBaselineLocked.
    void rebasePropagation(
        const NavigationState& corrected_state,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias);

    /// Recover propagated_state_ from the preintegrated mean and imu_state_.
    /// state_mutex_ must be held by the caller.
    void recoverPropagatedStateLocked(double stamp);

    bool useEskf() const {
        return config_.odometry.fusion == "eskf";
    }

    static EskfConfig makeEskfConfig(const Config& config);

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
    NavigationState imu_state_;
    NavigationState propagated_state_;
    detail::MeanOnlyGal3Integrator propagation_integrator_;
    Eskf eskf_;
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    /// Raw specific force → m/s², resolved at IMU initialization.
    double accel_unit_scale_ = 1.0;
    bool warned_gyro_unit_ = false;
    mutable std::mutex state_mutex_;
    Eigen::Vector3d gravity_world_ = Eigen::Vector3d::Zero();

    // ── First scan / synchronous local submap state ───────────────
    std::atomic<bool> has_first_scan_{false};
    SubmapManager submap_manager_;
    PoseGraphBackend pgo_backend_;

    // ── GICP Registration ─────────────────────────────────────────
    Registration registration_;
    std::size_t registration_attempt_count_ = 0;
    std::size_t registration_reject_count_ = 0;
    std::size_t registration_reject_streak_ = 0;
    std::size_t registration_reject_streak_max_ = 0;
    std::size_t registration_accept_streak_ = 0;
    std::size_t map_update_count_ = 0;
    std::size_t recovery_count_ = 0;
    int recovery_build_frames_remaining_ = 0;
    bool warned_rigid_deskew_ = false;
};

}  // namespace sapphire
