#pragma once

#include <sapphire/types.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

namespace sapphire {

/// Stationary IMU initialization with variance-driven convergence
///
/// Estimates gyro bias, accel bias, and gravity-aligned orientation
/// from a short stationary period.  Uses MAD-based outlier rejection
/// and adaptive convergence timeout — typically 1-3s instead of a
/// fixed 3s window.
///
/// Usage:
///   ImuInitializer init(ImuInitConfig{});
///   for (each IMU sample):
///     if (auto r = init.feedImu(stamp, accel, gyro)) {
///       // use r->gyro_bias, r->accel_bias, r->q_gravity
///     }
class ImuInitializer {
public:
    struct Result {
        Eigen::Vector3d gyro_bias;    // rad/s
        Eigen::Vector3d accel_bias;   // m/s²
        Eigen::Quaterniond q_gravity; // IMU → world (Z = up in world frame)
        double accel_std;             // quality indicator (m/s²)
        /// Multiply a raw specific-force sample to obtain m/s².
        double accel_scale = 1.0;
    };

    explicit ImuInitializer(
        const ImuInitConfig& config = ImuInitConfig{},
        std::string accel_unit = "auto");

    /// Feed a raw IMU measurement.
    /// Returns Result when calibration finishes and the acceleration unit is
    /// known. Unknown units return nullopt and do not initialize the caller.
    /// The caller should not feed data after a result is returned.
    std::optional<Result> feedImu(double stamp,
                                   const Eigen::Vector3d& accel,
                                   const Eigen::Vector3d& gyro);

private:
    enum class State {
        WAITING,      // not enough samples yet
        CALIBRATING,  // collecting, checking convergence periodically
        CONVERGED,    // converged — result available
        TIMEOUT,      // timed out — approximate result available
        FAILED        // acceleration unit could not be resolved
    };

    struct Sample {
        double stamp;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    };

    /// Compute median of a vector (in-place, sorts)
    static double computeMedian(std::vector<double>& v);

    /// MAD-based outlier rejection: remove samples where any channel
    /// deviates more than threshold_mad * MAD from the median.
    void rejectOutliers();

    /// Check if variance has converged below thresholds.
    bool checkConvergence();

    /// Compute final result from accumulated samples.
    /// Empty when the acceleration unit cannot be resolved.
    std::optional<Result> computeResult();

    /// g → 9.80665, m/s² → 1. auto uses |accel_mean|.
    std::optional<double> resolveAccelScale(
        const Eigen::Vector3d& accel_mean) const;

    ImuInitConfig cfg_;
    std::string accel_unit_;
    State state_ = State::WAITING;
    std::vector<Sample> samples_;     // contiguous storage, reserved on first use
    double start_stamp_ = 0.0;
    double last_check_stamp_ = 0.0;
};

}  // namespace sapphire
