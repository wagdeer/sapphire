#include <sapphire/odometry/observer.hpp>

#include <cmath>

namespace sapphire {

ObserverUpdate applyGeometricObserver(
    const NavigationState& prior_state,
    const Isometry3d& T_world_imu_measurement,
    double previous_state_stamp,
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias,
    const Config::Odometry::Observer& config,
    bool measurement_accepted)
{
    ObserverUpdate update;
    update.state = prior_state;
    update.accel_bias = accel_bias;
    update.gyro_bias = gyro_bias;

    const double dt = prior_state.stamp - previous_state_stamp;
    if (!measurement_accepted || dt <= 0.0) {
        return update;
    }

    const Eigen::Vector3d position_error =
        T_world_imu_measurement.translation()
        - prior_state.T_world_imu.translation();
    Eigen::Quaterniond q_prior(prior_state.T_world_imu.rotation());
    q_prior.normalize();
    Eigen::Quaterniond q_error =
        q_prior.conjugate()
        * Eigen::Quaterniond(T_world_imu_measurement.rotation());
    q_error.normalize();
    if (q_error.w() < 0.0) {
        q_error.coeffs() *= -1.0;
    }

    const Eigen::Vector3d body_position_error =
        q_prior.conjugate() * position_error;
    update.accel_bias -=
        dt * config.accel_bias_gain * body_position_error;
    update.gyro_bias -=
        dt * config.gyro_bias_gain * q_error.w() * q_error.vec();
    update.accel_bias = update.accel_bias.array()
        .min(config.accel_bias_max)
        .max(-config.accel_bias_max);
    update.gyro_bias = update.gyro_bias.array()
        .min(config.gyro_bias_max)
        .max(-config.gyro_bias_max);

    update.state.T_world_imu.translation() +=
        dt * config.position_gain * position_error;
    update.state.v_world +=
        dt * config.velocity_gain * position_error;

    Eigen::Quaterniond q_correction(
        1.0 - std::abs(q_error.w()),
        q_error.x(),
        q_error.y(),
        q_error.z());
    q_correction = q_prior * q_correction;
    Eigen::Quaterniond q_observer(
        q_prior.w() + dt * config.orientation_gain * q_correction.w(),
        q_prior.x() + dt * config.orientation_gain * q_correction.x(),
        q_prior.y() + dt * config.orientation_gain * q_correction.y(),
        q_prior.z() + dt * config.orientation_gain * q_correction.z());
    q_observer.normalize();
    update.state.T_world_imu.linear() = q_observer.toRotationMatrix();

    return update;
}

}  // namespace sapphire
