#include <sapphire/odometry/observer.hpp>

#include <SO3.hpp>

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

    const lie::SO3d so3_prior(prior_state.T_world_imu.rotation());
    const lie::SO3d so3_measurement(T_world_imu_measurement.rotation());
    const lie::SO3d so3_error = so3_prior.inv() * so3_measurement;

    const Eigen::Vector3d body_position_error =
        so3_prior.inv() * position_error;
    update.accel_bias -=
        dt * config.accel_bias_gain * body_position_error;
    update.gyro_bias -=
        dt * config.gyro_bias_gain * lie::SO3d::log(so3_error);
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

    Eigen::Vector3d omega = lie::SO3d::log(so3_error);
    omega *= dt * config.orientation_gain;
    const lie::SO3d so3_inc = lie::SO3d::exp(omega);
    update.state.T_world_imu.linear() =
        (so3_prior * so3_inc).R();

    return update;
}

}  // namespace sapphire
