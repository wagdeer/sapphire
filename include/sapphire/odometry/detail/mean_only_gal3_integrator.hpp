#pragma once

#include <sapphire/types.hpp>

#include <preintegration.hpp>

#include <algorithm>
#include <cmath>

namespace sapphire {
namespace detail {

/// Lightweight Gal(3) integrator for bias-corrected IMU measurements.
///
/// This is the exact mean propagation of EquivariantPreintegration when
/// xi0 is identity, biasHat is zero, and the input bias-rate term is zero.
/// It intentionally does not provide covariance or bias Jacobians.
class MeanOnlyGal3Integrator {
public:
    using Gal3 = lie::Gal3<double>;
    using Vec3 = Eigen::Vector3d;
    using Vec10 = Eigen::Matrix<double, 10, 1>;

    void reset() {
        Upsilon_ = Gal3();
    }

    void integrate(
        const Vec3& accel,
        const Vec3& gyro,
        double dt)
    {
        if (dt <= 0.0) {
            return;
        }

        constexpr double kMaxIntegrationStepSec = 0.02;
        const int steps = std::max(
            1,
            static_cast<int>(
                std::ceil(dt / kMaxIntegrationStepSec)));
        const double step_dt = dt / static_cast<double>(steps);

        Vec10 input;
        input << gyro, accel, Vec3::Zero(), 1.0;
        const Gal3 increment = Gal3::exp(input * step_dt);
        for (int step = 0; step < steps; ++step) {
            Upsilon_.multiplyRight(increment);
        }
    }

    void integrate(const ImuData& measurement, double dt) {
        integrate(measurement.accel, measurement.gyro, dt);
    }

    const Gal3& Upsilon() const {
        return Upsilon_;
    }

    double deltaT() const {
        return Upsilon_.s();
    }

private:
    Gal3 Upsilon_;
};

/// Recover a world-frame navigation state from a preintegrated Gal(3) mean.
inline MeanOnlyGal3Integrator::Gal3 recoverWorldState(
    const MeanOnlyGal3Integrator::Gal3& Upsilon,
    const MeanOnlyGal3Integrator::Gal3& initial_state,
    const Eigen::Vector3d& gravity_world)
{
    using Gal3 = MeanOnlyGal3Integrator::Gal3;
    using Mat5 = Gal3::MatrixType;

    const double dt = Upsilon.s();
    Mat5 gamma = Mat5::Identity();
    gamma.template block<3, 1>(0, 3) = gravity_world * dt;
    gamma.template block<3, 1>(0, 4) =
        -0.5 * gravity_world * dt * dt;
    gamma(3, 4) = -dt;
    return Gal3(gamma) * initial_state * Upsilon;
}

}  // namespace detail
}  // namespace sapphire
