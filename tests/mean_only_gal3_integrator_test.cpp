#include <sapphire/odometry/detail/integrate_measurement.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>

#include <SO3.hpp>

#include <preintegration.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

using ReferencePim =
    sapphire::preintegration::EquivariantPreintegration<double>;
using FastIntegrator = sapphire::detail::MeanOnlyGal3Integrator;

void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testMatchesEquivariantMeanOnlyPath() {
    auto params = std::make_shared<ReferencePim::Params>(
        Eigen::Vector3d(0.0, 0.0, -9.80665));
    ReferencePim reference(params);
    FastIntegrator fast;

    for (int i = 0; i < 2000; ++i) {
        const double t = static_cast<double>(i) * 0.005;
        sapphire::ImuData measurement;
        measurement.stamp = t;
        measurement.accel = Eigen::Vector3d(
            0.8 * std::sin(0.7 * t),
            -0.4 * std::cos(1.3 * t),
            9.80665 + 0.2 * std::sin(0.2 * t));
        measurement.gyro = Eigen::Vector3d(
            0.3 * std::cos(0.5 * t),
            -0.2 * std::sin(0.8 * t),
            1.1 + 0.1 * std::cos(0.3 * t));
        const double dt = (i % 197 == 0)
            ? 0.047
            : 0.003 + 0.0005 * static_cast<double>(i % 7);

        sapphire::detail::integrateMeasurement(reference, measurement, dt);
        fast.integrate(measurement, dt);
    }

    expect(
        reference.Upsilon().asMatrix().isApprox(
            fast.Upsilon().asMatrix(), 1e-10),
        "fast mean-only integrator must match equivariant mean propagation");
}

void testRecoveryMatchesPimFormula() {
    const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);
    auto params = std::make_shared<ReferencePim::Params>(gravity);
    ReferencePim reference(params);
    FastIntegrator fast;

    sapphire::ImuData measurement;
    measurement.accel = Eigen::Vector3d(0.7, -0.2, 9.9);
    measurement.gyro = Eigen::Vector3d(0.1, -0.3, 0.8);
    sapphire::detail::integrateMeasurement(reference, measurement, 0.085);
    fast.integrate(measurement, 0.085);

    FastIntegrator::Gal3::IsometriesType initial_isometries{
        Eigen::Vector3d(1.0, -2.0, 0.5),
        Eigen::Vector3d(3.0, 4.0, -1.0),
    };
    const FastIntegrator::Gal3 initial_state(
        lie::SO3d::exp(0.4 * Eigen::Vector3d::UnitY()).R(),
        initial_isometries,
        0.0);

    const FastIntegrator::Gal3 expected =
        reference.Gamma_ij() * initial_state * reference.Upsilon();
    const FastIntegrator::Gal3 actual =
        sapphire::detail::recoverWorldState(
            fast.Upsilon(), initial_state, gravity);
    expect(
        expected.asMatrix().isApprox(actual.asMatrix(), 1e-11),
        "fused world-state recovery must match the PIM formula");
}

}  // namespace

int main() {
    testMatchesEquivariantMeanOnlyPath();
    testRecoveryMatchesPimFormula();
    std::cout << "All mean-only Gal3 integrator tests passed\n";
    return EXIT_SUCCESS;
}
