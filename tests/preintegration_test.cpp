#include <preintegration.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

using Pim = sapphire::preintegration::EquivariantPreintegration<double>;

void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testMeanOnlyMatchesFullIntegration() {
    auto params = std::make_shared<Pim::Params>(
        Eigen::Vector3d(0.0, 0.0, -9.81),
        1.5e-3,
        2.5e-2,
        0.0,
        0.0,
        1.0e-5,
        2.0e-4,
        0.0,
        0.0);

    Pim::Vec10 bias = Pim::Vec10::Zero();
    bias.template head<3>() << 0.01, -0.02, 0.03;
    bias.template segment<3>(3) << 0.1, -0.05, 0.02;

    Pim full(params, bias);
    Pim mean_only(params, bias);
    const Pim::Mat20 initial_cov = mean_only.Cov();
    const Pim::Mat20 initial_jacobian = mean_only.Jxi();

    for (int i = 0; i < 500; ++i) {
        const double t = static_cast<double>(i) * 0.005;
        const Eigen::Vector3d accel(
            0.4 + 0.2 * std::sin(t),
            -0.3 + 0.1 * std::cos(2.0 * t),
            9.81 + 0.15 * std::sin(0.5 * t));
        const Eigen::Vector3d gyro(
            0.02 * std::cos(t),
            -0.03 * std::sin(1.5 * t),
            0.4 + 0.01 * std::cos(0.25 * t));
        const double dt = 0.004 + static_cast<double>(i % 5) * 0.0005;

        full.integrateMeasurement(accel, gyro, dt);
        mean_only.integrateMeasurementMeanOnly(accel, gyro, dt);
    }

    expect(
        full.Upsilon().asMatrix().isApprox(
            mean_only.Upsilon().asMatrix(), 1e-12),
        "mean-only and full integration must produce identical Upsilon");
    expect(
        mean_only.Cov().isApprox(initial_cov, 0.0),
        "mean-only integration must not update covariance");
    expect(
        mean_only.Jxi().isApprox(initial_jacobian, 0.0),
        "mean-only integration must not update bias Jacobian");
}

}  // namespace

int main() {
    testMeanOnlyMatchesFullIntegration();
    std::cout << "All preintegration tests passed\n";
    return EXIT_SUCCESS;
}
