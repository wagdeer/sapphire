#pragma once

#include <sapphire/types.hpp>

#include <algorithm>
#include <cmath>

namespace sapphire {
namespace detail {

constexpr double kMaxIntegrationStepSec = 0.02;

template <typename Pim>
void integrateMeasurement(
    Pim& pim,
    const ImuData& measurement,
    double dt)
{
    if (dt <= 0.0) {
        return;
    }
    const int steps = std::max(
        1, static_cast<int>(std::ceil(dt / kMaxIntegrationStepSec)));
    const double step_dt = dt / static_cast<double>(steps);
    for (int step = 0; step < steps; ++step) {
        pim.integrateMeasurementMeanOnly(
            measurement.accel, measurement.gyro, step_dt);
    }
}

}  // namespace detail
}  // namespace sapphire
