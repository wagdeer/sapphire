#pragma once
#include "backend/common/sensor_data.hpp"
#include "backend/sensor/lidar/submap_frame.hpp"
#include "app/parameters.h"
namespace sapphire {
// Node-local Gaussian statistics plus capture-time calibrated visual observations.
backend::SensorData makeBackendInput(const SubmapFrame&, const VisualLoopParameters&);
}  // namespace sapphire
