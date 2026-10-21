#pragma once
#include "frontend/common/measurement.hpp"
#include "backend/sensor/camera/image_observation.hpp"
#include "app/parameters.h"
namespace sapphire { VisualFrame makeImageAttribute(const ImageMeas&, const CameraParameters&); }
