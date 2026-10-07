#pragma once
#include "common/visual_frame.hpp"
#include "common/common.hpp"
#include "parameters.h"

namespace sapphire {
VisualFrame extractLoopFeatures(const ImageMeas &image, const VisualLoopParameters &config);
}
