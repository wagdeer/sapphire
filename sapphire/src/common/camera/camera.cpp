#include "common/camera/camera.hpp"
#include "parameters.h"

namespace sapphire {
CameraModel::CameraModel(const CameraParameters &calibration)
    : model_(calibration.distortion_model == "equidistant"
        ? decltype(model_)(std::in_place_type<Kb4CameraModel>, calibration)
        : decltype(model_)(std::in_place_type<PinholeCameraModel>, calibration)) {}
}  // namespace sapphire
