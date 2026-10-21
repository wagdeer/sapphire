#pragma once

#include "backend/sensor/camera/image_utils.hpp"
#include "frontend/common/measurement.hpp"
#include "app/parameters.h"

namespace sapphire {
// Apply this camera's clock offset once at ingress. Calibration already refers
// to the uniform image_algo_size; no second intrinsic scaling or rectification here.
inline bool preprocessImage(ImageMeas &image, const CameraParameters &camera) {
  const double stamp = image.timestamp + camera.time_offset;
  if (!std::isfinite(stamp) || image.gray.empty() || image.gray.type() != CV_8UC1 ||
      image.gray.cols != (camera.input_width ? camera.input_width : camera.width) ||
      image.gray.rows != (camera.input_height ? camera.input_height : camera.height)) return false;
  image.gray = resizeGrayArea(image.gray, {camera.width, camera.height});
  image.timestamp = stamp;
  image.owner.reset(); // Only the bounded owned pixels survive asynchronous admission.
  return true;
}
}  // namespace sapphire
