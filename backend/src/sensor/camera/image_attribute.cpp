#include "backend/sensor/camera/image_attribute.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
namespace sapphire {
VisualFrame makeImageAttribute(const ImageMeas &image, const CameraParameters &camera) {
  // Ingress already validated gray pixels and the active calibration.
  ImageProjection projection{camera.width, camera.height, {}};
  std::copy_n(camera.intrinsics.begin(), 4, projection.intrinsics.begin());
  projection.model = camera.distortion_model == "equidistant" ? ImageProjection::Model::Kb4 : ImageProjection::Model::Pinhole;
  projection.distortion = camera.distortion;
  std::vector<std::uint8_t> bytes;
  if (!cv::imencode(".png",image.gray,bytes,{cv::IMWRITE_PNG_COMPRESSION,1}))
    throw std::runtime_error("Image attribute encoding failed");
  if (bytes.size()>VisualFrame::max_image_bytes) throw std::length_error("Image attribute exceeds byte budget");
  VisualFrame frame(image.timestamp,image.camera_id);
  frame.set_projection(std::move(projection));
  frame.set_image_png(std::move(bytes));
  return frame;
}
} // namespace sapphire
