#include "common/visual_frame.hpp"
#include <algorithm>
#include <opencv2/imgcodecs.hpp>
namespace sapphire {
void VisualFrame::set_image_png(std::vector<std::uint8_t> bytes) {
  const std::uint8_t signature[]{137,80,78,71,13,10,26,10};
  if (!projection_ || !T_odom_camera_ || !points_.empty() || !descriptors_.empty() || bytes.size()<33 || bytes.size()>max_image_bytes ||
      !std::equal(std::begin(signature),std::end(signature),bytes.begin()))
    throw std::invalid_argument("Invalid image attribute payload");
  const auto number=[&](std::size_t i) { return (std::uint32_t(bytes[i])<<24)|(std::uint32_t(bytes[i+1])<<16)|(std::uint32_t(bytes[i+2])<<8)|bytes[i+3]; };
  // Bound allocation before invoking the image decoder. Only8-bit grayscale PNG.
  if(number(8)!=13 || number(12)!=0x49484452 || number(16)!=std::uint32_t(projection_->width) ||
     number(20)!=std::uint32_t(projection_->height) || std::uint64_t(number(16))*number(20)>max_image_pixels ||
     bytes[24]!=8 || bytes[25]!=0)
    throw std::invalid_argument("Invalid image attribute dimensions/encoding");
  const auto decoded=cv::imdecode(bytes,cv::IMREAD_UNCHANGED);
  if(decoded.empty() || decoded.type()!=CV_8UC1 || decoded.cols!=projection_->width || decoded.rows!=projection_->height)
    throw std::invalid_argument("Undecodable image attribute");
  image_png_=std::move(bytes);
}
} // namespace sapphire
