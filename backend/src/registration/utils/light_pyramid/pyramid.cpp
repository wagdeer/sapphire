#include "backend/registration/utils/light_pyramid/pyramid.h"

#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "backend/registration/utils/light_pyramid/gaussian_blur_7x7.h"

namespace sapphire::backend::feature {

void Pyramid::blur_level(const std::size_t level) {
  if (level >= levels_.size()) {
    throw std::out_of_range("pyramid blur level is out of range");
  }
  if (level_states_[level] != level_state_t::padded) {
    throw std::logic_error("pyramid level must be freshly computed before descriptor blur");
  }
  if (padding_ < 3) {
    throw std::logic_error("pyramid padding is insufficient for 7x7 Gaussian blur");
  }

  const cv::Mat source = levels_[level];
  const cv::Mat& buffer = buffers_[level];
  cv::Size whole_size;
  cv::Point offset;
  source.locateROI(whole_size, offset);
  const auto* expected_data = buffer.ptr<std::uint8_t>(padding_) + padding_;
  if (source.type() != CV_8UC1 || buffer.type() != CV_8UC1 || offset.x != padding_ || offset.y != padding_ || whole_size != buffer.size() ||
      source.step != buffer.step || source.data != expected_data) {
    throw std::logic_error("descriptor blur requires a Pyramid-owned padded level");
  }

#if defined(SIMD_NEON)
  gaussian_detail::blur_7x7_sigma_2_fixed_point(source, blurred_levels_[level], gaussian_workspaces_[level]);
#else
  cv::GaussianBlur(source, blurred_levels_[level], cv::Size(7, 7), 2.0, 2.0, cv::BORDER_REFLECT_101);
#endif
  levels_[level] = blurred_levels_[level];
  level_states_[level] = level_state_t::blurred;
}

}  // namespace sapphire::backend::feature
