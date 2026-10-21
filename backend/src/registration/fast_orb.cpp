#include <backend/registration/fast_orb.hpp>
#include <backend/sensor/camera/utils.hpp>
#include <opencv2/imgproc.hpp>

namespace sapphire::backend::feature {
namespace {
FeatureParams validated(FeatureParams configuration) {
  configuration.validate();
  return configuration;
}
}
FastOrb::FastOrb(const FeatureParams& configuration)
    : config_(validated(configuration)), scale_factors_(config_.orb_pyramid_levels, 1.f),
      inv_scale_factors_(config_.orb_pyramid_levels, 1.f), image_pyramid_(config_.orb_pyramid_levels, 3),
      detector_(*this), descriptor_(*this) {
  for (int level = 1; level < config_.orb_pyramid_levels; ++level) {
    scale_factors_[level] = config_.orb_pyramid_scale * scale_factors_[level - 1];
    inv_scale_factors_[level] = (1.f / config_.orb_pyramid_scale) * inv_scale_factors_[level - 1];
  }
}

void FastOrb::prepare_pyramid(const cv::Mat& image) {
  image_pyramid_.compute(image, inv_scale_factors_);
  pyramid_source_data_ = image.data;
  pyramid_source_rows_ = image.rows;
  pyramid_source_cols_ = image.cols;
  pyramid_source_type_ = image.type();
  pyramid_source_step_ = image.step;
  pyramid_ready_for_descriptors_ = true;
}

bool FastOrb::can_reuse_pyramid(const cv::Mat& image) const {
  return pyramid_ready_for_descriptors_ && pyramid_source_data_ == image.data && pyramid_source_rows_ == image.rows && pyramid_source_cols_ == image.cols &&
         pyramid_source_type_ == image.type() && pyramid_source_step_ == image.step;
}

void FastOrb::prepare_masks(const cv::Mat& mask, cv::Size size) {
  camera::validateImageMask(mask, size);
  if (mask.empty()) { level_masks_.clear(); mask_source_.release(); return; }
  if (mask.size() == mask_source_.size() && cv::norm(mask, mask_source_, cv::NORM_INF) == 0) return;
  mask_source_ = mask.clone();
  level_masks_.clear();
  level_masks_.resize(config_.orb_pyramid_levels);
  for (int level = 0; level < config_.orb_pyramid_levels; ++level) {
    const auto resized = camera::resizeImageMask(mask, image_pyramid_.at(level).size());
    // Exclude the descriptor's full support, including its Gaussian footprint.
    cv::dilate(resized, level_masks_[level], cv::getStructuringElement(cv::MORPH_RECT,
               cv::Size(2 * OrbDescriptor::support_radius + 1, 2 * OrbDescriptor::support_radius + 1)));
  }
}

void FastOrb::detect(const cv::_InputArray& image, std::vector<cv::KeyPoint>& points, const cv::Mat& mask) { detector_.detect(image, points, mask); }
void FastOrb::compute(const cv::_InputArray& image, std::vector<cv::KeyPoint>& points, const cv::_OutputArray& descriptors, const cv::Mat& mask,
                          cv::Mat* storage) {
  descriptor_.compute(image, points, descriptors, mask, storage);
}
void FastOrb::extract(const cv::_InputArray& image, std::vector<cv::KeyPoint>& points, const cv::_OutputArray& descriptors) {
  detect(image, points);
  compute(image, points, descriptors);
}
}  // namespace sapphire::backend::feature
