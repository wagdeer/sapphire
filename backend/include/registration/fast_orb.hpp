#pragma once

#include <backend/registration/descriptor/orb.hpp>
#include <backend/registration/detector/fast.hpp>
#include <backend/params/feature.hpp>
#include <backend/registration/utils/light_pyramid/pyramid.h>

namespace sapphire::backend::feature {

// Owns the shared pyramid and masks; detection and description borrow this state.
class FastOrb {
 public:
  explicit FastOrb(const FeatureParams& params);
  FastOrb(const FastOrb&) = delete;
  FastOrb& operator=(const FastOrb&) = delete;
  void extract(const cv::_InputArray& image, std::vector<cv::KeyPoint>& keypoints, const cv::_OutputArray& descriptors);
  void detect(const cv::_InputArray& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = {});
  void compute(const cv::_InputArray& image, std::vector<cv::KeyPoint>& keypoints, const cv::_OutputArray& descriptors, const cv::Mat& mask = {},
               cv::Mat* storage = nullptr);

 private:
  friend class FastDetector;
  friend class OrbDescriptor;
  void prepare_pyramid(const cv::Mat& image);
  bool can_reuse_pyramid(const cv::Mat& image) const;
  void prepare_masks(const cv::Mat& mask, cv::Size size);

  const FeatureParams config_;
  std::vector<float> scale_factors_, inv_scale_factors_;
  Pyramid image_pyramid_;
  std::vector<cv::Mat> level_masks_;
  bool pyramid_ready_for_descriptors_ = false;

  cv::Mat mask_source_;
  const std::uint8_t* pyramid_source_data_ = nullptr;
  int pyramid_source_rows_ = 0, pyramid_source_cols_ = 0, pyramid_source_type_ = -1;
  std::size_t pyramid_source_step_ = 0;
  FastDetector detector_;
  OrbDescriptor descriptor_;
};

}  // namespace sapphire::backend::feature
