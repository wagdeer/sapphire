#pragma once
#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include <backend/registration/matcher/stereo_zncc.hpp>
#include "backend/sensor/camera/perspective.hpp"

namespace sapphire::backend {
namespace feature {
class FastOrb;
}  // namespace feature
}  // namespace sapphire::backend

namespace sapphire::backend {

// FAST detection with ORB descriptors and sparse stereo. Calls are serial.
class FastOrbStereo final {
 public:
  explicit FastOrbStereo(const ParametersMap& parameters = ParametersMap());
  explicit FastOrbStereo(const FeatureParams& configuration);
  ~FastOrbStereo();

  /** @brief Selects the strongest keypoints while preserving original indices. */
  static void selectStrongest(const std::vector<cv::KeyPoint>& keypoints, std::vector<bool>& selected, int maxKeypoints);

  int getMaxFeatures() const { return maxFeatures_; }
  // Per-camera budget changes use the same extractor setup without parsing a configuration map.
  void setFeatureBudget(int features);

  /** @brief Detects SIMD ORB keypoints in a grayscale @p image (CV_8UC1). */
  std::vector<cv::KeyPoint> generateKeypoints(const cv::Mat& image, const cv::Mat& mask = {});
  /** @brief Computes descriptors for @p keypoints (may shrink the list in some detectors). */
  cv::Mat generateDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = {}) const;
  // Preserve ORB support filtering and orientations without computing its bytes.
  void prepareKeypoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = {}) const;

  std::vector<cv::Point3f> stereo_points(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::KeyPoint>& points,
                                         const camera::Perspective& camera, const cv::Mat& right_mask = {}, const cv::Mat& left_mask = {});

  // Prepare image workspaces with disposable input; no measurements escape.
  void prepare(cv::Size size, bool stereo);

  const FeatureParams& configuration() const { return config_; }

 private:
  void resetExtractor();
  FeatureParams config_;
  int maxFeatures_;  // Working budget allocated per camera; not a file reload.

  std::unique_ptr<feature::FastOrb> extractor_;
  StereoZncc stereo_zncc_;
};

}  // namespace sapphire::backend
