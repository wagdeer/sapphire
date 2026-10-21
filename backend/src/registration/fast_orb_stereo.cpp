#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/parameters.hpp>
#include "backend/registration/fast_orb_stereo.hpp"
#include "backend/solver/triangulator.hpp"

#include <algorithm>
#include <cmath>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/timer.hpp>
#include <list>
#include <numeric>
#include <stdexcept>

#include "backend/registration/fast_orb.hpp"
#include "backend/sensor/camera/utils.hpp"

namespace sapphire::backend {

FastOrbStereo::FastOrbStereo(const ParametersMap& parameters)
    : FastOrbStereo(FeatureParams{}.withOverrides(parameters)) {}

FastOrbStereo::FastOrbStereo(const FeatureParams& configuration)
    : config_(configuration), maxFeatures_(configuration.max_features) {
  config_.validate();
  resetExtractor();
}

FastOrbStereo::~FastOrbStereo() = default;

void FastOrbStereo::selectStrongest(const std::vector<cv::KeyPoint>& keypoints, std::vector<bool>& selected, int maxKeypoints) {
  selected.assign(keypoints.size(), true);
  if (maxKeypoints <= 0 || keypoints.size() <= static_cast<std::size_t>(maxKeypoints)) {
    return;
  }

  std::vector<std::size_t> indices(keypoints.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::partial_sort(indices.begin(), indices.begin() + maxKeypoints, indices.end(),
                    [&keypoints](std::size_t left, std::size_t right) { return std::fabs(keypoints[left].response) > std::fabs(keypoints[right].response); });

  selected.assign(keypoints.size(), false);
  for (int i = 0; i < maxKeypoints; ++i) {
    selected[indices[static_cast<std::size_t>(i)]] = true;
  }
}

void FastOrbStereo::setFeatureBudget(int features) {
  if (features == maxFeatures_) return;
  auto candidate = config_;
  candidate.max_features = features;
  candidate.validate();
  maxFeatures_ = features;
  resetExtractor();
}

void FastOrbStereo::resetExtractor() {
  auto configuration = config_;
  configuration.max_features = maxFeatures_;
  extractor_ = std::make_unique<feature::FastOrb>(configuration);
}

std::vector<cv::KeyPoint> FastOrbStereo::generateKeypoints(const cv::Mat& image, const cv::Mat& mask) {
  EXT_ASSERT(!image.empty() && image.type() == CV_8UC1);
  if (maxFeatures_ < 0) {
    return {};
  }

  tools::Timer timer;
  std::vector<cv::KeyPoint> keypoints;
  extractor_->detect(image, keypoints, mask);

  if (!mask.empty()) {
    keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), [&](const auto& key) {
      const int x = cvRound(key.pt.x), y = cvRound(key.pt.y);
      return x < 0 || y < 0 || x >= mask.cols || y >= mask.rows || mask.at<uint8_t>(y, x) != 0;
    }), keypoints.end());
  }
  EXT_LOG_DEBUG("FAST detection time={:.6f}s keypoints={}", timer.lap_seconds(), static_cast<int>(keypoints.size()));
  return keypoints;
}

void FastOrbStereo::prepareKeypoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const {
  EXT_ASSERT(!image.empty() && image.type() == CV_8UC1);
  extractor_->compute(image, keypoints, cv::noArray(), mask);
}

cv::Mat FastOrbStereo::generateDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const {
  EXT_ASSERT(!image.empty() && image.type() == CV_8UC1);
  cv::Mat descriptors;
  extractor_->compute(image, keypoints, descriptors, mask);
  if (descriptors.rows != static_cast<int>(keypoints.size())) {
    EXT_LOG_ERROR("ORB descriptor count mismatch: descriptors={} keypoints={}", descriptors.rows, static_cast<int>(keypoints.size()));
    keypoints.clear();
    descriptors.release();
  }
  return descriptors;
}

std::vector<cv::Point3f> FastOrbStereo::stereo_points(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::KeyPoint>& points,
                                               const camera::Perspective& camera, const cv::Mat& right_mask, const cv::Mat& left_mask) {
  if (right.empty()) return std::vector<cv::Point3f>(points.size(), {NAN, NAN, NAN});
  if (right.type() != CV_8U || left.size() != right.size()) throw std::invalid_argument("Stereo requires same-size rectified grayscale images");
  return stereo_zncc_.match(left, right, points, camera, left_mask, right_mask);
}

void FastOrbStereo::prepare(cv::Size size, bool stereo) {
  if (size.width <= 0 || size.height <= 0) throw std::invalid_argument("Feature workspace requires image dimensions");
  cv::Mat image(size, CV_8U);
  cv::RNG(0x53544552).fill(image, cv::RNG::UNIFORM, 0, 256);
  auto points = generateKeypoints(image);
  generateDescriptors(image, points);
  if (stereo) stereo_points(image, image, points,
      camera::Perspective(300, 300, size.width * .5, size.height * .5, StateGroupf::identity(), size, 30));
}

}  // namespace sapphire::backend
