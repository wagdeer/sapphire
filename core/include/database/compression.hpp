#pragma once

#include <opencv2/core.hpp>
#include "common/common_type.hpp"
#include <string>

namespace sapphire {

cv::Mat compressData(const cv::Mat& data);

cv::Mat uncompressData(const cv::Mat& bytes);

cv::Mat compressString(const std::string& str);

std::string uncompressString(const cv::Mat& bytes);

}  // namespace sapphire

namespace sapphire::lidar {
// Node-local Gaussian geometry; preserve covariance, counts and voxel identity.
cv::Mat encode(const GaussianCloud& cloud);
GaussianCloudPtr decode(const cv::Mat& archive);
}  // namespace sapphire::lidar
