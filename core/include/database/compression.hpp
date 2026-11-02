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
// Read only the archive prefix and matrix trailer for admission. Decode still
// validates their counts against the complete compressed geometry.
GaussianCloudSize decoded_size(const void* prefix, std::size_t prefix_size,
                              const void* trailer, std::size_t trailer_size);
}  // namespace sapphire::lidar
