#pragma once

#include <backend/maplayer/navimap/local_grid.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace sapphire::backend {

/** @brief Encodes an 8-bit image as a JPEG @c CV_8UC1 byte row. */
cv::Mat compressImage(const cv::Mat& image);
/** @brief Decodes JPEG or legacy PNG bytes to a @c cv::Mat. */
cv::Mat uncompressImage(const cv::Mat& bytes);

/** @brief Encodes a millimeter depth image as an RVL @c CV_8UC1 byte row. */
cv::Mat compressDepth(const cv::Mat& depth);
/** @brief Decodes an RVL depth buffer to a millimeter depth image (@c CV_16UC1). */
cv::Mat uncompressDepth(const cv::Mat& bytes);
/** @return True when @p bytes is a structurally valid DEPTHRVL buffer header. */
bool isDepthRvl(const cv::Mat& bytes);

/** @brief Compresses a matrix with zlib to a byte row, appending rows, cols and type. */
cv::Mat compressData(const cv::Mat& data);
cv::Mat compressGridCells(const GridCells& cells);
/** @brief Restores a matrix compressed with @ref compressData(). */
cv::Mat uncompressData(const cv::Mat& bytes);

/** @brief Compresses a null-terminated string using @ref compressData(). */
cv::Mat compressString(const std::string& str);
/** @brief Decompresses a string produced by @ref compressString(). */
std::string uncompressString(const cv::Mat& bytes);

} /* namespace sapphire::backend */
