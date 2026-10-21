#include "backend/sensor/camera/utils.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace sapphire::backend::camera {
void makeRectificationMaps(const KB4& source, const Eigen::Matrix3d& rotation,
                           const Eigen::Matrix3d& intrinsics, cv::Size size, cv::Mat& map1, cv::Mat& map2) {
  if (!rotation.allFinite() || !intrinsics.allFinite() || size.width <= 0 || size.height <= 0 ||
      intrinsics(0, 0) <= 0 || intrinsics(1, 1) <= 0)
    throw std::invalid_argument("Invalid rectification geometry");
  const cv::Matx33d k(source.fx(), 0, source.cx(), 0, source.fy(), source.cy(), 0, 0, 1);
  cv::Matx33d r, p;
  for (int y = 0; y < 3; ++y) for (int x = 0; x < 3; ++x) { r(y, x) = rotation(y, x); p(y, x) = intrinsics(y, x); }
  const auto& d = source.distortion();
  // Preserve OpenCV's fixed-point interpolation and border rounding exactly.
  cv::fisheye::initUndistortRectifyMap(k, cv::Vec4d(d[0], d[1], d[2], d[3]), r, p, size, CV_16SC2, map1, map2);
}

cv::Rect imageRoi(const cv::Size& imageSize, const std::vector<float>& roiRatios) {
  if (imageSize.height != 0 && imageSize.width != 0 && roiRatios.size() == 4) {
    float width = imageSize.width;
    float height = imageSize.height;
    cv::Rect roi(0, 0, width, height);
    EXT_LOG_DEBUG("roi ratios = {:.6f}, {:.6f}, {:.6f}, {:.6f}", roiRatios[0], roiRatios[1], roiRatios[2], roiRatios[3]);
    EXT_LOG_DEBUG("roi = {}, {}, {}, {}", roi.x, roi.y, roi.width, roi.height);

    // left roi
    if (roiRatios[0] > 0 && roiRatios[0] < 1.0f - roiRatios[1]) {
      roi.x = width * roiRatios[0];
    }

    // right roi
    if (roiRatios[1] > 0 && roiRatios[1] < 1.0f - roiRatios[0]) {
      roi.width -= width * roiRatios[1];
    }
    roi.width -= roi.x;

    // top roi
    if (roiRatios[2] > 0 && roiRatios[2] < 1.0f - roiRatios[3]) {
      roi.y = height * roiRatios[2];
    }

    // bottom roi
    if (roiRatios[3] > 0 && roiRatios[3] < 1.0f - roiRatios[2]) {
      roi.height -= height * roiRatios[3];
    }
    roi.height -= roi.y;
    EXT_LOG_DEBUG("roi = {}, {}, {}, {}", roi.x, roi.y, roi.width, roi.height);

    return roi;
  } else {
    EXT_LOG_ERROR("Image is null or _roiRatios(={}) != 4", (int)roiRatios.size());
    return cv::Rect();
  }
}

cv::Mat decimateImage(const cv::Mat& image, int decimation) {
  EXT_ASSERT(decimation >= 1);
  cv::Mat out;
  if (!image.empty()) {
    if (decimation > 1) {
      if ((image.type() == CV_32FC1 || image.type() == CV_16UC1)) {
        EXT_ASSERT_MSG(image.rows % decimation == 0 && image.cols % decimation == 0, "{}",
                       fmt::format("Decimation of depth images should be exact! (decimation={}, size={}x{})", decimation, image.cols, image.rows));

        out = cv::Mat(image.rows / decimation, image.cols / decimation, image.type());
        if (image.type() == CV_32FC1) {
          for (int j = 0; j < out.rows; ++j) {
            for (int i = 0; i < out.cols; ++i) {
              out.at<float>(j, i) = image.at<float>(j * decimation, i * decimation);
            }
          }
        } else  // CV_16UC1
        {
          for (int j = 0; j < out.rows; ++j) {
            for (int i = 0; i < out.cols; ++i) {
              out.at<unsigned short>(j, i) = image.at<unsigned short>(j * decimation, i * decimation);
            }
          }
        }
      } else {
        cv::resize(image, out, cv::Size(), 1.0f / float(decimation), 1.0f / float(decimation), cv::INTER_AREA);
      }
    } else {
      out = image;
    }
  }
  return out;
}


float sampleDepth(const cv::Mat& depth, float x, float y, bool smoothing, float errorRatio, bool estimateMissing) {
  if (depth.empty() || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
    throw std::invalid_argument("sampleDepth requires CV_16UC1 or CV_32FC1 depth");
  }
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return 0.0f;
  int column = static_cast<int>(x + 0.5f);
  int row = static_cast<int>(y + 0.5f);
  if (column == depth.cols && x < static_cast<float>(depth.cols)) --column;
  if (row == depth.rows && y < static_cast<float>(depth.rows)) --row;
  if (column < 0 || column >= depth.cols || row < 0 || row >= depth.rows) return 0.0f;

  const auto at = [&](int r, int c) {
    if (depth.type() == CV_32FC1) {
      const float value = depth.at<float>(r, c);
      return std::isfinite(value) && value > 0.0f ? value : 0.0f;
    }
    const std::uint16_t value = depth.at<std::uint16_t>(r, c);
    return value > 0U && value < std::numeric_limits<std::uint16_t>::max() ? static_cast<float>(value) * 0.001f : 0.0f;
  };

  float center = at(row, column);
  const int firstColumn = std::max(column - 1, 0);
  const int lastColumn = std::min(column + 1, depth.cols - 1);
  const int firstRow = std::max(row - 1, 0);
  const int lastRow = std::min(row + 1, depth.rows - 1);
  if (center == 0.0f && estimateMissing) {
    float sum = 0.0f;
    int count = 0;
    for (int r = firstRow; r <= lastRow; ++r) {
      for (int c = firstColumn; c <= lastColumn; ++c) {
        if ((r == row) == (c == column)) continue;
        const float value = at(r, c);
        if (value == 0.0f) continue;
        if (count == 0 || std::abs(value - sum / static_cast<float>(count)) < errorRatio * sum / static_cast<float>(count)) {
          sum += value;
          ++count;
        }
      }
    }
    if (count > 1) center = sum / static_cast<float>(count);
  }
  if (center == 0.0f || !smoothing) return center;

  float weightedSum = center * 4.0f;
  float weight = 4.0f;
  for (int r = firstRow; r <= lastRow; ++r) {
    for (int c = firstColumn; c <= lastColumn; ++c) {
      if (r == row && c == column) continue;
      const float value = at(r, c);
      if (value == 0.0f || std::abs(value - center) >= errorRatio * center) continue;
      const float sampleWeight = r == row || c == column ? 2.0f : 1.0f;
      weightedSum += value * sampleWeight;
      weight += sampleWeight;
    }
  }
  return weightedSum / weight;
}

cv::Mat depthToMillimeters(const cv::Mat& depth32F) {
  EXT_ASSERT(depth32F.empty() || depth32F.type() == CV_32FC1);
  cv::Mat depth16U;
  if (!depth32F.empty()) {
    depth16U = cv::Mat(depth32F.rows, depth32F.cols, CV_16UC1);
    int countOverMax = 0;
    for (int i = 0; i < depth32F.rows; ++i) {
      for (int j = 0; j < depth32F.cols; ++j) {
        float depth = (depth32F.at<float>(i, j) * 1000.0f);
        unsigned short depthMM = 0;
        if (std::isfinite(depth) && depth > 0 && depth < float(USHRT_MAX) - .5f) {
          depthMM = static_cast<unsigned short>(std::lround(depth));
        } else if (std::isfinite(depth) && depth >= float(USHRT_MAX) - .5f) {
          ++countOverMax;
        }
        depth16U.at<unsigned short>(i, j) = depthMM;
      }
    }
    if (countOverMax) {
      // Sparse invalid returns can occur on every depth frame. Aggregate them
      // instead of spending CPU and I/O on one warning per frame.
      static std::atomic<uint64_t> frames{0}, values{0}, maximum{0};
      static std::atomic<int64_t> lastLogNs{0};
      frames.fetch_add(1, std::memory_order_relaxed);
      values.fetch_add(static_cast<uint64_t>(countOverMax), std::memory_order_relaxed);
      auto previousMax = maximum.load(std::memory_order_relaxed);
      while (previousMax < static_cast<uint64_t>(countOverMax) &&
             !maximum.compare_exchange_weak(previousMax, static_cast<uint64_t>(countOverMax), std::memory_order_relaxed)) {}
      const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      auto previousLog = lastLogNs.load(std::memory_order_relaxed);
      if ((!previousLog || now - previousLog >= 5000000000LL) &&
          lastLogNs.compare_exchange_strong(previousLog, now, std::memory_order_relaxed)) {
        EXT_LOG_WARN(
            "Depth range filter (5s window): frames={} ignored_values={} max_per_frame={}; "
            "values >=65535 mm were discarded (32-bit input is expected in meters)",
            frames.exchange(0, std::memory_order_relaxed), values.exchange(0, std::memory_order_relaxed),
            maximum.exchange(0, std::memory_order_relaxed));
      }
    }
  }
  return depth16U;
}

void validateImageMask(const cv::Mat& mask, cv::Size size) {
  if (mask.empty()) return;
  if (mask.type() != CV_8UC1 || (!size.empty() && mask.size() != size))
    throw std::invalid_argument("Feature mask must be mono8 and match the image dimensions");
}
cv::Mat loadImageMask(const std::string& path) {
  if (path.empty()) return {};
  if (std::filesystem::path(path).extension() != ".png")
    throw std::invalid_argument("Feature mask must be a PNG: " + path);
  auto mask = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (mask.empty()) throw std::invalid_argument("Cannot read feature mask: " + path);
  cv::Mat nonbinary;
  cv::inRange(mask, cv::Scalar(1), cv::Scalar(254), nonbinary);
  if (cv::countNonZero(nonbinary))
    throw std::invalid_argument("Feature mask must contain only black (0) and white (255): " + path);
  return mask;
}
cv::Mat resizeImageMask(const cv::Mat& mask, cv::Size size) {
  if (mask.empty()) return {};
  validateImageMask(mask);
  if (size.empty()) throw std::invalid_argument("Feature mask target size must be positive");
  if (mask.size() == size) return mask;
  cv::Mat floating, resized, result;
  mask.convertTo(floating, CV_32F);
  cv::resize(floating, resized, size, 0, 0,
             size.width <= mask.cols && size.height <= mask.rows ? cv::INTER_AREA : cv::INTER_NEAREST);
  cv::compare(resized, 0, result, cv::CMP_GT);
  return result;
}

}  // namespace sapphire::backend::camera
