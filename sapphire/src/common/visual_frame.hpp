#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sapphire {

struct ImageMeas {
  double timestamp = -1.0;
  cv::Mat gray;
  std::shared_ptr<const void> owner;

  ImageMeas(double stamp, cv::Mat image, std::shared_ptr<const void> image_owner = {})
      : timestamp(stamp), gray(std::move(image)), owner(std::move(image_owner)) {
    if (gray.empty() || gray.channels() != 1 || gray.depth() != CV_8U) {
      throw std::invalid_argument("Image must be a single-channel 8-bit grayscale image!");
    }
  }

  ImageMeas(const ImageMeas &) = delete;
  ImageMeas &operator=(const ImageMeas &) = delete;
  ImageMeas(ImageMeas &&) noexcept = default;
  ImageMeas &operator=(ImageMeas &&) noexcept = default;
};

class VisualPoint final {
 public:
  VisualPoint(Eigen::Vector2f pixel, int level, float response) : pixel_(std::move(pixel)), level_(level), response_(response) {}

  const Eigen::Vector2f &pixel() const noexcept { return pixel_; }
  int level() const noexcept { return level_; }
  float response() const noexcept { return response_; }

 private:
  Eigen::Vector2f pixel_;
  int level_;
  float response_;
};

class VisualFrame final {
 public:
  explicit VisualFrame(double timestamp) : timestamp_(timestamp) {}

  VisualFrame(const VisualFrame &) = delete;
  VisualFrame &operator=(const VisualFrame &) = delete;
  VisualFrame(VisualFrame &&) noexcept = default;
  VisualFrame &operator=(VisualFrame &&) noexcept = default;

  void update_features(std::vector<VisualPoint> points, cv::Mat descriptors, std::shared_ptr<const void> descriptor_owner = {}) {
    if (!descriptors.empty() && (descriptors.channels() != 1 || (descriptors.depth() != CV_8U && descriptors.depth() != CV_32F) ||
                                 descriptors.rows != static_cast<int>(points.size()))) {
      throw std::invalid_argument("Feature descriptors must be row-aligned CV_8U or CV_32F matrices");
    }
    points_ = std::move(points);
    descriptors_ = std::move(descriptors);
    descriptor_owner_ = std::move(descriptor_owner);
  }

  double timestamp() const noexcept { return timestamp_; }
  const std::vector<VisualPoint> &points() const noexcept { return points_; }
  const cv::Mat &descriptors() const noexcept { return descriptors_; }
  bool has_features() const noexcept { return !points_.empty() && !descriptors_.empty(); }

 private:
  double timestamp_;
  std::vector<VisualPoint> points_;
  cv::Mat descriptors_;
  std::shared_ptr<const void> descriptor_owner_;
};

struct FeatureMatchingPair {
  uint32_t query_index = 0;
  uint32_t candidate_index = 0;
  float distance = 0.0F;
};

class VisualLoopCandidate final {
 public:
  VisualLoopCandidate(uint64_t query_frame_id, uint64_t candidate_frame_id, float retrieval_score)
      : query_frame_id_(query_frame_id), candidate_frame_id_(candidate_frame_id), retrieval_score_(retrieval_score) {}

  void update_matching_pairs(std::vector<FeatureMatchingPair> pairs) { matching_pairs_ = std::move(pairs); }

  uint64_t query_frame_id() const noexcept { return query_frame_id_; }
  uint64_t candidate_frame_id() const noexcept { return candidate_frame_id_; }
  float retrieval_score() const noexcept { return retrieval_score_; }
  const std::vector<FeatureMatchingPair> &matching_pairs() const noexcept { return matching_pairs_; }

 private:
  uint64_t query_frame_id_;
  uint64_t candidate_frame_id_;
  float retrieval_score_;
  std::vector<FeatureMatchingPair> matching_pairs_;
};

}  // namespace sapphire