#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sapphire {

// Runtime visual triangulation evidence, in this observation's optical camera
// frame (metres). Not LiDAR-associated depth. Legacy archives do not encode it.
enum class VisualDepthSource : std::uint8_t { Temporal = 1, Stereo = 2 };

struct VisualGeometry {
  Eigen::Vector3d position_camera = Eigen::Vector3d::Zero();
  double parallax_rad = 0;
  double max_reprojection_px = 0;
  std::uint32_t observations = 0;
  VisualDepthSource source = VisualDepthSource::Temporal;
};

// Frozen projection of this observation's already rectified pixels.
struct RectifiedProjection {
  int width = 0, height = 0;
  std::array<double, 4> intrinsics{}; // fx, fy, cx, cy
};

struct ImageMeas {
  double timestamp = -1.0;
  cv::Mat gray;
  std::size_t camera_id = 0;

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
  std::uint64_t track_id() const noexcept { return track_id_; }
  const std::optional<VisualGeometry> &geometry() const noexcept { return geometry_; }
  void set_tracking(std::uint64_t id, std::optional<VisualGeometry> geometry = {}) {
    if (!id || (geometry && (!geometry->position_camera.allFinite() || geometry->position_camera.z() <= 0 ||
        !std::isfinite(geometry->parallax_rad) || geometry->parallax_rad <= 0 || geometry->parallax_rad >= 3.141592653589793 ||
        !std::isfinite(geometry->max_reprojection_px) || geometry->max_reprojection_px < 0 || (geometry->source == VisualDepthSource::Temporal ? geometry->observations < 3 :
         geometry->source != VisualDepthSource::Stereo || geometry->observations != 2))))
      throw std::invalid_argument("Invalid visual track / triangulation evidence");
    track_id_ = id;
    geometry_ = std::move(geometry);
  }

 private:
  Eigen::Vector2f pixel_;
  int level_;
  float response_;
  std::uint64_t track_id_ = 0;  // zero = legacy appearance-only; local to tracker lifetime
  std::optional<VisualGeometry> geometry_;
};

class VisualFrame final {
 public:
  explicit VisualFrame(double timestamp, std::size_t camera_id = 0) : timestamp_(timestamp), camera_id_(camera_id) {}

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

  std::size_t camera_id() const noexcept { return camera_id_; }
  double timestamp() const noexcept { return timestamp_; }
  const std::vector<VisualPoint> &points() const noexcept { return points_; }
  const cv::Mat &descriptors() const noexcept { return descriptors_; }
  // Calibrated rectified grayscale image; feature/descriptor storage is empty in this mode.
  void set_image_png(std::vector<std::uint8_t> bytes);
  const std::vector<std::uint8_t> &image_png() const noexcept { return image_png_; }
  bool has_image() const noexcept { return !image_png_.empty(); }
  static constexpr std::size_t max_image_bytes = 2 * 1024 * 1024;
  static constexpr std::size_t max_image_pixels = 1024 * 1024;
  bool has_features() const noexcept { return !points_.empty() && !descriptors_.empty(); }
  const std::optional<Eigen::Isometry3d> &T_odom_camera() const noexcept { return T_odom_camera_; }
  const std::optional<RectifiedProjection> &projection() const noexcept { return projection_; }
  void set_projection(RectifiedProjection projection) {
    if (projection.width <= 0 || projection.height <= 0 ||
        !std::isfinite(projection.intrinsics[0]) || !std::isfinite(projection.intrinsics[1]) ||
        !std::isfinite(projection.intrinsics[2]) || !std::isfinite(projection.intrinsics[3]) ||
        projection.intrinsics[0] <= 0 || projection.intrinsics[1] <= 0)
      throw std::invalid_argument("Invalid rectified visual projection");
    projection_ = projection;
  }
  void set_observation_pose(const Eigen::Isometry3d &pose) {
    if (!pose.matrix().allFinite() || (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(pose.linear().determinant() - 1.0) > 1e-6 ||
        (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9)
      throw std::invalid_argument("Invalid visual observation pose");
    T_odom_camera_ = pose;
  }

 private:
  double timestamp_;
  std::size_t camera_id_ = 0;
  std::vector<VisualPoint> points_;
  cv::Mat descriptors_;
  std::shared_ptr<const void> descriptor_owner_;
  std::vector<std::uint8_t> image_png_;
  std::optional<Eigen::Isometry3d> T_odom_camera_;
  std::optional<RectifiedProjection> projection_;
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
