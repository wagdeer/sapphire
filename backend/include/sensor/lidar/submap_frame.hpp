#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include <voxelmaps.hpp>

#include "backend/sensor/lidar/aabb.hpp"
#include "frontend/common/point.hpp"
#include "backend/sensor/camera/image_observation.hpp"

namespace sapphire {

struct OdomPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
};

using OdomPoses = std::vector<OdomPose, Eigen::aligned_allocator<OdomPose>>;

enum class MetaTagType : std::uint8_t {
  kFloat = 0,
  kBinary = 1,
  kPose = 2,
};

class MetaTag final {
 public:
  MetaTag(MetaTagType type, cv::Mat descriptor, std::shared_ptr<const void> owner = {})
      : type_(type), descriptor_(std::move(descriptor)), owner_(std::move(owner)) {
    if (type_ != MetaTagType::kFloat && type_ != MetaTagType::kBinary && type_ != MetaTagType::kPose) {
      throw std::invalid_argument("Unknown meta tag type");
    }
    if (descriptor_.empty() || descriptor_.channels() != 1) {
      throw std::invalid_argument("Meta tag descriptor must be a non-empty single-channel matrix");
    }
    if (type_ == MetaTagType::kBinary && descriptor_.depth() != CV_8U) {
      throw std::invalid_argument("Binary meta tag descriptor must use CV_8U storage");
    }
    if ((type_ == MetaTagType::kFloat || type_ == MetaTagType::kPose) && descriptor_.depth() != CV_32F) {
      throw std::invalid_argument("Float and pose meta tag descriptors must use CV_32F storage");
    }
    if (type_ == MetaTagType::kPose && (descriptor_.rows != 4 || descriptor_.cols != 4)) {
      throw std::invalid_argument("Pose meta tag descriptor must be a 4x4 matrix");
    }
  }

  explicit MetaTag(const Eigen::Isometry3f &pose) : type_(MetaTagType::kPose), descriptor_(4, 4, CV_32FC1) { update_pose(pose); }

  MetaTagType type() const noexcept { return type_; }
  const cv::Mat &descriptor() const noexcept { return descriptor_; }

  void update_pose(const Eigen::Isometry3f &pose) {
    if (type_ != MetaTagType::kPose) {
      throw std::logic_error("Only pose meta tags can be updated with a pose");
    }
    Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> descriptor(descriptor_.ptr<float>());
    descriptor = pose.matrix();
  }

 private:
  MetaTagType type_;
  cv::Mat descriptor_;
  std::shared_ptr<const void> owner_;
};

class SubmapFrame final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using MetaTags = std::array<std::optional<MetaTag>, 3>;

  SubmapFrame(std::uint64_t id, const Eigen::Isometry3d &T_odom_base, GaussianCloudPtr cloud, double timestamp,
              std::uint64_t first_frame_id, std::uint64_t last_frame_id, std::size_t frame_count,
              double begin_journey, double end_journey, cpu::VoxelMapsData pyramid_voxels, OdomPoses odom_poses,
              std::vector<VisualFrame> visual_frames)
      : id_(id),
        T_odom_base_(T_odom_base),
        cloud_(std::move(cloud)),
        timestamp_(timestamp),
        bounds_(cloud_ ? AABB(*cloud_) : AABB{}),
        first_frame_id_(first_frame_id),
        last_frame_id_(last_frame_id),
        frame_count_(frame_count),
        begin_journey_(begin_journey),
        end_journey_(end_journey),
        pyramid_voxels_(std::move(pyramid_voxels)),
        odom_poses_(std::move(odom_poses)),
        visual_frames_(std::move(visual_frames)) {
    if (!cloud_) {
      throw std::invalid_argument("Submap frame requires a point cloud");
    }
    if (!bounds_.valid()) {
      throw std::invalid_argument("Submap frame requires a non-empty finite point cloud");
    }
    if (frame_count_ == 0) {
      throw std::invalid_argument("Submap frame requires at least one marginal frame");
    }
    if (odom_poses_.empty() || odom_poses_.size() != frame_count_) {
      throw std::invalid_argument("Submap odometry poses must match the marginal frame count");
    }
    for (std::size_t index = 0; index < odom_poses_.size(); ++index) {
      const OdomPose &pose = odom_poses_[index];
      if (!std::isfinite(pose.timestamp) || !pose.T_odom_base.matrix().allFinite() ||
          (index != 0 && pose.timestamp < odom_poses_[index - 1].timestamp)) {
        throw std::invalid_argument("Submap odometry poses must be finite and timestamp ordered");
      }
    }
    if (!std::isfinite(begin_journey_) || !std::isfinite(end_journey_) || end_journey_ < begin_journey_) {
      throw std::invalid_argument("Invalid submap journey range");
    }
    if (!pyramid_voxels_.valid()) {
      throw std::invalid_argument("Submap frame requires pyramid voxels");
    }
    tags_[static_cast<std::size_t>(MetaTagType::kPose)].emplace(T_odom_base_.cast<float>());
  }

  SubmapFrame(const SubmapFrame &) = delete;
  SubmapFrame &operator=(const SubmapFrame &) = delete;
  SubmapFrame(SubmapFrame &&) noexcept = default;
  SubmapFrame &operator=(SubmapFrame &&) noexcept = default;

  // Transfer a backend-built submap into the map's global key domain. Only
  // identity changes; original odometry and all local evidence remain intact.
  SubmapFrame withPersistentId(std::uint64_t id) && {
    id_ = id;
    return std::move(*this);
  }

  std::uint64_t id() const noexcept { return id_; }
  const Eigen::Isometry3d &T_odom_base() const noexcept { return T_odom_base_; }
  const GaussianCloudPtr &cloud() const noexcept { return cloud_; }
  double timestamp() const noexcept { return timestamp_; }
  const AABB &bounds() const noexcept { return bounds_; }
  std::uint64_t first_frame_id() const noexcept { return first_frame_id_; }
  std::uint64_t last_frame_id() const noexcept { return last_frame_id_; }
  std::size_t frame_count() const noexcept { return frame_count_; }
  double begin_journey() const noexcept { return begin_journey_; }
  double end_journey() const noexcept { return end_journey_; }
  double travel_distance() const noexcept { return end_journey_ - begin_journey_; }
  const cpu::VoxelMapsData &pyramid_voxels() const noexcept { return pyramid_voxels_; }
  const OdomPoses &odom_poses() const noexcept { return odom_poses_; }
  std::vector<VisualFrame> &visual_frames() noexcept { return visual_frames_; }
  const std::vector<VisualFrame> &visual_frames() const noexcept { return visual_frames_; }
  const MetaTags &tags() const noexcept { return tags_; }
  const MetaTag *tag(MetaTagType type) const noexcept {
    const std::size_t index = static_cast<std::size_t>(type);
    return index < tags_.size() && tags_[index] ? &tags_[index].value() : nullptr;
  }

  void attach_tag(MetaTag tag) { tags_[static_cast<std::size_t>(tag.type())].emplace(std::move(tag)); }

 private:
  std::uint64_t id_;
  // Original submap anchor and Gaussian geometry; optimized map poses live in Topograph.
  Eigen::Isometry3d T_odom_base_;
  GaussianCloudPtr cloud_;
  double timestamp_;
  AABB bounds_;
  std::uint64_t first_frame_id_;
  std::uint64_t last_frame_id_;
  std::size_t frame_count_;
  double begin_journey_;
  double end_journey_;
  cpu::VoxelMapsData pyramid_voxels_;
  OdomPoses odom_poses_;
  std::vector<VisualFrame> visual_frames_;
  MetaTags tags_;
};

}  // namespace sapphire
