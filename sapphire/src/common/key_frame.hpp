#pragma once

#include <Eigen/Geometry>
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

#include "common/aabb.hpp"
#include "common/lio_frame.hpp"
#include "common/trajectory_util.hpp"
#include "common/visual_frame.hpp"

namespace sapphire {

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

  SubmapFrame(std::uint64_t id, LioFrame lio, std::uint64_t first_frame_id, std::uint64_t last_frame_id, std::size_t frame_count,
              double begin_journey, double end_journey, cpu::VoxelMapsData pyramid_voxels, OdomPoses odom_poses, NavigationPath navigation,
              std::vector<VisualFrame> visual_frames)
      : id_(id),
        lio_(std::move(lio)),
        bounds_(lio_.pcd ? AABB(*lio_.pcd) : AABB{}),
        first_frame_id_(first_frame_id),
        last_frame_id_(last_frame_id),
        frame_count_(frame_count),
        begin_journey_(begin_journey),
        end_journey_(end_journey),
        pyramid_voxels_(std::move(pyramid_voxels)),
        odom_poses_(std::move(odom_poses)),
        navigation_(std::move(navigation)),
        visual_frames_(std::move(visual_frames)) {
    if (!lio_.pcd) {
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
    tags_[static_cast<std::size_t>(MetaTagType::kPose)].emplace(lio_.T_odom_base.cast<float>());
  }

  SubmapFrame(const SubmapFrame &) = delete;
  SubmapFrame &operator=(const SubmapFrame &) = delete;
  SubmapFrame(SubmapFrame &&) noexcept = default;
  SubmapFrame &operator=(SubmapFrame &&) noexcept = default;

  // Transfer a frozen frontend submap into the map's global key domain. Only
  // identity changes; original odometry and all local evidence remain intact.
  SubmapFrame withPersistentId(std::uint64_t id) && {
    id_ = id;
    return std::move(*this);
  }

  std::uint64_t id() const noexcept { return id_; }
  const LioFrame &lio() const noexcept { return lio_; }
  const AABB &bounds() const noexcept { return bounds_; }
  std::uint64_t first_frame_id() const noexcept { return first_frame_id_; }
  std::uint64_t last_frame_id() const noexcept { return last_frame_id_; }
  std::size_t frame_count() const noexcept { return frame_count_; }
  double begin_journey() const noexcept { return begin_journey_; }
  double end_journey() const noexcept { return end_journey_; }
  double travel_distance() const noexcept { return end_journey_ - begin_journey_; }
  const cpu::VoxelMapsData &pyramid_voxels() const noexcept { return pyramid_voxels_; }
  const OdomPoses &odom_poses() const noexcept { return odom_poses_; }
  const NavigationPath &navigation() const noexcept { return navigation_; }
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
  LioFrame lio_;
  AABB bounds_;
  std::uint64_t first_frame_id_;
  std::uint64_t last_frame_id_;
  std::size_t frame_count_;
  double begin_journey_;
  double end_journey_;
  cpu::VoxelMapsData pyramid_voxels_;
  OdomPoses odom_poses_;
  NavigationPath navigation_;
  std::vector<VisualFrame> visual_frames_;
  MetaTags tags_;
};

class SubmapFrameBuffer final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SubmapFrameBuffer(double inverse_voxel_size, double target_distance = 15.0, double max_point_range = 20.0)
      : inverse_voxel_size_(inverse_voxel_size),
        target_distance_(target_distance),
        max_point_range_(max_point_range),
        max_point_range_squared_(max_point_range * max_point_range) {
    if (!std::isfinite(inverse_voxel_size_) || inverse_voxel_size_ <= 0.0) {
      throw std::invalid_argument("Submap inverse voxel size must be positive");
    }
    if (!std::isfinite(target_distance_) || target_distance_ <= 0.0) {
      throw std::invalid_argument("Submap target distance must be positive");
    }
    if (!std::isfinite(max_point_range_) || max_point_range_ <= 0.0) {
      throw std::invalid_argument("Submap maximum point range must be positive");
    }
  }

  void clear() {
    clear_pending();
    next_submap_id_ = 0;
    next_frame_id_ = 0;
  }

  void push_visual(VisualFrame visual_frame, std::size_t max_frames = 8) {
    if (max_frames == 0) return;
    std::vector<std::size_t> matching;
    for (std::size_t i = 0; i < visual_frames_.size(); ++i)
      if (visual_frames_[i].camera_id() == visual_frame.camera_id()) matching.push_back(i);
    if (matching.size() >= max_frames) visual_frames_.erase(visual_frames_.begin() + matching[max_frames > 1 ? 1 : 0]);
    visual_frames_.emplace_back(std::move(visual_frame));
  }

  [[nodiscard]] std::optional<SubmapFrame> push(MargiFrame &&marginal_frame, bool close_by_distance = true) {
    const double journey = marginal_frame.journey;
    if (!std::isfinite(journey)) {
      throw std::invalid_argument("Submap journey must be finite");
    }
    if (!marginal_frame.x.R.allFinite() || !marginal_frame.x.p.allFinite() || !std::isfinite(marginal_frame.timestamp)) {
      throw std::invalid_argument("Cannot add a marginal frame with an invalid pose or timestamp to a submap");
    }
    if (!odom_poses_.empty() && marginal_frame.timestamp < odom_poses_.back().timestamp) {
      throw std::invalid_argument("Submap odometry timestamps must be monotonically non-decreasing");
    }
    if (has_reference_ && journey < end_journey_) {
      throw std::invalid_argument("Submap journey must be monotonically non-decreasing");
    }

    const std::uint64_t frame_id = next_frame_id_++;
    if (!has_reference_) {
      reference_rotation_ = marginal_frame.x.R;
      reference_position_ = marginal_frame.x.p;
      reference_timestamp_ = marginal_frame.timestamp;
      first_frame_id_ = frame_id;
      begin_journey_ = journey;
      has_reference_ = true;
    }

    append_odom_pose(marginal_frame);
    integrate(marginal_frame);
    last_frame_id_ = frame_id;
    ++frame_count_;
    end_journey_ = journey;

    if (!close_by_distance || travel_distance() < target_distance_) {
      return std::nullopt;
    }
    return build_submap();
  }

  /// Emits the final, shorter-than-target submap at end of input.
  [[nodiscard]] std::optional<SubmapFrame> flush() {
    if (!has_reference_) {
      return std::nullopt;
    }
    return build_submap();
  }

  bool empty() const noexcept { return !has_reference_; }
  std::size_t buffered_frame_count() const noexcept { return frame_count_; }
  std::size_t buffered_point_count() const noexcept { return voxel_gaussians_.size(); }
  std::size_t buffered_visual_frame_count() const noexcept { return visual_frames_.size(); }
  double target_distance() const noexcept { return target_distance_; }
  double max_point_range() const noexcept { return max_point_range_; }
  double travel_distance() const noexcept { return has_reference_ ? end_journey_ - begin_journey_ : 0.0; }

 private:
  void append_odom_pose(const MargiFrame &source) {
    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
    T_odom_base.linear() = source.x.R;
    T_odom_base.translation() = source.x.p;
    odom_poses_.push_back({source.timestamp, T_odom_base});
  }

  void integrate(const MargiFrame &source) {
    const Eigen::Matrix3d rotation_base_world = reference_rotation_.transpose();
    const Eigen::Vector3d translation_base_world = -rotation_base_world * reference_position_;
    for (const GaussianPoint &point : source.gaussians) {
      if (!point.mean.allFinite() || !point.covariance.allFinite()) {
        continue;
      }
      const Eigen::Vector3d source_mean = source.x.R.transpose() * (point.mean.cast<double>() - source.x.p);
      if (source_mean.squaredNorm() > max_point_range_squared_) {
        continue;
      }

      GaussianPoint transformed = point.transformed(rotation_base_world, translation_base_world);
      auto [iter, inserted] = voxel_gaussians_.try_emplace(transformed.voxel_key, transformed);
      if (!inserted) {
        iter->second += transformed;
      }
    }
  }

  [[nodiscard]] std::optional<SubmapFrame> build_submap() {
    auto cloud = std::make_shared<GaussianCloud>();
    cloud->reserve(voxel_gaussians_.size());
    for (auto &entry : voxel_gaussians_) {
      GaussianPoint &point = entry.second;
      if (point.N <= 0) {
        continue;
      }
      point.regularize();
      cloud->emplace_back(std::move(point));
    }
    if (cloud->empty()) {
      clear_pending();
      return std::nullopt;
    }

    cpu::VoxelMaps voxelmaps;
    voxelmaps.set_min_res(static_cast<float>(1.0 / inverse_voxel_size_));
    voxelmaps.create_voxelmaps(cloud->size(), [cloud](std::size_t index) { return (*cloud)[index].mean.cast<float>(); });

    LioFrame lio;
    lio.T_odom_base.setIdentity();
    lio.T_odom_base.linear() = reference_rotation_;
    lio.T_odom_base.translation() = reference_position_;
    lio.pcd = std::move(cloud);
    lio.timestamp = reference_timestamp_;

    SubmapFrame submap(next_submap_id_++, std::move(lio), first_frame_id_, last_frame_id_, frame_count_, begin_journey_, end_journey_,
                       voxelmaps.release_data(), std::move(odom_poses_), NavigationPath{}, std::move(visual_frames_));
    clear_pending();
    return submap;
  }

  void clear_pending() {
    voxel_gaussians_.clear();
    odom_poses_.clear();
    visual_frames_.clear();
    has_reference_ = false;
    reference_rotation_.setIdentity();
    reference_position_.setZero();
    reference_timestamp_ = -1.0;
    first_frame_id_ = 0;
    last_frame_id_ = 0;
    frame_count_ = 0;
    begin_journey_ = 0.0;
    end_journey_ = 0.0;
  }

  double inverse_voxel_size_;
  double target_distance_;
  double max_point_range_;
  double max_point_range_squared_;
  std::uint64_t next_submap_id_ = 0;
  std::uint64_t next_frame_id_ = 0;
  bool has_reference_ = false;
  Eigen::Matrix3d reference_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d reference_position_ = Eigen::Vector3d::Zero();
  double reference_timestamp_ = -1.0;
  std::uint64_t first_frame_id_ = 0;
  std::uint64_t last_frame_id_ = 0;
  std::size_t frame_count_ = 0;
  double begin_journey_ = 0.0;
  double end_journey_ = 0.0;
  OdomPoses odom_poses_;
  std::vector<VisualFrame> visual_frames_;
  std::unordered_map<VOXEL_LOC, GaussianPoint> voxel_gaussians_;
};

}  // namespace sapphire
