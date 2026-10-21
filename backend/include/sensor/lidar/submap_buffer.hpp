#pragma once
#include "backend/sensor/lidar/submap_frame.hpp"
#include "frontend/common/lidar_frame.hpp"

namespace sapphire {
// Builds submaps with LiDAR geometry and visual observations. Geometry comes
// from frozen frontend VoxelMap Gaussians. Images are appearance observations
// assigned by delivery to this submap, without a metric capture-time pose.
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

  [[nodiscard]] std::optional<SubmapFrame> push(LidarFrame &&marginal_frame, bool close_by_distance = true) {
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
  void append_odom_pose(const LidarFrame &source) {
    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
    T_odom_base.linear() = source.x.R;
    T_odom_base.translation() = source.x.p;
    odom_poses_.push_back({source.timestamp, T_odom_base});
  }

  void integrate(const LidarFrame &source) {
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

    Eigen::Isometry3d anchor_pose = Eigen::Isometry3d::Identity();
    anchor_pose.linear() = reference_rotation_;
    anchor_pose.translation() = reference_position_;

    SubmapFrame submap(next_submap_id_++, anchor_pose, std::move(cloud), reference_timestamp_,
                       first_frame_id_, last_frame_id_, frame_count_, begin_journey_, end_journey_,
                       voxelmaps.release_data(), std::move(odom_poses_), std::move(visual_frames_));
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
