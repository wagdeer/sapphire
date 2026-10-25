#pragma once
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include "common/lidar_frame.hpp"
#include "maplayer/topograph/node.hpp"

namespace sapphire {
// Build node-local Gaussian geometry from frozen odometry frames.
// Graph IDs and links are assigned by Memory on admission.
class NodeBuilder final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct OdomPose {
    double timestamp;
    Eigen::Isometry3d T_odom_base;
  };
  using OdomPoses = std::vector<OdomPose, Eigen::aligned_allocator<OdomPose>>;

  // The detached Node is retained intact while resume preparation is retried.
  // Remaining samples belong to trajectory storage, not the Node database row.
  struct Result {
    std::unique_ptr<Node> node;
    OdomPoses trajectory;
  };

  explicit NodeBuilder(double target_distance = 15.0, double max_point_range = 20.0)
      : target_distance_(target_distance), max_point_range_(max_point_range), max_point_range_squared_(max_point_range * max_point_range) {
    if (!std::isfinite(target_distance_) || target_distance_ <= 0.0) {
      throw std::invalid_argument("Submap target distance must be positive");
    }
    if (!std::isfinite(max_point_range_) || max_point_range_ <= 0.0) {
      throw std::invalid_argument("Submap maximum point range must be positive");
    }
  }

  void clear() {
    clear_pending();
  }

  [[nodiscard]] std::optional<Result> push(LidarFrame &&marginal_frame) {
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

    std::optional<Result> ready;
    if (odom_poses_.size() >= max_history_frames) ready = flush();

    if (!has_reference_) {
      reference_rotation_ = marginal_frame.x.R;
      reference_position_ = marginal_frame.x.p;
      begin_journey_ = journey;
      has_reference_ = true;
    }

    append_odom_pose(marginal_frame);
    integrate(marginal_frame);
    end_journey_ = journey;

    if (travel_distance() < target_distance_) {
      return ready;
    }
    return build_node();
  }

  /// Emits the final, shorter-than-target submap at end of input.
  [[nodiscard]] std::optional<Result> flush() {
    if (!has_reference_) {
      return std::nullopt;
    }
    return build_node();
  }

  bool empty() const noexcept { return !has_reference_; }
  std::size_t buffered_frame_count() const noexcept { return odom_poses_.size(); }
  std::size_t buffered_point_count() const noexcept { return voxel_gaussians_.size(); }
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

  [[nodiscard]] std::optional<Result> build_node() {
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

    const auto& anchor = odom_poses_.front();
    auto node = std::make_unique<Node>(std::move(cloud), anchor.timestamp,
                                      SE3f(Eigen::Matrix4f(anchor.T_odom_base.matrix().cast<float>())));
    odom_poses_.erase(odom_poses_.begin());
    Result result{std::move(node), std::move(odom_poses_)};
    clear_pending();
    return result;
  }

  void clear_pending() {
    voxel_gaussians_.clear();
    odom_poses_.clear();
    has_reference_ = false;
    reference_rotation_.setIdentity();
    reference_position_.setZero();
    begin_journey_ = 0.0;
    end_journey_ = 0.0;
  }

  // Bound retained trajectory history even when the platform is stationary.
  static constexpr std::size_t max_history_frames = 256;
  double target_distance_;
  double max_point_range_;
  double max_point_range_squared_;
  bool has_reference_ = false;
  Eigen::Matrix3d reference_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d reference_position_ = Eigen::Vector3d::Zero();
  double begin_journey_ = 0.0;
  double end_journey_ = 0.0;
  OdomPoses odom_poses_;
  std::unordered_map<VOXEL_LOC, GaussianPoint> voxel_gaussians_;
};

}  // namespace sapphire
