#pragma once

#include <deque>
#include <optional>
#include <sapphire/odometry/ieskf.hpp>
#include <sapphire/odometry/localmap.hpp>
#include <sapphire/odometry/voxel_filter.hpp>

#include "tsl/robin_hood.h"

namespace sapphire {

class RevisitExchange;
class LocalPlaneExchange;
struct RevisitTarget;
struct GaussianVoxelMap;

struct MeasureGroup {
  LidarData lidar;
  std::deque<FilterImuSample> imu;
  FilterImuSample end_imu;
};

class Registration2 {
 public:
  Registration2(const Config& config, RevisitExchange& exchange, LocalPlaneExchange& local_planes);
  bool process(MeasureGroup scan);
  void rememberKeyframe(double stamp, const Isometry3d& pose);
  const Isometry3d& globalFromOdom() const { return global_from_odom_; }
  std::size_t revisitVersion() const { return version_; }
  bool initialized() const { return state_ != &Registration2::initializeFilter; }
  const iEskf& filter() const { return filter_; }
  PointCloudConstPtr deskewed() const { return deskewed_; }
  std::size_t matches() const { return matches_; }
  double accelerationStd() const { return accel_scale_ * std::sqrt(accel_variation_ / imu_count_); }
  float accelerationScale() const { return accel_scale_; }

 private:
  friend struct Registration2TestAccess;

  struct VerificationScan {
    double stamp;
    Isometry3d prediction;
    VV3 points;
  };

  struct Keyframe {
    double stamp;
    Isometry3d odom_pose;
    std::optional<VerificationScan> next_scan;
  };

  const GaussianVoxelMap* updateRevisit(double stamp, const Isometry3d& odom, const VV3& scan);
  void verifyRevisit(const RevisitTarget& target, double stamp, const Keyframe& anchor);

  RevisitExchange& exchange_;
  LocalPlaneExchange& local_planes_;
  std::deque<Keyframe> keyframes_;
  const RevisitTarget* active_ = nullptr;
  std::size_t attempted_ = 0, version_ = 0;
  Eigen::Isometry3d global_from_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3f target_from_odom_ = Eigen::Isometry3f::Identity();

  class VoxelSampler {
   public:
    explicit VoxelSampler(float leaf_size);
    void filter(const PointCloud& input, PointCloud& output);

   private:
    float voxel_size_;
    float inv_voxel_size_;
    robin_hood::unordered_flat_map<std::size_t, std::size_t> voxel_map_;
    std::vector<Point, Eigen::aligned_allocator<Point>> points_;
    std::vector<float> dist2_;
    const Eigen::Vector3i offset_ = Eigen::Vector3i(1000, 1000, 1000);
  };

  bool initializeFilter();
  bool initializeMap();
  bool track();
  void deskew();
  void observe();
  void insertScan();

  using KNNHeapType = KNNHeap<5, Eigen::Vector3f>;
  using State = bool (Registration2::*)();
  State state_ = &Registration2::initializeFilter;
  iEskf filter_;
  LocalMap<V3, scalar> map_;
  VoxelSampler sampler_;
  FilterPose lidar_to_imu_;
  MeasureGroup measures_;
  double gravity_;
  int imu_count_ = 0;
  double accel_variation_ = 0.;
  V3 mean_gyro_ = V3::Zero();
  V3 mean_accel_ = V3::Zero();
  float accel_scale_ = 1.f;
  FilterPose initial_pose_;
  int initial_scans_ = 0;
  PointCloudPtr deskewed_ = std::make_shared<PointCloud>();
  PointCloudPtr sampled_ = std::make_shared<PointCloud>();
  std::vector<DynamicState> imu_states_;
  VV3 world_points_, scan_points_;
  std::vector<std::array<double, 4>> planes_;
  std::vector<int> match_indices_;
  std::vector<float> point_ranges_;
  std::vector<std::uint8_t> plane_mask_, neighbor_mask_;
  std::size_t matches_ = 0;
  std::size_t search_count_ = 0;
};

}  // namespace sapphire
