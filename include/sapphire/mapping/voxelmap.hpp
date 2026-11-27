#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <memory_resource>
#include <optional>
#include <sapphire/types.hpp>
#include <sapphire/utils.hpp>
#include <vector>

#include "tsl/robin_map.h"

namespace sapphire {
constexpr float leaf = 1.0f;

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};
struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& v) const { return uint64_t(v.x) * 73856093u ^ uint64_t(v.y) * 19349663u ^ uint64_t(v.z) * 83492791u; }
};
VoxelKey key(const Eigen::Vector3f& p);

struct VoxelPlane {
  Eigen::Vector3f center, normal;
  float radius, sigma;
};

struct GaussianSupport {
  std::uint32_t points = 0;
  std::uint32_t frames = 0;
  std::array<float, 6> covariance{};
};

struct FittedVoxel {
  VoxelPlane plane;
  GaussianSupport gaussian;
};

struct Moments {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
  int count = 0, frames = 0;
  std::size_t last_frame = static_cast<std::size_t>(-1);
  void add(const Eigen::Vector3f& p, std::size_t id = 0);
  std::optional<FittedVoxel> fit() const;
};

struct GaussianVoxelMap {
  GaussianVoxelMap() = default;
  GaussianVoxelMap(const GaussianVoxelMap&) = delete;
  GaussianVoxelMap& operator=(const GaussianVoxelMap&) = delete;

  void reserve(std::size_t count);
  bool insert(const VoxelKey& cell, const FittedVoxel& voxel);
  std::size_t residentBytes() const;
  std::uint64_t supportedPoints() const;
  const VoxelKey& cell(std::size_t i) const { return geometry_[i].key; }
  FittedVoxel voxel(std::size_t i) const { return {planes_[i], geometry_[i].gaussian}; }
  std::size_t size() const { return planes_.size(); }
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  double stamp = 0;
  std::size_t version = 0;
  const VoxelPlane* lookup(const Eigen::Vector3f& p) const;
  const VoxelPlane* coarseLookup(const Eigen::Vector3f& p) const;

  void write(std::ostream& output) const;
  static std::unique_ptr<GaussianVoxelMap> read(std::istream& input);

 private:
  struct Geometry {
    VoxelKey key;
    GaussianSupport gaussian;
  };

  class IndexMemory : public std::pmr::memory_resource {
   public:
    std::size_t bytes() const { return bytes_; }

   private:
    void* do_allocate(std::size_t bytes, std::size_t alignment) override;
    void do_deallocate(void* p, std::size_t bytes, std::size_t alignment) override;
    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override { return this == &other; }
    std::size_t bytes_ = 0;
  };

  using IndexValue = std::pair<VoxelKey, std::uint32_t>;
  using Allocator = std::pmr::polymorphic_allocator<IndexValue>;
  IndexMemory index_memory_;
  tsl::robin_map<VoxelKey, std::uint32_t, VoxelKeyHash, std::equal_to<VoxelKey>, Allocator> index_{Allocator(&index_memory_)};
  std::vector<VoxelPlane> planes_;
  std::vector<Geometry> geometry_;
};

bool supported(const VoxelPlane& plane, const Eigen::Vector3f& point);

struct MappingFrame {
  double stamp;
  Isometry3d odom;
  sapphire::PointCloudConstPtr cloud;
};

struct BaStats {
  std::size_t planes = 0;
  int updates = 0;
  double cost_first = 0.;
  double cost_last = 0.;
};
struct BoundarySummary {
  Mat_66<double> covariance = Mat_66<double>::Zero();
  double position_sigma = 0.;
  double rotation_sigma = 0.;
  double interior_shift_m = 0.;
  bool valid = false;
};

struct InertialEstimate {
  Eigen::Vector3d velocity, gyro_bias, acc_bias, gravity;
  double scale = 1.;
};
struct InertialFrame {
  MappingFrame frame;
  InertialEstimate nav;
  std::vector<ImuData> imu;
};

Mat_66<double> relativeOdometryCovariance(const std::vector<Isometry3d>& poses, std::size_t first, std::size_t end);

BaStats adjustInertial(const std::vector<InertialFrame>& frames, const std::vector<MappingFrame>& fixed, std::vector<Isometry3d>& poses,
                       std::vector<InertialEstimate>& nav, const Config::Submap::Ba& options = {}, BoundarySummary* boundary = nullptr);

}  // namespace sapphire
