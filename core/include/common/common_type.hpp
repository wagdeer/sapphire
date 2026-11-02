#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <type_traits>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace sapphire {

using ParametersMap = std::map<std::string, std::string, std::less<>>;

template <int R, int C, typename Scalar, int Options = Eigen::Matrix<Scalar, R, C>::Options>
using MatRC_t = Eigen::Matrix<Scalar, R, C, Options>;

template <int R, typename Scalar>
using VecR_t = Eigen::Matrix<Scalar, R, 1>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 6, 6>::Options>
using Mat66_t = MatRC_t<6, 6, Scalar, Options>;

template <typename Scalar>
using Vec3_t = VecR_t<3, Scalar>;


template <typename T, int N>
using vmat = std::vector<Eigen::Matrix<T, N, N>, Eigen::aligned_allocator<Eigen::Matrix<T, N, N>>>;

template <typename T, int N>
using vvec = std::vector<Eigen::Matrix<T, N, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, N, 1>>>;

class VOXEL_LOC {
 public:
  int64_t x, y, z;
  int level;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int voxel_level = 0) : x(vx), y(vy), z(vz), level(voxel_level) {}

  bool operator==(const VOXEL_LOC &other) const { return x == other.x && y == other.y && z == other.z && level == other.level; }
};

// Legacy SAPLID02 observation support, preserved when loading old archives.
// Newly frozen primitives do not allocate this metadata.
struct alignas(16) GaussianSupport {
  Eigen::Matrix3d covariance_lower;
  Eigen::Matrix3d plane_upper;
  Eigen::Matrix3d box_axes;
  Eigen::Vector3d box_center;
  Eigen::Vector3d box_half;
};

struct alignas(16) GaussianPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  // Spatial scatter, currently conditioned by NodeBuilder::regularize before
  // storage. Not mean or plane-estimation uncertainty;
  // raw observation scatter and the GICP metric are not yet stored separately.
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();
  VOXEL_LOC voxel_key;
  float radius = 0.0F;
  int N = 0;
  bool is_plane = false;
  std::shared_ptr<const GaussianSupport> support;

  bool valid() const { return N > 0 && mean.allFinite() && covariance.allFinite() && std::isfinite(radius) && radius > 0.0F; }

  GaussianPoint transformed(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) const {
    GaussianPoint output = *this;
    const Eigen::Vector3d mean_d = mean.cast<double>();
    const Eigen::Matrix3d covariance_d = covariance.cast<double>();
    output.mean = (rotation * mean_d + translation).cast<float>();
    output.covariance = (rotation * covariance_d * rotation.transpose()).cast<float>();
    if (support) {
      auto history = std::make_shared<GaussianSupport>(*support);
      history->covariance_lower = (rotation * support->covariance_lower * rotation.transpose()).eval();
      history->plane_upper = (rotation * support->plane_upper * rotation.transpose()).eval();
      history->box_axes = rotation * support->box_axes;
      history->box_center = rotation * support->box_center + translation;
      output.support = std::move(history);
    }
    return output;
  }

  GaussianPoint &operator+=(const GaussianPoint &source) {
    if (support || source.support)
      throw std::logic_error("Cannot merge immutable legacy geometry with observation support");
    if (!(voxel_key == source.voxel_key)) {
      throw std::invalid_argument("Cannot merge Gaussian points with different voxel keys");
    }
    if (N < 0 || source.N < 0) {
      throw std::invalid_argument("Cannot merge Gaussian points with negative counts");
    }
    if (!mean.allFinite() || !covariance.allFinite() || !source.mean.allFinite() || !source.covariance.allFinite()) {
      throw std::invalid_argument("Cannot merge Gaussian points with non-finite moments");
    }
    if (source.N == 0) {
      return *this;
    }
    if (N > std::numeric_limits<int>::max() - source.N) {
      throw std::overflow_error("Gaussian point count overflow");
    }
    if (N <= 0) {
      *this = source;
      return *this;
    }

    is_plane = is_plane && source.is_plane;
    const double target_count = static_cast<double>(N);
    const double source_count = static_cast<double>(source.N);
    const double total_count = target_count + source_count;
    const Eigen::Vector3d target_mean = mean.cast<double>();
    const Eigen::Vector3d delta = source.mean.cast<double>() - target_mean;
    const Eigen::Matrix3d merged_covariance =
        (target_count * covariance.cast<double>() + source_count * source.covariance.cast<double>() +
         (target_count * source_count / total_count) * delta * delta.transpose()) / total_count;
    mean = (target_mean + (source_count / total_count) * delta).cast<float>();
    covariance = merged_covariance.cast<float>();
    N += source.N;
    radius = 0.0F;
    return *this;
  }

  // A derived registration metric, not observation scatter or mean uncertainty.
  // In particular, evaluating it must not thicken a raw planar Gaussian stored
  // for later moment accumulation. Uses the existing regularize() conditioning.
  Eigen::Matrix3f registration_covariance() const {
    const Eigen::Matrix3d covariance_d = covariance.cast<double>();
    const Eigen::Matrix3d symmetric = 0.5 * (covariance_d + covariance_d.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() || !solver.eigenvectors().allFinite()) {
      return (1e-6 * Eigen::Matrix3d::Identity()).cast<float>();
    }
    const Eigen::Vector3d eigenvalues = conditioned_eigenvalues(solver.eigenvalues());
    return (solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose()).cast<float>();
  }

  void regularize(Eigen::Vector3d eigenvalues, const Eigen::Matrix3d &eigenvectors) {
    if (support) throw std::logic_error("Fused geometry must retain its observed covariance");
    if (!eigenvalues.allFinite() || !eigenvectors.allFinite()) {
      covariance = (1e-6 * Eigen::Matrix3d::Identity()).cast<float>();
      radius = static_cast<float>(3.0 * std::sqrt(1e-6));
      return;
    }
    eigenvalues = conditioned_eigenvalues(eigenvalues);
    covariance = (eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose()).cast<float>();
    radius = static_cast<float>(3.0 * std::sqrt(eigenvalues.maxCoeff()));
  }

  void regularize() {
    if (support) throw std::logic_error("Fused geometry must retain its observed covariance");
    const Eigen::Matrix3d covariance_d = covariance.cast<double>();
    const Eigen::Matrix3d symmetric = 0.5 * (covariance_d + covariance_d.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success) {
      covariance = (1e-6 * Eigen::Matrix3d::Identity()).cast<float>();
      radius = static_cast<float>(3.0 * std::sqrt(1e-6));
      return;
    }
    regularize(solver.eigenvalues(), solver.eigenvectors());
  }

 private:
  static Eigen::Vector3d conditioned_eigenvalues(const Eigen::Vector3d &eigenvalues) {
    const double largest = std::max(1e-6, eigenvalues.maxCoeff());
    return eigenvalues.cwiseMax(std::max(1e-6, largest / 1e3));
  }
};
using GaussianCloud = std::vector<GaussianPoint, Eigen::aligned_allocator<GaussianPoint>>;
using GaussianCloudPtr = std::shared_ptr<const GaussianCloud>;

inline void validateGaussianCloud(const GaussianCloud& cloud) {
  if (cloud.empty()) throw std::invalid_argument("Gaussian cloud must not be empty");
  for (const auto& p : cloud)
    if (!p.valid() || p.voxel_key.level < 0 || p.voxel_key.level > 30 || !p.covariance.isApprox(p.covariance.transpose(), 1e-5f) ||
        p.covariance.llt().info() != Eigen::Success)
      throw std::invalid_argument("LiDAR node requires finite positive-definite Gaussian geometry");
  for (const auto& p : cloud) if (p.support) {
    const auto& s = *p.support;
    if (!s.covariance_lower.allFinite() || !s.plane_upper.allFinite() || !s.box_axes.allFinite() ||
        !s.box_center.allFinite() || !s.box_half.allFinite() || (s.box_half.array() < 0).any() ||
        !s.covariance_lower.isApprox(s.covariance_lower.transpose(), 1e-10) ||
        !s.plane_upper.isApprox(s.plane_upper.transpose(), 1e-10) ||
        !(s.box_axes.transpose()*s.box_axes).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
        s.covariance_lower.llt().info() != Eigen::Success || s.plane_upper.llt().info() != Eigen::Success)
      throw std::invalid_argument("Invalid Gaussian history geometry");
  }
}

struct GaussianCloudSize {
  std::size_t points = 0, supports = 0;
  std::size_t bytes() const {
    if (supports > points || points > std::numeric_limits<std::size_t>::max() / sizeof(GaussianPoint) ||
        supports > (std::numeric_limits<std::size_t>::max() - points * sizeof(GaussianPoint)) / sizeof(GaussianSupport))
      throw std::length_error("Gaussian payload size overflow");
    return points * sizeof(GaussianPoint) + supports * sizeof(GaussianSupport);
  }
};

inline GaussianCloudSize gaussian_cloud_size(const GaussianCloud& cloud) {
  return {cloud.size(), std::size_t(std::count_if(cloud.begin(), cloud.end(), [](const auto& p) { return bool(p.support); }))};
}


inline GaussianPoint operator+(GaussianPoint lhs, const GaussianPoint &rhs) {
  lhs += rhs;
  return lhs;
}

}  // namespace sapphire

namespace std {
template <>
struct hash<sapphire::VOXEL_LOC> {
  size_t operator()(const sapphire::VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    size_t seed = hash<int64_t>()(s.x);
    seed ^= hash<int64_t>()(s.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash<int64_t>()(s.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash<int>()(s.level) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace std
