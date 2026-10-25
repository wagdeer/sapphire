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

struct alignas(16) GaussianPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();
  VOXEL_LOC voxel_key;
  float radius = 0.0F;
  int N = 0;
  bool is_plane = false;

  bool valid() const { return N > 0 && mean.allFinite() && covariance.allFinite() && std::isfinite(radius) && radius > 0.0F; }

  GaussianPoint transformed(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) const {
    GaussianPoint output = *this;
    const Eigen::Vector3d mean_d = mean.cast<double>();
    const Eigen::Matrix3d covariance_d = covariance.cast<double>();
    output.mean = (rotation * mean_d + translation).cast<float>();
    output.covariance = (rotation * covariance_d * rotation.transpose()).cast<float>();
    return output;
  }

  GaussianPoint &operator+=(const GaussianPoint &source) {
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

  void regularize(Eigen::Vector3d eigenvalues, const Eigen::Matrix3d &eigenvectors) {
    if (!eigenvalues.allFinite() || !eigenvectors.allFinite()) {
      covariance = (1e-6 * Eigen::Matrix3d::Identity()).cast<float>();
      radius = static_cast<float>(3.0 * std::sqrt(1e-6));
      return;
    }
    const double largest = std::max(1e-6, eigenvalues.maxCoeff());
    eigenvalues = eigenvalues.cwiseMax(std::max(1e-6, largest / 1e3));
    covariance = (eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose()).cast<float>();
    radius = static_cast<float>(3.0 * std::sqrt(eigenvalues.maxCoeff()));
  }

  void regularize() {
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
};
using GaussianCloud = std::vector<GaussianPoint, Eigen::aligned_allocator<GaussianPoint>>;
using GaussianCloudPtr = std::shared_ptr<const GaussianCloud>;

inline void validateGaussianCloud(const GaussianCloud& cloud) {
  if (cloud.empty()) throw std::invalid_argument("Gaussian cloud must not be empty");
  for (const auto& p : cloud)
    if (!p.valid() || p.voxel_key.level < 0 || p.voxel_key.level > 30 || !p.covariance.isApprox(p.covariance.transpose(), 1e-5f) ||
        p.covariance.llt().info() != Eigen::Success)
      throw std::invalid_argument("LiDAR node requires finite positive-definite Gaussian geometry");
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
