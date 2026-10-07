#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <small_gicp/points/traits.hpp>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "thirdparty/nanoflann.hpp"
#include "common/state_group.hpp"

namespace sapphire {

struct LidarPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
  float time_offset = 0.0f;
  float operator[](size_t index) const { return index == 0 ? x : index == 1 ? y : z; }
};

struct TrajectoryPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f, distance = 0.0f, session = 0.0f;
};

struct alignas(16) ImuMeas {
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct alignas(16) MeasGroup {
  double lidar_begin_time = -1.0;
  double lidar_end_time = -1.0;
  std::deque<ImuMeas> imu_buf;
  std::shared_ptr<std::vector<LidarPoint>> lidar_cloud;
  void clear() {
    lidar_begin_time = -1.0;
    lidar_end_time = -1.0;
    imu_buf.clear();
    lidar_cloud.reset();
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename T, int N>
using vmat = std::vector<Eigen::Matrix<T, N, N>, Eigen::aligned_allocator<Eigen::Matrix<T, N, N>>>;

template <typename T, int N>
using vvec = std::vector<Eigen::Matrix<T, N, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, N, 1>>>;

template <typename T>
inline const Eigen::Matrix<T, 3, 3> I33 = Eigen::Matrix<T, 3, 3>::Identity();

struct alignas(16) pointVar {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pnt;
  Eigen::Matrix3d var;

  // Right attitude error and world position error: J = [-R[p]x, I].
  // Project the point/pose covariance without discarding their pose cross terms.
  // Jacobian is caller-owned scratch; no pose or scratch is stored per point.
  double projectVariance(const Eigen::Vector3d &normal, const Eigen::Matrix3d &rotation,
                         const Eigen::Matrix<double, POSE_DOF, POSE_DOF> &covariance,
                         Eigen::Matrix<double, POSE_DOF, 1> &jacobian) const {
    const Eigen::Vector3d body_normal = rotation.transpose() * normal;
    jacobian.head<3>() = pnt.cross(body_normal);
    jacobian.tail<3>() = normal;
    return body_normal.dot(var * body_normal) + jacobian.dot(covariance * jacobian);
  }
};

using PointCloud = std::vector<pointVar>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

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

inline GaussianPoint operator+(GaussianPoint lhs, const GaussianPoint &rhs) {
  lhs += rhs;
  return lhs;
}

inline void calcMeasVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var) {
  if (pb[2] == 0) {
    pb[2] = 0.0001;
  }
  const float range = pb.norm();
  const float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  const double angle_sine = sin(static_cast<double>(degree_inc) * M_PI / 180.0);
  direction_var << angle_sine * angle_sine, 0, 0, angle_sine * angle_sine;
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat = lie::SO3d::wedge(direction);
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  const Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

inline void var_init(const StateGroup &extrinsic, const std::vector<LidarPoint> &cloud, const PointCloudPtr &points, double range_error,
                     double angle_error) {
  points->clear();
  points->resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    const LidarPoint &point = cloud[i];
    pointVar &output = points->at(i);
    output.pnt << point.x, point.y, point.z;
    calcMeasVar(output.pnt, range_error, angle_error, output.var);
    output.pnt = extrinsic.R * output.pnt + extrinsic.p;
    output.var = extrinsic.R * output.var * extrinsic.R.transpose();
  }
}

inline void pvec_update(const PointCloudPtr &points, const StateGroup &state, vvec<double, 3> &world_points) {
  const Eigen::Matrix3d rot_var = state.cov.block<3, 3>(0, 0);
  const Eigen::Matrix3d tsl_var = state.cov.block<3, 3>(3, 3);
  for (pointVar &point : *points) {
    const Eigen::Matrix3d point_hat = lie::SO3d::wedge(point.pnt);
    point.var = state.R * point.var * state.R.transpose() + point_hat * rot_var * point_hat.transpose() + tsl_var;
    world_points.push_back(state.R * point.pnt + state.p);
  }
}

struct Plane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> plane_var;
  float radius = 0;
  bool is_plane = false;

  Plane() { plane_var.setZero(); }
};

class PointCluster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d &vec) {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Vector3d mean() const {
    if (N <= 0) {
      return Eigen::Vector3d::Zero();
    }
    return v / static_cast<double>(N);
  }

  Eigen::Matrix3d cov() const {
    if (N <= 0) {
      return Eigen::Matrix3d::Zero();
    }
    const Eigen::Vector3d center = mean();
    return P / static_cast<double>(N) - center * center.transpose();
  }

  GaussianPoint to_gaussianpoint(const VOXEL_LOC &voxel_key = VOXEL_LOC()) const {
    GaussianPoint output;
    output.mean = mean().cast<float>();
    output.covariance = cov().cast<float>();
    output.voxel_key = voxel_key;
    output.N = N;
    return output;
  }

  GaussianPoint to_gaussianpoint(const Eigen::Vector3d &eigenvalues, const Eigen::Matrix3d &eigenvectors) const {
    GaussianPoint output;
    output.mean = mean().cast<float>();
    output.N = N;
    output.regularize(eigenvalues, eigenvectors);
    return output;
  }

  PointCluster &operator+=(const PointCluster &sigv) {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  PointCluster &operator-=(const PointCluster &sigv) {
    this->P -= sigv.P;
    this->v -= sigv.v;
    this->N -= sigv.N;

    return *this;
  }

  void transform(const PointCluster &sigv, const StateGroup &stat) {
    N = sigv.N;
    v = stat.R * sigv.v + N * stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R * sigv.P * stat.R.transpose() + rp + rp.transpose() + N * stat.p * stat.p.transpose();
  }
};

class EigenVectorAdapter {
 public:
  const vvec<double, 3> &points;

  EigenVectorAdapter(const vvec<double, 3> &points_in) : points(points_in) {}

  inline size_t kdtree_get_point_count() const { return points.size(); }

  inline double kdtree_get_pt(size_t index, size_t dim) const { return points[index][dim]; }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX &) const {
    return false;
  }
};
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, EigenVectorAdapter>, EigenVectorAdapter, 3, size_t>;

}  // namespace sapphire

namespace small_gicp::traits {

template <>
struct Traits<sapphire::GaussianCloud> {
  using Points = sapphire::GaussianCloud;

  static std::size_t size(const Points &points) { return points.size(); }

  static bool has_points(const Points &points) { return !points.empty(); }

  static bool has_covs(const Points &points) { return !points.empty(); }

  static Eigen::Vector4d point(const Points &points, std::size_t index) {
    const Eigen::Vector3d mean = points[index].mean.cast<double>();
    return Eigen::Vector4d(mean.x(), mean.y(), mean.z(), 1.0);
  }

  static Eigen::Matrix4d cov(const Points &points, std::size_t index) {
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
    covariance.topLeftCorner<3, 3>() = points[index].covariance.cast<double>();
    return covariance;
  }
};

}  // namespace small_gicp::traits

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

namespace sapphire {

// degrees to radians. degress in [-180, 180], radians in [-pi, pi]
template <typename T>
inline constexpr T toRAD(T degrees) {
  return static_cast<T>(degrees * M_PI / 180.0);
}

// radians to degrees. deg in [-180, 180], radians in [-pi, pi]
template <typename T>
inline constexpr T toDEG(T radians) {
  return static_cast<T>(radians * 180.0 / M_PI);
}

inline void down_sampling_voxel(std::vector<LidarPoint> &pl_feat, double inv_voxel_size) {
  using VoxelSum = Eigen::Matrix<double, 6, 1>;
  std::unordered_map<VOXEL_LOC, VoxelSum> feat_map;
  feat_map.reserve(pl_feat.size());
  float loc_xyz[3];
  for (const LidarPoint &p_c : pl_feat) {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] * inv_voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      VoxelSum sum;
      sum << p_c.x, p_c.y, p_c.z, p_c.intensity, p_c.time_offset, 1.0;
      feat_map.emplace(position, sum);
    } else {
      iter->second[0] += p_c.x;
      iter->second[1] += p_c.y;
      iter->second[2] += p_c.z;
      iter->second[3] += p_c.intensity;
      iter->second[4] += p_c.time_offset;
      iter->second[5] += 1.0;
    }
  }

  pl_feat.clear();
  pl_feat.reserve(feat_map.size());
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    LidarPoint point;
    const double count = iter->second[5];
    point.x = iter->second[0] / count;
    point.y = iter->second[1] / count;
    point.z = iter->second[2] / count;
    point.intensity = iter->second[3] / count;
    point.time_offset = static_cast<float>(iter->second[4] / count);
    pl_feat.push_back(point);
  }
}

inline void down_sampling_voxel(vvec<double, 3> &pl_feat, double inv_voxel_size) {
  std::unordered_map<VOXEL_LOC, Eigen::Vector4d> feat_map;
  double loc_xyz[3];

  for (Eigen::Vector3d &p_c : pl_feat) {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] * inv_voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      Eigen::Vector4d sum;
      sum << p_c, 1.0;
      feat_map[position] = sum;
    } else {
      iter->second.head<3>() += p_c;
      iter->second[3] += 1.0;
    }
  }

  pl_feat.clear();
  pl_feat.reserve(feat_map.size());

  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    pl_feat.push_back(iter->second.head<3>() / iter->second[3]);
  }
}

inline void down_sampling_voxel(vvec<float, 3> &pl_feat, double inv_voxel_size) {
  std::unordered_map<VOXEL_LOC, Eigen::Vector4d> feat_map;
  feat_map.reserve(pl_feat.size());

  for (const Eigen::Vector3f &point : pl_feat) {
    const VOXEL_LOC position(static_cast<int64_t>(floor(point.x() * inv_voxel_size)), static_cast<int64_t>(floor(point.y() * inv_voxel_size)),
                             static_cast<int64_t>(floor(point.z() * inv_voxel_size)));
    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      Eigen::Vector4d sum;
      sum << point.x(), point.y(), point.z(), 1.0;
      feat_map.emplace(position, sum);
    } else {
      iter->second.head<3>() += point.cast<double>();
      iter->second[3] += 1.0;
    }
  }

  pl_feat.clear();
  pl_feat.reserve(feat_map.size());
  for (const auto &entry : feat_map) {
    const Eigen::Vector4d &sum = entry.second;
    const double count_inv = 1.0 / sum[3];
    pl_feat.emplace_back(static_cast<float>(sum[0] * count_inv), static_cast<float>(sum[1] * count_inv), static_cast<float>(sum[2] * count_inv));
  }
}

inline void down_sampling_pvec(const PointCloud &pvec, double inv_voxel_size, vvec<float, 3> &pl_keep) {
  std::unordered_map<VOXEL_LOC, Eigen::Vector4d> feat_map;
  feat_map.reserve(pvec.size());
  double loc_xyz[3];

  for (const pointVar &pv : pvec) {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pv.pnt[j] * inv_voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      Eigen::Vector4d sum;
      sum << pv.pnt, 1.0;
      feat_map.emplace(position, sum);
    } else {
      iter->second.head<3>() += pv.pnt;
      iter->second[3] += 1.0;
    }
  }

  pl_keep.clear();
  pl_keep.reserve(feat_map.size());
  for (const auto &entry : feat_map) {
    const Eigen::Vector4d &sum = entry.second;
    const double count_inv = 1.0 / sum[3];
    pl_keep.emplace_back(static_cast<float>(sum[0] * count_inv), static_cast<float>(sum[1] * count_inv), static_cast<float>(sum[2] * count_inv));
  }
}

inline void down_sampling_close(std::vector<LidarPoint> &pl_feat, double inv_voxel_size) {
  struct CloseVoxel {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    size_t count = 0;
    double min_distance = std::numeric_limits<double>::max();
    LidarPoint closest;
  };

  auto voxel_location = [inv_voxel_size](const LidarPoint &point) {
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = point[j] * inv_voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    return VOXEL_LOC((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  };

  std::unordered_map<VOXEL_LOC, CloseVoxel> feat_map;
  feat_map.reserve(pl_feat.size());
  for (const LidarPoint &point : pl_feat) {
    CloseVoxel &voxel = feat_map.try_emplace(voxel_location(point)).first->second;
    voxel.center += Eigen::Vector3d(point.x, point.y, point.z);
    voxel.count++;
  }

  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    iter->second.center /= iter->second.count;
  }

  for (const LidarPoint &point : pl_feat) {
    CloseVoxel &voxel = feat_map.find(voxel_location(point))->second;
    const Eigen::Vector3d delta = voxel.center - Eigen::Vector3d(point.x, point.y, point.z);
    const double distance = delta.squaredNorm();
    if (distance < voxel.min_distance) {
      voxel.min_distance = distance;
      voxel.closest = point;
    }
  }

  pl_feat.clear();
  pl_feat.reserve(feat_map.size());
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    pl_feat.push_back(iter->second.closest);
  }
}

}  // namespace sapphire