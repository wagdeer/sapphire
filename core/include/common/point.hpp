#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include "common/common_type.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "thirdparty/nanoflann/nanoflann.hpp"
#include "imu_state.hpp"
#include "sensor/lidar_type.hpp"

namespace sapphire {

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
