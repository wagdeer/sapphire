#include "frontend/ricp/ricp.hpp"
#include "frontend/eskf/voxel_map.hpp"

#include <map>
#include <tuple>
#include <Eigen/Cholesky>
#include <gtsam/geometry/Pose3.h>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>

namespace sapphire::gicp_detail {
struct Gaussian {
  Eigen::Vector4d mean = Eigen::Vector4d::Ones();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
};

bool make_gaussian(const PointCluster &cluster, double floor, Gaussian &g) {
  g.mean.head<3>() = cluster.mean();
  const Eigen::Matrix3d covariance = cluster.cov();
  if (!g.mean.allFinite() || !covariance.allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig((covariance + covariance.transpose()) * .5);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[0] < -1e-6) return false;
  g.covariance.topLeftCorner<3,3>() = eig.eigenvectors() *
    (eig.eigenvalues().cwiseMax(0.0).array() + floor).matrix().asDiagonal() * eig.eigenvectors().transpose();
  return true;
}

// Call-local library adapter, not a second map. Indices remain stable while the
// cache grows. SerialReduction is required: mutable lazy lookup is not thread safe.
struct VoxelView {
  const VoxelMap &map;
  double distance, floor;
  mutable std::vector<Gaussian> cache;
  mutable std::unordered_map<const OctoTree *, size_t> indices;

  size_t nearest(const Eigen::Vector4d &point, size_t *index, double *squared_distance) const {
    const auto *node = map.nearest_cluster(point.head<3>(), distance);
    if (!node) return 0;
    auto found = indices.find(node);
    if (found == indices.end()) {
      Gaussian g;
      if (!make_gaussian(node->pcr_add, floor, g)) return 0;
      found = indices.emplace(node, cache.size()).first;
      cache.push_back(g);
    }
    *index = found->second;
    *squared_distance = (cache[*index].mean - point).squaredNorm();
    return 1;
  }
};
} // namespace sapphire::gicp_detail

namespace small_gicp::traits {
template <> struct Traits<sapphire::gicp_detail::VoxelView> {
  using View = sapphire::gicp_detail::VoxelView;
  static size_t size(const View &view) { return view.cache.size(); }
  static Eigen::Vector4d point(const View &view, size_t i) { return view.cache[i].mean; }
  static Eigen::Matrix4d cov(const View &view, size_t i) { return view.cache[i].covariance; }
  static size_t nearest_neighbor_search(const View &view, const Eigen::Vector4d &p, size_t *i, double *d) {
    return view.nearest(p,i,d);
  }
};
} // namespace small_gicp::traits

namespace sapphire {
Ricp::Result Ricp::observe(StateGroup &state, const PointCloud &points, const VoxelMap &map) const {
  const auto &cfg = parameters_;
  Result out;
  const auto finite = [](const StateGroup &s) {
    return std::isfinite(s.t) && s.R.allFinite() && s.p.allFinite() && s.v.allFinite() &&
      s.bg.allFinite() && s.ba.allFinite() && s.g.allFinite() && s.cov.allFinite() &&
      s.R.isUnitary(1e-6) && s.R.determinant() > 0;
  };
  if (!finite(state) || state.cov.llt().info() != Eigen::Success || points.empty() ||
      !std::isfinite(cfg.source_resolution) || cfg.source_resolution <= 0 ||
      !std::isfinite(cfg.variance_floor) || cfg.variance_floor <= 0 ||
      !std::isfinite(cfg.max_distance) || cfg.max_distance <= 0 || cfg.max_distance > OctoTree::voxel_size ||
      cfg.max_iterations < 1 || cfg.max_iterations > 20) return out;

  std::map<std::tuple<int64_t,int64_t,int64_t>, PointCluster> bins;
  for (const auto &p : points) {
    if (!p.pnt.allFinite() || !p.var.allFinite() || p.pnt.cwiseAbs().maxCoeff() > 1e6 ||
        (p.pnt / cfg.source_resolution).cwiseAbs().maxCoeff() > 1e12) return out;
    const Eigen::Vector3d k = (p.pnt / cfg.source_resolution).array().floor();
    bins[{static_cast<int64_t>(k.x()),static_cast<int64_t>(k.y()),static_cast<int64_t>(k.z())}].push(p.pnt);
  }
  small_gicp::PointCloud source;
  for (const auto &[key, cluster] : bins) {
    gicp_detail::Gaussian g;
    if (cluster.N >= 3 && gicp_detail::make_gaussian(cluster, 0, g)) {
      source.points.push_back(g.mean); source.covs.push_back(g.covariance);
    }
  }
  out.samples = static_cast<int>(source.size());
  if (out.samples < 12) { out.reason = "source_support"; return out; }

  Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  predicted.linear() = state.R; predicted.translation() = state.p;
  gicp_detail::VoxelView target{map,cfg.max_distance,cfg.variance_floor,{}, {}};
  for (const auto &p : source.points) {
    size_t index; double distance;
    target.nearest(predicted*p,&index,&distance);
  }
  if (target.cache.size() < 6) { out.reason = "target_support"; return out; }

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::SerialReduction> registration;
  registration.rejector.max_dist_sq = cfg.max_distance * cfg.max_distance;
  registration.optimizer.max_iterations = cfg.max_iterations;
  // No inertial prior enters the library objective: it is fused exactly once below.
  const auto result = registration.align(target,source,target,predicted);
  out.iterations = static_cast<int>(result.iterations) + 1;
  out.matches = static_cast<int>(result.num_inliers);
  if (!result.T_target_source.matrix().allFinite() || !result.H.allFinite() ||
      !result.b.allFinite() || !std::isfinite(result.error)) return out;
  if (!result.converged) { out.reason = "not_converged"; return out; }
  StateGroup candidate = state;
  candidate.R = result.T_target_source.linear(); candidate.p = result.T_target_source.translation();
  const auto bounded = [&](const StateGroup &x) {
    return (x.p-state.p).norm() <= cfg.max_distance &&
      lie::SO3d::log(lie::SO3d(state.R.transpose()*x.R)).norm() <= 10.0*M_PI/180.0;
  };
  if (!finite(candidate) || !bounded(candidate)) { out.reason = "correction"; return out; }

  // Library result.H is from before its last step. Obtain derivatives at the
  // actual returned pose with the same upstream factor/reduction implementation.
  std::vector<small_gicp::GICPFactor> factors(source.size());
  const auto [H,b,error] = registration.reduction.linearize(
    target,source,target,registration.rejector,result.T_target_source,factors);
  if (!H.allFinite() || !b.allFinite() || !std::isfinite(error)) return out;
  std::unordered_set<size_t> targets;
  out.matches = 0;
  for (const auto &factor : factors) if (factor.inlier()) { ++out.matches; targets.insert(factor.target_index); }
  out.targets = static_cast<int>(targets.size());
  if (out.matches < 12 || out.targets < 6 || out.matches < .3*out.samples) {
    out.reason = "target_support"; return out;
  }

  // Pose likelihood in the library's right-SE(3) chart. GTSAM supplies chart
  // derivatives; ESKF's translation increments are in world coordinates.
  // H is geometric information, NOT a calibrated independent pose covariance.
  const gtsam::Pose3 measurement(gtsam::Rot3(candidate.R),candidate.p);
  const gtsam::Pose3 prior_pose(gtsam::Rot3(state.R),state.p);
  gtsam::Matrix6 log_jac;
  const gtsam::Vector6 residual = gtsam::Pose3::Logmap(measurement.inverse()*prior_pose,log_jac);
  gtsam::Matrix6 chart = gtsam::Matrix6::Identity();
  chart.bottomRightCorner<3,3>() = state.R.transpose();
  const gtsam::Matrix6 J = log_jac * chart;
  using Matrix = Eigen::Matrix<double,STATE_DOF,STATE_DOF>;
  const Matrix I = Matrix::Identity();
  Matrix information = Matrix::Zero();
  information.topLeftCorner<6,6>() = J.transpose()*H*J;
  const Matrix prior_information = state.cov.llt().solve(I);
  Eigen::LLT<Matrix> llt(prior_information+information);
  if (llt.info()!=Eigen::Success) { out.reason = "information"; return out; }
  const Matrix posterior = llt.solve(I);
  const Eigen::Matrix<double,STATE_DOF,1> correction = -posterior.leftCols<6>()*J.transpose()*H*residual;
  if (!correction.allFinite()) return out;
  StateGroup next = state;
  next += correction;
  out.correction = (next.p-state.p).norm();
  if (!finite(next) || !bounded(next)) { out.reason = "correction"; return out; }

  Eigen::Isometry3d fused = Eigen::Isometry3d::Identity();
  fused.linear() = next.R; fused.translation() = next.p;
  double residual_sq = 0;
  for (const auto &factor : factors) if (factor.inlier()) {
    residual_sq += (target.cache[factor.target_index].mean-fused*source.points[factor.source_index]).squaredNorm();
  }
  out.rms = std::sqrt(residual_sq/out.matches);
  const double before = registration.reduction.error(target,source,predicted,factors);
  const double after = registration.reduction.error(target,source,fused,factors) + .5*correction.dot(prior_information*correction);
  if (!std::isfinite(after) || after > before + 1e-8 || out.rms > cfg.max_distance*.5) {
    out.reason = "objective"; return out;
  }
  // Reset the rotation error chart after the single filter correction.
  gtsam::Matrix3 rotation_reset;
  gtsam::Rot3::Expmap(correction.head<3>(),rotation_reset);
  Matrix reset = I; reset.topLeftCorner<3,3>() = rotation_reset;
  next.cov = reset*posterior*reset.transpose();
  next.cov = .5*(next.cov+next.cov.transpose()).eval();
  if (!finite(next) || next.cov.llt().info()!=Eigen::Success) return out;
  state = next;
  out.accepted = true; out.reason = "accepted";
  return out;
}
} // namespace sapphire
