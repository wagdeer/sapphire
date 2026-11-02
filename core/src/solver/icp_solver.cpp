#include "solver/icp_solver.hpp"

#include <Eigen/Eigenvalues>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/registration/registration.hpp>

#include "solver/bbs.hpp"
#include "thirdparty/nanoflann/nanoflann.hpp"
#include "tools/timer.hpp"

namespace {
Eigen::Vector4d homogeneous_point(const sapphire::GaussianPoint& point) {
  return Eigen::Vector4d(point.mean.x(), point.mean.y(), point.mean.z(), 1.0);
}

// Registration already rejects neighbors outside this radius. Bound the exact
// search itself so non-overlapping candidates do not search distant geometry.
// Borrows the immutable cloud for this call; no persistent index or extra copy.
struct GaussianSearch {
  using Tree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, GaussianSearch>, GaussianSearch, 3, size_t>;
  const sapphire::GaussianCloud& points;
  const double max_dist_sq;
  Tree tree;

  GaussianSearch(const sapphire::GaussianCloud& points, double max_dist_sq)
      : points(points), max_dist_sq(max_dist_sq), tree(3, *this, {20}) {}
  size_t kdtree_get_point_count() const { return points.size(); }
  double kdtree_get_pt(size_t index, size_t axis) const { return points[index].mean[axis]; }
  template <typename Box> bool kdtree_get_bbox(Box&) const { return false; }

  size_t nearest(const Eigen::Vector4d& point, size_t* index, double* squared) const {
    // nanoflann uses a strict radius; the existing correspondence gate is <=.
    const double inclusive = std::nextafter(max_dist_sq, std::numeric_limits<double>::infinity());
    return tree.rknnSearch(point.data(), 1, index, squared, inclusive);
  }
};

// A correspondence and its frozen metric for the current LM linearization.
struct GaussianFactor {
  struct Setting {};
  size_t target_index = std::numeric_limits<size_t>::max();
  double squared_distance = 0;
  Eigen::Vector3d target_body = Eigen::Vector3d::Zero();
  Eigen::Matrix3d precision = Eigen::Matrix3d::Zero();
  explicit GaussianFactor(const Setting& = Setting{}) {}
  bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }
};

struct GaussianReduction {
  Eigen::Matrix3d inverse_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d origin_translation = Eigen::Vector3d::Zero();

  auto linearize(const sapphire::GaussianCloud& target, const sapphire::GaussianCloud& source,
                 const GaussianSearch& tree, const small_gicp::DistanceRejector& rejector,
                 const Eigen::Isometry3d& pose, std::vector<GaussianFactor>& factors) {
    // The stored float rotation may be slightly nonorthogonal: use its actual
    // inverse, not transpose. B = (Q Ct Q^T + Cq)^-1 = R^T M R.
    inverse_rotation = pose.linear().inverse();
    origin_translation = pose.translation();
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    double error = 0;
    for (size_t i = 0; i < source.size(); ++i) {
      auto& factor = factors[i];
      factor.target_index = std::numeric_limits<size_t>::max();
      const Eigen::Vector3d point = source[i].mean.cast<double>();
      const Eigen::Vector4d transformed = pose * homogeneous_point(source[i]);
      size_t neighbor;
      double distance;
      if (!tree.nearest(transformed, &neighbor, &distance) || rejector(target, source, pose, neighbor, i, distance)) continue;
      factor.target_index = neighbor;
      factor.squared_distance = distance;
      factor.target_body = inverse_rotation * (target[neighbor].mean.cast<double>() - origin_translation);
      factor.precision = (inverse_rotation * target[neighbor].covariance.cast<double>() * inverse_rotation.transpose() +
                          source[i].covariance.cast<double>()).inverse();
      const Eigen::Matrix3d skew = small_gicp::skew(point);
      const Eigen::Matrix3d weighted_skew = factor.precision * skew;
      const Eigen::Vector3d residual = factor.target_body - point;
      const Eigen::Vector3d weighted = factor.precision * residual;
      // Right-increment Jacobian in this frame is [skew(point), -I].
      H.topLeftCorner<3, 3>() += skew.transpose() * weighted_skew;
      H.topRightCorner<3, 3>() -= skew.transpose() * factor.precision;
      H.bottomLeftCorner<3, 3>() -= weighted_skew;
      H.bottomRightCorner<3, 3>() += factor.precision;
      b.head<3>() += skew.transpose() * weighted;
      b.tail<3>() -= weighted;
      error += .5 * residual.dot(weighted);
    }
    return std::make_tuple(H, b, error);
  }

  double error(const sapphire::GaussianCloud&, const sapphire::GaussianCloud& source,
               const Eigen::Isometry3d& pose, std::vector<GaussianFactor>& factors) const {
    // Freeze correspondences and precision during LM trials, exactly as before.
    const Eigen::Matrix3d rotation = inverse_rotation * pose.linear();
    const Eigen::Vector3d translation = inverse_rotation * (pose.translation() - origin_translation);
    double error = 0;
    for (size_t i = 0; i < source.size(); ++i) if (factors[i].inlier()) {
      const auto& factor = factors[i];
      const Eigen::Vector3d residual = factor.target_body - (rotation * source[i].mean.cast<double>() + translation);
      error += .5 * residual.dot(factor.precision * residual);
    }
    return error;
  }
};
}  // namespace

namespace sapphire::lidar {
static Alignment register_clouds(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& initial, const RegistrationParams& config, bool optimize) {
  config.validate();
  Alignment out;
  if (!target || !query || target->size() < size_t(config.lidar_min_inliers) || query->size() < size_t(config.lidar_min_inliers) ||
      !initial.isValid()) {
    out.failure = "Missing LiDAR geometry, support, or valid initial pose";
    return out;
  }
  validateGaussianCloud(*target);
  validateGaussianCloud(*query);
  tools::Timer timer;
  GaussianSearch tree(*target, config.lidar_max_correspondence_m * config.lidar_max_correspondence_m);
  small_gicp::Registration<GaussianFactor, GaussianReduction> solver;
  solver.optimizer.max_iterations = config.lidar_max_iterations;
  solver.rejector.max_dist_sq = config.lidar_max_correspondence_m * config.lidar_max_correspondence_m;
  solver.criteria.translation_eps = 0.01;
  solver.criteria.rotation_eps = 0.01;
  Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
  seed.matrix() = initial.matrix().cast<double>();
  if (optimize && config.lidar_bbs_enabled) {
    cpu::VoxelMaps pyramid;
    pyramid.set_min_res(.25f);
    pyramid.create_voxelmaps(target->size(), [&](size_t i) { return target->at(i).mean; });
    auto voxels = pyramid.release_data();
    gpu::LocalSearchTarget candidate;
    candidate.voxelmaps = &voxels;
    candidate.initial_target_T_source = seed.cast<float>();
    out.pyramid_ms = timer.lap_seconds()*1000;
    const auto coarse = sapphire::alignLoopBbs(*query, {candidate});
    out.bbs_ms = timer.lap_seconds()*1000;
    out.bbs_roots = coarse.root_nodes;
    out.bbs_queue_peak = coarse.queue_peak_nodes;
    out.device_peak_bytes = coarse.device_peak_bytes;
    for (size_t level=0; level<4; ++level) { out.bbs_expanded += coarse.expanded_nodes[level]; out.bbs_pruned += coarse.pruned_nodes[level]; }
    if (!coarse.accepted) {
      out.failure = "LiDAR BBS overlap rejected";
      return out;
    }
    seed = coarse.T_target_query;
  }
  timer.reset();
  small_gicp::RegistrationResult result(seed);
  std::vector<GaussianFactor> factors(query->size());
  if (optimize) result = solver.optimizer.optimize(*target, *query, tree, solver.rejector, solver.criteria,
                                                   solver.reduction, seed, factors, solver.general_factor);
  else {
    std::tie(result.H, result.b, result.error) = solver.reduction.linearize(*target, *query, tree, solver.rejector, seed, factors);
    result.num_inliers = std::count_if(factors.begin(), factors.end(), [](const auto& f) { return f.inlier(); });
    result.converged = true;
  }
  out.gicp_ms = timer.lap_seconds()*1000;
  out.iterations = result.iterations;
  out.inliers = result.num_inliers;
  out.distribution_error = result.error / std::max<size_t>(1, out.inliers);
  if (!result.T_target_source.matrix().allFinite()) {
    out.failure = "LiDAR registration produced a non-finite pose";
    return out;
  }
  double error = 0;
  size_t count = 0;
  for (size_t i = 0; i < query->size(); ++i) {
    size_t neighbor;
    double distance;
    const Eigen::Vector4d input = homogeneous_point((*query)[i]);
    const Eigen::Vector4d point = result.T_target_source * input;
    // Linearization and metrics use the same point transform. evaluate() keeps
    // that pose fixed, so its correspondence and distance are already final.
    const bool reusable = !optimize;
    bool found;
    if (reusable) {
      found = factors[i].inlier();
      neighbor = factors[i].target_index;
      distance = factors[i].squared_distance;
    } else {
      found = tree.nearest(point, &neighbor, &distance);
    }
    if (found && distance <= solver.rejector.max_dist_sq) {
      out.matches.push_back({neighbor, i});
      error += distance;
      ++count;
    }
  }
  out.metric_ms = timer.lap_seconds()*1000;
  out.fitness = count ? error / count : std::numeric_limits<double>::infinity();
  // Diagnostics evaluate the final iterate even when it is rejected. Acceptance
  // still requires convergence and the original support/fitness/curvature gates.
  if (!result.converged) {out.failure="LiDAR GICP did not converge";return out;}
  if (out.inliers < size_t(config.lidar_min_inliers) || double(out.inliers)/query->size()<config.lidar_min_overlap) {
    out.failure="LiDAR geometric support rejected";return out;
  }
  out.fitted_pose=SE3f(Eigen::Matrix4f(result.T_target_source.matrix().cast<float>()));
  if (out.fitness > config.lidar_max_fitness_m2) {
    out.failure = "LiDAR metric fitness rejected";
    return out;
  }
  const Mat66_t<double> information = (result.H + result.H.transpose()) / (2.0 * std::max<size_t>(1, out.inliers));
  Eigen::SelfAdjointEigenSolver<Mat66_t<double>> eig(information);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues().minCoeff() <= 0) {
    out.failure = "LiDAR registration has singular curvature";
    return out;
  }
  const Mat66_t<double> rt = eig.eigenvectors() * eig.eigenvalues().cwiseInverse().asDiagonal() * eig.eigenvectors().transpose();
  Mat66_t<double> covariance;
  const int order[6] = {3, 4, 5, 0, 1, 2};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) covariance(r, c) = rt(order[r], order[c]);
  covariance.diagonal().head<3>().array() += config.lidar_translation_sigma_m * config.lidar_translation_sigma_m;
  covariance.diagonal().tail<3>().array() += config.lidar_rotation_sigma_rad * config.lidar_rotation_sigma_rad;
  out.covariance = covariance;
  out.T_target_query = SE3f(Eigen::Matrix4f(result.T_target_source.matrix().cast<float>()));
  return out;
}
GaussianMatches match(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& pose, double distance) {
  GaussianMatches matches;
  if (!target || !query) return matches;
  if (!pose.isValid() || !std::isfinite(distance) || distance <= 0) throw std::invalid_argument("Invalid Gaussian matcher inputs");
  validateGaussianCloud(*target); validateGaussianCloud(*query);
  GaussianSearch tree(*target, distance*distance);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = pose.matrix().cast<double>();
  matches.reserve(query->size());
  for (size_t i=0;i<query->size();++i) {
    size_t neighbor; double squared;
    const Eigen::Vector4d point=transform*homogeneous_point((*query)[i]);
    if (tree.nearest(point,&neighbor,&squared) && squared <= distance*distance) matches.push_back({neighbor,i});
  }
  return matches;
}
Alignment align(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& initial, const RegistrationParams& config) {
  return register_clouds(target,query,initial,config,true);
}
Alignment evaluate(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& pose, const RegistrationParams& config) {
  return register_clouds(target,query,pose,config,false);
}
}  // namespace sapphire::lidar
