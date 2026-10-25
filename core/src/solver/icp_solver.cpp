#include "solver/icp_solver.hpp"

#include <Eigen/Eigenvalues>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction.hpp>
#include <small_gicp/registration/registration.hpp>

#include "solver/bbs.hpp"

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

namespace sapphire::lidar {
Alignment align(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& initial, const RegistrationParams& config) {
  config.validate();
  Alignment out;
  if (!target || !query || target->size() < size_t(config.lidar_min_inliers) || query->size() < size_t(config.lidar_min_inliers) ||
      !initial.isValid()) {
    out.failure = "Missing LiDAR geometry, support, or valid initial pose";
    return out;
  }
  validateGaussianCloud(*target);
  validateGaussianCloud(*query);
  small_gicp::KdTree<GaussianCloud> tree(target);
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::SerialReduction> solver;
  solver.optimizer.max_iterations = config.lidar_max_iterations;
  solver.rejector.max_dist_sq = config.lidar_max_correspondence_m * config.lidar_max_correspondence_m;
  solver.criteria.translation_eps = 0.01;
  solver.criteria.rotation_eps = 0.01;
  Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
  seed.matrix() = initial.matrix().cast<double>();
  if (config.lidar_bbs_enabled) {
    cpu::VoxelMaps pyramid;
    pyramid.set_min_res(.25f);
    pyramid.create_voxelmaps(target->size(), [&](size_t i) { return target->at(i).mean; });
    auto voxels = pyramid.release_data();
    gpu::LocalSearchTarget candidate;
    candidate.voxelmaps = &voxels;
    candidate.initial_target_T_source = seed.cast<float>();
    const auto coarse = sapphire::alignLoopBbs(*query, {candidate});
    if (!coarse.accepted) {
      out.failure = "LiDAR BBS overlap rejected";
      return out;
    }
    seed = coarse.T_target_query;
  }
  const auto result = solver.align(*target, *query, tree, seed);
  out.inliers = result.num_inliers;
  if (!result.converged || !result.T_target_source.matrix().allFinite() || out.inliers < size_t(config.lidar_min_inliers) ||
      double(out.inliers) / query->size() < config.lidar_min_overlap) {
    out.failure = "LiDAR registration did not converge with sufficient overlap";
    return out;
  }
  double error = 0;
  size_t count = 0;
  for (size_t i = 0; i < query->size(); ++i) {
    size_t neighbor;
    double distance;
    const Eigen::Vector4d point = result.T_target_source.matrix() * small_gicp::traits::point(*query, i);
    if (small_gicp::traits::nearest_neighbor_search(tree, point, &neighbor, &distance) && distance <= solver.rejector.max_dist_sq) {
      error += distance;
      ++count;
    }
  }
  out.fitness = count ? error / count : std::numeric_limits<double>::infinity();
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
}  // namespace sapphire::lidar
