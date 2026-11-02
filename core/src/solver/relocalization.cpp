#include "solver/relocalization.hpp"

#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <map>
#include <tuple>

#include "registration.h"
namespace sapphire::lidar {
std::array<float, 64> geometry_signature(const GaussianCloud& cloud, float resolution) {
  std::array<float, 64> out{};
  if (cloud.empty()) return out;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& p : cloud) center += p.mean.cast<double>();
  center /= cloud.size();
  for (const auto& p : cloud) out[static_cast<size_t>(std::min(63.0, (p.mean.cast<double>() - center).norm() / resolution))] += 1;
  float norm = 0;
  for (float x : out) norm += x * x;
  norm = std::sqrt(norm);
  for (float& x : out) x /= norm;
  return out;
}
namespace {
struct Features {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::Normal>::Ptr normals{new pcl::PointCloud<pcl::Normal>};
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors{new pcl::PointCloud<pcl::FPFHSignature33>};
  std::vector<clique_solver::GraphVertex::Ptr> vertices;
};
Features features(const GaussianCloud& cloud, const g3reg::Config& config) {
  Features out;
  std::map<std::tuple<int, int, int>, const GaussianPoint*> cells;
  for (const auto& g : cloud) {
    const auto p = (g.mean / config.ds_resolution).array().floor().cast<int>();
    const auto key = std::make_tuple(p.x(), p.y(), p.z());
    auto& selected = cells[key];
    if (!selected || g.N > selected->N) selected = &g;
  }
  for (const auto& [key, g] : cells) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(g->covariance);
    if (eig.info() != Eigen::Success) continue;
    Eigen::Vector3f n = eig.eigenvectors().col(0);
    if (n.dot(g->mean) > 0) n = -n;
    out.points->push_back(pcl::PointXYZ(g->mean.x(), g->mean.y(), g->mean.z()));
    pcl::Normal normal;
    normal.normal_x = n.x();
    normal.normal_y = n.y();
    normal.normal_z = n.z();
    out.normals->push_back(normal);
    out.vertices.push_back(clique_solver::create_vertex(g->mean.cast<double>(), config.vertex_info));
  }
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> feature;
  feature.setInputCloud(out.points);
  feature.setInputNormals(out.normals);
  feature.setRadiusSearch(config.fpfh_radius);
  feature.compute(*out.descriptors);
  return out;
}
}  // namespace
std::vector<SE3f> initial_poses(const GaussianCloud& target, const GaussianCloud& query, const RelocalizerParams& config, std::string& failure) {
  if (target.empty() || query.empty()) {
    failure = "Missing geometry for global registration";
    return {};
  }
  g3reg::Config c;
  c.ds_resolution = config.global_resolution_m;
  c.fpfh_radius = config.global_feature_radius_m;
  c.max_clique_time_limit = config.global_clique_time_sec;
  c.grad_pmc = false;
  const auto a = features(query, c), b = features(target, c);
  if (a.vertices.size() < 3 || b.vertices.size() < 3) {
    failure = "Insufficient coarse geometric features";
    return {};
  }
  pcl::KdTreeFLANN<pcl::FPFHSignature33> ta, tb;
  ta.setInputCloud(a.descriptors);
  tb.setInputCloud(b.descriptors);
  std::vector<std::pair<int, int>> matches;
  std::vector<int> idx(1), back(1);
  std::vector<float> dist(1), bd(1);
  for (size_t i = 0; i < a.descriptors->size(); ++i) {
    const auto& descriptor = a.descriptors->at(i);
    bool finite = true;
    for (float v : descriptor.histogram) finite = finite && std::isfinite(v);
    if (!finite) continue;
    if (tb.nearestKSearch(descriptor, 1, idx, dist) != 1) continue;
    if (ta.nearestKSearch(b.descriptors->at(idx[0]), 1, back, bd) == 1 && back[0] == int(i)) matches.emplace_back(i, idx[0]);
    if (matches.size() > config.global_max_associations) {
      failure = "PAGOR association memory budget exceeded";
      return {};
    }
  }
  if (matches.size() < 3) {
    failure = "Insufficient reciprocal geometric associations";
    return {};
  }
  clique_solver::Association pairs(matches.size(), 2);
  for (size_t i = 0; i < matches.size(); ++i) pairs.row(i) << matches[i].first, matches[i].second;
  const auto result = pagor::PyramidRegistrationSolver(c).solve(a.vertices, b.vertices, pairs);
  if (!result.valid) {
    failure = "PAGOR found no consistent transform";
    return {};
  }
  std::vector<SE3f> seeds;
  for (const auto& matrix : result.candidates) {
    const SE3f pose{Eigen::Matrix4f(matrix.cast<float>())};
    if (!pose.isValid()) continue;
    bool duplicate = false;
    for (const auto& other : seeds)
      if ((other.matrix() - pose.matrix()).norm() < 1e-4) duplicate = true;
    if (!duplicate) seeds.push_back(pose);
  }
  if (seeds.empty()) failure = "PAGOR returned no finite SE3 hypothesis";
  return seeds;
}
}  // namespace sapphire::lidar
