/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "graph_vertex.h"
namespace clique_solver {
GraphVertex::Ptr PointVertex::operator-(const GraphVertex &other) const {
  const Eigen::Vector3d &diff = centroid - other.centroid;
  return GraphVertex::Ptr(new PointVertex(diff, vertex_info_));
}
Eigen::VectorXd PointVertex::consistent(const GraphVertex &other) const {
  const double &v1i_dist = this->norm();
  const double &v2i_dist = other.norm();
  if (v1i_dist < 1e-6 || v2i_dist < 1e-6) return Eigen::VectorXd::Zero(num_graphs());

  const double &meas = std::abs(v1i_dist - v2i_dist);

  Eigen::VectorXd prob = Eigen::VectorXd::Zero(num_graphs());
  //        const double& sigma2 = vertex_info_.sigma * vertex_info_.sigma;
  // noise bound vector is in ascending order
  for (int level = 0; level < num_graphs(); ++level) {
    const double &beta = 2 * vertex_info_.noise_bound_vec[level];
    if (meas < beta) {
      //                prob(level) = std::exp(-0.5 * (meas * meas) / sigma2);
      prob(level) = 0.99;
      for (int j = level + 1; j < num_graphs(); ++j) {
        prob(j) = prob(level);
      }
      break;
    } else {
      prob(level) = 0.0;
    }
  }
  return prob;
}
std::tuple<size_t, size_t> k2ij(size_t k, size_t n) {
  k += 1;

  const size_t l = n * (n - 1) / 2 - k;
  const size_t o = std::floor((std::sqrt(1 + 8 * l) - 1) / 2.);
  const size_t p = l - o * (o + 1) / 2;
  const size_t i = n - (o + 1);
  const size_t j = n - p;
  return {i - 1, j - 1};
}

}  // namespace clique_solver
