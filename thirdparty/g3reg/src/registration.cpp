/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "registration.h"

#include <algorithm>

#include "opt_utils.h"
#include "tic_toc.h"
namespace pagor {
g3reg::FRGresult PyramidRegistrationSolver::solve(const std::vector<clique_solver::GraphVertex::Ptr>& src,
                                                  const std::vector<clique_solver::GraphVertex::Ptr>& dst,
                                                  const clique_solver::Association& associations) {
  g3reg::FRGresult result;
  if (associations.rows() == 0) return result;
  A_ = associations;
  robot_utils::TicToc timer;
  buildGraphs(src, dst);
  result.graph_time = timer.toc();
  timer.tic();
  solveMaxClique();
  result.clique_time = timer.toc();
  timer.tic();
  result.candidates.resize(num_graphs_, Eigen::Matrix4d::Identity());
  for (int level = 0; level < num_graphs_; ++level) {
    const auto& clique = max_cliques_[level];
    if (level > 0 && clique == max_cliques_[level - 1]) {
      result.candidates[level] = result.candidates[level - 1];
      continue;
    }
    if (clique.size() < 3) {
      result.valid = false;
      continue;
    }
    Eigen::Matrix3Xd source(3, clique.size()), target(3, clique.size());
    for (std::size_t i = 0; i < clique.size(); ++i) {
      source.col(i) = src.at(A_(clique[i], 0))->centroid;
      target.col(i) = dst.at(A_(clique[i], 1))->centroid;
    }
    result.candidates[level] = gtsam::gncSE3(source, target, gtsam::svdSE3(source, target));
    result.valid = true;
  }
  result.tf_solver_time = timer.toc();
  return result;
}
void PyramidRegistrationSolver::solveMaxClique() {
  clique_solver::MaxCliqueSolver::Params params;
  params.time_limit = config_.max_clique_time_limit;
  int prune_level = 0;
  for (int level = 0; level < num_graphs_; ++level) {
    max_cliques_[level] = clique_solver::MaxCliqueSolver(params).findMaxClique(inlier_graphs_[level], prune_level);
    prune_level = config_.grad_pmc ? max_cliques_[level].size() : 0;
  }
  for (auto& clique : max_cliques_) std::sort(clique.begin(), clique.end());
}
void PyramidRegistrationSolver::buildGraphs(const std::vector<clique_solver::GraphVertex::Ptr>& v1,
                                            const std::vector<clique_solver::GraphVertex::Ptr>& v2) {
  num_corr_ = A_.rows();
  int num_tims = num_corr_ * (num_corr_ - 1) / 2;

  for (int level = 0; level < num_graphs_; ++level) {
    inlier_graphs_[level].populateVertices(num_corr_);
  }

  for (size_t k = 0; k < num_tims; ++k) {
    size_t i, j;
    std::tie(i, j) = clique_solver::k2ij(k, num_corr_);
    const auto& weights = (*v1[A_(j, 0)] - *v1[A_(i, 0)])->consistent(*(*v2[A_(j, 1)] - *v2[A_(i, 1)]));
    for (int level = 0; level < num_graphs_; ++level) {
      if (weights(level) > 0.0) {
        { inlier_graphs_[level].addEdge(i, j); }
      }
    }
  }

  // We assume no scale difference between the two vectors of points.
}
}  // namespace pagor
