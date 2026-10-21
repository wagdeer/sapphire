/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include "reglib.h"
#include "pmc_solver.h"
namespace pagor {
class PyramidRegistrationSolver {
 public:
  explicit PyramidRegistrationSolver(const g3reg::Config& config)
      : config_(config), num_graphs_(config.vertex_info.noise_bound_vec.size()), inlier_graphs_(num_graphs_), max_cliques_(num_graphs_) {
    for (auto& graph : inlier_graphs_) graph.setType(false);
  }
  g3reg::FRGresult solve(const std::vector<clique_solver::GraphVertex::Ptr>& source, const std::vector<clique_solver::GraphVertex::Ptr>& target,
                         const clique_solver::Association& associations);

 private:
  void buildGraphs(const std::vector<clique_solver::GraphVertex::Ptr>& source, const std::vector<clique_solver::GraphVertex::Ptr>& target);
  void solveMaxClique();
  const g3reg::Config& config_;
  int num_graphs_;
  std::size_t num_corr_ = 0;
  std::vector<clique_solver::Graph> inlier_graphs_;
  std::vector<std::vector<int>> max_cliques_;
  clique_solver::Association A_;
};
}  // namespace pagor
