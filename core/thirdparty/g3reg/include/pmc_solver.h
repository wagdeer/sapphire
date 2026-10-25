//
// Created by Zhijian QIAO on 23-5-24.
//

#ifndef SRC_PMC_SOLVER_H
#define SRC_PMC_SOLVER_H

#include "graph.h"
/**
 * A facade to the Parallel Maximum Clique (PMC) library.
 *  // 1. k-core pruning
    // 2. neigh-core pruning/ordering
    // 3. dynamic coloring bounds/sort
    // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum Clique Algorithms with Applications to Network Analysis,” SIAM J. Sci.
 Comput., vol. 37, no. 5, pp. C589–C616, Jan. 2015.
 * For details about PMC, please refer to:
 * https://github.com/ryanrossi/pmc
 * and
 * Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa Patwary, A Fast Parallel
 * Maximum Clique Algorithm for Large Sparse Graphs and Temporal Strong Components, arXiv preprint
 * 1302.6256, 2013.
 */

namespace clique_solver {
class MaxCliqueSolver {
 public:
  struct Params {
    double time_limit = 3600;
  };

  MaxCliqueSolver() = default;

  MaxCliqueSolver(Params params) : params_(params){};

  /**
   * Find the maximum clique within the graph provided. By maximum clique, it means the clique of
   * the largest size in an undirected graph.
   * @param graph
   * @return a vector of indices of cliques
   */
  std::vector<int> findMaxClique(Graph graph, int lower_bound = 0);

 private:
  Params params_;
};
}  // namespace clique_solver

#endif  // SRC_PMC_SOLVER_H
