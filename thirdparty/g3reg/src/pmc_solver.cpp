/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "pmc_solver.h"

#include <chrono>

#include "graph.h"
#include "pmc.h"

namespace clique_solver {

vector<int> clique_solver::MaxCliqueSolver::findMaxClique(clique_solver::Graph graph, int lower_bound) {
  // Create a PMC graph from the TEASER graph
  int num_vertices = graph.numVertices();
  vector<int> edges;
  edges.reserve(graph.numEdges() * 2);
  vector<long long> vertices;
  vertices.reserve(num_vertices + 1);

  vertices.push_back(0);
  for (size_t i = 0; i < num_vertices; ++i) {
    const auto &c_edges = graph.getEdges(i);
    edges.insert(edges.end(), c_edges.begin(), c_edges.end());
    vertices.push_back(edges.size());
  }

  // Use PMC to calculate
  pmc::pmc_graph G(vertices, edges);  // typically takes 0.005 ms
  // upper-bound of max clique
  G.compute_cores();
  int max_core = G.get_max_core();  // typically takes 0.040 ms, get the upper bound of clique size

  vector<int> C;  // vector to represent max clique

  pmc::input in;  // use default input
  in.time_limit = params_.time_limit;
  in.threads = 1;  // Sapphire: serial registration on the owning backend worker.
  in.lb = lower_bound;
  in.ub = in.ub == 0 ? max_core + 1 : in.ub;

  // lower-bound of max clique
  if (in.lb == 0) {
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, C);  // typically takes 0.120 ms
  }

  // This means that max clique has a size of one
  if (in.lb == 0) return C;  // error

  if (in.lb == in.ub) return C;

  if (G.num_vertices() < in.adj_limit) {
    G.create_adj();
    pmc::pmcx_maxclique finder(G, in);
    finder.search_dense(G, C);
  } else {
    std::cout << "PMC: Graph is too dense, so don't use adj matrix to speed up" << std::endl;
    pmc::pmcx_maxclique finder(G, in);
    finder.search(G, C);
  }

  return C;
}
}  // namespace clique_solver