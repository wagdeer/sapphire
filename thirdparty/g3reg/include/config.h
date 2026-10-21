/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include "graph_vertex.h"
namespace g3reg {
// Values match the measured upstream FPFH/velodyne64 profile. Distances in metres.
struct Config {
  double min_range = 2.0, max_range = 80.0;
  double ds_resolution = 0.5, normal_radius = 1.0, fpfh_radius = 2.5;
  double plane_resolution = 1.0;  // Dense candidate-scoring voxel size.
  clique_solver::VertexInfo vertex_info{{0.2, 0.4, 0.6, 0.8}};
  bool grad_pmc = true;
  double max_clique_time_limit = 3600.0;
  void validate() const;
};
}  // namespace g3reg
