/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <tuple>
#include <vector>
namespace clique_solver {
struct VertexInfo {
  std::vector<double> noise_bound_vec;
};
class GraphVertex {
 public:
  using Ptr = std::shared_ptr<GraphVertex>;
  GraphVertex(const Eigen::Vector3d& centroid, const VertexInfo& info) : centroid(centroid), vertex_info_(info) {}
  virtual ~GraphVertex() = default;
  double norm() const { return centroid.norm(); }
  int num_graphs() const { return static_cast<int>(vertex_info_.noise_bound_vec.size()); }
  virtual Ptr operator-(const GraphVertex&) const = 0;
  virtual Eigen::VectorXd consistent(const GraphVertex&) const = 0;
  Eigen::Vector3d centroid;
  VertexInfo vertex_info_;
};
class PointVertex final : public GraphVertex {
 public:
  using GraphVertex::GraphVertex;
  Ptr operator-(const GraphVertex& other) const override;
  Eigen::VectorXd consistent(const GraphVertex& other) const override;
};
inline GraphVertex::Ptr create_vertex(const Eigen::Vector3d& center, const VertexInfo& info) { return std::make_shared<PointVertex>(center, info); }
std::tuple<size_t, size_t> k2ij(size_t k, size_t n);
}  // namespace clique_solver
