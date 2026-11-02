#pragma once
#include <set>
#include <array>

#include "maplayer/topograph/node.hpp"
#include <parameters.hpp>
#include "solver/icp_solver.hpp"
namespace sapphire {
class Memory;
class Relocalizer final {
 public:
  Relocalizer(Memory&, const RelocalizerParams& = {}, const RegistrationParams& = {});
  std::vector<int> candidates(const Node&, const std::map<int, SE3f>&);
  struct Path { int anchor = 0; std::vector<int> nodes; };
  struct Verification {
    lidar::Alignment alignment;
    int component = 0;
    SE3f datum_from_anchor;
    std::vector<std::pair<SE3f,double>> alternatives;
    Path path;  // Effective contiguous target after payload admission.
    std::string resource_failure;
    bool duplicate = false;
    double load_ms = 0, assemble_ms = 0;
    double start_s = 0, end_s = 0, thread_cpu_ms = 0, process_cpu_ms = 0;
    std::size_t rss_before = 0, rss_after = 0;
    std::string initialization_trace;
    std::size_t target_points = 0, query_points = 0, target_bytes = 0, requested_bytes = 0;
  };
  std::vector<Path> paths(int anchor, int query, const std::map<int, SE3f>& poses) const;
  Verification verify(const Path&, int query, const std::map<int, SE3f>& poses,
                      const std::set<std::vector<int>>& attempted = {}) const;
  // Prepared exclusively on the map owner; evaluation owns only frozen geometry.
  struct Input {
    Verification result;
    GaussianCloudPtr target, source;
    SE3f guess;
  };
  Input prepare(const Path&, int query, const std::map<int, SE3f>&,
                const std::set<std::vector<int>>& attempted = {}) const;
  static Verification evaluate(Input, const RelocalizerParams&, const RegistrationParams&);
  static std::optional<std::size_t> select(const std::vector<Verification>&,
                                          const RegistrationParams&, std::string& reason);
  std::vector<int> path_neighbors(int id) const;
  lidar::Alignment verify(int reference, int query, SE3f guess) const;

 private:
  GaussianCloudPtr load_path(const Path&, int query, const std::map<int, SE3f>&,
                             const std::set<std::vector<int>>&, Verification&) const;
  std::map<int,std::array<float,64>> signatures_;
  RelocalizerParams config_;
  RegistrationParams lidar_registration_;
  Memory& memory_;
};
}  // namespace sapphire
