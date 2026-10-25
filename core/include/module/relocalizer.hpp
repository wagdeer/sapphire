#pragma once
#include <set>

#include "maplayer/topograph/node.hpp"
#include <parameters.hpp>
#include "solver/icp_solver.hpp"
namespace sapphire {
class Memory;
class Relocalizer final {
 public:
  Relocalizer(Memory&, const RelocalizerParams& = {}, const RegistrationParams& = {});
  std::vector<int> spatial_candidates(const Node&, const std::map<int, SE3f>&, const std::set<int>& excluded = {}) const;
  lidar::Alignment verify(int reference, int query, SE3f guess) const;

 private:
  RelocalizerParams config_;
  RegistrationParams lidar_registration_;
  Memory& memory_;
};
}  // namespace sapphire
