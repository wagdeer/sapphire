#pragma once
#include <algorithm>
#include <backend/maplayer/featmap/maintenance.hpp>

#include "validation.hpp"
namespace sapphire::backend::featmap::detail {
inline void merge_reliability(NodeObservations::Reliability& target, const NodeObservations::Reliability& source) {
  target.confirmed |= source.confirmed;
  for (const auto& trial : source.trials) {
    auto found = std::find_if(target.trials.begin(), target.trials.end(), [&](const auto& t) { return t.first == trial.first; });
    if (found == target.trials.end())
      target.trials.push_back(trial);
    else
      found->second |= trial.second;
  }
  if (target.trials.size() > 8) target.trials.erase(target.trials.begin(), target.trials.end() - 8);
}

void apply_refined_geometry(std::vector<Landmark>&, const std::map<int, StateGroupf>& before_poses, const std::map<uint64_t, int>& references, const NodeObservations& after,
                            const std::map<uint64_t, cv::Point3f>& solved);
void apply_depth_quality(std::vector<Landmark>&, const NodeObservations&, const std::map<uint64_t, Mat33_t<double>>&);
void update_representative_descriptors(std::vector<Landmark>&, const NodeObservations&, const std::map<int, StateGroupf>&);
void retire_unsupported_points(std::vector<Landmark>&, NodeObservations&, const std::set<uint64_t>&);
std::shared_ptr<const LandmarkMap> retire_unsupported(const LandmarkMap&, NodeObservations&, const std::set<uint64_t>&);

}  // namespace sapphire::backend::featmap::detail
