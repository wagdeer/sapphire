#include "module/relocalizer.hpp"

#include "database/memory.hpp"
namespace sapphire {
Relocalizer::Relocalizer(Memory& memory, const RelocalizerParams& parameters, const RegistrationParams& registration)
    : config_(parameters), lidar_registration_(registration), memory_(memory) { config_.validate(); lidar_registration_.validate(); }
std::vector<int> Relocalizer::spatial_candidates(const Node& query, const std::map<int, SE3f>& poses, const std::set<int>& excluded) const {
  if (!config_.spatial_enabled || !poses.count(query.id())) return {};
  const float radius = config_.max_loop_distance_m > 0 ? std::min(config_.spatial_radius_m, config_.max_loop_distance_m) : config_.spatial_radius_m;
  const auto facing = graph::findNearestPoses(query.id(), poses, radius, 0.f);
  struct Candidate {
    int id;
    float distance;
  };
  std::vector<Candidate> ranked;
  for (const auto& [id, pose] : facing) {
    const auto* node = memory_.getNode(id);
    if (id <= 0 || id == query.id() || memory_.isInSTM(id) || query.hasLink(id) || excluded.count(id)) continue;
    if (node ? node->getWeight() < 0 : !query.has_lidar()) continue;
    if (!node) {
      SE3f odom;
      int map = 0, weight = 0;
      std::string label;
      double stamp = 0;
      std::vector<float> velocity;

      if (!memory_.getNodeInfo(id, odom, map, weight, label, stamp, velocity, true) || weight < 0) continue;
    }
    ranked.push_back({id, poses.at(query.id()).translationSquaredDistance(pose)});
  }
  if (ranked.empty()) return {};
  std::set<int> reachable;
  if (config_.spatial_max_graph_hops) {
    // Use graph metadata, including cold intermediate nodes, without loading
    // their Gaussian payloads or solving a weighted path per candidate.
    std::vector<std::pair<int, unsigned int>> pending{{query.id(), 0}};
    reachable.insert(query.id());
    for (size_t i = 0; i < pending.size(); ++i) {
      const auto [id, depth] = pending[i];
      if (depth >= config_.spatial_max_graph_hops) continue;
      // Read committed adjacency from its owner; cold links need no cloud load.
      const auto links = id == query.id() ? query.getLinks() : memory_.getLinks(id, true);
      for (const auto& [neighbor, link] : links)
        if (link.isValid() && poses.count(neighbor) && neighbor > 0 && reachable.insert(neighbor).second)
          pending.emplace_back(neighbor, depth + 1);
    }
  }
  if (config_.spatial_max_graph_hops)
    ranked.erase(std::remove_if(ranked.begin(), ranked.end(), [&](const Candidate& c) { return !reachable.count(c.id); }), ranked.end());
  const auto count = std::min(ranked.size(), size_t(config_.spatial_max_candidates));
  std::partial_sort(ranked.begin(), ranked.begin() + count, ranked.end(), [](const Candidate& a, const Candidate& b) {
    if (a.distance != b.distance) return a.distance < b.distance;
    return a.id > b.id;
  });
  std::vector<int> result;
  result.reserve(count);
  for (size_t i = 0; i < count; ++i) result.push_back(ranked[i].id);
  return result;
}
lidar::Alignment Relocalizer::verify(int reference, int query, SE3f guess) const {
  double seconds = 0;
  memory_.reactivateNodes({reference, query}, 2, seconds);
  const auto* target = memory_.getNode(reference);
  const auto* source = memory_.getNode(query);
  if (!target || !source) {
    lidar::Alignment result;
    result.failure = "Missing LiDAR node";
    return result;
  }
  return lidar::align(target->lidar_cloud(), source->lidar_cloud(), guess, lidar_registration_);
}
}  // namespace sapphire
