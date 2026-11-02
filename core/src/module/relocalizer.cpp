#include "module/relocalizer.hpp"

#include "database/memory.hpp"
#include "database/driver.hpp"
#include "tools/timer.hpp"
#include "solver/relocalization.hpp"
#include <sstream>
#include <algorithm>
namespace sapphire {
Relocalizer::Relocalizer(Memory& memory, const RelocalizerParams& parameters, const RegistrationParams& registration)
    : config_(parameters), lidar_registration_(registration), memory_(memory) { config_.validate(); lidar_registration_.validate(); }
std::vector<int> Relocalizer::candidates(const Node& query, const std::map<int, SE3f>& poses) {
  if (!query.has_lidar() || !poses.count(query.id())) return {};

  // Session labels never establish a common datum. Use the same committed
  // connectivity as graph trial/commit, including cold nodes and landmarks.
  auto ids = memory_.getAllNodeIds();
  ids.insert(query.id());
  const auto connected = memory_.topograph().metric_component(query.id(), ids, [&](int id) {
    return id == query.id() ? query.getLinks() : memory_.getLinks(id, true);
  });
  std::set<int> reachable;
  if (config_.spatial_max_graph_hops) {
    std::vector<std::pair<int, unsigned int>> pending{{query.id(), 0}};
    reachable.insert(query.id());
    for (size_t i = 0; i < pending.size(); ++i) {
      const auto [id, depth] = pending[i];
      if (depth >= config_.spatial_max_graph_hops) continue;
      const auto links = id == query.id() ? query.getLinks() : memory_.getLinks(id, true);
      for (const auto& [neighbor, link] : links)
        if (link.isValid() && poses.count(neighbor) && neighbor > 0 && reachable.insert(neighbor).second)
          pending.emplace_back(neighbor, depth + 1);
    }
  }
  const auto signature = [&](int id) -> std::array<float, 64> {
    if (const auto found = signatures_.find(id); found != signatures_.end()) return found->second;
    const Node* node = id == query.id() ? &query : memory_.getNode(id);
    std::unique_ptr<Node> cold;
    if (!node && memory_.database()) { cold.reset(memory_.database()->loadNode(id)); node = cold.get(); }
    if (!node || !node->has_lidar()) return {};
    auto value = lidar::geometry_signature(*node->lidar_cloud(), config_.global_resolution_m);
    if (signatures_.size() >= config_.global_index_max_nodes) signatures_.erase(signatures_.begin());
    signatures_[id] = value;
    return value;
  };
  const float radius = config_.max_loop_distance_m > 0 ? std::min(config_.spatial_radius_m, config_.max_loop_distance_m) : config_.spatial_radius_m;
  struct Candidate { int id; bool local; float score; };
  std::vector<Candidate> ranked;
  std::optional<std::array<float, 64>> source_signature;
  for (const auto& [id, pose] : poses) {
    if (id <= 0 || id >= query.id() || query.hasLink(id)) continue;
    SE3f raw; int session, weight; std::string label; double stamp; std::vector<float> velocity;
    if (!memory_.getNodeInfo(id, raw, session, weight, label, stamp, velocity, true) || weight < 0) continue;
    // STM exclusion concerns adjacent observations in the current acquisition,
    // not independent older acquisitions that happen to remain resident.
    if (session == query.mapId() && memory_.isInSTM(id)) continue;
    const bool local = config_.spatial_enabled && connected.count(id) && pose.isValid() && poses.at(query.id()).isValid();
    float score = 0;
    if (local) {
      const float distance = poses.at(query.id()).translationSquaredDistance(pose);
      if (distance > radius * radius || (config_.spatial_max_graph_hops && !reachable.count(id))) continue;
      score = -distance;
    } else {
      if (!config_.global_enabled) continue;
      if (const auto db = memory_.database()) {
        const auto size = db->node_geometry_size(id);
        if (size && size->bytes() > config_.path_max_bytes) continue;
      }
      if (!source_signature) source_signature = signature(query.id());
      const auto target = signature(id);
      for (size_t i = 0; i < target.size(); ++i) score += (*source_signature)[i] * target[i];
    }
    ranked.push_back({id, local, score});
  }
  // Different units are never added or converted into a fake likelihood.
  std::sort(ranked.begin(), ranked.end(), [](const Candidate& a, const Candidate& b) {
    if (a.local != b.local) return a.local;
    if (a.score != b.score) return a.score > b.score;
    return a.id > b.id;
  });
  std::vector<int> result;
  for (const auto& candidate : ranked) {
    if (result.size() >= config_.spatial_max_candidates) break;
    result.push_back(candidate.id);
  }
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

std::vector<int> Relocalizer::path_neighbors(int id) const {
  std::vector<int> neighbors;
  for (const auto& [peer, edge] : memory_.getLinks(id, true))
    if (peer > 0 && peer != id && edge.isValid() && (edge.isOdometry() || edge.isRelocalization())) neighbors.push_back(peer);
  std::sort(neighbors.begin(), neighbors.end());
  neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  return neighbors;
}
std::vector<Relocalizer::Path> Relocalizer::paths(int anchor, int query, const std::map<int, SE3f>& poses) const {
  if (!poses.count(anchor)) return {};
  const auto admissible = [&](int id) { return id != query && poses.count(id) && !memory_.isInSTM(id); };
  auto adjacent = path_neighbors(anchor);
  adjacent.erase(std::remove_if(adjacent.begin(),adjacent.end(),[&](int id) { return !admissible(id); }),adjacent.end());
  std::vector<std::vector<int>> trails;
  if (config_.path_max_nodes == 1 || adjacent.empty()) trails.push_back({anchor});
  else if (config_.path_max_nodes == 2 && adjacent.size() > 2) {
    for (int peer : adjacent) if (trails.size() < config_.spatial_max_candidates) trails.push_back({anchor,peer});
  }
  else if (adjacent.size() == 1 || config_.path_max_nodes == 2) trails.push_back({anchor,adjacent.front()});
  else {
    // A degree-two chain has exactly one route through its anchor. Sliding or
    // reversing the same bounded segment is not a new path. Only graph branches
    // (including committed historical loops) produce alternative routes.
    for (size_t i=0;i<adjacent.size();++i) for (size_t j=i+1;j<adjacent.size();++j) {
      if (trails.size() < config_.spatial_max_candidates) trails.push_back({adjacent[i],anchor,adjacent[j]});
    }
  }
  for (unsigned iteration=0;iteration<config_.path_max_nodes;++iteration) {
    bool grew=false;
    std::vector<std::vector<int>> next;
    std::set<std::vector<int>> unique;
    const auto append = [&](std::vector<int> path) {
      auto identity=path; std::sort(identity.begin(),identity.end());
      if (next.size()<config_.spatial_max_candidates && unique.insert(identity).second) next.push_back(std::move(path));
    };
    for (const auto& trail:trails) {
      if (trail.size()>=config_.path_max_nodes) { append(trail); continue; }
      const size_t left=std::find(trail.begin(),trail.end(),anchor)-trail.begin(), right=trail.size()-1-left;
      bool extended=false;
      // Grow the shorter arm first; a chain still produces only one segment.
      for (bool front : {left<=right, left>right}) {
        for (int id:path_neighbors(front?trail.front():trail.back())) {
          if (!admissible(id) || std::find(trail.begin(),trail.end(),id)!=trail.end()) continue;
          auto candidate=trail;
          if(front)candidate.insert(candidate.begin(),id);else candidate.push_back(id);
          append(std::move(candidate)); extended=grew=true;
        }
        if(extended)break;
      }
      if(!extended)append(trail);
    }
    trails.swap(next);
    if(!grew)break;
  }
  std::vector<Path> result;
  for (auto& trail : trails) result.push_back({anchor, std::move(trail)});
  return result;
}
GaussianCloudPtr Relocalizer::load_path(const Path& requested, int query, const std::map<int, SE3f>& poses,
                                         const std::set<std::vector<int>>& attempted, Verification& result) const {
  result.path = requested;
  auto& path = result.path;
  tools::Timer timer;
  if (path.nodes.empty() || !poses.count(query) || !poses.count(path.anchor)) {
    result.alignment.failure = "Missing query or path anchor pose"; return {};
  }
  std::map<int, GaussianCloudSize> sizes;
  for (int id : path.nodes) {
    if (id == query || sizes.count(id) || !poses.count(id)) {
      result.alignment.failure = "Path includes query, duplicate or missing pose"; return {};
    }
    std::optional<GaussianCloudSize> size;
    if (const auto* node = memory_.getNode(id))
      size = node->has_lidar() ? gaussian_cloud_size(*node->lidar_cloud()) : GaussianCloudSize{};
    else if (memory_.database()) size = memory_.database()->node_geometry_size(id);
    if (!size || !size->points) { result.alignment.failure = "Missing path geometry"; return {}; }
    if (size->bytes() > std::numeric_limits<std::size_t>::max() - result.requested_bytes)
      throw std::length_error("Path Gaussian payload size overflow");
    sizes.emplace(id, *size);
    result.requested_bytes += size->bytes();
  }
  if (!sizes.count(path.anchor)) { result.alignment.failure = "Path does not contain anchor"; return {}; }
  result.target_bytes = result.requested_bytes;
  while (result.target_bytes > config_.path_max_bytes && path.nodes.size() > 1) {
    const auto left = std::size_t(std::find(path.nodes.begin(), path.nodes.end(), path.anchor) - path.nodes.begin());
    const auto right = path.nodes.size() - 1 - left;
    const bool front = left > right || (left == right && sizes.at(path.nodes.front()).bytes() > sizes.at(path.nodes.back()).bytes());
    const auto removed = front ? path.nodes.begin() : std::prev(path.nodes.end());
    result.target_bytes -= sizes.at(*removed).bytes();
    path.nodes.erase(removed);
  }
  auto identity = path.nodes;
  std::sort(identity.begin(), identity.end());
  if (attempted.count(identity)) { result.duplicate = true; return {}; }
  if (result.target_bytes > config_.path_max_bytes) {
    result.resource_failure = "Anchor Gaussian payload " + std::to_string(result.target_bytes) +
        " exceeds path target budget " + std::to_string(config_.path_max_bytes);
    result.load_ms = timer.elapsed_seconds()*1000;
    return {};  // In particular, do not activate/decode this cold anchor.
  }
  double elapsed = 0;
  memory_.reactivateNodes({path.anchor, query}, 2, elapsed);
  const auto* source = memory_.getNode(query);
  if (!source || !source->has_lidar()) { result.alignment.failure = "Missing query geometry"; return {}; }
  result.query_points = source->lidar_cloud()->size();
  result.load_ms = timer.lap_seconds()*1000;
  GaussianCloudPtr target;
  std::shared_ptr<GaussianCloud> assembled;
  if (path.nodes.size() > 1) {
    assembled = std::make_shared<GaussianCloud>();
    std::size_t points = 0;
    for (int id : path.nodes) points += sizes.at(id).points;
    assembled->reserve(points);
  }
  result.assemble_ms = timer.lap_seconds()*1000;
  for (int id : path.nodes) {
    timer.reset();
    std::unique_ptr<Node> owned;
    const Node* node = memory_.getNode(id);
    if (!node && memory_.database()) { owned.reset(memory_.database()->loadNode(id)); node = owned.get(); }
    if (!node || !node->has_lidar()) throw std::runtime_error("Admitted path geometry disappeared");
    const auto actual = gaussian_cloud_size(*node->lidar_cloud());
    if (actual.points != sizes.at(id).points || actual.supports != sizes.at(id).supports)
      throw std::runtime_error("Path geometry changed after payload admission");
    result.load_ms += timer.lap_seconds()*1000;
    if (!assembled) target = node->lidar_cloud();
    else {
      const auto transform = poses.at(path.anchor).inverse().compose(poses.at(id));
      for (const auto& point : *node->lidar_cloud())
        assembled->push_back(point.transformed(transform.rotationMatrix().cast<double>(), transform.translationVector().cast<double>()));
    }
    result.assemble_ms += timer.lap_seconds()*1000;
    // A cold support Node is released here, not retained alongside every other
    // support until registration and graph commit have finished.
  }
  if (assembled) target = std::move(assembled);
  result.target_points = target->size();
  return target;
}
Relocalizer::Verification Relocalizer::verify(const Path& requested, int query, const std::map<int, SE3f>& poses,
                                             const std::set<std::vector<int>>& attempted) const {
  return evaluate(prepare(requested,query,poses,attempted),config_,lidar_registration_);
}
Relocalizer::Input Relocalizer::prepare(const Path& requested, int query, const std::map<int, SE3f>& poses,
                                       const std::set<std::vector<int>>& attempted) const {
  Input input;
  input.target=load_path(requested,query,poses,attempted,input.result);
  if(!input.target)return input;
  input.source=memory_.getNode(query)->lidar_cloud();
  const int anchor=input.result.path.anchor;
  const auto component=memory_.metric_component(anchor);
  input.result.component=*component.begin();
  input.result.datum_from_anchor=poses.at(anchor);
  if(config_.spatial_enabled && poses.at(query).isValid() && poses.at(anchor).isValid() &&
     memory_.metric_component(query).count(anchor))
    input.guess=poses.at(anchor).inverse().compose(poses.at(query));
  return input;
}
Relocalizer::Verification Relocalizer::evaluate(Input input, const RelocalizerParams& config_, const RegistrationParams& lidar_registration_) {
  auto result=std::move(input.result);
  const auto target=std::move(input.target),source=std::move(input.source);
  if(!target || !source)return result;
  std::vector<SE3f> seeds;
  const bool use_graph_pose=!input.guess.isNull();
  if(use_graph_pose)seeds.push_back(input.guess);
  else if(config_.global_enabled) {
    std::string failure;
    seeds=lidar::initial_poses(*target,*source,config_,failure);
    if(failure.find("budget")!=std::string::npos)result.resource_failure=failure;
    else result.alignment.failure=failure;
  }
  if(seeds.empty() && result.resource_failure.empty())result.alignment.failure="No common datum or automatic initial pose";
  std::ostringstream trace;trace<<(use_graph_pose?"graph_pose_seed;":"automatic_pagor;");std::vector<lidar::Alignment> accepted;
  auto& alternatives=result.alternatives;
  for(size_t i=0;i<seeds.size();++i) {
    auto aligned=lidar::align(target,source,seeds[i],lidar_registration_);
    trace << "seed=" << i << ":support=" << aligned.inliers << '/' << source->size()
          << ":fitness=" << aligned.fitness << ":iterations=" << aligned.iterations << ":" << aligned.failure;
    if(!aligned.fitted_pose.isNull()) {
      // Acceptance precision is not evidence that another converged,
      // overlap-supported place is impossible. Keep it as a competing explanation.
      alternatives.emplace_back(aligned.fitted_pose,aligned.fitness);
      trace<<":xyz="<<aligned.fitted_pose.translationVector().transpose();
    }
    trace<<";";
    if(aligned.T_target_query.isNull()) {if(accepted.empty())result.alignment=std::move(aligned);}
    else accepted.push_back(std::move(aligned));
  }
  result.initialization_trace=trace.str();
  if(!accepted.empty()) {
    std::sort(accepted.begin(),accepted.end(),[](const auto&a,const auto&b){return a.fitness<b.fitness;});
    result.alignment=std::move(accepted.front());
  }
  return result;
}
std::optional<std::size_t> Relocalizer::select(const std::vector<Verification>& results,
                                              const RegistrationParams& config, std::string& reason) {
  std::optional<std::size_t> best;
  for(std::size_t i=0;i<results.size();++i) {
    if(!results[i].resource_failure.empty()) { reason="incomplete_geometry_search";return {}; }
    if(!results[i].alignment.T_target_query.isNull() &&
       (!best || results[i].alignment.fitness<results[*best].alignment.fitness))best=i;
  }
  if(!best) {reason="no_verified_geometry";return {};}
  std::vector<std::pair<int,SE3f>> interpretations;
  for(const auto& result:results) {
    auto alternatives=result.alternatives;
    if(!result.alignment.T_target_query.isNull())
      alternatives.emplace_back(result.alignment.T_target_query,result.alignment.fitness);
    for(const auto& [relative,fitness]:alternatives) {
      if(!std::isfinite(fitness))continue;
      const auto pose=result.datum_from_anchor.compose(relative);
      // Pairwise agreement prevents a middle candidate from hiding two distinct
      // end solutions; acceptance cannot depend on which one has the best score.
      for(const auto& [component,other]:interpretations) {
        const auto delta=other.inverse().compose(pose);
        if(result.component!=component || delta.translationNorm()>config.lidar_max_correspondence_m ||
           Eigen::AngleAxisf(delta.rotationMatrix()).angle()>2*config.lidar_rotation_sigma_rad) {
          reason="ambiguous_geometry";return {};
        }
      }
      interpretations.emplace_back(result.component,pose);
    }
  }
  reason="verified_geometry";return best;
}
}  // namespace sapphire
