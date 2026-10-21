#include "backend/sensor/lidar/registration.hpp"
#include <ctime>
#include <Eigen/Cholesky>
#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/topograph.hpp>
#include "backend/solver/ba_solver.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <backend/maplayer/node.hpp>
#include <backend/memory.hpp>
#include <backend/system/action/relocalizer.hpp>
#include <backend/registration/retrieval/fbow.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <stdexcept>
#include <backend/database/driver.hpp>
#include <backend/tools/stats.hpp>
#include <backend/maplayer/topograph.hpp>

namespace sapphire::backend {

namespace {
std::shared_ptr<const bow::FeatureBlock> restore_query_measurements(const Memory& memory, const Node& query,
    const camera::Perspective& camera, const std::vector<double>& variances, std::string& reason) {
  const int owner = memory.topograph().submap_owner(query.id());
  const auto* resident = memory.getNode(owner);
  const auto stored = resident && resident->has_observation_history() ? featmap::NodeObservations{}
      : featmap::NodeObservations::decode(memory.load_observation_archive(owner));
  const auto& history = resident && resident->has_observation_history() ? resident->observation_history() : stored;
  const auto bank = memory.map_features_for(owner);
  if (!bank) { reason="Cold query: missing owning submap bank"; return {}; }
  std::map<uint32_t, std::pair<const camera::StereoObservation*, const featmap::Appearance*>> measurements;
  if (history.reference_node || !history.observations.empty()) {
    for (const auto& o : history.observations)
      if (o.node == query.id() && o.feature_id != UINT32_MAX)
        measurements.emplace(o.feature_id, std::make_pair(&o.stereo, &o.appearance));
  } else if (owner == query.id()) {
    // A never-refreshed single capture keeps its original stereo/appearance in
    // each landmark. A refreshed submap has an explicit observation archive.
    for (const auto& p : bank->points())
      if (p.id && p.id <= bow::FeatureBlock::max_features && p.stereo.valid())
        measurements.emplace(uint32_t(p.id - 1), std::make_pair(&p.stereo, &p.appearance));
  }
  if (measurements.empty()) { reason="Cold query: no retained capture measurements with feature identity"; return {}; }
  if (measurements.size() > bow::FeatureBlock::max_features) { reason="Cold query: observation count exceeds frame capacity"; return {}; }
  std::vector<cv::KeyPoint> pixels;
  std::vector<cv::Point3f> points;
  std::vector<uint32_t> tree_nodes;
  std::vector<int> words;
  pixels.reserve(measurements.size());points.reserve(measurements.size());
  if (bank->vocabulary_hash()) {
    tree_nodes.reserve(measurements.size());
    words.reserve(measurements.size());
  }
  cv::Mat descriptors(int(measurements.size()), bank->descriptor_dimensions(), bank->descriptor_model_id() ? CV_32F : CV_8U);
  const std::array<float,5> calibration{float(camera.fx()),float(camera.fy()),float(camera.cx()),float(camera.cy()),float(camera.focalXBaseline())};
  for (const auto& [feature, refs] : measurements) {
    const auto& o = *refs.first;const auto& appearance = *refs.second;
    if (!o.valid() || o.calibration() != calibration ||
        !o.anchor_from_camera().rotationMatrix().isApprox(camera.cameraToBase().rotationMatrix(), 1e-5f) ||
        o.anchor_from_camera().translationDistance(camera.cameraToBase()) > 1e-5f || !camera.isInImage(o.pixel().x,o.pixel().z)) { reason="Cold query: incompatible observation camera or pixel layout"; return {}; }
    int octave = -1;
    for (size_t i=0;i<variances.size();++i)
      if (std::abs(double(o.sigma())*o.sigma() - variances[i]) <= variances[i]*2e-6) { octave=int(i);break; }
    // Do not silently reinterpret archived pixel noise after a config change.
    if (octave < 0) { reason="Cold query: archived pixel noise does not match configured pyramid"; return {}; }
    // ORB angle/response/size are not consumed by geometric registration. Keep
    // original pixel identity in class_id; FeatureBlock rows are temporary.
    pixels.emplace_back(o.pixel().x,o.pixel().z,1.f,-1.f,0.f,octave,int(feature));
    cv::Point3f point(NAN,NAN,NAN);
    if (o.pixel().y >= 0 && o.pixel().x > o.pixel().y && calibration[4] > 0) {
      const float z=calibration[4]/(o.pixel().x-o.pixel().y);
      const auto body=o.anchor_from_camera().transformPoint({(o.pixel().x-calibration[2])*z/calibration[0],(o.pixel().z-calibration[3])*z/calibration[1],z});
      point={body.x(),body.y(),body.z()};
    }
    points.push_back(point);
    if (bank->vocabulary_hash()) {
      tree_nodes.push_back(appearance.tree_node);
      words.push_back(appearance.word_id);
    }
    if(appearance.descriptor.model_id()!=bank->descriptor_model_id()) {reason="Cold query: descriptor model mismatch";return {};}
    appearance.descriptor.copy_to(descriptors,int(pixels.size()-1));
  }
  if (bank->descriptor_model_id())
    return bow::FeatureBlock::create_learned_owned(std::move(pixels), std::move(points), std::move(descriptors), bank->descriptor_model_id());
  return bank->vocabulary_hash() ? bow::FeatureBlock::create(bank->vocabulary_hash(),tree_nodes,words,pixels,points,descriptors)
      : bow::FeatureBlock::create_raw_owned(std::move(pixels),std::move(points),std::move(descriptors));
}

}  // namespace

std::vector<int> Relocalizer::spatial_candidates(const Node& query, const std::map<int, StateGroupf>& poses,
                                                const Relocalization& recognition, const std::set<int>& excluded,
                                                size_t* insufficient_support) const {
  if (insufficient_support) *insufficient_support = 0;
  if (!config_.spatial_enabled || !poses.count(query.id())) return {};
  const float radius = config_.max_loop_distance_m > 0
                           ? std::min(config_.spatial_radius_m, config_.max_loop_distance_m) : config_.spatial_radius_m;
  const auto facing = graph::findNearestPoses(query.id(), poses, radius, (query.has_lidar() ? 0.f : config_.spatial_max_view_angle_deg * M_PI / 180.f));
  struct Candidate { int id; float score, distance; };
  std::vector<Candidate> ranked;
  for (const auto& [id, pose] : facing) {
    const auto* node = memory_.getNode(id);
    if (id <= 0 || id == query.id() || memory_.isInSTM(id) || query.hasLink(id) || excluded.count(id) ||
        memory_.topograph().is_submap_member(id)) continue;
    if (node ? node->getWeight()<0 : !query.has_lidar()) continue;
    if (!node) {
      StateGroupf odom; int map=0,weight=0;std::string label;double stamp=0;std::vector<float> velocity;EnvSensors sensors;
      if(!memory_.getNodeInfo(id,odom,map,weight,label,stamp,velocity,sensors,true) || weight<0) continue;
    }
    const auto found = std::lower_bound(recognition.ids.begin(), recognition.ids.end(), id);
    const float score = found != recognition.ids.end() && *found == id
                            ? recognition.scores[size_t(found - recognition.ids.begin())] : 0.f;
    ranked.push_back({id, score, poses.at(query.id()).translationSquaredDistance(pose)});
  }
  if (ranked.empty()) return {};
  std::set<int> reachable;
  if (config_.spatial_max_graph_hops) {
    // Use graph metadata, including cold intermediate nodes, without loading
    // their feature banks or solving a weighted path per candidate.
    std::unordered_map<int, std::vector<int>> adjacency;
    for (const auto& [id, link] : memory_.topograph().constraints())
      if (link.from() > 0 && link.to() > 0 && poses.count(link.from()) && poses.count(link.to())) {
        adjacency[link.from()].push_back(link.to());
        adjacency[link.to()].push_back(link.from());
      }
    std::vector<std::pair<int, unsigned int>> pending{{query.id(), 0}};
    reachable.insert(query.id());
    for (size_t i = 0; i < pending.size(); ++i) {
      const auto [id, depth] = pending[i];
      if (depth >= config_.spatial_max_graph_hops) continue;
      const auto found = adjacency.find(id);
      if (found == adjacency.end()) continue;
      for (int neighbor : found->second)
        if (reachable.insert(neighbor).second)
          pending.emplace_back(neighbor, depth + 1);
    }
  }
  if (config_.spatial_max_graph_hops)
    ranked.erase(std::remove_if(ranked.begin(), ranked.end(), [&](const Candidate& c) { return !reachable.count(c.id); }), ranked.end());
  const auto query_features = query.loop_features();
  if (!query.has_lidar() && !config_.invert_registration && query_features && !query_features->is_raw()) {
    // A matching tree group cannot supply more independent pairs than its
    // smaller side. This is an upper bound, not a heuristic score threshold.
    // Inspect only banks already resident; never fetch a bank to screen it.
    ranked.erase(std::remove_if(ranked.begin(), ranked.end(), [&](const Candidate& candidate) {
      const auto* node = memory_.getNode(candidate.id);
      const auto map = node->map_features();
      const auto frame = node->loop_features();
      const auto* groups = map ? &map->groups() : (!node->has_map_features() && frame ? &frame->groups : nullptr);
      const auto hash = map ? map->vocabulary_hash() : (frame ? frame->vocabulary_hash : 0);
      if (!groups || hash != query_features->vocabulary_hash) return false;
      size_t a = 0, b = 0, upper_bound = 0;
      while (a < groups->size() && b < query_features->groups.size()) {
        const auto& left = groups->at(a);
        const auto& right = query_features->groups[b];
        if (left.node_id < right.node_id) ++a;
        else if (right.node_id < left.node_id) ++b;
        else {
          upper_bound += std::min(left.end - left.begin, right.end - right.begin);
          if (upper_bound >= size_t(geometry_.getMinVisualCorrespondences())) return false;
          ++a;
          ++b;
        }
      }
      if (insufficient_support) ++*insufficient_support;
      return true;
    }), ranked.end());
  }
  const auto count = std::min(ranked.size(), size_t(config_.spatial_max_candidates));
  std::partial_sort(ranked.begin(), ranked.begin() + count, ranked.end(), [](const Candidate& a, const Candidate& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.distance != b.distance) return a.distance < b.distance;
    return a.id > b.id;
  });
  std::vector<int> result;
  result.reserve(count);
  for (size_t i = 0; i < count; ++i) result.push_back(ranked[i].id);
  return result;
}

Relocalizer::Relocalizer(Memory& memory, const ParametersMap& parameters)
    : Relocalizer(memory, RelocalizerParams{}.withOverrides(parameters), RegistrationParams{}.withOverrides(parameters), FeatureParams{}.withOverrides(parameters), PreprocessParams{}.withOverrides(parameters), MemoryParams{}.withOverrides(parameters).protected_local_ratio, TopographParams{}.withOverrides(parameters).metric_enabled) {}
Relocalizer::Relocalizer(Memory& memory, const RelocalizerParams& retrieval, const RegistrationParams& registration, const FeatureParams& feature, const PreprocessParams& preprocess, float protected_local_ratio, bool metric_enabled, std::shared_ptr<feature::RknnModels> models)
    : config_(retrieval), lidar_registration_(registration), visits_(retrieval.visits_max_entries, retrieval.visits_max_gap_s),
      memory_(memory),
      _localImmunizationRatio(protected_local_ratio),
      _rgbdSlamMode(metric_enabled), geometry_(registration, feature, preprocess, models) {
  config_.validate();
  if (models) models->validate(feature);
}
Relocalizer::~Relocalizer() = default;
void Relocalizer::reset() {
  visits_.break_sequence();
  score_positions_.clear();
}
size_t Relocalizer::memory_size() const {
  return sizeof(*this) - sizeof(geometry_) - sizeof(visits_) + geometry_.memory_size() + visits_.memory_size() + score_positions_.bucket_count() * sizeof(void*) +
         score_positions_.size() * (sizeof(int) + sizeof(size_t) + sizeof(void*));
}
void Relocalizer::restore_visits() {
  if (!config_.visits_enabled || !memory_.database() || !memory_.topograph_params().use_previous_map) return;
  try {
    visits_.restore(memory_.database()->load_visit_history());
  } catch (const std::exception& error) {
    visits_.restore({});
    EXT_LOG_WARN("[visits] Ignoring unusable advisory history: {}", error.what());
  }
}
void Relocalizer::save_visits() const {
  if (!config_.visits_enabled || !memory_.database()) return;
  try {
    memory_.database()->save_visit_history(visits_.encode());
  } catch (const std::exception& error) {
    // This optional hint must not turn an otherwise valid map save into a failure.
    EXT_LOG_WARN("[visits] Could not save advisory history: {}", error.what());
  }
}
void Relocalizer::record_visit(int query, int session, double stamp, int mapped_place, int committed_loop) {
  if (!config_.visits_enabled) return;
  const int place = memory_.topograph().submap_owner(committed_loop > 0 ? committed_loop : mapped_place);
  visits_.record(session, query, stamp, place, committed_loop > 0);
}
std::vector<int> Relocalizer::visit_predictions(int session, double stamp) const {
  if (!config_.visits_enabled) return {};
  return visits_.predict(session, stamp, [&](int id) {
    const int owner = memory_.topograph().submap_owner(id);
    return owner > 0 && memory_.retrieval_mass().count(owner) ? owner : 0;
  }, std::max(size_t(config_.max_retrieved), memory_.feature_cache_capacity()));
}
void Relocalizer::prioritize_prefetch(int session, double stamp, std::vector<int>& predicted) const {
  if (!config_.visits_enabled || !memory_.predictive_prefetch_enabled()) return;
  const auto visits = visit_predictions(session, stamp);
  if (visits.empty()) return;
  std::vector<int> merged;
  const auto append = [&](int id) {
    if (id > 0 && !memory_.isInSTM(id) && merged.size() < memory_.feature_cache_capacity() &&
        std::find(merged.begin(), merged.end(), id) == merged.end()) merged.push_back(id);
  };
  if (!predicted.empty()) append(predicted.front());  // Preserve the existing predictor's leader.
  for (int id : visits) append(id);
  for (int id : predicted) append(id);
  predicted = std::move(merged);
}
std::vector<int> Relocalizer::candidates(const Node& query, size_t limit, bool cold_only) const {
  if (memory_.uses_scene_index()) return memory_.global_candidates(query, limit, nullptr, cold_only);
  if (!limit || !query.has_natural_features()) return {};
  std::set<int> matched;
  for (const auto& word : query.retrieval_words()) {
    const auto* references = memory_.fbow_index()->getReferences(word.word_id);
    if (!references) continue;
    for (const auto& [id, count] : *references)
      if (id != query.id() && !memory_.getStMem().count(id) && memory_.retrieval_mass().count(id) && (!cold_only || !memory_.getNode(id))) matched.insert(id);
  }
  std::vector<int> ids(matched.begin(), matched.end());
  if (ids.empty()) return {};
  std::vector<float> scores;
  score(&query, ids, scores);
  std::vector<std::pair<float, int>> ranked;
  for (size_t i = 0; i < ids.size(); ++i)
    if (scores[i] > 0) ranked.emplace_back(scores[i], ids[i]);
  const size_t count = std::min(limit, ranked.size());
  std::partial_sort(ranked.begin(), ranked.begin() + count, ranked.end(),
                    [](const auto& a, const auto& b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
  std::vector<int> result;
  for (size_t i = 0; i < count; ++i) result.push_back(ranked[i].second);
  return result;
}

void Relocalizer::score(const Node* node, const std::vector<int>& ids, std::vector<float>& scores) const {
  tools::Timer timer;
  timer.reset();
  scores.clear();

  if (!node) {
    EXT_LOG_ERROR("The node is null");
    return;
  } else if (ids.empty()) {
    EXT_LOG_DEBUG("[retrieval] No candidate nodes; skipping likelihood evaluation");
    return;
  }

  scores.assign(ids.size(), 0.0f);
  if (memory_.uses_scene_index()) {
    scores = memory_.global_scores(*node, ids);
    return;
  }
  if (!node->has_natural_features()) return;
  score_positions_.clear();
  score_positions_.reserve(ids.size());
  for (std::size_t index = 0; index < ids.size(); ++index) {
    EXT_ASSERT(index == 0U || ids[index - 1U] < ids[index]);
    score_positions_.emplace(ids[index], index);
  }

  std::vector<bow::WordCount> query_words;
  if (node->has_map_features() && node->loop_features()) {
    auto ids = node->loop_features()->word_ids;
    std::sort(ids.begin(), ids.end());
    for (const int id : ids)
      if (id > 0 && (query_words.empty() || query_words.back().word_id != id)) query_words.push_back({id, 1});
  }
  const auto& wordIds = node->has_map_features() && node->loop_features() ? query_words : node->retrieval_words();
  const auto mass_for = [&](int id) {
    const auto it = memory_.retrieval_mass().find(id);
    return it == memory_.retrieval_mass().end() ? 0 : int(it->second);
  };
  const float nodeCount = memory_.retrieval_mass().size();
  if (nodeCount) {
    for (auto wordId = wordIds.begin(); wordId != wordIds.end(); ++wordId) {
      if (wordId->word_id <= 0) {
        continue;
      }
      const FbowIndex::References* referencesPointer = memory_.fbow_index()->getReferences(wordId->word_id);
      if (!referencesPointer) continue;
      const FbowIndex::References& references = *referencesPointer;
      const float referencedNodeCount = references.size();
      if (!referencedNodeCount) {
        continue;
      }
      const float inverseDocumentFrequency = log10(nodeCount / referencedNodeCount);
      if (!inverseDocumentFrequency) {
        continue;
      }
      for (std::map<int, int>::const_iterator reference = references.begin(); reference != references.end(); ++reference) {
        const auto score = score_positions_.find(reference->first);
        if (score != score_positions_.end()) {
          const float wordCount = reference->second;
          const float totalWords = mass_for(reference->first);
          if (totalWords != 0.0f) {
            scores[score->second] += (wordCount * inverseDocumentFrequency) / totalWords;
          }
        }
      }
    }
  }

  EXT_LOG_DEBUG("compute likelihood (tf-idf) {:.6f} s", timer.lap_seconds());
}

// compute transform fromId -> toId
Registration Relocalizer::verify(int fromId, int toId, StateGroupf guess, RegistrationTrace* trace, std::optional<uint64_t> pnp_seed) const {
  const auto* source = memory_.getNode(fromId);
  const auto* target = memory_.getNode(toId);
  if (source && target) return verify(*source, *target, guess, trace, pnp_seed);
  RegistrationTrace local_trace;
  if (!trace) trace = &local_trace;
  *trace = {};
  Registration result;
  result.failure = fmt::format("Did not find nodes {:d} and/or {:d}", fromId, toId);
  EXT_LOG_WARN("{}", result.failure);
  return result;
}

Registration Relocalizer::verify(const Node& source, const Node& target, StateGroupf guess, RegistrationTrace* trace, std::optional<uint64_t> pnp_seed) const {
  if (trace) *trace = {};
  Registration result;
  if (source.has_lidar() && target.has_lidar() && (!source.has_natural_features() || !target.has_natural_features())) {
    if (guess.isNull()) {
      const auto& poses=memory_.topograph().poses();
      const auto a=poses.find(source.id()), b=poses.find(target.id());
      if (a==poses.end() || b==poses.end()) { result.failure="LiDAR nodes have no common graph-frame prior"; return result; }
      guess=a->second.inverse().compose(b->second);
    }
    const auto alignment=lidar::align(source.lidar_cloud(),target.lidar_cloud(),guess,lidar_registration_);
    result.pose=alignment.T_target_query; result.covariance=alignment.covariance; result.failure=alignment.failure;
    if (trace) { trace->match_count=int(target.lidar_cloud()->size()); trace->inlier_ratio=float(alignment.inliers)/trace->match_count; }
    EXT_LOG_INFO("[lidar_loop] reference={} query={} accepted={} inliers={} fitness={} failure={}",
                 source.id(), target.id(), !result.pose.isNull(), alignment.inliers, alignment.fitness, result.failure);
    return result;
  }
  // One submap source and one real query frame. Cold banks live only for this verification.
  const auto& map_node = config_.invert_registration ? target : source;
  const auto& query_node = config_.invert_registration ? source : target;
  if (memory_.topograph().is_submap_member(map_node.id())) {
    result.failure = "Historical boundary keyframe is not a canonical retrieval submap";
    return result;
  }
  const auto requested_direction = [&](Registration verified) {
    EXT_LOG_INFO("[loop_verification] reference={} query={} accepted={} source={} natural_inliers={} failure={}",
        map_node.id(),query_node.id(),!verified.pose.isNull(),"natural",verified.inliers.size(),verified.failure);
    if (trace && trace->stereo_bundle_observations > 0) {
      EXT_LOG_INFO("[stereo_bundle] reference={} query={} available={} used={} inliers={} ms={:.3f} rejection={}", map_node.id(),
                   query_node.id(), trace->stereo_bundle_observations, trace->stereo_bundle_refined, verified.inliers.size(),
                   trace->stereo_bundle_ms, verified.failure);
    }
    if (config_.invert_registration && !verified.pose.isNull()) {
      if (verified.covariance) {
        const auto adjoint=StateGroupd(verified.pose).adjoint();
        verified.covariance=(adjoint * *verified.covariance * adjoint.transpose()).eval();
      }
      verified.pose=verified.pose.inverse();
    }
    if (source.has_lidar() && target.has_lidar()) {
      auto initial=verified.pose.isNull() ? guess : verified.pose;
      if(initial.isNull()) {
        const auto& poses=memory_.topograph().poses();
        auto a=poses.find(source.id()),b=poses.find(target.id());
        if(a!=poses.end() && b!=poses.end()) initial=a->second.inverse().compose(b->second);
      }
      const auto alignment=lidar::align(source.lidar_cloud(),target.lidar_cloud(),initial,lidar_registration_);
      verified.pose=alignment.T_target_query;verified.covariance=alignment.covariance;verified.failure=alignment.failure;
    }
    return verified;
  };
  const auto* cameras = &query_node.visualFrame().perspectives();
  std::vector<camera::Perspective> loaded_cameras;
  if (cameras->empty()) {
    memory_.load_calibration(query_node.id(), loaded_cameras);
    cameras = &loaded_cameras;
  }
  auto query = query_node.loop_features();
  if (!query && !query_node.has_map_features()) query = memory_.frame_features_for(query_node);
  const bool cold_query = !query && (query_node.has_map_features() || memory_.topograph().is_submap_member(query_node.id()));
  std::string cold_query_error;
  if (cold_query) {
    if (cameras->size() == 1) query = restore_query_measurements(memory_,query_node,cameras->front(),geometry_.pixelVariances(),cold_query_error);
    else cold_query_error="Cold query: requires the original single feature camera";
  }
  auto map = map_node.map_features();
  auto frame = map_node.loop_features();
  if (map_node.has_map_features()) {
    map = memory_.map_features_for(map_node.id(), &map_node);
  } else if (!frame)
    frame = memory_.frame_features_for(map_node);
  const auto* vocabulary = memory_.fbow_index();
  const uint64_t hash = vocabulary ? vocabulary->getFBoWVocabularyHash() : 0;
  if (!query || (map_node.has_map_features() ? !map : !frame) ||
      (vocabulary && (query->vocabulary_hash != hash || (map ? map->vocabulary_hash() != hash : frame->vocabulary_hash != hash)))) {
    result.failure = cold_query_error.empty() ? "Missing or incompatible submap/query features" : cold_query_error;
    return requested_direction(std::move(result));
  }
  const auto initial = config_.invert_registration && !guess.isNull() ? guess.inverse() : guess;
  result = map_node.has_map_features() ? geometry_.verify_submap(map.get(), query.get(), *cameras, initial, trace, pnp_seed)
                                               : geometry_.verify_features(frame.get(), query.get(), *cameras, initial, trace, pnp_seed);
  return requested_direction(std::move(result));
}

Relocalization Relocalizer::recognize(const Node& query,
                                     PlaceHypothesis previous, const std::map<int, StateGroupf>& poses) {
  Relocalization result;
  tools::Timer timer;
  const auto* node = &query;
  const bool scene = memory_.uses_scene_index();
  result.visit_predictions = visit_predictions(query.mapId(), query.getStamp());
  const bool has_appearance = scene ? !query.featmap().global_descriptors().empty() : node->has_natural_features();

  // Admission owns keyframe selection; recall every admitted visual node.
  if (has_appearance && node->getWeight() >= 0) {
    // Search global lightweight postings before the resident-only Bayes stage.
    // Only ranked cold candidates enter WM; geometric verification is unchanged.
    if (config_.max_retrieved && !memory_.allNodesInWM()) {
      // Optional visit history only reorders eligible cold references before
      // the existing I/O limit. It must not be clipped by a verification quota.
      const size_t recall_count = result.visit_predictions.empty() ? size_t(config_.max_retrieved) : memory_.retrieval_mass().size();
      auto ranked = candidates(*node, recall_count, true);
      result.visits_reordered = VisitHistory::reorder(ranked, result.visit_predictions);
      if (result.visits_reordered)
        EXT_LOG_DEBUG("[visits] query={} reordered cold-load candidates; appearance leader={}", query.id(), ranked.front());
      std::list<int> cold;
      for (int id : ranked) if (!memory_.getNode(id)) cold.push_back(id);
      if (!cold.empty()) {
        double load_time = 0;
        const auto loaded = memory_.reactivateNodes(cold, config_.max_retrieved, load_time);
        result.loaded.insert(loaded.begin(), loaded.end());
        result.protected_ids.insert(loaded.begin(), loaded.end());
        result.database_seconds += load_time;
      }
      for (int id : ranked) memory_.updateAge(id);
    }
    // If the working memory is empty, don't do the detection. It happens when it
    // is the first time the detector is started (there needs some images to
    // fill the short-time memory before a node is added to the working memory).
    if (memory_.getWorkingMemSize(true)) {
      //============================================================
      // Likelihood computation
      // Get the likelihood of the new node
      // with all images contained in the working memory + reactivated.
      //============================================================

      for (std::map<int, double>::const_iterator iter = memory_.getWorkingMem().begin(); iter != memory_.getWorkingMem().end(); ++iter) {
        if (iter->first > 0) {
          const Node* s = memory_.getNode(iter->first);
          EXT_ASSERT(s != 0);
          if (s->getWeight() != -1 && (!scene || (!memory_.topograph().is_submap_member(s->id()) && !query.hasLink(s->id()) && !s->featmap().global_descriptors().empty())))  // eligible references
          {
            result.ids.push_back(iter->first);
          }
        } else {
          // virtual node should be added
          result.ids.push_back(iter->first);
        }
      }

      // Score the final resident set once, after cold loading has updated IDF.
      score(node, result.ids, result.scores);
      if (scene) {
        result.semantic_seconds = timer.lap_seconds();
        result.raw_scene_scores = result.scores;
        // Cosine is appearance evidence, not a probability. The shared
        // likelihood adapter below precedes exactly the same temporal filter.
      }

      // A cosine floor is not virtual-place evidence. Use deviation from the
      // best match for scene vectors; preserve the configured BoW likelihood.
      // These are relative likelihoods, not calibrated cosine probabilities.
      adjust_likelihood(result.ids, result.scores, scene ? 1 : config_.virtual_place_likelihood_method);

      result.likelihood_seconds = timer.lap_seconds();

      //============================================================
      // Apply the Bayes filter
      //  Posterior = Likelihood x Prior
      //============================================================

      // Compute the result.posterior
      result.posterior = memory_.topograph().observe(memory_.getStMem(), result.ids, result.scores);
      if (memory_.predictive_prefetch_enabled()) result.predicted = memory_.topograph().belief().rank_next_places(memory_.feature_cache_capacity());
      result.posterior_seconds = timer.lap_seconds();

      //============================================================
      // Select the highest hypothesis
      //============================================================
      if (result.posterior.size()) {
        for (std::map<int, float>::const_reverse_iterator iter = result.posterior.rbegin(); iter != result.posterior.rend(); ++iter) {
          if (iter->first > 0 && iter->second > result.hypothesis.posterior) {
            result.hypothesis.place_id = iter->first;
            result.hypothesis.posterior = iter->second;
          }
        }
        // With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
        result.hypothesis.known_probability = 1 - result.posterior.begin()->second;
      }
      result.selection_seconds = timer.lap_seconds();
      EXT_LOG_DEBUG("[retrieval] Best loop candidate: node={:d} posterior={:.3f} known_place_probability={:.3f}", result.hypothesis.place_id, result.hypothesis.posterior, result.hypothesis.known_probability);

      if (result.hypothesis.place_id > 0) {
        float loopThr = config_.min_known_probability;
        if (!((poses).find(result.hypothesis.place_id) != (poses).end()) &&  // not linked to previous map of that hypothesis
            memory_.getWorkingMemSize(true) > 0 &&                                            // should have an old map
            _rgbdSlamMode && loopThr > config_.min_unlinked_known_probability) {
          // use the best hypothesis directly.
          EXT_LOG_DEBUG("[retrieval] Using {}={:.6f}", "relocalizer.min_unlinked_known_probability", config_.min_unlinked_known_probability);
          loopThr = config_.min_unlinked_known_probability;
        }

        // Loop closure Threshold
        if (result.hypothesis.known_probability >= loopThr) {
          result.rejected = true;
          if (config_.min_known_probability_ratio > 0.0f && previous.known_probability && result.hypothesis.known_probability < config_.min_known_probability_ratio * previous.known_probability) {
            EXT_LOG_DEBUG("[retrieval] rejected hypothesis: not satisfying hypothesis ratio ({:.6f} < {:.6f} * {:.6f})", result.hypothesis.known_probability, config_.min_known_probability_ratio,
                         previous.known_probability);
          } else if (config_.min_known_probability_ratio > 0.0f && previous.known_probability == 0) {
            EXT_LOG_DEBUG("[retrieval] rejected hypothesis: last closure hypothesis is null (loop ratio is on)");
          } else {
            result.qualified_id = result.hypothesis.place_id;
            result.rejected = false;
          }

          result.validation_seconds = timer.lap_seconds();
        } else if (result.hypothesis.known_probability < config_.min_known_probability_ratio * previous.known_probability) {
          // Used for Precision-Recall computation.
          // When analyzing logs, it's convenient to know
          // if the hypothesis would be rejected if T_loop would be lower.
          result.rejected = true;
          EXT_LOG_DEBUG("[retrieval] rejected hypothesis: under loop ratio {:.6f} < {:.6f}", result.hypothesis.known_probability, config_.min_known_probability_ratio * previous.known_probability);
        }

        // for statistic...
        result.ratio = result.qualified_id && result.hypothesis.known_probability > 0 ? 1.0f : 0.0f;
      }
    }  // if(memory_.getWorkingMemSize())
  } else {
    EXT_LOG_DEBUG("[retrieval] Skipped: current node has insufficient visual features");
  }

  if (scene && result.qualified_id > 0) {
    result.scene_candidates.push_back(result.qualified_id);
    std::vector<int> alternatives;
    for (size_t i = 0; i < result.ids.size(); ++i)
      if (result.ids[i] > 0 && result.ids[i] != result.qualified_id && result.scores[i] > 1.f)
        alternatives.push_back(result.ids[i]);
    std::stable_sort(alternatives.begin(), alternatives.end(), [&](int a, int b) {
      return result.posterior.at(a) > result.posterior.at(b);
    });
    result.scene_candidates.insert(result.scene_candidates.end(), alternatives.begin(), alternatives.end());
  }
  if (scene) for (int id : result.scene_candidates) {
    const auto at = std::lower_bound(result.ids.begin(), result.ids.end(), id);
    EXT_ASSERT(at != result.ids.end() && *at == id);
    result.scene_candidate_scores.push_back(result.raw_scene_scores[size_t(at - result.ids.begin())]);
  }
  if (scene) EXT_LOG_INFO("[scene_retrieval] query={} candidates={} index=siglip_faiss known_probability={:.6f} qualified={}",
      query.id(), result.scene_candidates.size(), result.hypothesis.known_probability, result.qualified_id);
  EXT_LOG_INFO("[relocalization] query={} appearance_candidate={} known_probability={:.3f}",
      query.id(), result.hypothesis.place_id, result.hypothesis.known_probability);
  result.retrieval_id = result.hypothesis.place_id;
  return result;
}
void Relocalizer::retrieve_neighbors(const Node& query, const std::map<int, StateGroupf>& poses,
                                    const std::multimap<int, Link>& constraints, bool memory_limited,
                                    bool any_transferred, Relocalization& result) {
  const auto* node = &query;
  std::list<int> reactivatedIds;
  double timeGetNeighborsTimeDb = 0.0;
  double timeGetNeighborsSpaceDb = 0.0;
  auto& maxLocalLocationsImmunized = result.protected_local_limit;
  if (memory_limited) {
    // with memory management, we have to immunize some nodes
    maxLocalLocationsImmunized = _localImmunizationRatio * float(memory_.getWorkingMemSize(true));
  }
  // no need to do retrieval or immunization of locations if memory management
  // is disabled and all nodes are in WM.
  // Also skip memory mangement on intermediate nodes
  if (!((memory_.allNodesInWM() || config_.max_retrieved == 0) && maxLocalLocationsImmunized == 0) && node->getWeight() >= 0) {
    if (result.retrieval_id > 0) {
      // Load neighbors
      EXT_LOG_DEBUG("[retrieval] Retrieving locations... around id={:d}", result.retrieval_id);
      int neighborhoodSize = (int)memory_.topograph().belief().getPredictionLC().size() - 1;
      EXT_ASSERT(neighborhoodSize >= 0);
      EXT_LOG_DEBUG("[retrieval] margin={:d} maxRetieved={:d}", neighborhoodSize, config_.max_retrieved);

      tools::Timer timeGetN;
      unsigned int nbLoadedFromDb = 0;
      std::set<int> reactivatedIdsSet;
      int nbDirectNeighborsInDb = 0;
      // Preserve time-before-space priority, then increasing graph depth and
      // descending node ID. Visit each returned neighbor once, not once per depth.
      const auto append_neighbors = [&](const std::map<int, int>& neighbors, bool protect) {
        std::vector<std::pair<int, int>> ordered;
        ordered.reserve(neighbors.size());
        for (const auto& [id, depth] : neighbors) {
          if (depth < 0 || depth >= neighborhoodSize || memory_.isInSTM(id) || !reactivatedIdsSet.insert(id).second) continue;
          ordered.emplace_back(depth, id);
          const auto* resident = memory_.getNode(id);
          if (depth == 1 && !resident) ++nbDirectNeighborsInDb;
          if (protect && result.protected_ids.insert(id).second && (!resident || resident->getWeight() >= 0))
            ++result.protected_global;
        }
        std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
          return a.first != b.first ? a.first < b.first : a.second > b.second;
        });
        for (const auto& [depth, id] : ordered) reactivatedIds.push_back(id);
      };
      append_neighbors(memory_.getNeighborsId(result.retrieval_id, neighborhoodSize, config_.max_retrieved, true, true, false,
                                               {}, &timeGetNeighborsTimeDb), true);
      append_neighbors(memory_.getNeighborsId(result.retrieval_id, neighborhoodSize, config_.max_retrieved, true, false, false,
                                               {}, &timeGetNeighborsSpaceDb), false);
      EXT_LOG_DEBUG(
          "[retrieval] neighborhoodSize={:d}, "
          "reactivatedIds.size={:d}, "
          "nbLoadedFromDb={:d}, "
          "nbDirectNeighborsInDb={:d}, "
          "time={:.6f}s ({:.6f}s {:.6f}s)",
          neighborhoodSize, (int)reactivatedIds.size(), (int)nbLoadedFromDb, nbDirectNeighborsInDb, timeGetN.lap_seconds(), timeGetNeighborsTimeDb,
          timeGetNeighborsSpaceDb);
    }
  }

  std::list<int> retrievalLocalIds;
  if (_rgbdSlamMode && node->getWeight() >= 0) {
    if (!(memory_.allNodesInWM() && maxLocalLocationsImmunized == 0)) {
      // immunize the path from the nearest local location to the current location
      if (result.protected_local < maxLocalLocationsImmunized) {
        std::map<int, StateGroupf> resident_poses;
        // remove poses from STM
        for (auto iter = poses.begin(); iter != poses.end(); ++iter) {
          if (iter->first > 0 && !memory_.isInSTM(iter->first)) {
            resident_poses.insert(*iter);
          }
        }
        int nearestId = graph::findNearestNode(resident_poses, poses.at(node->id()));

        if (nearestId > 0 && (config_.local_radius_m == 0 || poses.at(node->id()).translationDistance(poses.at(nearestId)) < config_.local_radius_m)) {
          std::multimap<int, int> links;
          for (auto iter = constraints.begin(); iter != constraints.end(); ++iter) {
            if (((poses).find(iter->second.from()) != (poses).end()) &&
                ((poses).find(iter->second.to()) != (poses).end())) {
              links.insert(std::make_pair(iter->second.from(), iter->second.to()));
              links.insert(std::make_pair(iter->second.to(), iter->second.from()));  // <->
            }
          }

          std::list<std::pair<int, StateGroupf>> path = graph::computePath(poses, links, nearestId, node->id());
          if (path.size() == 0) {
            EXT_LOG_WARN("[retrieval] No local graph path from node {:d} to {:d}", nearestId, node->id());
          } else {
            for (std::list<std::pair<int, StateGroupf>>::iterator iter = path.begin(); iter != path.end(); ++iter) {
              if (iter->first > 0) {
                if (result.protected_local >= maxLocalLocationsImmunized) {
                  // set 20 to avoid this warning when starting mapping
                  if (maxLocalLocationsImmunized > 20 && any_transferred) {
                    EXT_LOG_WARN(
                        "[retrieval] Could not immunize the whole local path ({:d}) between "
                        "{:d} and {:d} (max location immunized={:d}). You may want "
                        "to increase {} (current={:.6f} ({:d} of WM={:d})) "
                        "to be able to immunize longer paths.",
                        (int)path.size(), nearestId, node->id(), maxLocalLocationsImmunized, "memory.protected_local_ratio",
                        _localImmunizationRatio, maxLocalLocationsImmunized, (int)memory_.getWorkingMemSize(true));
                  }
                  break;
                } else if (!memory_.isInSTM(iter->first)) {
                  if (result.protected_ids.insert(iter->first).second) {
                    // Count only non-intermediate nodes (intermediate nodes are still
                    // immunized but don't consume the immunization budget/statistic).
                    const Node* sLocal = memory_.getNode(iter->first);
                    if (sLocal == 0 || sLocal->getWeight() >= 0) {
                      ++result.protected_local;
                    }
                  }
                }
              }
            }
          }
        }
      }

      // retrieval based on the nodes close the the nearest pose in WM
      // immunize closest nodes
      std::map<int, float> nearNodes = graph::findNearestNodes(node->id(), poses, config_.local_radius_m);
      // sort by distance
      std::multimap<float, int> nearNodesByDist;
      for (std::map<int, float>::iterator iter = nearNodes.lower_bound(1); iter != nearNodes.end(); ++iter) {
        nearNodesByDist.insert(std::make_pair(iter->second, iter->first));
      }
      EXT_LOG_DEBUG("[retrieval] near nodes={:d}, max local immunized={:d} (immunized by path so far={:d}), ratio={:.6f} WM={:d}", (int)nearNodesByDist.size(),
                   maxLocalLocationsImmunized, result.protected_local, _localImmunizationRatio, (int)memory_.getWorkingMemSize(true));
      std::list<int> retrievalLocalIdsIntermediate;
      for (std::multimap<float, int>::iterator iter = nearNodesByDist.begin();
           iter != nearNodesByDist.end() && (retrievalLocalIds.size() < config_.max_local_retrieved || result.protected_local < maxLocalLocationsImmunized); ++iter) {
        const Node* s = memory_.getNode(iter->second);
        if (s != 0) {
          if (s->getWeight() != -1 && retrievalLocalIds.size() < config_.max_local_retrieved) {
            // If there is a change of direction, better to be retrieving
            // all nearest nodes than only newest neighbors.
            // Use getNeighborsId instead of direct links to support intermediate nodes.
            std::map<int, int> ids = memory_.getNeighborsId(s->id(), 2, config_.max_local_retrieved - retrievalLocalIds.size(), true, false, false);
            for (std::map<int, int>::const_reverse_iterator jter = ids.rbegin();
                 jter != ids.rend() && (retrievalLocalIds.size() < config_.max_local_retrieved || jter->second == 0); ++jter) {
              if (memory_.getNode(jter->first) == 0) {
                if (jter->second == 0) {
                  EXT_LOG_DEBUG("[retrieval] retrieval of intermediate node {:d} (margin={:d}) on local map (from={:d})", jter->first, jter->second, s->id());
                  retrievalLocalIdsIntermediate.push_back(jter->first);
                } else {
                  EXT_LOG_DEBUG("[retrieval] retrieval of node {:d} (margin={:d}) on local map (from={:d})", jter->first, jter->second, s->id());
                  retrievalLocalIds.push_back(jter->first);
                }
              }
            }
          }
          if (!memory_.isInSTM(s->id()) && result.protected_local < maxLocalLocationsImmunized) {
            // Count only non-intermediate nodes (intermediate nodes are still
            // immunized but don't consume the immunization budget/statistic).
            if (result.protected_ids.insert(s->id()).second && s->getWeight() >= 0) {
              ++result.protected_local;
            }
          }
        }
      }
      // well, if the maximum retrieved is not reached, look for neighbors in database
      if (retrievalLocalIds.size() < config_.max_local_retrieved) {
        std::set<int> retrievalLocalIdsSet(retrievalLocalIds.begin(), retrievalLocalIds.end());
        retrievalLocalIdsSet.insert(retrievalLocalIdsIntermediate.begin(), retrievalLocalIdsIntermediate.end());
        for (std::list<int>::iterator iter = retrievalLocalIds.begin(); iter != retrievalLocalIds.end() && retrievalLocalIds.size() < config_.max_local_retrieved;
             ++iter) {
          std::map<int, int> ids = memory_.getNeighborsId(*iter, 2, config_.max_local_retrieved - (unsigned int)retrievalLocalIds.size() + 1, true, false, false);
          for (std::map<int, int>::reverse_iterator jter = ids.rbegin();
               jter != ids.rend() && (retrievalLocalIds.size() < config_.max_local_retrieved || jter->second == 0); ++jter) {
            if (memory_.getNode(jter->first) == 0 && retrievalLocalIdsSet.find(jter->first) == retrievalLocalIdsSet.end()) {
              if (jter->second == 0) {
                EXT_LOG_DEBUG("[retrieval] retrieval of intermediate node {:d} (margin={:d}) on local map (from={:d})", jter->first, jter->second, *iter);
                retrievalLocalIdsIntermediate.push_back(jter->first);
              } else {
                EXT_LOG_DEBUG("[retrieval] retrieval of node {:d} (margin={:d}) on local map (from={:d})", jter->first, jter->second, *iter);
                retrievalLocalIds.push_back(jter->first);
              }
              retrievalLocalIdsSet.insert(jter->first);
            }
          }
        }
      }

      // update Age of the close nodes (oldest the farthest)
      for (std::multimap<float, int>::reverse_iterator iter = nearNodesByDist.rbegin(); iter != nearNodesByDist.rend(); ++iter) {
        memory_.updateAge(iter->second);
      }

      // insert them first to make sure they are loaded.
      reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIdsIntermediate.begin(), retrievalLocalIdsIntermediate.end());
      reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIds.begin(), retrievalLocalIds.end());
    }
  }

  //============================================================
  // RETRIEVAL 3/3 : Load nodes from the database
  //============================================================
  if (reactivatedIds.size()) {
    double neighbor_load_time = 0;
    const unsigned int remaining = config_.max_retrieved > result.loaded.size() ? config_.max_retrieved - result.loaded.size() : 0;
    const unsigned int quota = remaining + static_cast<unsigned int>(retrievalLocalIds.size());
    if (quota) {
      const auto loaded = memory_.reactivateNodes(reactivatedIds, quota, neighbor_load_time);
      result.loaded.insert(loaded.begin(), loaded.end());
    }
    result.database_seconds += neighbor_load_time;

    EXT_LOG_DEBUG("[retrieval] Restored {:d} candidate nodes from database", (int)result.loaded.size());

    result.database_seconds += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;

    // Immunize just retrieved nodes
    result.protected_ids.insert(result.loaded.begin(), result.loaded.end());
  }
}
void Relocalizer::adjust_likelihood(const std::vector<int>& ids, std::vector<float>& likelihood, int virtual_place_ratio) {
  EXT_LOG_DEBUG("[retrieval] likelihood.size()={:d}", (int)likelihood.size());
  if (likelihood.empty()) {
    return;
  }
  EXT_ASSERT(ids.size() == likelihood.size());
  for (std::size_t i = 1U; i < ids.size(); ++i) {
    EXT_ASSERT(ids[i - 1U] < ids[i]);
  }
  const bool virtualPlaceUsed = ids.front() < 0;
  const std::size_t realBegin = virtualPlaceUsed ? 1U : 0U;

  // Use only non-null values (ignore virtual place)
  std::list<float> values;
  bool likelihoodNullValuesIgnored = true;
  for (std::size_t i = realBegin; i < likelihood.size(); ++i) {
    if ((likelihood[i] >= 0 && !likelihoodNullValuesIgnored) || (likelihood[i] > 0 && likelihoodNullValuesIgnored)) {
      values.push_back(likelihood[i]);
    }
  }
  EXT_LOG_DEBUG("[retrieval] values.size={:d}", (int)values.size());

  float mean = tools::mean(values);
  float stdDev = std::sqrt(tools::sample_variance(values, mean));

  // Adjust likelihood with mean and standard deviation (see Angeli phd)
  float epsilon = 0.0001;
  float max = 0.0f;
  int maxId = 0;
  for (std::size_t i = realBegin; i < likelihood.size(); ++i) {
    float value = likelihood[i];
    likelihood[i] = 1.0f;
    // A lone nonzero score has zero estimated spread. Preserve its evidence
    // conservatively without dividing by zero or scaling with database size.
    if (values.size() == 1 && value > 0) {
      likelihood[i] = 2.0f;
    } else if (value > mean + stdDev) {
      if (virtual_place_ratio == 0 && mean) {
        likelihood[i] = (value - (stdDev - epsilon)) / mean;
      } else if (virtual_place_ratio != 0 && stdDev) {
        likelihood[i] = (value - mean) / stdDev;
      }
    }

    if (value > max) {
      max = value;
      maxId = ids[i];
    }
  }

  if (virtualPlaceUsed) {
    if (virtual_place_ratio == 0 && stdDev > epsilon && max) {
      likelihood.front() = mean / stdDev + 1.0f;
    } else if (virtual_place_ratio != 0 && max > mean) {
      likelihood.front() = stdDev / (max - mean) + 1.0f;
    } else {
      likelihood.front() = 2.0f;  // 2 * std dev
    }
  }

  EXT_LOG_DEBUG("[retrieval] Similarity: mean={:.3f} stddev={:.3f} max={:.3f} node={:d}", mean, stdDev, max, maxId);
}


}  // namespace sapphire::backend
