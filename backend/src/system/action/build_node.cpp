#include <Eigen/Cholesky>
#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/database/compression.hpp>
#include <backend/database/statistics.hpp>
#include <backend/system/action/build_node.hpp>
#include <backend/system/action/preprocess.hpp>
#include <backend/system/action/relocalizer.hpp>
#include <backend/system/action/update_database.hpp>
#include <backend/memory.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/parameters.hpp>
#include <backend/registration/retrieval/fbow.hpp>
#include <backend/maplayer/featmap/maintenance.hpp>
#include <backend/tools/timer.hpp>
#include <cstring>
#include <stdexcept>

#include "backend/maplayer/featmap/refinement.hpp"

namespace sapphire::backend {
AdmissionResult admit_frame(Memory& memory, FrameBuilder& frames, const VisualRegistration& registration, ArchiveWriter& archives,
                            const LocalGridMaker& grids, const SensorData& data, const StateGroupf& pose, const Mat66_t<double>& covariance,
                            const std::vector<float>& velocity, Statistics* stats, FrameBuilder::Result* prepared, bool allow_rehearsal) {
  AdmissionResult result;
  tools::Timer timer;
  tools::Timer admission_timer;
  memory.begin_update();
  if (stats) stats->addStatistic(Statistics::kTimingMemPre_update(), timer.lap_seconds() * 1000.0f);
  bool isIntermediateNode = data.id() < 0;

  if (memory.empty() && isIntermediateNode) {
    EXT_LOG_WARN("Ignoring input data with stamp {:.6f} because the first node in memory cannot be an intermediate node.", data.stamp());
    return {};
  }

  EXT_ASSERT_MSG(data.imageRaw().empty() || data.imageRaw().type() == CV_8UC1, "Visual admission accepts only CV_8UC1 grayscale images.");
  EXT_ASSERT_MSG(data.depthRaw().empty() ||
                     (data.depthRaw().type() == CV_16UC1 && (data.imageRaw().empty() || data.depthRaw().size() == data.imageRaw().size())),
                 "image=({:d}/{:d}, type={:d}, [accepted={:d}(grayscale)]) depth=({:d}/{:d}, type={:d} [accepted={:d}(depth mm)]). "
                 "Depth must be CV_16UC1 in millimeters and grayscale/depth mosaics must have the same size.",
                 data.imageRaw().cols, data.imageRaw().rows, data.imageRaw().type(), CV_8UC1, data.depthRaw().cols, data.depthRaw().rows,
                 data.depthRaw().type(), CV_16UC1);
  if (!data.depthRaw().empty() && data.perspectives().empty() && !pose.isNull()) {
    EXT_LOG_ERROR("[input] Cannot create node: depth supplied without a camera model");
    return {};
  }

  const int id = memory.reserve_node_id(data.id());
  if (!id) return {};
  const bool keep_intermediate = memory.archive_params().keep_intermediate_data;
  const bool use_gravity = memory.topograph_params().use_odometry_gravity;
  const bool create_grid = memory.navimap_params().frame_grid_enabled;
  auto frame = prepared ? std::move(*prepared) : frames.build(data);
  if (stats) {
    stats->addStatistic(Statistics::kTimingMemKeypoints_detection(), frame.timing.keypoints_ms);
    stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), frame.timing.descriptors_ms);
    stats->addStatistic(Statistics::kTimingMemPost_decimation(), frame.timing.post_decimation_ms);
  }

  timer.reset();

  SensorRecord record{SensorData(frame.image, frame.depth, frame.cameras, id, data.stamp()), {}};
  auto s = std::make_unique<Node>(id, memory.current_map_id(), isIntermediateNode ? -1 : 0, data.stamp(), "", pose, record.observation.visualFrame());
  s->set_lidar_cloud(data.lidar_cloud);
  const NodeBuildOptions build_options{frames.max_features(), memory.uses_raw_features(), keep_intermediate, use_gravity};
  double quantization_ms = 0;

  build_node(*s, frame, data, build_options, memory.fbow_index(), registration.pixelVariances(), &quantization_ms);
  if (!isIntermediateNode && !frame.image.empty()) {
    if (memory.feature_params().uses_xfeat() && frame.cameras.size() != 1)
      throw std::invalid_argument("Global descriptor encoding requires one reference camera");
    tools::Timer global_timer;
    auto descriptors = frames.global_descriptors(frame.image);
    if (!descriptors.empty()) {
      if (!data.globalDescriptors().empty()) throw std::invalid_argument("Cannot mix supplied and locally encoded global descriptors");
      s->set_global_descriptors(std::move(descriptors));
      if (stats) stats->addStatistic("Timing/GlobalDescriptor/ms", global_timer.elapsed_seconds() * 1000.0f);
    }
  }
  if (stats && memory.fbow_index()) stats->addStatistic(Statistics::kTimingMemAdd_new_words(), quantization_ms);

  archives.prepare(record, data, *s, stats);
  if (prepare_navimap(record.observation, data, pose, grids, create_grid, stats)) encodeSensorData(record, false, false, true, archives.executor());
  if (stats) stats->addStatistic(Statistics::kTimingMemNode_creation(), admission_timer.lap_seconds() * 1000.0f);
  timer.reset();
  auto* node = memory.admit_node(std::move(s), std::move(record), covariance, velocity);
  if (allow_rehearsal && memory.getSimilarityThreshold() < 1.0f) result.rehearsed_id = rehearse_node(memory, node, registration, stats);
  if (stats) stats->addStatistic(Statistics::kTimingMemRehearsal(), timer.lap_seconds() * 1000.0f);
  result.reduced_ids = memory.maintain_residency();
  if (stats) stats->setReducedIds(result.reduced_ids);
  result.accepted = true;
  return result;
}

int rehearse_node(Memory& memory, const Node* node, const VisualRegistration& registration, Statistics* stats) {
  tools::Timer timer;
  if (node->has_lidar() || node->isBadNode()) {
    return 0;
  }

  //============================================================
  // Compare with the last (not intermediate node)
  //============================================================
  const Node* sB = 0;
  for (std::set<int>::reverse_iterator iter = memory.getStMem().rbegin(); iter != memory.getStMem().rend(); ++iter) {
    const Node* s = memory.getNode(*iter);
    EXT_ASSERT(s != 0);
    if (s->getWeight() >= 0 && s->id() != node->id()) {
      sB = s;
      break;
    }
  }
  if (sB) {
    int id = sB->id();
    EXT_LOG_DEBUG("Comparing with node ({:d})...", id);

    float sim = 0;
    if (memory.fbow_index())
      sim = node->compareTo(*sB);
    else if (node->loop_features()) {
      auto bank = sB->map_features();
      if (!bank && sB->loop_features()) bank = featmap::LandmarkMap::from_frame(*sB->loop_features(), featmap::point_capacity);
      if (bank) {
        auto matches = registration.match_local(*bank, *node->loop_features());
        const auto count = std::max(bank->points().size(), node->loop_features()->keypoints.size());
        sim = count ? float(matches.size()) / count : 0;
      }
    }

    int merged = 0;
    if (sim >= memory.getSimilarityThreshold()) {
      bool skip_merge = false;
      const auto& config = memory.node_params();
      const float max_distance = config.min_translation_m;
      const float max_angle = config.min_rotation_rad;
      const bool ignore_moving = config.rehearsal_ignore_moving;
      bool fullMerge;
      bool intermediateMerge = false;
      if (!node->getLinks().empty() && !node->getLinks().begin()->second.transform().isNull()) {
        // we are in metric SLAM mode:
        // 1) Normal merge if not moving AND has direct link
        // 2) StateGroupf to intermediate node (weight = -1) if not moving AND hasn't direct link.
        float x, y, z, roll, pitch, yaw;
        const StateGroupf& linkTransform = node->getLinks().begin()->second.transform();
        const StateGroupf::Vector3 translation = linkTransform.translationVector();
        const StateGroupf::Vector3 yawPitchRoll = linkTransform.rotationMatrix().eulerAngles(2, 1, 0);
        x = translation.x();
        y = translation.y();
        z = translation.z();
        roll = yawPitchRoll.z();
        pitch = yawPitchRoll.y();
        yaw = yawPitchRoll.x();
        bool isMoving = fabs(x) > max_distance || fabs(y) > max_distance || fabs(z) > max_distance || fabs(roll) > max_angle ||
                        fabs(pitch) > max_angle || fabs(yaw) > max_angle;
        if (isMoving && ignore_moving) {
          EXT_LOG_DEBUG("Rehearsal ignored because the robot has moved more than {:.6f} m or {:.6f} rad (\"node.rehearsal_ignore_moving\"=true)",
                        max_distance, max_angle);
          skip_merge = true;
        }
        fullMerge = !isMoving && node->hasLink(sB->id());
        intermediateMerge = !isMoving && !node->hasLink(sB->id());
      } else {
        fullMerge = node->hasLink(sB->id()) && node->getLinks().begin()->second.transform().isNull();
      }
      if (!skip_merge && memory.apply_rehearsal(id, node->id(), fullMerge, intermediateMerge)) {
        merged = id;
      }
    }

    if (stats) stats->addStatistic(Statistics::kMemoryRehearsal_merged(), merged);
    if (stats) stats->addStatistic(Statistics::kMemoryRehearsal_sim(), sim);
    if (stats) stats->addStatistic(Statistics::kMemoryRehearsal_id(), sim >= memory.getSimilarityThreshold() ? id : 0);
    EXT_LOG_DEBUG("merged={:d}, sim={:.6f} t={:.6f}s", merged, sim, timer.lap_seconds());
    return merged;
  } else {
    if (stats) stats->addStatistic(Statistics::kMemoryRehearsal_merged(), 0);
    if (stats) stats->addStatistic(Statistics::kMemoryRehearsal_sim(), 0);
  }
  return 0;
}

void build_node(Node& draft, FrameBuilder::Result& frame, const SensorData& input, const NodeBuildOptions& options, const FbowIndex* vocabulary,
                const std::vector<double>& pixel_variances, double* quantization_ms) {
  if (draft.loop_features() || draft.has_map_features() || draft.isEnabled())
    throw std::invalid_argument("Node construction requires a fresh private draft");
  if (options.raw_features && vocabulary) throw std::invalid_argument("Node construction must select one retrieval backend");
  if (quantization_ms) *quantization_ms = 0;
  const bool isIntermediateNode = input.id() < 0;
  const auto& pose = draft.getPose();
  auto& keypoints = frame.features.keypoints;
  auto& keypoints3D = frame.features.points;
  auto& descriptors = frame.features.descriptors;
  if (keypoints.size() > bow::FeatureBlock::max_features) throw std::invalid_argument("Feature count exceeds the online BoW limit (8192)");
  tools::Timer timer;
  std::vector<int> wordIds;
  std::vector<uint32_t> match_node_ids;
  bool quantized = false;
  const bool rawFeatures = options.raw_features && !keypoints.empty() && descriptors.rows && !isIntermediateNode;
  if (vocabulary && !keypoints.empty()) {
    if (descriptors.rows && !isIntermediateNode) {
      wordIds = vocabulary->quantize(descriptors, &match_node_ids);
      std::vector<bool> selected;
      if (options.max_features > 0 && descriptors.rows > options.max_features)
        FastOrbStereo::selectStrongest(keypoints, selected, options.max_features);
      int invalid_id = -1;
      for (size_t i = 0; i < wordIds.size(); ++i) {
        if (wordIds[i] <= 0 || (!selected.empty() && !selected[i])) wordIds[i] = invalid_id--;
      }
      quantized = true;
    } else {
      // Set all words as not used in dictionary
      int negIndex = -1;
      for (size_t i = 0; i < keypoints.size(); ++i) {
        wordIds.push_back(negIndex--);
      }
    }

    if (quantization_ms) *quantization_ms = timer.lap_seconds() * 1000.0;
  }

  std::shared_ptr<const bow::FeatureBlock> query;
  if (rawFeatures) {
    std::vector<bool> selected;
    if (options.max_features > 0 && descriptors.rows > options.max_features)
      FastOrbStereo::selectStrongest(keypoints, selected, options.max_features);
    if (frame.features.descriptor_model_id) {
      if (!selected.empty()) throw std::logic_error("Learned extraction exceeded node admission feature budget");
      query = bow::FeatureBlock::create_learned_owned(std::move(keypoints), std::move(keypoints3D), std::move(descriptors),
                                                      frame.features.descriptor_model_id);
    } else if (selected.empty()) {
      // Extraction owns these arrays; supplied descriptors may share only the
      // input VisualFrame's immutable storage. No writable aliases escape.
      query = bow::FeatureBlock::create_raw_owned(std::move(keypoints), std::move(keypoints3D), std::move(descriptors));
    } else {
      const size_t count = std::count(selected.begin(), selected.end(), true);
      VisualFeatures filtered;
      filtered.keypoints.reserve(count);
      if (!keypoints3D.empty()) filtered.points.reserve(count);
      filtered.descriptors.create(static_cast<int>(count), descriptors.cols, descriptors.type());
      int row = 0;
      for (size_t i = 0; i < keypoints.size(); ++i) {
        if (!selected[i]) continue;
        filtered.keypoints.push_back(keypoints[i]);
        if (!keypoints3D.empty()) filtered.points.push_back(keypoints3D[i]);
        descriptors.row(i).copyTo(filtered.descriptors.row(row++));
      }
      query = bow::FeatureBlock::create_raw_owned(std::move(filtered.keypoints), std::move(filtered.points), std::move(filtered.descriptors));
    }
  } else if (quantized) {
    // Stable vocabulary grouping is the only required row permutation/copy.
    query = bow::FeatureBlock::create(vocabulary->getFBoWVocabularyHash(), match_node_ids, wordIds, keypoints, keypoints3D, descriptors);
  }
  // A metric visual keyframe owns a bank even when sparse 3D is absent.
  // Appearance-only and intermediate observations retain their existing policy.
  const bool submap_input = !isIntermediateNode && !pose.isNull() && bool(query);
  if (!isIntermediateNode || options.keep_intermediate) {
    if (query) {
      draft.set_loop_features(std::move(query));
      if (submap_input) {
        auto map = featmap::LandmarkMap::from_frame(
            *draft.loop_features(), Node::max_natural_points,
            draft.visualFrame().perspectives().size() == 1 ? &draft.visualFrame().perspectives().front() : nullptr, pixel_variances);
        draft.set_map_features(std::move(map));
      }
    } else if (!wordIds.empty()) {
      std::multimap<int, int> words;
      for (size_t i = 0; i < wordIds.size(); ++i) words.emplace(wordIds[i], static_cast<int>(i));
      draft.setWords(words, keypoints, keypoints3D, descriptors);
    } else {
      draft.setWords({}, {}, {}, {});
    }
  }

  if (!isIntermediateNode || options.keep_intermediate) draft.set_global_descriptors(input.globalDescriptors());

  // prior
  if (!input.globalPose().isNull() && input.globalPoseCovariance()) {
    draft.addLink(
        Link(draft.id(), draft.id(), Link::kPosePrior, input.globalPose(), input.globalPoseCovariance()->llt().solve(Mat66_t<double>::Identity())));
    EXT_LOG_DEBUG("Added global pose prior: {}", input.globalPose().debugString().c_str());
  }

  // Gravity constraint
  if (options.use_odometry_gravity && !pose.isNull()) {
    draft.addLink(Link(draft.id(), draft.id(), Link::kGravity, pose.rotationPart()));
    EXT_LOG_DEBUG("Added gravity constraint from odom pose: {}", pose.rotationPart().debugString().c_str());
  }
}
namespace {
struct ProfilePhase {
  double& total;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  ~ProfilePhase() { total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count(); }
};
std::map<int, StateGroupf> bundle_boundaries(int anchor, const featmap::NodeObservations& bundle, const Memory& memory,
                                             const std::map<int, StateGroupf>& current) {
  if (bundle.reference_node && bundle.reference_node != anchor) throw std::runtime_error("Node-local pose reference mismatch");
  std::set<int> needed;
  for (const auto& o : bundle.observations) needed.insert(o.node);
  std::map<int, StateGroupf> result;
  for (auto it = needed.begin(); it != needed.end();) {
    const auto local = bundle.observation_poses.find(*it);
    if (local != bundle.observation_poses.end())
      result.emplace(*it, local->second);
    else if (*it == anchor)
      result.emplace(anchor, StateGroupf::identity());
    else {
      ++it;
      continue;
    }
    it = needed.erase(it);
  }
  // A submap owns its local geometry even while a new session has no world
  // alignment. Resolve only missing historical placements from the graph.
  if (needed.empty()) return result;
  needed.insert(anchor);
  std::map<int, StateGroupf> saved;
  if (memory.has_database() && std::any_of(needed.begin(), needed.end(), [&](int id) { return !current.count(id); }))
    saved = memory.loadOptimizedPoses(nullptr);
  const auto find = [&](int id) -> const StateGroupf* {
    auto i = current.find(id);
    if (i != current.end()) return &i->second;
    auto j = saved.find(id);
    return j == saved.end() ? nullptr : &j->second;
  };
  const auto* anchor_pose = find(anchor);
  if (!anchor_pose) throw std::runtime_error("Missing submap anchor pose");
  const auto origin = anchor_pose->inverse();
  // One composition per keyframe, regardless of how many pixels it observed.
  needed.erase(anchor);
  for (int id : needed)
    if (const auto* p = find(id)) result.emplace(id, origin.compose(*p));
  return result;
}
}  // namespace

featmap::MergeDraft prepare_node_content(const NodeMergeCandidate& candidate, const StateGroupf& target_from_query,
                                         const std::shared_ptr<const featmap::LandmarkMap>& bank, const std::vector<featmap::Match>& inliers,
                                         const std::vector<uint32_t>& rejected_rows) {
  const auto query_from_target = target_from_query.inverse();
  // Neutral ORB quality is not depth confidence. Shared with offline diagnostics.
  const std::vector<float> quality(candidate.query_features->keypoints.size(), 0.5f);
  // Associations were checked before graph validation; never add new ones after PGO.
  auto draft = candidate.refresh
                   ? featmap::prepare_refresh(bank, candidate.query, candidate.query_features, query_from_target, inliers, rejected_rows, quality,
                                              candidate.geometry, candidate.appearance, candidate.bundle.landmark_high_water)
                   : featmap::prepare_merge(bank, candidate.query, candidate.query_features, query_from_target, inliers, {}, rejected_rows, quality,
                                            candidate.geometry, candidate.appearance);
  if (!draft.features) {
    const auto& stats = draft.geometry.stats();
    const auto reason = "Node geometry rejected: status=" + std::to_string(int(draft.geometry.status())) +
                        ", seeds=" + std::to_string(stats.seed_support) + ", compatible=" + std::to_string(stats.compatible) +
                        ", conflicts=" + std::to_string(stats.conflicts) + ", unavailable=" + std::to_string(stats.unavailable) +
                        ", cells=" + std::to_string(stats.image_cells) + ", projection=" + std::to_string(stats.projection_conflicts) + "/" +
                        std::to_string(stats.seed_projection_conflicts) + ", image_outliers=" + std::to_string(stats.seed_ray_conflicts) + "/" +
                        std::to_string(stats.seed_ray_checks);
    throw featmap::MergeRejected(reason);
  }
  return draft;
}
featmap::MergeDraft prepare_node_content(const NodeMergeCandidate& candidate, const StateGroupf& target_from_query) {
  return prepare_node_content(candidate, target_from_query, candidate.target, candidate.inliers, candidate.rejected_rows);
}
namespace {
NodeMergeCandidate submap_merge_evidence(const Memory& memory, const Relocalizer& relocalizer, int retained, int removed,
                                         const Registration& registration) {
  const auto* target = memory.getNode(retained);
  const auto* query = memory.getNode(removed);
  if (relocalizer.inverted() || !target || !query || retained == removed || !target->has_map_features() || !query->has_map_features() ||
      !query->loop_features() || (!registration.map_matches || registration.inliers.empty()) || registration.pose.isNull())
    throw std::runtime_error("Missing canonical submap/query evidence");
  NodeMergeCandidate candidate;
  candidate.geometry.pyramid_scale = memory.feature_params().orb_pyramid_scale;
  candidate.geometry.pyramid_levels = memory.feature_params().orb_pyramid_levels;
  candidate.geometry.pixel_scale = relocalizer.geometry().pixel_scale();
  candidate.retained = retained;
  candidate.removed = removed;
  candidate.target = memory.map_features_for(retained);
  if (!candidate.target) throw std::runtime_error("Missing submap feature bank");
  candidate.query = query->visualFrame();
  candidate.query_features = query->loop_features();
  std::vector<bool> used_points(candidate.target->points().size()), used_features(candidate.query_features->keypoints.size());
  for (const auto& match : registration.inliers) {
    if (match.landmark >= used_points.size() || match.feature >= used_features.size() || used_points[match.landmark] || used_features[match.feature])
      throw std::runtime_error("Invalid final loop inlier identity");
    used_points[match.landmark] = used_features[match.feature] = true;
  }
  candidate.association_complete = registration.projection_complete;
  candidate.inliers = registration.inliers;
  candidate.rejected_rows = registration.rejected_features;
  if (!candidate.association_complete) {
    candidate.rejected_rows.clear();
    for (uint32_t row = 0; row < candidate.query.keypoints().size(); ++row) candidate.rejected_rows.push_back(row);
  }
  return candidate;
}
}  // namespace
NodeMergeCandidate prepare_node_merge(const Memory& memory, const Relocalizer& relocalizer, int retained, int removed,
                                      const Registration& registration) {
  auto candidate = submap_merge_evidence(memory, relocalizer, retained, removed, registration);
  prepare_node_content(candidate, registration.pose);
  return candidate;
}
namespace {
std::unique_ptr<Node> prepare_refresh_content(const Memory&, const Relocalizer&, const NodeMergeCandidate&,
                                              const sapphire::backend::featmap::NodeBundleResult&, std::map<int, StateGroupf>);
}
NodeMergeCandidate prepare_node_refresh(const Memory& memory, const Relocalizer& relocalizer, int retained, int query,
                                        const Registration& registration, const std::map<int, StateGroupf>& poses) {
  const auto& transform = registration.pose;
  if (!memory.has_database()) throw std::runtime_error("Node refresh requires a persistent map database");
  if (memory.topograph().is_submap_member(retained) || memory.topograph().is_submap_member(query))
    throw std::runtime_error("Node member cannot own a canonical bank");
  // Refresh performs the same geometry checks below, with its actual history
  // and capacity policy. Do not build and discard an ordinary-merge bank first.
  auto candidate = submap_merge_evidence(memory, relocalizer, retained, query, registration);
  candidate.refresh = true;
  candidate.bundle_options = memory.local_ba_options();
  const auto* owner = memory.getNode(retained);
  candidate.bundle = owner && owner->has_observation_history() ? owner->observation_history()
                                                               : featmap::NodeObservations::decode(memory.load_observation_archive(retained));
  for (const auto& p : candidate.target->points()) candidate.bundle.landmark_high_water = std::max(candidate.bundle.landmark_high_water, p.id);
  if (candidate.bundle.observations.empty())
    for (const auto& p : candidate.target->points())
      if (p.stereo.valid()) {
        candidate.bundle.observations.push_back({retained, p.id, p.stereo, p.appearance, uint32_t(p.id - 1)});
      }
  candidate.bundle.refresh_support();
  candidate.bundle.reference_node = retained;
  const auto coverage = prepare_node_content(candidate, registration.pose);
  // Point retention is a content decision. Capacity cannot reject an otherwise
  // valid node fusion, and discarded new coverage is not geometric redundancy.
  candidate.shared_geometry_redundant =
      featmap::redundant_node_observations(*candidate.target, candidate.bundle, *candidate.query_features, candidate.inliers,
                                           relocalizer.geometry().pixelVariances(), candidate.geometry.pyramid_scale);
  candidate.redundant_submap = candidate.shared_geometry_redundant && coverage.stats.eligible == 0;
  EXT_LOG_DEBUG("[submap_admission] submap={} query={} eligible={} added={} redundant={}", retained, query, coverage.stats.eligible,
                coverage.stats.added, candidate.redundant_submap);
  // New coverage is an insertion decision, not evidence that already mature
  // shared geometry must be reestimated from the same repeated pixels.
  auto boundaries = bundle_boundaries(retained, candidate.bundle, memory, poses);
  // The first refresh materializes a one-frame history from immutable seeds.
  // Retain the exact old poses already resolved for BA, before any solved pose
  // is applied; unmatched/deferred points need the same original chart.
  candidate.bundle.observation_poses.insert(boundaries.begin(), boundaries.end());
  // Internal placement survives history culling. Recover every older member
  // before freezing the candidate, including legacy content-only members.
  for (const auto& [capture, owner_id] : memory.topograph().members()) {
    if (owner_id != retained || candidate.bundle.observation_poses.count(capture)) continue;
    auto local = memory.topograph().local_poses().find(capture);
    if (local != memory.topograph().local_poses().end())
      candidate.bundle.observation_poses[capture] = local->second;
    else if (poses.count(capture) && poses.count(retained))
      candidate.bundle.observation_poses[capture] = poses.at(retained).between(poses.at(capture));
    else
      throw std::runtime_error("Missing historical capture placement");
  }
  sapphire::backend::featmap::NodeBundleResult solved;
  if (candidate.shared_geometry_redundant) {
    solved.status = solver::BASolver::Status::Success;
    solved.pose = transform;
    for (size_t i = 0; i < candidate.inliers.size(); ++i) solved.inliers.push_back(i);
  } else {
    solved = sapphire::backend::featmap::refine_node(*candidate.target, candidate.bundle, boundaries, *candidate.query_features,
                                                     candidate.query.perspectives().front(), candidate.inliers, transform,
                                                     relocalizer.geometry().pixelVariances(), relocalizer.geometry().getMinVisualCorrespondences(),
                                                     false, false, retained, candidate.bundle_options);
  }
  if (solved.pose.isNull()) throw featmap::MergeRejected("Node BA: " + solved.failure);
  candidate.relative = solved.pose;
  candidate.bundle_views = solved.views;
  candidate.bundle_ms = solved.elapsed_ms;
  candidate.bundle_measurements = solved.measurements;
  candidate.bundle_survivors = solved.inliers.size();
  for (int index : solved.inliers) {
    const auto& match = candidate.inliers.at(size_t(index));
    for (uint64_t value : {candidate.target->points().at(match.landmark).id, uint64_t(match.feature)}) {
      candidate.bundle_identity_digest ^= value;
      candidate.bundle_identity_digest *= 1099511628211ULL;
    }
  }
  candidate.replacement = prepare_refresh_content(memory, relocalizer, candidate, solved, std::move(boundaries));
  // This is the same visual evidence as the initializing registration, with
  // uncertain landmarks now optimized and marginalized. Replace that estimate,
  // rather than testing this solution against a fixed-landmark PnP covariance.
  // The reference is fixed; the query marginal uses Link's [translation, rotation]
  // right tangent. It is conditional on calibration and fixed historical views,
  // not an independent measurement or a full-system uncertainty guarantee.
  candidate.covariance = solved.covariance;

  return candidate;
}
namespace {
std::unique_ptr<Node> prepare_refresh_content(const Memory& memory, const Relocalizer& relocalizer, const NodeMergeCandidate& candidate,
                                              const sapphire::backend::featmap::NodeBundleResult& solved, std::map<int, StateGroupf> boundaries) {
  const auto* target = memory.getNode(candidate.retained);
  const auto* source = memory.getNode(candidate.removed);
  if (!target || !source) throw std::runtime_error("Expired submap content owners");
  std::vector<featmap::Match> final_inliers;
  auto rejected_rows = candidate.rejected_rows;
  std::set<int> accepted_ids(solved.inliers.begin(), solved.inliers.end());
  for (size_t i = 0; i < candidate.inliers.size(); ++i) {
    if (accepted_ids.count(i))
      final_inliers.push_back(candidate.inliers[i]);
    else
      rejected_rows.push_back(candidate.inliers[i].feature);
  }
  for (const auto& [node, p] : solved.observation_poses) boundaries[node] = p;
  auto bundle = candidate.bundle;
  if (bundle.observation_poses.count(source->id())) throw std::logic_error("Refresh capture was already committed");
  std::set<uint64_t> previously_observed;
  for (const auto& o : bundle.observations) previously_observed.insert(o.landmark);
  bundle.reject_observations(solved.rejected_observations);
  bundle.reference_node = target->id();
  for (const auto& [node, p] : solved.observation_poses) bundle.observation_poses[node] = p;
  bundle.observation_poses[target->id()] = StateGroupf::identity();
  bundle.observation_poses[source->id()] = solved.pose;
  const auto& frame = *candidate.query_features;
  const auto& camera = candidate.query.perspectives().front();
  const auto& variances = relocalizer.geometry().pixelVariances();
  const auto measurement_frame = camera::stereo_frame(camera, frame.keypoints, frame.points, variances, true);
  const auto observe = [&](uint64_t id, uint32_t row) {
    const auto& pixel = frame.keypoints.at(row);
    if (pixel.octave < 0 || size_t(pixel.octave) >= variances.size()) return;
    const camera::StereoObservation o{measurement_frame, row};
    featmap::Appearance sample;
    sample.descriptor = feature::LocalDescriptor::from_row(frame.descriptors, int(row), frame.descriptor_model_id);
    sample.tree_node = frame.tree_node(row);
    sample.word_id = frame.word_id(row);
    sample.quality = .5f;
    bundle.observations.push_back({source->id(), id, o, sample, frame.feature_ids.at(row)});
  };
  if (!candidate.redundant_submap)
    for (const auto& match : final_inliers) observe(candidate.target->points().at(match.landmark).id, match.feature);
  const auto coherent = featmap::refined_node_features(*candidate.target, candidate.bundle, bundle, solved.points, solved.point_covariances);
  auto content = prepare_node_content(candidate, solved.pose, coherent, final_inliers, rejected_rows);
  if (!candidate.redundant_submap) {
    for (const auto& [id, row] : content.inserted_rows) observe(id, row);
  } else {
    // Revalidated pose/association can attach the source submap, but its
    // redundant pixels must not perturb geometry or inflate observation counts.
    if (content.stats.eligible) throw std::runtime_error("New submap coverage appeared after redundant-node admission");
    // Keep the updated activity metadata even when no pixels need storing.
  }
  auto replacement = std::make_unique<Node>(*target);
  replacement->set_feature_content(std::move(content.features), std::move(bundle));
  replacement->retire_unsupported(previously_observed);
  if (!candidate.redundant_submap) {
    std::map<uint64_t, uint32_t> rows;
    for (uint32_t i = 0; i < replacement->map_features()->points().size(); ++i) rows[replacement->map_features()->points()[i].id] = i;
    std::vector<featmap::Match> seeds;
    for (auto m : final_inliers) {
      const auto row = rows.find(candidate.target->points().at(m.landmark).id);
      if (row != rows.end()) {
        m.landmark = row->second;
        seeds.push_back(m);
      }
    }
    replacement->fuse_landmarks(candidate.query, candidate.query_features, solved.pose.inverse(), seeds);
  }
  if (candidate.association_complete) {
    std::map<uint64_t, uint32_t> verified_rows;
    for (const auto& m : final_inliers) verified_rows.emplace(candidate.target->points().at(m.landmark).id, m.feature);
    // A redundant revisit is still reliability evidence. Do not retain duplicate
    // BA pixels merely to record its successful/failed feature confirmations.
    replacement->maintain_landmarks(candidate.query, candidate.query_features, source->id(), solved.pose.inverse(), variances, verified_rows);
  }
  replacement->prune_views(candidate.geometry.pyramid_scale, {source->id()});
  boundaries[source->id()] = solved.pose;
  replacement->update_descriptors(boundaries);
  replacement->observation_history().validate();
  return replacement;
}
}  // namespace

void prepare_node_previous(Memory& memory, const Relocalizer& relocalizer, NodeMergeCandidate& candidate, int previous_keyframe,
                           const std::map<int, StateGroupf>& poses) {
  if (!memory.node_params().merge_previous || !candidate.replacement || previous_keyframe <= 0) return;
  const int retained = candidate.retained, source = memory.topograph().submap_owner(previous_keyframe);
  if (source == retained || source == candidate.removed || !poses.count(source) || !poses.count(retained)) return;
  try {
    const auto archive = memory.load_observation_archive(source);
    if (archive.empty()) return;
    double load_ms = 0;
    memory.reactivateNodes({source}, 1, load_ms);
    const auto* owner = memory.getNode(source);
    if (!owner) throw std::runtime_error("Previous region owner unavailable");
    const auto bank = memory.map_features_for(source), target = candidate.replacement->map_features();
    const auto budget = memory.node_params().merge_max_comparisons;
    if (!bank || !budget || 2 * uint64_t(bank->points().size()) * target->points().size() > budget)
      throw featmap::MergeRejected("Region descriptor comparison budget exceeded");
    // The resident owner already holds this accepted history. The archive is
    // retained as the transaction's expected version; decoding it again would
    // rebuild every camera/frame and observation on each region comparison.
    featmap::NodeObservations loaded_history;
    if (!owner->has_observation_history()) loaded_history = featmap::NodeObservations::decode(archive);
    const auto& history = owner->has_observation_history() ? owner->observation_history() : loaded_history;
    cv::Mat descriptors(int(bank->points().size()), bank->descriptor_dimensions(), bank->descriptor_model_id() ? CV_32F : CV_8U);
    for (size_t row = 0; row < bank->points().size(); ++row) bank->points()[row].appearance.descriptor.copy_to(descriptors, int(row));
    auto keys = std::vector<cv::KeyPoint>(bank->points().size(), cv::KeyPoint(0, 0, 1));
    const auto query = bank->descriptor_model_id()
                           ? bow::FeatureBlock::create_learned_owned(std::move(keys), {}, std::move(descriptors), bank->descriptor_model_id())
                           : bow::FeatureBlock::create_raw_owned(std::move(keys), {}, std::move(descriptors));
    const auto proposed = relocalizer.geometry().match_local(*target, *query);
    const auto relative = poses.at(retained).between(poses.at(source));
    const auto pairs =
        featmap::supported_region_correspondences(*target, candidate.replacement->observation_history(), *bank, history, relative, proposed);
    if (pairs.size() < size_t(relocalizer.geometry().getMinVisualCorrespondences()))
      throw featmap::MergeRejected("Insufficient original-pixel support for previous region fusion");
    std::vector<camera::Perspective> cameras;
    memory.load_calibration(source, cameras);
    if (cameras.size() != 1) throw featmap::MergeRejected("Previous region camera unavailable");
    // Only the target has consumed this capture. Importing historical evidence
    // does not reactivate source-only points; shared identities keep the target's
    // actual observation state. Do not age the resident source before commit.
    auto historical_points = bank->points();
    for (auto& point : historical_points)
      if (point.miss_count != std::numeric_limits<uint32_t>::max()) ++point.miss_count;
    const auto historical_bank =
        featmap::LandmarkMap::create(bank->vocabulary_hash(), std::move(historical_points), bank->max_points(), bank->descriptor_model_id());
    auto merged = featmap::merge_node_regions(*target, candidate.replacement->observation_history(), *historical_bank, history, relative, pairs,
                                              cameras.front(), relocalizer.geometry().getMinVisualCorrespondences(), candidate.bundle_options);
    auto replacement = std::make_shared<Node>(*candidate.replacement);
    replacement->set_feature_content(merged.features, std::move(merged.bundle));
    replacement->prune_views(candidate.geometry.pyramid_scale, {candidate.removed});
    replacement->update_descriptors(replacement->observation_history().observation_poses);
    candidate.primary_replacement = candidate.replacement;
    candidate.replacement = std::move(replacement);
    if (candidate.removed) {
      candidate.region = source;
      candidate.expected_region = archive;
    } else {
      candidate.removed = source;
      candidate.expected_source = archive;
      candidate.expected_target = memory.load_observation_archive(retained);
    }
    candidate.bundle_ms += merged.refinement.elapsed_ms;
  } catch (const featmap::MergeRejected& e) {
    EXT_LOG_INFO("[region_fusion] keep separate owner={} target={}: {}", source, retained, e.what());
  }
}

NodeMergeCandidate prepare_node_regions(Memory& memory, const Relocalizer& relocalizer, int retained, int removed, const StateGroupf& relative,
                                        const std::map<uint64_t, uint64_t>& correspondences) {
  if (!memory.has_database() || retained == removed || memory.topograph().is_submap_member(retained) || memory.topograph().is_submap_member(removed))
    throw std::runtime_error("Two persistent canonical regions required");
  double load_ms = 0;
  memory.reactivateNodes({retained, removed}, 2, load_ms);
  if (!memory.getNode(retained) || !memory.getNode(removed)) throw std::runtime_error("Mature region owners unavailable");
  // Region content must not silently bridge disconnected world components.
  // Spatial topology is the graph's explicit constraints, never numeric ID order.
  const auto links = memory.getAllLinks(true, false);
  std::map<int, std::set<int>> adjacent;
  for (const auto& [from, edge] : links)
    if (edge.from() > 0 && edge.to() > 0 && edge.from() != edge.to()) {
      adjacent[edge.from()].insert(edge.to());
      adjacent[edge.to()].insert(edge.from());
    }
  std::set<int> reached{retained};
  std::deque<int> queue{retained};
  while (!queue.empty() && !reached.count(removed)) {
    const int id = queue.front();
    queue.pop_front();
    for (int neighbor : adjacent[id])
      if (reached.insert(neighbor).second) queue.push_back(neighbor);
  }
  if (!reached.count(removed)) throw std::runtime_error("Region merge requires an already connected world component");
  const auto target_archive = memory.load_observation_archive(retained), source_archive = memory.load_observation_archive(removed);
  if (target_archive.empty() || source_archive.empty()) throw std::runtime_error("Both regions must own mature observation history");
  const auto target = memory.map_features_for(retained), source = memory.map_features_for(removed);
  if (!target || !source) throw std::runtime_error("Missing mature region feature bank");
  std::vector<camera::Perspective> cameras;
  memory.load_calibration(removed, cameras);
  if (cameras.size() != 1) throw std::runtime_error("Mature region reference requires one recorded camera model");
  const auto& options = memory.local_ba_options();
  auto draft = featmap::merge_node_regions(*target, featmap::NodeObservations::decode(target_archive), *source,
                                           featmap::NodeObservations::decode(source_archive), relative, correspondences, cameras.front(),
                                           relocalizer.geometry().getMinVisualCorrespondences(), options);
  const float pyramid_scale = memory.feature_params().orb_pyramid_scale;
  auto replacement = std::make_unique<Node>(*memory.getNode(retained));
  replacement->set_feature_content(draft.features, std::move(draft.bundle));
  replacement->prune_views(pyramid_scale);
  replacement->update_descriptors(replacement->observation_history().observation_poses);
  draft.features = replacement->map_features();
  // The returned diagnostic owns its history; publication retains the same state.
  draft.bundle = replacement->observation_history();
  NodeMergeCandidate candidate;
  candidate.refresh = true;
  candidate.retained = retained;
  candidate.removed = removed;
  candidate.target = target;
  candidate.relative = draft.refinement.pose;
  candidate.covariance = draft.refinement.covariance;
  candidate.bundle_ms = draft.refinement.elapsed_ms;
  candidate.expected_target = target_archive;
  candidate.expected_source = source_archive;
  candidate.replacement = std::move(replacement);
  return candidate;
}

std::optional<NodeMergeCandidate> prepare_node_fusion(Memory& memory, const Relocalizer& relocalizer, const Node& query, int retained,
                                                      const Registration& registration, const std::map<int, StateGroupf>& poses, int previous_capture,
                                                      const LocalMapCollection* navimap) {
  if (!navimap && !memory.submap_refresh_enabled()) return {};
  const auto* reference = memory.getNode(retained);
  if (!reference || !reference->has_map_features() || !query.has_map_features() || (!registration.map_matches || registration.inliers.empty())) {
    EXT_LOG_DEBUG("[node_fusion] query={} target={} has no canonical natural-feature correspondences; retain the verified graph measurement",
                  query.id(), retained);
    return {};
  }
  if (memory.submap_refresh_enabled()) {
    auto candidate = prepare_node_refresh(memory, relocalizer, retained, query.id(), registration, poses);
    prepare_node_previous(memory, relocalizer, candidate, previous_capture, poses);
    return candidate;
  }
  if (!memory.topograph().isGraphReduced() || !navimap || !navimap->maps().count(retained)) return {};
  return prepare_node_merge(memory, relocalizer, retained, query.id(), registration);
}

}  // namespace sapphire::backend
