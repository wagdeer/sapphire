#include "backend/system/action/mapping.hpp"

#include <stdlib.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/system/action/build_frame.hpp>
#include <backend/system/action/build_node.hpp>
#include <backend/system/action/preprocess.hpp>
#include <backend/system/action/relocalizer.hpp>
#include <backend/system/action/sensor_runtime.hpp>
#include <backend/system/action/update_database.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/parameters.hpp>
#include <backend/params/archive.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/navimap.hpp>
#include <backend/params/node.hpp>
#include <backend/params/pgo.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/statistics.hpp>
#include <backend/params/topograph.hpp>
#include <backend/tools/associative.hpp>
#include <backend/tools/filesystem.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/parallel_executor.hpp>
#include <backend/tools/process.hpp>
#include <backend/tools/stats.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/timer.hpp>
#include <cctype>
#include <cstdint>
#include <optional>
#include <set>
#include <thread>

#include "backend/common/state_group.hpp"
#include "backend/database/compression.hpp"
#include "backend/database/driver.hpp"
#include "backend/registration/fast_orb_stereo.hpp"
#include "backend/maplayer/topograph.hpp"
#include "backend/maplayer/topograph/bayes_filter.hpp"
#include "backend/maplayer/node.hpp"
#include "backend/system/action/visual_registration.hpp"
#include "backend/memory.hpp"
#include "backend/registration/retrieval/fbow.hpp"
#include "backend/solver/pgo_solver.hpp"

namespace sapphire::backend {
std::set<int> Mapping::swap_results() {
  auto changes = _graphOptimizer ? _graphOptimizer->swap_results() : std::set<int>{};
  auto internal = topograph_.take_pose_changes();
  changes.merge(internal);
  auto content = localmaps_.take_changes();
  changes.merge(content);
  return changes;
}

namespace {

void stateGroupToXyzRpy(const StateGroupf& state, float& x, float& y, float& z, float& roll, float& pitch, float& yaw) {
  const StateGroupf::Vector3 translation = state.translationVector();
  const StateGroupf::Vector3 yawPitchRoll = state.rotationMatrix().eulerAngles(2, 1, 0);
  x = translation.x();
  y = translation.y();
  z = translation.z();
  roll = yawPitchRoll.z();
  pitch = yawPitchRoll.y();
  yaw = yawPitchRoll.x();
}

}  // namespace

Mapping::Mapping() = default;

Mapping::~Mapping() {
  try {
    close();
  } catch (const std::exception& error) {
    EXT_LOG_ERROR("[storage] Backend shutdown failed: {}", error.what());
    try {
      close(false);
    } catch (const std::exception& cleanup) {
      EXT_LOG_ERROR("[storage] Backend cleanup failed: {}", cleanup.what());
    }
  }
}

void Mapping::prepare_models(const ParametersMap& parameters, const RuntimeResources& resources) {
  if (_memory) throw std::logic_error("Prepare models before opening a map");
  const auto startup = LoadParameters::from_map(parameters);
  const auto feature = FeatureParams{}.withOverrides(startup);
  if (!feature.uses_xfeat()) {
    prepared_models_.reset();
    return;
  }
  if (prepared_models_) {
    prepared_models_->validate(feature, resources);
    return;
  }
  prepared_models_ = std::make_shared<feature::RknnModels>(feature, resources);
}

void Mapping::init(const ParametersMap& parameters, const std::string& databasePath, const RuntimeResources& resources) {
  if (_memory) throw std::logic_error("Close the map before initializing a new configuration");
  const auto allParameters = LoadParameters::from_map(parameters);
  const auto admission = NodeParams{}.withOverrides(allParameters);
  const auto archive = ArchiveParams{}.withOverrides(allParameters);
  const auto feature = FeatureParams{}.withOverrides(allParameters);
  const auto memory = MemoryParams{}.withOverrides(allParameters);
  const auto navimap = NavimapParams{}.withOverrides(allParameters);
  const auto pose_graph = PGOParams{}.withOverrides(allParameters);
  const auto preprocess = PreprocessParams{}.withOverrides(allParameters);
  const auto registration = RegistrationParams{}.withOverrides(allParameters);
  const auto relocalizer = RelocalizerParams{}.withOverrides(allParameters);
  const auto statistics = StatisticsParams{}.withOverrides(allParameters);
  const auto topograph = TopographParams{}.withOverrides(allParameters);
  if (topograph.persistent_root_frames && (pose_graph.anchor_at_last_node || pose_graph.max_iterations <= 0))
    throw std::invalid_argument("Persistent component frames require oldest datum and an enabled optimizer");
  try {
    node_params_ = admission;
    archive_params_ = archive;
    memory_params_ = memory;
    pose_graph_params_ = pose_graph;
    relocalizer_params_ = relocalizer;
    statistics_params_ = statistics;
    topograph_params_ = topograph;
    _parameters = allParameters;
    if (!databasePath.empty()) {
      EXT_ASSERT(tools::extension(databasePath).compare("db") == 0);
      EXT_LOG_DEBUG("[storage] Opening database={} use_previous_map={}", databasePath, topograph_params_.use_previous_map);
    } else {
      EXT_LOG_DEBUG("[storage] Using in-memory database; persistence requires an output path at close");
    }
    const bool newDatabase = databasePath.empty() || !tools::path_exists(databasePath);

    // If doesn't exist, create a memory
    if (!_memory) {
      prepare_models(allParameters, resources);
      auto frames = std::make_unique<FrameBuilder>(preprocess, feature, sensor_runtime_, prepared_models_);
      auto grids = std::make_unique<LocalGridMaker>(navimap);
      const size_t workers = archive_params_.parallel_compression ? std::min<size_t>(4, std::max(1U, std::thread::hardware_concurrency())) : 1;
      if (!archive_executor_ || archive_executor_->thread_count() != workers)
        archive_executor_ = std::make_shared<tools::ParallelExecutor<>>(workers);
      auto archives = std::make_unique<ArchiveWriter>(archive_params_, archive_executor_);
      auto memory = std::make_unique<Memory>(allParameters, &topograph_, resources);
      auto relocalizer = std::make_unique<Relocalizer>(*memory, relocalizer_params_, registration, feature, preprocess,
                                                       memory_params_.protected_local_ratio, topograph_params_.metric_enabled, prepared_models_);
      if (!memory->init(databasePath)) throw std::runtime_error("Cannot open map database");
      frame_builder_ = std::move(frames);
      grid_builder_ = std::move(grids);
      archive_writer_ = std::move(archives);
      relocalizer_ = std::move(relocalizer);
      _memory = std::move(memory);
    }

    std::map<int, StateGroupf> restored_poses;
    std::multimap<int, Link> restored_constraints;
    StateGroupf restored_last;
    _graphOptimizer = std::make_unique<solver::PoseGraphSolver>(pose_graph_params_);

    // Restore the metric snapshot before map publication or a no-input close.
    if (_memory && !newDatabase && topograph_params_.metric_enabled) {
      restored_poses = _memory->loadOptimizedPoses(&restored_last);
      if (!restored_poses.empty()) {
        const auto saved = restored_poses;
        _memory->getMetricConstraints(tools::key_set(restored_poses), restored_poses, restored_constraints, true);
        for (const auto& [id, pose] : saved)
          if (id < 0) restored_poses.emplace(id, pose);
      } else if (const auto* last = _memory->getLastWorkingNode(true)) {
        // Recover from a missing snapshot using the persisted graph, not raw odometry alone.
        std::optional<Mat66_t<double>> covariance;
        optimizeCurrentMap(last->id(), true, restored_poses, covariance, &restored_constraints);
      }
    }

    if (_memory->topograph().persistent_root_frames()) {
      // Restore only explicitly persisted component datums; do not invent a frame on load.
      for (const auto& [anchor, frame] : _memory->topograph().root_frames())
        if (!restored_poses.count(anchor)) throw std::runtime_error("Saved snapshot is missing a component datum; recovery required");
      auto pending = _memory->getAllNodeIds();
      while (!pending.empty()) {
        const auto connected = _memory->metric_component(*pending.begin());
        if (!connected.count(*pending.begin())) throw std::runtime_error("Component traversal omitted its seed");
        std::map<int, StateGroupf> component;
        for (int id : connected) {
          pending.erase(id);
          if (id > 0 && restored_poses.count(id)) component.emplace(id, restored_poses.at(id));
        }
        if (component.empty()) throw std::runtime_error("Component has no saved metric snapshot; explicit recovery required");
        const RootFrame* frame = topograph_.rootFrame(component);
        if (!frame) throw std::runtime_error("Saved component has no persistent datum; rebuild with persistent root frames enabled");
        topograph_.validateFramePose(component);
      }
    }

    topograph_.restore_solution(restored_poses, restored_constraints, restored_last);
    if (topograph_params_.use_previous_map && !topograph_.stored_poses().empty()) selectWorldComponent(topograph_.stored_poses().rbegin()->first);
    topograph_.activate_restored_world(topograph_params_.use_previous_map);
    relocalizer_->restore_visits();

    statistics_log_.open(
        resources.working_directory.empty() ? std::filesystem::path(databasePath).parent_path().string() : resources.working_directory,
        statistics_params_, newDatabase);

  } catch (...) {
    close(false);
    throw;
  }
}

std::map<int, StateGroupf> Mapping::optimizeOfflineComponents(const std::map<int, StateGroupf>& input, const std::multimap<int, Link>& links) const {
  std::map<int, StateGroupf> remaining = input, result;
  while (!remaining.empty()) {
    const auto seed = remaining.lower_bound(1);
    if (seed == remaining.end()) break;
    std::map<int, StateGroupf> component;
    std::multimap<int, Link> edges;
    std::set<int> nodes;
    for (const auto& [id, pose] : remaining)
      if (id > 0) nodes.insert(id);
    const auto connected = topograph_.metric_component(seed->first, nodes, [&](int id) {
      std::multimap<int, Link> adjacent;
      for (const auto& [key, edge] : links) {
        if (edge.from() == id)
          adjacent.emplace(edge.to(), edge);
        else if (edge.to() == id)
          adjacent.emplace(edge.from(), edge.inverse());
      }
      return adjacent;
    });
    for (int id : connected) component.emplace(id, remaining.at(id));
    for (const auto& entry : links) {
      const auto& edge = entry.second;
      if (connected.count(edge.from()) && connected.count(edge.to())) {
        edges.insert(entry);
      }
    }
    int root = seed->first;
    const auto* frame = topograph_.rootFrame(component);
    if (_memory->topograph().persistent_root_frames() && !frame) throw std::runtime_error("Offline graph has no component datum");
    _graphOptimizer->fixed_root = frame != nullptr;
    if (frame) {
      root = frame->anchor;
      component.at(root) = frame->frame_from_anchor;
    }
    auto optimized = component;
    std::optional<Mat66_t<double>> covariance;
    if (!solve_submaps(root, optimized, edges, covariance)) optimized.clear();
    if (optimized.empty()) return {};
    topograph_.validateFramePose(optimized);
    for (const auto& entry : component) remaining.erase(entry.first);
    result.insert(optimized.begin(), optimized.end());
  }
  return result;
}

void Mapping::selectWorldComponent(int reference) {
  std::set<int> connected;
  connected = _memory->metric_component(reference);
  topograph_.select_world_component(connected);
}

void Mapping::set_sensor_runtime(std::shared_ptr<SensorRuntime> runtime) {
  if (_memory) throw std::logic_error("Replace sensor runtime only while mapping is closed");
  sensor_runtime_ = std::move(runtime);
}

void Mapping::getCompleteGraph(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& links) const {
  if (!_memory) throw std::logic_error("No open graph");
  _memory->getMetricConstraints(_memory->getAllNodeIds(), poses, links, true);
  for (const auto& [id, pose] : topograph_.stored_poses())
    if (poses.count(id)) poses[id] = pose;
  for (const auto& [id, pose] : topograph_.world_poses())
    if (poses.count(id)) poses[id] = pose;
  for (const auto& [id, pose] : topograph_.poses())
    if (poses.count(id)) poses[id] = pose;
}

void Mapping::checkpoint() {
  if (!_memory) return;
  trajectory_writer_.flush(_memory.get());
  _memory->checkpoint_nodes();
  _memory->joinTrashThread();
  trajectory_writer_.reap(*_memory);
  const auto surviving = _memory->getAllNodeIds();
  topograph_.prepare_archive(surviving, _memory->current_odom_pose().isNull());
  topograph_.finish_archive(surviving, {});
  _memory->saveOptimizedPoses(topograph_.stored_poses(), topograph_.world_last_pose());
  if (relocalizer_) relocalizer_->save_visits();
  _memory->joinTrashThread();
}

void Mapping::append_lidar_odometry(double stamp, const StateGroupf& pose) {
  if (!localmaps_.active()) throw std::logic_error("LiDAR trajectory needs an admitted anchor");
  // Intermediate LIO poses carry no new geometric observation. Preserve the
  // native trajectory contract without applying footprint clearing or hits.
  auto grid = std::make_shared<LocalGrid>();
  grid->cellSize = _memory->navimap_params().cell_size_m;
  grid->skip_integration = true;
  append_localmap(localmaps_, stamp, pose, std::move(grid));
  trajectory_writer_.record(localmaps_, _memory.get());
}

bool Mapping::process_lidar(const SensorData& data, const StateGroupf& odom, const Mat66_t<double>& covariance) {
  if (!data.lidar_cloud || !grid_builder_) throw std::invalid_argument("LIO requires initialized mapping and frozen geometry");
  auto grid = preprocess_depth(data, odom, *grid_builder_, navimap_enabled_);
  return process_frame(data, odom, covariance, {}, {}, &localmaps_, std::move(grid), nullptr, false);
}

void Mapping::close(bool databaseSaved, const std::string& ouputDatabasePath) {
  // Drain owned map evidence before tearing down the storage owner. Failure
  // retains owners until the caller can safely close the failed task.
  if (databaseSaved && _memory) seal_localmap();
  previous_submap_keyframe_ = 0;
  previous_submap_map_ = -1;
  EXT_LOG_DEBUG("[storage] close save={}", databaseSaved);
  topograph_.end_observations();
  run_ = {};

  statistics_log_.close();

  if (_memory) {
    if (databaseSaved) {
      const auto surviving_ids = _memory->getAllNodeIds();
      topograph_.prepare_archive(surviving_ids, _memory->current_odom_pose().isNull());
      std::map<int, int> reduced;
      if (_memory->topograph().isGraphReduced()) _memory->incrementMapId(&reduced);
      topograph_.finish_archive(surviving_ids, reduced);
      _memory->saveOptimizedPoses(topograph_.stored_poses(), topograph_.world_last_pose());
      if (relocalizer_) relocalizer_->save_visits();
    }
    _memory->close(databaseSaved, ouputDatabasePath);
    relocalizer_.reset();
    frame_builder_.reset();
    grid_builder_.reset();
    archive_writer_.reset();
    _memory.reset();
  }
  localmaps_.clear();
  keyframe_gate_.reset();
  depth_stamp_ = 0;
  tracking_session_ = 0;
  trajectory_writer_.reset();
  topograph_.clear_publication();

  _graphOptimizer.reset();
  // Reset configuration without constructing another optimizer while closing.

  _parameters.clear();
}

void Mapping::setParameterRecord(const ParametersMap& parameters) {
  if (!_memory) throw std::logic_error("Open a map before setting its parameter record");
  _memory->setParameterRecord(parameters);
}

int Mapping::getLastLocationId() const {
  int id = 0;
  if (_memory) {
    id = _memory->getLastNodeId();
  }
  return id;
}

bool Mapping::isIDsGenerated() const {
  if (_memory) {
    return _memory->isIDsGenerated();
  }
  return sapphire::backend::NodeParams{}.generate_node_ids;
}

const Statistics& Mapping::getStatistics() const { return statistics_; }

StateGroupf Mapping::getPose(int locationId) const { return sapphire::backend::tools::value_or(topograph_.poses(), locationId, StateGroupf()); }

int Mapping::triggerNewMap() {
  seal_localmap();
  keyframe_gate_.reset();
  depth_stamp_ = 0;
  if (grid_builder_) grid_builder_->resetGroundReference();
  previous_submap_keyframe_ = 0;
  previous_submap_map_ = -1;
  int mapId = -1;
  if (_memory) {
    run_.distance_travelled = 0.0f;
    run_.distance_since_last_loop = 0.0f;

    std::map<int, int> reducedIds;
    mapId = _memory->incrementMapId(&reducedIds);
    topograph_.remove_reduced_world(reducedIds);
    EXT_LOG_INFO("[mapping] Started odometry segment: segment={}", mapId);

    if (relocalizer_) relocalizer_->reset();
    topograph_.start_session();
  }
  return mapId;
}

//============================================================
// MAIN LOOP
//============================================================

void Mapping::seal_localmap() {
  trajectory_writer_.flush(_memory.get());
  if (localmaps_.active())
    sapphire::backend::seal_localmap(localmaps_, [&](LocalMap& map) {
      if (_memory && !_memory->getDatabaseUrl().empty()) persist_localmap(*_memory, map);
    });
  if (_memory && _memory->database()) {
    _memory->joinTrashThread();
    trajectory_writer_.reap(*_memory);  // Stop/Save is a durability boundary.
  }
}

void Mapping::restore_localmaps(bool navimap_enabled, bool load_previous) {
  if (!_memory) throw std::logic_error("Initialize the action before restoring local maps");
  navimap_enabled_ = navimap_enabled;
  localmaps_.clear();
  localmaps_.configure(_parameters, navimap_enabled);
  trajectory_writer_.reset();
  if (load_previous) sapphire::backend::restore_localmaps(*_memory, localmaps_);
  localmaps_.advance_sequence(_memory->trajectory_high_water());
  trajectory_writer_.reset(localmaps_.observation_count());
}

ObservationResult Mapping::process(const KeyframeGate::Observation& tracking, const SensorData& data, const StateGroupf& odom,
                                   const Mat66_t<double>& covariance, const std::vector<float>& velocity, const std::map<std::string, float>& stats,
                                   std::shared_ptr<const LocalGrid> depth_grid) {
  ObservationResult info;
  const bool visual = !data.imageRaw().empty();
  if (visual) {
    if (relocalizer_ && (!odom.isValid() || !std::isfinite(data.stamp()) || data.stamp() <= 0)) relocalizer_->record_visit(0, 0, 0, 0, 0);
    if (!keyframe_gate_.evaluate(tracking).healthy) {
      if (relocalizer_) relocalizer_->record_visit(0, 0, 0, 0, 0);
      info.outcome = ObservationResult::Outcome::InvalidProtocol;
      return info;
    }
    if (tracking.session && tracking_session_ && tracking.session != tracking_session_) {
      triggerNewMap();
      info.session_changed = true;
    }
    tracking_session_ = tracking.session;
  } else {
    // Only vision establishes a new session/anchor. A delayed depth packet must
    // not reset the graph or replay an older frontend keyframe sequence.
    if (!tracking.session || !std::isfinite(data.stamp()) || data.stamp() <= depth_stamp_ || !odom.isValid() ||
        (tracking_session_ && tracking.session < tracking_session_)) {
      info.outcome = ObservationResult::Outcome::InvalidProtocol;
      return info;
    }
    if (tracking.session != tracking_session_ || !localmaps_.active()) {
      info.outcome = ObservationResult::Outcome::WaitingAnchor;
      return info;
    }
  }
  const bool insert = visual && keyframe_gate_.evaluate(tracking).insert;
  info.outcome = insert ? ObservationResult::Outcome::NodeRejected : ObservationResult::Outcome::Integrated;
  if (!insert && !localmaps_.active()) {
    info.outcome = ObservationResult::Outcome::WaitingAnchor;
    return info;
  }
  tools::Timer depth_timer;
  if (!depth_grid) {
    if (!grid_builder_) throw std::logic_error("Tracked processing requires an initialized backend");
    depth_grid = preprocess_depth(data, odom, *grid_builder_, navimap_enabled_, &info.depth);
  }
  info.depth_ms = depth_timer.lap_seconds() * 1000;
  info.skip_integration = depth_grid->skip_integration;
  info.ground_cells = depth_grid->ground.size();
  info.obstacle_cells = depth_grid->obstacles.size();
  info.free_cells = depth_grid->empty.size();
  if (!visual) {
    localmaps_.integrate_depth(odom, *depth_grid);
    depth_stamp_ = data.stamp();
    return info;
  }
  if (!data.depthRaw().empty()) depth_stamp_ = std::max(depth_stamp_, data.stamp());
  FrameBuilder::Result prepared;
  bool built = false;
  if (insert && keyframe_gate_.reference_stamp() > 0 && localmaps_.active() && topograph_.aligned() && odom.isValid() && _memory &&
      previous_submap_keyframe_ > 0) {
    const auto previous_odom = _memory->current_odom_pose();
    const bool reset = !previous_odom.isValid() || (!previous_odom.isIdentity() && odom.isIdentity()) ||
                       (topograph_params_.new_session_odometry_jump_m > 0 &&
                        previous_odom.between(odom).translationVector().norm() > topograph_params_.new_session_odometry_jump_m);
    // Original capture identity and immutable measurements remain the reference
    // even after its map content joins another owner. Never substitute canonical
    // map points, a BA camera estimate, or a corrected global pose here.
    const auto* reference = _memory->getNode(previous_submap_keyframe_);
    const auto* reference_camera =
        reference && reference->visualFrame().perspectives().size() == 1 ? &reference->visualFrame().perspectives().front() : nullptr;
    if (!reset && reference && !reference->isBadNode() && reference->getWeight() >= 0 && reference->loop_features() &&
        reference->getPose().isValid() && reference_camera && reference_camera->isValidForReprojection() &&
        reference_camera->cameraToBase().isValid() && data.perspectives().size() == 1 && data.perspectives().front().isValidForProjection() &&
        data.perspectives().front().cameraToBase().isValid()) {
      tools::Timer feature_timer;
      prepared = frame_builder_->build(data);
      built = true;
      info.feature_ms = feature_timer.elapsed_seconds() * 1000;
      if (prepared.cameras.size() == 1) {
        tools::Timer geometry_timer;
        const auto query_from_reference = odom.inverse().compose(reference->getPose());
        info.geometry_reference = reference->id();
        info.geometry =
            KeyframeGate::assess(*reference->loop_features(), *reference_camera, prepared.features, prepared.cameras.front(), query_from_reference,
                                 relocalizer_->geometry().pixelVariances(), relocalizer_->geometry().matching_options());
        info.geometry_ms = geometry_timer.elapsed_seconds() * 1000;
      }
    }
  }
  if (info.geometry.redundant) {
    info.outcome = ObservationResult::Outcome::Absorbed;
    // Consume the proposal, not the reference map. No
    // node/recall/solver/history mutation is reachable from this branch; the
    // same raw odom reaches navimap.
    append_localmap(localmaps_, data.stamp(), odom, std::move(depth_grid));
    trajectory_writer_.record(localmaps_, _memory.get());
    keyframe_gate_.accept(tracking);
    statistics_ = Statistics();
    statistics_.setMapCorrection(topograph_.correction());
    statistics_.addStatistic("Admission/Absorbed/", 1);
    statistics_.addStatistic("Admission/Matches/", info.geometry.matches);
    statistics_.addStatistic("Admission/Reference/", info.geometry_reference);
    statistics_.addStatistic("Admission/GeometryMs/", info.geometry_ms);
    statistics_.addStatistic("Admission/FeatureMs/", info.feature_ms);
    return info;
  }
  if (insert) {
    if (process_frame(data, odom, covariance, velocity, stats, &localmaps_, depth_grid, built ? &prepared : nullptr, false)) {
      trajectory_writer_.record(localmaps_, _memory.get(), true);
      keyframe_gate_.accept(tracking);
      info.outcome = ObservationResult::Outcome::NodeAdmitted;
      return info;
    }
  }
  if (localmaps_.active()) {
    append_localmap(localmaps_, data.stamp(), odom, std::move(depth_grid));
    trajectory_writer_.record(localmaps_, _memory.get());
  }
  return info;
}
bool Mapping::process(const SensorData& data, StateGroupf odomPose, const Mat66_t<double>& odomCovariance, const std::vector<float>& odomVelocity,
                      const std::map<std::string, float>& externalStats, LocalMapCollection* localmaps, std::shared_ptr<const LocalGrid> depth_grid) {
  return process_frame(data, odomPose, odomCovariance, odomVelocity, externalStats, localmaps, std::move(depth_grid), nullptr, true);
}

struct Mapping::LoopProposal {
  std::list<std::pair<int, int>> loopClosureLinksAdded;
  std::vector<Link> candidate_links;
  int loopClosureVisualInliers = 0;  // for statistics
  float loopClosureVisualInliersRatio = 0.0f;
  int loopClosureVisualMatches = 0;
  float loopClosureVisualVariance = 0.0f;
  float loopClosureLinearVariance = 0.0f;
  float loopClosureAngularVariance = 0.0f;
  float loopClosureVisualInliersDistribution = 0;

  std::optional<NodeMergeCandidate> submap_candidate;
  // Keep the initializing visual estimate privately until fusion is accepted.
  // A rejected rigid draft falls back to that original independent-node trial.
  StateGroupf registration_pose;
  std::optional<Mat66_t<double>> registration_covariance;
  int registration_source = 0;
  int loop_candidate = 0;
  bool spatial = false;
  int spatial_candidates_checked = 0;
  size_t spatial_insufficient_support = 0;
  double spatial_search_seconds = 0;
  double timeAddLoopClosureLink = 0;
};

Mapping::LoopProposal Mapping::prepare_loops(const Node& query, Relocalization& recognition, LocalMapCollection* localmaps) {
  LoopProposal proposal;
  proposal.loop_candidate = recognition.qualified_id;
  const auto* node = &query;
  const auto& graph_poses = topograph_.poses();
  // Pure LiDAR has no separate global appearance recall. Include archived
  // poses in the same published component, while keeping payload loads bounded.
  // Never mix a new unlocalized session with historical world coordinates.
  std::map<int, StateGroupf> lidar_poses;
  if (query.has_lidar() && topograph_.aligned()) {
    lidar_poses = topograph_.world_poses();
    for (const auto& [id, pose] : graph_poses) lidar_poses.insert_or_assign(id, pose);
  }
  const auto& spatial_poses = lidar_poses.empty() ? graph_poses : lidar_poses;
  tools::Timer timer;
  const auto prepare_fusion = [&](int retained, const Registration& registration, StateGroupf& transform, std::optional<Mat66_t<double>>& covariance,
                                  float max_distance) {
    if (query.has_lidar() || transform.isNull() || _graphOptimizer->options.max_iterations == 0 || proposal.submap_candidate) return std::string{};
    try {
      auto candidate = prepare_node_fusion(*_memory, *relocalizer_, query, retained, registration, graph_poses,
                                           previous_submap_map_ == query.mapId() ? previous_submap_keyframe_ : 0, localmaps);
      if (!candidate) return std::string{};
      // Reject the draft before attaching it to the proposal. Otherwise a
      // post-BA distance rejection can leave a stale draft blocking later nodes.
      const auto& refined = candidate->covariance ? candidate->relative : transform;
      if (max_distance > 0 && refined.translationNorm() > max_distance) {
        transform = {};
        covariance.reset();
        return std::string("Fused relative pose exceeds loop distance");
      }
      // Admission is already accepted. Persist its original measurements before
      // making a graph proposal, whose rejection must not erase the capture.
      _memory->saveLocationData(query.id());
      proposal.registration_pose = transform;
      proposal.registration_covariance = covariance;
      proposal.registration_source = retained;
      if (candidate->covariance) {
        transform = candidate->relative;
        covariance = candidate->covariance;
      }
      proposal.submap_candidate = std::move(candidate);
    } catch (const featmap::MergeRejected& error) {
      EXT_LOG_INFO("[node_fusion] candidate rejected query={} target={}: {}", query.id(), retained, error.what());
      // Natural geometry must pass the fusion solve before it becomes a loop.
      transform = {};
      covariance.reset();
      return std::string(error.what());
    }
    return std::string{};
  };

  //=============================================================
  // Global loop closure detection
  // (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
  //=============================================================
  std::vector<int> global_candidates;
  if (!recognition.scene_candidates.empty())
    global_candidates = recognition.scene_candidates;
  else if (proposal.loop_candidate > 0)
    global_candidates.push_back(proposal.loop_candidate);
  // Appearance candidates retain their existing acceptance policy. Spatial
  // fallback is loaded lazily and has its own bounded additional trial count.
  std::set<int> verified_candidates;
  const size_t appearance_count = global_candidates.size();
  bool spatial_selected = false;
  // Verify lazily and stop at the first geometry + fusion acceptance.
  proposal.loop_candidate = 0;
  for (size_t candidate_index = 0;; ++candidate_index) {
    if (candidate_index == global_candidates.size()) {
      if (spatial_selected || !relocalizer_params_.spatial_enabled || !topograph_params_.metric_enabled || query.getWeight() < 0 ||
          _graphOptimizer->options.max_iterations == 0)
        break;
      spatial_selected = true;
      tools::Timer spatial_timer;
      const auto spatial =
          relocalizer_->spatial_candidates(query, spatial_poses, recognition, verified_candidates, &proposal.spatial_insufficient_support);
      proposal.spatial_search_seconds = spatial_timer.elapsed_seconds();
      global_candidates.insert(global_candidates.end(), spatial.begin(), spatial.end());
      if (candidate_index == global_candidates.size()) break;
    }
    const bool spatial = candidate_index >= appearance_count;
    const float max_distance = spatial ? (relocalizer_params_.max_loop_distance_m > 0
                                              ? std::min(relocalizer_params_.spatial_radius_m, relocalizer_params_.max_loop_distance_m)
                                              : relocalizer_params_.spatial_radius_m)
                                       : relocalizer_params_.max_loop_distance_m;
    const int candidate = global_candidates[candidate_index];
    if (!verified_candidates.insert(candidate).second) continue;
    Registration verified;
    RegistrationTrace verified_trace;
    proposal.loop_candidate = candidate;
    proposal.spatial = spatial;
    if (!spatial) recognition.qualified_id = candidate;
    recognition.rejected = false;
    if (proposal.loop_candidate > 0 && node->hasLink(proposal.loop_candidate)) {
      // A fused anchor remains searchable in WM, so the next frame can retrieve
      // its own odometry neighbor. Memory::addLink() is idempotent: success does
      // not imply an edge was inserted. Never register an existing edge as a
      // provisional loop, or rejection/submap rollback can delete odometry.
      EXT_LOG_DEBUG("[loop] Ignoring loop candidate {:d}: already linked to current node {:d}", proposal.loop_candidate, node->id());
      proposal.loop_candidate = 0;
    }
    if (proposal.loop_candidate > 0 && topograph_params_.metric_enabled) {
      const auto appearance = std::find(recognition.scene_candidates.begin(), recognition.scene_candidates.end(), candidate);
      if (appearance != recognition.scene_candidates.end()) {
        const auto rank = size_t(appearance - recognition.scene_candidates.begin());
        if (rank < recognition.scene_candidate_scores.size())
          EXT_LOG_INFO("[scene_candidate] query={} reference={} appearance_rank={} cosine={:.6f}", node->id(), candidate, rank + 1,
                       recognition.scene_candidate_scores[rank]);
      }
      if (spatial) {
        ++proposal.spatial_candidates_checked;
        if (query.has_lidar() && !_memory->getNode(candidate)) {
          double database_seconds = 0;
          _memory->reactivateNodes({candidate}, 1, database_seconds);
        }
      }
      auto prior = !spatial && relocalizer_params_.identity_initial_guess ? StateGroupf::identity() : StateGroupf();
      if (query.has_lidar() && spatial && spatial_poses.count(candidate) && spatial_poses.count(query.id()))
        prior = spatial_poses.at(candidate).inverse().compose(spatial_poses.at(query.id()));
      verified = relocalizer_->verify(candidate, node->id(), prior, &verified_trace, std::nullopt);
      if (verified.pose.isNull() || (!spatial && max_distance > 0 && verified.pose.translationNorm() > max_distance)) {
        EXT_LOG_INFO("[loop] Rejected candidate reference={} query={} reason={}", candidate, node->id(), verified.failure);
        proposal.loop_candidate = 0;
      }
    }
    if (proposal.loop_candidate > 0) {
      if (proposal.candidate_links.empty() && (!localmaps || !proposal.submap_candidate)) {
        // Compute transform if metric data are present
        StateGroupf transform;
        RegistrationTrace trace;
        Registration registration;
        std::optional<Mat66_t<double>> covariance = Mat66_t<double>::Identity();
        if (topograph_params_.metric_enabled) {
          registration = std::move(verified);
          trace = std::move(verified_trace);
          transform = registration.pose;
          covariance = registration.covariance;
          const auto merge_failure = prepare_fusion(proposal.loop_candidate, registration, transform, covariance, max_distance);

          statistics_.addStatistic(Statistics::kLoopBow_comparison_bound(), trace.bow_comparison_bound);
          statistics_.addStatistic(Statistics::kLoopBow_comparisons(), trace.bow_comparisons);

          proposal.loopClosureVisualInliersDistribution = trace.inlier_distribution;

          proposal.loopClosureVisualInliers = trace.inliers.size();
          proposal.loopClosureVisualInliersRatio = trace.inlier_ratio;
          proposal.loopClosureVisualMatches = trace.match_count;
          proposal.loopClosureVisualVariance = (!covariance.has_value() ? 0 : (*covariance)(0, 0));
          recognition.rejected = transform.isNull();
          if (recognition.rejected) {
            EXT_LOG_INFO("[loop] Rejected loop closure {:d} -> {:d}: {}", proposal.loop_candidate, node->id(),
                         (merge_failure.empty() ? registration.failure : merge_failure).c_str());
          } else if (max_distance > 0.0f && transform.translationNorm() > max_distance) {
            recognition.rejected = true;
            EXT_LOG_INFO("[loop] Rejected loop closure {:d} -> {:d} because its transform ({:.6f}m) is over {}={:.6f}m.", proposal.loop_candidate,
                         node->id(), transform.translationNorm(), "candidate_distance_limit", max_distance);
          }
        }
        if (!recognition.rejected) {
          // Make the new one the parent of the old one
          EXT_ASSERT(covariance.has_value() && (*covariance)(0, 0) > 0.0 && (*covariance)(5, 5) > 0.0);

          proposal.loopClosureLinearVariance = std::max(
              {(*covariance)(0, 0), (*covariance)(1, 1) >= 9999 ? 0 : (*covariance)(1, 1), (*covariance)(2, 2) >= 9999 ? 0 : (*covariance)(2, 2)});
          proposal.loopClosureAngularVariance = std::max(
              {(*covariance)(3, 3) >= 9999 ? 0 : (*covariance)(3, 3), (*covariance)(4, 4) >= 9999 ? 0 : (*covariance)(4, 4), (*covariance)(5, 5)});
          Mat66_t<double> information = getInformation(*covariance);
          recognition.geometry_verified_id = proposal.loop_candidate;
          // The verified relative pose is the only graph measurement. Detector
          // poses are initializers; raw pixels remain in capture/BA history.
          proposal.candidate_links.push_back(Link(proposal.loop_candidate, node->id(), Link::kLoopClosure, transform, information).inverse());
          proposal.loopClosureLinksAdded.emplace_back(node->id(), proposal.loop_candidate);
        }

        if (recognition.rejected) {
          proposal.loop_candidate = 0;
        }
      } else {
        proposal.loop_candidate = 0;
      }
    }
    if (proposal.loop_candidate > 0) break;
  }

  if (proposal.loop_candidate && proposal.submap_candidate && proposal.submap_candidate->retained == proposal.loop_candidate)
    recognition.geometry_verified_id = proposal.loop_candidate;
  proposal.timeAddLoopClosureLink = timer.lap_seconds();

  return proposal;
}

struct Mapping::GraphUpdate {
  explicit GraphUpdate(const Topograph& graph)
      : poses(graph.poses()),
        constraints(graph.constraints()),
        correction(graph.correction()),
        mapped(graph.last_mapped_pose()),
        covariance(graph.correction_covariance()) {}
  std::map<int, StateGroupf> poses;
  std::multimap<int, Link> constraints;
  StateGroupf correction, mapped, previous_correction;
  std::optional<Mat66_t<double>> covariance;
  std::vector<std::pair<int, int>> repaired_links;
  graph::MaxGraphErrors maxGraphErrors;
  std::pair<int, int> maxGraphErrorsLinearIds{0, 0};
  std::pair<int, int> maxGraphErrorsAngularIds{0, 0};
  std::pair<int, int> maxGraphErrorsRemovedIds{0, 0};
  int maxGraphErrorsRemovedCount = 0;
  double optimizationError = 0.0;
  int optimizationIterations = 0;
  std::size_t optimizationPoseCount = 0;
  std::size_t optimizationConstraintCount = 0;
  std::uint32_t optimizationTriggerMask = 0U;
  std::string optimizationTriggers;
  bool graphOptimizationExecuted = false;
  std::optional<sapphire::backend::solver::PoseGraphSolver::Trial> optimizer_trial;
};

void Mapping::optimize_observation(const Node& query, LoopProposal& loops, Relocalization& recognition, bool reduced, GraphUpdate& update) {
  const auto* node = &query;
  const auto appendOptimizationTrigger = [&update](const char* trigger, Statistics::OptimizationTrigger flag) {
    if (!update.optimizationTriggers.empty()) {
      update.optimizationTriggers += ',';
    }
    update.optimizationTriggers += trigger;
    update.optimizationTriggerMask |= static_cast<std::uint32_t>(flag);
  };
  const bool hasPosePrior = node->hasLink(node->id(), Link::kPosePrior) && !_graphOptimizer->options.ignore_priors;
  const bool hasGravityConstraint =
      node->hasLink(node->id(), Link::kGravity) && _graphOptimizer->options.gravity_sigma > 0.0f && !topograph_params_.use_odometry_gravity;
  if (loops.loop_candidate > 0) appendOptimizationTrigger(loops.spatial ? "spatial_loop" : "global_loop", Statistics::kOptimizationGlobalLoop);
  if (reduced) appendOptimizationTrigger("reduced_nodes", Statistics::kOptimizationReducedNodes);
  if (hasPosePrior) appendOptimizationTrigger("pose_prior", Statistics::kOptimizationPosePrior);
  if (hasGravityConstraint) appendOptimizationTrigger("gravity", Statistics::kOptimizationGravity);
  if (!recognition.loaded.empty()) appendOptimizationTrigger("retrieved_nodes", Statistics::kOptimizationRetrievedNodes);

  if (topograph_params_.metric_enabled && node->getWeight() != -1 && !update.optimizationTriggers.empty()) {
    if (!loops.loopClosureLinksAdded.empty()) update.optimizer_trial.emplace(*_graphOptimizer);
    update.graphOptimizationExecuted = true;
    EXT_ASSERT(((update.poses).find(node->id()) != (update.poses).end()));

    std::map<int, StateGroupf> poses = update.poses;

    std::multimap<int, Link> constraints;
    std::optional<Mat66_t<double>> covariance;
    // WM eviction does not remove the historical graph. A verified loop must
    // propagate through cold nodes too; this reads poses/links, not feature banks.
    const bool global_correction = _memory->topograph().persistent_root_frames() || loops.loop_candidate > 0;
    const auto* fusion =
        loops.submap_candidate && loops.submap_candidate->replacement ? &loops.submap_candidate->replacement->observation_history() : nullptr;
    optimizeCurrentMap(node->id(), global_correction, poses, covariance, &constraints, &update.optimizationError, &update.optimizationIterations,
                       loops.candidate_links, fusion);
    while (fusion) {
      const auto errors = poses.empty() ? graph::MaxGraphErrors{} : graph::computeMaxGraphErrors(poses, constraints);
      // Disabling loop rejection does not license an inconsistent rigid model.
      const double limit = pose_graph_params_.max_constraint_error_sigma > 0 ? pose_graph_params_.max_constraint_error_sigma
                                                                             : sapphire::backend::PGOParams{}.max_constraint_error_sigma;
      if (!poses.empty() && errors.linearRatio <= limit && errors.angularRatio <= limit) break;
      EXT_LOG_INFO("[submap_fusion] rigid trial rejected query={} owner={} linear_sigma={} angular_sigma={}", node->id(), fusion->reference_node,
                   errors.linearRatio, errors.angularRatio);
      if (loops.submap_candidate->region && loops.submap_candidate->primary_replacement) {
        loops.submap_candidate->replacement = std::move(loops.submap_candidate->primary_replacement);
        loops.submap_candidate->region = 0;
        loops.submap_candidate->expected_region = cv::Mat();
        fusion = &loops.submap_candidate->replacement->observation_history();
      } else {
        loops.submap_candidate.reset();
        fusion = nullptr;
        // Only supersede the new same-evidence loop. Never exempt other internal
        // visual edges, odometry, or external constraints.
        if (loops.registration_source) {
          for (auto& edge : loops.candidate_links) {
            const bool forward = edge.from() == loops.registration_source && edge.to() == node->id();
            const bool backward = edge.to() == loops.registration_source && edge.from() == node->id();
            if ((forward || backward) && (edge.isLoopClosure())) {
              const Link original(loops.registration_source, node->id(), edge.type(), loops.registration_pose,
                                  getInformation(*loops.registration_covariance));
              edge = forward ? original : original.inverse();
            }
          }
        }
      }
      update.optimizer_trial.reset();
      update.optimizer_trial.emplace(*_graphOptimizer);
      poses = update.poses;
      constraints.clear();
      optimizeCurrentMap(node->id(), global_correction, poses, covariance, &constraints, &update.optimizationError, &update.optimizationIterations,
                         loops.candidate_links, fusion);
    }
    update.optimizationPoseCount = poses.size();
    update.optimizationConstraintCount = constraints.size();

    // Check added loop closures have broken the graph
    // (in case of wrong loop closures).
    bool updateConstraints = true;
    if (poses.empty()) {
      EXT_LOG_WARN("[graph] Optimization failed; reverting trial loop constraints");
      for (std::list<std::pair<int, int>>::iterator iter = loops.loopClosureLinksAdded.begin(); iter != loops.loopClosureLinksAdded.end(); ++iter) {
        if (iter->second < 0) _memory->topograph().removeLink(iter->first, iter->second);
      }
      updateConstraints = false;
      loops.loop_candidate = 0;
      recognition.rejected = true;
    } else if (loops.loopClosureLinksAdded.size() && update.optimizationIterations > 0 && constraints.size()) {
      update.maxGraphErrors = graph::computeMaxGraphErrors(poses, constraints);
      if (!update.maxGraphErrors.linearLink.isValid() && !update.maxGraphErrors.angularLink.isValid()) {
        EXT_LOG_WARN("[graph] Cannot check residuals: no valid constraints; loop consistency unverified");
      }

      bool reject = false;
      if (update.maxGraphErrors.linearLink.isValid()) {
        update.maxGraphErrorsLinearIds = std::make_pair(update.maxGraphErrors.linearLink.from(), update.maxGraphErrors.linearLink.to());
        EXT_LOG_DEBUG("[graph] Max optimization linear error = {:.6f} m (link {:d}->{:d}, var={:.6f}, ratio error/std={:.6f})",
                      update.maxGraphErrors.linear, update.maxGraphErrors.linearLink.from(), update.maxGraphErrors.linearLink.to(),
                      update.maxGraphErrors.linearLink.transVariance(),
                      update.maxGraphErrors.linear / sqrt(update.maxGraphErrors.linearLink.transVariance()));
        if (pose_graph_params_.max_constraint_error_sigma > 0.0f &&
            update.maxGraphErrors.linearRatio > pose_graph_params_.max_constraint_error_sigma) {
          if (!loops.submap_candidate &&
              topograph_.submap_owner(update.maxGraphErrorsLinearIds.first) != topograph_.submap_owner(update.maxGraphErrorsLinearIds.second) &&
              pose_graph_params_.repair_radius_m > 0.0 && update.maxGraphErrorsLinearIds == topograph_.rejected_edge() &&
              graph::findLink(constraints, update.maxGraphErrorsLinearIds.first, update.maxGraphErrorsLinearIds.second) != constraints.end()) {
            EXT_LOG_WARN("[graph] Repeated excessive residual on constraint {:d} -> {:d}; attempting graph repair",
                         update.maxGraphErrorsLinearIds.first, update.maxGraphErrorsLinearIds.second);

            std::list<std::pair<int, int>> removedLinks =
                repairGraph(update.maxGraphErrors, poses, constraints, update.optimizationError, update.optimizationIterations, covariance);

            if (removedLinks.empty()) {
              EXT_LOG_WARN("[graph] Repair failed; rejecting trial loop constraints");
              reject = true;
            } else {
              for (auto link : removedLinks) {
                EXT_LOG_WARN("[graph] Repair removed constraint: {:d} -> {:d}", link.first, link.second);
                update.repaired_links.push_back(link);
              }
              EXT_LOG_INFO("[graph] Repair succeeded; conflicting constraints removed");
              update.maxGraphErrorsRemovedIds = removedLinks.front();
              update.maxGraphErrorsRemovedCount = removedLinks.size();
            }
          } else {
            reject = true;
          }

          if (reject) {
            EXT_LOG_WARN(
                "[graph] Rejecting {:d} trial loop constraints (first {:d} -> {:d}): residual_sigma_ratio={:.3f}; "
                "worst_constraint={:d} -> {:d} type={:d} position_error_m={:.6f} sigma_m={:.6f}; threshold {} = {:.3f}",
                (int)loops.loopClosureLinksAdded.size(), loops.loopClosureLinksAdded.front().first, loops.loopClosureLinksAdded.front().second,
                update.maxGraphErrors.linearRatio, update.maxGraphErrors.linearLink.from(), update.maxGraphErrors.linearLink.to(),
                update.maxGraphErrors.linearLink.type(), update.maxGraphErrors.linear, sqrt(update.maxGraphErrors.linearLink.transVariance()),
                "pose_graph.max_constraint_error_sigma", pose_graph_params_.max_constraint_error_sigma);

            if (update.maxGraphErrors.linearLink.isLoopClosure()) {
              topograph_.record_rejected_edge(update.maxGraphErrorsLinearIds);
            }
          }
        } else if (pose_graph_params_.max_constraint_error_sigma == 0.0f && update.maxGraphErrors.linearRatio > 100 &&
                   !_graphOptimizer->options.robust) {
          EXT_LOG_ERROR(
              "[graph] Excessive position residual accepted: residual_sigma_ratio={:.3f} constraint={:d} -> {:d} type={:d} error_m={:.6f}; "
              "sigma_m={:.6f}; rejection threshold {} disabled",
              update.maxGraphErrors.linearRatio, update.maxGraphErrors.linearLink.from(), update.maxGraphErrors.linearLink.to(),
              update.maxGraphErrors.linearLink.type(), update.maxGraphErrors.linear, sqrt(update.maxGraphErrors.linearLink.transVariance()),
              "pose_graph.max_constraint_error_sigma");
        }
      }

      if (update.maxGraphErrors.angularLink.isValid()) {
        update.maxGraphErrorsAngularIds = std::make_pair(update.maxGraphErrors.angularLink.from(), update.maxGraphErrors.angularLink.to());
        EXT_LOG_DEBUG("[graph] Max optimization angular error = {:.6f} deg (link {:d}->{:d}, var={:.6f}, ratio error/std={:.6f})",
                      update.maxGraphErrors.angular * 180.0f / CV_PI, update.maxGraphErrors.angularLink.from(),
                      update.maxGraphErrors.angularLink.to(), update.maxGraphErrors.angularLink.rotVariance(),
                      update.maxGraphErrors.angular / sqrt(update.maxGraphErrors.angularLink.rotVariance()));
        if (pose_graph_params_.max_constraint_error_sigma > 0.0f &&
            update.maxGraphErrors.angularRatio > pose_graph_params_.max_constraint_error_sigma) {
          EXT_LOG_WARN(
              "[graph] Rejecting {:d} trial loop constraints (first {:d} -> {:d}): residual_sigma_ratio={:.3f}; "
              "worst_constraint={:d} -> {:d} type={:d} angular_error_deg={:.6f} sigma_rad={:.6f}; threshold {} = {:.3f}",
              (int)loops.loopClosureLinksAdded.size(), loops.loopClosureLinksAdded.front().first, loops.loopClosureLinksAdded.front().second,
              update.maxGraphErrors.angularRatio, update.maxGraphErrors.angularLink.from(), update.maxGraphErrors.angularLink.to(),
              update.maxGraphErrors.angularLink.type(), update.maxGraphErrors.angular * 180.0f / CV_PI,
              sqrt(update.maxGraphErrors.angularLink.rotVariance()), "pose_graph.max_constraint_error_sigma",
              pose_graph_params_.max_constraint_error_sigma);
          reject = true;
        } else if (pose_graph_params_.max_constraint_error_sigma == 0.0f && update.maxGraphErrors.angularRatio > 100 &&
                   !_graphOptimizer->options.robust) {
          EXT_LOG_ERROR(
              "[graph] Excessive angular residual accepted: residual_sigma_ratio={:.3f} constraint={:d} -> {:d} type={:d} error_deg={:.6f}; "
              "sigma_rad={:.6f}; rejection threshold {} disabled",
              update.maxGraphErrors.angularRatio, update.maxGraphErrors.angularLink.from(), update.maxGraphErrors.angularLink.to(),
              update.maxGraphErrors.angularLink.type(), update.maxGraphErrors.angular * 180.0f / CV_PI,
              sqrt(update.maxGraphErrors.angularLink.rotVariance()), "pose_graph.max_constraint_error_sigma");
        }
      }

      // Removing a rejected private candidate requires no committed edit. A
      // historical repair, however, must be applicable by its state owner in
      // full. Never publish poses from a repair while silently skipping a cold
      // endpoint.
      update.repaired_links.erase(std::remove_if(update.repaired_links.begin(), update.repaired_links.end(),
                                                 [&](const auto& removed) {
                                                   return std::any_of(
                                                       loops.candidate_links.begin(), loops.candidate_links.end(), [&](const Link& candidate) {
                                                         return (candidate.from() == removed.first && candidate.to() == removed.second) ||
                                                                (candidate.to() == removed.first && candidate.from() == removed.second);
                                                       });
                                                 }),
                                  update.repaired_links.end());
      for (const auto& edge : update.repaired_links) {
        if ((edge.first > 0 && !_memory->getNode(edge.first)) || (edge.second > 0 && !_memory->getNode(edge.second))) {
          EXT_LOG_WARN("[graph] Reject trial: repair endpoint {}->{} is not resident", edge.first, edge.second);
          reject = true;
        }
      }

      if (reject) {
        update.maxGraphErrorsRemovedIds = {0, 0};
        update.maxGraphErrorsRemovedCount = 0;
        for (std::list<std::pair<int, int>>::iterator iter = loops.loopClosureLinksAdded.begin(); iter != loops.loopClosureLinksAdded.end(); ++iter) {
          if (iter->second < 0) _memory->topograph().removeLink(iter->first, iter->second);
        }
        updateConstraints = false;
        loops.loop_candidate = 0;
        recognition.rejected = true;
      }
    }

    if (updateConstraints) {
      EXT_LOG_DEBUG("[graph] Updated local map (old size={:d}, new size={:d})", (int)update.poses.size(), (int)poses.size());
      update.poses = poses;
      update.constraints = constraints;
      update.covariance = covariance;
      loops.candidate_links.erase(
          std::remove_if(loops.candidate_links.begin(), loops.candidate_links.end(),
                         [&](const Link& edge) { return graph::findLink(constraints, edge.from(), edge.to()) == constraints.end(); }),
          loops.candidate_links.end());
      if (loops.loop_candidate && graph::findLink(constraints, node->id(), loops.loop_candidate) == constraints.end()) loops.loop_candidate = 0;
      if (loops.submap_candidate && graph::findLink(constraints, node->id(), loops.submap_candidate->retained) == constraints.end())
        loops.submap_candidate.reset();

    } else {
      loops.candidate_links.clear();
      update.repaired_links.clear();
      update.optimizer_trial.reset();
    }

    // Update map correction, it should be identify when optimizing from the last node
    EXT_ASSERT(update.poses.find(node->id()) != update.poses.end());
    update.previous_correction = update.correction;
    update.correction = update.poses.at(node->id()).compose(node->getPose().inverse());
    update.mapped = update.poses.at(node->id());  // update
    if (update.correction.translationSquaredNorm() > 0.1f && pose_graph_params_.anchor_at_last_node) {
      bool hasPrior = node->hasLink(node->id());
      if (!_graphOptimizer->options.ignore_priors) {
        for (std::multimap<int, Link>::reverse_iterator iter = update.constraints.rbegin(); !hasPrior && iter != update.constraints.rend(); ++iter) {
          if (iter->second.type() == Link::kPosePrior) {
            hasPrior = true;
          }
        }
      }
      if ((!hasPrior || _graphOptimizer->options.ignore_priors) && _graphOptimizer->options.gravity_sigma == 0.0f) {
        EXT_LOG_ERROR("[graph] Map correction should be identity when optimizing from the last node. T={}", update.correction.debugString().c_str());
      }
    }
  }
}

bool Mapping::commit_node_fusion(int current_id, const StateGroupf& relative, LocalMapCollection& localmaps, LoopProposal& loops,
                                 GraphUpdate& graph) {
  const int retained = loops.submap_candidate->retained;
  try {
    const int removed = current_id;
    const auto* target = _memory->getNode(retained);
    const auto* source = _memory->getNode(removed);
    if (!target || !source || source->loop_features() != loops.submap_candidate->query_features ||
        (target->map_features() && target->map_features() != loops.submap_candidate->target))
      throw std::runtime_error("Changed submap feature snapshot");
    const auto content = prepare_node_content(*loops.submap_candidate, relative);
    const auto local = localmaps.prepare_merge(retained, removed, relative);
    auto next_poses = graph.poses;
    next_poses.erase(removed);
    std::multimap<int, Link> next_constraints;
    std::set<std::tuple<int, int, int>> seen;
    const auto add_constraint = [&](const Link& edge) {
      if (!next_poses.count(edge.from()) || !next_poses.count(edge.to())) return;
      auto key =
          std::make_tuple(std::min(edge.from(), edge.to()), std::max(edge.from(), edge.to()), edge.from() == edge.to() ? int(edge.type()) : -1);
      if (seen.insert(key).second) next_constraints.emplace(edge.from(), edge);
    };
    // Preserve the optimizer's original edge orientation and residual chart.
    // Unchanged constraints take precedence over redirected duplicates.
    for (const auto& entry : graph.constraints) {
      const auto& edge = entry.second;
      if (edge.from() != removed && edge.to() != removed) add_constraint(edge);
    }
    for (const auto& entry : graph.constraints) {
      const auto& edge = entry.second;
      if (edge.from() != removed && edge.to() != removed) continue;
      if ((edge.from() == retained && edge.to() == removed) || (edge.from() == removed && edge.to() == retained)) continue;
      add_constraint(edge.reanchor(retained, removed, relative));
    }

    auto update = _memory->prepare_node_merge(retained, removed, content, local.merged(), relative, next_constraints, loops.candidate_links,
                                              graph.repaired_links);
    std::vector<Node*> writes;
    for (auto& entry : update.nodes) writes.push_back(entry.second.get());
    if (!_memory->can_apply(update) || !localmaps.can_apply(local)) throw std::runtime_error("Expired submap owners");
    auto published_poses = next_poses;
    auto published_constraints = next_constraints;
    if (const auto& database = _memory->database())
      persist_submap_merge(*database, retained, removed, writes, local, next_poses, graph.mapped, relative);
    _memory->apply_submap_merge(std::move(update));
    if (!localmaps.apply_merge(local)) std::terminate();
    graph.poses.swap(next_poses);
    graph.constraints.swap(next_constraints);
    topograph_.commit_solution(published_poses, published_constraints, graph.correction, graph.covariance, graph.mapped);
    loops.candidate_links.clear();
    graph.repaired_links.clear();
    if (graph.optimizer_trial) graph.optimizer_trial->commit();
    return true;
  } catch (const featmap::MergeRejected& error) {
    EXT_LOG_INFO("[graph] Node candidate {} -> {} rejected: {}", current_id, retained, error.what());
  } catch (...) {
    // Only admitted observations survive; the private graph and optimizer
    // trial roll back while unwinding. Never publish provisional statistics.
    statistics_ = Statistics{};
    throw;
  }
  return false;
}

bool Mapping::process_frame(const SensorData& data, StateGroupf odomPose, const Mat66_t<double>& odomCovariance,
                            const std::vector<float>& odomVelocity, const std::map<std::string, float>& externalStats, LocalMapCollection* localmaps,
                            std::shared_ptr<const LocalGrid> depth_grid, FrameBuilder::Result* prepared, bool allow_rehearsal) {
  //============================================================
  // Initialization
  //============================================================
  tools::Timer timer;
  tools::Timer timerTotal;
  Relocalization recognition;
  double timeMemoryUpdate = 0;
  double timeCleaningNeighbors = 0;
  double timeReactivations = 0;
  double timeMapOptimization = 0;
  double timeRealTimeLimitReachedProcess = 0;
  double timeMemoryCleanup = 0;
  double timeEmptyingTrash = 0;
  double timeFinalizingStatistics = 0;
  double timeJoiningTrash = 0;
  double timeStatsCreation = 0;

  const Node* node = 0;
  const Node* sLoop = 0;

  topograph_.commit_loop(0, 0);
  const auto lastHighestHypothesis = topograph_.hypothesis();
  topograph_.record_hypothesis({});

  statistics_ = Statistics();  // reset
  for (std::map<std::string, float>::const_iterator iter = externalStats.begin(); iter != externalStats.end(); ++iter) {
    statistics_.addStatistic(iter->first, iter->second);
  }

  //============================================================
  // Wait for an image...
  //============================================================

  timer.reset();
  timerTotal.reset();

  EXT_ASSERT_MSG(_memory, "[mapping] Mapping backend is not initialized");
  EXT_ASSERT_MSG(_graphOptimizer, "[mapping] Mapping backend is not initialized");

  //============================================================
  // If RGBD SLAM is enabled, a pose must be set.
  //============================================================
  if (topograph_params_.metric_enabled) {
    if (!odomPose.isNull()) {
      // this will make sure that all inverse operations will work!
      if (!odomPose.isValid()) {
        EXT_LOG_WARN("[input] Invalid rotation: node={} stamp={:.6f}; attempting normalization; original_pose={}", data.id(), data.stamp(),
                     odomPose.debugString());
        odomPose = odomPose.normalized();
        EXT_ASSERT_MSG(odomPose.isValid(),
                       "Odometry pose is not invertible! {}\n"
                       "[{:.6f} {:.6f} {:.6f} {:.6f};\n"
                       " {:.6f} {:.6f} {:.6f} {:.6f};\n"
                       " {:.6f} {:.6f} {:.6f} {:.6f};\n"
                       " 0 0 0 1]",
                       odomPose.debugString().c_str(), odomPose.rotation(0, 0), odomPose.rotation(0, 1), odomPose.rotation(0, 2), odomPose.x(),
                       odomPose.rotation(1, 0), odomPose.rotation(1, 1), odomPose.rotation(1, 2), odomPose.y(), odomPose.rotation(2, 0),
                       odomPose.rotation(2, 1), odomPose.rotation(2, 2), odomPose.z());
        EXT_LOG_DEBUG("[input] Rotation normalized: node={} corrected_pose={}", data.id(), odomPose.debugString());
      }
    }

    if (odomPose.isNull()) {
      EXT_LOG_ERROR("[input] Skipping observation: node={} missing frontend pose", data.id());
      relocalizer_->record_visit(0, 0, 0, 0, 0);
      return false;
    } else {
      // Detect if the odometry is reset. If yes, trigger a new map.
      if (_memory->getLastWorkingNode(false)) {
        const StateGroupf lastPose = _memory->current_odom_pose();  // current visit, including a fused anchor

        // look for identity
        if (!lastPose.isNull() && !lastPose.isIdentity() && odomPose.isIdentity()) {
          int mapId = triggerNewMap();
          EXT_LOG_WARN("[input] Odometry returned to origin; started segment={:d}", mapId);
        } else if (!lastPose.isNull() && topograph_params_.new_session_odometry_jump_m > 0.0) {
          // look for large change
          StateGroupf lastPoseToNewPose = lastPose.between(odomPose);
          float x, y, z, roll, pitch, yaw;
          stateGroupToXyzRpy(lastPoseToNewPose, x, y, z, roll, pitch, yaw);
          if ((x * x + y * y + z * z) > topograph_params_.new_session_odometry_jump_m * topograph_params_.new_session_odometry_jump_m) {
            int mapId = triggerNewMap();
            EXT_LOG_WARN("[input] Odometry jump exceeds {:.3f} m; started segment={:d}; previous_pose={} current_pose={}",
                         topograph_params_.new_session_odometry_jump_m, mapId, lastPose.debugString().c_str(), odomPose.debugString().c_str());
          }
        }
      }
    }
  }

  //============================================================
  // Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
  //============================================================
  const auto previous_odom_pose = _memory->current_odom_pose();
  if (previous_odom_pose.isNull()) {
    previous_submap_keyframe_ = 0;
    previous_submap_map_ = -1;
    // A loaded snapshot is not a localization measurement. Keep the old world
    // separate until an accepted graph connection aligns this new component.
    topograph_.reset_projection();
  }
  // Tracked input has a fixed-pose geometric gate before allocation. A proposal
  // it retains must not be collapsed later by the old appearance-only rehearsal.
  auto admission = topograph_params_.metric_enabled
                       ? admit_frame(*_memory, *frame_builder_, relocalizer_->geometry(), *archive_writer_, *grid_builder_, data, odomPose,
                                     odomCovariance, odomVelocity, &statistics_, prepared, allow_rehearsal)
                       : admit_frame(*_memory, *frame_builder_, relocalizer_->geometry(), *archive_writer_, *grid_builder_, data, StateGroupf(),
                                     Mat66_t<double>::Identity(), {}, &statistics_, prepared, allow_rehearsal);
  if (!admission.accepted) return false;
  auto reduced_ids = std::move(admission.reduced_ids);

  node = _memory->getLastWorkingNode(false);
  if (!node) {
    EXT_LOG_FATAL("[mapping] Internal error: current node missing after successful update");
  }
  // Capture identity before a successful fusion replaces the current node by an old owner.
  const int visit_query = node->id(), visit_session = node->mapId();

  if (_memory->topograph().persistent_root_frames() && (previous_odom_pose.isNull() || !topograph_.rootFrame(topograph_.poses())) &&
      node->getWeight() >= 0 && !node->isBadNode()) {
    _memory->create_root_frame(node->id(), node->getPose());
  }
  topograph_.begin_mapping(previous_odom_pose.isNull());

  EXT_LOG_DEBUG("[mapping] Observation node={:d} stamp={:.6f} weight={:d} map_id={:d}", node->id(), node->getStamp(), node->getWeight(),
                node->mapId());
  timeMemoryUpdate = timer.lap_seconds();

  //============================================================
  // Metric
  //============================================================
  std::list<int> nodesRemoved;
  float distanceToClosestNodeInTheGraph = 0;
  float angleToClosestNodeInTheGraph = 0;
  if (topograph_params_.metric_enabled) {
    double linVar = std::max(
        {odomCovariance(0, 0), odomCovariance(1, 1) >= 9999 ? 0 : odomCovariance(1, 1), odomCovariance(2, 2) >= 9999 ? 0 : odomCovariance(2, 2)});
    double angVar = std::max(
        {odomCovariance(3, 3) >= 9999 ? 0 : odomCovariance(3, 3), odomCovariance(4, 4) >= 9999 ? 0 : odomCovariance(4, 4), odomCovariance(5, 5)});
    statistics_.addStatistic(Statistics::kMemoryOdometry_variance_lin(), (float)linVar);
    statistics_.addStatistic(Statistics::kMemoryOdometry_variance_ang(), (float)angVar);

    // Verify if there was a rehearsal
    const int rehearsedId = admission.rehearsed_id;
    if (rehearsedId > 0) {
      topograph_.on_rehearsal(rehearsedId);
    }

    if (!node->getLinks().empty() && node->getLinks().begin()->second.type() == Link::kOdometry) {
      const auto distance = previous_odom_pose.isNull() ? node->getLinks().begin()->second.inverse().transform().translationNorm()
                                                        : previous_odom_pose.translationDistance(odomPose);
      const float old_distance = run_.distance_travelled;
      run_.distance_travelled += distance;
      run_.distance_since_last_loop += run_.distance_travelled - old_distance;
    }
    topograph_.track_observation(*node, odomCovariance, _memory->keeps_depth_nodes());

    //============================================================
    // Reduced graph
    //============================================================
    // Verify if there are nodes that were merged through graph reduction
    topograph_.remove_reduced(reduced_ids);
  }

  //============================================================
  // Bayes filter update
  //============================================================
  recognition = relocalizer_->recognize(*node, lastHighestHypothesis, topograph_.poses());
  statistics_.addStatistic("Timing/SemanticRetrieval/ms", recognition.semantic_seconds * 1000);
  statistics_.addStatistic("Loop/SceneCandidates/", recognition.scene_candidates.size());
  topograph_.record_hypothesis(recognition.hypothesis);
  timer.reset();

  //============================================================
  // Pending snapshots are visible to retrieval; only graph transactions drain.
  //============================================================
  trajectory_writer_.reap(*_memory);
  timeEmptyingTrash = _memory->getDbSavingTime();
  timeJoiningTrash = timer.lap_seconds();

  //============================================================
  // RETRIEVAL 1/3 : Loop closure neighbors reactivation
  //============================================================
  relocalizer_->retrieve_neighbors(
      *node, topograph_.poses(), topograph_.constraints(),
      memory_params_.update_budget_ms != 0 || memory_params_.resident_budget_nodes != 0 || _memory->working_memory_byte_limit(),
      run_.nodes_transferred, recognition);
  if (localmaps)
    for (int id : recognition.loaded) {
      const auto map = localmaps->maps().find(id);
      if (map != localmaps->maps().end()) _memory->attach_navimap(map->second);
    }
  timeReactivations = timer.lap_seconds();

  //============================================================
  // Appearance retrieval followed by bounded spatial fallback.
  //============================================================
  GraphUpdate graph(topograph_);
  auto loops = prepare_loops(*node, recognition, localmaps);
  timer.reset();

  //============================================================
  // Optimize map graph
  //============================================================
  optimize_observation(*node, loops, recognition, !reduced_ids.empty(), graph);

  if (loops.submap_candidate && loops.submap_candidate->refresh && !recognition.rejected) {
    auto update = _memory->prepare_feature_content(loops.submap_candidate->retained, loops.submap_candidate->removed,
                                                   std::make_unique<Node>(*loops.submap_candidate->replacement), false, loops.candidate_links,
                                                   graph.repaired_links, loops.submap_candidate->region);
    if (!_memory->can_apply(update)) throw std::runtime_error("Expired fused graph owners");
    std::vector<Node*> writes;
    for (auto& [id, draft] : update.link_nodes) writes.push_back(&draft);
    // Prepare publication allocations before ordered submission. Raw capture IDs,
    // odom links, navigation anchors and the next odom predecessor stay intact.
    auto accepted_poses = graph.poses;
    auto accepted_constraints = graph.constraints;
    const auto fusion_sequence = persist_fused_submap(*_memory->database(), *update.replacement, update.source, writes, graph.poses, graph.mapped, {},
                                                      {}, loops.submap_candidate->region, loops.submap_candidate->expected_region);
    _memory->apply_feature_content(std::move(update));
    topograph_.commit_solution(accepted_poses, accepted_constraints, graph.correction, graph.covariance, graph.mapped);
    loops.candidate_links.clear();
    graph.repaired_links.clear();
    if (graph.optimizer_trial) graph.optimizer_trial->commit();
    statistics_.addStatistic("Node/Consolidated/", loops.submap_candidate->region ? 1 : 0);
    statistics_.addStatistic("Node/Refreshed/", 1);
    statistics_.addStatistic("Node/Target/", loops.submap_candidate->retained);
    statistics_.addStatistic("Node/BundleMs/", loops.submap_candidate->bundle_ms);
    const auto& evidence = *loops.submap_candidate;
    EXT_LOG_INFO(
        "[constraint_submission] query={} target={} descriptor_model={} mode={} proposed={} measurements={} survivors={} identity_digest={} "
        "sql_sequence={}",
        evidence.removed, evidence.retained, evidence.target->descriptor_model_id(), evidence.shared_geometry_redundant ? "reused" : "fresh_ba",
        evidence.inliers.size(), evidence.bundle_measurements, evidence.bundle_survivors, evidence.bundle_identity_digest, fusion_sequence);
    // Submission follows graph acceptance but is not a durable SQL acknowledgement.
    // Offline acceptance must also verify the healthy writer and persisted membership.
    loops.submap_candidate.reset();
  } else if (!loops.submap_candidate && !recognition.rejected) {
    auto accepted_poses = graph.poses;
    auto accepted_constraints = graph.constraints;
    _memory->topograph().apply_links(loops.candidate_links, graph.repaired_links);
    topograph_.commit_solution(accepted_poses, accepted_constraints, graph.correction, graph.covariance, graph.mapped);
    loops.candidate_links.clear();
    graph.repaired_links.clear();
    if (graph.optimizer_trial) graph.optimizer_trial->commit();
  }

  timeMapOptimization = timer.lap_seconds();
  if (graph.graphOptimizationExecuted) {
    statistics_.setGraphOptimizationInfo({graph.optimizationTriggerMask, node->id(), graph.optimizationPoseCount, graph.optimizationConstraintCount,
                                          graph.optimizationIterations, graph.optimizationError, timeMapOptimization * 1000.0});
    EXT_LOG_DEBUG("[graph] Optimization: triggers=[{}] node={} poses={} constraints={} iterations={} objective={:.6f} elapsed_ms={:.1f}",
                  graph.optimizationTriggers, node->id(), graph.optimizationPoseCount, graph.optimizationConstraintCount,
                  graph.optimizationIterations, graph.optimizationError, timeMapOptimization * 1000.0);
  }

  //============================================================
  // Prepare statistics
  //============================================================
  // Data used for the statistics event and for the log files
  int dictionarySize = 0;
  int refWordsCount = 0;
  int lcHypothesisReactivated = 0;
  sLoop = _memory->getNode(loops.loop_candidate ? loops.loop_candidate : topograph_.hypothesis().place_id);
  if (sLoop) {
    lcHypothesisReactivated = sLoop->isSaved() ? 1.0f : 0.0f;
  }
  dictionarySize = static_cast<int>((_memory->fbow_index() ? _memory->fbow_index()->getIndexedWordsCount() : 0));
  refWordsCount = (int)node->feature_count();

  // Posterior is empty if a bad node is detected
  float vpHypothesis = recognition.posterior.size() ? recognition.posterior.at(Memory::kIdVirtual) : 0.0f;

  // prepare statistics
  if (loops.loop_candidate || statistics_params_.publish_enabled || statistics_params_.log_enabled) {
    statistics_.setRefImageId(_memory->getLastNodeId());  // Use last id from Memory (in case of rehearsal)
    statistics_.setRefImageMapId(node->mapId());
    statistics_.setStamp(data.stamp());
    if (loops.loop_candidate != Memory::kIdInvalid) {
      EXT_LOG_DEBUG("[graph] Candidate node={} passed graph validation; scene commit pending", loops.loop_candidate);
    }

    if (statistics_params_.publish_enabled || statistics_params_.log_enabled) {
      statistics_.setExtended(1);

      statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), topograph_.hypothesis().place_id);
      statistics_.addStatistic(Statistics::kLoopKnown_place_probability(), topograph_.hypothesis().known_probability);
      statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
      statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
      statistics_.addStatistic(Statistics::kLoopReactivate_id(), recognition.retrieval_id);
      statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), recognition.ratio);
      statistics_.addStatistic(Statistics::kLoopVisual_inliers(), loops.loopClosureVisualInliers);
      statistics_.addStatistic(Statistics::kLoopVisual_inliers_ratio(), loops.loopClosureVisualInliersRatio);
      statistics_.addStatistic(Statistics::kLoopVisual_matches(), loops.loopClosureVisualMatches);
      statistics_.addStatistic(Statistics::kLoopVisual_variance(), loops.loopClosureVisualVariance);
      statistics_.addStatistic(Statistics::kLoopLinear_variance(), loops.loopClosureLinearVariance);
      statistics_.addStatistic(Statistics::kLoopAngular_variance(), loops.loopClosureAngularVariance);
      if (graph.maxGraphErrors.linear >= 0 || graph.maxGraphErrors.angular >= 0) {
        statistics_.addStatistic(Statistics::kLoopOptimization_max_error(), graph.maxGraphErrors.linear);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_error_ratio(), graph.maxGraphErrors.linearRatio);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error(), graph.maxGraphErrors.angular * 180.0f / M_PI);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_ratio(), graph.maxGraphErrors.angularRatio);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_error_from_id(), graph.maxGraphErrorsLinearIds.first);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_error_to_id(), graph.maxGraphErrorsLinearIds.second);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_from_id(), graph.maxGraphErrorsAngularIds.first);
        statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_to_id(), graph.maxGraphErrorsAngularIds.second);

        if (pose_graph_params_.repair_radius_m > 0) {
          statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_from_id(), graph.maxGraphErrorsRemovedIds.first);
          statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_to_id(), graph.maxGraphErrorsRemovedIds.second);
          statistics_.addStatistic(Statistics::kLoopOptimization_max_error_removed_count(), graph.maxGraphErrorsRemovedCount);
        }
      }
      statistics_.addStatistic(Statistics::kLoopOptimization_error(), graph.optimizationError);
      statistics_.addStatistic(Statistics::kLoopOptimization_iterations(), graph.optimizationIterations);
      statistics_.addStatistic(Statistics::kLoopVisual_inliers_distribution(), loops.loopClosureVisualInliersDistribution);

      statistics_.addStatistic(Statistics::kLoopDistance_since_last_closure(), run_.distance_since_last_loop);

      float x, y, z, roll, pitch, yaw;
      if (loops.loop_candidate) {
        if (loops.loop_candidate) {
          // Loop closure transform
          EXT_ASSERT(sLoop);
          const auto loopIter = graph::findLink(graph.constraints, sLoop->id(), node->id());
          EXT_ASSERT(loopIter != graph.constraints.end());
          const auto measured = loopIter->second.from() == sLoop->id() ? loopIter->second.transform() : loopIter->second.transform().inverse();
          statistics_.setLoopClosureTransform(measured);

          statistics_.addStatistic(Statistics::kLoopVisual_words(), sLoop->feature_count());
        }

        statistics_.addStatistic(Statistics::kLoopMapToOdom_norm(), graph.correction.translationNorm());
        statistics_.addStatistic(Statistics::kLoopMapToOdom_angle(), graph.correction.rotationDistance(StateGroupf::identity()) * 180.0f / M_PI);
        stateGroupToXyzRpy(graph.correction, x, y, z, roll, pitch, yaw);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_x(), x);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_y(), y);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_z(), z);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_roll(), roll * 180.0f / M_PI);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_pitch(), pitch * 180.0f / M_PI);
        statistics_.addStatistic(Statistics::kLoopMapToOdom_yaw(), yaw * 180.0f / M_PI);

        // Odom correction (actual odometry pose change), ignoring the first correction.
        if (!odomPose.isNull() && !graph.previous_correction.isNull() && !graph.previous_correction.isIdentity()) {
          const StateGroupf previousCorrectedOdom = graph.previous_correction.compose(odomPose);
          const StateGroupf odomCorrection = previousCorrectedOdom.between(graph.correction).compose(odomPose);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_norm(), odomCorrection.translationNorm());
          statistics_.addStatistic(Statistics::kLoopOdom_correction_angle(),
                                   odomCorrection.rotationDistance(StateGroupf::identity()) * 180.0f / M_PI);
          stateGroupToXyzRpy(odomCorrection, x, y, z, roll, pitch, yaw);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_x(), x);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_y(), y);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_z(), z);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_roll(), roll * 180.0f / M_PI);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_pitch(), pitch * 180.0f / M_PI);
          statistics_.addStatistic(Statistics::kLoopOdom_correction_yaw(), yaw * 180.0f / M_PI);
        }
      }
      if (!graph.mapped.isNull() && !graph.mapped.isIdentity()) {
        stateGroupToXyzRpy(graph.mapped, x, y, z, roll, pitch, yaw);
        statistics_.addStatistic(Statistics::kLoopMapToBase_x(), x);
        statistics_.addStatistic(Statistics::kLoopMapToBase_y(), y);
        statistics_.addStatistic(Statistics::kLoopMapToBase_z(), z);
        statistics_.addStatistic(Statistics::kLoopMapToBase_roll(), roll * 180.0f / M_PI);
        statistics_.addStatistic(Statistics::kLoopMapToBase_pitch(), pitch * 180.0f / M_PI);
        statistics_.addStatistic(Statistics::kLoopMapToBase_yaw(), yaw * 180.0f / M_PI);
        EXT_LOG_DEBUG("[graph] Mapped pose = {}", graph.mapped.debugString().c_str());

        if (graph.covariance.has_value()) {
          double varLin = std::max({(*graph.covariance)(0, 0), (*graph.covariance)(1, 1), (*graph.covariance)(2, 2)});

          statistics_.addStatistic(Statistics::kLoopMapToBase_lin_std(), sqrt(varLin));
          statistics_.addStatistic(Statistics::kLoopMapToBase_lin_var(), varLin);
        }
      }

      statistics_.setMapCorrection(graph.correction);
      EXT_LOG_DEBUG("[graph] Set map correction = {}", graph.correction.debugString().c_str());
      statistics_.setMapCorrectionCovariance(graph.covariance);

      // timings...
      statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate * 1000);
      statistics_.addStatistic(Statistics::kTimingReactivation(), timeReactivations * 1000);
      statistics_.addStatistic(Statistics::kTimingAdd_loop_closure_link(), loops.timeAddLoopClosureLink * 1000);
      statistics_.addStatistic(Statistics::kTimingMap_optimization(), timeMapOptimization * 1000);
      statistics_.addStatistic(Statistics::kTimingLikelihood_computation(), recognition.likelihood_seconds * 1000);
      statistics_.addStatistic(Statistics::kTimingPosterior_computation(), recognition.posterior_seconds * 1000);
      statistics_.addStatistic(Statistics::kTimingHypotheses_creation(), recognition.selection_seconds * 1000);
      statistics_.addStatistic(Statistics::kTimingHypotheses_validation(), recognition.validation_seconds * 1000);
      statistics_.addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors * 1000);

      // retrieval
      statistics_.addStatistic(Statistics::kMemoryNodes_retrieved(), (float)recognition.loaded.size());

      // Feature specific parameters
      statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);
      statistics_.addStatistic(Statistics::kKeypointCurrent_frame(), refWordsCount);
      statistics_.addStatistic(Statistics::kKeypointVocabulary_words(), (_memory->fbow_index() ? _memory->fbow_index()->getIndexedWordsCount() : 0));
      statistics_.addStatistic(Statistics::kKeypointVocabulary_memory_usage(),
                               (_memory->fbow_index() ? _memory->fbow_index()->getIndexMemoryUsed() : 0));

      // Epipolar geometry constraint

      statistics_.addStatistic(Statistics::kMemoryDistance_travelled(), run_.distance_travelled);

      if (distanceToClosestNodeInTheGraph > 0.0) {
        statistics_.addStatistic(Statistics::kMemoryClosest_node_distance(), distanceToClosestNodeInTheGraph);
        statistics_.addStatistic(Statistics::kMemoryClosest_node_angle(), angleToClosestNodeInTheGraph);
      }

      if (statistics_params_.collect_memory_usage) {
        tools::Timer ramTimer;
        std::error_code rssError;
        const std::optional<std::uint64_t> rss = tools::resident_set_size(rssError);
        const std::uint64_t rssMegabytes = rss && !rssError ? *rss / (1024U * 1024U) : 0U;
        statistics_.addStatistic(Statistics::kMemoryRAM_usage(), rssMegabytes);
        long estimatedMemoryUsage = sizeof(Mapping);
        estimatedMemoryUsage += graph.poses.size() * (sizeof(int) + sizeof(StateGroupf) + sizeof(std::map<int, StateGroupf>::iterator)) +
                                sizeof(std::map<int, StateGroupf>);
        estimatedMemoryUsage +=
            graph.constraints.size() * (sizeof(int) + sizeof(Link) + sizeof(std::map<int, Link>::iterator)) + sizeof(std::map<int, Link>);
        estimatedMemoryUsage += _memory->getMemoryUsed();
        estimatedMemoryUsage += topograph_.belief().getMemoryUsed() - sizeof(BayesFilter);
        estimatedMemoryUsage += relocalizer_->memory_size() + sizeof(FrameBuilder) + sizeof(LocalGridMaker) + sizeof(ArchiveWriter);
        estimatedMemoryUsage += sizeof(frame_builder_->extractor().configuration());
        estimatedMemoryUsage += _parameters.size() * (sizeof(std::string) * 2 + sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);
        statistics_.addStatistic(Statistics::kMemoryRAM_estimated(), (float)(estimatedMemoryUsage / (1024 * 1024)));  // MB
        statistics_.addStatistic(Statistics::kTimingRAM_estimation(), ramTimer.lap_seconds() * 1000);
      }

      statistics_.setLabels(_memory->getAllLabels());
    }

    timeStatsCreation = timer.lap_seconds();
  }

  Node lastNodeData = *node;
  StateGroupf lastNodeLocalizedPose;
  if (graph.poses.find(node->id()) != graph.poses.end()) {
    lastNodeLocalizedPose = graph.poses.at(node->id());
  }
  if (!archive_params_.keep_images_in_memory) {
    _memory->removeRawData(node->id(), true, true, false);
  }

  bool lastNodeWasIntermediateNode = node->getWeight() == -1;

  // Remove the last node if it is bad and bad nodes are ignored.
  int nodeRemoved = _memory->cleanup();
  if (nodeRemoved) {
    nodesRemoved.push_back(nodeRemoved);
  }

  // Apply node-quality filtering to the new map immediately.
  if (nodeRemoved != lastNodeData.id()) {
    if (topograph_params_.start_session_on_good_frame && node->isBadNode() &&
        std::none_of(node->getLinks().begin(), node->getLinks().end(),
                     [](const auto& entry) { return entry.second.from() != entry.second.to(); }))  // alone in the current map
    {
      EXT_LOG_DEBUG("[mapping] Skipping node={:d}: insufficient features for first node; waiting for next frame", node->id());
      nodesRemoved.push_back(node->id());
      _memory->deleteLocation(node->id());
    }
  }

  std::shared_ptr<const LocalMap> sealed_map;
  std::vector<OdomObservation> current_trajectory;
  const auto submit_keyframe = [&](const Statistics* stats) {
    trajectory_writer_.submit(*_memory, lastNodeData.id(), current_trajectory, sealed_map, stats, statistics_params_.save_resident_nodes);
    current_trajectory.clear();
    sealed_map.reset();
  };
  if (localmaps && depth_grid) {
    const int current_id = lastNodeData.id();
    if (_memory->getNode(current_id)) {
      if (localmaps->active()) {
        tools::Timer archive_timer;
        sealed_map = sapphire::backend::seal_localmap(*localmaps);
        statistics_.addStatistic("LocalMap/SealMs/", archive_timer.elapsed_seconds() * 1000);
        statistics_.addStatistic("LocalMap/Sealed/", 1);
      }
      localmaps->begin(current_id, odomPose);
      _memory->attach_navimap(localmaps->maps().at(current_id));
      append_localmap(*localmaps, data.stamp(), odomPose, std::move(depth_grid));
    } else if (localmaps->active()) {
      append_localmap(*localmaps, data.stamp(), odomPose, std::move(depth_grid));
    }
    if (localmaps->active()) {
      const auto& map = *localmaps->maps().at(localmaps->active_id());
      current_trajectory.push_back(map.last_observation);
    }
    if (loops.submap_candidate && !loops.submap_candidate->refresh && _memory->getNode(current_id) &&
        loops.loop_candidate == loops.submap_candidate->retained && graph.poses.count(loops.submap_candidate->retained) &&
        graph.poses.count(current_id)) {
      // The graph transaction must observe all accepted keyframe products.
      // This is an intentional durability boundary, not a per-frame wait.
      submit_keyframe(nullptr);
      _memory->joinTrashThread();
      trajectory_writer_.reap(*_memory);
      const int retained = loops.submap_candidate->retained;
      const auto relative = graph.poses.at(retained).between(graph.poses.at(current_id));
      tools::Timer merge_timer;
      const bool committed = commit_node_fusion(current_id, relative, *localmaps, loops, graph);
      if (committed) {
        lastNodeData = *_memory->getNode(retained);
        lastNodeLocalizedPose = {};
        statistics_.setRefImageId(retained);
        statistics_.setRefImageMapId(lastNodeData.mapId());
        statistics_.setLoopClosureTransform({});
        reduced_ids[current_id] = retained;
        statistics_.setReducedIds(reduced_ids);
        statistics_.addStatistic("Node/Merged/", 1);
        statistics_.addStatistic("Node/Source/", current_id);
        statistics_.addStatistic("Node/Target/", retained);
        statistics_.addStatistic("Node/MergeMs/", merge_timer.elapsed_seconds() * 1000);
        EXT_LOG_DEBUG("[graph] Scene merge complete: node={:d} retained_node={:d} features={:d} elapsed_ms={:.1f}", current_id, retained,
                      int(lastNodeData.feature_count()), merge_timer.elapsed_seconds() * 1000);
      } else {
        graph.optimizer_trial.reset();
        loops.candidate_links.clear();
        graph.repaired_links.clear();
        const auto* source = _memory->getNode(current_id);
        graph.poses = topograph_.poses();
        graph.constraints = topograph_.constraints();
        graph.correction = topograph_.correction();
        graph.covariance = topograph_.correction_covariance();
        graph.mapped = topograph_.last_mapped_pose();
        loops.loop_candidate = 0;
        recognition.rejected = true;
        statistics_.setLoopClosureTransform({});
        statistics_.setMapCorrection(graph.correction);
        statistics_.setMapCorrectionCovariance(graph.covariance);
        lastNodeData = *source;
      }
    }
  }

  statistics_.addStatistic("Node/PreviousKeyframe/", previous_submap_map_ == lastNodeData.mapId() ? previous_submap_keyframe_ : 0);

  topograph_.commit_solution(graph.poses, graph.constraints, graph.correction, graph.covariance, graph.mapped);

  if (loops.loop_candidate) run_.distance_since_last_loop = 0.0f;

  // Publish acceptance facts from the final owner state. A valid trial can
  // still fail its SQL/content commit; provisional statistics are not a commit.
  // Spatial evidence must not be reported to the appearance belief as an
  // accepted appearance hypothesis. Both sources share graph commit/rollback.
  recognition.committed_id = loops.spatial ? 0 : loops.loop_candidate;
  const int committed_spatial = loops.spatial ? loops.loop_candidate : 0;
  EXT_LOG_INFO("[loop] nominated={} geometry-verified={} committed_global={} committed_local={} best-posterior={} known-place-probability={}",
               recognition.qualified_id, recognition.geometry_verified_id, recognition.committed_id, committed_spatial,
               recognition.hypothesis.posterior, recognition.hypothesis.known_probability);
  topograph_.commit_loop(recognition.committed_id, recognition.hypothesis.known_probability);
  const int committed_loop = loops.loop_candidate;
  if (!lastNodeWasIntermediateNode && topograph_params_.metric_enabled) {
    const bool retained = !lastNodeData.isBadNode() && _memory->getNode(lastNodeData.id()) && graph.poses.count(lastNodeData.id());
    relocalizer_->record_visit(visit_query, visit_session, data.stamp(), retained ? lastNodeData.id() : 0, retained ? committed_loop : 0);
  }
  statistics_.addStatistic("Memory/VisitEntries/", relocalizer_->visit_count());
  statistics_.addStatistic("Retrieval/VisitPredictions/", recognition.visit_predictions.size());
  statistics_.addStatistic("Retrieval/VisitReordered/", recognition.visits_reordered ? 1 : 0);
  statistics_.setLoopClosureId(loops.loop_candidate);
  statistics_.setLoopClosureMapId(loops.loop_candidate > 0 ? _memory->getMapId(loops.loop_candidate) : -1);
  statistics_.addStatistic("Loop/SpatialCandidates/", loops.spatial_candidates_checked);
  statistics_.addStatistic("Loop/SpatialInsufficientSupport/", loops.spatial_insufficient_support);
  statistics_.addStatistic("Loop/SpatialId/", committed_spatial);
  statistics_.addStatistic("Timing/SpatialSearch/ms", loops.spatial_search_seconds * 1000);
  if (statistics_params_.publish_enabled || statistics_params_.log_enabled) {
    statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->topograph().getLastLoopClosureId());
    statistics_.addStatistic(Statistics::kLoopId(), committed_loop);
    statistics_.addStatistic(Statistics::kLoopMap_id(), committed_loop > 0 ? _memory->getMapId(committed_loop) : -1);
    statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), recognition.rejected ? 1.0f : 0);
    statistics_.addStatistic(Statistics::kLoopAccepted_hypothesis_id(), recognition.committed_id);
  }

  // Evaluate only committed graph state, after a possible submap rollback.
  // New-session poses have never been mixed with the persisted world snapshot.
  if (const int component = topograph_.publication_component(topograph_params_.use_previous_map, _graphOptimizer->options.max_iterations > 0))
    selectWorldComponent(component);
  topograph_.publish_current(reduced_ids, nodesRemoved);
  statistics_.addStatistic("Localization/MapAligned/", topograph_.aligned() ? 1.0f : 0.0f);
  // Announce accepted results only after graph validation and submap rollback.
  // Earlier candidate/optimization messages are diagnostics, not acceptance.
  if (statistics_.loopClosureId()) {
    EXT_LOG_INFO("[loop] Loop accepted: retained_node={} match={} source={} output_frame={}", statistics_.refImageId(), statistics_.loopClosureId(),
                 loops.spatial ? "spatial" : "appearance", topograph_.aligned() ? "main_map" : "current_segment");
  }

  // Pass this point node should not be used, since it could have been transferred...
  node = 0;

  timeMemoryCleanup = timer.lap_seconds();

  //============================================================
  // TRANSFER
  //============================================================
  // If time allowed for the detection exceeds the limit of
  // real-time, move the oldest node with less frequency
  // entry (from X oldest) from the short term memory to the
  // long term memory.
  //============================================================
  double totalTime = timerTotal.lap_seconds();
  if (!lastNodeWasIntermediateNode)  // skip memory management on intermediate nodes
  {
    size_t workingMemSize = _memory->getWorkingMemSize(true);
    const bool budget_pressure = _memory->working_memory_over_budget(memory_params_.resident_budget_nodes);
    if ((memory_params_.update_budget_ms != 0 && totalTime * 1000 > memory_params_.update_budget_ms) || budget_pressure) {
      if (memory_params_.update_budget_ms != 0 && totalTime * 1000 > memory_params_.update_budget_ms) {
        EXT_LOG_DEBUG("[mapping] Removing old nodes because time limit is reached {:.6f} ms > {:.6f} ms...", totalTime * 1000,
                      memory_params_.update_budget_ms);
      }
      if (memory_params_.resident_budget_nodes != 0 && workingMemSize > static_cast<size_t>(memory_params_.resident_budget_nodes)) {
        EXT_LOG_DEBUG("[mapping] Removing old nodes because memory limit is reached {:d} > {:d}...", (int)workingMemSize,
                      memory_params_.resident_budget_nodes);
      }
      std::list<int> transferred = budget_pressure ? _memory->trim_working_memory(memory_params_.resident_budget_nodes, recognition.protected_ids)
                                                   : _memory->forget(recognition.protected_ids);
      nodesRemoved.insert(nodesRemoved.end(), transferred.begin(), transferred.end());
      if (!run_.nodes_transferred && transferred.size()) {
        run_.nodes_transferred = true;  // only used to hide a warning on close nodes immunization
      }
    }
  }
  statistics_.addStatistic("Memory/WorkingPayloadBytes/", _memory->working_memory_bytes());
  statistics_.addStatistic("Memory/GlobalRetrievalDocuments/", _memory->retrieval_document_count());

  // Remove optimized poses from nodes transferred
  if (nodesRemoved.size() && (topograph_.poses().size() || topograph_.constraints().size())) {
    // refresh the local map because some transferred nodes may have broken the tree
    int id = 0;
    if (topograph_.poses().size() && _memory->getLastWorkingNode(true)) {
      id = _memory->getLastWorkingNode(true)->id();
      EXT_LOG_DEBUG("[mapping] Refresh local map from {:d}", id);
    }
    EXT_LOG_DEBUG("[mapping] id={:d} topograph_.poses()={:d}", id, (int)topograph_.poses().size());
    if (id > 0) {
      EXT_ASSERT_MSG(_memory->getNode(id) != 0, "id={:d}", id);

      if (nodesRemoved.size() == 1 && nodesRemoved.front() == lastNodeData.id()) {
        topograph_.remove_last_projection(nodesRemoved.front());
      } else {
        // Eviction changes RAM residency, not metric graph membership. Keep the
        // full current component so a cold canonical datum cannot be replaced
        // by a younger alias on the next non-optimization frame.
        std::map<int, int> ids;
        if (_memory->topograph().persistent_root_frames()) {
          for (int node : _memory->metric_component(id)) ids.emplace(node, 0);
        } else
          ids = _memory->getNeighborsId(id, 0, 0, true);
        topograph_.retain_component(ids);
      }
    } else {
      if (!topograph_.poses().empty()) EXT_LOG_DEBUG("[mapping] Optimized poses cleared!");
      topograph_.clear_projection();
    }
  }

  timeRealTimeLimitReachedProcess = timer.lap_seconds();

  //==============================================================
  // Finalize statistics and log files
  //==============================================================
  if (statistics_params_.publish_enabled || statistics_params_.log_enabled) {
    statistics_.addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation * 1000);
    statistics_.addStatistic(Statistics::kTimingTotal(), totalTime * 1000);
    statistics_.addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess * 1000);
    statistics_.addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash * 1000);
    statistics_.addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash * 1000);
    statistics_.addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup * 1000);

    // Transfer
    statistics_.addStatistic(Statistics::kMemoryNodes_removed(), nodesRemoved.size());
    statistics_.addStatistic(Statistics::kMemoryImmunized_globally(), recognition.protected_global);
    statistics_.addStatistic(Statistics::kMemoryImmunized_locally(), recognition.protected_local);
    statistics_.addStatistic(Statistics::kMemoryImmunized_locally_max(), recognition.protected_local_limit);

    // place after transfer because the memory/local graph may have changed
    statistics_.addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMemSize(true));
    statistics_.addStatistic(Statistics::kMemoryWorking_memory_inter_size(), _memory->getWorkingMemIntermediateNodesCount());
    statistics_.addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size() - _memory->getStMemIntermediateNodesCount());
    statistics_.addStatistic(Statistics::kMemoryShort_time_memory_inter_size(), _memory->getStMemIntermediateNodesCount());
    statistics_.addStatistic(Statistics::kMemoryDatabase_memory_used(), _memory->getDatabaseMemoryUsed());

    // Set local graph
    std::map<int, StateGroupf> poses;
    std::multimap<int, Link> constraints;
    if (!topograph_params_.metric_enabled) {
      // no optimization on appearance-only mode, create a local graph
      std::map<int, int> ids = _memory->getNeighborsId(lastNodeData.id(), 0, 0, true);
      _memory->getMetricConstraints(sapphire::backend::tools::key_set(ids), poses, constraints, false);
    } else  // RGBD-SLAM mode
    {
      poses = topograph_.poses();
      constraints = topograph_.constraints();
    }
    statistics_.addNodeData(lastNodeData);

    if (!lastNodeLocalizedPose.isNull()) {
      poses.insert(std::make_pair(lastNodeData.id(), lastNodeLocalizedPose));
    }
    statistics_.setPoses(poses);
    statistics_.setConstraints(constraints);

    statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());

    std::vector<int> ids;
    ids.reserve(_memory->getWorkingMemSize(false) + _memory->getStMem().size());
    for (std::set<int>::const_iterator iter = _memory->getStMem().begin(); iter != _memory->getStMem().end(); ++iter) {
      ids.push_back(*iter);
    }
    for (std::map<int, double>::const_iterator iter = _memory->getWorkingMem().lower_bound(0); iter != _memory->getWorkingMem().end(); ++iter) {
      ids.push_back(iter->first);
    }
    statistics_.setWmState(ids);
    EXT_LOG_DEBUG("[mapping] wmState={:d}", (int)ids.size());
  }

  // Save statistics to database
  submit_keyframe(&statistics_);

  // Start trashing
  _memory->emptyTrash();

  relocalizer_->prioritize_prefetch(visit_session, data.stamp(), recognition.predicted);
  _memory->prefetch_map_features(recognition.predicted);
  if (!lastNodeWasIntermediateNode && !lastNodeData.isBadNode() &&
      std::find(nodesRemoved.begin(), nodesRemoved.end(), lastNodeData.id()) == nodesRemoved.end()) {
    previous_submap_keyframe_ = lastNodeData.id();
    previous_submap_map_ = lastNodeData.mapId();
  }
  timeFinalizingStatistics = timer.lap_seconds();

  if (statistics_params_.publish_enabled || statistics_params_.log_enabled) {
    statistics_.addStatistic(Statistics::kTimingFinalizing_statistics(), timeFinalizingStatistics * 1000);
  }
  statistics_log_.write(statistics_);

  return true;
}

void Mapping::setOptimizedPoses(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints) {
  const auto* last = _memory ? _memory->getLastWorkingNode(false) : nullptr;
  topograph_.accept_explicit_solution(poses, constraints, last ? last->id() : 0, _graphOptimizer->options.ignore_priors);
}

bool Mapping::solve_submaps(int root, std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                            std::optional<Mat66_t<double>>& covariance, double* error, int* iterations,
                            const featmap::NodeObservations* fusion) const {
  if (poses.empty()) return false;
  // Covariance follows the actual active capture, not the largest numeric ID.
  // Offline components without that capture report their chosen root instead.
  const int active = _memory ? _memory->getLastNodeId() : 0;
  const int capture = poses.count(active) ? active : root;
  int target = capture;
  StateGroupf local = StateGroupf::identity();
  if (fusion && fusion->observation_poses.count(capture)) {
    target = fusion->reference_node;
    local = fusion->observation_poses.at(capture);
  } else if (topograph_.local_poses().count(capture)) {
    target = topograph_.submap_owner(capture);
    local = topograph_.local_poses().at(capture);
  }
  _graphOptimizer->covariance_target = target;
  if (!fusion && topograph_.members().empty()) return _graphOptimizer->optimizeInPlace(root, poses, constraints, covariance, error, iterations);
  auto projected = constraints;
  root = topograph_.project_submaps(root, poses, projected, fusion);
  if (!_graphOptimizer->optimizeInPlace(root, poses, projected, covariance, error, iterations)) return false;
  // Covariance is conditional on the fitted rigid internal geometry. It is not
  // an exact marginalization of the removed cameras or independent BA evidence.
  if (target != capture && covariance.has_value()) {
    const auto adjoint = StateGroupd(local).inverse().adjoint();
    covariance = (adjoint * *covariance * adjoint.transpose()).eval();
  }
  topograph_.expand_submaps(poses, fusion);
  return true;
}

void Mapping::optimizeCurrentMap(int id, bool lookInDatabase, std::map<int, StateGroupf>& optimizedPoses, std::optional<Mat66_t<double>>& covariance,
                                 std::multimap<int, Link>* constraints, double* error, int* iterationsDone, const std::vector<Link>& candidates,
                                 const featmap::NodeObservations* fusion) const {
  // The persistent datum may live outside RAM; its complete component is needed.
  if (_memory && (_memory->topograph().persistent_root_frames())) lookInDatabase = true;
  // Optimize the map
  if (_memory && id > 0) {
    if (!lookInDatabase && (!_memory->getNode(id) || (_memory->getNode(id)->getWeight() == -1 && !_memory->keeps_depth_nodes()))) {
      EXT_LOG_ERROR("[graph] Optimization rejected root={:d}: node is absent or ineligible as an anchor", id);
      optimizedPoses.clear();
      if (constraints) {
        constraints->clear();
      }
      return;
    }

    std::map<int, int> ids;
    for (int node : _memory->metric_component(id)) ids.emplace(node, 0);
    // Candidate edges connect components without entering Memory's committed
    // adjacency. Include only components reached from this trial's seed.
    bool expanded = true;
    while (expanded) {
      expanded = false;
      for (const auto& edge : candidates) {
        for (const auto& endpoints : {std::pair<int, int>{edge.from(), edge.to()}, {edge.to(), edge.from()}}) {
          if (!ids.count(endpoints.first) || endpoints.second <= 0 || ids.count(endpoints.second)) continue;
          for (int peer : _memory->metric_component(endpoints.second)) expanded |= ids.emplace(peer, 0).second;
        }
      }
    }
    if (!pose_graph_params_.anchor_at_last_node && ids.size() > 1) {
      if (lookInDatabase) {
        id = ids.begin()->first;
      } else {
        // Find first node that is not intermediate
        for (auto pair : ids) {
          // Make sure fromId is not an intermediate node
          const Node* s = _memory->getNode(pair.first);
          if (s && s->getWeight() != -1) {
            id = pair.first;
            break;
          } else if (!s) {
            EXT_LOG_WARN("[graph] Node {:d} missing from memory", pair.first);
          }
        }
      }
    }
    if (!pose_graph_params_.anchor_at_last_node) {
      for (const auto& entry : ids) {
        if (entry.first > 0 &&
            (topograph_.world_poses().count(entry.first) || (topograph_params_.use_previous_map && topograph_.stored_poses().count(entry.first)))) {
          id = entry.first;
          break;
        }
      }
    }

    std::map<int, StateGroupf> poses = Mapping::optimizeGraph(id, sapphire::backend::tools::key_set(ids), optimizedPoses, lookInDatabase, covariance,
                                                              constraints, error, iterationsDone, candidates, fusion);

    if (poses.size()) {
      optimizedPoses = poses;

      if (_memory->getNode(id) && ((optimizedPoses).find(id) != (optimizedPoses).end())) {
        StateGroupf t = optimizedPoses.at(id).compose(_memory->getNode(id)->getPose().inverse());
        EXT_LOG_DEBUG("[graph] Root correction node={:d} transform={}", id, t.debugString().c_str());
      }
    } else {
      EXT_LOG_WARN("[graph] Connected graph optimization produced no poses; returning empty result");
      optimizedPoses.clear();
      if (constraints) {
        constraints->clear();
      }
    }
  }
}

std::map<int, StateGroupf> Mapping::optimizeGraph(int fromId, const std::set<int>& ids, const std::map<int, StateGroupf>& guessPoses,
                                                  bool lookInDatabase, std::optional<Mat66_t<double>>& covariance,
                                                  std::multimap<int, Link>* constraints, double* error, int* iterationsDone,
                                                  const std::vector<Link>& candidates, const featmap::NodeObservations* fusion) const {
  std::map<int, StateGroupf> optimizedPoses;
  std::map<int, StateGroupf> poses;
  std::multimap<int, Link> edgeConstraints;
  _memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase);
  for (const auto& edge : candidates) {
    if (!poses.count(edge.from()) || !poses.count(edge.to())) continue;
    if (graph::findLink(edgeConstraints, edge.from(), edge.to()) != edgeConstraints.end()) continue;
    // Preserve the factor orientation used by the committed graph reader.
    const auto factor = edge.from() < edge.to() ? edge : edge.inverse();
    edgeConstraints.emplace(factor.from(), factor);
  }

  const auto* fixed_frame = topograph_.rootFrame(poses);
  _graphOptimizer->fixed_root = fixed_frame != nullptr;
  if (fixed_frame) {
    fromId = fixed_frame->anchor;
    poses.at(fromId) = fixed_frame->frame_from_anchor;
  }

  // A retained root may have raw odometry from another session. Anchor it at
  // its persisted world pose; restoring its original odometry moves the world.
  if (!fixed_frame && !pose_graph_params_.anchor_at_last_node && poses.count(fromId)) {
    if (topograph_.world_poses().count(fromId))
      poses.at(fromId) = topograph_.world_poses().at(fromId);
    else if (topograph_params_.use_previous_map && topograph_.stored_poses().count(fromId))
      poses.at(fromId) = topograph_.stored_poses().at(fromId);
  }

  if (_graphOptimizer->options.max_iterations > 0) {
    for (std::map<int, StateGroupf>::iterator iter = poses.begin(); iter != poses.end(); ++iter) {
      // Apply guess poses (if some), ignore for rootid to avoid origin drifting
      std::map<int, StateGroupf>::const_iterator foundGuess = guessPoses.find(iter->first);
      if (foundGuess != guessPoses.end() && iter->first != fromId) {
        iter->second = foundGuess->second;
      }
    }
  }

  EXT_ASSERT(_graphOptimizer != 0);
  if (_graphOptimizer->options.max_iterations == 0) {
    // Optimization disabled! Return not optimized poses.
    optimizedPoses = poses;
    if (constraints) {
      *constraints = edgeConstraints;
    }
  } else {
    if (poses.size() != guessPoses.size()) {
      EXT_LOG_DEBUG("[graph] recompute poses using only links (robust to multi-session)");
      std::map<int, StateGroupf> posesOut;
      std::multimap<int, Link> edgeConstraintsOut;
      graph::getConnectedGraph(fromId, poses, edgeConstraints, posesOut, edgeConstraintsOut, _graphOptimizer->options.ignore_priors);
      if (solve_submaps(fromId, posesOut, edgeConstraintsOut, covariance, error, iterationsDone, fusion)) {
        optimizedPoses.swap(posesOut);
      }
      if (constraints) {
        *constraints = edgeConstraintsOut;
      }
    } else {
      EXT_LOG_DEBUG("[graph] use input guess poses");
      if (solve_submaps(fromId, poses, edgeConstraints, covariance, error, iterationsDone, fusion)) {
        optimizedPoses.swap(poses);
      }
      if (constraints) {
        *constraints = edgeConstraints;
      }
    }

    if (!poses.empty() && optimizedPoses.empty()) {
      EXT_LOG_WARN("[graph] Solver produced no poses: input_poses={:d} initial_poses={:d} constraints={:d}", (int)poses.size(),
                   (int)guessPoses.size(), (int)edgeConstraints.size());
    }
  }

  topograph_.validateFramePose(optimizedPoses);
  return optimizedPoses;
}

// If repairing works, all input arguments are updated accordingly to new graph
// Returns IDs of the links removed from constraints
std::list<std::pair<int, int>> Mapping::repairGraph(graph::MaxGraphErrors& maxGraphErrors, std::map<int, StateGroupf>& poses,
                                                    std::multimap<int, Link>& constraints, double& optimizationError, int& optimizationIterations,
                                                    std::optional<Mat66_t<double>>& optimizationCovariance) {
  EXT_ASSERT(maxGraphErrors.linearLink.isValid());
  EXT_ASSERT(graph::findLink(constraints, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to()) != constraints.end());

  int originalMaxErrorLinkFrom = maxGraphErrors.linearLink.from();
  int originalMaxErrorLinkTo = maxGraphErrors.linearLink.to();

  graph::MaxGraphErrors subMaxGraphErrors = maxGraphErrors;
  std::list<std::pair<int, int>> removedLinks;
  std::map<int, StateGroupf> subPoses = poses;
  std::multimap<int, Link> subConstraints = constraints;
  while (subMaxGraphErrors.linearLink.isValid() && subMaxGraphErrors.linearRatio > pose_graph_params_.max_constraint_error_sigma) {
    removedLinks.push_back(std::make_pair(subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
    subConstraints.erase(graph::findLink(subConstraints, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
    subMaxGraphErrors.linearLink = Link();

    // Get the connected graph to check whether a node became disconnected.
    std::map<int, StateGroupf> posesOut;
    std::multimap<int, Link> subConstraintsOut;
    graph::getConnectedGraph(subPoses.rbegin()->first, subPoses, subConstraints, posesOut, subConstraintsOut, _graphOptimizer->options.ignore_priors);
    if (subPoses.size() - posesOut.size() > 1) {
      EXT_LOG_WARN("[graph] Repair aborted: multiple poses removed in one iteration");
      break;
    } else if (subPoses.size() != posesOut.size()) {
      for (std::map<int, StateGroupf>::iterator pter = subPoses.begin(); pter != subPoses.end(); ++pter) {
        if (posesOut.find(pter->first) == posesOut.end()) {
          subPoses.erase(pter);
          break;  // should be only one if different, break now
        }
      }
    }
    std::optional<Mat66_t<double>> subOptimizationCovariance;
    double subOptimizationError = 0.0;
    int subOptimizationIterations = 0;

    int fromId = subPoses.rbegin()->first;
    if (!pose_graph_params_.anchor_at_last_node) {
      // Find first node that is not intermediate
      for (std::map<int, StateGroupf>::iterator iter = subPoses.lower_bound(1); iter != subPoses.end(); ++iter) {
        // Make sure fromId is not an intermediate node
        const Node* s = _memory->getNode(iter->first);
        if (s && s->getWeight() != -1) {
          fromId = iter->first;
          break;
        } else if (!s) {
          EXT_LOG_WARN("[graph] Node {:d} missing from memory", iter->first);
        }
      }
    }

    const auto* fixed_frame = topograph_.rootFrame(subPoses);
    _graphOptimizer->fixed_root = fixed_frame != nullptr;
    if (fixed_frame) {
      fromId = fixed_frame->anchor;
      subPoses.at(fromId) = fixed_frame->frame_from_anchor;
    }
    if (!solve_submaps(fromId, subPoses, subConstraintsOut, subOptimizationCovariance, &subOptimizationError, &subOptimizationIterations))
      subPoses.clear();

    topograph_.validateFramePose(subPoses);
    if (subPoses.empty()) {
      EXT_LOG_WARN("[graph] Repair aborted: re-optimization failed");
    } else {
      subMaxGraphErrors = graph::computeMaxGraphErrors(subPoses, subConstraintsOut);
      if (!subMaxGraphErrors.linearLink.isValid()) {
        EXT_LOG_WARN("[graph] Cannot check residuals: no valid constraints; loop consistency unverified");
      } else if (subMaxGraphErrors.linearRatio > pose_graph_params_.max_constraint_error_sigma) {
        float distance = poses.at(originalMaxErrorLinkFrom).translationDistance(poses.at(subMaxGraphErrors.linearLink.from()));
        if (subMaxGraphErrors.linearLink.isLoopClosure() && distance < pose_graph_params_.repair_radius_m) {
          EXT_LOG_WARN(
              "[graph] Optimization error is still high ({:.6f}, on link {:d}->{:d} type={:d}) after removing the loop closure with the highest "
              "error. "
              "As it is close ({:.6f} m < {}={:.6f} m) to original loop closure with high error ({:d}->{:d}), we will reject again this one to see "
              "if it "
              "helps.",
              subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to(),
              subMaxGraphErrors.linearLink.type(), distance, "pose_graph.repair_radius_m", pose_graph_params_.repair_radius_m,
              originalMaxErrorLinkFrom, originalMaxErrorLinkTo);
        } else {
          EXT_LOG_WARN("[graph] Optimization error is still high ({:.6f}, on link {:d}->{:d}) after removing loop closure with highest error.",
                       subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to());
          subMaxGraphErrors.linearLink = Link();
        }
      } else {
        EXT_LOG_WARN(
            "[graph] Optimization error is lower ({:.6f}, on link {:d}->{:d}) after removing loop "
            "closure with highest error. We will remove the old link ({:d}->{:d} collaterals={:d}) and accept the new one.",
            subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to(), removedLinks.front().first,
            removedLinks.front().second, removedLinks.size() - 1);

        maxGraphErrors = subMaxGraphErrors;

        // Make sure the link pointers in maxGraphErrors point on same constraints
        std::swap(poses, subPoses);
        std::swap(constraints, subConstraintsOut);

        optimizationCovariance = subOptimizationCovariance;
        optimizationError = subOptimizationError;
        optimizationIterations = subOptimizationIterations;

        return removedLinks;
      }
    }
  }
  return std::list<std::pair<int, int>>();
}

Node Mapping::getNodeCopy(int id, bool withWords, bool withGlobalDescriptors) const {
  Node s;
  if (_memory) {
    StateGroupf odomPoseLocal;
    int weight = -1;
    int mapId = -1;
    std::string label;
    double stamp = 0;
    std::vector<float> velocity;
    EnvSensors sensors;
    _memory->getNodeInfo(id, odomPoseLocal, mapId, weight, label, stamp, velocity, sensors, true);
    VisualFrame frame;
    if (withWords) {
      std::vector<camera::Perspective> models;
      _memory->getNodeCalibration(id, models);
      frame.setPerspectives(std::move(models));
    }
    s = Node(id, mapId, weight, stamp, label, odomPoseLocal, std::move(frame));

    std::multimap<int, Link> links = _memory->getLinks(id, true);
    for (std::multimap<int, Link>::iterator iter = links.begin(); iter != links.end(); ++iter) {
      s.addLink(iter->second);
    }

    _memory->copyNodeFeatures(id, s, withWords, withGlobalDescriptors);
    if (velocity.size() == 6) {
      s.setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
    }
  }
  return s;
}

void Mapping::getGraph(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& constraints, bool optimized, bool global,
                       std::map<int, Node>* nodes, bool withWords, bool withGlobalDescriptors) const {
  if (_memory && _memory->getLastWorkingNode(!global)) {
    if (topograph_params_.metric_enabled) {
      if (optimized) {
        poses = topograph_.poses();  // guess
        std::optional<Mat66_t<double>> covariance;
        this->optimizeCurrentMap(_memory->getLastWorkingNode(!global)->id(), global, poses, covariance, &constraints);
        if (!global && !topograph_.poses().empty()) {
          // We send directly the already optimized poses if they are set
          EXT_LOG_DEBUG("[retrieval] topograph_.poses()={:d} poses={:d}", topograph_.poses().size(), poses.size());
          poses = topograph_.poses();
        }
      } else {
        std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingNode(!global)->id(), 0, global ? -1 : 0, true);
        _memory->getMetricConstraints(sapphire::backend::tools::key_set(ids), poses, constraints, global);
      }
    } else {
      // no optimization on appearance-only mode
      std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingNode(!global)->id(), 0, global ? -1 : 0, true);
      _memory->getMetricConstraints(sapphire::backend::tools::key_set(ids), poses, constraints, global);
    }

    if (nodes) {
      // Get data
      std::set<int> ids = sapphire::backend::tools::key_set(_memory->getWorkingMem());  // WM

      // remove virtual node
      ids.erase(Memory::kIdVirtual);

      ids.insert(_memory->getStMem().begin(), _memory->getStMem().end());  // STM + WM
      if (global) {
        ids = _memory->getAllNodeIds();  // STM + WM + LTM, ignoreChildren=true
      }

      for (std::set<int>::iterator iter = ids.begin(); iter != ids.end(); ++iter) {
        nodes->insert(std::make_pair(*iter, getNodeCopy(*iter, withWords, withGlobalDescriptors)));
      }
    }
  } else if (_memory && (_memory->getStMem().size() || _memory->getWorkingMemSize(!global) > 0)) {
    EXT_LOG_ERROR("[mapping] Current node unavailable");
  } else if (_memory == 0) {
    EXT_LOG_WARN("[mapping] Node storage is not initialized");
  }
}

std::optional<Link> Mapping::verify_loop(int from, int to, const StateGroupf& guess) const {
  if (!_memory || !topograph_params_.metric_enabled || _graphOptimizer->options.max_iterations <= 0)
    throw std::logic_error("Loop verification requires an initialized metric map and optimizer");
  const auto source = getNodeCopy(from, true, false);
  const auto target = getNodeCopy(to, true, false);
  const auto registration = relocalizer_->verify(source, target, guess);
  if (registration.pose.isNull()) return {};
  return Link(from, to, Link::kLoopClosure, registration.pose, getInformation(*registration.covariance));
}

std::map<int, StateGroupf> Mapping::solve_graph(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& links) const {
  if (!_memory || !topograph_params_.metric_enabled || _graphOptimizer->options.max_iterations <= 0)
    throw std::logic_error("Graph trial requires an initialized metric map and optimizer");
  // Explicit scratch solve: even a successful offline query must not promote
  // the online incremental cache before a caller commits the graph.
  solver::PoseGraphSolver::Trial trial(*_graphOptimizer);
  return optimizeOfflineComponents(poses, links);
}

void Mapping::commit_loops(const std::vector<Link>& added, std::map<int, StateGroupf> poses, const std::multimap<int, Link>& links) {
  if (added.empty()) return;
  if (!_memory || poses.empty()) throw std::invalid_argument("Cannot commit an empty graph solution");
  topograph_.validateFramePose(poses);
  const auto errors = graph::computeMaxGraphErrors(poses, links);
  const auto limit = pose_graph_params_.max_constraint_error_sigma;
  if (limit > 0 && (errors.linearRatio > limit || errors.angularRatio > limit))
    throw std::invalid_argument("Offline graph exceeds the configured residual limit");
  for (const auto& edge : added) {
    if (!edge.isLoopClosure() || !poses.count(edge.from()) || !poses.count(edge.to()) ||
        graph::findLink(links, edge.from(), edge.to()) == links.end())
      throw std::invalid_argument("Offline loop is absent from the accepted graph");
  }
  if (added.size()) {
    for (const auto& edge : added) _memory->addLink(edge, true);
    if (_memory->topograph().persistent_root_frames()) {
      topograph_.remember_solution(poses);
      if (topograph_.active_anchor() && poses.count(topograph_.active_anchor())) {
        std::map<int, StateGroupf> connected;
        std::multimap<int, Link> connected_links;
        const auto ids = _memory->metric_component(topograph_.active_anchor());
        for (int id : ids)
          if (poses.count(id)) connected.emplace(id, poses.at(id));
        for (const auto& edge : links)
          if ((edge.second.from() < 0 || ids.count(edge.second.from())) && (edge.second.to() < 0 || ids.count(edge.second.to()))) {
            connected_links.insert(edge);
            for (int id : {edge.second.from(), edge.second.to()})
              if (id < 0 && poses.count(id)) connected.emplace(id, poses.at(id));
          }
        setOptimizedPoses(connected, connected_links);
      }
    }
    // Update optimized poses
    std::map<int, StateGroupf> tmp;
    std::multimap<int, Link> refreshed;
    _memory->getMetricConstraints(sapphire::backend::tools::key_set(topograph_.poses()), tmp, refreshed, false);
    topograph_.apply_offline_solution(poses, refreshed, false);
    // Invalidate the saved occupancy grid after graph optimization.
    _memory->save2DMap(cv::Mat(), 0, 0, 0);
  }
}

bool Mapping::addLink(const Link& link) {
  const StateGroupf& t = link.transform();
  if (!topograph_params_.metric_enabled) {
    EXT_LOG_ERROR("[graph] Cannot add constraint: pose graph mapping disabled");
    return false;
  }
  if (!_memory) {
    EXT_LOG_ERROR("[graph] Cannot add constraint: node storage is not initialized");
    return false;
  }
  if (t.isNull()) {
    EXT_LOG_ERROR("[graph] Link's transform is null! ({:d}->{:d} type={})", link.from(), link.to(), link.typeName());
    return false;
  }
  if (_memory->getNode(link.from()) == 0) {
    EXT_LOG_ERROR("[graph] Link's \"from id\" {:d} is not in working memory", link.from());
    return false;
  }
  if (_memory->getNode(link.to()) == 0) {
    EXT_LOG_ERROR("[graph] Link's \"to id\" {:d} is not in working memory", link.to());
    return false;
  }

  if (topograph_.poses().find(link.from()) == topograph_.poses().end() && topograph_.poses().find(link.to()) == topograph_.poses().end()) {
    EXT_LOG_ERROR("[graph] Cannot add constraint: nodes {:d} and {:d} both missing from current graph ({:d} nodes)", link.from(), link.to(),
                  (int)topograph_.poses().size());
    return false;
  }

  if (_memory->getNode(link.from())->hasLink(link.to())) return true;
  sapphire::backend::solver::PoseGraphSolver::Trial trial(*_graphOptimizer);

  // optimize with new link
  std::map<int, StateGroupf> poses = topograph_.poses();
  std::multimap<int, Link> links;
  std::optional<Mat66_t<double>> covariance;
  optimizeCurrentMap(_memory->getLastWorkingNode(true)->id(), _memory->topograph().persistent_root_frames(), poses, covariance, &links, nullptr,
                     nullptr, {link});

  if (poses.find(link.from()) == poses.end()) {
    EXT_LOG_ERROR("[graph] Link's \"from id\" {:d} is not in the graph (size={:d})", link.from(), (int)poses.size());
    return false;
  }
  if (poses.find(link.to()) == poses.end()) {
    EXT_LOG_ERROR("[graph] Link's \"to id\" {:d} is not in the graph (size={:d})", link.to(), (int)poses.size());
    return false;
  }

  std::string msg;
  if (poses.empty()) {
    msg = fmt::format("Rejecting edge {:d}->{:d} because graph optimization has failed!", link.from(), link.to());
  } else {
    graph::MaxGraphErrors maxGraphErrors;

    maxGraphErrors = graph::computeMaxGraphErrors(poses, links);
    if (maxGraphErrors.linearLink.isValid()) {
      EXT_LOG_DEBUG("[graph] Maximum linear residual = {:.6f} m (link {:d}->{:d})", maxGraphErrors.linear, maxGraphErrors.linearLink.from(),
                    maxGraphErrors.linearLink.to());
      if (pose_graph_params_.max_constraint_error_sigma > 0.0f && maxGraphErrors.linearRatio > pose_graph_params_.max_constraint_error_sigma) {
        msg = fmt::format(
            "Rejecting edge {:d}->{:d} because "
            "graph error is too large after optimization ({:.6f} m for edge {:d}->{:d} with ratio {:.6f} > std={:.6f} m). "
            "\"{}\" is {:.6f}.",
            link.from(), link.to(), maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to(),
            maxGraphErrors.linearRatio, sqrt(maxGraphErrors.linearLink.transVariance()), "pose_graph.max_constraint_error_sigma",
            pose_graph_params_.max_constraint_error_sigma);
      } else if (pose_graph_params_.max_constraint_error_sigma == 0.0f && maxGraphErrors.linearRatio > 100 && !_graphOptimizer->options.robust) {
        EXT_LOG_ERROR(
            "[graph] Excessive position residual accepted: residual_sigma_ratio={:.3f} constraint={:d} -> {:d} type={:d} error_m={:.6f}; "
            "sigma_m={:.6f}; rejection threshold {} disabled",
            maxGraphErrors.linearRatio, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to(), maxGraphErrors.linearLink.type(),
            maxGraphErrors.linear, sqrt(maxGraphErrors.linearLink.transVariance()), "pose_graph.max_constraint_error_sigma");
      }
    }
    if (maxGraphErrors.angularLink.isValid()) {
      EXT_LOG_DEBUG("[graph] Maximum angular residual = {:.6f} deg (link {:d}->{:d})", maxGraphErrors.angular * 180.0f / M_PI,
                    maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to());
      if (pose_graph_params_.max_constraint_error_sigma > 0.0f && maxGraphErrors.angularRatio > pose_graph_params_.max_constraint_error_sigma) {
        msg = fmt::format(
            "Rejecting edge {:d}->{:d} because "
            "graph error is too large after optimization ({:.6f} deg for edge {:d}->{:d} with ratio {:.6f} > std={:.6f} deg). "
            "\"{}\" is {:.6f} m.",
            link.from(), link.to(), maxGraphErrors.angular * 180.0f / M_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to(),
            maxGraphErrors.angularRatio, sqrt(maxGraphErrors.angularLink.rotVariance()), "pose_graph.max_constraint_error_sigma",
            pose_graph_params_.max_constraint_error_sigma);
      } else if (pose_graph_params_.max_constraint_error_sigma == 0.0f && maxGraphErrors.angularRatio > 100 && !_graphOptimizer->options.robust) {
        EXT_LOG_ERROR(
            "[graph] Excessive angular residual accepted: residual_sigma_ratio={:.3f} constraint={:d} -> {:d} type={:d} error_deg={:.6f}; "
            "sigma_rad={:.6f}; rejection threshold {} disabled",
            maxGraphErrors.angularRatio, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to(), maxGraphErrors.angularLink.type(),
            maxGraphErrors.angular * 180.0f / CV_PI, sqrt(maxGraphErrors.angularLink.rotVariance()), "pose_graph.max_constraint_error_sigma");
      }
    }
  }
  if (!msg.empty()) {
    EXT_LOG_ERROR("{}", msg.c_str());
    return false;
  }

  _memory->topograph().apply_links({link}, {});
  trial.commit();
  if (_memory->topograph().persistent_root_frames()) {
    setOptimizedPoses(poses, links);
    topograph_.remember_solution(poses);
  }
  // Update optimized poses
  std::map<int, StateGroupf> tmp;
  std::multimap<int, Link> refreshed;
  _memory->getMetricConstraints(sapphire::backend::tools::key_set(topograph_.poses()), tmp, refreshed, false);
  topograph_.apply_offline_solution(poses, refreshed, !pose_graph_params_.anchor_at_last_node);
  // This will force rtabmap_ros to regenerate the global occupancy grid if there was one
  _memory->save2DMap(cv::Mat(), 0, 0, 0);

  return true;
}

Mat66_t<double> Mapping::getInformation(const Mat66_t<double>& covariance) const {
  const Eigen::LLT<Mat66_t<double>> factor(covariance);
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-8) || factor.info() != Eigen::Success)
    throw std::invalid_argument("Registration covariance must be finite, symmetric and positive definite");
  Mat66_t<double> information = factor.solve(Mat66_t<double>::Identity());
  if (relocalizer_params_.limit_covariance_by_odometry) {
    const std::vector<double>& odomMaxInf = _memory->topograph().getOdomMaxInf();
    if (odomMaxInf.size() == 6) {
      // Pixel geometry produces a full information matrix. Clipping only its
      // diagonal can make it indefinite; uniformly weaken the factor instead.
      double scale = 1.0;
      for (int i = 0; i < 6; ++i)
        if (std::isfinite(odomMaxInf[i]) && odomMaxInf[i] > 0 && information(i, i) > odomMaxInf[i])
          scale = std::min(scale, odomMaxInf[i] / information(i, i));
      information *= scale;
    }
  }
  return information;
}

}  // namespace sapphire::backend
