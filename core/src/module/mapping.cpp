#include "module/mapping.hpp"

#include <stdlib.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <common/common_type.hpp>
#include <parameters.hpp>
#include <module/relocalizer.hpp>
#include <tools/associative.hpp>
#include <tools/filesystem.hpp>
#include <tools/logging.hpp>
#include <tools/strings.hpp>
#include <tools/timer.hpp>
#include <cctype>
#include <cstdint>
#include <optional>
#include <set>
#include <thread>
#include <future>
#include <omp.h>
#include <ctime>
#include <iomanip>
#include <sys/resource.h>
#include "tools/process.hpp"

#include "common/se3_state.hpp"
#include "database/compression.hpp"
#include "database/driver.hpp"
#include "maplayer/topograph/node.hpp"
#include "maplayer/topograph.hpp"
#include "database/memory.hpp"
#include "solver/pgo_solver.hpp"

namespace sapphire {
struct Relocalization {
  std::set<int> loaded;
  bool rejected = false;
};
std::set<int> Mapping::swap_results() {
  auto changes = _graphOptimizer ? _graphOptimizer->swap_results() : std::set<int>{};
  auto internal = topograph_.take_pose_changes();
  changes.merge(internal);
  return changes;
}

namespace {

double clock_seconds(clockid_t clock) {
  timespec value{};
  if (clock_gettime(clock, &value)) throw std::runtime_error("Cannot read resource clock");
  return value.tv_sec + value.tv_nsec * 1e-9;
}
bool prepare_trace(const std::filesystem::path& path, const std::string& columns,
                   const std::string& archive_prefix, bool fresh) {
  if(fresh || !std::filesystem::exists(path) || !std::filesystem::file_size(path))return true;
  std::ifstream previous(path);std::string header;std::getline(previous,header);
  if(header+"\n"==columns)return false;
  previous.close();unsigned revision=1;std::filesystem::path saved;
  do {saved=path.parent_path()/(archive_prefix+std::to_string(revision++)+".tsv");}
  while(std::filesystem::exists(saved));
  std::filesystem::rename(path,saved);return true;
}
Relocalizer::Verification evaluate_geometry(Relocalizer::Input input, const RelocalizerParams& config,
                                             const RegistrationParams& registration) {
  const auto start=clock_seconds(CLOCK_MONOTONIC),cpu=clock_seconds(CLOCK_THREAD_CPUTIME_ID),process=clock_seconds(CLOCK_PROCESS_CPUTIME_ID);
  const auto rss=tools::resident_set_size();
  auto result=Relocalizer::evaluate(std::move(input),config,registration);
  result.start_s=start;result.end_s=clock_seconds(CLOCK_MONOTONIC);
  result.thread_cpu_ms=(clock_seconds(CLOCK_THREAD_CPUTIME_ID)-cpu)*1000;
  result.process_cpu_ms=(clock_seconds(CLOCK_PROCESS_CPUTIME_ID)-process)*1000;
  result.rss_before=rss;result.rss_after=tools::resident_set_size();
  return result;
}
void stateGroupToXyzRpy(const SE3f& state, float& x, float& y, float& z, float& roll, float& pitch, float& yaw) {
  const SE3f::Vector3 translation = state.translationVector();
  const SE3f::Vector3 yawPitchRoll = state.rotationMatrix().eulerAngles(2, 1, 0);
  x = translation.x();
  y = translation.y();
  z = translation.z();
  roll = yawPitchRoll.z();
  pitch = yawPitchRoll.y();
  yaw = yawPitchRoll.x();
}

}  // namespace

struct Mapping::RelocalizationWork {
  int query = 0;
  std::map<int,SE3f> poses;
  std::future<std::vector<Relocalizer::Verification>> result;
};

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

void Mapping::init(const MappingParameters& parameters, const std::string& databasePath, const std::string& log_directory) {
  if (_memory) throw std::logic_error("Close the map before initializing a new configuration");
  parameters.validate();
  try {
    _parameters = parameters;
    if (!databasePath.empty()) {
      EXT_ASSERT(tools::extension(databasePath).compare("db") == 0);
      EXT_LOG_DEBUG("[storage] Opening database={} use_previous_map={}", databasePath, _parameters.topograph.use_previous_map);
    } else {
      EXT_LOG_DEBUG("[storage] Using in-memory database; persistence requires an output path at close");
    }
    const bool newDatabase = databasePath.empty() || !tools::path_exists(databasePath);

    // If doesn't exist, create a memory
    if (!_memory) {
      auto memory = std::make_unique<Memory>(parameters, &topograph_);
      auto relocalizer = std::make_unique<Relocalizer>(*memory, parameters.relocalizer, parameters.registration);
      if (!memory->init(databasePath)) throw std::runtime_error("Cannot open map database");
      relocalizer_ = std::move(relocalizer);
      _memory = std::move(memory);
    }

    trajectory_sequence_ = _memory->trajectory_high_water();
    std::map<int, SE3f> restored_poses;
    SE3f restored_last;
    _graphOptimizer = std::make_unique<solver::PoseGraphSolver>(_parameters.pose_graph);

    // Restore the metric snapshot before map publication or a no-input close.
    if (_memory && !newDatabase) {
      restored_poses = _memory->loadOptimizedPoses(&restored_last);
      if (restored_poses.empty()) {
        if (const auto* last = _memory->getLastWorkingNode(true)) {
          // Recover from a missing snapshot using the persisted graph, not raw odometry alone.
          std::optional<Mat66_t<double>> covariance;
          optimizeCurrentMap(last->id(), true, restored_poses, covariance);
        }
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
        std::map<int, SE3f> component;
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

    topograph_.restore_solution(restored_poses, restored_last);
    if (_parameters.topograph.use_previous_map && !topograph_.stored_poses().empty()) selectWorldComponent(topograph_.stored_poses().rbegin()->first);
    topograph_.activate_restored_world(_parameters.topograph.use_previous_map);

    statistics_log_.open(
        log_directory.empty() ? std::filesystem::path(databasePath).parent_path().string() : log_directory,
        _parameters.statistics, newDatabase);

    if (_parameters.statistics.log_enabled) {
      const auto directory = log_directory.empty() ? std::filesystem::path(databasePath).parent_path() : std::filesystem::path(log_directory);
      const std::string loop_columns = "query\tanchor\tpath\tstart_s\tend_s\tthread_cpu_ms\tprocess_cpu_ms\trss_before\trss_after\tquery_points\ttarget_points\tload_ms\tassemble_ms\tpyramid_ms\tbbs_ms\tgicp_ms\tmetric_ms\tinliers\tfitness\titerations\tbbs_roots\tbbs_expanded\tbbs_pruned\tbbs_queue_peak\tdevice_peak_bytes\tfailure\tused_path\trequested_bytes\ttarget_bytes\tresource_limited\tinitialization_trace\tT_reference_query\n";
      const std::string query_columns = "query\tstamp\tstart_s\tend_s\tthread_cpu_ms\tprocess_cpu_ms\trss_before\trss_after\tprocess_hwm_bytes\tprepare_ms\tpgo_ms\tcommit_ms\tstorage_ms\tcandidate_trials\tloop_anchor\tmatch_decision\n";
      const bool loop_header=prepare_trace(directory/"relocalization.tsv",loop_columns,"loops.previous.",newDatabase);
      const bool query_header=prepare_trace(directory/"relocalization_queries.tsv",query_columns,"queries.previous.",newDatabase);
      const auto mode = std::ios::out | (newDatabase ? std::ios::trunc : std::ios::app);
      loop_trace_.exceptions(std::ios::failbit | std::ios::badbit);
      query_trace_.exceptions(std::ios::failbit | std::ios::badbit);
      loop_trace_.open(directory / "relocalization.tsv", mode);
      query_trace_.open(directory / "relocalization_queries.tsv", mode);
      loop_trace_ << std::setprecision(17);
      query_trace_ << std::setprecision(17);
      if (loop_header) {
        loop_trace_ << loop_columns;
      }
      if (query_header) {
        query_trace_ << query_columns;
      }
    }

  } catch (...) {
    close(false);
    throw;
  }
}

std::map<int, SE3f> Mapping::optimizeOfflineComponents(const std::map<int, SE3f>& input, const std::multimap<int, Link>& links) const {
  std::map<int, SE3f> remaining = input, result;
  while (!remaining.empty()) {
    const auto seed = remaining.lower_bound(1);
    if (seed == remaining.end()) break;
    std::map<int, SE3f> component;
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

void Mapping::getCompleteGraph(std::map<int, SE3f>& poses, std::multimap<int, Link>& links) const {
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

  _memory->checkpoint_nodes();
  _memory->joinTrashThread();
  const auto surviving = _memory->getAllNodeIds();
  topograph_.prepare_archive(surviving, topograph_.current_odom_pose().isNull());
  topograph_.finish_archive(surviving, {});
  _memory->saveOptimizedPoses(topograph_.stored_poses(), topograph_.world_last_pose());
  _memory->joinTrashThread();
}

void Mapping::append_lidar_odometry(double stamp, const SE3f& pose) {
  if (!_memory || !_memory->getLastWorkingNode(false)) throw std::logic_error("LiDAR trajectory needs an admitted anchor");
  const auto* anchor = _memory->getLastWorkingNode(false);
  OdomObservation observation{anchor->id(), ++trajectory_sequence_, stamp, anchor->getPose().inverse().compose(pose), pose};
  _memory->save_keyframe(0, {observation});
}

void Mapping::close(bool databaseSaved, const std::string& ouputDatabasePath) {
  if (databaseSaved) poll_relocalization(true);
  else relocalization_work_.reset(); // joins immutable work before owner destruction
  // Drain owned map evidence before tearing down the storage owner. Failure
  // retains owners until the caller can safely close the failed task.

  EXT_LOG_DEBUG("[storage] close save={}", databaseSaved);
  topograph_.end_observations();
  run_ = {};

  statistics_log_.close();
  if (loop_trace_.is_open()) loop_trace_.close();
  if (query_trace_.is_open()) query_trace_.close();

  if (_memory) {
    if (databaseSaved) {
      const auto surviving_ids = _memory->getAllNodeIds();
      topograph_.prepare_archive(surviving_ids, topograph_.current_odom_pose().isNull());
      std::map<int, int> reduced;
      if (topograph_.isGraphReduced() && !topograph_.current_odom_pose().isNull()) {
        reduced = maintain_residency(true);
        _memory->incrementMapId();
      }
      topograph_.finish_archive(surviving_ids, reduced);
      _memory->saveOptimizedPoses(topograph_.stored_poses(), topograph_.world_last_pose());
    }
    _memory->close(databaseSaved, ouputDatabasePath);
    relocalizer_.reset();
    _memory.reset();
  }
  topograph_.clear_publication();

  _graphOptimizer.reset();
  // Reset configuration without constructing another optimizer while closing.

  _parameters = {};
  async_relocalization_ = false;
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
  return sapphire::NodeParams{}.generate_node_ids;
}


SE3f Mapping::getPose(int locationId) const { return sapphire::tools::value_or(topograph_.poses(), locationId, SE3f()); }

int Mapping::triggerNewMap() {
  int mapId = -1;
  if (_memory) {
    run_.distance_travelled = 0.0f;
    run_.distance_since_last_loop = 0.0f;

    const auto reducedIds = maintain_residency(true);
    mapId = _memory->incrementMapId();
    topograph_.remove_reduced_world(reducedIds);
    EXT_LOG_INFO("[mapping] Started odometry segment: segment={}", mapId);

    topograph_.start_session();
  }
  return mapId;
}

//============================================================
// MAIN LOOP
//============================================================

struct Mapping::RelocalizationProposal {
  std::vector<Link> candidate_links;
  int loop_candidate = 0;
  unsigned candidate_trials = 0;
  Relocalizer::Path path;
  std::string match_decision;
};

std::vector<Relocalizer::Input> Mapping::prepare_relocalization_inputs(const Node& query, Relocalization& recognition) {
  std::vector<Relocalizer::Input> inputs;
  if ((!_parameters.relocalizer.spatial_enabled && !_parameters.relocalizer.global_enabled) || _graphOptimizer->options.max_iterations <= 0) return inputs;
  std::map<int,SE3f> poses;std::multimap<int,Link> links;getCompleteGraph(poses,links);
  const auto candidates=relocalizer_->candidates(query,poses);
  std::vector<std::vector<Relocalizer::Path>> paths;
  std::size_t rounds=0,bytes=0;
  for(int id:candidates) {paths.push_back(relocalizer_->paths(id,query.id(),poses));rounds=std::max(rounds,paths.back().size());}
  std::set<std::vector<int>> attempted;
  for(std::size_t round=0;round<rounds;++round)for(std::size_t index=0;index<paths.size();++index) {
    if(round>=paths[index].size() || inputs.size()>=_parameters.relocalizer.spatial_max_candidates)continue;
    auto input=relocalizer_->prepare(paths[index][round],query.id(),poses,attempted);
    if(input.result.duplicate)continue;
    auto identity=input.result.path.nodes;std::sort(identity.begin(),identity.end());attempted.insert(identity);
    recognition.loaded.insert(input.result.path.anchor);
    // One query owns a bounded aggregate of frozen targets, never K full budgets.
    if(input.target && input.result.target_bytes>_parameters.relocalizer.path_max_bytes-bytes) {
      input.target.reset();input.source.reset();input.result.resource_failure="Query target payload budget exceeded";
    } else if(input.target)bytes+=input.result.target_bytes;
    inputs.push_back(std::move(input));
  }
  return inputs;
}
Mapping::RelocalizationProposal Mapping::finish_relocalization(int query, std::vector<Relocalizer::Verification> results) {
  RelocalizationProposal proposal;proposal.candidate_trials=results.size();
  for(auto& verified:results) {
    const int candidate=verified.path.anchor;
    auto& alignment=verified.alignment;
    if (loop_trace_.is_open()) {
        loop_trace_ << query << '\t' << candidate << '\t';
        for (int id : verified.path.nodes) loop_trace_ << id << ',';
        loop_trace_ << '\t' << verified.start_s << '\t' << verified.end_s
          << '\t' << verified.thread_cpu_ms << '\t' << verified.process_cpu_ms
          << '\t' << verified.rss_before << '\t' << verified.rss_after << '\t' << verified.query_points << '\t' << verified.target_points
          << '\t' << verified.load_ms << '\t' << verified.assemble_ms << '\t' << alignment.pyramid_ms << '\t' << alignment.bbs_ms
          << '\t' << alignment.gicp_ms << '\t' << alignment.metric_ms  << '\t' << alignment.inliers
          << '\t' << alignment.fitness << '\t' << alignment.iterations << '\t' << alignment.bbs_roots << '\t' << alignment.bbs_expanded
          << '\t' << alignment.bbs_pruned << '\t' << alignment.bbs_queue_peak << '\t' << alignment.device_peak_bytes
          << '\t' << (verified.resource_failure.empty() ? alignment.failure : verified.resource_failure) << '\t';
        for (int id : verified.path.nodes) loop_trace_ << id << ',';
        loop_trace_ << '\t' << verified.requested_bytes << '\t' << verified.target_bytes
                    << '\t' << !verified.resource_failure.empty() << '\t' << verified.initialization_trace << '\t';
        if (!alignment.T_target_query.isNull()) {
          const auto transform = alignment.T_target_query.matrix();
          for (int row = 0; row < 3; ++row) for (int column = 0; column < 4; ++column)
            loop_trace_ << transform(row,column) << ',';
        }
        loop_trace_ << '\n';
        loop_trace_.flush();
      }

  }
  const auto selected=Relocalizer::select(results,_parameters.registration,proposal.match_decision);
  if(selected) {
    auto& verified=results[*selected];auto& a=verified.alignment;
    const int candidate=verified.path.anchor;
    proposal.candidate_links.push_back(Link(candidate,query,Link::kRelocalization,a.T_target_query,getInformation(*a.covariance)).inverse());
    proposal.loop_candidate=candidate;proposal.path=verified.path;
  }
  EXT_LOG_INFO("[relocalization] query={} decision={} candidates={}",query,proposal.match_decision,results.size());
  return proposal;
}
Mapping::RelocalizationProposal Mapping::prepare_relocalization(const Node& query, Relocalization& recognition) {
  auto inputs=prepare_relocalization_inputs(query,recognition);
  std::vector<Relocalizer::Verification> results;
  for(auto& input:inputs)results.push_back(evaluate_geometry(std::move(input),_parameters.relocalizer,_parameters.registration));
  return finish_relocalization(query.id(),std::move(results));
}
void Mapping::enable_async_relocalization() {
  async_relocalization_=true;
}
bool Mapping::relocalization_pending() const { return bool(relocalization_work_); }
void Mapping::schedule_relocalization(const Node& query) {
  if(relocalization_work_)return; // Optional matching has no backlog; all observations are still archived.
  Relocalization recognition;
  auto inputs=prepare_relocalization_inputs(query,recognition);
  if(inputs.empty())return;
  auto work=std::make_unique<RelocalizationWork>();work->query=query.id();
  std::map<int,SE3f> poses;std::multimap<int,Link> links;getCompleteGraph(poses,links);
  for(const auto& input:inputs)for(int id:input.result.path.nodes)work->poses.emplace(id,poses.at(id));
  const auto config=_parameters.relocalizer;const auto registration=_parameters.registration;
  work->result=std::async(std::launch::async,[inputs=std::move(inputs),config,registration]() mutable {
    omp_set_num_threads(1);
    std::vector<Relocalizer::Verification> results;
    for(auto& input:inputs)results.push_back(evaluate_geometry(std::move(input),config,registration));
    return results;
  });
  relocalization_work_=std::move(work);
}
bool Mapping::poll_relocalization(bool wait) {
  if(!relocalization_work_)return false;
  if(!wait && relocalization_work_->result.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return false;
  auto work=std::move(relocalization_work_);
  auto proposal=finish_relocalization(work->query,work->result.get());
  const bool committed=[&]() -> bool {
    if(proposal.candidate_links.empty())return false;
    std::map<int,SE3f> current;std::multimap<int,Link> links;getCompleteGraph(current,links);
    // Appending nodes/reset does not invalidate immutable old evidence. A changed
    // target's internal geometry does; never apply a stale assembled-path result.
    const int anchor=proposal.loop_candidate;
    for(int id:proposal.path.nodes) {
      if(!current.count(id) || !work->poses.count(id))return false;
      const auto before=work->poses.at(anchor).inverse().compose(work->poses.at(id));
      const auto after=current.at(anchor).inverse().compose(current.at(id));
      if((before.matrix()-after.matrix()).norm()>1e-4)return false;
    }
    // Frozen batch payloads never change after admission.
    const auto& edge=proposal.candidate_links.front();
    if(getNodeCopy(edge.from()).hasLink(edge.to()))return false;
    if(!prepare_relocalization_graph(edge,current,links)) {
      EXT_LOG_INFO("[relocalization] query={} decision=graph_rejected",work->query);return false;
    }
    commit_relocalizations(proposal.candidate_links,std::move(current),links);
    checkpoint();
    EXT_LOG_INFO("[relocalization] query={} decision=committed anchor={}",work->query,anchor);
    return true;
  }();
  if(async_relocalization_ && getLastLocationId()>work->query) {
    const auto latest=getNodeCopy(getLastLocationId());schedule_relocalization(latest);
  }
  if(wait && relocalization_work_)return poll_relocalization(true) || committed;
  return committed;
}

struct Mapping::GraphUpdate {
  std::map<int, SE3f> poses;
  SE3f correction, mapped;
  std::optional<Mat66_t<double>> covariance;
};

void Mapping::optimize_observation(const Node& query, RelocalizationProposal& loops, Relocalization& recognition, bool reduced, GraphUpdate& update) {
  const auto* node = &query;
  if(!loops.candidate_links.empty()) {
    std::map<int,SE3f> poses;std::multimap<int,Link> links;
    if(prepare_relocalization_graph(loops.candidate_links.front(),poses,links)) {
      // Only the connected component belongs in the live projection. All other
      // solutions retain their independent datums in storage.
      topograph_.remember_solution(poses);
      auto connected=_memory->metric_component(query.id());
      for(int endpoint:{loops.candidate_links.front().from(),loops.candidate_links.front().to()}) {
        const auto ids=_memory->metric_component(endpoint);connected.insert(ids.begin(),ids.end());
      }
      for(int id:connected)update.poses.emplace(id,poses.at(id));
      update.correction=update.poses.at(query.id()).compose(query.getPose().inverse());
      update.mapped=update.poses.at(query.id());
    } else {
      loops.candidate_links.clear();loops.loop_candidate=0;recognition.rejected=true;
    }
    return;
  }
  // No relocalization proposal: only refresh existing priors/resident projection.
  // All new metric constraints have already taken the shared graph-trial path.
  const bool has_prior = node->hasLink(node->id(), Link::kPosePrior) && !_graphOptimizer->options.ignore_priors;
  const bool has_gravity = node->hasLink(node->id(), Link::kGravity) && _graphOptimizer->options.gravity_sigma > 0.f &&
                           !_parameters.topograph.use_odometry_gravity;
  if (node->getWeight() == -1 || !(reduced || has_prior || has_gravity || !recognition.loaded.empty())) return;
  EXT_ASSERT(topograph_.poses().count(node->id()));
  auto poses = topograph_.poses();
  std::multimap<int, Link> constraints;
  std::optional<Mat66_t<double>> covariance;
  optimizeCurrentMap(node->id(), _memory->topograph().persistent_root_frames(), poses, covariance, &constraints);
  if (poses.empty()) {
    recognition.rejected = true;
    EXT_LOG_WARN("[graph] Existing-map optimization failed; retaining committed projection");
    return;
  }
  update.poses = std::move(poses);
  update.covariance = covariance;
  EXT_ASSERT(update.poses.count(node->id()));
  update.correction = update.poses.at(node->id()).compose(node->getPose().inverse());
  update.mapped = update.poses.at(node->id());
}

std::map<int, int> Mapping::maintain_residency(bool all) {
  std::map<int, int> reduced;
  for (int id : _memory->promote_recent_nodes(all)) {
    if (!topograph_.isGraphReduced()) continue;
    // Keep loop/trajectory anchors even if all their geometry has been fused.
    const auto* node = topograph_.getNode(id);
    if (node && node->hasLink(0, Link::kRelocalization)) continue;
    const int retained = topograph_.reduceNode(id, 0, 0);
    if (!retained) continue;
    // The graph rewrites constraints; storage retains the historical record.
    _memory->deleteLocation(id, true);
    reduced.emplace(id, retained);
  }
  return reduced;
}

bool Mapping::process_node(std::unique_ptr<Node> input, const Mat66_t<double>& covariance) {
  if (!_memory) throw std::logic_error("Initialize mapping before node input");
  poll_relocalization();
  if (!input || input->mapId() != -1 || input->isSaved() || !input->getLinks().empty() ||
      !input->getPose().isValid() || !input->has_lidar() || !std::isfinite(input->getStamp()))
    throw std::invalid_argument("Mapping requires a detached LiDAR node");
  const auto odom = input->getPose();
  const double stamp = input->getStamp();
  statistics_ = Statistics{};
  tools::Timer processing_timer;
  const auto query_start = clock_seconds(CLOCK_MONOTONIC), query_cpu = clock_seconds(CLOCK_THREAD_CPUTIME_ID), query_process_cpu = clock_seconds(CLOCK_PROCESS_CPUTIME_ID);
  const auto query_rss = tools::resident_set_size();
  const auto last_pose = topograph_.current_odom_pose();
  if (!last_pose.isNull() &&
      ((!last_pose.isIdentity() && odom.isIdentity()) ||
       (_parameters.topograph.new_session_odometry_jump_m > 0 && last_pose.translationDistance(odom) > _parameters.topograph.new_session_odometry_jump_m))) {
    triggerNewMap();
  }
  const auto previous_pose = topograph_.current_odom_pose();
  const bool first = previous_pose.isNull();
  if (first) topograph_.reset_projection();
  if (!_memory->admit_node(std::move(input), covariance)) return false;
  const auto reduced = maintain_residency();
  const auto* node = _memory->getLastWorkingNode(false);
  if (topograph_.persistent_root_frames() && (first || !topograph_.rootFrame(topograph_.poses())))
    _memory->create_root_frame(node->id(), node->getPose());
  topograph_.begin_mapping(first);
  if (!first) {
    const float distance = previous_pose.translationDistance(odom);
    run_.distance_travelled += distance;
    run_.distance_since_last_loop += distance;
  }
  topograph_.track_observation(*node, covariance);
  topograph_.remove_reduced(reduced);
  Relocalization recognition;
  tools::Timer stages;
  auto loops = async_relocalization_ ? RelocalizationProposal{} : prepare_relocalization(*node, recognition);
  const auto prepare_ms = stages.lap_seconds()*1000;
  GraphUpdate graph;
  optimize_observation(*node, loops, recognition, !reduced.empty(), graph);
  const auto pgo_ms = stages.lap_seconds()*1000;
  std::map<int, Node> drafts;
  int last_loop = 0;
  topograph_.prepare_links(loops.candidate_links, {}, drafts, last_loop);
  if (!drafts.empty()) topograph_.apply_links(drafts, last_loop);
  if (!graph.poses.empty()) topograph_.commit_solution(graph.poses, graph.correction, graph.covariance, graph.mapped);
  topograph_.commit_loop(loops.loop_candidate);
  if (loops.loop_candidate) {
    run_.distance_since_last_loop = 0;
  }
  const auto commit_ms = stages.lap_seconds()*1000;
  if (const int component = topograph_.publication_component(_parameters.topograph.use_previous_map, _graphOptimizer->options.max_iterations > 0))
    selectWorldComponent(component);
  topograph_.publish_current(reduced, {});
  statistics_.setRefNodeId(node->id());
  statistics_.setStamp(stamp);
  statistics_.addStatistic("Relocalization/SpatialId/", loops.loop_candidate);
  statistics_.addStatistic("Relocalization/CandidateTrials/", loops.candidate_trials);
  statistics_.addStatistic("Localization/MapAligned/", topograph_.aligned());
  statistics_.addStatistic("Relocalization/Id/", loops.loop_candidate);
  statistics_.addStatistic("Memory/Distance_travelled/m", run_.distance_travelled);
  statistics_.addStatistic("Relocalization/Distance_since_last_closure/m", run_.distance_since_last_loop);
  const OdomObservation observation{node->id(), ++trajectory_sequence_, stamp, SE3f::identity(), odom};
  _memory->save_keyframe(node->id(), {observation}, &statistics_, _parameters.statistics.save_resident_nodes, loops.loop_candidate > 0);
  std::list<int> removed;
  if (_memory->working_memory_over_budget(_parameters.memory.resident_budget_nodes))
    removed = _memory->trim_working_memory(_parameters.memory.resident_budget_nodes, recognition.loaded);
  else if (_parameters.memory.update_budget_ms > 0 && processing_timer.elapsed_seconds() * 1000 > _parameters.memory.update_budget_ms)
    removed = _memory->forget(recognition.loaded);
  topograph_.publish_current({}, removed);
  _memory->emptyTrash();
  statistics_.addStatistic("Timing/Total/ms", processing_timer.elapsed_seconds() * 1000);
  statistics_.addStatistic("Memory/WorkingPayloadBytes/", _memory->working_memory_bytes());
  statistics_log_.write(statistics_);
  if (query_trace_.is_open()) {
    rusage usage{}; getrusage(RUSAGE_SELF,&usage);
    query_trace_ << node->id() << '\t' << stamp << '\t' << query_start << '\t' << clock_seconds(CLOCK_MONOTONIC)
      << '\t' << (clock_seconds(CLOCK_THREAD_CPUTIME_ID)-query_cpu)*1000 << '\t' << (clock_seconds(CLOCK_PROCESS_CPUTIME_ID)-query_process_cpu)*1000
      << '\t' << query_rss << '\t' << tools::resident_set_size() << '\t' << usage.ru_maxrss*1024ULL
      << '\t' << prepare_ms << '\t' << pgo_ms << '\t' << commit_ms << '\t' << stages.elapsed_seconds()*1000
      << '\t' << loops.candidate_trials << '\t' << loops.loop_candidate
      << '\t' << loops.match_decision << '\n';
    query_trace_.flush();
  }
  if(async_relocalization_)schedule_relocalization(*node);
  return true;
}

void Mapping::setOptimizedPoses(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& constraints) {
  const auto* last = _memory ? _memory->getLastWorkingNode(false) : nullptr;
  topograph_.accept_explicit_solution(poses, constraints, last ? last->id() : 0, _graphOptimizer->options.ignore_priors);
}

bool Mapping::solve_submaps(int root, std::map<int, SE3f>& poses, const std::multimap<int, Link>& constraints,
                            std::optional<Mat66_t<double>>& covariance, double* error, int* iterations) const {
  if (poses.empty()) return false;
  const int active = _memory ? _memory->getLastNodeId() : 0;
  _graphOptimizer->covariance_target = poses.count(active) ? active : root;
  return _graphOptimizer->optimizeInPlace(root, poses, constraints, covariance, error, iterations);
}

void Mapping::optimizeCurrentMap(int id, bool lookInDatabase, std::map<int, SE3f>& optimizedPoses, std::optional<Mat66_t<double>>& covariance,
                                 std::multimap<int, Link>* constraints, double* error, int* iterationsDone,
                                 const std::vector<Link>& candidates) const {
  // The persistent datum may live outside RAM; its complete component is needed.
  if (_memory && (_memory->topograph().persistent_root_frames())) lookInDatabase = true;
  // Optimize the map
  if (_memory && id > 0) {
    if (!lookInDatabase && (!_memory->getNode(id) || (_memory->getNode(id)->getWeight() == -1))) {
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
    if (!_parameters.pose_graph.anchor_at_last_node && ids.size() > 1) {
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
    if (!_parameters.pose_graph.anchor_at_last_node) {
      for (const auto& entry : ids) {
        if (entry.first > 0 &&
            (topograph_.world_poses().count(entry.first) || (_parameters.topograph.use_previous_map && topograph_.stored_poses().count(entry.first)))) {
          id = entry.first;
          break;
        }
      }
    }

    std::map<int, SE3f> poses = Mapping::optimizeGraph(id, sapphire::tools::key_set(ids), optimizedPoses, lookInDatabase, covariance,
                                                              constraints, error, iterationsDone, candidates);

    if (poses.size()) {
      optimizedPoses = std::move(poses);

      if (_memory->getNode(id) && ((optimizedPoses).find(id) != (optimizedPoses).end())) {
        SE3f t = optimizedPoses.at(id).compose(_memory->getNode(id)->getPose().inverse());
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

std::map<int, SE3f> Mapping::optimizeGraph(int fromId, const std::set<int>& ids, const std::map<int, SE3f>& guessPoses,
                                                  bool lookInDatabase, std::optional<Mat66_t<double>>& covariance,
                                                  std::multimap<int, Link>* constraints, double* error, int* iterationsDone,
                                                  const std::vector<Link>& candidates) const {
  std::map<int, SE3f> optimizedPoses;
  std::map<int, SE3f> poses;
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
  if (!fixed_frame && !_parameters.pose_graph.anchor_at_last_node && poses.count(fromId)) {
    if (topograph_.world_poses().count(fromId))
      poses.at(fromId) = topograph_.world_poses().at(fromId);
    else if (_parameters.topograph.use_previous_map && topograph_.stored_poses().count(fromId))
      poses.at(fromId) = topograph_.stored_poses().at(fromId);
  }

  if (_graphOptimizer->options.max_iterations > 0) {
    for (std::map<int, SE3f>::iterator iter = poses.begin(); iter != poses.end(); ++iter) {
      // Apply guess poses (if some), ignore for rootid to avoid origin drifting
      std::map<int, SE3f>::const_iterator foundGuess = guessPoses.find(iter->first);
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
      std::map<int, SE3f> posesOut;
      std::multimap<int, Link> edgeConstraintsOut;
      graph::getConnectedGraph(fromId, poses, edgeConstraints, posesOut, edgeConstraintsOut, _graphOptimizer->options.ignore_priors);
      if (solve_submaps(fromId, posesOut, edgeConstraintsOut, covariance, error, iterationsDone)) {
        optimizedPoses.swap(posesOut);
      }
      if (constraints) {
        *constraints = edgeConstraintsOut;
      }
    } else {
      EXT_LOG_DEBUG("[graph] use input guess poses");
      if (solve_submaps(fromId, poses, edgeConstraints, covariance, error, iterationsDone)) {
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
std::list<std::pair<int, int>> Mapping::repairGraph(graph::MaxGraphErrors& maxGraphErrors, std::map<int, SE3f>& poses,
                                                    std::multimap<int, Link>& constraints, double& optimizationError, int& optimizationIterations,
                                                    std::optional<Mat66_t<double>>& optimizationCovariance) {
  EXT_ASSERT(maxGraphErrors.linearLink.isValid());
  EXT_ASSERT(graph::findLink(constraints, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to()) != constraints.end());

  int originalMaxErrorLinkFrom = maxGraphErrors.linearLink.from();
  int originalMaxErrorLinkTo = maxGraphErrors.linearLink.to();

  graph::MaxGraphErrors subMaxGraphErrors = maxGraphErrors;
  std::list<std::pair<int, int>> removedLinks;
  std::map<int, SE3f> subPoses = poses;
  std::multimap<int, Link> subConstraints = constraints;
  while (subMaxGraphErrors.linearLink.isValid() && subMaxGraphErrors.linearRatio > _parameters.pose_graph.max_constraint_error_sigma) {
    removedLinks.push_back(std::make_pair(subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
    subConstraints.erase(graph::findLink(subConstraints, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to()));
    subMaxGraphErrors.linearLink = Link();

    // Get the connected graph to check whether a node became disconnected.
    std::map<int, SE3f> posesOut;
    std::multimap<int, Link> subConstraintsOut;
    graph::getConnectedGraph(subPoses.rbegin()->first, subPoses, subConstraints, posesOut, subConstraintsOut, _graphOptimizer->options.ignore_priors);
    if (subPoses.size() - posesOut.size() > 1) {
      EXT_LOG_WARN("[graph] Repair aborted: multiple poses removed in one iteration");
      break;
    } else if (subPoses.size() != posesOut.size()) {
      for (std::map<int, SE3f>::iterator pter = subPoses.begin(); pter != subPoses.end(); ++pter) {
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
    if (!_parameters.pose_graph.anchor_at_last_node) {
      // Find first node that is not intermediate
      for (std::map<int, SE3f>::iterator iter = subPoses.lower_bound(1); iter != subPoses.end(); ++iter) {
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
      } else if (subMaxGraphErrors.linearRatio > _parameters.pose_graph.max_constraint_error_sigma) {
        float distance = poses.at(originalMaxErrorLinkFrom).translationDistance(poses.at(subMaxGraphErrors.linearLink.from()));
        if (subMaxGraphErrors.linearLink.isRelocalization() && distance < _parameters.pose_graph.repair_radius_m) {
          EXT_LOG_WARN(
              "[graph] Optimization error is still high ({:.6f}, on link {:d}->{:d} type={:d}) after removing the loop closure with the highest "
              "error. "
              "As it is close ({:.6f} m < {}={:.6f} m) to original loop closure with high error ({:d}->{:d}), we will reject again this one to see "
              "if it "
              "helps.",
              subMaxGraphErrors.linearRatio, subMaxGraphErrors.linearLink.from(), subMaxGraphErrors.linearLink.to(),
              subMaxGraphErrors.linearLink.type(), distance, "pose_graph.repair_radius_m", _parameters.pose_graph.repair_radius_m,
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

Node Mapping::getNodeCopy(int id) const {
  if (!_memory) return {};
  if (const auto* node = _memory->getNode(id)) return *node;
  std::list<Node*> loaded;
  if (_memory->database()) _memory->database()->loadNodes({id}, loaded);
  if (loaded.empty()) return {};
  std::unique_ptr<Node> node(loaded.front());
  return *node;
}

void Mapping::getGraph(std::map<int, SE3f>& poses, std::multimap<int, Link>& constraints, bool optimized, bool global,
                       std::map<int, Node>* nodes) const {
  if (_memory && _memory->getLastWorkingNode(!global)) {
    {
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
        _memory->getMetricConstraints(sapphire::tools::key_set(ids), poses, constraints, global);
      }
    }

    if (nodes) {
      // Get data
      std::set<int> ids = sapphire::tools::key_set(_memory->getWorkingMem());  // WM

      // remove virtual node
      ids.erase(Memory::kIdVirtual);

      ids.insert(_memory->getStMem().begin(), _memory->getStMem().end());  // STM + WM
      if (global) {
        ids = _memory->getAllNodeIds();  // STM + WM + LTM, ignoreChildren=true
      }

      for (std::set<int>::iterator iter = ids.begin(); iter != ids.end(); ++iter) {
        nodes->insert(std::make_pair(*iter, getNodeCopy(*iter)));
      }
    }
  } else if (_memory && (_memory->getStMem().size() || _memory->getWorkingMemSize(!global) > 0)) {
    EXT_LOG_ERROR("[mapping] Current node unavailable");
  } else if (_memory == 0) {
    EXT_LOG_WARN("[mapping] Node storage is not initialized");
  }
}

std::optional<Link> Mapping::verify_relocalization(int from, int to, const SE3f& guess) const {
  if (!_memory || _graphOptimizer->options.max_iterations <= 0)
    throw std::logic_error("Loop verification requires an initialized metric map and optimizer");
  const auto source = getNodeCopy(from);
  const auto target = getNodeCopy(to);
  const auto registration = relocalizer_->verify(from, to, guess);
  if (registration.T_target_query.isNull()) return {};
  return Link(from, to, Link::kRelocalization, registration.T_target_query, getInformation(*registration.covariance));
}

std::map<int, SE3f> Mapping::solve_graph(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& links) const {
  if (!_memory || _graphOptimizer->options.max_iterations <= 0)
    throw std::logic_error("Graph trial requires an initialized metric map and optimizer");
  // Explicit scratch solve: even a successful offline query must not promote
  // the online incremental cache before a caller commits the graph.
  solver::PoseGraphSolver::Trial trial(*_graphOptimizer);
  return optimizeOfflineComponents(poses, links);
}

void Mapping::commit_relocalizations(const std::vector<Link>& added, std::map<int, SE3f> poses, const std::multimap<int, Link>& links) {
  if (added.empty()) return;
  if (!_memory || poses.empty()) throw std::invalid_argument("Cannot commit an empty graph solution");
  topograph_.validateFramePose(poses);
  const auto errors = graph::computeMaxGraphErrors(poses, links);
  const auto limit = _parameters.pose_graph.max_constraint_error_sigma;
  if (limit > 0 && (errors.linearRatio > limit || errors.angularRatio > limit))
    throw std::invalid_argument("Offline graph exceeds the configured residual limit");
  for (const auto& edge : added) {
    if (!edge.isRelocalization() || !poses.count(edge.from()) || !poses.count(edge.to()) ||
        graph::findLink(links, edge.from(), edge.to()) == links.end())
      throw std::invalid_argument("Offline loop is absent from the accepted graph");
  }
  if (added.size()) {
    for (const auto& edge : added) _memory->addLink(edge, true);
    if (_memory->topograph().persistent_root_frames()) {
      topograph_.remember_solution(poses);
      if (topograph_.active_anchor() && poses.count(topograph_.active_anchor())) {
        std::map<int, SE3f> connected;
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
    topograph_.apply_offline_solution(poses, false);
  }
}

std::optional<Link> Mapping::join_segments(int reference, int query, const SE3f& seed) {
  if (!_memory || reference <= 0 || query <= 0 || reference == query || !seed.isValid())
    throw std::invalid_argument("Joining segments requires distinct saved nodes and a finite T_reference_query seed");
  if (!topograph_.current_odom_pose().isNull())
    throw std::logic_error("Join a reopened archive with no active odometry session");
  const auto edge=verify_relocalization(reference,query,seed);
  if(!edge)return {};
  std::map<int,SE3f> poses;std::multimap<int,Link> links;
  if(!prepare_relocalization_graph(*edge,poses,links))return {};
  commit_relocalizations({*edge},std::move(poses),links);
  topograph_.clear_projection();selectWorldComponent(reference);checkpoint();return edge;
}

bool Mapping::prepare_relocalization_graph(const Link& edge,std::map<int,SE3f>& poses,std::multimap<int,Link>& links) const {
  const auto a=_memory->metric_component(edge.from()),b=_memory->metric_component(edge.to());
  getCompleteGraph(poses,links);
  if(!a.count(edge.to())) {
    const auto oldest=[&](const std::set<int>& ids) {
      std::int64_t order=std::numeric_limits<std::int64_t>::max();
      for(const auto& [id,frame]:topograph_.root_frames())if(ids.count(id))order=std::min(order,frame.creation_order);
      return order;
    };
    const auto order_a=oldest(a),order_b=oldest(b);
    if(order_a==std::numeric_limits<std::int64_t>::max()||order_b==std::numeric_limits<std::int64_t>::max())throw std::runtime_error("Missing component datum");
    const auto a_from_b=poses.at(edge.from()).compose(edge.transform()).compose(poses.at(edge.to()).inverse());
    const auto& moving=order_a<order_b?b:a;const auto T=order_a<order_b?a_from_b:a_from_b.inverse();
    for(int id:moving)poses.at(id)=T.compose(poses.at(id));
  }
  links.emplace(edge.from(),edge);
  poses=solve_graph(poses,links);if(poses.empty())return false;
  const auto errors=graph::computeMaxGraphErrors(poses,links);const auto limit=_parameters.pose_graph.max_constraint_error_sigma;
  return limit<=0 || (errors.linearRatio<=limit && errors.angularRatio<=limit);
}

std::size_t Mapping::relocalize() {
  if(!_memory || !topograph_.current_odom_pose().isNull())throw std::logic_error("Automatic archive relocalization requires a reopened idle map");
  size_t committed=0;
  for(int id:_memory->getAllNodeIds()) {
    const auto query=getNodeCopy(id);Relocalization recognition;
    auto proposal=prepare_relocalization(query,recognition);
    if(query_trace_.is_open()) {
      query_trace_<<id<<'\t'<<query.getStamp();
      for(int column=2;column<15;++column)query_trace_<<'\t';
      query_trace_<<'\t'<<proposal.match_decision<<'\n';query_trace_.flush();
    }
    if(proposal.candidate_links.empty())continue;
    std::map<int,SE3f> poses;std::multimap<int,Link> links;
    if(!prepare_relocalization_graph(proposal.candidate_links.front(),poses,links))continue;
    commit_relocalizations(proposal.candidate_links,std::move(poses),links);
    topograph_.clear_projection();selectWorldComponent(id);checkpoint();
    ++committed;
  }
  return committed;
}

bool Mapping::addLink(const Link& link) {
  const SE3f& t = link.transform();
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
  sapphire::solver::PoseGraphSolver::Trial trial(*_graphOptimizer);

  // optimize with new link
  std::map<int, SE3f> poses = topograph_.poses();
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
      if (_parameters.pose_graph.max_constraint_error_sigma > 0.0f && maxGraphErrors.linearRatio > _parameters.pose_graph.max_constraint_error_sigma) {
        msg = fmt::format(
            "Rejecting edge {:d}->{:d} because "
            "graph error is too large after optimization ({:.6f} m for edge {:d}->{:d} with ratio {:.6f} > std={:.6f} m). "
            "\"{}\" is {:.6f}.",
            link.from(), link.to(), maxGraphErrors.linear, maxGraphErrors.linearLink.from(), maxGraphErrors.linearLink.to(),
            maxGraphErrors.linearRatio, sqrt(maxGraphErrors.linearLink.transVariance()), "pose_graph.max_constraint_error_sigma",
            _parameters.pose_graph.max_constraint_error_sigma);
      } else if (_parameters.pose_graph.max_constraint_error_sigma == 0.0f && maxGraphErrors.linearRatio > 100 && !_graphOptimizer->options.robust) {
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
      if (_parameters.pose_graph.max_constraint_error_sigma > 0.0f && maxGraphErrors.angularRatio > _parameters.pose_graph.max_constraint_error_sigma) {
        msg = fmt::format(
            "Rejecting edge {:d}->{:d} because "
            "graph error is too large after optimization ({:.6f} deg for edge {:d}->{:d} with ratio {:.6f} > std={:.6f} deg). "
            "\"{}\" is {:.6f} m.",
            link.from(), link.to(), maxGraphErrors.angular * 180.0f / M_PI, maxGraphErrors.angularLink.from(), maxGraphErrors.angularLink.to(),
            maxGraphErrors.angularRatio, sqrt(maxGraphErrors.angularLink.rotVariance()), "pose_graph.max_constraint_error_sigma",
            _parameters.pose_graph.max_constraint_error_sigma);
      } else if (_parameters.pose_graph.max_constraint_error_sigma == 0.0f && maxGraphErrors.angularRatio > 100 && !_graphOptimizer->options.robust) {
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
  topograph_.apply_offline_solution(poses, !_parameters.pose_graph.anchor_at_last_node);

  return true;
}

Mat66_t<double> Mapping::getInformation(const Mat66_t<double>& covariance) const {
  const Eigen::LLT<Mat66_t<double>> factor(covariance);
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-8) || factor.info() != Eigen::Success)
    throw std::invalid_argument("Registration covariance must be finite, symmetric and positive definite");
  Mat66_t<double> information = factor.solve(Mat66_t<double>::Identity());
  if (_parameters.relocalizer.limit_covariance_by_odometry) {
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

}  // namespace sapphire
