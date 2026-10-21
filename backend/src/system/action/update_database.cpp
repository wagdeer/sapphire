#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/database/driver.hpp>
#include <backend/database/localmap_archive.hpp>
#include <backend/database/statistics.hpp>
#include <backend/maplayer/node.hpp>
#include <backend/system/action/update_database.hpp>
#include <backend/memory.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/params/archive.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/parallel_executor.hpp>
#include <backend/tools/timer.hpp>
#include <thread>
namespace sapphire::backend {
void append_localmap(LocalMapCollection& maps, double stamp, const StateGroupf& odom, std::shared_ptr<const LocalGrid> grid) {
  if (auto* map = maps.active_map()) LocalMapArchive::prepare_append(*map);
  maps.append(stamp, odom, std::move(grid));
}
std::shared_ptr<const LocalMap> seal_localmap(LocalMapCollection& maps, const std::function<void(LocalMap&)>& persist) {
  auto* map = maps.active_map();
  if (!map) return {};
  LocalMapArchive::seal(*map);
  if (persist) persist(*map);
  return maps.release_sealed();
}

void persist_submap_merge(DBDriver& database, int retained, int removed, const std::vector<Node*>& writes, const LocalMapMerge& local,
                          const std::map<int, StateGroupf>& poses, const StateGroupf& last_pose, const StateGroupf& transform) {
  auto snapshot = *local.merged();
  LocalMapArchive::seal(snapshot);
  const auto archive = LocalMapArchive::encode(snapshot);
  auto atlas = database.loadOptimizedPoses();
  for (const auto& entry : poses) atlas.insert_or_assign(entry.first, entry.second);
  atlas.erase(removed);
  database.merge_submap(retained, removed, writes, archive, atlas, last_pose, transform);
}

void persist_feature_content(DBDriver* database, Node& replacement, int source, const cv::Mat& expected_target, const cv::Mat& expected_source) {
  if (!database) {
    if (!expected_source.empty()) throw std::runtime_error("Persistent region consolidation requires a database");
    return;
  }
  const auto archive = replacement.observation_history().encode();
  if (!expected_source.empty())
    database->consolidate_submap(replacement, source, archive, expected_target, expected_source);
  else
    database->refresh_submap(replacement, source, archive);
}

uint64_t persist_fused_submap(DBDriver& database, Node& replacement, int source, const std::vector<Node*>& links,
                              const std::map<int, StateGroupf>& poses, const StateGroupf& last, const cv::Mat& expected_target,
                              const cv::Mat& expected_source, int region, const cv::Mat& expected_region) {
  const auto sequence = database.submit_submap(replacement, source, replacement.observation_history().encode(), links, poses, last, expected_target,
                                               expected_source, region, expected_region);
  if (!sequence) throw std::runtime_error("Cannot submit fused graph: " + database.getWorkerError());
  return sequence;
}

void persist_localmap(const Memory& memory, LocalMap& map) {
  const auto& database = memory.database();
  if (!database) return;  // Unpersisted geometry must stay resident.
  const auto archive = LocalMapArchive::encode(map);
  memory.save_localmap(map.anchor_id, archive);
  LocalMapArchive::unload_navimap(map, archive, database);
}
void restore_localmaps(const Memory& memory, LocalMapCollection& maps) {
  const auto& database = memory.database();
  if (!database) return;
  // Startup already decodes these archives to restore summaries. Keep bounded
  // immutable 2D rasters from that work, without a second background SQL scan.
  // Unaligned history remains unpublished; over-budget maps keep their cold loader.
  size_t remaining = maps.startup_raster_budget();
  // Stream archives so restart peak memory does not scale with total payloads.
  for (int id : database->localmap_ids()) {
    const auto archive = database->load_localmap(id);
    auto map = LocalMapArchive::decode(archive);
    if (map->anchor_id != id) throw std::runtime_error("Localmap archive anchor mismatch");
    if (remaining) remaining -= map->retain_navimap_raster(remaining);
    LocalMapArchive::unload_navimap(*map, archive, database);
    maps.restore(map);
    memory.attach_navimap(std::move(map));
  }
}

ArchiveWriter::ArchiveWriter(const ParametersMap& parameters, std::shared_ptr<tools::ParallelExecutor<>> executor)
    : ArchiveWriter(ArchiveParams{}.withOverrides(parameters), std::move(executor)) {}
ArchiveWriter::ArchiveWriter(const ArchiveParams& configuration, std::shared_ptr<tools::ParallelExecutor<>> executor)
    : config_(configuration), executor_(std::move(executor)) {
  if (!executor_) {
    const size_t workers = config_.parallel_compression ? std::min<size_t>(4, std::max<unsigned>(1, std::thread::hardware_concurrency())) : 1;
    executor_ = std::make_shared<tools::ParallelExecutor<>>(workers);
  }
}
void ArchiveWriter::prepare(SensorRecord& record, const SensorData& input, const Node& draft, Statistics* stats) const {
  tools::Timer timer;
  const bool retained = input.id() >= 0 || config_.keep_intermediate_data;
  if (retained) record.observation.setUserData(input.userDataRaw());
  encodeSensorData(record, !draft.has_map_features() && config_.keep_sensor_data && retained, retained, false, executor());
  record.observation.setEnvSensors(input.envSensors());
  if (stats) stats->addStatistic(Statistics::kTimingMemCompressing_data(), timer.lap_seconds() * 1000.0f);
}
void TrajectoryWriter::flush(const Memory* memory) {
  if (pending_.empty()) return;
  if (memory && !memory->getDatabaseUrl().empty()) {
    const auto& database = memory->database();
    DBDriver::WriteBatch batch;
    batch.epoch = database->write_epoch();
    // Observations reference anchors already accepted at a keyframe boundary.
    batch.trajectory = pending_;
    if (!database->asyncWrite(std::move(batch))) throw std::runtime_error("Cannot submit trajectory persistence: " + database->getWorkerError());
  }
  pending_.clear();
}
void TrajectoryWriter::submit(Memory& memory, int node_id, const std::vector<OdomObservation>& current, std::shared_ptr<const LocalMap> sealed,
                              const Statistics* stats, bool save_wm) {
  auto observations = pending_;
  observations.insert(observations.end(), current.begin(), current.end());
  std::map<int, cv::Mat> archives;
  if (sealed && memory.database()) archives.emplace(sealed->anchor_id, LocalMapArchive::encode(*sealed));
  // Reserve acknowledgement ownership before submission, so an allocation
  // failure cannot leave an accepted archive without its retained geometry.
  if (!archives.empty()) navimap_.reserve(navimap_.size() + 1);
  const auto epoch = memory.database() ? memory.database()->write_epoch() : 0;
  const auto sequence = memory.save_keyframe(node_id, observations, archives, stats, save_wm);
  if (sequence && !archives.empty()) navimap_.push_back({std::move(sealed), archives.begin()->second, sequence, epoch});
  pending_.clear();
}
void TrajectoryWriter::reap(const Memory& memory) {
  const auto& database = memory.database();
  if (!database) return;
  if (!database->isWorkerHealthy()) throw std::runtime_error("Database writer failed: " + database->getWorkerError());
  for (auto it = navimap_.begin(); it != navimap_.end();) {
    if (!database->writeCommitted(it->sequence)) {
      ++it;
      continue;
    }
    // A graph transaction can advance the epoch without changing this map.
    // Recheck only then; don't permanently pin every map across a loop closure.
    bool unchanged = it->epoch == database->write_epoch();
    if (!unchanged) {
      const auto current = database->load_localmap(it->map->anchor_id);
      unchanged = current.type() == it->archive.type() && current.size() == it->archive.size() && cv::norm(current, it->archive, cv::NORM_INF) == 0;
    }
    // A superseded snapshot retains its unique geometry for remaining readers.
    if (unchanged) LocalMapArchive::unload_navimap(*std::const_pointer_cast<LocalMap>(it->map), it->archive, database);
    it = navimap_.erase(it);
  }
}
void TrajectoryWriter::record(const LocalMapCollection& maps, const Memory* memory, bool persisted) {
  if (!maps.active()) return;
  const auto& map = *maps.maps().at(maps.active_id());
  if (map.last_observation.sequence <= recorded_sequence_) return;
  if (persisted) {
    recorded_sequence_ = map.last_observation.sequence;
    return;
  }
  if (pending_.size() >= 128) flush(memory);
  pending_.push_back(map.last_observation);
  recorded_sequence_ = map.last_observation.sequence;
  if (pending_.size() >= 128) flush(memory);
}
std::vector<OdomObservation> TrajectoryWriter::recent(const Memory& memory, int limit) const {
  if (limit <= 0 || limit > 2048) throw std::invalid_argument("Invalid trajectory display limit");
  const auto retained = std::min(pending_.size(), size_t(limit));
  auto samples = retained < size_t(limit) ? memory.load_recent_trajectory(limit - int(retained)) : std::vector<OdomObservation>{};
  samples.insert(samples.end(), pending_.end() - retained, pending_.end());
  return samples;
}
}  // namespace sapphire::backend
