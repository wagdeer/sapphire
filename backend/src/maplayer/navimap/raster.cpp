#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/parameters.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/tools/logging.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <backend/tools/timer.hpp>
#include <backend/maplayer/topograph.hpp>
#include <chrono>
#include <cstdlib>
namespace sapphire::backend {
LocalMapRasterState::LocalMapRasterState()
    : occupancy_(std::make_unique<OccupancyGrid>(&source_cache_)), elevation_(std::make_unique<ElevationLayer>(&source_cache_)) {
  const char* path = std::getenv("PDR_NAVIMAP_TRACE");
  if (!path || !*path) path = std::getenv("PDR_NAVIGATION_TRACE"); // Existing replay tooling.
  if (path && *path) {
    navimap_trace_.open(path, std::ios::app);
    if (navimap_trace_ && navimap_trace_.tellp() == 0) navimap_trace_ << "wall_ns,sources_ms,occupancy_ms,elevation_ms,release_ms,changed,removed,maps,asynchronous\n";
    navimap_trace_.flush();
  }
}
void LocalMapRasterState::configure(const ParametersMap& parameters, std::function<void()> completed, bool asynchronous, std::function<void(const std::string&)> failed) {
  configure(NavimapParams{}.withOverrides(parameters), std::move(completed), asynchronous, std::move(failed));
}
void LocalMapRasterState::configure(const NavimapParams& parameters, std::function<void()> completed, bool asynchronous, std::function<void(const std::string&)> failed) {
  parameters.validate();
  auto occupancy = std::make_unique<OccupancyGrid>(&source_cache_, parameters);
  auto elevation = std::make_unique<ElevationLayer>(&source_cache_, parameters);
  clear();
  notify_completed_ = std::move(completed);
  notify_failed_ = std::move(failed);
  parameters_ = parameters;
  async_navimap_ = asynchronous;
  set_cache_budget(parameters.raster_cache_budget_bytes);
  occupancy_ = std::move(occupancy);
  elevation_ = std::move(elevation);
}
void LocalMapRasterState::clear_sources() {
  sources_initialized_ = false;
  seen_.clear();
  poses_.clear();
  cached_bytes_ = 0;
}
void LocalMapRasterState::clear() {
  worker_.reset();
  synchronous_map_.reset();
  clear_sources();
  source_cache_.clear();
  occupancy_->clear();
  elevation_->clear();
  occupancy_enabled_ = elevation_enabled_ = false;
  occupancy_updated_ = elevation_updated_ = false;
  submitted_revision_ = 0;
  projection_version_ = 0;
  failure_ = {};
}
void LocalMapRasterState::release_unused(bool elevation, bool sources) {
  if (elevation) { elevation_->clear(); elevation_enabled_ = false; }
  if (sources) source_cache_.clear();
}
void LocalMapRasterState::invalidate_projection() {
  projection_version_.reset();
  synchronous_map_.reset();
}
std::shared_ptr<const NavimapWorker::Map> LocalMapRasterState::navimap() const {
  if (failure_) std::rethrow_exception(failure_);
  if (!projection_version_) return {};
  if (worker_) {
    auto map = worker_->latest();
    return map && map->version >= *projection_version_ ? map : nullptr;
  }
  if (!synchronous_map_ || synchronous_map_->version != occupancy_->revision()) {
    auto snapshot = std::make_shared<NavimapWorker::Map>();
    snapshot->version = occupancy_->revision();
    occupancy_->getMaps(snapshot->state, snapshot->probability, snapshot->x, snapshot->y);
    snapshot->cell_size = occupancy_->getCellSize();
    synchronous_map_ = std::move(snapshot);
  }
  return synchronous_map_;
}
const std::map<int, StateGroupf>& LocalMapRasterState::render(
    const std::map<int, StateGroupf>& anchors, const LocalMapCollection& localmaps,
    bool occupancy, bool elevation, const std::set<int>& affected, int64_t stamp_ns, const StateGroupf& from_world) {
  if (failure_) std::rethrow_exception(failure_);
  occupancy_updated_ = elevation_updated_ = false;
  const bool rebuild = (occupancy && !occupancy_enabled_) || (elevation && !elevation_enabled_);
  std::vector<int> retired;
  const bool disable_occupancy = !occupancy && occupancy_enabled_;
  if (rebuild || disable_occupancy)
    for (const auto& [id, pose] : poses_) retired.push_back(id);
  if (rebuild) {
    // Rebuild both projections from the same source set. A newly requested
    // layer must not leave removed nodes in the other projection/worker.
    synchronous_map_.reset();
    occupancy_->clear();
    elevation_->clear();
  }
  occupancy_enabled_ = occupancy;
  elevation_enabled_ = elevation;
  if (!occupancy && !elevation) {
    static const std::map<int, sapphire::backend::StateGroupf> empty;
    return empty;
  }
  if (rebuild) clear_sources();
  const char* stage = "read_submaps";
  try {
    sapphire::backend::tools::Timer timer;
    // Keep the worker and its monotonic completion counter across demand
    // changes. Retire its old contributions instead of restarting at version 1.
    if (!occupancy && worker_ && (rebuild || disable_occupancy))
      submitted_revision_ = worker_->submit(retired, {}, stamp_ns);
    auto delta = update(anchors, localmaps, source_cache_, affected, from_world);
    if (rebuild) delta.removed.insert(delta.removed.end(), retired.begin(), retired.end());
    const double sourcesMs = timer.lap_seconds() * 1000.0;
    const auto& poses = this->poses();
    stage = "assemble_raster";
    if (occupancy && async_navimap_) {
      stage = "submit_raster";
      if (!worker_) worker_ = std::make_unique<NavimapWorker>(parameters_, notify_completed_, notify_failed_);
      std::map<int, NavimapWorker::Source> changed;
      for (const auto& [id, pose] : delta.changed) changed.emplace(id, NavimapWorker::Source{pose, source_cache_.find(id)->second});
      submitted_revision_ = worker_->submit(delta.removed, changed, stamp_ns);
      occupancy_updated_ = !delta.changed.empty() || !delta.removed.empty();
    } else if (occupancy) occupancy_updated_ = occupancy_->update_delta(delta.removed, delta.changed);
    const double occupancyMs = timer.lap_seconds() * 1000.0;
    if (elevation) elevation_updated_ = elevation_->update_delta(delta.removed, delta.changed);
    const double elevationMs = timer.lap_seconds() * 1000.0;
    if (!projection_version_) projection_version_ = target_revision();
    // Source rasters are transient. Projected contributions remain in the layers;
    // a loop correction obtains the exact archived source through update().
    for (const auto& [id, pose] : delta.changed) {
      occupancy_->release_source(id);
      elevation_->release_source(id);
      source_cache_.erase(id);
    }
    if (navimap_trace_.is_open()) {
      const double releaseMs = timer.lap_seconds() * 1000.0;
      navimap_trace_ << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
          << ',' << sourcesMs << ',' << occupancyMs << ',' << elevationMs << ',' << releaseMs
          << ',' << delta.changed.size() << ',' << delta.removed.size() << ',' << poses.size() << ',' << async_navimap_ << '\n';
      navimap_trace_.flush();
    }
    return poses;
  } catch (const std::exception& error) {
    // Async errors belong to task failure handling. Do not turn a failed source
    // read into a synchronous worker join on the input-processing thread.
    // Synchronous export callers can retry from an empty state.
    const auto failure = std::make_exception_ptr(std::runtime_error(std::string(stage) + ": " + error.what()));
    if (worker_) failure_ = failure;
    else clear();
    std::rethrow_exception(failure);
  }
}

void LocalMapRasterState::trim_cache() {
  while (cached_bytes_ > cache_budget_) {
    auto oldest = seen_.end();
    for (auto it = seen_.begin(); it != seen_.end(); ++it)
      if (it->second.source && (oldest == seen_.end() || it->second.used < oldest->second.used)) oldest = it;
    if (oldest == seen_.end()) break;
    cached_bytes_ -= oldest->second.bytes;
    oldest->second.source.reset();
    oldest->second.bytes = 0;
  }
}
LocalMapRasterDelta LocalMapRasterState::update(const std::map<int, StateGroupf>& anchors, const LocalMapCollection& maps, LocalGridCache& cache,
                                                const std::set<int>& affected, const StateGroupf& from_world) {
  std::set<int> initial;
  const bool refresh_all = !sources_initialized_ || !projection_version_;
  if (refresh_all) {
    for (const auto& [id, map] : maps.maps()) initial.insert(id);
    // A projection/alignment can also change which historical nodes are visible.
    for (const auto& [id, previous] : seen_) initial.insert(id);
  }
  const auto& candidates = refresh_all ? initial : affected;
  LocalMapRasterDelta delta;
  for (int id : candidates) {
    const auto found = maps.maps().find(id);
    const auto anchor = anchors.find(id);
    const auto previous = seen_.find(id);
    if (found == maps.maps().end() || anchor == anchors.end()) {
      if (previous != seen_.end()) {
        delta.removed.push_back(id);
        cache.erase(id);
        poses_.erase(id);
        cached_bytes_ -= previous->second.bytes;
        seen_.erase(previous);
      }
      continue;
    }
    const auto& map = found->second;
    const auto pose = from_world.compose(anchor->second);
    const bool sameContent = previous != seen_.end() && previous->second.map == map && previous->second.revision == map->revision;
    if (sameContent && previous->second.pose == pose) continue;
    auto source = sameContent && previous->second.source ? previous->second.source : map->raster_grid();
    cache.add(id, source);
    delta.changed.emplace_back(id, pose);
    poses_.insert_or_assign(id, pose);
    if (previous != seen_.end()) cached_bytes_ -= previous->second.bytes;
    const size_t bytes = sizeof(LocalGrid) + source->fused_cells.capacity() * sizeof(NavimapCell);
    // LocalMap rasters are fused cells, with no retained image/depth payload.
    const bool retain = source->fused_map && bytes <= cache_budget_;
    seen_.insert_or_assign(id, Seen{map, pose, map->revision, retain ? source : nullptr, retain ? bytes : 0, ++use_sequence_});
    if (retain) cached_bytes_ += bytes;
  }
  sources_initialized_ = true;
  trim_cache();
  return delta;
}


}
