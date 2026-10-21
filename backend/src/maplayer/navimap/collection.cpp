#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/parameters.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/tools/logging.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace sapphire::backend {
void LocalMapCollection::begin(int anchor_id, const StateGroupf& odom_from_anchor) {
  if (active() || anchor_id <= 0 || odom_from_anchor.isNull() || maps_.count(anchor_id)) throw std::logic_error("Invalid localmap anchor");
  auto map = std::make_shared<LocalMap>();
  map->navimap.settings = settings_;
  map->navimap_enabled = navimap_enabled_;
  map->anchor_id = anchor_id;
  map->odom_from_anchor = odom_from_anchor;
  active_map_ = map;
  maps_.emplace(anchor_id, std::move(map));
  active_id_ = anchor_id;
  changed_.insert(anchor_id);
}
void LocalMapCollection::integrate_depth(const StateGroupf& odom_from_body, const LocalGrid& grid) {
  if (!active()) throw std::logic_error("No active localmap");
  const auto revision = active_map_->revision;
  active_map_->integrate_depth(odom_from_body, grid);
  if (active_map_->revision != revision) changed_.insert(active_id_);
}
std::shared_ptr<const LocalMap> LocalMapCollection::release_sealed() {
  if (!active()) return {};
  auto result = active_map_;
  if (!result->sealed) throw std::logic_error("Prepare and persist the active map before release");
  active_id_ = 0;
  active_map_.reset();
  return result;
}
void LocalMapCollection::restore(std::shared_ptr<LocalMap> map) {
  if (!map || !map->sealed || !map->observation_count || maps_.count(map->anchor_id)) throw std::invalid_argument("Invalid restored localmap");
  map->navimap_enabled = navimap_enabled_;
  advance_sequence(map->last_observation.sequence);
  const int id = map->anchor_id;
  maps_.emplace(id, std::move(map));
  changed_.insert(id);
}

void LocalMapCollection::advance_sequence(int last) {
  if (last < 0 || last == std::numeric_limits<int>::max()) throw std::runtime_error("Exhausted localmap sequence IDs");
  next_sequence_ = std::max(next_sequence_, last + 1);
}
void LocalMapCollection::clear() {
  for (const auto& [id, map] : maps_) changed_.insert(id);
  maps_.clear();
  active_map_.reset();
  active_id_ = 0;
  next_sequence_ = 1;
}
void LocalMapCollection::configure(const ParametersMap& parameters, bool navimap_enabled) {
  configure(NavimapParams{}.withOverrides(parameters), navimap_enabled);
}
void LocalMapCollection::configure(const NavimapParams& parameters, bool navimap_enabled) {
  parameters.validate();
  if (!maps_.empty()) throw std::logic_error("Cannot reconfigure an existing LocalMap collection");
  navimap_enabled_ = navimap_enabled;
  startup_raster_budget_ = parameters.raster_cache_budget_bytes;
  const auto log_odds = [](float value) { return std::log(value / (1 - value)); };
  settings_.hit = log_odds(parameters.raster_hit_probability);
  settings_.miss = log_odds(parameters.raster_miss_probability);
  settings_.minimum = log_odds(parameters.raster_min_probability);
  settings_.maximum = log_odds(parameters.raster_max_probability);
  settings_.footprint = parameters.raster_footprint_radius_m;
  settings_.truncation = parameters.tsdf_truncation_cells * parameters.cell_size_m;
  settings_.surface_weight = parameters.tsdf_surface_weight;
  settings_.maximum_weight = parameters.tsdf_maximum_weight;
}

std::map<int, StateGroupf> LocalMapCollection::raster_poses(const std::map<int, StateGroupf>& anchors, LocalGridCache& cache) const {
  std::map<int, StateGroupf> result;
  for (const auto& [id, map] : maps_) {
    const auto anchor = anchors.find(id);
    if (anchor == anchors.end()) continue;
    cache.add(id, map->raster_grid());
    result.emplace(id, anchor->second);
  }
  return result;
}

LocalMapMerge LocalMapCollection::prepare_merge(int retained, int removed, const StateGroupf& retained_from_removed) const {
  const auto target = maps_.find(retained), source = maps_.find(removed);
  if (target == maps_.end() || source == maps_.end() || active_id_ == retained || (active() && active_id_ != removed))
    throw std::invalid_argument("Missing or inconsistent localmap merge anchors");
  LocalMapMerge draft;
  draft.target_ = target->second;
  draft.source_ = source->second;
  draft.target_revision_ = target->second->revision;
  draft.source_revision_ = source->second->revision;
  draft.active_id_ = active_id_;
  draft.merged_ = fuse_localmaps(*target->second, *source->second, retained_from_removed);
  return draft;
}
bool LocalMapCollection::can_apply(const LocalMapMerge& draft) const {
  if (!draft.target_ || !draft.source_ || !draft.merged_ || active_id_ != draft.active_id_) return false;
  const auto target = maps_.find(draft.target_->anchor_id), source = maps_.find(draft.source_->anchor_id);
  if (target == maps_.end() || source == maps_.end() || target == source || target->second != draft.target_ || source->second != draft.source_ ||
      target->second->revision != draft.target_revision_ || source->second->revision != draft.source_revision_ ||
      draft.merged_->anchor_id != draft.target_->anchor_id || active_id_ == target->first)
    return false;
  return true;
}
bool LocalMapCollection::apply_merge(const LocalMapMerge& draft) {
  if (!can_apply(draft)) return false;
  const auto target = maps_.find(draft.target_->anchor_id), source = maps_.find(draft.source_->anchor_id);
  target->second = draft.merged_;
  if (active_id_ == source->first) {
    active_id_ = target->first;
    active_map_ = draft.merged_;
  }
  changed_.insert(target->first);
  changed_.insert(source->first);
  maps_.erase(source);
  return true;
}

}
