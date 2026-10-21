#pragma once
#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/maplayer/navimap/local_grid.hpp>
#include <backend/common/odom_observation.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/maplayer/navimap/navimap_worker.hpp>
#include <backend/maplayer/navimap/elevation_layer.hpp>
#include <set>
#include <limits>
#include <stdexcept>
#include <functional>
#include <optional>

namespace sapphire::backend {
// Memory scales with observed cells; per-frame poses belong to the database journal.
struct LocalMap {
  int anchor_id = 0;
  StateGroupf odom_from_anchor;
  uint64_t observation_count = 0;
  uint64_t visit_count = 1;
  OdomObservation first_observation, last_observation;
  NavimapGrid navimap;
  bool navimap_enabled = true;
  std::shared_ptr<const LocalGrid> raster_grid() const;
  // Mapping-owner thread only. The loader must read the exact committed snapshot.
  // Load complete fusion state once; snapshots derive contours from that state.
  // Readers may retain immutable rasters; releasing this map never invalidates them.
  using NavimapLoader = std::function<NavimapGrid(const LocalMap&)>;
  void unload_navimap(NavimapLoader loader);
  void materialize_navimap();
  // Keep an already decoded startup raster until its first consumer takes it.
  size_t retain_navimap_raster(size_t budget);
  bool navimap_resident() const { return !navimap_loader_; }
  NavimapGrid navimap_snapshot() const;
  uint64_t revision = 0;
  bool sealed = false;
  void append(int sequence, double stamp, const StateGroupf& odom_from_body, std::shared_ptr<const LocalGrid> grid);

  // Late depth uses its capture pose in the current anchor; it is not a trajectory sample.
  void integrate_depth(const StateGroupf& odom_from_body, const LocalGrid& grid);

  void validate_summary() const;

 private:
  NavimapLoader navimap_loader_;
  mutable std::shared_ptr<const LocalGrid> startup_raster_;
  mutable std::weak_ptr<const LocalGrid> raster_cache_;
  mutable uint64_t raster_revision_ = UINT64_MAX;
};


// Multiple submaps and their drafts. The content above belongs to one Node;
// the collection below owns the active content and its sequence/residency state.
std::shared_ptr<LocalMap> fuse_localmaps(const LocalMap& target, const LocalMap& source, const StateGroupf& target_from_source);
class LocalMapMerge {
 public:
  std::shared_ptr<const LocalMap> merged() const { return merged_; }
  int retained_id() const { return target_ ? target_->anchor_id : 0; }
  int removed_id() const { return source_ ? source_->anchor_id : 0; }

 private:
  friend class LocalMapCollection;
  std::shared_ptr<const LocalMap> target_, source_;
  std::shared_ptr<LocalMap> merged_;
  uint64_t target_revision_ = 0, source_revision_ = 0;
  int active_id_ = 0;
};

class LocalMapCollection {
 public:
  void configure(const ParametersMap& parameters, bool navimap_enabled = true);
  void configure(const NavimapParams& parameters, bool navimap_enabled = true);
  bool active() const { return active_id_ > 0; }
  int active_id() const { return active_id_; }
  int observation_count() const { return next_sequence_ - 1; }
  // Owner-thread views. The active entry evolves until seal(); consumers that
  // retain render/query data take immutable raster_grid() snapshots instead.
  const std::map<int, std::shared_ptr<const LocalMap>>& maps() const { return maps_; }
  void begin(int anchor_id, const StateGroupf& odom_from_anchor);
  void append(double stamp, const StateGroupf& odom_from_body, std::shared_ptr<const LocalGrid> grid) {
    if (!active() || next_sequence_ == std::numeric_limits<int>::max()) throw std::logic_error("No active localmap or exhausted observation IDs");
    active_map_->append(next_sequence_, stamp, odom_from_body, std::move(grid));
    changed_.insert(active_id_);
    ++next_sequence_;
  }
  void integrate_depth(const StateGroupf& odom_from_body, const LocalGrid& grid);
  // Consumed once by the mapping owner, then shared with raster consumers.
  std::set<int> take_changes() { std::set<int> out; out.swap(changed_); return out; }
  // Storage preparation/persistence precedes this owner-only state transition.
  LocalMap* active_map() { return active_map_.get(); }
  std::shared_ptr<const LocalMap> release_sealed();
  void restore(std::shared_ptr<LocalMap> localmap);
  LocalMapMerge prepare_merge(int retained, int removed, const StateGroupf& retained_from_removed) const;
  bool can_apply(const LocalMapMerge& draft) const;
  bool apply_merge(const LocalMapMerge& draft);
  void clear();
  void advance_sequence(int last);
  size_t startup_raster_budget() const { return navimap_enabled_ ? startup_raster_budget_ : 0; }
  std::map<int, StateGroupf> raster_poses(const std::map<int, StateGroupf>& anchors, LocalGridCache& cache) const;

 private:
  std::set<int> changed_;
  NavimapSettings settings_;
  unsigned int startup_raster_budget_ = 64 * 1024 * 1024;
  bool navimap_enabled_ = true;
  int active_id_ = 0;
  int next_sequence_ = 1;
  std::shared_ptr<LocalMap> active_map_;
  std::map<int, std::shared_ptr<const LocalMap>> maps_;
};

// One raster contribution per node. Changed content replaces its old contribution.
struct LocalMapRasterDelta {
  std::vector<int> removed;
  std::vector<std::pair<int, StateGroupf>> changed;
};
class LocalMapRasterState {
 public:
  LocalMapRasterState();
  void configure(const ParametersMap& parameters, std::function<void()> completed = {}, bool asynchronous = true,
                 std::function<void(const std::string&)> failed = {});
  void configure(const NavimapParams& parameters, std::function<void()> completed = {}, bool asynchronous = true,
                 std::function<void(const std::string&)> failed = {});
  // First render (or enabling a layer) seeds all maps. Subsequent calls visit
  // only affected IDs, including removals; an empty set means no changes.
  // The producer owns source deltas, synchronous layers and the async worker.
  // Consumers retain completed immutable rasters, never mutable layer state.
  const std::map<int, StateGroupf>& render(const std::map<int, StateGroupf>& anchors, const LocalMapCollection& maps,
      bool occupancy, bool elevation, const std::set<int>& affected = {}, int64_t stamp_ns = 0,
      const StateGroupf& from_world = StateGroupf::identity());
  std::shared_ptr<const NavimapWorker::Map> navimap() const;
  const grid_map::GridMap* elevation_map() const { return elevation_->gridMap(); }
  bool occupancy_updated() const { return occupancy_updated_; }
  bool elevation_updated() const { return elevation_updated_; }
  uint64_t target_revision() const { return worker_ ? submitted_revision_ : occupancy_->revision(); }
  void drain() { if (worker_) worker_->drain(); }
  void release_unused(bool elevation, bool sources);
  void set_cache_budget(size_t bytes) { cache_budget_ = bytes; trim_cache(); }
  size_t cached_bytes() const { return cached_bytes_; }
  LocalMapRasterDelta update(const std::map<int, StateGroupf>& anchors, const LocalMapCollection& maps, LocalGridCache& cache,
                             const std::set<int>& affected = {}, const StateGroupf& from_world = StateGroupf::identity());
  const std::map<int, StateGroupf>& poses() const { return poses_; }
  void clear();
  // A new publication transform changes poses, not immutable local geometry.
  // Keep the worker/cache; hide old-frame output until this projection is applied.
  void invalidate_projection();

 private:
  std::exception_ptr failure_;
  // Empty until a changed projection has been submitted; otherwise the oldest
  // completion that belongs to the current publication frame.
  std::optional<uint64_t> projection_version_ = 0;
  void clear_sources();
  bool sources_initialized_ = false;
  LocalGridCache source_cache_;
  std::unique_ptr<OccupancyGrid> occupancy_;
  std::unique_ptr<ElevationLayer> elevation_;
  std::function<void()> notify_completed_;
  std::function<void(const std::string&)> notify_failed_;
  std::unique_ptr<NavimapWorker> worker_;
  mutable std::shared_ptr<const NavimapWorker::Map> synchronous_map_;
  NavimapParams parameters_;
  bool async_navimap_ = true;
  bool occupancy_enabled_ = false, elevation_enabled_ = false;
  bool occupancy_updated_ = true, elevation_updated_ = true;
  uint64_t submitted_revision_ = 0;
  std::ofstream navimap_trace_;
  struct Seen {
    std::shared_ptr<const LocalMap> map;
    StateGroupf pose;
    uint64_t revision = 0;
    std::shared_ptr<const LocalGrid> source;
    size_t bytes = 0;
    uint64_t used = 0;
  };
  void trim_cache();
  size_t cache_budget_ = 64 * 1024 * 1024, cached_bytes_ = 0;
  uint64_t use_sequence_ = 0;
  std::map<int, Seen> seen_;
  std::map<int, StateGroupf> poses_;
};


} // namespace sapphire::backend
