#include <backend/maplayer/navimap.hpp>
#include <stdexcept>

namespace sapphire::backend {
void LocalMap::append(int sequence, double stamp, const StateGroupf& odom_from_body, std::shared_ptr<const LocalGrid> grid) {
  if (sealed || anchor_id <= 0 || odom_from_anchor.isNull() || odom_from_body.isNull() || !std::isfinite(stamp) || sequence <= 0 || !grid ||
      !std::isfinite(grid->cellSize) || grid->cellSize <= 0 || !grid->ground.valid() || !grid->obstacles.valid() || !grid->empty.valid() ||
      (observation_count && (sequence <= last_observation.sequence || stamp < last_observation.stamp)))
    throw std::invalid_argument("Invalid localmap observation");
  if (!observation_count) {
    const auto rotation = odom_from_anchor.rotationMatrix();
    const float yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const auto horizontal = StateGroupf::fromRotationTranslation(
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix(), odom_from_anchor.translationVector());
    navimap.anchor_from_grid = odom_from_anchor.inverse().compose(horizontal);
  }
  const auto pose = odom_from_anchor.inverse().compose(odom_from_body);
  // Resolution is archive metadata even when online navigation is disabled.
  if (navimap.cell_size == 0) navimap.cell_size = grid->cellSize;
  if (navimap_enabled) {
    navimap.integrate(*grid, navimap.anchor_from_grid.inverse().compose(pose), sequence);
  }
  last_observation = {anchor_id, sequence, stamp, pose, odom_from_body};
  if (!observation_count) first_observation = last_observation;
  ++observation_count;
  ++revision;
}

void LocalMap::integrate_depth(const StateGroupf& odom_from_body, const LocalGrid& grid) {
  if (sealed || !observation_count || !odom_from_anchor.isValid() || !odom_from_body.isValid() ||
      !grid.ground.valid() || !grid.obstacles.valid() || !grid.empty.valid())
    throw std::invalid_argument("Depth fusion requires an initialized active localmap and valid geometry");
  if (!navimap_enabled || grid.skip_integration) return;
  if (!navimap_resident()) materialize_navimap();
  // Depth is consumed in capture order. Its sequence identifies the current
  // visual interval, keeping inter-node fusion ordered without inventing a pose
  // journal entry or changing the node's first/last observation timestamps.
  navimap.integrate(grid, odom_from_anchor.compose(navimap.anchor_from_grid).inverse().compose(odom_from_body), last_observation.sequence);
  ++revision;
}

void LocalMap::validate_summary() const {
  if (first_observation.anchor_id != anchor_id || last_observation.anchor_id != anchor_id ||
      anchor_id <= 0 || !observation_count || !visit_count || visit_count > observation_count || first_observation.sequence <= 0 ||
      last_observation.sequence < first_observation.sequence || observation_count > uint64_t(last_observation.sequence - first_observation.sequence) + 1 ||
      !std::isfinite(first_observation.stamp) || !std::isfinite(last_observation.stamp) || !first_observation.anchor_from_body.isValid() ||
      !first_observation.odom_from_body.isValid() || !last_observation.anchor_from_body.isValid() || !last_observation.odom_from_body.isValid())
    throw std::invalid_argument("Invalid localmap summary");
}

std::shared_ptr<const LocalGrid> LocalMap::raster_grid() const {
  if (raster_revision_ == revision)
    if (auto cached = raster_cache_.lock()) {
      startup_raster_.reset();  // Transfer lifetime to the actual reader/cache.
      return cached;
    }
  auto grid = std::make_shared<LocalGrid>();
  grid->cellSize = navimap.cell_size;
  grid->anchor_from_grid = navimap.anchor_from_grid;
  grid->fused_map = true;
  grid->fused_cells = navimap_loader_ ? navimap_loader_(*this).ordered_cells() : navimap.ordered_cells();
  raster_cache_ = grid;
  raster_revision_ = revision;
  return grid;
}
size_t LocalMap::retain_navimap_raster(size_t budget) {
  if (!sealed || !navimap_resident()) throw std::logic_error("Preload requires a decoded sealed map");
  const size_t estimate = sizeof(LocalGrid) + navimap.cells.size() * sizeof(NavimapCell);
  if (estimate > budget) return 0;
  auto raster = raster_grid();
  const size_t bytes = sizeof(LocalGrid) + raster->fused_cells.capacity() * sizeof(NavimapCell);
  if (bytes > budget) return 0;
  startup_raster_ = std::move(raster);
  return bytes;
}
void LocalMap::unload_navimap(NavimapLoader loader) {
  if (!sealed || !loader) throw std::logic_error("Only committed sealed maps can unload navigation");
  navimap_loader_ = std::move(loader);
  decltype(navimap.cells){}.swap(navimap.cells);
  decltype(navimap.distance_tiles){}.swap(navimap.distance_tiles);
  // Weak cache preserves sharing with a live navigation reader, without pinning it.
}
void LocalMap::materialize_navimap() {
  if (navimap_loader_) navimap = navimap_snapshot();
  navimap_loader_ = {};
  startup_raster_.reset();
  raster_cache_.reset();
}
NavimapGrid LocalMap::navimap_snapshot() const {
  return navimap_loader_ ? navimap_loader_(*this) : navimap;
}

}  // namespace sapphire::backend
