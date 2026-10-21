#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <stdexcept>

namespace sapphire::backend {
namespace {
int32_t index(float value, float size) {
  const double cell = std::floor(double(value) / size);
  if (!std::isfinite(cell) || std::abs(cell) > 1000000) throw std::invalid_argument("Navigation coordinate out of range");
  return static_cast<int32_t>(cell);
}
struct FrameCell {
  uint8_t flags = 0;  // miss, hit, footprint
  float projection_z = std::numeric_limits<float>::quiet_NaN();
  float height = std::numeric_limits<float>::quiet_NaN();
};
}  // namespace
void NavimapGrid::integrate(const LocalGrid& frame, const StateGroupf& pose, int sequence) {
  if (frame.fused_map || !pose.isValid() || sequence <= 0 || !std::isfinite(frame.cellSize) || frame.cellSize <= 0)
    throw std::invalid_argument("Invalid navigation observation");
  if (settings.footprint / frame.cellSize > 1000) throw std::invalid_argument("Navigation footprint out of range");
  if (cell_size == 0) cell_size = frame.cellSize;
  if (std::abs(cell_size - frame.cellSize) > 1e-6f) throw std::invalid_argument("Navigation resolution changed");
  if (frame.skip_integration) return;
  if (!frame.depth_images.empty()) {
    integrate_depth(frame, pose, sequence);
    return;
  }
  const auto rotation = pose.rotationMatrix();
  const auto translation = pose.translationVector();
  std::unordered_map<uint64_t, FrameCell> observed;
  observed.reserve(frame.ground.size() + frame.obstacles.size() + frame.empty.size());
  const auto collect = [&](const GridCells& points, uint8_t flag, bool elevation) {
    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3f p = rotation * Eigen::Vector3f(points.xy.x[i], points.xy.y[i], points.is3D() ? points.z[i] : 0) + translation;
      if (!p.allFinite()) throw std::invalid_argument("Nonfinite navigation observation");
      auto& cell = observed[key(index(p.x(), cell_size), index(p.y(), cell_size))];
      cell.flags |= flag;
      cell.projection_z = std::isfinite(cell.projection_z) ? std::max(cell.projection_z, p.z()) : p.z();
      if (elevation && points.is3D()) cell.height = std::isfinite(cell.height) ? std::max(cell.height, p.z()) : p.z();
    }
  };
  // Free rays, when enabled by LocalGridMaker, have already used this frame's real origin.
  collect(frame.ground, 1, true);
  collect(frame.empty, 1, false);
  collect(frame.obstacles, 2, true);
  if (settings.footprint >= cell_size * 1.5f) {
    const int cx = index(pose.x(), cell_size), cy = index(pose.y(), cell_size);
    const int radius = std::ceil(settings.footprint / cell_size);
    for (int y = cy - radius; y <= cy + radius; ++y)
      for (int x = cx - radius; x <= cx + radius; ++x) {
        const float dx = (x + .5f) * cell_size - pose.x(), dy = (y + .5f) * cell_size - pose.y();
        if (dx * dx + dy * dy <= settings.footprint * settings.footprint) observed[key(x, y)].flags |= 4;
      }
  }
  for (const auto& [k, observation] : observed) {
    auto& cell = cells[k];
    cell.x = int32_t(k >> 32);
    cell.y = int32_t(k);
    if (observation.flags & 4) {
      cell.evidence = {0, settings.minimum, settings.minimum};
      cell.flags = 2;
    } else {
      // A floor return below a table does not negate the table in the same XY
      // column. One hit wins within this frame; later free frames can clear it.
      if ((observation.flags & 1) && !(observation.flags & 2)) cell.evidence.then({settings.miss, settings.minimum, settings.maximum});
      if (observation.flags & 2) cell.evidence.then({settings.hit, settings.minimum, settings.maximum});
      cell.flags = observation.flags & 2 ? 1 : 0;
    }
    // Footprint evidence belongs to the observer plane, not to an obstacle
    // cleared in the same XY cell. Otherwise pitch/roll reprojects it sideways.
    cell.projection_z = (observation.flags & 4) ? pose.z() :
        (std::isfinite(observation.projection_z) ? observation.projection_z : pose.z());
    cell.sequence = sequence;
    if (std::isfinite(observation.height)) {
      cell.height = observation.height;
      cell.height_sequence = sequence;
    }
  }
}
std::vector<NavimapCell> NavimapGrid::ordered_cells() const {
  if (!distance_tiles.empty()) return surface_cells();
  std::vector<NavimapCell> result;
  result.reserve(cells.size());
  for (const auto& [key, cell] : cells) result.push_back(cell);
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.y != b.y ? a.y < b.y : a.x < b.x; });
  return result;
}
namespace {
void merge_cells(NavimapGrid& target, const std::unordered_map<uint64_t, NavimapCell>& source, float source_cell_size, const NavimapSettings& source_settings,
                 const StateGroupf& pose) {
  const float cell_size = target.cell_size;
  const auto& settings = target.settings;
  if (!pose.isValid() || std::abs(cell_size - source_cell_size) > 1e-6f || settings.hit != source_settings.hit || settings.miss != source_settings.miss ||
      settings.minimum != source_settings.minimum || settings.maximum != source_settings.maximum)
    throw std::invalid_argument("Incompatible navigation maps");
  // Rotation may project several source cells into one target cell. Keep one
  // (latest, then occupied) contribution, rather than multiplying its confidence.
  struct ProjectedCell {
    NavimapCell cell;
    int32_t source_x, source_y;
  };
  std::unordered_map<uint64_t, ProjectedCell> projected;
  projected.reserve(source.size());
  for (const auto& [source_key, original] : source) {
    auto cell = original;
    const auto p = pose.transformPoint({(cell.x + .5f) * cell_size, (cell.y + .5f) * cell_size, cell.projection_z});
    if (std::isfinite(cell.height)) cell.height = pose.transformPoint({(cell.x + .5f) * cell_size, (cell.y + .5f) * cell_size, cell.height}).z();
    cell.projection_z = p.z();
    cell.x = index(p.x(), cell_size);
    cell.y = index(p.y(), cell_size);
    const auto k = NavimapGrid::key(cell.x, cell.y);
    const auto existing = projected.find(k);
    bool replace = existing == projected.end();
    if (!replace) {
      const auto& winner = existing->second;
      // ordered_cells() previously broke equal sequence/flag ties by source y,x.
      // Preserve that rule explicitly, independent of hash iteration order.
      replace = cell.sequence > winner.cell.sequence ||
                (cell.sequence == winner.cell.sequence &&
                 (cell.flags > winner.cell.flags ||
                  (cell.flags == winner.cell.flags &&
                   (original.y < winner.source_y || (original.y == winner.source_y && original.x < winner.source_x)))));
    }
    if (replace) projected.insert_or_assign(k, ProjectedCell{cell, original.x, original.y});
  }
  for (const auto& [k, winner] : projected) {
    const auto& newer = winner.cell;
    auto [found, inserted] = target.cells.emplace(k, newer);
    if (inserted) continue;
    auto& cell = found->second;
    cell.evidence.then(newer.evidence);
    cell.projection_z = newer.projection_z;
    cell.sequence = newer.sequence;
    cell.flags = newer.flags;
    if (std::isfinite(newer.height)) {
      cell.height = newer.height;
      cell.height_sequence = newer.height_sequence;
    }
  }
}
}  // namespace

void NavimapGrid::merge(const NavimapGrid& source, const StateGroupf& pose) {
  if (!source.distance_tiles.empty() && (settings.truncation != source.settings.truncation ||
      settings.surface_weight != source.settings.surface_weight || settings.maximum_weight != source.settings.maximum_weight))
    throw std::invalid_argument("Incompatible navimap TSDF settings");
  const auto grid_from_source = anchor_from_grid.inverse().compose(pose).compose(source.anchor_from_grid);
  merge_cells(*this, source.cells, source.cell_size, source.settings, grid_from_source);
  merge_distance(source, grid_from_source);
}

}  // namespace sapphire::backend
