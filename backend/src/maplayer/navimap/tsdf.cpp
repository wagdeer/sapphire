#include <algorithm>
#include <climits>
#include <backend/maplayer/navimap/local_grid.hpp>
#include <stdexcept>

namespace sapphire::backend {
namespace {
using Distance = NavimapGrid::Distance;
uint64_t key(int x, int y) { return NavimapGrid::key(x, y); }
int index(float value, float size) {
  const double cell = std::floor(double(value) / size);
  if (!std::isfinite(cell) || std::abs(cell) > 1000000) throw std::invalid_argument("Navimap coordinate out of range");
  return static_cast<int>(cell);
}
Distance& value_at(NavimapGrid& map, int x, int y) {
  const int bx = NavimapGrid::tile_index(x), by = NavimapGrid::tile_index(y);
  return map.distance_tiles[key(bx, by)].values[(y - by * 8) * 8 + x - bx * 8];
}
const Distance* lookup(const NavimapGrid& map, int x, int y) {
  const int bx = NavimapGrid::tile_index(x), by = NavimapGrid::tile_index(y);
  const auto tile = map.distance_tiles.find(key(bx, by));
  if (tile == map.distance_tiles.end()) return nullptr;
  const auto& value = tile->second.values[(y - by * 8) * 8 + x - bx * 8];
  return value.weight > 0 ? &value : nullptr;
}
void accumulate(Distance& value, const Distance& observation, float maximum_weight) {
  const float sum = value.weight + observation.weight;
  value.distance = (value.distance * value.weight + observation.distance * observation.weight) / sum;
  value.weight = std::min(maximum_weight, sum);
  value.height = observation.height;
  value.sequence = std::max(value.sequence, observation.sequence);
}
struct Candidate {
  float squared = INFINITY, height = 0;
};
struct Observation {
  uint8_t flags = 0;
  float projection_z = -INFINITY, height = NAN;
};
struct Endpoint {
  Eigen::Vector3f point;
  int x, y;
};
struct Workspace {
  std::vector<Candidate> candidates;
  std::vector<Endpoint> endpoints;
  std::unordered_map<uint64_t, Observation> observed;
};
}  // namespace

void NavimapGrid::integrate_depth(const LocalGrid& frame, const StateGroupf& pose, int sequence) {
  // Per-observation contract checks precede all point/cell loops.
  if (!frame.ground_plane.allFinite() || frame.min_obstacle_height <= 0 || frame.max_obstacle_height <= frame.min_obstacle_height ||
      settings.truncation < cell_size || settings.truncation / cell_size > 64 || settings.surface_weight <= 0 ||
      settings.maximum_weight < settings.surface_weight)
    throw std::invalid_argument("Invalid navimap distance observation");
  for (const auto* points : {&frame.ground, &frame.empty, &frame.obstacles})
    if (!points->valid() || (!points->empty() && !points->is3D())) throw std::invalid_argument("TSDF requires body-frame 3-D points");
  for (const auto& image : frame.depth_images)
    if (image.meters.empty() || image.meters.type() != CV_32FC1 || !image.camera.isValidForProjection() ||
        image.camera.imageSize() != image.meters.size() || !image.camera.cameraToBase().isValid())
      throw std::invalid_argument("Invalid normalized navimap depth image");
  thread_local Workspace work;
  work.observed.clear();
  work.endpoints.clear();
  const auto rotation = pose.rotationMatrix();
  const auto translation = pose.translationVector();
  const int radius = std::ceil(settings.truncation / cell_size);
  int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;
  // The obstacle pass both records same-frame precedence and bounds TSDF work.
  const auto collect = [&](const GridCells& points, auto surface, auto elevation) {
    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3f p = rotation * Eigen::Vector3f(points.xy.x[i], points.xy.y[i], points.z[i]) + translation;
      const int x = index(p.x(), cell_size), y = index(p.y(), cell_size);
      auto& observation = work.observed[key(x, y)];
      observation.flags |= surface ? 2 : 1;
      observation.projection_z = std::max(observation.projection_z, p.z());
      if constexpr (decltype(elevation)::value) observation.height = std::isfinite(observation.height) ? std::max(observation.height, p.z()) : p.z();
      if constexpr (decltype(surface)::value) {
        work.endpoints.push_back({p, x, y});
        x0 = std::min(x0, x - radius);
        y0 = std::min(y0, y - radius);
        x1 = std::max(x1, x + radius);
        y1 = std::max(y1, y + radius);
      }
    }
  };
  collect(frame.ground, std::false_type{}, std::true_type{});
  collect(frame.empty, std::false_type{}, std::false_type{});
  collect(frame.obstacles, std::true_type{}, std::true_type{});
  // Same-column surfaces win within the observation. TSDF carries occupied
  // evidence; these cells carry free-space evidence and floor elevation only.
  for (const auto& [k, observation] : work.observed) {
    if (observation.flags != 1) continue;
    auto& cell = cells[k];
    cell.x = int32_t(k >> 32);
    cell.y = int32_t(k);
    cell.evidence.then({settings.miss, settings.minimum, settings.maximum});
    cell.projection_z = observation.projection_z;
    cell.sequence = sequence;
    if (std::isfinite(observation.height)) {
      cell.height = observation.height;
      cell.height_sequence = sequence;
    }
    cell.flags = 0;
  }
  if (settings.footprint >= cell_size * 1.5f) {
    const int cx = index(pose.x(), cell_size), cy = index(pose.y(), cell_size);
    const int r = std::ceil(settings.footprint / cell_size);
    for (int y = cy - r; y <= cy + r; ++y)
      for (int x = cx - r; x <= cx + r; ++x) {
        const float dx = (x + .5f) * cell_size - pose.x(), dy = (y + .5f) * cell_size - pose.y();
        if (dx * dx + dy * dy > settings.footprint * settings.footprint) continue;
        auto& cell = cells[key(x, y)];
        cell.x = x;
        cell.y = y;
        cell.evidence = {0, settings.minimum, settings.minimum};
        cell.projection_z = pose.z();
        cell.sequence = sequence;
        cell.flags = 2;
      }
  }
  if (work.endpoints.empty()) return;
  const size_t width = size_t(x1 - x0) + 1, height = size_t(y1 - y0) + 1;
  work.candidates.assign(width * height, {});
  for (const auto& endpoint : work.endpoints) {
    const auto& p = endpoint.point;
    const int cx = endpoint.x, cy = endpoint.y;
    for (int y = cy - radius; y <= cy + radius; ++y)
      for (int x = cx - radius; x <= cx + radius; ++x) {
        const float dx = (x + .5f) * cell_size - p.x(), dy = (y + .5f) * cell_size - p.y(), squared = dx * dx + dy * dy;
        if (squared > settings.truncation * settings.truncation) continue;
        auto& candidate = work.candidates[size_t(y - y0) * width + x - x0];
        if (squared < candidate.squared || (squared == candidate.squared && p.z() > candidate.height)) candidate = {squared, p.z()};
      }
  }
  const auto body_from_grid = pose.inverse();
  const Eigen::Vector3f normal = body_from_grid.rotationMatrix().transpose() * frame.ground_plane.head<3>();
  const float offset = frame.ground_plane.head<3>().dot(body_from_grid.translationVector()) + frame.ground_plane.w();
  for (const auto& image : frame.depth_images) {
    const auto& camera = image.camera;
    const auto grid_from_camera = pose.compose(camera.cameraToBase());
    const auto camera_from_grid = grid_from_camera.inverse();
    const auto R = camera_from_grid.rotationMatrix();
    const auto t = camera_from_grid.translationVector();
    const auto origin = grid_from_camera.translationVector();
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        const auto& candidate = work.candidates[size_t(y - y0) * width + x - x0];
        if (!std::isfinite(candidate.squared)) continue;
        const Eigen::Vector3f center((x + .5f) * cell_size, (y + .5f) * cell_size, candidate.height);
        const Eigen::Vector3f p = R * center + t;
        if (p.z() <= 0) continue;
        const double u = camera.fx() * p.x() / p.z() + camera.cx(), v = camera.fy() * p.y() / p.z() + camera.cy();
        if (u <= -.5 || v <= -.5 || u >= image.meters.cols - .5 || v >= image.meters.rows - .5) continue;
        const float depth = image.meters.at<float>(std::lround(v), std::lround(u));
        if (!std::isfinite(depth) || depth <= frame.min_depth || depth >= frame.max_depth) continue;
        const Eigen::Vector3f measured = grid_from_camera.transformPoint(p * (depth / p.z()));
        const float obstacle_height = normal.dot(measured) + offset;
        if (obstacle_height < frame.min_obstacle_height || obstacle_height > frame.max_obstacle_height) continue;
        // Horizontal projective distance preserves the 2-D metric for pitched rays.
        const float distance = (depth / p.z() - 1.f) * (center - origin).head<2>().norm();
        if (std::abs(distance) > settings.truncation) continue;
        accumulate(value_at(*this, x, y), {distance, std::min(1.f, 4.f / (depth * depth)), measured.z(), sequence}, settings.maximum_weight);
      }
  }
}

std::vector<NavimapCell> NavimapGrid::surface_cells() const {
  auto raster = cells;
  // Weak surface support remains unknown; interpolating it must not expose free space.
  for (const auto& [k, tile] : distance_tiles) {
    const int bx = int32_t(k >> 32) * 8, by = int32_t(k) * 8;
    for (int i = 0; i < 64; ++i) {
      const auto& value = tile.values[i];
      if (value.weight <= 0 || value.distance >= cell_size) continue;
      const auto cell = raster.find(key(bx + i % 8, by + i / 8));
      if (cell != raster.end() && !(cell->second.flags == 2 && cell->second.sequence >= value.sequence)) raster.erase(cell);
    }
  }
  for (const auto& [k, tile] : distance_tiles) {
    const int bx = int32_t(k >> 32) * 8, by = int32_t(k) * 8;
    for (int i = 0; i < 64; ++i) {
      const auto& a = tile.values[i];
      if (a.weight < settings.surface_weight) continue;
      const int x = bx + i % 8, y = by + i / 8;
      const std::array<const Distance*, 4> values{&a, lookup(*this, x + 1, y), lookup(*this, x + 1, y + 1), lookup(*this, x, y + 1)};
      if (std::any_of(values.begin(), values.end(), [&](const auto* value) { return !value || value->weight < settings.surface_weight; })) continue;
      const std::array<Eigen::Vector2f, 4> points{Eigen::Vector2f(x + .5f, y + .5f), Eigen::Vector2f(x + 1.5f, y + .5f),
                                                  Eigen::Vector2f(x + 1.5f, y + 1.5f), Eigen::Vector2f(x + .5f, y + 1.5f)};
      std::array<Eigen::Vector2f, 4> crossings;
      std::array<float, 4> heights{};
      std::array<int, 4> edges{};
      int count = 0;
      const int sequence = std::max({values[0]->sequence, values[1]->sequence, values[2]->sequence, values[3]->sequence});
      for (int edge = 0; edge < 4; ++edge) {
        const int next = (edge + 1) % 4;
        if ((values[edge]->distance < 0) == (values[next]->distance < 0)) continue;
        const float alpha = values[edge]->distance / (values[edge]->distance - values[next]->distance);
        crossings[edge] = points[edge] + alpha * (points[next] - points[edge]);
        heights[edge] = values[edge]->height + alpha * (values[next]->height - values[edge]->height);
        edges[count++] = edge;
      }
      // Cover the supported segment itself, with no dilation or bridging across
      // missing corners. All coordinates below are in grid-cell units.
      const auto segment = [&](int first, int second) {
        const auto& begin = crossings[first];
        const auto& end = crossings[second];
        const Eigen::Vector2f delta = end - begin;
        for (int yy = std::floor(std::min(begin.y(), end.y())); yy <= std::floor(std::max(begin.y(), end.y())); ++yy)
          for (int xx = std::floor(std::min(begin.x(), end.x())); xx <= std::floor(std::max(begin.x(), end.x())); ++xx) {
            float lo = 0, hi = 1;
            for (int axis = 0; axis < 2; ++axis) {
              const float lower = axis == 0 ? xx : yy;
              if (std::abs(delta[axis]) < 1e-7f) {
                if (begin[axis] < lower || begin[axis] > lower + 1) hi = -1;
              } else {
                float u = (lower - begin[axis]) / delta[axis], v = (lower + 1 - begin[axis]) / delta[axis];
                if (u > v) std::swap(u, v);
                lo = std::max(lo, u);
                hi = std::min(hi, v);
              }
            }
            if (lo > hi) continue;
            NavimapCell cell;
            cell.x = xx;
            cell.y = yy;
            cell.flags = 1;
            cell.sequence = cell.height_sequence = sequence;
            cell.projection_z = cell.height = heights[first] + .5f * (lo + hi) * (heights[second] - heights[first]);
            cell.evidence = {0, settings.maximum, settings.maximum};
            auto [found, inserted] = raster.emplace(key(xx, yy), cell);
            if (!inserted) {
              const auto& previous = found->second;
              if (previous.flags == 2 && previous.sequence >= sequence) continue;
              if (previous.flags != 1 || sequence > previous.sequence || (sequence == previous.sequence && cell.height > previous.height))
                found->second = cell;
            }
          }
      };
      if (count == 2)
        segment(edges[0], edges[1]);
      else if (count == 4) {
        // Bilinear asymptotic decider resolves the only ambiguous square topology.
        if (values[0]->distance * values[2]->distance - values[1]->distance * values[3]->distance >= 0) {
          segment(0, 1);
          segment(2, 3);
        } else {
          segment(0, 3);
          segment(1, 2);
        }
      }
    }
  }
  std::vector<NavimapCell> result;
  result.reserve(raster.size());
  for (const auto& [k, cell] : raster) result.push_back(cell);
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.y != b.y ? a.y < b.y : a.x < b.x; });
  return result;
}

void NavimapGrid::merge_distance(const NavimapGrid& source, const StateGroupf& pose) {
  struct Projected {
    Distance value;
    float squared;
    uint64_t source_key;
  };
  std::unordered_map<uint64_t, Projected> projected;
  for (const auto& [k, tile] : source.distance_tiles) {
    const int bx = int32_t(k >> 32) * 8, by = int32_t(k) * 8;
    for (int i = 0; i < 64; ++i) {
      const auto& value = tile.values[i];
      if (value.weight <= 0) continue;
      const int sx = bx + i % 8, sy = by + i / 8;
      const auto p = pose.transformPoint({(sx + .5f) * source.cell_size, (sy + .5f) * source.cell_size, value.height});
      const int x = index(p.x(), cell_size), y = index(p.y(), cell_size);
      const float dx = p.x() - (x + .5f) * cell_size, dy = p.y() - (y + .5f) * cell_size, squared = dx * dx + dy * dy;
      const auto target_key = key(x, y), source_key = key(sx, sy);
      auto found = projected.find(target_key);
      // A rotation collision contributes once, with deterministic tie-breaking.
      if (found == projected.end() || squared < found->second.squared ||
          (squared == found->second.squared && source_key < found->second.source_key)) {
        auto copy = value;
        copy.height = p.z();
        projected.insert_or_assign(target_key, Projected{copy, squared, source_key});
      }
    }
  }
  for (const auto& [k, value] : projected) accumulate(value_at(*this, int32_t(k >> 32), int32_t(k)), value.value, settings.maximum_weight);
}
}  // namespace sapphire::backend
