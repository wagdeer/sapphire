#include "backend/grid/rolling_grid.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sapphire {
RollingGrid::RollingGrid(Parameters p) : parameters_(p) {
  for (double value : {p.extent, p.resolution, p.max_age, p.sensor_height, p.obstacle_min_height,
                       p.obstacle_max_height, p.ground_tolerance, p.min_range})
    if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite local grid parameter");
  if (p.extent <= 0 || p.resolution < .01 || p.max_age <= 0 || p.sensor_height <= 0 ||
      p.obstacle_min_height < 0 || p.obstacle_max_height <= p.obstacle_min_height ||
      p.ground_tolerance < 0 || p.min_range < 0 || !p.ray_budget || p.ray_budget > 8192)
    throw std::invalid_argument("Invalid local grid parameters");
  const double dimension = std::ceil(p.extent / p.resolution);
  if (dimension < 4 || dimension > 512) throw std::length_error("Local grid must have 4..512 cells per side");
  width_ = static_cast<std::size_t>(dimension);
  cells_.resize(width_ * width_); scratch_.resize(cells_.size()); marks_.resize(cells_.size());
}
void RollingGrid::reset() {
  std::fill(cells_.begin(), cells_.end(), Cell{});
  std::fill(scratch_.begin(), scratch_.end(), Cell{});
  std::fill(marks_.begin(), marks_.end(), 0);
  stamp_ = -1; generation_ = 0;
}
std::size_t RollingGrid::residentBytes() const {
  return (cells_.capacity() + scratch_.capacity()) * sizeof(Cell) + marks_.capacity();
}
bool RollingGrid::inside(int x, int y) const { return x >= 0 && y >= 0 && x < int(width_) && y < int(width_); }
void RollingGrid::move(const Eigen::Vector3d& origin) {
  const double r = parameters_.resolution;
  const double x = (std::floor(origin.x() / r) - double(width_ / 2)) * r;
  const double y = (std::floor(origin.y() / r) - double(width_ / 2)) * r;
  if (stamp_ >= 0 && (x != origin_x_ || y != origin_y_)) {
    std::fill(scratch_.begin(), scratch_.end(), Cell{});
    const double dx = std::round((x - origin_x_) / r), dy = std::round((y - origin_y_) / r);
    if (std::abs(dx) < width_ && std::abs(dy) < width_) {
      const int ix = int(dx), iy = int(dy);
      for (int row = 0; row < int(width_); ++row)
        for (int col = 0; col < int(width_); ++col)
          if (inside(col + ix, row + iy)) scratch_[row * width_ + col] = cells_[(row + iy) * width_ + col + ix];
    }
    cells_.swap(scratch_);
  }
  origin_x_ = x; origin_y_ = y;
}
void RollingGrid::ray(int x, int y, int end_x, int end_y, bool include_end) {
  const int dx = std::abs(end_x - x), dy = -std::abs(end_y - y);
  const int sx = x < end_x ? 1 : -1, sy = y < end_y ? 1 : -1;
  int error = dx + dy;
  for (;;) {
    const bool end = x == end_x && y == end_y;
    if (inside(x, y) && (!end || include_end)) marks_[y * width_ + x] |= 1;
    if (end) break;
    const int twice = 2 * error;
    if (twice >= dy) { error += dy; x += sx; }
    if (twice <= dx) { error += dx; y += sy; }
  }
}
bool RollingGrid::update(const vvec<double, 3>& points, const Eigen::Vector3d& origin,
                         double stamp, std::uint64_t generation) {
  if (!origin.allFinite() || origin.cwiseAbs().maxCoeff() > 1e9 || !std::isfinite(stamp) || stamp < 0 || stamp > 9e9)
    throw std::invalid_argument("Invalid local grid observation");
  if (stamp_ >= 0 && generation == generation_ && stamp <= stamp_) return false;
  if (generation != generation_) reset();
  move(origin); generation_ = generation;
  std::fill(marks_.begin(), marks_.end(), 0);
  const double r = parameters_.resolution, side = width_ * r;
  const double floor = origin.z() - parameters_.sensor_height;
  const int sx = int(std::floor((origin.x() - origin_x_) / r));
  const int sy = int(std::floor((origin.y() - origin_y_) / r));
  const std::size_t stride = std::max(std::size_t(1), points.size() / parameters_.ray_budget + (points.size() % parameters_.ray_budget != 0));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (!p.allFinite()) continue;
    const double height = p.z() - floor;
    if (height < -parameters_.ground_tolerance || height > parameters_.obstacle_max_height) continue;
    Eigen::Vector2d delta = p.head<2>() - origin.head<2>();
    if (!delta.allFinite() || delta.norm() < parameters_.min_range) continue;
    const bool obstacle = height >= parameters_.obstacle_min_height;
    double ex = (p.x() - origin_x_) / r, ey = (p.y() - origin_y_) / r;
    const bool endpoint_inside = ex >= 0 && ey >= 0 && ex < width_ && ey < width_;
    if (endpoint_inside && obstacle) marks_[std::size_t(ey) * width_ + std::size_t(ex)] |= 2;
    if (!parameters_.raytrace || i % stride) continue;
    if (!endpoint_inside) {
      double fraction = 1;
      for (int axis = 0; axis < 2; ++axis) {
        const double lower = axis == 0 ? origin_x_ : origin_y_;
        if (delta[axis] > 0) fraction = std::min(fraction, (lower + side - r * 1e-6 - origin[axis]) / delta[axis]);
        else if (delta[axis] < 0) fraction = std::min(fraction, (lower + r * 1e-6 - origin[axis]) / delta[axis]);
      }
      const auto clipped = origin.head<2>() + fraction * delta;
      ex = (clipped.x() - origin_x_) / r; ey = (clipped.y() - origin_y_) / r;
    }
    const int tx = std::clamp(int(std::floor(ex)), 0, int(width_) - 1);
    const int ty = std::clamp(int(std::floor(ey)), 0, int(width_) - 1);
    ray(sx, sy, tx, ty, !obstacle || !endpoint_inside);
  }
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    auto& cell = cells_[i];
    if (cell.observed >= 0 && stamp - cell.observed > parameters_.max_age) cell = {};
    // A newly observed obstacle must not remain free because of old free rays.
    if (marks_[i] & 2) { cell.score = std::clamp(cell.score + 3, 3, 6); cell.observed = stamp; }
    else if (marks_[i] & 1) { cell.score = std::max(-6, cell.score - 3); cell.observed = stamp; }
  }
  stamp_ = stamp; return true;
}
void RollingGrid::raster(std::vector<std::int8_t>& cells) const {
  cells.resize(cells_.size());
  for (std::size_t i = 0; i < cells.size(); ++i) cells[i] = cells_[i].observed < 0 ? -1 : cells_[i].score > 0 ? 100 : 0;
}
}  // namespace sapphire
