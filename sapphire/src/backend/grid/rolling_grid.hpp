#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>
#include "common/common.hpp"

namespace sapphire {
// Transient, gravity-aligned occupancy in one live odometry domain. No graph,
// archive or global-map dependency. Call only from its owning output worker.
class RollingGrid {
 public:
  struct Parameters {
    double extent = 20.0, resolution = .1, max_age = 5.0;
    double sensor_height = .8, obstacle_min_height = .15, obstacle_max_height = 2.0;
    double ground_tolerance = .15, min_range = .5;
    std::size_t ray_budget = 1024;
    bool raytrace = true;
  };
  explicit RollingGrid(Parameters parameters);
  bool update(const vvec<double, 3>& points, const Eigen::Vector3d& sensor_origin,
              double stamp, std::uint64_t generation);
  void reset();
  void raster(std::vector<std::int8_t>& cells) const;
  std::size_t width() const { return width_; }
  double resolution() const { return parameters_.resolution; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double stamp() const { return stamp_; }
  std::size_t residentBytes() const;

 private:
  struct Cell { double observed = -1; int score = 0; };
  void move(const Eigen::Vector3d& origin);
  bool inside(int x, int y) const;
  void ray(int x, int y, int end_x, int end_y, bool include_end);
  Parameters parameters_;
  std::size_t width_ = 0;
  std::vector<Cell> cells_, scratch_;
  std::vector<std::uint8_t> marks_;
  double origin_x_ = 0, origin_y_ = 0, stamp_ = -1;
  std::uint64_t generation_ = 0;
};
}  // namespace sapphire
