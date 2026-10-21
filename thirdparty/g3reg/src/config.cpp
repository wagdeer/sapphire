/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "config.h"

#include <cmath>
#include <stdexcept>
namespace g3reg {
void Config::validate() const {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
  if (!std::isfinite(min_range) || min_range < 0 || !positive(max_range) || max_range <= min_range || !positive(ds_resolution) ||
      !positive(normal_radius) || !positive(fpfh_radius) || fpfh_radius < normal_radius || !positive(plane_resolution) ||
      !positive(max_clique_time_limit) || vertex_info.noise_bound_vec.empty())
    throw std::invalid_argument("Invalid G3Reg geometric/resource options");
  double previous = 0;
  for (double bound : vertex_info.noise_bound_vec) {
    if (!positive(bound) || bound <= previous) throw std::invalid_argument("G3Reg noise bounds must increase");
    previous = bound;
  }
}
}  // namespace g3reg
