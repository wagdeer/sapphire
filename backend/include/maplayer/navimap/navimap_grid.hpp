#pragma once
#include <array>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <backend/common/state_group.hpp>
#include <limits>
#include <unordered_map>
#include <vector>

namespace sapphire::backend {
struct LocalGrid;

// A sequence of bounded log-odds updates is a single clamp(x + shift, lower, upper).
// Composition preserves repeated observations and saturation, without their history.
struct OccupancyEvidence {
  float shift = 0;
  float lower = -std::numeric_limits<float>::infinity();
  float upper = std::numeric_limits<float>::infinity();
  float apply(float prior) const { return std::clamp(prior + shift, lower, upper); }
  void then(const OccupancyEvidence& next) {
    lower = next.apply(lower);
    upper = next.apply(upper);
    shift = lower == upper ? 0 : shift + next.shift;
  }
};

struct NavimapCell {
  int32_t x = 0, y = 0;
  OccupancyEvidence evidence;
  float projection_z = 0;  // Representative ray/surface height for pose reprojection.
  float height = std::numeric_limits<float>::quiet_NaN();
  int sequence = 0, height_sequence = 0;
  uint8_t flags = 0;  // 1: last observation occupied; 2: last observation footprint
};

struct NavimapSettings {
  float hit = std::log(.7f / .3f), miss = std::log(.4f / .6f);
  float minimum = std::log(.1192f / .8808f), maximum = std::log(.971f / .029f);
  float footprint = 0;
  float truncation = .15f, surface_weight = 1.f, maximum_weight = 10.f;
};

// Sparse evidence in one horizontal grid frame. No images or observation history.
struct NavimapGrid {
  struct Distance { float distance = 0, weight = 0, height = 0; int sequence = 0; };
  struct Tile { std::array<Distance, 64> values{}; };
  // Horizontal integration plane expressed in the node anchor frame.
  StateGroupf anchor_from_grid = StateGroupf::identity();
  std::unordered_map<uint64_t, Tile> distance_tiles;
  static int tile_index(int x) { return x >= 0 ? x / 8 : (x - 7) / 8; }

  float cell_size = 0;
  NavimapSettings settings;
  std::unordered_map<uint64_t, NavimapCell> cells;
  void integrate(const LocalGrid& frame, const StateGroupf& grid_from_body, int sequence);
  void merge(const NavimapGrid& source, const StateGroupf& target_from_source);
  // Derived classified raster, including the supported TSDF zero contour.
  std::vector<NavimapCell> ordered_cells() const;
  static uint64_t key(int32_t x, int32_t y) { return (uint64_t(uint32_t(x)) << 32) | uint32_t(y); }

 private:
  std::vector<NavimapCell> surface_cells() const;
  void integrate_depth(const LocalGrid&, const StateGroupf& grid_from_body, int sequence);
  void merge_distance(const NavimapGrid&, const StateGroupf& grid_from_source);
};
}  // namespace sapphire::backend
