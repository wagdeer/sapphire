#pragma once

#include <backend/common/common_type.hpp>
#include <array>
#include <string>

namespace sapphire::backend {

struct NavimapParams {
  float cell_size_m = 0.05;
  unsigned int depth_decimation = 4;
  std::string depth_roi_ratios = "0.0 0.0 0.0 0.0";
  float footprint_height_m = 0.0;
  float footprint_length_m = 0.0;
  float footprint_width_m = 0.0;
  bool frame_grid_enabled = false;
  float ground_max_height_correction_m = 0.1;
  int ground_method = 0;
  float ground_plane_tolerance_m = 0.03;
  float max_depth_m = 3.0;
  float max_obstacle_height_m = 0.55;
  int method = 1;
  float min_depth_m = 0.0;
  float min_obstacle_height_m = 0.15;
  bool project_in_map_frame = false;
  unsigned int raster_cache_budget_bytes = 67108864;
  bool raster_erode_obstacles = true;
  float tsdf_truncation_cells = 3.f;
  float tsdf_surface_weight = 1.f;
  float tsdf_maximum_weight = 10.f;
  float raster_footprint_radius_m = 0.0;
  float raster_hit_probability = 0.7;
  float raster_max_probability = 0.971;
  float raster_min_probability = 0.1192;
  float raster_miss_probability = 0.4;
  float raster_occupied_probability = 0.5;
  bool ray_tracing_enabled = false;
  bool voxel_filter_enabled = true;

  NavimapParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
  std::array<float, 4> roiRatios() const;
};

}  // namespace sapphire::backend
