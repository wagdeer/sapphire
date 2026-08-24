#pragma once

#include "common.hpp"
#include "parameters.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct NavigationGrid {
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int width = 0;
  int height = 0;
  std::vector<int8_t> data;
  size_t revision = 0;
};

using NavigationGridCallback = std::function<void(std::shared_ptr<const NavigationGrid>)>;

/// Asynchronous ROS-free pose-graph and loop-closure backend.
class PoseGraphBackend {
public:
  PoseGraphBackend(const PoseGraphParameters &config, const NaviMapParameters &navi_map, const std::string &database_path, NavigationGridCallback navigation_grid_callback);

  ~PoseGraphBackend();

  PoseGraphBackend(const PoseGraphBackend &) = delete;
  PoseGraphBackend &operator=(const PoseGraphBackend &) = delete;

  bool enabled() const;

  void addFrame(const std::shared_ptr<const vvec<float, 3>> &cloud_lidar, const Eigen::Isometry3d &T_odom_lidar, double stamp);

  Eigen::Isometry3d T_map_odom() const;

  void requestGlobalMap();

  std::shared_ptr<const vvec<float, 3>> latestGlobalMap() const;

  std::shared_ptr<const NavigationGrid> latestOccupancyGrid() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
