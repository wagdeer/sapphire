#pragma once
#include <spdlog/spdlog.h>

#include <functional>
#include <memory>
#include <utility>

#include "frontend/common/point.hpp"
namespace sapphire {
struct OutputSink {
  std::function<void()> ready_changed;
  std::function<void(bool)> owner_reset;    // Normal thread acknowledgement; never under backend/owner locks.
  std::function<void(bool)> optional_drop;  // true = distinct local-map increment lost.
  std::function<void(const StateGroup &)> odom_state;
  // Synchronous odometry-thread diagnostic, after LiDAR correction and before BA.
  std::function<void(const StateGroup &, bool, double)> lidar_update;
  // Completion of one synchronized scan, including initialization/rejection/BA.
  std::function<void(double, double)> scan_processed;
  // Current scan points in the odometry frame.
  std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_scan;
  std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_map;
  std::function<void(const Eigen::Isometry3d &)> map_odom;
  std::function<void(const Eigen::Isometry3d &, double)> map_pose;
};

namespace detail {
template <typename Callback, typename... Args>
void invoke_output(const Callback &callback, Args &&...args) {
  if (!callback) {
    return;
  }
  try {
    callback(std::forward<Args>(args)...);
  } catch (const std::exception &error) {
    spdlog::error("Output callback failed: {}", error.what());
  } catch (...) {
    spdlog::error("Output callback failed with an unknown exception");
  }
}

}  // namespace detail
}  // namespace sapphire
