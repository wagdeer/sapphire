#pragma once
#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/maplayer/navimap/occupancy_layer.hpp>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>

namespace sapphire::backend {
// One owner submits immutable sources; the worker owns every mutable raster.
// Pending changes coalesce by node, including removals. No per-frame work queue.
class NavimapWorker {
 public:
  struct Source { StateGroupf pose; std::shared_ptr<const LocalGrid> grid; };
  struct Map {
    uint64_t version = 0;
    int64_t stamp_ns = 0;
    cv::Mat state, probability;
    float x = 0, y = 0, cell_size = 0;
  };
  // Called after completion/failure, outside the worker lock. Notification only:
  // must not throw or wait for the producer; destruction joins the last callback.
  explicit NavimapWorker(const ParametersMap& parameters, std::function<void()> completed = {}, std::function<void(const std::string&)> failed = {});
  explicit NavimapWorker(const NavimapParams& parameters, std::function<void()> completed = {}, std::function<void(const std::string&)> failed = {});
  ~NavimapWorker();
  NavimapWorker(const NavimapWorker&) = delete;
  NavimapWorker& operator=(const NavimapWorker&) = delete;
  uint64_t submit(const std::vector<int>& removed, const std::map<int, Source>& changed, int64_t stamp_ns = 0);
  std::shared_ptr<const Map> latest() const;
  void wait(uint64_t version);
  void drain();
 private:
  void run();
  LocalGridCache cache_;
  OccupancyGrid grid_;
  mutable std::mutex mutex_;
  std::condition_variable changed_, completed_;
  std::map<int, Source> pending_; // null grid is a removal
  uint64_t submitted_ = 0;
  int64_t stamp_ns_ = 0;
  bool stopping_ = false;
  std::exception_ptr failure_;
  std::shared_ptr<const Map> latest_map_;
  std::ofstream trace_;
  const std::function<void()> notify_completed_;
  const std::function<void(const std::string&)> notify_failed_;
  std::thread worker_;
};
} // namespace sapphire::backend
