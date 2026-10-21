#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/maplayer/navimap/navimap_worker.hpp>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <stdexcept>

namespace sapphire::backend {
NavimapWorker::NavimapWorker(const ParametersMap& parameters, std::function<void()> completed, std::function<void(const std::string&)> failed)
    : NavimapWorker(NavimapParams{}.withOverrides(parameters), std::move(completed), std::move(failed)) {}
NavimapWorker::NavimapWorker(const NavimapParams& parameters, std::function<void()> completed, std::function<void(const std::string&)> failed)
    : grid_(&cache_, parameters), notify_completed_(std::move(completed)), notify_failed_(std::move(failed)) {
  if (const char* dir = std::getenv("PERF_RUN"); dir && *dir) {
    trace_.open(std::string(dir) + "/navimap_worker.csv", std::ios::app);
    if (trace_ && trace_.tellp() == 0)
      trace_ << "wall_ns,version,stamp_ns,apply_ms,raster_ms,changed,removed,maps\n";
    trace_ << std::setprecision(17);
  }
  worker_ = std::thread([this] { run(); });
}
NavimapWorker::~NavimapWorker() {
  { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
  changed_.notify_one();
  worker_.join(); // Finish the last accepted state before releasing its sources.
}
uint64_t NavimapWorker::submit(const std::vector<int>& removed, const std::map<int, Source>& changed, int64_t stamp_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
  if (stopping_) throw std::logic_error("Navigation worker stopped");
  if (removed.empty() && changed.empty()) return submitted_;
  for (const auto& [id, source] : changed)
    if (!source.grid || !source.pose.isValid()) throw std::invalid_argument("Invalid navigation map source id=" + std::to_string(id));
  for (int id : removed) pending_[id] = {};
  for (const auto& [id, source] : changed) pending_[id] = source;
  stamp_ns_ = stamp_ns;
  ++submitted_;
  changed_.notify_one();
  return submitted_;
}
std::shared_ptr<const NavimapWorker::Map> NavimapWorker::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_) std::rethrow_exception(failure_);
  return latest_map_;
}
void NavimapWorker::wait(uint64_t version) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (version > submitted_) throw std::invalid_argument("Unknown navigation version");
  completed_.wait(lock, [&] { return failure_ || version == 0 || (latest_map_ && latest_map_->version >= version); });
  if (failure_) std::rethrow_exception(failure_);
}
void NavimapWorker::drain() {
  uint64_t version;
  { std::lock_guard<std::mutex> lock(mutex_); version = submitted_; }
  wait(version);
}
void NavimapWorker::run() {
  using Clock = std::chrono::steady_clock;
  const char* stage = "prepare_sources";
  uint64_t version = 0;
  try {
    for (;;) {
      stage = "prepare_sources";
      std::map<int, Source> changes;
      auto result = std::make_shared<Map>();
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [&] { return stopping_ || !pending_.empty(); });
        if (pending_.empty() && stopping_) return;
        changes.swap(pending_);
        result->version = submitted_;
        version = submitted_;
        result->stamp_ns = stamp_ns_;
      }
      std::vector<int> removed;
      std::vector<std::pair<int, StateGroupf>> changed;
      for (const auto& [id, source] : changes) {
        if (source.grid) { cache_.add(id, source.grid); changed.emplace_back(id, source.pose); }
        else { cache_.erase(id); removed.push_back(id); }
      }
      const auto begin = Clock::now();
      stage = "assemble_raster";
      grid_.update_delta(removed, changed);
      const auto applied = Clock::now();
      stage = "build_raster";
      grid_.getMaps(result->state, result->probability, result->x, result->y);
      result->cell_size = grid_.getCellSize();
      const auto end = Clock::now();
      for (const auto& [id, pose] : changed) { grid_.release_source(id); cache_.erase(id); }
      if (trace_) {
        trace_ << std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count() << ',' << result->version
          << ',' << result->stamp_ns << ',' << std::chrono::duration<double, std::milli>(applied - begin).count()
          << ',' << std::chrono::duration<double, std::milli>(end - applied).count() << ',' << changed.size() << ',' << removed.size()
          << ',' << grid_.addedNodes().size() << '\n';
        trace_.flush();
      }
      { std::lock_guard<std::mutex> lock(mutex_); latest_map_ = std::move(result); }
      completed_.notify_all();
      if (notify_completed_) notify_completed_();
    }
  } catch (const std::exception& error) {
    const auto message = std::string(stage) + " version=" + std::to_string(version) + ": " + error.what();
    auto failure = std::make_exception_ptr(std::runtime_error(message));
    { std::lock_guard<std::mutex> lock(mutex_); failure_ = failure; pending_.clear(); }
    if (notify_failed_) notify_failed_(message);
    completed_.notify_all();
    if (notify_completed_) notify_completed_();
  }
}
} // namespace sapphire::backend
