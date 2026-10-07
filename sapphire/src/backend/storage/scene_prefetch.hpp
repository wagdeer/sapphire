#pragma once
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <thread>

#include "backend/visual/feature/scene_features.hpp"

namespace sapphire::mapping::scene {
// Owns immutable snapshots only. The mapping owner supplies incarnation tokens;
// the worker never reads or mutates live Signatures, graph state, or vocabulary.
class PrefetchCache {
 public:
  using Map = std::shared_ptr<const FeatureMap>;
  using Key = std::pair<int, uint64_t>;
  using Loader = std::function<Map(Key)>;
  struct Stats {
    uint64_t loaded = 0, used = 0, unused = 0, discarded = 0, errors = 0;
    double load_ms = 0;
    size_t entries = 0, bytes = 0;
  };
  PrefetchCache(size_t capacity, size_t byte_capacity, Loader loader);
  ~PrefetchCache();
  PrefetchCache(const PrefetchCache&) = delete;
  PrefetchCache& operator=(const PrefetchCache&) = delete;
  void plan(const std::vector<Key>& ranked);
  Map get(Key key, bool* prefetched = nullptr);
  void remember(Key key, Map map);
  void invalidate(int id);
  Stats stats() const;

 private:
  struct Entry {
    uint64_t token = 0, used = 0;
    Map map;
    bool prefetched = false, consumed = false;
  };
  void put(Key key, Map map, bool prefetched);
  void run();
  size_t capacity_, byte_capacity_;
  Loader loader_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_ = false;
  uint64_t clock_ = 0;
  std::map<int, Entry> cache_;
  std::map<int, uint64_t> wanted_;
  std::deque<Key> queue_;
  Key loading_{0, 0};
  Stats stats_;
  std::thread worker_;
};
}  // namespace sapphire::mapping::scene
