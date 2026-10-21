#pragma once
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>

namespace sapphire {
// Owner supplies synchronization. Readers keep immutable snapshots alive after eviction.
template <class T>
class LruCache {
 public:
  using Pointer = std::shared_ptr<const T>;
  struct Stats {
    std::size_t entries = 0, bytes = 0, budget = 0;
    std::uint64_t hits = 0, misses = 0, evictions = 0;
  };
  explicit LruCache(std::size_t budget) : budget_(budget) {}
  Pointer get(std::uint64_t id) {
    auto it = entries_.find(id);
    if (it == entries_.end()) {
      ++misses_;
      return {};
    }
    ++hits_;
    order_.splice(order_.begin(), order_, it->second.position);
    return it->second.value;
  }
  void erase(std::uint64_t id) {
    auto it = entries_.find(id);
    if (it == entries_.end()) return;
    bytes_ -= it->second.bytes;
    order_.erase(it->second.position);
    entries_.erase(it);
  }
  void put(std::uint64_t id, Pointer value, std::size_t bytes) {
    erase(id);
    if (!value || bytes > budget_ || !budget_) return;
    while (bytes_ > budget_ - bytes && !order_.empty()) {
      erase(order_.back());
      ++evictions_;
    }
    order_.push_front(id);
    try {
      entries_.emplace(id, Entry{std::move(value), bytes, order_.begin()});
    } catch (...) {
      // A failed cache fill may evict disposable entries, but must not leave an
      // orphan list ID that makes a later eviction loop unable to advance.
      order_.pop_front();
      throw;
    }
    bytes_ += bytes;
  }
  Stats stats() const { return {entries_.size(), bytes_, budget_, hits_, misses_, evictions_}; }

 private:
  struct Entry {
    Pointer value;
    std::size_t bytes;
    typename std::list<std::uint64_t>::iterator position;
  };
  std::size_t budget_, bytes_ = 0;
  std::uint64_t hits_ = 0, misses_ = 0, evictions_ = 0;
  std::list<std::uint64_t> order_;
  std::unordered_map<std::uint64_t, Entry> entries_;
};
}  // namespace sapphire
