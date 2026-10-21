#include "backend/database/feature_prefetch.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace sapphire::backend::featmap {
PrefetchCache::PrefetchCache(size_t capacity, Loader loader):capacity_(capacity),loader_(std::move(loader)) {
  if (!capacity_ || !loader_) throw std::invalid_argument("Invalid submap prefetch configuration");
}
PrefetchCache::~PrefetchCache() {
  { std::lock_guard<std::mutex> lock(mutex_); stop_=true; queue_.clear(); }
  wake_.notify_one();
  if (worker_.joinable()) worker_.join();
}
void PrefetchCache::plan(const std::vector<Key>& ranked) {
  std::lock_guard<std::mutex> lock(mutex_);
  wanted_.clear(); queue_.clear();
  for (auto key:ranked) {
    if (wanted_.size()>=capacity_) break;
    if (key.first<=0 || !key.second || !wanted_.emplace(key).second) continue;
    const auto it=cache_.find(key.first);
    if (it!=cache_.end() && it->second.token==key.second) continue;
    if (key!=loading_) queue_.push_back(key);
  }
  // Demand reads reuse this cache without starting a background loader.
  if (!queue_.empty() && !worker_.joinable()) worker_ = std::thread([this] { run(); });
  wake_.notify_one();
}
PrefetchCache::Map PrefetchCache::get(Key key, bool* prefetched) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it=cache_.find(key.first);
  if (it==cache_.end()) return {};
  if (it->second.token!=key.second) return {};
  if (prefetched) *prefetched=it->second.prefetched;
  if (it->second.prefetched && !it->second.consumed) ++stats_.used;
  it->second.consumed=true;
  it->second.used=++clock_; return it->second.map;
}
void PrefetchCache::put(Key key, Map map, bool prefetched) {
  if (!map) return;
  if (!cache_.count(key.first) && cache_.size()>=capacity_) {
    const auto oldest=std::min_element(cache_.begin(),cache_.end(),[](const auto& a,const auto& b){return a.second.used<b.second.used;});
    if (oldest->second.prefetched && !oldest->second.consumed) ++stats_.unused;
    cache_.erase(oldest);
  }
  auto existing=cache_.find(key.first);
  if (existing!=cache_.end() && existing->second.prefetched && !existing->second.consumed) ++stats_.unused;
  cache_.insert_or_assign(key.first,Entry{key.second,++clock_,std::move(map),prefetched,false});
}
void PrefetchCache::remember(Key key, Map map) {
  std::lock_guard<std::mutex> lock(mutex_); put(key,std::move(map),false);
}
PrefetchCache::Stats PrefetchCache::stats() const {
  std::lock_guard<std::mutex> lock(mutex_); auto result=stats_; result.entries=cache_.size();
  for (const auto& item:cache_) result.bytes+=item.second.map->memory_size();
  return result;
}
void PrefetchCache::run() {
  for (;;) {
    Key key;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock,[this]{return stop_ || !queue_.empty();});
      if (stop_) return;
      key=queue_.front(); queue_.pop_front(); loading_=key;
      const auto it=cache_.find(key.first);
      if (it!=cache_.end() && it->second.token==key.second) { loading_={0,0}; continue; }
    }
    const auto started=std::chrono::steady_clock::now();
    Map map; bool failed=false;
    try { map=loader_(key.first); } catch (...) { failed=true; }
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    {
      std::lock_guard<std::mutex> lock(mutex_); loading_={0,0};
      stats_.load_ms+=ms;
      const auto wanted=wanted_.find(key.first);
      if (failed || !map) ++stats_.errors;
      else if (stop_ || wanted==wanted_.end() || wanted->second!=key.second) ++stats_.discarded;
      else { put(key,std::move(map),true); ++stats_.loaded; }
    }
  }
}
}
