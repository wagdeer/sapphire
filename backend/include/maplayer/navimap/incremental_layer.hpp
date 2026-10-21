#pragma once
#include <backend/params/navimap.hpp>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <backend/tools/logging.hpp>
#include <backend/maplayer/navimap/local_grid.hpp>

namespace sapphire::backend {
template <typename Derived>
class IncrementalLayer {
 public:
  static float logOdds(float value) { return std::log(value / (1.0f - value)); }
  static float probability(float value) { return 1.0f - 1.0f / (1.0f + std::exp(value)); }

  bool update(const std::map<int, StateGroupf>& poses);
  bool update_delta(const std::vector<int>& removed, const std::vector<std::pair<int, StateGroupf>>& changed) {
    if (!static_cast<Derived&>(*this).apply(removed, changed)) return false;
    for (int id : removed) addedNodes_.erase(id);
    for (const auto& item : changed) addedNodes_.insert_or_assign(item.first, item.second);
    return true;
  }
  void clear();

  float getCellSize() const { return cellSize_; }
  const std::map<int, StateGroupf>& addedNodes() const { return addedNodes_; }

  void getGridMin(double& x, double& y) const {
    x = minValues_[0];
    y = minValues_[1];
  }
  void getGridMax(double& x, double& y) const {
    x = maxValues_[0];
    y = maxValues_[1];
  }
  void getGridMin(double& x, double& y, double& z) const {
    x = minValues_[0];
    y = minValues_[1];
    z = minValues_[2];
  }
  void getGridMax(double& x, double& y, double& z) const {
    x = maxValues_[0];
    y = maxValues_[1];
    z = maxValues_[2];
  }

  unsigned long getMemoryUsed() const;

 protected:
  IncrementalLayer(const LocalGridCache* cache, const NavimapParams& parameters);

  const std::map<int, std::shared_ptr<const LocalGrid>>& cache() const { return cache_->localGrids(); }

 protected:
  float cellSize_;

  float occupancyThr_;
  float logOddsHit_;
  float logOddsMiss_;
  float logOddsClampingMin_;
  float logOddsClampingMax_;

  double minValues_[3];
  double maxValues_[3];

 private:
  const LocalGridCache* cache_;
  std::map<int, StateGroupf> addedNodes_;
};

template <typename Derived>
IncrementalLayer<Derived>::IncrementalLayer(const LocalGridCache* cache, const NavimapParams& parameters)
    : cellSize_(parameters.cell_size_m),
      occupancyThr_(parameters.raster_occupied_probability),
      logOddsHit_(logOdds(parameters.raster_hit_probability)),
      logOddsMiss_(logOdds(parameters.raster_miss_probability)),
      logOddsClampingMin_(logOdds(parameters.raster_min_probability)),
      logOddsClampingMax_(logOdds(parameters.raster_max_probability)),
      minValues_{}, maxValues_{}, cache_(cache) {
  parameters.validate();
  EXT_ASSERT(cache_);
}

template <typename Derived>
void IncrementalLayer<Derived>::clear() {
  addedNodes_.clear();
  std::fill(std::begin(minValues_), std::end(minValues_), 0.0);
  std::fill(std::begin(maxValues_), std::end(maxValues_), 0.0);
}

template <typename Derived>
unsigned long IncrementalLayer<Derived>::getMemoryUsed() const {
  return addedNodes_.size() * (sizeof(int) + sizeof(StateGroupf) + sizeof(typename std::map<int, StateGroupf>::iterator)) +
         sizeof(std::map<int, StateGroupf>);
}

template <typename Derived>
bool IncrementalLayer<Derived>::update(const std::map<int, StateGroupf>& poses) {
  std::vector<int> removed;
  std::vector<std::pair<int, StateGroupf>> changed;
  changed.reserve(poses.size());
  for (const auto& [id, pose] : addedNodes_) {
    const auto current = poses.find(id);
    if (current == poses.end()) {
      removed.push_back(id);
    } else if (pose != current->second) {
      removed.push_back(id);
      changed.push_back(*current);
    }
  }
  for (auto pose = poses.lower_bound(1); pose != poses.end(); ++pose) {
    if (!addedNodes_.count(pose->first) && cache().count(pose->first)) changed.push_back(*pose);
  }
  removed.push_back(-1);
  const auto temporary = poses.find(0);
  if (temporary != poses.end() && cache().count(-1)) changed.emplace_back(-1, temporary->second);
  if (!static_cast<Derived&>(*this).apply(removed, changed)) return false;
  for (const int id : removed) {
    if (id > 0) addedNodes_.erase(id);
  }
  for (const auto& [id, pose] : changed) {
    if (id > 0) addedNodes_.insert_or_assign(id, pose);
  }
  return true;
}


}  // namespace sapphire::backend
