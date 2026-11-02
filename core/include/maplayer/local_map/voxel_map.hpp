#pragma once
#include <list>
#include <unordered_set>

#include "maplayer/local_map/voxel.hpp"

namespace sapphire {

class VoxelMap {
  friend struct VoxelMapTestAccess;
 private:
  // Root cache metadata belongs to this owner, not to recursive geometry.
  struct Root {
    unique_ptr<OctoTree> tree;
    std::list<VOXEL_LOC>::iterator fixed_position;
    size_t fixed_bytes = 0; // Zero means pinned / not on the fixed LRU.
    Root(unique_ptr<OctoTree> value) : tree(std::move(value)) {}
  };
  using TreeMap = std::unordered_map<VOXEL_LOC, Root>;
  using ActiveMap = std::unordered_map<VOXEL_LOC, OctoTree *>;

  OctoTree::Layout layout_;
  const double voxel_size_, voxel_size_inv_;
  TreeMap local_voxel_map_;
  ActiveMap active_voxels_;
  std::list<VOXEL_LOC> fixed_lru_; // newest first; never contains an active root
  size_t fixed_bytes_ = 0;
  const size_t fixed_budget_bytes_;
  size_t evicted_roots_ = 0;

  static size_t treeBytes(const OctoTree &tree) {
    size_t bytes = sizeof(tree) + tree.point_fix.capacity() * sizeof(pointVar);
    if (tree.sw) bytes += tree.sw->allocatedBytes();
    for (const auto &child : tree.leaves) if (child) bytes += treeBytes(*child);
    return bytes;
  }
  void pin(Root &root) {
    if (!root.fixed_bytes) return;
    fixed_bytes_ -= root.fixed_bytes;
    fixed_lru_.erase(root.fixed_position);
    root.fixed_bytes = 0;
  }
  void cacheFixed(const VOXEL_LOC &key) {
    auto &root = local_voxel_map_.at(key);
    pin(root);
    fixed_lru_.push_front(key);
    root.fixed_position = fixed_lru_.begin();
    root.fixed_bytes = sizeof(Root) + treeBytes(*root.tree);
    fixed_bytes_ += root.fixed_bytes;
  }
  vector<unique_ptr<OctoTree>> retired_trees_;
  vector<vector<SlideWindow *>> slide_window_pool_;
  int thread_num_;
  tools::ParallelExecutor<> &executor_; // Borrowed; owner must outlive this map. Dispatches never nest.
  // Only the odometry owner reads/writes this accounting, between worker joins.
  // Empty pooled windows cannot grow until borrowed by a mutating operation.
  std::size_t pooled_window_bytes_ = 0;
  bool pool_accounting_dirty_ = true;

 public:
  VoxelMap(const OdometryParameters &odometry, const LocalSubmapParameters &local_submap, tools::ParallelExecutor<> &executor)
      : layout_(local_submap), voxel_size_(odometry.voxel_size), voxel_size_inv_(odometry.voxel_size_inv), fixed_budget_bytes_(odometry.fixed_map_budget_bytes), slide_window_pool_(std::max(1, local_submap.thread_num)), thread_num_(std::max(1, local_submap.thread_num)), executor_(executor) {
    if (executor_.thread_count() != static_cast<std::size_t>(thread_num_))
      throw std::invalid_argument("VoxelMap executor must match configured parallelism");
  }

  double voxel_size() const { return voxel_size_; }

  ~VoxelMap() {
    clear();
    retired_trees_.clear();
    for (vector<SlideWindow *> &pool : slide_window_pool_) {
      for (SlideWindow *window : pool) {
        delete window;
      }
    }
  }

  // Owner-thread diagnostic after all per-frame workers have joined. Capacity
  // accounting excludes allocator metadata, hash nodes and other pipeline owners.
  struct MemoryUsage {
    std::size_t trees = 0, active_windows = 0, pooled_windows = 0;
    std::size_t tree_bytes = 0, window_bytes = 0, pooled_window_bytes = 0, point_capacity_bytes = 0, empty_slot_bytes = 0, leaf_point_bytes = 0;
  };
  MemoryUsage memoryUsage() const {
    MemoryUsage result;
    const auto window = [&](const SlideWindow &w, bool leaf) {
      result.window_bytes += sizeof(w) + w.points.capacity() * sizeof(PointCloud) + w.pcrs_local.capacity() * sizeof(PointCluster);
      for (const auto &points : w.points) {
        const auto bytes = points.capacity() * sizeof(pointVar);
        result.window_bytes += bytes;
        result.point_capacity_bytes += bytes;
        if (points.empty()) result.empty_slot_bytes += bytes;
        if (leaf) result.leaf_point_bytes += bytes;
      }
    };
    const auto visit = [&](const auto &self, const OctoTree &tree) -> void {
      ++result.trees;
      result.tree_bytes += sizeof(tree) + tree.point_fix.capacity() * sizeof(pointVar);
      if (tree.sw) { ++result.active_windows; window(*tree.sw, tree.layer >= layout_.max_layer); }
      for (const auto &child : tree.leaves) if (child) self(self, *child);
    };
    for (const auto &entry : local_voxel_map_) visit(visit, *entry.second.tree);
    for (const auto &tree : retired_trees_) visit(visit, *tree);
    for (const auto &pool : slide_window_pool_) for (const auto *w : pool) { ++result.pooled_windows; result.pooled_window_bytes += w->allocatedBytes(); window(*w, false); }
    return result;
  }

  VoxelMap(const VoxelMap &) = delete;
  VoxelMap &operator=(const VoxelMap &) = delete;

  void cut_voxel(const PointCloudPtr &pvec, int win_count, int wdsize, const vvec<double, 3> &pwld) {
    pool_accounting_dirty_ = true;
    int plsize = pvec->size();
    for (int i = 0; i < plsize; i++) {
      pointVar &pv = (*pvec)[i];
      const Eigen::Vector3d &pw = pwld[i];
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pw[j] * voxel_size_inv_;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = local_voxel_map_.find(position);
      if (iter != local_voxel_map_.end()) {
        pin(iter->second);
        OctoTree *tree = iter->second.tree.get();
        tree->allocate(win_count, pv, pw, slide_window_pool_[0]);
        tree->isexist = true;
        active_voxels_.try_emplace(position, tree);
      } else {
        auto owner = make_unique<OctoTree>(layout_, 0, wdsize, position);
        OctoTree *ot = owner.get();
        ot->voxel_center[0] = (0.5 + position.x) * voxel_size_;
        ot->voxel_center[1] = (0.5 + position.y) * voxel_size_;
        ot->voxel_center[2] = (0.5 + position.z) * voxel_size_;
        ot->quater_length = voxel_size_ / 4.0;
        ot->allocate(win_count, pv, pw, slide_window_pool_[0]);
        local_voxel_map_.emplace(position, std::move(owner));
        active_voxels_.emplace(position, ot);
      }
    }
  }

  void cut_voxel_multi(const PointCloudPtr &pvec, int win_count, int wdsize, const vvec<double, 3> &pwld) {
    pool_accounting_dirty_ = true;
    unordered_map<OctoTree *, vector<int>> map_pvec;
    int plsize = pvec->size();
    for (int i = 0; i < plsize; i++) {
      pointVar &pv = (*pvec)[i];
      const Eigen::Vector3d &pw = pwld[i];
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pw[j] * voxel_size_inv_;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = local_voxel_map_.find(position);
      OctoTree *ot = nullptr;
      if (iter != local_voxel_map_.end()) {
        pin(iter->second);
        ot = iter->second.tree.get();
        ot->isexist = true;
        active_voxels_.try_emplace(position, ot);
      } else {
        auto owner = make_unique<OctoTree>(layout_, 0, wdsize, position);
        ot = owner.get();
        ot->voxel_center[0] = (0.5 + position.x) * voxel_size_;
        ot->voxel_center[1] = (0.5 + position.y) * voxel_size_;
        ot->voxel_center[2] = (0.5 + position.z) * voxel_size_;
        ot->quater_length = voxel_size_ / 4.0;
        local_voxel_map_.emplace(position, std::move(owner));
        active_voxels_.emplace(position, ot);
      }

      map_pvec[ot].push_back(i);
    }

    vector<pair<OctoTree *const, vector<int>> *> octs;
    octs.reserve(map_pvec.size());
    for (auto iter = map_pvec.begin(); iter != map_pvec.end(); iter++) {
      octs.push_back(&(*iter));
    }

    int thd_num = thread_num_;
    int g_size = octs.size();
    if (g_size < thd_num) {
      for (auto &entry : map_pvec) {
        for (int index : entry.second) {
          entry.first->allocate(win_count, (*pvec)[index], pwld[index], slide_window_pool_[0]);
        }
      }
      return;
    }
    int swsize = slide_window_pool_[0].size() / thd_num;
    for (int i = 1; i < thd_num; i++) {
      slide_window_pool_[i].insert(slide_window_pool_[i].end(), slide_window_pool_[0].end() - swsize, slide_window_pool_[0].end());
      slide_window_pool_[0].erase(slide_window_pool_[0].end() - swsize, slide_window_pool_[0].end());
    }

    const int part = (g_size + thd_num - 1) / thd_num;

    auto allocate_range = [&](int head, int tail, vector<SlideWindow *> &pool) {
      for (int j = head; j < tail; ++j) {
        for (int index : octs[j]->second) {
          octs[j]->first->allocate(win_count, (*pvec)[index], pwld[index], pool);
        }
      }
    };

    // Dispatch worker IDs, preserving the original ceil partitions and pools.
    executor_.parallel_for(thd_num, 1, [&](size_t, size_t begin, size_t end) {
      for (size_t i = begin; i < end; ++i) {
        const int head = std::min(static_cast<int>(i) * part, g_size);
        allocate_range(head, std::min(head + part, g_size), slide_window_pool_[i]);
      }
    });
  }

  void cut_voxel(PointCloud &pvec, int wdsize, double jour) {
    std::unordered_set<VOXEL_LOC> touched;
    for (pointVar &pv : pvec) {
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pv.pnt[j] * voxel_size_inv_;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      touched.insert(position);
      auto iter = local_voxel_map_.find(position);
      if (iter != local_voxel_map_.end()) {
        pin(iter->second);
        iter->second.tree->allocate_fix(pv);
      } else {
        auto owner = make_unique<OctoTree>(layout_, 0, wdsize, position);
        OctoTree *ot = owner.get();
        ot->push_fix_novar(pv);
        ot->voxel_center[0] = (0.5 + position.x) * voxel_size_;
        ot->voxel_center[1] = (0.5 + position.y) * voxel_size_;
        ot->voxel_center[2] = (0.5 + position.z) * voxel_size_;
        ot->quater_length = voxel_size_ / 4.0;
        ot->jour = jour;
        local_voxel_map_.emplace(position, std::move(owner));
      }
    }
    for (const auto &key : touched) if (!active_voxels_.count(key)) cacheFixed(key);
    trimFixed();
  }

  const OctoTree *nearest_cluster(const Eigen::Vector3d &wld, double max_distance) const {
    if (!wld.allFinite() || !std::isfinite(max_distance) || max_distance <= 0 ||
        max_distance > voxel_size_ || wld.cwiseAbs().maxCoeff() > 1e6) return nullptr;
    // Same indexing convention as insertion, including exact negative boundaries.
    int64_t key[3];
    for (int i = 0; i < 3; ++i) {
      float v = wld[i] * voxel_size_inv_;
      if (v < 0) v -= 1;
      key[i] = static_cast<int64_t>(v);
    }
    double d = max_distance * max_distance;
    const OctoTree *best = nullptr;
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
      const auto found = local_voxel_map_.find(VOXEL_LOC(key[0]+x,key[1]+y,key[2]+z));
      if (found != local_voxel_map_.end()) found->second.tree->nearest_cluster(wld, d, best);
    }
    return best;
  }

  int match(Eigen::Vector3d &wld, Plane *&pla, const pointVar &point, const Eigen::Matrix3d &rotation,
            const Eigen::Matrix<double, POSE_DOF, POSE_DOF> &pose_covariance,
            Eigen::Matrix<double, POSE_DOF, 1> &jacobian, double &sigma_d, OctoTree *&oc) {
    int flag = 0;

    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = wld[j] * voxel_size_inv_;
      if (loc[j] < 0) {
        loc[j] -= 1;
      }
    }
    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = local_voxel_map_.find(position);
    if (iter != local_voxel_map_.end()) {
      double max_prob = 0;
      flag = iter->second.tree->match(wld, pla, max_prob, point, rotation, pose_covariance, jacobian, sigma_d, oc);
      if (flag && pla == nullptr) {
        spdlog::warn("pla null max_prob: {} {} {} {}", max_prob, iter->first.x, iter->first.y, iter->first.z);
      }
    }

    return flag;
  }

  void recut(int win_count, vector<StateGroup> &states, double eigen_threshold, const vector<double> &plane_thresholds) {
    pool_accounting_dirty_ = true;
    for (auto &entry : active_voxels_) {
      entry.second->recut(win_count, states, slide_window_pool_[0], eigen_threshold, plane_thresholds);
    }
  }

  void recut_multi(int win_count, vector<StateGroup> &states, double eigen_threshold, const vector<double> &plane_thresholds) {
    pool_accounting_dirty_ = true;
    vector<OctoTree *> trees;
    trees.reserve(active_voxels_.size());
    for (auto &entry : active_voxels_) {
      trees.push_back(entry.second);
    }

    const int tree_count = static_cast<int>(trees.size());
    const int worker_count = std::min(thread_num_, tree_count);
    if (worker_count <= 1) {
      recut(win_count, states, eigen_threshold, plane_thresholds);
      return;
    }

    const int part = (tree_count + worker_count - 1) / worker_count;

    auto recut_range = [&](int head, int tail, vector<SlideWindow *> &pool) {
      for (int i = head; i < tail; ++i) {
        trees[i]->recut(win_count, states, pool, eigen_threshold, plane_thresholds);
      }
    };

    executor_.parallel_for(worker_count, 1, [&](size_t, size_t begin, size_t end) {
      for (size_t i = begin; i < end; ++i) {
        const int head = std::min(static_cast<int>(i) * part, tree_count);
        recut_range(head, std::min(head + part, tree_count), slide_window_pool_[i]);
      }
    });

    for (int i = 1; i < worker_count; ++i) {
      slide_window_pool_[0].insert(slide_window_pool_[0].end(), slide_window_pool_[i].begin(), slide_window_pool_[i].end());
      slide_window_pool_[i].clear();
    }
  }

  // Assemble joint factors from current active geometry; trial evaluation does
  // not own or mutate the map used for accepted-state marginalization.
  void collect_factors(LidarFactor &factor) {
    for (auto &entry : active_voxels_) entry.second->tras_opt(factor);
  }

  GaussianCloud marginalize(double journey, int win_count, int margin_size, vector<StateGroup> &states,
                            bool emit_gaussians = true) {
    pool_accounting_dirty_ = true;
    vector<OctoTree *> trees;
    trees.reserve(active_voxels_.size());
    for (auto &entry : active_voxels_) {
      entry.second->jour = journey;
      trees.push_back(entry.second);
    }

    const int tree_count = static_cast<int>(trees.size());
    const int worker_count = std::min(thread_num_, tree_count);
    vector<GaussianCloud> worker_clouds(std::max(1, worker_count));
    if (worker_count <= 1) {
      vector<PointCluster> moments(win_count);
      for (OctoTree *tree : trees) {
        tree->margi(win_count, margin_size, states, worker_clouds[0], moments, emit_gaussians);
      }
    } else {
      const int part = (tree_count + worker_count - 1) / worker_count;
      auto margi_range = [&](int head, int tail, GaussianCloud &cloud) {
        vector<PointCluster> moments(win_count);
        for (int i = head; i < tail; ++i) {
          trees[i]->margi(win_count, margin_size, states, cloud, moments, emit_gaussians);
        }
      };

      executor_.parallel_for(worker_count, 1, [&](size_t, size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
          const int head = std::min(static_cast<int>(i) * part, tree_count);
          margi_range(head, std::min(head + part, tree_count), worker_clouds[i]);
        }
      });
    }

    for (auto iter = active_voxels_.begin(); iter != active_voxels_.end();) {
      if (iter->second->isexist) {
        ++iter;
      } else {
        iter->second->clear_slwd(slide_window_pool_[0]);
        cacheFixed(iter->first);
        iter = active_voxels_.erase(iter);
      }
    }
    layout_.advance_window_order(margin_size);

    size_t gaussian_count = 0;
    for (const GaussianCloud &cloud : worker_clouds) {
      gaussian_count += cloud.size();
    }
    GaussianCloud marginalized;
    marginalized.reserve(gaussian_count);
    for (GaussianCloud &cloud : worker_clouds) {
      marginalized.insert(marginalized.end(), std::make_move_iterator(cloud.begin()), std::make_move_iterator(cloud.end()));
    }
    return marginalized;
  }

  void clear() {
    pool_accounting_dirty_ = true;
    active_voxels_.clear();
    for (auto &entry : local_voxel_map_) {
      entry.second.tree->clear_slwd(slide_window_pool_[0]);
    }
    fixed_lru_.clear();
    fixed_bytes_ = 0;
    local_voxel_map_.clear();
    layout_.reset_window_order();
  }

  void reset() {
    pool_accounting_dirty_ = true;
    active_voxels_.clear();
    retired_trees_.reserve(retired_trees_.size() + local_voxel_map_.size());
    for (auto &entry : local_voxel_map_) {
      entry.second.tree->clear_slwd(slide_window_pool_[0]);
      retired_trees_.push_back(std::move(entry.second.tree));
    }
    fixed_lru_.clear();
    fixed_bytes_ = 0;
    local_voxel_map_.clear();
    layout_.reset_window_order();
  }

  // Called only after marginalization/handoff, with numerical workers joined.
  // Destruction is immediate: deferred garbage is not outside the budget.
  size_t trimFixed() {
    size_t released = 0;
    while (fixed_bytes_ > fixed_budget_bytes_ && !fixed_lru_.empty()) {
      auto victim = local_voxel_map_.find(fixed_lru_.back());
      if (active_voxels_.count(victim->first))
        throw std::logic_error("Active optimization root entered fixed-map LRU");
      pin(victim->second);
      local_voxel_map_.erase(victim);
      ++released;
    }
    evicted_roots_ += released;
    return released;
  }
  size_t fixedBytes() const { return fixed_bytes_; }
  size_t fixedSize() const { return fixed_lru_.size(); }
  size_t evictedRoots() const { return evicted_roots_; }

  size_t release_retired(size_t max_count) {
    const size_t count = std::min(max_count, retired_trees_.size());
    for (size_t i = 0; i < count; ++i) {
      retired_trees_.pop_back();
    }
    return count;
  }

  // Only empty reusable windows are evicted. Active windows/observations are
  // untouched. One byte budget covers all worker pools, not one budget per worker.
  static constexpr std::size_t window_pool_byte_budget = 64U * 1024 * 1024;
  size_t trim_window_pool(size_t max_retained, size_t max_release) {
    if (pool_accounting_dirty_) {
      pooled_window_bytes_ = 0;
      for (const auto &pool : slide_window_pool_)
        for (const auto *window : pool) pooled_window_bytes_ += window->allocatedBytes();
      pool_accounting_dirty_ = false;
    }
    size_t released = 0;
    for (vector<SlideWindow *> &pool : slide_window_pool_) {
      while (!pool.empty() && released < max_release && (pool.size() > max_retained || pooled_window_bytes_ > window_pool_byte_budget)) {
        pooled_window_bytes_ -= pool.back()->allocatedBytes();
        delete pool.back();
        pool.pop_back();
        ++released;
      }
      if (released == max_release) break;
    }
    return released;
  }

  size_t size() const { return local_voxel_map_.size(); }
  size_t active_size() const { return active_voxels_.size(); }
};

}  // namespace sapphire
