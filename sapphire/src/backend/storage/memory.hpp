#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "backend/storage/map_database.hpp"
#include "backend/graph/loop_policy.hpp"
#include "backend/visual/faiss/descriptor_archive.hpp"
#include "backend/storage/retrieval_index.hpp"
#include "backend/storage/scene_prefetch.hpp"
#include "tools/lru_cache.hpp"

namespace sapphire {

class Memory final {
 public:
  explicit Memory(const std::string &database_path, RecallIndexAdapters adapters = {}, StorageParameters storage = {},
                  const std::string &config_identity = map_config_identity(PoseGraphParameters{}, NaviMapParameters{}),
                  const std::string &mode = "new", float grid_resolution = .1f)
      : database_(database_path, mode, config_identity, grid_resolution),
        cloud_cache_(std::size_t(storage.cloud_cache_mb) * 1024 * 1024),
        grid_cache_(std::size_t(storage.grid_cache_mb) * 1024 * 1024) {
    if (mode != "new") {
      database_.validateHistoricalRecords(grid_resolution);
      // Historical reads are synchronous. No prefetch/mapping worker is needed.
      return;
    }
    scene_cache_ = std::make_unique<mapping::scene::PrefetchCache>(
        storage.scene_cache_entries, std::size_t(storage.scene_cache_mb) * 1024 * 1024,
        [this](mapping::scene::PrefetchCache::Key key) { return readScene(key.first - 1, key.second); });
    descriptor_indexes_[static_cast<std::size_t>(MetaTagType::kFloat)] = std::move(adapters.float_index);
    descriptor_indexes_[static_cast<std::size_t>(MetaTagType::kBinary)] = std::move(adapters.binary_index);
    database_.visitSpatialRecords([this](std::uint64_t submap_id, const AABB &local_bounds, const Eigen::Isometry3f &map_T_submap) {
      spatial_index_.upsert(submap_id, local_bounds, map_T_submap);
      pose_index_.upsert(submap_id, map_T_submap);
    });
    for (auto &[submap_id, tag] : database_.loadMetaTags()) {
      RecallIndex *index = indexFor(tag.type());
      if (index != nullptr && index->available()) {
        index->upsert(submap_id, tag);
      }
    }
  }

  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;

  void promoteToWritable(float grid_resolution) {
    scene_cache_.reset();  // Join any existing prefetch before replacing its SQLite handle.
    database_.promoteToWritable(grid_resolution);
  }
  void finish() {
    scene_cache_.reset();
    database_.finish();
  }
  MapDatabase::State storageState() const { return database_.state(); }
  std::uint64_t committedRevision() const { return database_.committedRevision(); }
  bool sceneActive(std::uint64_t key) const { return database_.sceneActive(node_id(key)); }
  std::optional<std::uint64_t> refreshCoveredScene(int old_id, int new_id) {
    auto revision=database_.refreshCoveredScene(old_id,new_id);
    if(revision) {
      { std::lock_guard<std::mutex> lock(cloud_cache_mutex_); cloud_cache_.erase(old_id-1); }
      if(scene_cache_) scene_cache_->invalidate(old_id);
      std::lock_guard<std::mutex> lock(index_mutex_);
      spatial_index_.setActive(old_id-1, false);
      for(auto &index:descriptor_indexes_)if(index)index->erase(old_id-1);
    }
    return revision;
  }
  std::uint64_t commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                      const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                      const std::optional<GraphLink> &loop, const std::string &uuid, std::uint64_t revision, int next_id,
                                      const std::string &config, int chain_root) {
    return database_.commitFinalizedSubmap(submap, grid, scene, poses, base, loop, uuid, revision, next_id, config, chain_root);
  }

  void visitHistoricalNodes(const MapDatabase::NodeVisitor &visitor) const { database_.visitHistoricalNodes(visitor); }
  std::vector<GraphLink> historicalLinks() const { return database_.loadGraphLinks(); }
  const std::string &mapUuid() const { return database_.mapUuid(); }
  void validateReadIdentity() const { database_.validateReadIdentity(); }
  void rebuildCommittedIndexes() {
    // Called under backend lifecycle ownership, after A2 validation or W COMMIT.
    // Append-only graph identities, but searchable membership follows SceneState.
    // Upsert also refreshes every historical changed committed pose.
    std::lock_guard<std::mutex> lock(index_mutex_);
    database_.visitHistoricalNodes([this](int id, const auto &, const auto &committed, const auto &bounds) {
      pose_index_.upsert(id - 1, committed);
      spatial_index_.upsert(id - 1, bounds, committed, database_.sceneActive(id));
    });
    chain_roots_.assign(pose_index_.size(), 0);
    std::vector<bool> predecessor(chain_roots_.size(), false);
    for (const auto &link : database_.loadGraphLinks())
      if (link.type == 0) predecessor.at(link.to_id - 1) = true;
    for (std::size_t i = 1; i < chain_roots_.size(); ++i)
      chain_roots_[i] = predecessor[i] ? chain_roots_[i - 1] : i;
    // Topology has already passed authoritative A2/W validation.
    // Pose tags were validated against Node poses; they are never another source.
  }

  std::uint64_t chainRoot(std::uint64_t key) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    return chain_roots_.at(key);
  }
  bool eligibleLoop(std::uint64_t query, std::uint64_t target, std::uint64_t query_root) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    return target < chain_roots_.size() && spatial_index_.active(target) &&
           loop_policy::eligible(query, target, query_root, chain_roots_[target], chain_roots_.size());
  }
  // DB-free, post-COMMIT only, under backend lifecycle ownership. No vector KD rebuild.
  void materializeCommittedDelta(const SubmapFrame &query, const std::vector<std::pair<int, Eigen::Isometry3f>> &poses) {
    std::lock_guard<std::mutex> lock(index_mutex_);
    const auto q = query.id();
    if (q != pose_index_.size() || q != spatial_index_.size() || q != chain_roots_.size() || !q)
      throw std::logic_error("Unexpected committed append membership");
    bool new_pose = false;
    for (const auto &[sql, pose] : poses) {
      if (sql <= 0 || std::uint64_t(sql - 1) > q) throw std::logic_error("Invalid committed delta identity");
      const auto key = std::uint64_t(sql - 1);
      if (key == q) new_pose = true;
      else if (!pose_index_.contains(key) || !spatial_index_.contains(key)) throw std::logic_error("Missing committed delta membership");
    }
    if (!new_pose) throw std::logic_error("Missing new committed pose");
    for (const auto &[sql, pose] : poses) {
      const auto key = std::uint64_t(sql - 1);
      pose_index_.upsert(key, pose);
      if (key == q) spatial_index_.upsert(key, query.bounds(), pose);
      else spatial_index_.updatePoses({{key, pose}});
    }
    chain_roots_.push_back(chain_roots_.back());
  }

  void saveSubmap(const SubmapFrame &submap, const LocalGrid &grid, std::shared_ptr<const mapping::scene::FeatureMap> scene = {}) {
    const auto payload = scene ? scene->encode() : std::vector<std::uint8_t>{};
    {
      std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
      database_.saveSubmap(submap, grid, payload);
      cloud_cache_.put(submap.id(), submap.lio().pcd, cloudBytes(*submap.lio().pcd));
      if (database_.archivesNavigation()) grid_cache_.put(submap.id(), std::make_shared<const LocalGrid>(grid), gridBytes(grid));
    }
    scene_cache_->invalidate(node_id(submap.id()));
    if (scene) scene_cache_->remember({node_id(submap.id()), mapping::DescriptorArchive::fingerprint(*scene)}, scene);
    std::lock_guard<std::mutex> lock(index_mutex_);
    if (submap.id() != chain_roots_.size()) throw std::logic_error("Noncontiguous new-map root lookup");
    chain_roots_.push_back(0); // New-map producer has one live odometry chain.
    spatial_index_.upsert(submap.id(), submap.bounds(), submap.lio().T_odom_base.cast<float>());
    pose_index_.upsert(submap.id(), submap.lio().T_odom_base.cast<float>());
    for (std::unique_ptr<RecallIndex> &index : descriptor_indexes_) {
      if (index) {
        index->erase(submap.id());
      }
    }
    for (const std::optional<MetaTag> &tag : submap.tags()) {
      if (tag) {
        RecallIndex *index = indexFor(tag->type());
        if (index != nullptr && index->available()) {
          index->upsert(submap.id(), *tag);
        }
      }
    }
  }

  std::vector<RecallMatch> recall(std::uint64_t query_id, const Eigen::Isometry3f &query_pose, std::size_t top_k, float max_distance) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    std::vector<RecallMatch> descriptor_matches;
    bool used_descriptor_index = false;
    for (const std::unique_ptr<RecallIndex> &index : descriptor_indexes_) {
      if (!index || !index->available() || !index->contains(query_id)) {
        continue;
      }
      used_descriptor_index = true;
      std::vector<RecallMatch> matches = index->query(query_id, query_pose, top_k, std::numeric_limits<float>::infinity());
      descriptor_matches.insert(descriptor_matches.end(), matches.begin(), matches.end());
    }
    if (!used_descriptor_index) {
      const float radius_squared = max_distance * max_distance;
      return pose_index_.query(query_id, query_pose, top_k, radius_squared);
    }

    std::unordered_map<std::uint64_t, RecallMatch> unique;
    for (RecallMatch &match : descriptor_matches) {
      const Eigen::Isometry3f *pose = pose_index_.pose(match.submap_id);
      if (pose == nullptr) {
        continue;
      }
      match.pose = *pose;
      const auto existing = unique.find(match.submap_id);
      if (existing == unique.end() || match.squared_distance < existing->second.squared_distance) {
        unique[match.submap_id] = match;
      }
    }
    descriptor_matches.clear();
    descriptor_matches.reserve(unique.size());
    for (auto &[submap_id, match] : unique) {
      (void)submap_id;
      descriptor_matches.push_back(std::move(match));
    }
    std::sort(descriptor_matches.begin(), descriptor_matches.end(),
              [](const RecallMatch &left, const RecallMatch &right) { return left.squared_distance < right.squared_distance; });
    if (descriptor_matches.size() > top_k) {
      descriptor_matches.resize(top_k);
    }
    return descriptor_matches;
  }

  std::vector<SpatialMatch> recallSpatial(std::uint64_t query_id, const Eigen::Isometry3f &map_T_query) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    return spatial_index_.query(query_id, map_T_query);
  }

  std::vector<SpatialMatch> recallSpatial(const AABB &bounds, const Eigen::Isometry3f &pose) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    return spatial_index_.query(bounds, pose);
  }

  std::optional<Eigen::Isometry3f> submapPose(std::uint64_t id) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    const auto *pose = pose_index_.pose(id);
    return pose ? std::optional<Eigen::Isometry3f>(*pose) : std::nullopt;
  }

  GaussianCloudPtr loadCloud(std::uint64_t submap_id) {
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
    if(!sceneActive(submap_id)) return {};
    if (auto cloud = cloud_cache_.get(submap_id)) return cloud;
    GaussianCloudPtr cloud = database_.loadCloud(node_id(submap_id));
    if (cloud) cloud_cache_.put(submap_id, cloud, cloudBytes(*cloud));
    return cloud;
  }

  Eigen::Isometry3d loadOriginalAnchor(std::uint64_t id) const { return database_.loadOriginalAnchor(node_id(id)); }

  std::vector<VisualFrame> loadVisualFrames(std::uint64_t submap_id) { return database_.loadVisualFrames(node_id(submap_id)); }

  std::vector<std::uint8_t> loadVisualScene(std::uint64_t id) const { return database_.loadVisualScene(node_id(id)); }

  using Scene = std::shared_ptr<const mapping::scene::FeatureMap>;
  Scene loadScene(std::uint64_t id, std::uint64_t fingerprint = 0) {
    if(!sceneActive(id)) return {};
    if (fingerprint && scene_cache_)
      if (auto scene = scene_cache_->get({node_id(id), fingerprint})) return scene;
    auto scene = readScene(id, fingerprint);
    if (scene && scene_cache_) scene_cache_->remember({node_id(id), mapping::DescriptorArchive::fingerprint(*scene)}, scene);
    return scene;
  }
  void prefetchScenes(const std::vector<mapping::scene::PrefetchCache::Key> &ranked) {
    if (scene_cache_) scene_cache_->plan(ranked);
  }
  struct CacheStats {
    LruCache<GaussianCloud>::Stats clouds;
    LruCache<LocalGrid>::Stats grids;
    mapping::scene::PrefetchCache::Stats scenes;
  };
  CacheStats cacheStats() const {
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
    return {cloud_cache_.stats(), grid_cache_.stats(), scene_cache_ ? scene_cache_->stats() : mapping::scene::PrefetchCache::Stats{}};
  }

  NavigationPath loadNavigation(std::uint64_t submap_id) { return database_.loadNavigation(node_id(submap_id)); }

  cpu::VoxelMapsData loadPyramidVoxel(std::uint64_t submap_id) { return database_.loadPyramidVoxel(node_id(submap_id)); }

  bool loadLocalGrid(std::uint64_t submap_id, LocalGrid &grid) {
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
    if (auto cached = grid_cache_.get(submap_id)) {
      grid = *cached;
      return true;
    }
    if (!database_.loadLocalGrid(node_id(submap_id), grid)) return false;
    grid_cache_.put(submap_id, std::make_shared<const LocalGrid>(grid), gridBytes(grid));
    return true;
  }

  void saveSubmapPose(int id, const Eigen::Isometry3f &pose) { saveSubmapPoses({{id, pose}}); }

  void saveSubmapPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const std::vector<GraphLink> &links = {}) {
    database_.saveSubmapPoses(poses, links);
    std::lock_guard<std::mutex> lock(index_mutex_);
    std::vector<std::pair<std::uint64_t, Eigen::Isometry3f>> index_updates;
    index_updates.reserve(poses.size());
    for (const auto &[node_id, pose] : poses) {
      if (node_id > 0) {
        index_updates.emplace_back(static_cast<std::uint64_t>(node_id - 1), pose);
      }
    }
    pose_index_.upsert(index_updates);
    spatial_index_.updatePoses(index_updates);
  }

  std::uint64_t graphRevision() const { return database_.graphRevision(); }

  void saveLink(int from_id, int to_id, int type, const Eigen::Isometry3f &transform) { database_.saveLink(from_id, to_id, type, transform); }

 private:
  static int node_id(std::uint64_t submap_id) {
    if (submap_id >= static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("Submap ID exceeds map-database integer range");
    }
    return static_cast<int>(submap_id) + 1;
  }

  RecallIndex *indexFor(MetaTagType type) {
    if (type == MetaTagType::kPose) {
      return &pose_index_;
    }
    const std::size_t index = static_cast<std::size_t>(type);
    return index < descriptor_indexes_.size() ? descriptor_indexes_[index].get() : nullptr;
  }

  Scene readScene(std::uint64_t id, std::uint64_t expected) const {
    if(!sceneActive(id)) return {};
    const auto bytes = database_.loadVisualScene(node_id(id));
    if (bytes.empty()) return {};
    auto scene = mapping::scene::FeatureMap::decode(bytes.data(), bytes.size());
    if (expected && mapping::DescriptorArchive::fingerprint(*scene) != expected) {
      if (!database_.writable()) throw MapError(MapErrorCode::HistoricalData, "Historical scene fingerprint mismatch");
      return {};
    }
    return scene;
  }
  static std::size_t cloudBytes(const GaussianCloud &cloud) { return sizeof(cloud) + cloud.capacity() * sizeof(GaussianPoint); }
  static std::size_t gridBytes(const LocalGrid &grid) {
    return sizeof(grid) + (grid.groundCells.capacity() + grid.obstacleCells.capacity() + grid.emptyCells.capacity()) * sizeof(GridPoint);
  }
  MapDatabase database_;
  mutable std::mutex cloud_cache_mutex_;
  LruCache<GaussianCloud> cloud_cache_;
  LruCache<LocalGrid> grid_cache_;
  mutable std::mutex index_mutex_;
  std::array<std::unique_ptr<RecallIndex>, 2> descriptor_indexes_;
  std::vector<std::uint64_t> chain_roots_;
  PoseRecallIndex pose_index_;
  SubmapSpatialIndex spatial_index_;
  // Last member: join the prefetch worker before destroying database/cache state.
  std::unique_ptr<mapping::scene::PrefetchCache> scene_cache_;
};

}  // namespace sapphire
