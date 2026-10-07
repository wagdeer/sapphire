#include <unistd.h>

#include <cstdio>
#include <future>
#include <iostream>

#include "backend/storage/memory.hpp"
#include "backend/visual/visual_loop.hpp"
using namespace sapphire;
void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
SubmapFrame makeSubmap(std::uint64_t id) {
  auto cloud = std::make_shared<GaussianCloud>();
  for (int i = 0; i < 100; ++i) {
    GaussianPoint p;
    p.N = 20;
    p.mean = {i * .1f, 0, 0};
    p.covariance = Eigen::Matrix3f::Identity() * .001f;
    p.regularize();
    cloud->push_back(p);
  }
  cpu::VoxelMaps voxels;
  voxels.set_min_res(.25);
  voxels.create_voxelmaps(cloud->size(), [&](size_t i) { return (*cloud)[i].mean; });
  LioFrame lio;
  lio.pcd = cloud;
  lio.timestamp = id;
  NavigationPath navigation;
  navigation.samples.push_back({double(id), 0, 0, 0, 0, 0, 0, 1, 0});
  VisualFrame frame(id);
  cv::Mat descriptors(1, 32, CV_8UC1, cv::Scalar(id + 1));
  frame.update_features({VisualPoint(Eigen::Vector2f(20, 20), 0, 1)}, descriptors);
  std::vector<VisualFrame> frames;
  frames.push_back(std::move(frame));
  return SubmapFrame(id, std::move(lio), id, id, 1, 0, 0, voxels.release_data(), {{double(id), Eigen::Isometry3d::Identity()}}, std::move(navigation),
                     std::move(frames));
}
void sql(sqlite3 *db, const char *query) {
  char *error = nullptr;
  if (sqlite3_exec(db, query, nullptr, nullptr, &error) != SQLITE_OK) {
    std::string message = error;
    sqlite3_free(error);
    throw std::runtime_error(message);
  }
}
int count(sqlite3 *db, const char *query) {
  sqlite3_stmt *statement = nullptr;
  check(sqlite3_prepare_v2(db, query, -1, &statement, nullptr) == SQLITE_OK, "prepare SQL");
  check(sqlite3_step(statement) == SQLITE_ROW, "read SQL");
  int value = sqlite3_column_int(statement, 0);
  sqlite3_finalize(statement);
  return value;
}
int main() {
  const std::string path = "/tmp/sapphire-storage-" + std::to_string(getpid()) + ".db";
  Eigen::Isometry3f updated = Eigen::Isometry3f::Identity();
  updated.translation().x() = 8;
  {
    Memory memory(path);
    auto first = makeSubmap(0);
    auto scene = buildVisualScene(first);
    memory.saveSubmap(first, LocalGrid{}, scene);
    const auto identity = mapping::DescriptorArchive::fingerprint(*scene);
    auto retained = memory.loadCloud(0);
    check(retained && memory.loadCloud(0) == retained, "hot laser snapshot reused");
    check(memory.cacheStats().clouds.hits >= 2, "cache reports actual hits");
    memory.prefetchScenes({{1, identity}});
    check(memory.loadScene(0, identity) != nullptr, "prefetch returns exact committed appearance");
    check(!memory.loadScene(0, identity + 1), "wrong incarnation cannot be returned");
    sqlite3 *db = nullptr;
    check(sqlite3_open(path.c_str(), &db) == SQLITE_OK, "open verifier");
    sql(db, "CREATE TRIGGER reject_scene BEFORE INSERT ON VisualScene WHEN NEW.node_id=2 BEGIN SELECT RAISE(ABORT,'injected scene failure'); END;");
    auto second = makeSubmap(1);
    bool rejected = false;
    try {
      memory.saveSubmap(second, LocalGrid{}, buildVisualScene(second));
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected && count(db, "SELECT COUNT(*) FROM Node WHERE id=2") == 0, "appearance failure rolls back entire submap");
    check(!memory.submapPose(1), "failed commit is not published to spatial/pose indexes");
    sql(db, "DROP TRIGGER reject_scene;");
    memory.saveSubmap(second, LocalGrid{}, buildVisualScene(second));
    const auto revision = memory.graphRevision();
    rejected = false;
    try {
      memory.saveSubmapPoses({{1, updated}}, {{1, 999, 1, Eigen::Isometry3f::Identity()}});
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected && memory.graphRevision() == revision, "graph link failure rolls back revision");
    check(memory.submapPose(0)->translation().norm() == 0, "failed graph transaction leaves live pose unchanged");
    memory.saveSubmapPoses({{1, updated}}, {{1, 2, 1, Eigen::Isometry3f::Identity()}});
    check(memory.graphRevision() == revision + 1 && count(db, "SELECT COUNT(*) FROM Link") == 1, "poses and graph links committed together");
    // Canonical insertion is not a cache invalidation/replacement API.
    rejected = false;
    try {
      memory.saveSubmap(first, LocalGrid{});
    } catch (const MapError &error) {
      rejected = error.code() == MapErrorCode::DuplicateNode;
    }
    check(rejected && memory.graphRevision() == revision + 1, "duplicate node leaves revision unchanged");
    check(memory.loadScene(0, identity) != nullptr, "duplicate node preserves historical appearance");
    check(memory.submapPose(0)->matrix().isApprox(updated.matrix()), "duplicate node preserves optimized pose");
    sqlite3_close(db);
  }
  {
    MapDatabase restored(path, "resume");
    bool found = false;
    restored.visitSpatialRecords([&](std::uint64_t id, const AABB &, const Eigen::Isometry3f &pose) {
      if (id == 0) {
        found = true;
        check(pose.matrix().isApprox(updated.matrix()), "read-only reopen preserves committed pose");
      }
    });
    check(found && restored.loadCloud(1)->size() == 100, "cold records reload without backend restoration");
    auto second = makeSubmap(1);
    auto scene = buildVisualScene(second);
    const auto fingerprint = mapping::DescriptorArchive::fingerprint(*scene);
    // Exercise prefetch independently; A1 resume itself starts no cache/index workers.
    mapping::scene::PrefetchCache prefetch(2, 1024 * 1024, [&](auto key) {
      const auto bytes = restored.loadVisualScene(key.first);
      return mapping::scene::FeatureMap::decode(bytes.data(), bytes.size());
    });
    prefetch.plan({{2, fingerprint}});
    check(prefetch.get({2, fingerprint}) != nullptr, "cold appearance prefetch reloads after restart");
    check(prefetch.stats().loaded == 1 && prefetch.stats().used == 1, "cold prefetch is consumed without duplicate foreground read");
  }
  {
    auto first = makeSubmap(0), second = makeSubmap(1);
    auto old_scene = buildVisualScene(first), new_scene = buildVisualScene(second);
    const auto old_version = mapping::DescriptorArchive::fingerprint(*old_scene), new_version = mapping::DescriptorArchive::fingerprint(*new_scene);
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    mapping::scene::PrefetchCache cache(2, 1024 * 1024, [&](auto) {
      entered.set_value();
      released.wait();
      return old_scene;
    });
    cache.plan({{1, old_version}});
    entered.get_future().wait();
    cache.invalidate(1);
    cache.remember({1, new_version}, new_scene);
    release.set_value();
    check(!cache.get({1, old_version}), "retired in-flight prefetch is discarded");
    check(cache.get({1, new_version}) == new_scene, "late prefetch cannot replace current snapshot");
    check(cache.stats().discarded == 1, "stale prefetch is accounted");
  }
  LruCache<GaussianCloud> cache(2 * sizeof(GaussianCloud));
  auto a = std::make_shared<const GaussianCloud>();
  auto b = std::make_shared<const GaussianCloud>();
  auto c = std::make_shared<const GaussianCloud>();
  cache.put(0, a, sizeof(GaussianCloud));
  cache.put(1, b, sizeof(GaussianCloud));
  auto held = cache.get(0);
  cache.put(2, c, sizeof(GaussianCloud));
  check(!cache.get(1) && cache.get(0) == held && held == a, "LRU eviction preserves reader-owned immutable snapshot");
  check(cache.stats().bytes <= cache.stats().budget, "working set respects byte budget");
  std::remove(path.c_str());
  std::cout << "map storage tests passed\n";
}
