#include "backend/visual/visual_loop.hpp"
#include "common/camera/camera.hpp"
#include "backend/visual/solver/visual_pnp.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <limits>
#include "backend/visual/faiss/descriptor_archive.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <set>

#include "backend/storage/memory.hpp"
#include "tools/timer.hpp"

namespace sapphire {


std::shared_ptr<const mapping::scene::FeatureMap> buildVisualScene(const SubmapFrame &submap) {
  return buildVisualScene(submap.visual_frames(), submap.lio().T_odom_base);
}

std::shared_ptr<const mapping::scene::FeatureMap> buildVisualScene(const std::vector<VisualFrame> &frames,
    const Eigen::Isometry3d &T_odom_submap) {
  if (!T_odom_submap.matrix().allFinite() ||
      (T_odom_submap.linear().transpose() * T_odom_submap.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(T_odom_submap.linear().determinant() - 1) > 1e-6 ||
      (T_odom_submap.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9)
    throw std::invalid_argument("Invalid visual scene anchor");
  std::vector<mapping::scene::Landmark> landmarks;
  std::set<std::array<std::uint8_t, 32>> unique;
  for (const auto &frame : frames) {
    if (!frame.has_features() || frame.descriptors().type() != CV_8UC1 || frame.descriptors().cols != 32) continue;
    for (int row = 0; row < frame.descriptors().rows && landmarks.size() < mapping::scene::point_capacity; ++row) {
      mapping::scene::Landmark landmark;
      landmark.id = landmarks.size() + 1;
      landmark.appearance_count = 1;
      landmark.position = {0, 0, 0};
      landmark.has_position = false;
      const auto &observation = frame.points().at(row);
      if (frame.T_odom_camera() && observation.geometry()) {
        const Eigen::Vector3d xyz = (T_odom_submap.inverse() * *frame.T_odom_camera()) * observation.geometry()->position_camera;
        if (!xyz.allFinite() || xyz.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
          throw std::invalid_argument("Unrepresentable visual scene geometry");
        landmark.position = {float(xyz.x()), float(xyz.y()), float(xyz.z())};
        landmark.has_position = true;
      }
      auto &appearance = landmark.appearances[0];
      appearance.quality = .5f;
      std::memcpy(appearance.descriptor.data(), frame.descriptors().ptr(row), 32);
      if (unique.insert(appearance.descriptor).second) landmarks.push_back(landmark);
    }
  }
  return mapping::scene::FeatureMap::create(0, std::move(landmarks), mapping::scene::point_capacity);
}

class VisualSubmapIndex::Impl {
 public:
  Impl(const VisualLoopParameters &config, Memory &memory) : config(config), memory(memory) {}
  struct QueryFeatures {
    std::shared_ptr<const features::FeatureBlock> block;
    std::size_t camera_id;
    std::optional<Eigen::Isometry3d> T_query_camera;
    std::optional<RectifiedProjection> projection;
  };
  static std::vector<QueryFeatures> queryFeatures(const std::vector<VisualFrame> &images, const Eigen::Isometry3d &anchor) {
    std::vector<QueryFeatures> frames;
    for (const auto &frame : images) {
      if (frame.camera_id() > 1) throw std::invalid_argument("Unknown visual query camera");
      if (!frame.has_features() || frame.descriptors().type() != CV_8UC1 || frame.descriptors().cols != 32) continue;
      std::vector<cv::KeyPoint> keypoints;
      for (const auto &p : frame.points()) keypoints.emplace_back(p.pixel().x(), p.pixel().y(), 31, -1, p.response(), p.level());
      std::optional<Eigen::Isometry3d> camera;
      if (frame.T_odom_camera()) camera = anchor.inverse() * *frame.T_odom_camera();
      frames.push_back({features::FeatureBlock::create_raw(keypoints, {}, frame.descriptors()), frame.camera_id(), camera, frame.projection()});
    }
    return frames;
  }
  void add(const SubmapFrame &submap) {
    pending[submap.id()] = queryFeatures(submap.visual_frames(), submap.lio().T_odom_base);
    spdlog::info("[visual-loop] submap={} frames={}", submap.id(), pending[submap.id()].size());
  }
  void preparePrefix(std::uint64_t limit) {
    if (config.attributes_only) { next_history = limit; return; }
    if (active && limit != next_history) throw std::logic_error("Active visual archive cannot change population during a query");
    if (historical && next_history > limit) {
      archive.clear();
      next_history = 0;
    }
    while (next_history < limit) {
      const auto scene = config.attributes_only ? nullptr : memory.loadScene(next_history);
      if (scene && !scene->points().empty()) archive.upsert(static_cast<int>(next_history + 1), *scene);
      ++next_history;
      if (historical) archive.poll_training(true);
    }
    archive.poll_training(false);
  }
  void prepareHistory(std::uint64_t id) {
    const auto root = historical ? memory.chainRoot(id) : 0;
    preparePrefix(loop_policy::prefix(id, root));
  }
  void beginContinuation(std::uint64_t root, std::size_t count) {
    if (active || !count || root >= count || memory.chainRoot(count - 1) != root)
      throw std::logic_error("Invalid visual continuation handoff");
    historical = true;
    archive.clear();
    pending.clear();
    next_history = 0;
    node_count = count;
    active_root = root;
    preparePrefix(loop_policy::prefix(count, root));
    archive.poll_training(true);
    active = true;
  }
  void advanceContinuation(std::size_t count) {
    if (!active || count != node_count + 1) throw std::logic_error("Noncontiguous visual commit");
    const auto limit = loop_policy::prefix(count, active_root);
    if (limit < next_history || limit > next_history + 1) throw std::logic_error("Nonmonotone visual prefix");
    if (next_history < limit) {
      const auto scene = config.attributes_only ? nullptr : memory.loadScene(next_history);
      if (scene && !scene->points().empty()) archive.upsert(static_cast<int>(next_history + 1), *scene);
      ++next_history;
    }
    archive.poll_training(true);
    node_count = count;
  }
  void restore(std::size_t count) {
    historical = true;
    active = false;
    node_count = count;
    archive.clear();
    pending.clear();
    next_history = 0;
    if (count) prepareHistory(count - 1);
  }
  std::vector<VisualSubmapMatch> queryHistory(std::uint64_t id) {
    if (id >= node_count) throw std::out_of_range("Unknown historical visual query");
    pending[id] = queryFeatures(memory.loadVisualFrames(id), memory.loadOriginalAnchor(id));
    try {
      auto result = query(id);
      pending.erase(id);
      return result;
    } catch (...) {
      pending.erase(id);
      throw;
    }
  }
  std::vector<VisualSubmapMatch> query(std::uint64_t id) {
    const auto start = timer::Clock::now();
    prepareHistory(id);
    if (timer::enabled()) spdlog::info("[backend-profile] visual_prefix query={} ms={:.3f}", id, timer::ms(start));
    return searchFrames(id, pending.count(id) ? pending.at(id) : std::vector<QueryFeatures>{});
  }
  std::vector<VisualSubmapMatch> searchFrames(std::uint64_t id, const std::vector<QueryFeatures> &frames) {
    if (config.attributes_only) return {};
    const auto start = timer::Clock::now();
    double archive_ms = 0, scene_ms = 0;
    std::map<std::uint64_t, VisualSubmapMatch> retrieved;
    struct Support { std::size_t frame = 0, metric = 0; std::uint64_t fingerprint = 0; std::vector<mapping::scene::Match> matches; };
    std::map<std::uint64_t, Support> supports;
    const VisualPnpParameters pnp_parameters;
    for (std::size_t frame_id = 0; frame_id < frames.size(); ++frame_id) {
      const auto &frame = frames[frame_id].block;
      auto stage = timer::Clock::now();
      auto matches = archive.search(*frame);
      archive_ms += timer::ms(stage);
      stage = timer::Clock::now();
      std::vector<std::pair<int, mapping::DescriptorArchive::Candidate>> ranked(matches.begin(), matches.end());
      std::sort(ranked.begin(), ranked.end(),
                [](const auto &a, const auto &b) { return a.second.score != b.second.score ? a.second.score > b.second.score : a.first < b.first; });
      if (ranked.size() > static_cast<std::size_t>(config.top_k)) ranked.resize(config.top_k);
      std::vector<mapping::scene::PrefetchCache::Key> prefetch;
      for (const auto &[node, candidate] : ranked) prefetch.emplace_back(node, candidate.fingerprint);
      memory.prefetchScenes(prefetch);
      for (const auto &[node, candidate] : ranked) {
        const auto scene = memory.loadScene(node - 1, candidate.fingerprint);
        if (!scene) continue;
        if (candidate.matches.size() < static_cast<std::size_t>(config.min_matches) ||
            candidate.fingerprint != mapping::DescriptorArchive::fingerprint(*scene))
          continue;
        VisualSubmapMatch match{std::uint64_t(node - 1), candidate.matches.size(), candidate.score};
        if (!retrieved.count(node - 1) || retrieved.at(node - 1).matches < match.matches) retrieved[node - 1] = match;
        if (config.enabled && frames[frame_id].T_query_camera && frames[frame_id].projection && candidate.matches.size() <= pnp_parameters.max_pairs) {
          const auto metric = std::count_if(candidate.matches.begin(), candidate.matches.end(),
              [&](const auto &m) { return scene->points().at(m.landmark).has_position; });
          if (metric >= pnp_parameters.min_inliers && (!supports.count(node - 1) || supports.at(node - 1).metric < std::size_t(metric)))
            supports[node - 1] = {frame_id, std::size_t(metric), candidate.fingerprint, candidate.matches};
        }
      }
      scene_ms += timer::ms(stage);
    }
    std::vector<VisualSubmapMatch> results;
    for (const auto &[key, seed] : retrieved) results.push_back(seed);
    std::sort(results.begin(), results.end(),
              [](const auto &a, const auto &b) { return a.matches != b.matches ? a.matches > b.matches : a.target_id < b.target_id; });
    if (results.size() > static_cast<std::size_t>(config.top_k)) results.resize(config.top_k);
    for (auto &match : results) {
      const auto support = supports.find(match.target_id);
      if (support == supports.end()) continue;
      const auto &input = support->second;
      const auto scene = memory.loadScene(match.target_id, input.fingerprint);
      if (!scene || mapping::DescriptorArchive::fingerprint(*scene) != input.fingerprint)
        throw std::logic_error("Visual PnP scene snapshot changed after matching");
      const auto &query = frames.at(input.frame);
      const auto pnp_start = timer::Clock::now();
      CameraParameters camera;
      camera.width = query.projection->width; camera.height = query.projection->height;
      camera.intrinsics.assign(query.projection->intrinsics.begin(), query.projection->intrinsics.end());
      auto estimate = estimateVisualPnpSeed(*scene, *query.block, input.matches, camera,
                                          *query.T_query_camera, pnp_parameters);
      database_detail::writableStorageTestPoint("visual-after-pnp", &estimate);
      match.T_target_query = estimate.T_target_query;
      spdlog::info("[visual-pnp] query={} target={} camera={} metric_pairs={} inliers={} cells={} status={} ms={:.3f}",
          id, match.target_id, query.camera_id, estimate.metric_pairs, estimate.inliers.size(), estimate.occupied_cells,
          int(estimate.status), timer::ms(pnp_start));
    }
    spdlog::info("[visual-loop] query={} retrieved_candidates={} indexed_scenes={}", id, results.size(), archive.stats().nodes);
    if (timer::enabled()) spdlog::info("[backend-profile] visual_query query={} frames={} archive_ms={:.3f} scene_ms={:.3f} total_ms={:.3f}",
        id, frames.size(), archive_ms, scene_ms, timer::ms(start));
    return results;
  }
  std::vector<VisualSubmapMatch> queryTransient(const SubmapFrame &query) {
    if (!historical || !node_count || query.id() != node_count)
      throw std::logic_error("Transient visual query requires the next committed identity");
    if (active) {
      if (next_history != loop_policy::prefix(query.id(), active_root))
        throw std::logic_error("Transient visual query requires current continuation prefix");
    } else preparePrefix(loop_policy::prefix(query.id(), memory.chainRoot(node_count - 1)));
    return searchFrames(query.id(), queryFeatures(query.visual_frames(), query.lio().T_odom_base));
  }
  std::vector<VisualSubmapMatch> queryFreshSession(const SubmapFrame &query, const std::vector<VisualFrame> *observations) {
    if (!historical || active) throw std::logic_error("Fresh association requires an inactive historical archive");
    preparePrefix(node_count);
    archive.poll_training(true);
    return searchFrames(query.id(), queryFeatures(observations ? *observations : query.visual_frames(), query.lio().T_odom_base));
  }
  bool active = false;
  std::uint64_t active_root = 0;
  VisualLoopParameters config;
  Memory &memory;
  mapping::DescriptorArchive archive;
  std::map<std::uint64_t, std::vector<QueryFeatures>> pending;
  std::uint64_t next_history = 0;
  bool historical = false;
  std::size_t node_count = 0;
};
VisualSubmapIndex::VisualSubmapIndex(const VisualLoopParameters &c, Memory &m) : impl_(std::make_unique<Impl>(c, m)) {}
void VisualSubmapIndex::retireScene(std::uint64_t id) {
  if(impl_->memory.sceneActive(id)) throw std::logic_error("Cannot retire an active committed scene");
  impl_->archive.erase(static_cast<int>(id+1)); impl_->pending.erase(id); impl_->archive.poll_training(true);
}
VisualSubmapIndex::~VisualSubmapIndex() = default;
void VisualSubmapIndex::addSubmap(const SubmapFrame &submap) { impl_->add(submap); }
std::vector<VisualSubmapMatch> VisualSubmapIndex::query(std::uint64_t id) { return impl_->historical ? impl_->queryHistory(id) : impl_->query(id); }
void VisualSubmapIndex::settle() { impl_->archive.poll_training(true); }
void VisualSubmapIndex::finishQuery(std::uint64_t id) { impl_->pending.erase(id); }
void VisualSubmapIndex::restoreHistory(std::size_t count) { impl_->restore(count); }
void VisualSubmapIndex::materializeCommittedRoot(std::size_t count) {
  impl_->restore(count);
  impl_->preparePrefix(count);
}
mapping::DescriptorArchive::IndexNodeMetadata VisualSubmapIndex::historyMetadata() const { return impl_->archive.node_metadata(); }
}  // namespace sapphire

namespace sapphire {
void VisualSubmapIndex::beginContinuation(std::uint64_t root, std::size_t count) { impl_->beginContinuation(root, count); }
void VisualSubmapIndex::advanceContinuation(std::size_t count) { impl_->advanceContinuation(count); }
std::vector<VisualSubmapMatch> VisualSubmapIndex::queryTransient(const SubmapFrame &q) { return impl_->queryTransient(q); }
std::uint64_t VisualSubmapIndex::eligiblePrefix() const { return impl_->next_history; }
}

namespace sapphire { mapping::DescriptorArchive::Stats VisualSubmapIndex::historyStats() const { return impl_->archive.stats(); } }

namespace sapphire { std::vector<VisualSubmapMatch> VisualSubmapIndex::queryFreshSession(const SubmapFrame &q, const std::vector<VisualFrame> *observations) { return impl_->queryFreshSession(q, observations); } }
