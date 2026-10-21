#include "backend/maplayer/featmap/merge.hpp"
#include "backend/solver/triangulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>

namespace sapphire::backend::featmap {
static MergeDraft prepare_content(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
                         const std::vector<Match>& inliers, const std::vector<Match>& projected, const std::vector<uint32_t>& rejected_rows,
                         const std::vector<float>& quality, const GeometryOptions& geometry, const AppearanceOptions& appearance, uint64_t identity_floor, bool refresh) {
  MergeDraft result;
  result.geometry = prepare_node_update(target, query, query_features, query_from_target, inliers, projected, quality, geometry, appearance);
  if (result.geometry.status() != UpdateStatus::ready) return result;
  const auto& frame = *query_features;
  std::vector<bool> blocked(frame.keypoints.size(), false), supported(target->points().size(), false);
  for (const uint32_t row : rejected_rows) {
    if (row >= blocked.size()) throw std::invalid_argument("Rejected submap row outside query");
    blocked[row] = true;
  }
  for (const auto* pairs : {&inliers, &projected}) {
    for (const auto& pair : *pairs) {
      if (result.geometry.query_checks()[pair.feature] == PointCheck::compatible) supported.at(pair.landmark) = true;
    }
  }
  const auto target_from_query = query_from_target.inverse();
  const auto camera_from_query = query.perspectives().front().cameraToBase().inverse();
  std::vector<Landmark> incoming;
  uint64_t next_id = identity_floor;
  for (const auto& point : target->points()) next_id = std::max(next_id, point.id);
  for (uint32_t row = 0; row < frame.keypoints.size(); ++row) {
      if ((blocked[row] && result.geometry.query_checks()[row] != PointCheck::compatible) || result.geometry.query_checks()[row] == PointCheck::conflict) {
        ++result.stats.blocked;
        continue;
      }
      if (result.geometry.query_checks()[row] != PointCheck::unmatched || !std::isfinite(quality[row]) || quality[row] < appearance.min_quality) continue;
      const auto& point = frame.points[row];
      const Eigen::Vector3f body(point.x, point.y, point.z);
      if (!body.allFinite()) continue;
      const auto optical = camera_from_query.transformPoint(body);
      const auto pixel = query.perspectives().front().project(cv::Point3f(optical.x(), optical.y(), optical.z()));
      if (!optical.allFinite() || optical.z() <= 0 || !pixel || !query.perspectives().front().isInImage(frame.keypoints[row].pt.x, frame.keypoints[row].pt.y) ||
          cv::norm(cv::Point2f(pixel->x, pixel->y) - frame.keypoints[row].pt) > geometry.max_pixel_error)
        continue;
      const float sigma = geometry.pixel_scale * std::pow(geometry.pyramid_scale,frame.keypoints[row].octave);
      const auto stereo = camera::stereo_pixel(query.perspectives().front(),frame.keypoints[row],point,sigma);
      if (query.perspectives().front().focalXBaseline()>0 && !solver::Triangulator::stereo_seed_valid(camera::stereo_calibration(query.perspectives().front()), stereo)) continue;
      if (next_id == std::numeric_limits<uint64_t>::max()) throw std::overflow_error("Node point IDs exhausted");
      Landmark added;
      added.id = ++next_id;
      const auto position = target_from_query.transformPoint(body);
      added.position = {position.x(), position.y(), position.z()};
      if (stereo.valid()) {
        const auto& sensor = query.perspectives().front();
        const camera::StereoObservation measured(sensor.cameraToBase(), camera::stereo_calibration(sensor), stereo.pixel, stereo.sigma);
        added.geometry = PointSummary::from_observation(measured, StateGroupd(target_from_query).normalized(), position.cast<double>());
      }
      // This observation belongs to another keyframe. Its estimated transform
      // into this submap is not an exact camera extrinsic. Keep it as a normal
      // map point; do not manufacture a fixed-reference stereo factor here.
      // The retained query node owns the original stereo observation.
      added.has_appearance = true;
      auto& sample = added.appearance;
      sample.tree_node = frame.tree_node(row);
      sample.word_id = frame.word_id(row) > 0 ? frame.word_id(row) : -1;
      sample.quality = quality[row];
      sample.ray = observation_ray(query.perspectives().front(), frame.keypoints[row].pt, target_from_query);
      sample.descriptor = feature::LocalDescriptor::from_row(frame.descriptors, int(row), frame.descriptor_model_id);
      result.inserted_rows.emplace(added.id,row);
      incoming.push_back(added);
  }
  result.stats.eligible = incoming.size();
  const auto& existing = result.geometry.draft()->points();
  std::vector<Landmark> selected;
  selected.reserve(existing.size() + incoming.size());
  for (size_t i = 0; i < existing.size(); ++i) {
    if (!refresh && !supported[i]) continue;
    auto point = existing[i];
    if (supported[i]) point.miss_count = 0;
    else if (point.miss_count != std::numeric_limits<uint32_t>::max()) ++point.miss_count;
    selected.push_back(std::move(point));
  }
  selected.insert(selected.end(), incoming.begin(), incoming.end());
  LandmarkMap::retain_best(selected, target->max_points());
  std::sort(selected.begin(), selected.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
  std::set<uint64_t> retained_ids;
  for (const auto& point : selected) retained_ids.insert(point.id);
  for (auto it = result.inserted_rows.begin(); it != result.inserted_rows.end();)
    if (!retained_ids.count(it->first)) it = result.inserted_rows.erase(it);
    else ++it;
  result.stats.added = result.inserted_rows.size();
  result.stats.retired = existing.size() - (selected.size() - result.stats.added);
  result.features = LandmarkMap::create(refresh ? target->vocabulary_hash() : (frame.is_raw() ? 0 : target->vocabulary_hash()),
                                      std::move(selected), target->max_points(), target->descriptor_model_id(), target->descriptor_dimensions());
  return result;
}
MergeDraft prepare_merge(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
    const std::vector<Match>& inliers, const std::vector<Match>& projected, const std::vector<uint32_t>& rejected_rows,
    const std::vector<float>& quality, const GeometryOptions& geometry, const AppearanceOptions& appearance, uint64_t identity_floor) {
  return prepare_content(std::move(target), query, query_features, query_from_target, inliers, projected, rejected_rows,
                         quality, geometry, appearance, identity_floor, false);
}
MergeDraft prepare_refresh(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
    const std::vector<Match>& inliers, const std::vector<uint32_t>& rejected_rows, const std::vector<float>& quality,
    const GeometryOptions& geometry, const AppearanceOptions& appearance, uint64_t identity_floor) {
  return prepare_content(std::move(target), query, query_features, query_from_target, inliers, {}, rejected_rows,
                         quality, geometry, appearance, identity_floor, true);
}

MergeDraft prepare_loop_merge(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
                              const std::vector<Match>& inliers, const std::vector<uint32_t>& rejected_rows, const std::vector<float>& quality,
                              const GeometryOptions& geometry, const AppearanceOptions& appearance, const ProjectionOptions& options) {
  if (!target) throw std::invalid_argument("Missing loop map");
  auto projection = project_unmatched(*target, query, query_features, query_from_target, inliers, options);
  bool incomplete = projection.status == MatchStatus::budget_exceeded;
  if (inliers.size() <= geometry.max_pairs && projection.pairs.size() > geometry.max_pairs - inliers.size()) {
    projection.pairs.clear();
    projection.status = MatchStatus::budget_exceeded;
    incomplete = true;
  }
  auto insertion_exclusions = rejected_rows;
  if (incomplete) {
    // Without a completed association search, an unmatched row may be a
    // duplicate. Do not insert unexamined rows as new map points.
    insertion_exclusions.resize(query_features->keypoints.size());
    for (uint32_t i = 0; i < insertion_exclusions.size(); ++i) insertion_exclusions[i] = i;
  }
  auto result = prepare_merge(target, query, query_features, query_from_target, inliers, projection.pairs, insertion_exclusions, quality, geometry, appearance);
  result.projection = std::move(projection);
  return result;
}
}  // namespace sapphire::backend::featmap
