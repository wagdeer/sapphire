#include "backend/maplayer/featmap/update.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace sapphire::backend::featmap {
namespace {
bool positive(float value) { return std::isfinite(value) && value > 0; }
bool ratio(float value) { return std::isfinite(value) && value >= 0 && value <= 1; }

enum class ConflictReason { none, projection };
struct PointEvidence {
  PointCheck check;
  ConflictReason reason = ConflictReason::none;
  bool ray_checked = true;
};
PointEvidence check_point(const Landmark& point, const cv::Point2f& pixel, const camera::Perspective& camera,
                          const StateGroupf& camera_from_map, double pixel_variance) {
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !camera.isInImage(pixel.x, pixel.y))
    return {PointCheck::unavailable, ConflictReason::none, false};
  const auto& source = point.position;
  const auto predicted = camera_from_map.transformPoint(Eigen::Vector3f(source.x, source.y, source.z));
  const auto projection = camera.project(cv::Point3d(predicted.x(), predicted.y(), predicted.z()));
  if (!predicted.allFinite() || predicted.z() <= 0 || !projection) return {PointCheck::conflict, ConflictReason::projection};
  const auto error = *projection - cv::Point2d(pixel);
  return error.dot(error) <= 5.99146 * pixel_variance
             ? PointEvidence{PointCheck::compatible}
             : PointEvidence{PointCheck::conflict, ConflictReason::projection};
}
}  // namespace

NodeUpdate prepare_node_update(std::shared_ptr<const LandmarkMap> map, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_map,
                                 const std::vector<Match>& verified_seeds, const std::vector<Match>& projected, const std::vector<float>& observation_quality,
                                 const GeometryOptions& geometry, const AppearanceOptions& appearance) {
  const auto start = std::chrono::steady_clock::now();
  if (!map || (!query_features || query.shared_features() != query_features) || query.perspectives().size() != 1 || !query_from_map.isValid() || !positive(geometry.max_pixel_error) ||
      !positive(geometry.pyramid_scale) || (geometry.pyramid_levels > 1 && geometry.pyramid_scale <= 1) ||
      geometry.pyramid_levels < 1 || geometry.pyramid_levels > 64 || !positive(geometry.pixel_scale) ||
      ((geometry.min_seed_support == 0) != (geometry.min_support == 0)) || geometry.max_pairs > point_capacity)
    throw std::invalid_argument("Invalid submap geometry options or input");
  const auto& camera = query.perspectives().front();
  if (!camera.isValidForProjection() || !camera.cameraToBase().isValid() || camera.imageSize().width <= 0 || camera.imageSize().height <= 0)
    throw std::invalid_argument("Invalid submap geometry camera");
  const auto& features = *query_features;
  features.validate();
  if(features.descriptor_model_id!=map->descriptor_model_id() || features.descriptors.cols!=map->descriptor_dimensions()) throw std::invalid_argument("Fusion descriptor model mismatch");
  std::vector<double> pixel_variances(geometry.pyramid_levels);
  double scale = geometry.pixel_scale;
  for (auto& variance : pixel_variances) {
    variance = scale * scale;
    if (!std::isfinite(variance) || variance <= 0) throw std::invalid_argument("Invalid submap pixel variance");
    scale *= geometry.pyramid_scale;
  }
  for (const auto& keypoint : features.keypoints)
    if (keypoint.octave < 0 || keypoint.octave >= geometry.pyramid_levels) throw std::invalid_argument("Invalid submap feature octave");
  if (observation_quality.size() != features.keypoints.size()) throw std::invalid_argument("Node quality must follow query feature rows");
  for (const float quality : observation_quality)
    if (!ratio(quality)) throw std::invalid_argument("Invalid submap observation quality");
  // Validate even if no geometry passes, so malformed settings never hide in data.
  Landmark probe;
  update_appearance(probe, Appearance{}, false, appearance);

  NodeUpdate result;
  result.query_checks_.assign(features.keypoints.size(), PointCheck::unmatched);
  const auto finish = [&](UpdateStatus status) {
    result.status_ = status;
    result.stats_.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return std::move(result);
  };
  if (!features.is_raw() && features.vocabulary_hash != map->vocabulary_hash()) return finish(UpdateStatus::vocabulary_mismatch);
  if (verified_seeds.size() > geometry.max_pairs || projected.size() > geometry.max_pairs - verified_seeds.size()) return finish(UpdateStatus::budget_exceeded);

  std::vector<uint8_t> used_points(map->points().size());
  std::vector<uint8_t> used_features(features.keypoints.size());
  for (const auto* pairs : {&verified_seeds, &projected})
    for (const auto& pair : *pairs) {
      if (pair.landmark >= used_points.size() || pair.feature >= used_features.size() || used_points[pair.landmark] || used_features[pair.feature] ||
          pair.distance < 0 || pair.distance > 256)
        throw std::invalid_argument("Invalid or nonunique submap association");
      used_points[pair.landmark] = used_features[pair.feature] = 1;
    }

  const auto camera_from_query = camera.cameraToBase().inverse();
  const auto camera_from_map = camera_from_query.compose(query_from_map);
  std::array<bool, 12> image_cells{};
  const auto map_from_query = query_from_map.inverse();
  for (const auto* pairs : {&verified_seeds, &projected}) {
    const bool seed = pairs == &verified_seeds;
    for (const auto& pair : *pairs) {
      const auto& pixel = features.keypoints[pair.feature].pt;
      const auto evidence = check_point(map->points()[pair.landmark], pixel, camera, camera_from_map,
                                        pixel_variances[features.keypoints[pair.feature].octave]);
      const auto check = evidence.check;
      if (seed && evidence.ray_checked) ++result.stats_.seed_ray_checks;
      if (evidence.reason == ConflictReason::projection) {
        ++result.stats_.projection_conflicts;
        if (seed) ++result.stats_.seed_projection_conflicts;
      }
      result.query_checks_[pair.feature] = check;
      ++result.stats_.checked;
      if (check == PointCheck::compatible) {
        ++result.stats_.compatible;
        if (seed) {
          ++result.stats_.seed_support;
          const int col = std::clamp(int(pixel.x * 4 / camera.imageSize().width), 0, 3);
          const int row = std::clamp(int(pixel.y * 3 / camera.imageSize().height), 0, 2);
          image_cells[row * 4 + col] = true;
        }
      } else if (check == PointCheck::conflict) {
        ++result.stats_.conflicts;
        if (seed) ++result.stats_.seed_ray_conflicts;
      } else {
        ++result.stats_.unavailable;
        if (seed) ++result.stats_.seed_unavailable;
      }
    }
  }
  result.stats_.image_cells = std::count(image_cells.begin(), image_cells.end(), true);
  // Reject image outliers per association, then count the remaining support.
  // There is no independent depth-consensus veto. Projection additions cannot
  // replace the minimum support and coverage from the original PnP seeds.
  if (result.stats_.seed_support < geometry.min_seed_support || result.stats_.compatible < geometry.min_support)
    return finish(UpdateStatus::insufficient_support);

  // Most revisits carry duplicate appearances. Delay the full point-bank copy
  // until an appearance changes; unchanged visits reuse the snapshot.
  std::vector<Landmark> points;
  std::vector<uint32_t> nodes(features.keypoints.size(), bow::FeatureBlock::invalid_node_id);
  for (const auto& group : features.groups) std::fill(nodes.begin() + group.begin, nodes.begin() + group.end, group.node_id);
  for (const auto* pairs : {&verified_seeds, &projected})
    for (const auto& pair : *pairs) {
      if (result.query_checks_[pair.feature] != PointCheck::compatible) continue;
      Appearance candidate;
      candidate.descriptor = feature::LocalDescriptor::from_row(features.descriptors, int(pair.feature), features.descriptor_model_id);
      candidate.quality = observation_quality[pair.feature];
      candidate.tree_node = nodes[pair.feature];
      candidate.word_id = features.word_id(pair.feature);
      candidate.ray = observation_ray(camera, features.keypoints[pair.feature].pt, map_from_query);
      auto point = points.empty() ? map->points()[pair.landmark] : points[pair.landmark];
      const auto update = update_appearance(point, candidate, true, appearance);
      if (update == AppearanceUpdate::appended || update == AppearanceUpdate::replaced) {
        if (points.empty()) points = map->points();
        points[pair.landmark] = std::move(point);
      }
      if (update == AppearanceUpdate::appended) ++result.stats_.appended;
      if (update == AppearanceUpdate::replaced) ++result.stats_.replaced;
    }
  if (features.is_raw() && map->vocabulary_hash() && points.empty()) points = map->points();
  result.draft_ = !points.empty() ? LandmarkMap::create(features.is_raw() ? 0 : map->vocabulary_hash(), std::move(points), map->max_points(), map->descriptor_model_id()) : map;
  return finish(UpdateStatus::ready);
}

}  // namespace sapphire::backend::featmap
