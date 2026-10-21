#include <backend/registration/descriptor/landmark_descriptors.hpp>
#include <backend/registration/matcher/projection_match.hpp>
#include <backend/solver/triangulator.hpp>
#include <numeric>
#include <unordered_set>

#include "backend/registration/matcher/nearest.hpp"
namespace sapphire::backend::featmap {
using detail::budget_result;
using detail::descriptor_match_gate;
using detail::finish;
using detail::DescriptorNearest;
using detail::valid_pixel;
template <class Format>
MatchResult match_fixed_pose_impl(const VisualFeatures& reference, const camera::Perspective& reference_camera, const VisualFeatures& features,
                                  const camera::Perspective& camera, const StateGroupf& query_from_reference, const std::vector<double>& variances,
                                  const MatchOptions& options) {
  const auto gate = descriptor_match_gate<Format>(options.ratio, options.max_distance);
  const auto valid_descriptors = [](const VisualFeatures& f) {
    return f.descriptors.type() == descriptor_traits<Format>::cv_type && f.descriptors.cols == descriptor_traits<Format>::dimensions;
  };
  if (!reference_camera.isValidForReprojection() || !(reference_camera.focalXBaseline() > 0) || !reference_camera.cameraToBase().isValid() ||
      reference_camera.imageSize().width <= 0 || reference_camera.imageSize().height <= 0 || !valid_descriptors(reference) ||
      reference.descriptors.rows != int(reference.keypoints.size()) || reference.points.size() != reference.keypoints.size())
    throw std::invalid_argument("Invalid fixed-pose reference measurements");
  if (!query_from_reference.isValid() || !camera.isValidForProjection() || !camera.cameraToBase().isValid() || camera.imageSize().width <= 0 ||
      camera.imageSize().height <= 0 || variances.empty() || options.min_matches == 0 || !valid_descriptors(features) ||
      features.descriptors.rows != int(features.keypoints.size()) || features.points.size() != features.keypoints.size())
    throw std::invalid_argument("Invalid fixed-pose projection input");
  MatchResult result;
  if (reference.descriptor_model_id != features.descriptor_model_id || reference.descriptors.cols != features.descriptors.cols) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  const feature::descriptor_matrix<Format> reference_rows(reference.descriptors), query_rows(features.descriptors);
  if (features.keypoints.size() > bow::FeatureBlock::max_features || reference.keypoints.size() > bow::FeatureBlock::max_features) return result;
  double max_variance = 0;
  for (double v : variances) {
    if (!(v > 0) || !std::isfinite(v)) throw std::invalid_argument("Invalid fixed-pose pixel variance");
    max_variance = std::max(v, max_variance);
  }
  // This one-shot index stores only row links, borrowing the immutable input
  // arrays. No FeatureBlock, descriptor copy or vocabulary quantization is
  // needed.
  constexpr int cell = 32;
  const int columns = (camera.imageSize().width + cell - 1) / cell, rows = (camera.imageSize().height + cell - 1) / cell;
  if (int64_t(columns) * rows > 1048576) return result;
  std::vector<int> heads(size_t(columns) * rows, -1), next(features.keypoints.size(), -1);
  std::vector<double> right(features.keypoints.size(), -1);
  const auto optical_from_body = camera.cameraToBase().inverse();
  for (size_t q = 0; q < features.keypoints.size(); ++q) {
    const auto& k = features.keypoints[q];
    if (!valid_pixel(k) || !camera.isInImage(k.pt.x, k.pt.y) || k.octave < 0 || size_t(k.octave) >= variances.size()) continue;
    const size_t bin = size_t(int(k.pt.y) / cell) * columns + int(k.pt.x) / cell;
    next[q] = heads[bin];
    heads[bin] = int(q);
    if (!features.points.empty() && camera.focalXBaseline() > 0) {
      const auto& p = features.points[q];
      const auto optical = optical_from_body.transformPoint(Eigen::Vector3f(p.x, p.y, p.z));
      if (optical.allFinite() && optical.z() > 0) right[q] = k.pt.x - camera.focalXBaseline() / optical.z();
    }
  }
  const auto camera_from_map = optical_from_body.compose(query_from_reference);
  const auto map_from_camera = camera_from_map.inverse();
  const Eigen::Matrix3d rotation = map_from_camera.rotationMatrix().cast<double>();
  const Eigen::Vector3d translation = map_from_camera.translationVector().cast<double>();
  const std::array<float, 5> calibration{float(camera.fx()), float(camera.fy()), float(camera.cx()), float(camera.cy()), float(camera.focalXBaseline())};
  const float radius = std::min(32.0, std::sqrt(5.99146 * max_variance));
  std::vector<DescriptorNearest<Format>> forward(reference.keypoints.size()), reverse(features.keypoints.size());
  for (uint32_t p = 0; p < reference.keypoints.size(); ++p) {
    const feature::descriptor_distance<Format> matcher(reference_rows.row(int(p)));
    const auto& point = reference.points[p];
    const auto& key = reference.keypoints[p];
    if (!valid_pixel(key) || !reference_camera.isInImage(key.pt.x, key.pt.y) || key.octave < 0 || size_t(key.octave) >= variances.size() ||
        !solver::Triangulator::stereo_seed_valid(camera::stereo_calibration(reference_camera),
                                                 camera::stereo_pixel(reference_camera, key, point, std::sqrt(variances[key.octave]))))
      continue;
    const Eigen::Vector3d xyz(point.x, point.y, point.z);
    if (!xyz.allFinite()) continue;
    const auto optical = camera_from_map.transformPoint(Eigen::Vector3f(xyz.cast<float>()));
    const auto pixel = camera.project(cv::Point3d(optical.x(), optical.y(), optical.z()));
    if (!pixel || !camera.isInImage(pixel->x, pixel->y)) continue;
    ++result.projected_points;
    const int x0 = std::max(0, int(std::floor((pixel->x - radius) / cell))), x1 = std::min(columns - 1, int((pixel->x + radius) / cell));
    const int y0 = std::max(0, int(std::floor((pixel->y - radius) / cell))), y1 = std::min(rows - 1, int((pixel->y + radius) / cell));
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x)
        for (int q = heads[size_t(y) * columns + x]; q >= 0; q = next[q]) {
          if (++result.spatial_visits > options.max_comparisons) return budget_result(std::move(result));
          const auto& k = features.keypoints[q];
          if (std::abs(k.pt.x - pixel->x) > radius || std::abs(k.pt.y - pixel->y) > radius) continue;
          const double error = camera::Perspective::squaredPixelError(rotation, translation, xyz, calibration, {k.pt.x, -1, k.pt.y});
          if (!std::isfinite(error) || error > 5.99146 * variances[k.octave]) continue;
          if (result.comparisons >= options.max_comparisons) return budget_result(std::move(result));
          const float score = matcher.score(query_rows.row(q));
          ++result.comparisons;
          forward[p].add(q, score);
          reverse[q].add(p, score);
        }
  }
  for (uint32_t p = 0; p < forward.size(); ++p) {
    const auto best = forward[p].distances();
    if (!best.accepts(gate)) continue;
    const auto back = reverse[best.first.id].distances();
    if (back.first.id != p || !back.accepts(gate)) continue;
    const auto q = best.first.id;
    const auto& k = features.keypoints[q];
    const auto& point = reference.points[p];
    const Eigen::Vector3d xyz(point.x, point.y, point.z);
    const double error = camera::Perspective::squaredPixelError(rotation, translation, xyz, calibration, {k.pt.x, right[q], k.pt.y});
    if (!std::isfinite(error)) continue;
    if (right[q] >= 0 && error > 7.81473 * variances[k.octave]) {
      // Left-image geometry and a unique appearance agree, but the measured
      // stereo depth does not. Compare right-pixel disagreement against both
      // captures' pixel scales; no fitted pose or depth confidence is invented.
      const auto optical = camera_from_map.transformPoint(Eigen::Vector3f(point.x, point.y, point.z));
      const double predicted_right = camera.fx() * optical.x() / optical.z() + camera.cx() - camera.focalXBaseline() / optical.z();
      const double dr = predicted_right - right[q];
      if (solver::Triangulator::stereo_seed_valid(camera::stereo_calibration(camera),
                                                  camera::stereo_pixel(camera, k, features.points[q], std::sqrt(variances[k.octave]))) &&
          dr * dr > 9.0 * (variances[k.octave] + variances[reference.keypoints[p].octave]))
        ++result.inconsistent_depth;
      continue;
    }
    result.pairs.push_back({p, q, best.first.distance});
  }
  finish(result, options.min_matches);
  return result;
}
MatchResult match_fixed_pose(const VisualFeatures& reference, const camera::Perspective& reference_camera, const VisualFeatures& features,
                             const camera::Perspective& camera, const StateGroupf& query_from_reference, const std::vector<double>& variances,
                             const MatchOptions& options) {
  return feature::dispatch_descriptor(reference.descriptors.cols, [&](auto format) {
    return match_fixed_pose_impl<decltype(format)>(reference, reference_camera, features, camera, query_from_reference, variances, options);
  });
}

template <class Format>
MatchResult project_unmatched_impl(const LandmarkMap& map, const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& query_features,
                                   const StateGroupf& query_from_map, const std::vector<Match>& verified_seeds, const ProjectionOptions& options) {
  const auto gate = descriptor_match_gate<Format>(1.f, options.max_distance);
  if (!std::isfinite(options.radius) || options.radius <= 0 || options.radius > 32 || !query_from_map.isValid() || query.perspectives().size() != 1 ||
      (!query_features || query.shared_features() != query_features))
    throw std::invalid_argument("Invalid monocular projection query");
  const auto& camera = query.perspectives().front();
  if (!camera.isValidForProjection() || camera.imageSize().width <= 0 || camera.imageSize().height <= 0 || !camera.cameraToBase().isValid())
    throw std::invalid_argument("Invalid projection camera");
  const auto& features = *query_features;
  features.validate();
  MatchResult result;
  if (features.descriptor_model_id != map.descriptor_model_id() || features.descriptors.cols != map.descriptor_dimensions() ||
      (!features.is_raw() && features.vocabulary_hash != map.vocabulary_hash())) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  const feature::landmark_descriptors<Format> map_rows(map);
  const feature::descriptor_matrix<Format> descriptors(features.descriptors);
  std::vector<uint8_t> used_points(map.points().size()), used_queries(features.keypoints.size());
  for (const auto& seed : verified_seeds) {
    if (seed.landmark >= used_points.size() || seed.feature >= used_queries.size() || used_points[seed.landmark] || used_queries[seed.feature] ||
        !valid_pixel(features.keypoints[seed.feature]))
      throw std::invalid_argument("Invalid or nonunique projection seed");
    used_points[seed.landmark] = used_queries[seed.feature] = 1;
  }
  if (verified_seeds.size() < options.min_seed_matches) return result;

  const auto camera_from_map = camera.cameraToBase().inverse().compose(query_from_map);
  std::vector<DescriptorNearest<Format>> forward(map.points().size()), reverse(features.keypoints.size());
  std::vector<size_t> indices;
  for (uint32_t p = 0; p < map.points().size(); ++p) {
    if (used_points[p]) continue;
    if (result.projected_points == options.max_projected_points) return budget_result(std::move(result));
    ++result.projected_points;
    const auto& point = map.points()[p];
    const feature::descriptor_distance<Format> matcher(map_rows.row(p));
    const auto xyz = camera_from_map.transformPoint(Eigen::Vector3f(point.position.x, point.position.y, point.position.z));
    const auto pixel = camera.project(cv::Point3d(xyz.x(), xyz.y(), xyz.z()));
    if (!pixel || !camera.isInImage(pixel->x, pixel->y)) continue;
    size_t visits = 0;
    const bool complete =
        query.collect_keypoint_indices(cv::Point2f(pixel->x, pixel->y), options.radius, options.max_spatial_visits - result.spatial_visits, indices, visits);
    result.spatial_visits += visits;
    if (!complete) return budget_result(std::move(result));
    for (const auto q : indices) {
      if ((options.detect_duplicates ? !used_queries[q] : bool(used_queries[q])) || !valid_pixel(features.keypoints[q]) ||
          !camera.isInImage(features.keypoints[q].pt.x, features.keypoints[q].pt.y))
        continue;
      if (result.comparisons >= options.max_comparisons) return budget_result(std::move(result));
      const float score = matcher.score(descriptors.row(int(q)));
      ++result.comparisons;
      forward[p].add(static_cast<uint32_t>(q), score);
      reverse[q].add(p, score);
    }
  }
  for (uint32_t p = 0; p < forward.size(); ++p) {
    const auto nearest = forward[p].distances();
    if (!nearest.accepts(gate)) continue;
    const auto other = reverse[nearest.first.id].distances();
    if (other.first.id != p || !other.accepts(gate)) continue;
    result.pairs.push_back({p, nearest.first.id, nearest.first.distance});
  }
  finish(result, 1);
  return result;
}

MatchResult project_unmatched(const LandmarkMap& map, const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& features,
                              const StateGroupf& pose, const std::vector<Match>& seeds, const ProjectionOptions& options) {
  return feature::dispatch_descriptor(map.descriptor_dimensions(),
                                      [&](auto format) { return project_unmatched_impl<decltype(format)>(map, query, features, pose, seeds, options); });
}

}  // namespace sapphire::backend::featmap
