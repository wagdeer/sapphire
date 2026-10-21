#include "backend/maplayer/featmap/landmark_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include "backend/registration/descriptor/landmark_descriptors.hpp"
#include "backend/solver/triangulator.hpp"

namespace sapphire::backend::featmap {
namespace {
void validate_ray(const ObservationRay& ray) {
  const Eigen::Vector3f origin(ray.origin.x, ray.origin.y, ray.origin.z), direction(ray.direction.x, ray.direction.y, ray.direction.z);
  if (!std::isfinite(ray.pixel_angle) || ray.pixel_angle < 0 || ray.pixel_angle > 1 || !origin.allFinite() || !direction.allFinite() ||
      (ray.pixel_angle > 0 && std::abs(direction.squaredNorm() - 1) > 1e-3f))
    throw std::invalid_argument("Invalid submap observation ray");
}

void validate_appearance(const Appearance& appearance) {
  if (!std::isfinite(appearance.quality) || appearance.quality < 0 || appearance.quality > 1 || appearance.word_id < -1)
    throw std::invalid_argument("Invalid submap appearance");
  validate_ray(appearance.ray);
}

bool same_ray(const ObservationRay& a, const ObservationRay& b) { return a.origin == b.origin && a.direction == b.direction && a.pixel_angle == b.pixel_angle; }

}  // namespace

ObservationRay observation_ray(const camera::Perspective& camera, cv::Point2f pixel, const StateGroupf& anchor_from_body) {
  if (!camera.isValidForProjection() || !camera.cameraToBase().isValid() || !anchor_from_body.isValid() || !std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
      !camera.isInImage(pixel.x, pixel.y))
    throw std::invalid_argument("Invalid camera observation");
  const auto bearing = camera.bearing(pixel);
  if (!bearing) throw std::invalid_argument("Invalid observation bearing");
  const auto pose = anchor_from_body.compose(camera.cameraToBase());
  const Eigen::Vector3f direction = (pose.rotationMatrix() * Eigen::Vector3f(bearing->x, bearing->y, bearing->z)).normalized();
  const auto center = pose.translationVector();
  return {{center.x(), center.y(), center.z()}, {direction.x(), direction.y(), direction.z()}, float(1.0 / std::min(camera.fx(), camera.fy()))};
}

AppearanceUpdate update_appearance(Landmark& point, const Appearance& candidate, bool geometry_verified, const AppearanceOptions& options) {
  if (!std::isfinite(options.min_quality) || options.min_quality < 0 || options.min_quality > 1)
    throw std::invalid_argument("Invalid appearance quality threshold");
  validate_appearance(candidate);
  if (point.has_appearance) validate_appearance(point.appearance);
  if (!geometry_verified) return AppearanceUpdate::unverified;
  if (candidate.quality < options.min_quality) return AppearanceUpdate::discarded;
  const bool initialized = point.has_appearance;
  const auto& previous = point.appearance;
  if (initialized && candidate.descriptor == previous.descriptor && candidate.tree_node == previous.tree_node && candidate.word_id == previous.word_id &&
      candidate.quality == previous.quality && same_ray(candidate.ray, previous.ray))
    return AppearanceUpdate::discarded;
  point.appearance = candidate;
  point.has_appearance = true;
  return initialized ? AppearanceUpdate::replaced : AppearanceUpdate::appended;
}

void LandmarkMap::retain_best(std::vector<Landmark>& points, uint32_t max_points) {
  if (!max_points || max_points > point_capacity) throw std::invalid_argument("Invalid point retention capacity");
  if (points.size() <= max_points) return;
  std::vector<float> separation(points.size());
  const auto calculate = [&](auto format) {
    using Format = decltype(format);
    std::fill(separation.begin(), separation.end(), descriptor_traits<Format>::metric == DistanceMetric::l2 ? 2.f : 256.f);
    const auto rows = feature::make_descriptor_range<Format>(
        points, [](const auto& point) -> const auto& { return point.appearance.descriptor; });
    for (size_t i = 0; i < points.size(); ++i) {
      const feature::descriptor_distance<Format> matcher(rows.row(i));
      for (size_t j = i + 1; j < points.size(); ++j) {
        const float distance = matcher.distance(rows.row(j));
        separation[i] = std::min(separation[i], distance);
        separation[j] = std::min(separation[j], distance);
      }
    }
  };
  feature::dispatch_descriptor(points.front().appearance.descriptor.dimensions(), calculate);
  std::vector<size_t> order(points.size());
  std::iota(order.begin(), order.end(), 0);
  const double distance_scale = (points.front().appearance.descriptor.metric() == DistanceMetric::l2) ? 2.0 : 256.0;
  const auto quality = [&](size_t i) { return separation[i] / distance_scale + points[i].depth_quality; };
  std::partial_sort(order.begin(), order.begin() + max_points, order.end(), [&](size_t a, size_t b) {
    if (points[a].miss_count != points[b].miss_count) return points[a].miss_count < points[b].miss_count;
    const double qa = quality(a), qb = quality(b);
    if (qa != qb) return qa > qb;
    return points[a].id < points[b].id;
  });
  order.resize(max_points);
  std::sort(order.begin(), order.end());
  for (size_t i = 0; i < order.size(); ++i)
    if (i != order[i]) points[i] = std::move(points[order[i]]);
  points.resize(max_points);
}

std::shared_ptr<const LandmarkMap> LandmarkMap::create(uint64_t vocabulary_hash, std::vector<Landmark> points, uint32_t max_points, uint64_t model_id, int dimensions) {
  if (max_points == 0 || max_points > point_capacity || points.size() > max_points) throw std::invalid_argument("Invalid submap point capacity");
  auto map = std::shared_ptr<LandmarkMap>(new LandmarkMap);
  map->vocabulary_hash_ = vocabulary_hash;
  map->descriptor_model_id_ = model_id ? model_id : points.empty() ? 0 : points.front().appearance.descriptor.model_id();
  map->descriptor_dimensions_ = dimensions ? dimensions : points.empty() ? (map->descriptor_model_id_ ? 256 : 32) : points.front().appearance.descriptor.dimensions();
  if (map->descriptor_model_id_ ? (map->descriptor_dimensions_ != 64 && map->descriptor_dimensions_ != 256) : map->descriptor_dimensions_ != 32)
    throw std::invalid_argument("Invalid map descriptor dimensions");
  if (map->descriptor_model_id_ && vocabulary_hash) throw std::invalid_argument("Learned map cannot use ORB vocabulary");
  map->max_points_ = max_points;
  map->points_ = std::move(points);
  std::unordered_set<uint64_t> ids;
  map->descriptors_.reserve(map->points_.size());
  for (uint32_t i = 0; i < map->points_.size(); ++i) {
    auto& point = map->points_[i];
    if (point.appearance.descriptor.model_id() != map->descriptor_model_id_ || point.appearance.descriptor.dimensions() != map->descriptor_dimensions_) throw std::invalid_argument("Mixed map descriptor domains");
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) || !std::isfinite(point.position.z) || !point.has_appearance ||
        !ids.insert(point.id).second)
      throw std::invalid_argument("Invalid or duplicate submap landmark");
    if (!std::isfinite(point.depth_quality) || point.depth_quality < 0 || point.depth_quality > 1) throw std::invalid_argument("Invalid BA depth quality");
    if (point.geometry) point.geometry->validate();
    const auto& stereo = point.stereo;
    if (!std::isfinite(stereo.sigma()) || stereo.sigma() < 0) throw std::invalid_argument("Invalid stereo pixel noise");
    if (stereo.valid()) {
      if (!stereo.anchor_from_camera().isValid() || !std::isfinite(stereo.pixel().x) || !std::isfinite(stereo.pixel().y) || !std::isfinite(stereo.pixel().z) ||
          stereo.pixel().x <= stereo.pixel().y || stereo.calibration()[0] <= 0 || stereo.calibration()[1] <= 0 || stereo.calibration()[4] <= 0 ||
          !std::all_of(stereo.calibration().begin(), stereo.calibration().end(), [](float x) { return std::isfinite(x); }))
        throw std::invalid_argument("Invalid original stereo observation");
    }
    if (!vocabulary_hash) {
      point.appearance.word_id = -1;
      point.appearance.tree_node = bow::FeatureBlock::invalid_node_id;
    }
    validate_appearance(point.appearance);
    map->descriptors_.push_back(i);
  }
  if (!vocabulary_hash) return map;
  std::stable_sort(map->descriptors_.begin(), map->descriptors_.end(),
                   [&](const auto& a, const auto& b) { return map->points_[a].appearance.tree_node < map->points_[b].appearance.tree_node; });
  for (uint32_t i = 0; i < map->descriptors_.size(); ++i) {
    const auto ref = map->descriptors_[i];
    const auto node = map->points_[ref].appearance.tree_node;
    if (map->groups_.empty() || map->groups_.back().node_id != node) map->groups_.push_back({node, i, i});
    ++map->groups_.back().end;
  }
  return map;
}

std::shared_ptr<const LandmarkMap> LandmarkMap::from_frame(const bow::FeatureBlock& frame, uint32_t max_points, const camera::Perspective* camera,
                                                           const std::vector<double>& pixel_variances) {
  if (!max_points || max_points > point_capacity) throw std::invalid_argument("Invalid node point capacity");
  frame.validate();
  const auto measurement_frame = camera ? camera::stereo_frame(*camera, frame.keypoints, frame.points, pixel_variances, false, 1) : nullptr;
  std::vector<Landmark> points;
  points.reserve(frame.points.size());
  for (uint32_t row = 0; row < frame.keypoints.size(); ++row) {
    const auto& xyz = frame.points[row];
    if (!std::isfinite(xyz.x) || !std::isfinite(xyz.y) || !std::isfinite(xyz.z)) continue;
    Landmark point;
    point.id = uint64_t(frame.feature_ids[row]) + 1;  // Node-local stable identity, independent of sorted row.
    point.position = xyz;
    point.has_appearance = true;
    auto& sample = point.appearance;
    sample.tree_node = frame.tree_node(row);
    sample.word_id = frame.word_id(row) > 0 ? frame.word_id(row) : -1;
    // Neutral initialization, not a calibrated confidence or ORB-quality claim.
    sample.quality = 0.5f;
    if (camera) {
      const auto pixel = frame.keypoints[row].pt;
      if (!camera->isInImage(pixel.x, pixel.y)) continue;
      sample.ray = observation_ray(*camera, pixel);
      const camera::StereoObservation observation{measurement_frame, row};
      if (observation.valid()) point.stereo = observation;
      // Preserve the matcher's finite weak-depth seeds too. Geometry-update
      // reliability is checked later using joint observations/covariance.
      if (camera->focalXBaseline() > 0) {
        const auto measured = point.stereo.valid() ? point.stereo.measurement() : camera::stereo_pixel(*camera, frame.keypoints[row], xyz, 1);
        if (!solver::Triangulator::stereo_seed_valid(measurement_frame->calibration, measured)) continue;
      }
    }
    if (point.stereo.valid()) point.geometry = PointSummary::from_observation(point.stereo, StateGroupd::identity(), Vec3_t<double>(xyz.x, xyz.y, xyz.z));
    sample.descriptor = feature::LocalDescriptor::from_row(frame.descriptors, int(row), frame.descriptor_model_id);
    points.push_back(point);
  }
  retain_best(points, max_points);
  return create(frame.vocabulary_hash, std::move(points), max_points, frame.descriptor_model_id, frame.descriptors.cols);
}

std::vector<bow::WordCount> LandmarkMap::retrieval_words() const {
  std::map<int, uint32_t> counts;
  for (const auto& point : points_) {
    const int word = point.appearance.word_id;
    if (word > 0) counts[word] += word_credits;
  }
  std::vector<bow::WordCount> result;
  result.reserve(counts.size());
  for (const auto& item : counts) result.push_back({item.first, item.second});
  return result;
}

size_t LandmarkMap::memory_size() const {
  size_t appearance_bytes = 0;
  for (const auto& p : points_) appearance_bytes += p.appearance.descriptor.dynamic_bytes();
  std::vector<const camera::StereoFrame*> frames;
  frames.reserve(points_.size());
  for (const auto& point : points_) frames.push_back(point.stereo.frame.get());
  return appearance_bytes + camera::stereo_frame_memory(std::move(frames)) + sizeof(*this) + points_.capacity() * sizeof(Landmark) +
         descriptors_.capacity() * sizeof(uint32_t) + groups_.capacity() * sizeof(bow::FeatureGroup);
}

}  // namespace sapphire::backend::featmap
