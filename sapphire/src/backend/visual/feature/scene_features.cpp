#include "backend/visual/feature/scene_features.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>


namespace sapphire::mapping::scene {
namespace {
class KeypointGrid2D {
 public:
  KeypointGrid2D(const std::vector<cv::KeyPoint>& keypoints) {
    points_.reserve(keypoints.size());
    for (std::size_t i = 0; i < keypoints.size(); ++i) {
      points_.push_back(keypoints[i].pt);
      if (std::isfinite(keypoints[i].pt.x) && std::isfinite(keypoints[i].pt.y)) {
        const int x = cellCoordinate(keypoints[i].pt.x);
        const int y = cellCoordinate(keypoints[i].pt.y);
        cells_[cellKey(x, y)].push_back(i);
        minCellX_ = std::min(minCellX_, x);
        minCellY_ = std::min(minCellY_, y);
        maxCellX_ = std::max(maxCellX_, x);
        maxCellY_ = std::max(maxCellY_, y);
      }
    }
  }

  bool collect_indices(const cv::Point2f& point, float radius, std::size_t max_visits, std::vector<std::size_t>& output, std::size_t& visits) const {
    output.clear();
    visits = 0;
    if (cells_.empty() || radius < 0 || !std::isfinite(radius) || !std::isfinite(point.x) || !std::isfinite(point.y)) return true;
    const int first_x = std::max(minCellX_, cellCoordinate(point.x - radius));
    const int last_x = std::min(maxCellX_, cellCoordinate(point.x + radius));
    const int first_y = std::max(minCellY_, cellCoordinate(point.y - radius));
    const int last_y = std::min(maxCellY_, cellCoordinate(point.y + radius));
    const double radius_squared = double(radius) * radius;
    for (std::int64_t y = first_y; y <= last_y; ++y) {
      for (std::int64_t x = first_x; x <= last_x; ++x) {
        const auto cell = cells_.find(cellKey(static_cast<int>(x), static_cast<int>(y)));
        if (cell == cells_.end()) continue;
        for (const auto index : cell->second) {
          if (visits == max_visits) {
            output.clear();
            return false;
          }
          ++visits;
          const double dx = double(points_[index].x) - point.x;
          const double dy = double(points_[index].y) - point.y;
          if (dx * dx + dy * dy <= radius_squared) output.push_back(index);
        }
      }
    }
    return true;
  }


 private:
  static constexpr float kCellSize = 32.0f;

  static int cellCoordinate(float coordinate) {
    const double cell = std::floor(static_cast<double>(coordinate) / kCellSize);
    if (cell <= std::numeric_limits<int>::min()) {
      return std::numeric_limits<int>::min();
    }
    if (cell >= std::numeric_limits<int>::max()) {
      return std::numeric_limits<int>::max();
    }
    return static_cast<int>(cell);
  }

  static std::uint64_t cellKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
  }

  std::vector<cv::Point2f> points_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells_;
  int minCellX_ = std::numeric_limits<int>::max();
  int minCellY_ = std::numeric_limits<int>::max();
  int maxCellX_ = std::numeric_limits<int>::min();
  int maxCellY_ = std::numeric_limits<int>::min();
};

constexpr uint32_t invalid_index = std::numeric_limits<uint32_t>::max();

void validate_ray(const ObservationRay& ray) {
  const Eigen::Vector3f origin(ray.origin.x, ray.origin.y, ray.origin.z), direction(ray.direction.x, ray.direction.y, ray.direction.z);
  if (!std::isfinite(ray.pixel_angle) || ray.pixel_angle < 0 || ray.pixel_angle > 1 || !origin.allFinite() || !direction.allFinite() ||
      (ray.pixel_angle > 0 && std::abs(direction.squaredNorm() - 1) > 1e-3f))
    throw std::invalid_argument("Invalid scene observation ray");
}

void validate_appearance(const Appearance& appearance) {
  if (!std::isfinite(appearance.quality) || appearance.quality < 0 || appearance.quality > 1 || appearance.word_id < -1)
    throw std::invalid_argument("Invalid scene appearance");
  validate_ray(appearance.ray);
}

bool same_ray(const ObservationRay& a, const ObservationRay& b) {
  return a.origin == b.origin && a.direction == b.direction && a.pixel_angle == b.pixel_angle;
}

void validate_options(float ratio, int max_distance) {
  if (!std::isfinite(ratio) || ratio <= 0 || ratio > 1 || max_distance < 0 || max_distance > 256)
    throw std::invalid_argument("Invalid scene match thresholds");
}

bool valid_pixel(const cv::KeyPoint& keypoint) { return std::isfinite(keypoint.pt.x) && std::isfinite(keypoint.pt.y); }

struct Neighbor {
  uint32_t id = invalid_index;
  uint32_t appearance = 0;
  int distance = 257;
};

struct Nearest {
  Neighbor first, second;

  void add(uint32_t id, int distance, uint32_t appearance) {
    Neighbor candidate{id, appearance, distance};
    if (id == first.id) {
      if (distance < first.distance) first = candidate;
      return;
    }
    if (id == second.id) {
      if (distance < second.distance) second = candidate;
      if (second.distance < first.distance) std::swap(first, second);
      return;
    }
    if (distance < first.distance) {
      second = first;
      first = candidate;
    } else if (distance < second.distance) {
      second = candidate;
    }
  }

  bool accepts(float ratio, int max_distance, bool allow_singleton) const {
    if (first.id == invalid_index || first.distance > max_distance) return false;
    if (second.id == invalid_index) return allow_singleton;
    return first.distance < second.distance && first.distance <= ratio * second.distance;
  }
};

void finish(MatchResult& result, uint32_t min_matches) {
  result.status = result.pairs.size() >= min_matches ? MatchStatus::success : MatchStatus::insufficient_matches;
}

MatchResult budget_result(MatchResult result) {
  result.pairs.clear();
  result.status = MatchStatus::budget_exceeded;
  return result;
}
}  // namespace

ObservationRay observation_ray(const UcmCameraModel& camera, cv::Point2f pixel, const Eigen::Isometry3f& anchor_from_body) {
  if (!camera.camera_to_base().matrix().allFinite() || !anchor_from_body.matrix().allFinite() || !std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
      !camera.contains(pixel.x, pixel.y))
    throw std::invalid_argument("Invalid camera observation");
  const auto bearing = camera.unproject(Eigen::Vector2f(pixel.x, pixel.y));
  if (!bearing) throw std::invalid_argument("Invalid observation bearing");
  const auto pose = (anchor_from_body * camera.camera_to_base());
  const Eigen::Vector3f direction = (pose.linear() * *bearing).normalized();
  const auto center = pose.translation();
  return {{center.x(), center.y(), center.z()}, {direction.x(), direction.y(), direction.z()}, float(1.0 / std::min(camera.fx(), camera.fy()))};
}

AppearanceUpdate update_appearance(Landmark& point, const Appearance& candidate, bool geometry_verified, const AppearanceOptions& options) {
  if (point.appearance_count > appearance_capacity || !std::isfinite(options.min_quality) || options.min_quality < 0 || options.min_quality > 1)
    throw std::invalid_argument("Invalid appearance capacity or quality threshold");
  validate_appearance(candidate);
  if (point.appearance_count) validate_appearance(point.appearances[0]);
  if (!geometry_verified) return AppearanceUpdate::unverified;
  if (candidate.quality < options.min_quality) return AppearanceUpdate::discarded;
  const bool initialized = point.appearance_count != 0;
  const auto& previous = point.appearances[0];
  if (initialized && candidate.descriptor == previous.descriptor && candidate.tree_node == previous.tree_node &&
      candidate.word_id == previous.word_id && candidate.quality == previous.quality && same_ray(candidate.ray, previous.ray))
    return AppearanceUpdate::discarded;
  point.appearances[0] = candidate;
  point.appearance_count = 1;
  return initialized ? AppearanceUpdate::replaced : AppearanceUpdate::appended;
}

std::shared_ptr<const FeatureMap> FeatureMap::create(uint64_t vocabulary_hash, std::vector<Landmark> points, uint32_t max_points) {
  if (max_points == 0 || max_points > point_capacity || points.size() > max_points) throw std::invalid_argument("Invalid scene point capacity");
  auto map = std::shared_ptr<FeatureMap>(new FeatureMap);
  map->vocabulary_hash_ = vocabulary_hash;
  map->max_points_ = max_points;
  map->points_ = std::move(points);
  std::unordered_set<uint64_t> ids;
  map->descriptors_.reserve(map->points_.size() * appearance_capacity);
  for (uint32_t i = 0; i < map->points_.size(); ++i) {
    auto& point = map->points_[i];
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) || !std::isfinite(point.position.z) || point.appearance_count == 0 ||
        point.appearance_count > appearance_capacity || !ids.insert(point.id).second)
      throw std::invalid_argument("Invalid or duplicate scene landmark");
    for (uint32_t j = 0; j < point.appearance_count; ++j) {
      if (!vocabulary_hash) {
        point.appearances[j].word_id = -1;
        point.appearances[j].tree_node = features::FeatureBlock::invalid_node_id;
      }
      validate_appearance(point.appearances[j]);
      map->descriptors_.push_back({i, j});
    }
  }
  if (!vocabulary_hash) return map;
  std::stable_sort(map->descriptors_.begin(), map->descriptors_.end(), [&](const auto& a, const auto& b) {
    return map->points_[a.landmark].appearances[a.appearance].tree_node < map->points_[b.landmark].appearances[b.appearance].tree_node;
  });
  for (uint32_t i = 0; i < map->descriptors_.size(); ++i) {
    const auto ref = map->descriptors_[i];
    const auto node = map->points_[ref.landmark].appearances[ref.appearance].tree_node;
    if (map->groups_.empty() || map->groups_.back().node_id != node) map->groups_.push_back({node, i, i});
    ++map->groups_.back().end;
  }
  return map;
}

MatchResult match_features(const FeatureMap& map, const features::FeatureBlock& query, const MatchOptions& options) {
  validate_options(options.ratio, options.max_distance);
  if (options.min_matches == 0) throw std::invalid_argument("Zero scene match count");
  query.validate();
  MatchResult result;
  if (map.vocabulary_hash() != query.vocabulary_hash) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  if (map.points().size() < options.min_matches || query.keypoints.size() < options.min_matches) return result;
  std::vector<Nearest> queries(query.keypoints.size()), reverse(map.points().size());
  size_t a = 0, b = 0;
  while (a < map.groups().size() && b < query.groups.size()) {
    const auto& left = map.groups()[a];
    const auto& right = query.groups[b];
    if (left.node_id == features::FeatureBlock::invalid_node_id || right.node_id == features::FeatureBlock::invalid_node_id) break;
    if (left.node_id < right.node_id) {
      ++a;
      continue;
    }
    if (right.node_id < left.node_id) {
      ++b;
      continue;
    }
    for (uint32_t q = right.begin; q < right.end; ++q) {
      if (!valid_pixel(query.keypoints[q])) continue;
      const uint64_t count = left.end - left.begin;
      if (count > options.max_comparisons - result.comparisons) return budget_result(std::move(result));
      features::DescriptorQuery descriptor(query.descriptors.ptr(q));
      for (uint32_t d = left.begin; d < left.end; ++d) {
        const auto ref = map.descriptors()[d];
        const int distance = descriptor.distance(map.points()[ref.landmark].appearances[ref.appearance].descriptor.data());
        ++result.comparisons;
        queries[q].add(ref.landmark, distance, ref.appearance);
        reverse[ref.landmark].add(q, distance, ref.appearance);
      }
    }
    ++a;
    ++b;
  }
  for (uint32_t q = 0; q < queries.size(); ++q) {
    const auto& nearest = queries[q];
    // Like vslam BowTree, a singleton group uses the absolute distance gate.
    // It is only a proposed association; PnP still establishes geometry.
    if (!nearest.accepts(options.ratio, options.max_distance, true)) continue;
    const auto& other = reverse[nearest.first.id];
    // Resolve competition after all distances have been evaluated. Never remove a
    // used candidate before NNDR, which could artificially increase the second distance.
    if (other.first.id != q || other.first.distance == other.second.distance) continue;
    result.pairs.push_back({nearest.first.id, q, nearest.first.appearance, nearest.first.distance});
  }
  finish(result, options.min_matches);
  return result;
}

MatchResult project_unmatched(const FeatureMap& map, const features::FeatureBlock& features, const UcmCameraModel& camera,
                              const Eigen::Isometry3f& query_from_map, const std::vector<Match>& verified_seeds, const ProjectionOptions& options) {
  validate_options(1.0f, options.max_distance);
  if (!std::isfinite(options.radius) || options.radius <= 0 || options.radius > 32 || options.min_seed_matches == 0 ||
      !query_from_map.matrix().allFinite())
    throw std::invalid_argument("Invalid monocular projection query");
  if (camera.width() <= 0 || camera.height() <= 0 || !camera.camera_to_base().matrix().allFinite())
    throw std::invalid_argument("Invalid projection camera");
  const KeypointGrid2D grid(features.keypoints);
  features.validate();
  MatchResult result;
  if (!features.is_raw() && features.vocabulary_hash != map.vocabulary_hash()) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  std::vector<uint8_t> used_points(map.points().size()), used_queries(features.keypoints.size());
  for (const auto& seed : verified_seeds) {
    if (seed.landmark >= used_points.size() || seed.feature >= used_queries.size() ||
        seed.appearance >= map.points()[seed.landmark].appearance_count || used_points[seed.landmark] || used_queries[seed.feature] ||
        !valid_pixel(features.keypoints[seed.feature]))
      throw std::invalid_argument("Invalid or nonunique projection seed");
    used_points[seed.landmark] = used_queries[seed.feature] = 1;
  }
  if (verified_seeds.size() < options.min_seed_matches) return result;

  const auto camera_from_map = (camera.camera_to_base().inverse() * query_from_map);
  std::vector<Nearest> forward(map.points().size()), reverse(features.keypoints.size());
  std::vector<size_t> indices;
  for (uint32_t p = 0; p < map.points().size(); ++p) {
    if (used_points[p]) continue;
    if (result.projected_points == options.max_projected_points) return budget_result(std::move(result));
    ++result.projected_points;
    const auto& point = map.points()[p];
    if (!point.has_position) continue;
    const Eigen::Vector3f xyz = (camera_from_map * Eigen::Vector3f(point.position.x, point.position.y, point.position.z));
    const auto pixel = camera.project(xyz);
    if (!pixel || !camera.contains(pixel->x(), pixel->y())) continue;
    size_t visits = 0;
    const bool complete = grid.collect_indices(cv::Point2f(pixel->x(), pixel->y()), options.radius,
                                               options.max_spatial_visits - result.spatial_visits, indices, visits);
    result.spatial_visits += visits;
    if (!complete) return budget_result(std::move(result));
    for (const auto q : indices) {
      if (used_queries[q] || !valid_pixel(features.keypoints[q]) || !camera.contains(features.keypoints[q].pt.x, features.keypoints[q].pt.y))
        continue;
      if (point.appearance_count > options.max_comparisons - result.comparisons) return budget_result(std::move(result));
      features::DescriptorQuery descriptor(features.descriptors.ptr(static_cast<int>(q)));
      int distance = 257;
      uint32_t appearance = 0;
      for (uint32_t d = 0; d < point.appearance_count; ++d) {
        const int candidate = descriptor.distance(point.appearances[d].descriptor.data());
        ++result.comparisons;
        if (candidate < distance) {
          distance = candidate;
          appearance = d;
        }
      }
      forward[p].add(static_cast<uint32_t>(q), distance, appearance);
      reverse[q].add(p, distance, appearance);
    }
  }
  for (uint32_t p = 0; p < forward.size(); ++p) {
    const auto& nearest = forward[p];
    if (!nearest.accepts(1.0f, options.max_distance, true)) continue;
    const auto& other = reverse[nearest.first.id];
    if (other.first.id != p || !other.accepts(1.0f, options.max_distance, true)) continue;
    result.pairs.push_back({p, nearest.first.id, nearest.first.appearance, nearest.first.distance});
  }
  finish(result, 1);
  return result;
}

}  // namespace sapphire::mapping::scene
