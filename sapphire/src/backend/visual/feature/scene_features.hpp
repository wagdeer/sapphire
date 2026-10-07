#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "common/camera/ucm.hpp"
#include "backend/visual/feature/feature_block.hpp"

namespace sapphire::mapping::scene {

constexpr uint32_t appearance_capacity = 1;
constexpr uint32_t point_capacity = 8192;
// Fixed integer vote per point, unchanged from the historical multi-appearance index.
constexpr uint32_t word_credits = 6;

// A real image measurement expressed in the scene anchor. Zero pixel_angle
// denotes absent measurement provenance, never a synthesized ray from map 3D.
struct ObservationRay {
  cv::Point3f origin{};
  cv::Point3f direction{};
  float pixel_angle = 0;
};
ObservationRay observation_ray(const UcmCameraModel& camera, cv::Point2f pixel,
                               const Eigen::Isometry3f& anchor_from_body = Eigen::Isometry3f::Identity());

struct Appearance {
  std::array<uint8_t, 32> descriptor{};
  uint32_t tree_node = features::FeatureBlock::invalid_node_id;
  int word_id = -1;
  // Comparable, normalized observation quality, supplied by the observation producer.
  // This is not a probability or a geometric confidence estimate.
  float quality = 0;
  // Latest descriptor's actual observation provenance; never a geometry history.
  ObservationRay ray;
};

struct Landmark {
  uint64_t id = 0;
  cv::Point3f position;
  bool has_position = true;  // false: appearance-only observation, never geometric evidence
  std::array<Appearance, appearance_capacity> appearances;
  uint32_t appearance_count = 0;
};

struct AppearanceOptions {
  float min_quality = 0.25f;
};

enum class AppearanceUpdate { unverified, discarded, appended, replaced };

// Call on a private draft, after independent geometric verification. Matching below
// never modifies a landmark. No descriptor is averaged or promoted by matching alone.
// Every qualified, geometrically verified observation replaces the sole appearance.
// Descriptor, quantization and observation provenance are updated together.
AppearanceUpdate update_appearance(Landmark& point, const Appearance& candidate, bool geometry_verified, const AppearanceOptions& options = {});

struct DescriptorRef {
  uint32_t landmark;
  uint32_t appearance;
};

// Immutable matching snapshot. The point budget is explicit; capacity eviction and
// geometric-observation storage belong to the owning scene-node transaction.
class FeatureMap {
 public:
  static std::shared_ptr<const FeatureMap> create(uint64_t vocabulary_hash, std::vector<Landmark> points, uint32_t max_points);
  static std::shared_ptr<const FeatureMap> from_frame(const features::FeatureBlock& frame, uint32_t max_points, const UcmCameraModel* camera = nullptr);
  static std::shared_ptr<const FeatureMap> decode(const void* data, size_t size);
  std::vector<uint8_t> encode() const;
  std::vector<features::WordCount> retrieval_words() const;
  uint32_t max_points() const { return max_points_; }
  size_t memory_size() const;
  uint64_t vocabulary_hash() const { return vocabulary_hash_; }
  const std::vector<Landmark>& points() const { return points_; }
  const std::vector<features::FeatureGroup>& groups() const { return groups_; }
  const std::vector<DescriptorRef>& descriptors() const { return descriptors_; }

 private:
  FeatureMap() = default;
  uint64_t vocabulary_hash_ = 0;
  uint32_t max_points_ = 0;
  std::vector<Landmark> points_;
  std::vector<features::FeatureGroup> groups_;
  std::vector<DescriptorRef> descriptors_;
};

struct Match {
  uint32_t landmark;  // Index in the exact FeatureMap snapshot, not a persistent ID.
  uint32_t feature;   // Row in the query FeatureBlock, not its pre-sort feature ID.
  uint32_t appearance;
  int distance;
};

enum class MatchStatus { success, insufficient_matches, budget_exceeded, vocabulary_mismatch };
struct MatchResult {
  MatchStatus status = MatchStatus::insufficient_matches;
  std::vector<Match> pairs;
  uint64_t comparisons = 0;
  uint64_t spatial_visits = 0;
  uint32_t projected_points = 0;
};

struct MatchOptions {
  float ratio = 0.75f;
  int max_distance = 50;
  uint64_t max_comparisons = 50000;
  uint32_t min_matches = 6;
};

// NNDR compares different landmark IDs inside the selected vocabulary group.
// Singletons use the absolute gate; ties are rejected. No BF fallback or partial result.
MatchResult match_features(const FeatureMap& map, const features::FeatureBlock& query, const MatchOptions& options = {});

struct ProjectionOptions {
  float radius = 7.5f;     // Bounded primary-image search window.
  int max_distance = 100;  // vslam loop projection threshold (initial BoW remains 50).
  uint64_t max_comparisons = 20000;
  uint64_t max_spatial_visits = 20000;
  uint32_t max_projected_points = 2048;
  uint32_t min_seed_matches = 6;
};

// Requires a geometrically verified pose and unique PnP seed associations. Returns
// ONLY provisional new associations, never appearance updates or a loop decision.
// Monocular query; query_from_map maps map coordinates into the query body frame.
MatchResult project_unmatched(const FeatureMap& map, const features::FeatureBlock& features, const UcmCameraModel& camera,
                              const Eigen::Isometry3f& query_from_map, const std::vector<Match>& verified_seeds,
                              const ProjectionOptions& options = {});

}  // namespace sapphire::mapping::scene
