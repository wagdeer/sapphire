#pragma once

#include "backend/common/stereo_frame.hpp"
#include "backend/maplayer/featmap/point_summary.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "backend/common/visual_frame.hpp"
#include "backend/registration/descriptor/feature_block.hpp"
#include "backend/registration/descriptor/local_descriptor.hpp"

namespace sapphire::backend::featmap {

constexpr uint32_t point_capacity = 8192;
// Fixed integer vote per point, unchanged from the historical multi-appearance index.
constexpr uint32_t word_credits = 6;

// A real image measurement expressed in the submap anchor. Zero pixel_angle
// denotes absent measurement provenance, never a synthesized ray from map 3D.
struct ObservationRay {
  cv::Point3f origin{};
  cv::Point3f direction{};
  float pixel_angle = 0;
};
ObservationRay observation_ray(const camera::Perspective& camera, cv::Point2f pixel, const StateGroupf& anchor_from_body = StateGroupf::identity());

struct Appearance {
  feature::LocalDescriptor descriptor;
  uint32_t tree_node = bow::FeatureBlock::invalid_node_id;
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
  Appearance appearance;
  bool has_appearance = false;  // Draft initialization; every committed map point has an appearance.
  camera::StereoObservation stereo;
  std::optional<PointSummary> geometry;
  // Consecutive accepted node updates without a verified association. A hit
  // resets this to zero; newly measured points also start at zero.
  uint32_t miss_count = 0;
  // Reliable BA depth quality: 1 - (3 sigma / closest observation range)^2.
  // Zero means no accepted BA depth estimate; it never excludes a valid seed.
  float depth_quality = 0;
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

// Immutable matching snapshot. The point budget is explicit; capacity eviction and
// geometric-observation storage belong to the owning submap-node transaction.
class LandmarkMap {
 public:
  // Capacity only: activity first, then descriptor separation and BA depth.
  // No lifetime observation-count preference or reserved new-point quota.
  static void retain_best(std::vector<Landmark>& points, uint32_t max_points);
  static std::shared_ptr<const LandmarkMap> create(uint64_t vocabulary_hash, std::vector<Landmark> points, uint32_t max_points, uint64_t model_id = 0, int dimensions = 0);
  static std::shared_ptr<const LandmarkMap> from_frame(const bow::FeatureBlock& frame, uint32_t max_points, const camera::Perspective* camera = nullptr,
                                                      const std::vector<double>& pixel_variances = {});
  static std::shared_ptr<const LandmarkMap> decode(const void* data, size_t size);
  std::vector<uint8_t> encode() const;
  std::vector<bow::WordCount> retrieval_words() const;
  uint32_t max_points() const { return max_points_; }
  size_t memory_size() const;
  uint64_t vocabulary_hash() const { return vocabulary_hash_; }
  int descriptor_dimensions() const { return descriptor_dimensions_; }
  uint64_t descriptor_model_id() const { return descriptor_model_id_; }
  const std::vector<Landmark>& points() const { return points_; }
  const std::vector<bow::FeatureGroup>& groups() const { return groups_; }
  // Descriptor order -> point row (grouped by vocabulary node when present).
  const std::vector<uint32_t>& descriptors() const { return descriptors_; }

 private:
  LandmarkMap() = default;
  uint64_t vocabulary_hash_ = 0;
  uint64_t descriptor_model_id_ = 0;
  uint16_t descriptor_dimensions_ = 32;
  uint32_t max_points_ = 0;
  std::vector<Landmark> points_;
  std::vector<bow::FeatureGroup> groups_;
  std::vector<uint32_t> descriptors_;
};

}  // namespace sapphire::backend::featmap
