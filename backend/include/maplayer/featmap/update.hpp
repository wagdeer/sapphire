#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <backend/registration/matcher/match.hpp>


namespace sapphire::backend::featmap {

// Loop image consistency uses primary-camera pixels and ORB octave variance.
// Existing point positions are never changed by submap appearance maintenance.
struct GeometryOptions {
  float max_pixel_error = 3;

  float pyramid_scale = 1.2f;
  int pyramid_levels = 8;
  float pixel_scale = 1;  // Extraction-image pixels to the stored query image.
  // Both zero: pose was verified independently; validate each point without
  // requiring natural features to establish the pose again.
  uint32_t min_seed_support = 6;
  uint32_t min_support = 8;
  uint32_t max_pairs = 2048;
};

enum class PointCheck : uint8_t { unmatched, compatible, conflict, unavailable };
enum class UpdateStatus { ready, insufficient_support, inconsistent_geometry, insufficient_coverage, budget_exceeded, vocabulary_mismatch };

struct UpdateStats {
  uint32_t checked = 0;
  uint32_t compatible = 0;
  uint32_t conflicts = 0;
  uint32_t unavailable = 0;
  uint32_t projection_conflicts = 0;
  uint32_t seed_projection_conflicts = 0;
  uint32_t seed_support = 0;
  uint32_t seed_ray_checks = 0;
  uint32_t seed_ray_conflicts = 0;
  uint32_t seed_unavailable = 0;
  uint32_t image_cells = 0;
  uint32_t appended = 0;
  uint32_t replaced = 0;
  double elapsed_ms = 0;
};

// Preparation result, not a committed loop. The transaction owner checks source
// snapshots and prepares content at the final pose before committing.
// Appearances change only in a fully accepted private draft; point positions stay fixed.
class NodeUpdate {
 public:
  UpdateStatus status() const { return status_; }
  const UpdateStats& stats() const { return stats_; }
  const std::vector<PointCheck>& query_checks() const { return query_checks_; }
  const std::shared_ptr<const LandmarkMap>& draft() const { return draft_; }

 private:
  friend NodeUpdate prepare_node_update(std::shared_ptr<const LandmarkMap>, const VisualFrame&, const std::shared_ptr<const bow::FeatureBlock>&, const StateGroupf&, const std::vector<Match>&,
                                          const std::vector<Match>&, const std::vector<float>&, const GeometryOptions&, const AppearanceOptions&);
  UpdateStatus status_ = UpdateStatus::insufficient_support;
  UpdateStats stats_;
  std::vector<PointCheck> query_checks_;
  std::shared_ptr<const LandmarkMap> draft_;
};

// PnP seeds must already be verified; projected associations are provisional.
// Query 3D is in query BODY coordinates, source 3D in map/anchor coordinates.
// Quality is indexed by the sorted FeatureBlock row, exactly like Match::feature.
// Checks existing point projections at fixed PnP/PGO poses. No historical rays,
// point-position updates, BA, new-point insertion or second PnP.
// Conflicting query rows must not be reinserted as unmatched new points.
NodeUpdate prepare_node_update(std::shared_ptr<const LandmarkMap> map, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_map,
                                 const std::vector<Match>& verified_seeds, const std::vector<Match>& projected, const std::vector<float>& observation_quality,
                                 const GeometryOptions& geometry = {}, const AppearanceOptions& appearance = {});

}  // namespace sapphire::backend::featmap
