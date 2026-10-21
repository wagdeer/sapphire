#pragma once
#include <backend/common/common_type.hpp>
#include <backend/maplayer/featmap/refinement.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include <backend/registration/matcher/descriptor_match.hpp>
#include <backend/registration/utils/rknn_models.hpp>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <string>
#include <vector>

namespace sapphire::backend {

class Node;
class VisualFrame;

// A complete verified measurement. Null pose means failure; no usable inliers or
// covariance are published on failure. Rows stay in the original source/query
// snapshots even when Relocalizer returns the inverse pose direction.
struct Registration {
  StateGroupf pose;  // source_from_target, right tangent covariance
  std::optional<Mat66_t<double>> covariance;
  std::vector<featmap::Match> inliers;
  std::vector<uint32_t> rejected_features;  // Query rows rejected during verification.
  bool map_matches = false;  // Match::landmark addresses a LandmarkMap, otherwise a FeatureBlock.
  bool projection_complete = true;
  std::string failure;
};

// Optional per-call diagnostics for statistics and visualizations. Indices refer
// to matches here, never to Registration::inliers. Not consumed by fusion/PGO.
struct RegistrationTrace {
  uint64_t bow_comparison_bound = 0, bow_comparisons = 0;
  std::vector<featmap::Match> matches;
  std::vector<int> pnp_inliers, inliers, projected;
  StateGroupf pnp_pose;  // Native map_from_query, before any requested inversion.
  double projection_ms = 0;
  bool stereo_bundle_refined = false;
  int stereo_bundle_observations = 0;
  double stereo_bundle_ms = 0, elapsed_seconds = 0;
  int match_count = 0;
  float inlier_ratio = 0, inlier_distribution = 0;
  std::vector<int> inliers_per_camera;
};

class VisualRegistration {
 public:
  static double COVARIANCE_LINEAR_EPSILON;
  static double COVARIANCE_ANGULAR_EPSILON;

  VisualRegistration(const ParametersMap &parameters = ParametersMap());
  ~VisualRegistration();

  VisualRegistration(const VisualRegistration &) = delete;
  VisualRegistration &operator=(const VisualRegistration &) = delete;

  explicit VisualRegistration(const RegistrationParams& registration, const FeatureParams& feature, const PreprocessParams& preprocess, std::shared_ptr<feature::RknnModels> models = {});

  MatchOptions matching_options() const {
    return {config_.matching_distance_ratio, config_.matching_max_hamming_distance,
            uint64_t(config_.matching_max_comparisons), uint32_t(config_.min_inliers)};
  }

  float pixel_scale() const { return pixel_scale_; }

  int getMinVisualCorrespondences() const { return config_.min_inliers; }

  const std::vector<double> &pixelVariances() const { return pixel_variances_; }

  // Associate points at an independently verified pose; never estimate a pose.
  featmap::MatchResult associate_at_pose(const featmap::LandmarkMap&, const VisualFrame&,
                                       const std::shared_ptr<const bow::FeatureBlock>&, const StateGroupf& query_from_map,
                                       std::vector<uint32_t>& rejected_rows, const std::vector<featmap::Match>& seeds = {}) const;

  size_t memory_size() const;

  // Optional per-call RANSAC seed for repeatable comparisons; omission retains clock seeding.
  Registration computeTransformation(const Node &from, const Node &to, StateGroupf guess = StateGroupf::identity(),
                                    RegistrationTrace *trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;

  Registration verify_features(const bow::FeatureBlock *source, const bow::FeatureBlock *target, const std::vector<camera::Perspective> &target_cameras,
                              const StateGroupf &guess, RegistrationTrace *trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;

  Registration verify_submap(const featmap::LandmarkMap *source, const bow::FeatureBlock *target, const std::vector<camera::Perspective> &cameras,
                           const StateGroupf &guess, RegistrationTrace *trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;

  // External retrieval may supply one-to-one proposals into these exact immutable
  // snapshots. Geometry, refinement and covariance gates remain identical.
  Registration verify_submap_matches(const featmap::LandmarkMap &source, const bow::FeatureBlock &target, const std::vector<camera::Perspective> &cameras,
                                   const std::vector<featmap::Match> &matches, RegistrationTrace *trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;

  // Owner-thread workspace, reused between calls. The returned matches own their data.
  std::vector<featmap::Match> match_local(const featmap::LandmarkMap &map, const bow::FeatureBlock &query, int max_distance = 50, float ratio = .75f) const;

 private:
  std::shared_ptr<feature::XFeatKnn> xfeat_matcher_;
  Registration verify_data(const bow::FeatureBlock *source, const featmap::LandmarkMap *submap, const bow::FeatureBlock *target,
                          const std::vector<camera::Perspective> &cameras, const StateGroupf &guess, RegistrationTrace *trace,
                          const std::vector<featmap::Match> *proposals, std::optional<uint64_t> pnp_seed) const;

  Registration verify_pnp(const bow::FeatureBlock *source_block, const bow::FeatureBlock *target_block, const std::vector<camera::Perspective> &cameras,
                         const StateGroupf &guess, RegistrationTrace *trace, const featmap::LandmarkMap *map,
                         const std::vector<featmap::Match> *proposals, std::optional<uint64_t> pnp_seed) const;

  // Bounded by the existing per-frame/per-submap feature limits; no global cache.
  mutable BinaryKnn local_matcher_;
  const RegistrationParams config_;
  float pixel_scale_ = 1.f;
  std::vector<double> pixel_variances_;
};

}  // namespace sapphire::backend
