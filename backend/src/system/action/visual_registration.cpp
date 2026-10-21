#include <backend/registration/matcher/match.hpp>
#include <backend/registration/matcher/descriptor_match.hpp>
#include <backend/registration/matcher/projection_match.hpp>
#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include "backend/maplayer/featmap/refinement.hpp"
#include "backend/solver/ba_solver.hpp"
#include "backend/sensor/camera/utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <backend/system/action/visual_registration.hpp>
#include <backend/tools/timer.hpp>
#include <stdexcept>

#include "backend/maplayer/node.hpp"
#include "backend/registration/descriptor/distance.hpp"

namespace sapphire::backend {

double VisualRegistration::COVARIANCE_LINEAR_EPSILON = 1e-8;
double VisualRegistration::COVARIANCE_ANGULAR_EPSILON = 3e-8;

VisualRegistration::VisualRegistration(const ParametersMap &parameters)
    : VisualRegistration(RegistrationParams{}.withOverrides(parameters), FeatureParams{}.withOverrides(parameters), PreprocessParams{}.withOverrides(parameters)) {}
VisualRegistration::~VisualRegistration() = default;

size_t VisualRegistration::memory_size() const { return sizeof(*this) + local_matcher_.buffer_bytes() + pixel_variances_.capacity() * sizeof(double); }

std::vector<featmap::Match> VisualRegistration::match_local(const featmap::LandmarkMap &map, const bow::FeatureBlock &query, int max_distance, float ratio) const {
  if(map.descriptor_model_id() || query.descriptor_model_id) {
    auto options = matching_options();
    options.ratio = ratio;
    options.max_distance = max_distance;
    return xfeat_matcher_ ? xfeat_matcher_->match(map,query,options).pairs : featmap::l2_knn(map,query,options).pairs;
  }
  return local_matcher_.match(map, query, max_distance, ratio);
}

featmap::MatchResult VisualRegistration::associate_at_pose(const featmap::LandmarkMap& map, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& features, const StateGroupf& query_from_map, std::vector<uint32_t>& rejected_rows,
    const std::vector<featmap::Match>& seeds) const {
  featmap::ProjectionOptions options;
  options.min_seed_matches = 0;
  auto matches = featmap::project_unmatched(map, query, features, query_from_map, seeds, options);
  rejected_rows.clear();
  const auto& camera = query.perspectives().front();
  const auto camera_from_map = camera.cameraToBase().inverse().compose(query_from_map);
  matches.pairs.erase(std::remove_if(matches.pairs.begin(), matches.pairs.end(), [&](const auto& match) {
    const auto& point = map.points().at(match.landmark).position;
    const auto xyz = camera_from_map.transformPoint({point.x, point.y, point.z});
    const auto pixel = camera.project(cv::Point3d(xyz.x(), xyz.y(), xyz.z()));
    const auto& observed = features->keypoints.at(match.feature);
    if (observed.octave < 0 || size_t(observed.octave) >= pixel_variances_.size())
      throw std::invalid_argument("Invalid independently placed feature octave");
    if (xyz.z() > 0 && pixel) {
      const auto error = *pixel - cv::Point2d(observed.pt);
      if (error.dot(error) <= 5.99146 * pixel_variances_[observed.octave]) return false;
    }
    rejected_rows.push_back(match.feature);
    return true;
  }), matches.pairs.end());
  if (matches.status == featmap::MatchStatus::success && matches.pairs.empty()) matches.status = featmap::MatchStatus::insufficient_matches;
  return matches;
}

VisualRegistration::VisualRegistration(const RegistrationParams& registration, const FeatureParams& feature, const PreprocessParams& preprocess, std::shared_ptr<feature::RknnModels> models)
    : config_(registration) {
  if (feature.uses_xfeat()) {
    if (!models) models = std::make_shared<feature::RknnModels>(feature);
    models->validate(feature); xfeat_matcher_ = models->xfeat_matcher();
  }
  registration.validate();
  feature.validate();
  preprocess.validate();
  pixel_scale_ = float(preprocess.feature_image_decimation) / preprocess.storage_image_decimation;
  pixel_variances_.resize(feature.orb_pyramid_levels);
  double scale = double(preprocess.feature_image_decimation) / preprocess.storage_image_decimation;
  for (auto &variance : pixel_variances_) {
    variance = scale * scale;
    if (!std::isfinite(variance) || variance <= 0) throw std::invalid_argument("Invalid PnP pixel variance");
    scale *= feature.orb_pyramid_scale;
  }
}

Registration VisualRegistration::computeTransformation(const Node &source, const Node &target, StateGroupf guess, RegistrationTrace *trace, std::optional<uint64_t> pnp_seed) const {
  return source.has_map_features()
             ? verify_submap(source.map_features().get(), target.loop_features().get(), target.visualFrame().perspectives(), guess, trace, pnp_seed)
             : verify_features(source.loop_features().get(), target.loop_features().get(), target.visualFrame().perspectives(), guess, trace, pnp_seed);
}

Registration VisualRegistration::verify_features(const bow::FeatureBlock *source, const bow::FeatureBlock *target,
                                                 const std::vector<camera::Perspective> &cameras, const StateGroupf &guess, RegistrationTrace *trace, std::optional<uint64_t> pnp_seed) const {
  return verify_data(source, nullptr, target, cameras, guess, trace, nullptr, pnp_seed);
}

Registration VisualRegistration::verify_submap(const featmap::LandmarkMap *source, const bow::FeatureBlock *target,
                                               const std::vector<camera::Perspective> &cameras, const StateGroupf &guess, RegistrationTrace *trace, std::optional<uint64_t> pnp_seed) const {
  if (!source) {
    if (trace) *trace = {};
    Registration result;
    result.failure = "Missing submap feature bank";
    return result;
  }
  return verify_data(nullptr, source, target, cameras, guess, trace, nullptr, pnp_seed);
}

Registration VisualRegistration::verify_submap_matches(const featmap::LandmarkMap &source, const bow::FeatureBlock &target,
    const std::vector<camera::Perspective> &cameras, const std::vector<featmap::Match> &matches, RegistrationTrace *trace, std::optional<uint64_t> pnp_seed) const {
  return verify_data(nullptr, &source, &target, cameras, {}, trace, &matches, pnp_seed);
}

Registration VisualRegistration::verify_data(const bow::FeatureBlock *source, const featmap::LandmarkMap *map, const bow::FeatureBlock *target,
    const std::vector<camera::Perspective> &cameras, const StateGroupf &guess, RegistrationTrace *trace, const std::vector<featmap::Match> *proposals, std::optional<uint64_t> pnp_seed) const {
  tools::Timer timer;
  if (trace) *trace = {};
  auto result = verify_pnp(source, target, cameras, guess, trace, map, proposals, pnp_seed);
  if (!result.pose.isNull()) {
    for (int i = 0; i < 6; ++i)
      (*result.covariance)(i, i) = std::max((*result.covariance)(i, i),
          i < 3 ? COVARIANCE_LINEAR_EPSILON : COVARIANCE_ANGULAR_EPSILON);
  }
  if (trace) trace->elapsed_seconds = timer.lap_seconds();
  return result;
}

Registration VisualRegistration::verify_pnp(const bow::FeatureBlock *source_block, const bow::FeatureBlock *target_block,
                                        const std::vector<camera::Perspective> &cameras, const StateGroupf &guess, RegistrationTrace *trace,
                                        const featmap::LandmarkMap *map, const std::vector<featmap::Match> *proposals, std::optional<uint64_t> pnp_seed) const {
  Registration registration;
  registration.map_matches = map != nullptr;
  std::vector<featmap::Match> pairs;
  std::vector<int> match_ids, inlier_ids, pnp_inliers;
  int match_count = 0, inlier_count = 0;
  float inlier_ratio = 0, inlier_distribution = 0;
  const auto finish = [&](const StateGroupf& pose) {
    registration.pose = pose;
    if (!pose.isNull()) {
      std::vector<bool> accepted(target_block->keypoints.size(), false);
      for (int index : inlier_ids) {
        registration.inliers.push_back(pairs.at(index));
        accepted.at(pairs.at(index).feature) = true;
      }
      for (size_t i = 0; i < pairs.size(); ++i)
        if (!accepted[pairs[i].feature]) registration.rejected_features.push_back(pairs[i].feature);
      auto& rejected = registration.rejected_features;
      rejected.erase(std::remove_if(rejected.begin(), rejected.end(), [&](uint32_t row) { return accepted.at(row); }), rejected.end());
      std::sort(rejected.begin(), rejected.end());
      rejected.erase(std::unique(rejected.begin(), rejected.end()), rejected.end());
    } else {
      registration.covariance.reset();
    }
    if (trace) {
      trace->match_count = match_count;
      trace->inlier_ratio = inlier_ratio;
      trace->inlier_distribution = inlier_distribution;
      trace->matches = std::move(pairs);
      trace->inliers = std::move(inlier_ids);
    }
    return std::move(registration);
  };
  if ((!source_block && !map) || !target_block) {
    registration.failure = "Missing precomputed feature block";
    return finish({});
  }
  if (cameras.empty() || std::any_of(cameras.begin(), cameras.end(), [](const auto &camera) {
        return !camera.isValidForProjection() || !camera.cameraToBase().isValid() || camera.imageSize().width <= 0 || camera.imageSize().height <= 0;
      })) {
    registration.failure = "Missing target camera calibration";
    return finish({});
  }
  // Matching follows descriptor organization, independently of place retrieval.
  const bool ungrouped_matching = target_block->is_raw();
  const auto options = matching_options();
  if (map) {
    featmap::MatchResult result;
    if (proposals) {
      target_block->validate();

      if (map->descriptor_model_id()!=target_block->descriptor_model_id || (!ungrouped_matching && map->vocabulary_hash() != target_block->vocabulary_hash)) {
        registration.failure = "Node vocabulary mismatch";
        return finish({});
      }
      std::vector<bool> used_point(map->points().size()), used_pixel(target_block->keypoints.size());
      if (proposals->size() > std::min(used_point.size(), used_pixel.size())) {
        registration.failure = "Invalid external correspondence count";
        return finish({});
      }
      for (const auto &p : *proposals) {
        if (p.landmark >= used_point.size() || p.feature >= used_pixel.size() ||
            used_point[p.landmark] || used_pixel[p.feature] || !std::isfinite(p.distance) || p.distance < 0 || p.distance > (map->descriptor_model_id() ? UnitL2MatchPolicy::gate().max_distance : float(options.max_distance))) {
          registration.failure = "Invalid external correspondence identity or distance";
          return finish({});
        }
        used_point[p.landmark] = used_pixel[p.feature] = true;
      }
      result.pairs = *proposals;
      result.status = result.pairs.size() >= options.min_matches ? featmap::MatchStatus::success : featmap::MatchStatus::insufficient_matches;
    } else if (map->descriptor_model_id()) {
      result = xfeat_matcher_ ? xfeat_matcher_->match(*map,*target_block,options) : featmap::l2_knn(*map,*target_block,options);
    } else if (ungrouped_matching) {
      result.pairs = match_local(*map, *target_block, options.max_distance, options.ratio);
      result.status = result.pairs.size() >= options.min_matches ? featmap::MatchStatus::success : featmap::MatchStatus::insufficient_matches;
    } else {
      result = featmap::bow_knn(*map, *target_block, options);
    }
    // RANSAC sampling must not depend on query storage order (raw vs word-grouped).
    if (ungrouped_matching)
      std::sort(result.pairs.begin(), result.pairs.end(),
                [&](const auto &a, const auto &b) { return map->points()[a.landmark].id < map->points()[b.landmark].id; });
    if (trace) trace->bow_comparisons = result.comparisons;
    if (trace) trace->bow_comparison_bound = options.max_comparisons;
    match_count = result.pairs.size();
    pairs = std::move(result.pairs);
    if (result.status != featmap::MatchStatus::success) {
      registration.failure = result.status == featmap::MatchStatus::vocabulary_mismatch ? "Node vocabulary mismatch"
                         : result.status == featmap::MatchStatus::budget_exceeded   ? "Node matching budget exceeded"
                                                                                  : fmt::format("Insufficient submap correspondences: matched={} required={}", match_count, config_.min_inliers);
      return finish({});
    }
  } else {
    bow::MatchResult result;
    if (ungrouped_matching) {
      auto bank = featmap::LandmarkMap::from_frame(*source_block, featmap::point_capacity);
      std::vector<uint32_t> original_rows(source_block->feature_ids.size());
      for (uint32_t i = 0; i < original_rows.size(); ++i) original_rows[source_block->feature_ids[i]] = i;
      for (auto pair : match_local(*bank, *target_block, options.max_distance, options.ratio))
        pairs.push_back({original_rows.at(bank->points()[pair.landmark].id - 1), pair.feature, pair.distance});
      result.status = pairs.size() >= size_t(config_.min_inliers) ? bow::MatchStatus::success : bow::MatchStatus::insufficient_matches;
    } else {
      result = bow::bow_knn(*source_block, *target_block, options);
      const feature::descriptor_matrix<Binary_U8> source_rows(source_block->descriptors), target_rows(target_block->descriptors);
      pairs.reserve(result.pairs.size());
      for (const auto& pair : result.pairs) {
        const feature::descriptor_distance<Binary_U8> matcher(source_rows.row(int(pair.source)));
        pairs.push_back({pair.source, pair.target, float(matcher.distance(target_rows.row(int(pair.target))))});
      }
    }
    if (ungrouped_matching)
      std::sort(pairs.begin(), pairs.end(),
                [&](const auto &a, const auto &b) { return source_block->feature_ids[a.landmark] < source_block->feature_ids[b.landmark]; });
    if (trace) trace->bow_comparison_bound = result.comparison_bound;
    if (trace) trace->bow_comparisons = result.comparisons;
    match_count = pairs.size();
    if (result.status != bow::MatchStatus::success) {
      registration.failure = result.status == bow::MatchStatus::vocabulary_mismatch ? "BoW vocabulary mismatch"
                         : result.status == bow::MatchStatus::comparison_budget ? "BoW comparison budget exceeded"
                                                                                : fmt::format("Insufficient correspondences with valid source 3D: matched={} required={}", match_count, config_.min_inliers);
      return finish({});
    }
  }
  const auto camera_rois = camera::cameraImageRois(cameras);
  std::vector<solver::PnPMatch> observations;
  observations.reserve(pairs.size());
  for (const auto &pair : pairs) {
    const auto &keypoint = target_block->keypoints[pair.feature];
    if (keypoint.octave < 0 || size_t(keypoint.octave) >= pixel_variances_.size()) {
      registration.failure = "Invalid target ORB octave";
      return finish({});
    }
    const int camera_index = camera::cameraIndexForPoint(keypoint.pt, camera_rois);
    if (camera_index < 0) {
      registration.failure = "Target feature outside camera image";
      return finish({});
    }
    cv::Point2f pixel = keypoint.pt - cv::Point2f(camera_rois[camera_index].x, camera_rois[camera_index].y);
    const auto bearing = cameras[camera_index].bearing(pixel);
    if (!bearing || !std::isfinite(bearing->z) || bearing->z <= 0) {
      registration.failure = "Invalid target feature bearing";
      return finish({});
    }
    observations.push_back({map ? map->points()[pair.landmark].position : source_block->points[pair.landmark],
                            {pixel, pixel_variances_.at(keypoint.octave), camera_index}});
  }
  StateGroupf transform;
  const auto initial_pose = guess.isNull() ? StateGroupf::identity() : guess;
  const solver::BASolver::PnPOptions pnp_options{config_.min_inliers, config_.ransac_iterations, config_.refine_iterations, config_.max_reprojection_error_px, pnp_seed};
  auto pnp = solver::BASolver::estimate_motion_pnp(observations, cameras, initial_pose, pnp_options);
  transform = pnp.pose;
  match_ids = std::move(pnp.matches);
  inlier_ids = std::move(pnp.inliers);
  if (trace) trace->inliers_per_camera = std::move(pnp.inliers_per_camera);
  match_count = match_ids.size();
  inlier_count = inlier_ids.size();
  inlier_ratio = target_block->keypoints.empty() ? 0 : float(inlier_count) / target_block->keypoints.size();
  if (transform.isNull() || inlier_count < config_.min_inliers) {
    registration.failure = fmt::format("PnP rejected: inliers={} required={} matches={}", inlier_count, config_.min_inliers, match_count);
    return finish({});
  }
  if (map && cameras.size() == 1) {
    if (trace) trace->pnp_pose = transform;
    pnp_inliers = inlier_ids;
    if (trace) trace->pnp_inliers = pnp_inliers;
    std::vector<featmap::Match> seeds;
    for (int id : pnp_inliers) seeds.push_back(pairs.at(id));
    VisualFrame query;
    query.setPerspectives(cameras);
    // Borrow only for this synchronous call; the caller owns the immutable block.
    const auto query_features = std::shared_ptr<const bow::FeatureBlock>(target_block, [](const bow::FeatureBlock *) {});
    query.share_features(query_features);
    const auto projection_start = std::chrono::steady_clock::now();
    // Fixed-pose association only. The subsequent node BA owns joint pose/point
    // refinement; projection must not introduce a second pose-only solve.
    auto projected = associate_at_pose(*map, query, query_features, transform.inverse(), registration.rejected_features, seeds);
    if (trace) trace->projection_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - projection_start).count();
    registration.projection_complete = projected.status != featmap::MatchStatus::budget_exceeded;
    for (const auto &pair : projected.pairs) {
      const int id = pairs.size();
      inlier_ids.push_back(id);
      if (trace) trace->projected.push_back(id);
      match_ids.push_back(id);
      pairs.push_back(pair);
      const auto& keypoint = target_block->keypoints.at(pair.feature);
      observations.push_back({map->points().at(pair.landmark).position,
                              {keypoint.pt, pixel_variances_.at(keypoint.octave), 0}});
    }
    match_count = match_ids.size();
    inlier_count = inlier_ids.size();

    if (trace) trace->inliers_per_camera = {inlier_count};
    inlier_ratio = float(inlier_count) / target_block->keypoints.size();
  }
  bool stereo_covariance = false;
  if (config_.stereo_refinement_enabled && map && cameras.size() == 1) {
    std::vector<featmap::Match> seeds;
    for (int id : inlier_ids) seeds.push_back(pairs.at(id));
    auto bundle = sapphire::backend::featmap::refine_stereo_pose(*map, *target_block, cameras.front(), seeds, transform, pixel_variances_, config_.min_inliers);
    if (trace) trace->stereo_bundle_observations = bundle.available;
    if (trace) trace->stereo_bundle_ms = bundle.elapsed_ms;
    if (bundle.available >= size_t(config_.min_inliers)) {
      if (bundle.pose.isNull()) {
        registration.failure = "Stereo bundle rejected: " + bundle.failure;
        return finish({});
      }
      std::vector<int> inliers;
      for (int i : bundle.inliers) inliers.push_back(inlier_ids.at(i));
      inlier_ids = std::move(inliers);
      inlier_count = inlier_ids.size();
      if (trace) trace->inliers_per_camera = {inlier_count};
      inlier_ratio = float(inlier_count) / target_block->keypoints.size();
      transform = bundle.pose;
      registration.covariance = bundle.covariance;
      stereo_covariance = true;
      if (trace) trace->stereo_bundle_refined = true;
    }
  }
  if (!stereo_covariance) {
    // Added associations have not estimated this pose. Keep the PnP estimate's
    // conditional covariance until BA replaces both pose and covariance.
    const auto covariance = solver::BASolver::compute_covariance(observations, pnp_inliers.empty() ? inlier_ids : pnp_inliers, cameras, transform);
    if (covariance) registration.covariance = covariance->matrix;
  }
  if (!registration.covariance.has_value()) {
    registration.failure = "PnP pixel geometry is unobservable";
    return finish({});
  }
  cv::Mat distribution;
  if (config_.min_inlier_distribution > 0) distribution.create(inlier_count, 2, CV_32F);
  for (size_t i = 0; i < inlier_ids.size(); ++i) {
    const auto &observation = observations.at(inlier_ids[i]);
    const auto &camera = cameras[observation.target.camera_index];
    if (!distribution.empty()) {
      distribution.at<float>(i, 0) = (observation.target.pixel.x - camera.cx()) / camera.imageSize().width;
      distribution.at<float>(i, 1) = (observation.target.pixel.y - camera.cy()) / camera.imageSize().height;
    }
  }
  if (!distribution.empty()) {
    const cv::PCA pca(distribution, cv::Mat(), cv::PCA::DATA_AS_ROW);
    inlier_distribution = pca.eigenvalues.at<float>(1);
    if (inlier_distribution < config_.min_inlier_distribution) {
      registration.failure = "PnP inlier distribution too narrow";
      return finish({});
    }
  }
  return finish(transform);
}
}  // namespace sapphire::backend
