#include <Eigen/Eigenvalues>
#include <backend/registration/matcher/descriptor_match.hpp>
#include <backend/registration/matcher/projection_match.hpp>
#include <backend/maplayer/featmap.hpp>

#include "backend/registration/descriptor/landmark_descriptors.hpp"
#include "backend/solver/triangulator.hpp"
#include "backend/maplayer/featmap/internal.hpp"
#include "backend/maplayer/featmap/reprojection.hpp"
#include "backend/tools/logging.hpp"

namespace sapphire::backend::featmap {
using geometry::inlier;

void detail::update_representative_descriptors(std::vector<Landmark>& points, const NodeObservations& bundle,
                                               const std::map<int, StateGroupf>& boundaries) {
  std::map<uint64_t, std::vector<const NodeObservations::Observation*>> tracks;
  for (const auto& o : bundle.observations)
    if (boundaries.count(o.node)) tracks[o.landmark].push_back(&o);
  for (auto& point : points) {
    const auto track = tracks.find(point.id);
    if (track == tracks.end()) continue;
    const auto& candidates = track->second;
    const auto choose = [&](auto format) {
      using Format = decltype(format);
      const auto descriptors = feature::make_descriptor_range<Format>(
          candidates, [](const auto* o) -> const auto& { return o->appearance.descriptor; });
      size_t best = 0;
      float best_median = std::numeric_limits<float>::infinity();
      // Small tracks use stack storage; persistent tracks may exceed the active
      // BA window. Preserve exact medoid and first-sample tie semantics.
      std::array<std::array<float, 8>, 8> distances{};
      std::vector<float> scratch;
      if constexpr (descriptor_traits<Format>::metric == DistanceMetric::l2)
        if (candidates.size() > distances.size()) scratch.resize(candidates.size());
      if (candidates.size() <= distances.size()) {
        for (size_t i = 0; i < candidates.size(); ++i) {
          const feature::descriptor_distance<Format> matcher(descriptors.row(i));
          for (size_t j = i + 1; j < candidates.size(); ++j) distances[i][j] = distances[j][i] = matcher.distance(descriptors.row(j));
        }
      }
      for (size_t i = 0; i < candidates.size(); ++i) {
        const feature::descriptor_distance<Format> matcher(descriptors.row(i));
        float median;
        if (candidates.size() <= distances.size()) {
          auto& row = distances[i];
          std::sort(row.begin(), row.begin() + candidates.size());
          median = row[(candidates.size() - 1) / 2];
        } else if constexpr (descriptor_traits<Format>::metric == DistanceMetric::l2) {
          for (size_t j = 0; j < candidates.size(); ++j) scratch[j] = matcher.distance(descriptors.row(j));
          const auto mid = scratch.begin() + (scratch.size() - 1) / 2;
          std::nth_element(scratch.begin(), mid, scratch.end());
          median = *mid;
        } else {
          // ORB Hamming distance is an integer in [0,256]. Its exact median
          // needs a fixed histogram, not a growing allocation and sort per row.
          std::array<size_t, 257> histogram{};
          for (size_t j = 0; j < candidates.size(); ++j) ++histogram[matcher.distance(descriptors.row(j))];
          const size_t middle = (candidates.size() - 1) / 2;
          size_t cumulative = 0;
          size_t bin = 0;
          for (; bin < histogram.size(); ++bin) {
            cumulative += histogram[bin];
            if (cumulative > middle) break;
          }
          median = float(bin);
        }
        if (median < best_median) {
          best_median = median;
          best = i;
        }
        if (best_median == 0) break;  // Global lower bound; preserve the first tie.
      }
      return best;
    };
    const auto best = feature::dispatch_descriptor(candidates.front()->appearance.descriptor.dimensions(), choose);
    const auto& o = *candidates[best];
    auto sample = o.appearance;
    sample.quality = .5f;
    const auto camera = boundaries.at(o.node).compose(o.stereo.anchor_from_camera());
    const auto& k = o.stereo.calibration();
    Eigen::Vector3f ray((o.stereo.pixel().x - k[2]) / k[0], (o.stereo.pixel().z - k[3]) / k[1], 1);
    ray = camera.rotationMatrix() * ray.normalized();
    const auto origin = camera.translationVector();
    sample.ray = {{origin.x(), origin.y(), origin.z()}, {ray.x(), ray.y(), ray.z()}, 2.f / (k[0] + k[1])};
    point.appearance = sample;
    point.has_appearance = true;
  }
}
std::shared_ptr<const LandmarkMap> representative_descriptors(const LandmarkMap& map, const NodeObservations& bundle,
                                                              const std::map<int, StateGroupf>& boundaries) {
  auto points = map.points();
  detail::update_representative_descriptors(points, bundle, boundaries);
  return LandmarkMap::create(map.vocabulary_hash(), std::move(points), map.max_points(), map.descriptor_model_id(), map.descriptor_dimensions());
}
void detail::apply_refined_geometry(std::vector<Landmark>& points, const std::map<int, StateGroupf>& before_poses,
                                    const std::map<uint64_t, int>& references, const NodeObservations& after,
                                    const std::map<uint64_t, cv::Point3f>& solved) {
  // A sorted borrowed lookup keeps observer membership explicit without a
  // second copied history/index. New query rows cannot steal an old reference.
  std::vector<const NodeObservations::Observation*> active;
  active.reserve(after.observations.size());
  const auto key = [](const auto* o) { return std::make_pair(o->landmark, o->node); };
  for (const auto& o : after.observations) active.push_back(&o);
  std::sort(active.begin(), active.end(), [&](const auto* a, const auto* b) { return key(a) < key(b); });
  std::map<uint64_t, const NodeObservations::Observation*> reference;
  for (const auto& [point, node] : references) {
    const auto wanted = std::make_pair(point, node);
    const auto at = std::lower_bound(active.begin(), active.end(), wanted, [&](const auto* a, const auto& b) { return key(a) < b; });
    if (at != active.end() && key(*at) == wanted) reference.emplace(point, *at);
  }
  std::map<int, StateGroupf> correction;
  for (auto& point : points) {
    const auto ref = reference.find(point.id);
    if (ref == reference.end()) {
      const auto retained =
          std::lower_bound(active.begin(), active.end(), std::make_pair(point.id, 0), [&](const auto* a, const auto& b) { return key(a) < b; });
      if (retained == active.end() || (*retained)->landmark != point.id) continue;  // No remaining support; caller retires it.
      if (!solved.count(point.id)) throw std::runtime_error("Missing original landmark reference observation");
      reference.emplace(point.id, *retained);  // A reliable solved point needs no old-depth transport.
    }
    const auto selected = reference.find(point.id);
    const auto& o = *selected->second;
    const auto old_pose = before_poses.find(o.node);
    const auto new_pose = after.observation_poses.find(o.node);
    if (new_pose == after.observation_poses.end() || !new_pose->second.isValid()) throw std::runtime_error("Missing refined landmark reference pose");
    const auto found = solved.find(point.id);
    if (found != solved.end()) {
      point.position = found->second;
    } else if (old_pose == before_poses.end() || !old_pose->second.isValid()) {
      throw std::runtime_error("Missing original landmark reference pose");
    } else if (old_pose->second.coefficients() != new_pose->second.coefficients()) {
      auto it = correction.find(o.node);
      if (it == correction.end()) it = correction.emplace(o.node, new_pose->second.compose(old_pose->second.inverse())).first;
      const auto xyz = it->second.transformPoint({point.position.x, point.position.y, point.position.z});
      point.position = {xyz.x(), xyz.y(), xyz.z()};
      if (point.geometry) point.geometry = point.geometry->reanchored(StateGroupd(it->second).normalized());
      point.depth_quality = 0;
    }
    // A deferred depth keeps its old reference-camera geometry, not the
    // unobservable BA depth. Other observers' residuals may legitimately change.
    const Eigen::Vector3d xyz(point.position.x, point.position.y, point.position.z);
    const geometry::CameraPose camera(geometry::CameraPose(new_pose->second), geometry::CameraPose(o.stereo.anchor_from_camera()));
    if (!xyz.allFinite() || !(camera.rotation.col(2).dot(xyz - camera.translation) > 0))
      throw std::runtime_error("Invalid refined landmark reference depth");
  }
}
void detail::apply_depth_quality(std::vector<Landmark>& points, const NodeObservations& history,
                                 const std::map<uint64_t, Mat33_t<double>>& covariances) {
  std::map<uint64_t, size_t> rows;
  for (size_t i = 0; i < points.size(); ++i) rows.emplace(points[i].id, i);
  std::vector<double> range_squared(points.size(), std::numeric_limits<double>::infinity());
  for (const auto& observation : history.observations) {
    const auto row = rows.find(observation.landmark);
    if (row == rows.end() || !covariances.count(observation.landmark)) continue;
    const auto& p = points[row->second].position;
    const auto origin =
        history.observation_poses.at(observation.node).compose(observation.stereo.anchor_from_camera()).translationVector().cast<double>().eval();
    range_squared[row->second] = std::min(range_squared[row->second], (Eigen::Vector3d(p.x, p.y, p.z) - origin).squaredNorm());
  }
  for (size_t i = 0; i < points.size(); ++i) {
    const auto covariance = covariances.find(points[i].id);
    if (covariance == covariances.end()) continue;
    const Eigen::SelfAdjointEigenSolver<Mat33_t<double>> spectrum(covariance->second, Eigen::EigenvaluesOnly);
    if (!covariance->second.allFinite() || spectrum.info() != Eigen::Success || spectrum.eigenvalues().minCoeff() <= 0)
      throw std::logic_error("Invalid accepted landmark covariance");
    points[i].depth_quality = std::isfinite(range_squared[i]) && range_squared[i] > 0
                                  ? float(std::max(0.0, 1.0 - 9 * spectrum.eigenvalues().maxCoeff() / range_squared[i]))
                                  : 0;
  }
}
std::shared_ptr<const LandmarkMap> refined_node_features(const LandmarkMap& map, const NodeObservations& before, const NodeObservations& after,
                                                         const std::map<uint64_t, cv::Point3f>& solved,
                                                         const std::map<uint64_t, Mat33_t<double>>& covariances) {
  auto points = map.points();
  std::set<std::pair<uint64_t, int>> remaining;
  for (const auto& o : after.observations) remaining.emplace(o.landmark, o.node);
  std::map<uint64_t, int> references;
  for (const auto& o : before.observations)
    if (remaining.count({o.landmark, o.node})) {
      auto [it, inserted] = references.emplace(o.landmark, o.node);
      if (!inserted) it->second = std::min(it->second, o.node);
    }
  detail::apply_refined_geometry(points, before.observation_poses, references, after, solved);
  std::set<std::pair<int, uint64_t>> already_counted;
  for (const auto& observation : before.observations) already_counted.emplace(observation.node, observation.landmark);
  std::map<uint64_t, size_t> point_rows;
  for (size_t i = 0; i < points.size(); ++i) point_rows.emplace(points[i].id, i);
  for (const auto& observation : after.observations) {
    if (!already_counted.emplace(observation.node, observation.landmark).second) continue;
    const auto row = point_rows.find(observation.landmark);
    if (row == point_rows.end()) continue;
    auto& point = points[row->second];
    const Vec3_t<double> position(point.position.x, point.position.y, point.position.z);
    const StateGroupd capture = StateGroupd(after.observation_poses.at(observation.node)).normalized();
    point.geometry = point.geometry ? point.geometry->with_observation(observation.stereo, capture, position)
                                    : PointSummary::from_observation(observation.stereo, capture, position);
  }
  detail::apply_depth_quality(points, after, covariances);
  detail::update_representative_descriptors(points, after, after.observation_poses);
  return LandmarkMap::create(map.vocabulary_hash(), std::move(points), map.max_points(), map.descriptor_model_id(), map.descriptor_dimensions());
}
bool redundant_node_observations(const LandmarkMap& map, const NodeObservations& history, const bow::FeatureBlock& query,
                                 const std::vector<Match>& matches, const std::vector<double>& variances, float pyramid_scale) {
  if (matches.empty()) return false;
  std::map<uint64_t, std::map<int, float>> observers;
  for (const auto& o : history.observations) observers[o.landmark][o.node] = o.stereo.sigma();
  size_t redundant = 0;
  for (const auto& m : matches) {
    const auto& keypoint = query.keypoints.at(m.feature);
    if (keypoint.octave < 0 || size_t(keypoint.octave) >= variances.size()) return false;
    const auto sigma = std::sqrt(variances[keypoint.octave]);
    size_t better = 0;
    for (const auto& [node, s] : observers[map.points().at(m.landmark).id]) better += s <= sigma * pyramid_scale + 1e-6;
    redundant += better >= 3;
  }
  return redundant * 10 >= matches.size() * 9;
}
namespace {
// The index validates original pixel/landmark uniqueness. Test conflicts before
// allocating or editing a draft: a rejected hypothesis changes no history.
bool compatible_replacement(const NodeObservations& history, const NodeObservations::Index& index, uint64_t removed, uint64_t retained) {
  const auto track = index.landmarks.find(removed);
  if (track == index.landmarks.end()) return true;
  for (const auto& [node, row] : track->second) {
    const auto& frame = index.frames.at(node);
    const auto existing = frame.find(retained);
    if (existing == frame.end()) continue;
    const auto& o = history.observations[row];
    const auto& other = history.observations[existing->second];
    if (o.feature_id == std::numeric_limits<uint32_t>::max() || o.feature_id != other.feature_id || o.stereo.pixel() != other.stereo.pixel() ||
        o.stereo.calibration() != other.stereo.calibration() ||
        o.stereo.anchor_from_camera().coefficients() != other.stereo.anchor_from_camera().coefficients())
      return false;
  }
  return true;
}
// A separate, deliberately narrower permission than arbitrary ID replacement.
// Nearby detections at different pyramid scales can describe one corner. Different
// camera models/extrinsics or unknown provenance are never silently collapsed.
bool duplicate_measurements(const NodeObservations::Observation& a, const NodeObservations::Observation& b) {
  if (a.feature_id == std::numeric_limits<uint32_t>::max() || b.feature_id == std::numeric_limits<uint32_t>::max() ||
      a.stereo.calibration() != b.stereo.calibration() ||
      a.stereo.anchor_from_camera().coefficients() != b.stereo.anchor_from_camera().coefficients())
    return false;
  if (a.feature_id == b.feature_id) return a.stereo.pixel() == b.stereo.pixel();
  if (a.stereo.sigma() == b.stereo.sigma()) return false;
  const double dx = a.stereo.pixel().x - b.stereo.pixel().x, dy = a.stereo.pixel().z - b.stereo.pixel().z;
  if (dx * dx + dy * dy > 1.0) return false;
  if (a.stereo.pixel().y >= 0 && b.stereo.pixel().y >= 0 &&
      std::abs((a.stereo.pixel().x - a.stereo.pixel().y) - (b.stereo.pixel().x - b.stereo.pixel().y)) > 1.0)
    return false;
  if (a.appearance.descriptor.model_id() != b.appearance.descriptor.model_id()) return false;
  return a.appearance.descriptor.distance(b.appearance.descriptor) <= ((a.appearance.descriptor.metric() == DistanceMetric::l2) ? .7f : 30.f);
}
void replace_observations(NodeObservations& draft, const NodeObservations::Index& index, uint64_t removed, uint64_t retained) {
  const auto state = draft.reliability.find(removed);
  if (state != draft.reliability.end()) {
    detail::merge_reliability(draft.reliability[retained], state->second);
    draft.reliability.erase(state);
  }
  auto& observations = draft.observations;
  // The compatibility check used the unmodified rows. Only frame membership is
  // needed during compaction; original row offsets are not read after this point.
  observations.erase(std::remove_if(observations.begin(), observations.end(),
                                    [&](const auto& o) { return o.landmark == removed && index.frames.at(o.node).count(retained); }),
                     observations.end());
  for (auto& o : observations)
    if (o.landmark == removed) o.landmark = retained;
  draft.landmark_high_water = std::max({draft.landmark_high_water, removed, retained});
}
void refresh_replaced_support(NodeObservations& history) {
  // Every accepted replacement was checked against a current, validated index.
  // Identity remapping/compaction preserves uniqueness. Support is derived once
  // from the final rows; intermediate support is never consumed by fusion.
  history.support.clear();
  for (const auto& o : history.observations) {
    ++history.support[o.landmark];
    history.landmark_high_water = std::max(history.landmark_high_water, o.landmark);
  }
}
}  // namespace
void FeatureMap::fuse_landmarks(const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& query_features,
                                const StateGroupf& query_from_map, const std::vector<Match>& seeds) {
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  const auto& original = history();
  ProjectionOptions options;
  options.detect_duplicates = true;
  options.max_distance = 50;
  const auto candidates = project_unmatched(*map_features_, query, query_features, query_from_map, seeds, options);
  if (candidates.status != MatchStatus::success || candidates.pairs.empty()) return;
  std::map<uint32_t, uint64_t> owners;
  for (const auto& m : seeds) owners[m.feature] = map_features_->points().at(m.landmark).id;
  std::vector<std::pair<uint64_t, uint64_t>> pairs;
  pairs.reserve(candidates.pairs.size());
  for (const auto& m : candidates.pairs) pairs.emplace_back(map_features_->points().at(m.landmark).id, owners.at(m.feature));
  auto index = original.index();
  std::map<uint64_t, size_t> point_rows;
  for (size_t i = 0; i < map_features_->points().size(); ++i) point_rows.emplace(map_features_->points()[i].id, i);
  std::shared_ptr<NodeObservations> draft;
  std::vector<Landmark> points;
  std::set<uint64_t> erased_points;
  std::set<size_t> erased_observations;
  for (auto [removed, retained] : pairs) {
    const auto& bundle = draft ? *draft : original;
    if (!index.landmarks.count(removed) || !index.landmarks.count(retained)) continue;
    if (index.landmarks.at(removed).size() > index.landmarks.at(retained).size()) std::swap(removed, retained);
    if (!point_rows.count(removed) || !point_rows.count(retained)) continue;
    const auto& current_points = draft ? points : map_features_->points();
    const auto& p = current_points[point_rows.at(retained)];
    const auto& q = current_points[point_rows.at(removed)];
    // Stable identity is independent of which existing geometry best explains
    // the union. Never add summaries whose capture histories may overlap.
    const Eigen::Vector3d first(p.position.x, p.position.y, p.position.z), second(q.position.x, q.position.y, q.position.z);
    if (!p.geometry || !q.geometry) continue;
    std::map<int, size_t> selected = index.landmarks.at(retained);
    bool compatible = true;
    for (const auto& [node, row] : index.landmarks.at(removed)) {
      const auto [at, inserted] = selected.emplace(node, row);
      if (inserted) continue;
      const auto& a = bundle.observations.at(row);
      const auto& b = bundle.observations.at(at->second);
      if (!duplicate_measurements(a, b)) {
        compatible = false;
        break;
      }
      // Keep one immutable measurement, preferring finer localization. Never
      // average pixels, invent a new feature identity, or count both scales.
      if (a.stereo.sigma() < b.stereo.sigma()) at->second = row;
    }
    if (!compatible) continue;
    if (selected.size() < 2) continue;
    // Validate the new identity association with the two committed summaries.
    // Shared capture history is not replayed or added twice.
    const auto association_cost = [&](const Vec3_t<double>& xyz) {
      try {
        return std::max(p.geometry->evaluate(xyz).squaredNorm(), q.geometry->evaluate(xyz).squaredNorm());
      } catch (const std::domain_error&) {
        return std::numeric_limits<double>::infinity();
      }
    };
    const double first_cost = association_cost(first), second_cost = association_cost(second);
    const bool use_second = second_cost < first_cost;
    const Vec3_t<double> xyz = use_second ? second : first;
    if (std::min(first_cost, second_cost) > 7.81473) continue;
    const auto& selected_geometry = *(use_second ? q.geometry : p.geometry);
    Mat33_t<double> jacobian;
    (void)selected_geometry.evaluate(xyz, &jacobian);
    const Eigen::SelfAdjointEigenSolver<Mat33_t<double>> information(jacobian.transpose() * jacobian, Eigen::EigenvaluesOnly);
    const double range2 = (xyz - selected_geometry.reference.translationVector()).squaredNorm();
    if (information.info() != Eigen::Success || information.eigenvalues().minCoeff() * range2 <= 9) continue;
    if (!draft) {
      draft = std::make_shared<NodeObservations>(original);
      points = map_features_->points();
    }
    // Keep row addresses stable until all hypotheses have been processed. Update
    // only the two affected tracks; rebuilding the entire index per merge is quadratic.
    for (const auto& [node, row] : index.landmarks.at(removed)) {
      auto& frame = index.frames.at(node);
      const auto other = frame.find(retained);
      if (other != frame.end()) {
        if (selected.at(node) == row) {
          erased_observations.insert(other->second);
          other->second = row;
          draft->observations[row].landmark = retained;
          index.landmarks.at(retained).at(node) = row;
        } else
          erased_observations.insert(row);
      } else {
        draft->observations[row].landmark = retained;
        frame.emplace(retained, row);
        index.landmarks.at(retained).emplace(node, row);
      }
      frame.erase(removed);
    }
    const auto state = draft->reliability.find(removed);
    if (state != draft->reliability.end()) {
      detail::merge_reliability(draft->reliability[retained], state->second);
      draft->reliability.erase(state);
    }
    index.landmarks.erase(removed);
    erased_points.insert(removed);
    auto& point = points[point_rows.at(retained)];
    // These tracks may share capture information. Their identity is unified,
    // but exactly one existing summary survives; never add correlated histories.
    const auto chosen_geometry = second_cost < first_cost ? q.geometry : p.geometry;
    point.geometry = chosen_geometry;
    point.position = {float(xyz.x()), float(xyz.y()), float(xyz.z())};
    point.miss_count = std::min(point.miss_count, points[point_rows.at(removed)].miss_count);
    point.depth_quality = 0;  // Changed observation union has not yet received a joint BA marginal.
    point.stereo = {};
    const auto ref = index.landmarks.at(retained).find(draft->reference_node);
    if (ref != index.landmarks.at(retained).end()) point.stereo = draft->observations[ref->second].stereo;
    draft->landmark_high_water = std::max({draft->landmark_high_water, removed, retained});
    EXT_LOG_DEBUG("[point_fusion] submap={} removed={} retained={}", draft->reference_node, removed, retained);
  }
  if (!draft) return;
  size_t row = 0;
  auto& observations = draft->observations;
  observations.erase(std::remove_if(observations.begin(), observations.end(), [&](const auto&) { return erased_observations.count(row++); }),
                     observations.end());
  points.erase(std::remove_if(points.begin(), points.end(), [&](const auto& p) { return erased_points.count(p.id); }), points.end());
  refresh_replaced_support(*draft);
  auto bank = LandmarkMap::create(map_features_->vocabulary_hash(), std::move(points), map_features_->max_points(),
                                  map_features_->descriptor_model_id(), map_features_->descriptor_dimensions());
  // All allocation/validation happens on the private content. Publish the bank
  // and its matching history together only after the complete operation succeeds.
  map_features_ = std::move(bank);
  observation_history_ = std::move(draft);
}

}  // namespace sapphire::backend::featmap

namespace sapphire::backend::featmap {
void detail::retire_unsupported_points(std::vector<Landmark>& points, NodeObservations& history, const std::set<uint64_t>& previously_observed) {
  history.refresh_support();
  std::set<uint64_t> reference_observed;
  for (const auto& o : history.observations)
    if (o.node == history.reference_node) reference_observed.insert(o.landmark);
  for (auto& p : points) {
    history.landmark_high_water = std::max(history.landmark_high_water, p.id);
    // Do not leave a rejected original stereo factor available to another
    // solver through the canonical bank. Raw source features stay untouched.
    if (history.reference_node && previously_observed.count(p.id) && !reference_observed.count(p.id)) p.stereo = {};
  }
  points.erase(
      std::remove_if(points.begin(), points.end(), [&](const auto& p) { return previously_observed.count(p.id) && !history.support.count(p.id); }),
      points.end());
}
std::shared_ptr<const LandmarkMap> detail::retire_unsupported(const LandmarkMap& bank, NodeObservations& history,
                                                              const std::set<uint64_t>& previously_observed) {
  auto points = bank.points();
  retire_unsupported_points(points, history, previously_observed);
  return LandmarkMap::create(bank.vocabulary_hash(), std::move(points), bank.max_points(), bank.descriptor_model_id(), bank.descriptor_dimensions());
}
void FeatureMap::replace_landmark(uint64_t removed, uint64_t retained) {
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  if (!removed || !retained || removed == retained) throw std::invalid_argument("Invalid landmark replacement");
  for (uint64_t id : {removed, retained})
    if (std::none_of(map_features_->points().begin(), map_features_->points().end(), [&](const auto& p) { return p.id == id; }))
      throw std::invalid_argument("Replacement landmark outside owning submap");
  const auto index = history().index();
  if (!compatible_replacement(history(), index, removed, retained)) throw std::runtime_error("Conflicting observations during landmark replacement");
  auto draft = std::make_shared<NodeObservations>(history());
  auto points = map_features_->points();
  replace_observations(*draft, index, removed, retained);
  refresh_replaced_support(*draft);
  points.erase(std::remove_if(points.begin(), points.end(), [&](const auto& p) { return p.id == removed; }), points.end());
  auto bank = LandmarkMap::create(map_features_->vocabulary_hash(), std::move(points), map_features_->max_points(),
                                  map_features_->descriptor_model_id(), map_features_->descriptor_dimensions());
  map_features_ = std::move(bank);
  observation_history_ = std::move(draft);
}

}  // namespace sapphire::backend::featmap

namespace sapphire::backend::featmap {
void FeatureMap::maintain_landmarks(const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& features, int capture,
                                    const StateGroupf& query_from_map, const std::vector<double>& pixel_variances,
                                    const std::map<uint64_t, uint32_t>& verified_rows) {
  if (!map_features_ || !features || query.shared_features() != features || query.perspectives().size() != 1 || capture <= 0 ||
      !query_from_map.isValid())
    throw std::invalid_argument("Invalid landmark maintenance revisit");
  const auto& original = history();
  if (!original.observation_poses.count(capture)) throw std::invalid_argument("Maintenance capture lacks placement");
  if (original.maintenance_captures.count(capture)) return;
  const auto& sensor = query.perspectives().front();
  if (!sensor.isValidForReprojection() || sensor.imageSize().width <= 0 || sensor.imageSize().height <= 0)
    throw std::invalid_argument("Invalid maintenance camera");
  features->validate();
  if (pixel_variances.empty() || std::any_of(pixel_variances.begin(), pixel_variances.end(), [](double v) { return !std::isfinite(v) || v <= 0; }))
    throw std::invalid_argument("Invalid maintenance pixel variances");
  if (!query_from_map.inverse().matrix().isApprox(original.observation_poses.at(capture).matrix(), 1e-4))
    throw std::invalid_argument("Maintenance pose does not match committed geometry");
  const auto index = original.index();
  // A revisit must contain actual accepted evidence, not just a predicted pose.
  const auto current = index.frames.find(capture);
  if ((current == index.frames.end() || current->second.size() < 6) && verified_rows.size() < 6) return;
  std::set<uint32_t> query_rows;
  for (const auto& [id, row] : verified_rows)
    if (!id || row >= features->keypoints.size() || !query_rows.insert(row).second) throw std::invalid_argument("Invalid maintenance association");
  const auto measurement_frame = camera::stereo_frame(sensor, features->keypoints, features->points, pixel_variances, true);
  const auto map_from_query = query_from_map.inverse();
  const geometry::CameraPose camera(geometry::CameraPose(map_from_query), geometry::CameraPose(sensor.cameraToBase()));
  auto draft = std::make_shared<NodeObservations>(original);
  std::set<uint64_t> retired;
  size_t spatial_visits = 0;
  std::vector<size_t> neighbors;
  for (const auto& point : map_features_->points()) {
    const auto track = index.landmarks.find(point.id);
    if (track == index.landmarks.end()) continue;
    const Eigen::Vector3d xyz(point.position.x, point.position.y, point.position.z);
    auto& state = draft->reliability[point.id];
    if (!original.reliability.count(point.id)) {
      // Legacy maps begin with positive evidence only. Pruned pixels never
      // become historical failures and no numeric node ID is used as an age.
      for (const auto& [node, row] : track->second)
        if (node != capture && original.observation_poses.count(node)) state.trials.emplace_back(node, true);
      if (state.trials.size() > 8) state.trials.erase(state.trials.begin(), state.trials.end() - 8);
    }
    bool hit = track->second.count(capture) &&
               geometry::inlier(original.observations[track->second.at(capture)].stereo, original.observation_poses.at(capture), xyz);
    camera::StereoObservation ephemeral;
    const auto verified = verified_rows.find(point.id);
    if (!hit && verified != verified_rows.end() && !track->second.count(capture)) {
      const auto row = verified->second;
      const auto& key = features->keypoints[row];
      if (key.octave < 0 || size_t(key.octave) >= pixel_variances.size()) throw std::invalid_argument("Invalid maintenance feature octave");
      ephemeral = {measurement_frame, row};
      hit = geometry::inlier(ephemeral, map_from_query, xyz);
    }
    if (!hit) {
      // Different sensors are not failed revisits of this feature's camera.
      const auto& source = original.observations[track->second.begin()->second].stereo;
      const auto k = source.calibration();
      if (source.anchor_from_camera().coefficients() != sensor.cameraToBase().coefficients() ||
          k != std::array<float, 5>{float(sensor.fx()), float(sensor.fy()), float(sensor.cx()), float(sensor.cy()), float(sensor.focalXBaseline())})
        continue;
      const Eigen::Vector3d optical = camera.rotation.transpose() * (xyz - camera.translation);
      if (!optical.allFinite() || optical.z() <= 0) continue;
      const auto pixel = sensor.project({optical.x(), optical.y(), optical.z()});
      if (!pixel || !sensor.isInImage(pixel->x - 8, pixel->y - 8) || !sensor.isInImage(pixel->x + 8, pixel->y + 8)) continue;
      const auto source_camera = original.observation_poses.find(track->second.begin()->first);
      if (source_camera == original.observation_poses.end()) continue;
      const Eigen::Vector3d origin = source_camera->second.compose(source.anchor_from_camera()).translationVector().cast<double>();
      const Eigen::Vector3d old_ray = xyz - origin, new_ray = xyz - camera.translation;
      if (old_ray.dot(new_ray) <= .5 * old_ray.norm() * new_ray.norm()) continue;
      // As in scale-pyramid projection matching, being in the image is not
      // enough: the feature must still fall within the detector's scale range.
      const double maximum_range = old_ray.norm() * source.sigma() / std::sqrt(pixel_variances.front());
      const double minimum_range = maximum_range * std::sqrt(pixel_variances.front() / pixel_variances.back());
      if (new_ray.norm() < .8 * minimum_range || new_ray.norm() > 1.2 * maximum_range) continue;
      size_t visits = 0;
      if (!query.collect_keypoint_indices({float(pixel->x), float(pixel->y)}, 8, 20000 - spatial_visits, neighbors, visits)) return;
      spatial_visits += visits;
      bool foreground = false;
      const auto camera_from_query = sensor.cameraToBase().inverse();
      for (size_t row : neighbors) {
        const auto& p = features->points.at(row);
        const Eigen::Vector3f q = camera_from_query.transformPoint({p.x, p.y, p.z});
        if (!q.allFinite() || q.z() <= 0 || q.z() >= .7 * optical.z()) continue;
        const auto octave = features->keypoints.at(row).octave;
        if (octave < 0 || size_t(octave) >= pixel_variances.size()) continue;
        if (solver::Triangulator::stereo_depth_support(camera::StereoObservation{measurement_frame, row})) {
          foreground = true;
          break;
        }
      }
      if (foreground) continue;  // Sparse foreground evidence is a veto, never proof of visibility.
    }
    bool depth_ok = state.confirmed;
    if (!depth_ok && point.geometry) {
      // Inspect the committed geometry, without reopening accepted pixels.
      // A verified but uncommitted measurement is temporary evidence only.
      const auto evidence =
          hit && ephemeral.valid() ? point.geometry->with_observation(ephemeral, StateGroupd(map_from_query).normalized(), xyz) : *point.geometry;
      Mat33_t<double> jacobian;
      (void)evidence.evaluate(xyz, &jacobian);
      const Eigen::SelfAdjointEigenSolver<Mat33_t<double>> information(jacobian.transpose() * jacobian, Eigen::EigenvaluesOnly);
      const double range_squared = (xyz - evidence.reference.translationVector()).squaredNorm();
      depth_ok = information.info() == Eigen::Success && information.eigenvalues().minCoeff() * range_squared > 9;
    }
    if (std::any_of(state.trials.begin(), state.trials.end(), [&](const auto& trial) { return trial.first == capture; })) continue;
    state.trials.emplace_back(capture, hit);
    if (state.trials.size() > 8) state.trials.erase(state.trials.begin());
    const size_t hits = std::count_if(state.trials.begin(), state.trials.end(), [](const auto& t) { return t.second; });
    if (depth_ok && hits >= 3) state.confirmed = true;
    // Probation applies to transient features even if their initial stereo depth
    // was excellent. Confirmed points can also expire after repeated poor revisits.
    const bool probation_failure = !state.confirmed && state.trials.size() >= 3 && hits < 2;
    const bool depth_failure = !depth_ok && state.trials.size() >= 5;
    const bool persistent_failure = state.trials.size() == 8 && hits <= 2;
    if (probation_failure || depth_failure || persistent_failure) retired.insert(point.id);
  }
  draft->maintenance_captures.insert(capture);
  auto bank = map_features_;
  if (!retired.empty()) {
    auto points = map_features_->points();
    points.erase(std::remove_if(points.begin(), points.end(), [&](const auto& p) { return retired.count(p.id); }), points.end());
    auto& obs = draft->observations;
    obs.erase(std::remove_if(obs.begin(), obs.end(), [&](const auto& o) { return retired.count(o.landmark); }), obs.end());
    bank = LandmarkMap::create(map_features_->vocabulary_hash(), std::move(points), map_features_->max_points(), map_features_->descriptor_model_id(),
                               map_features_->descriptor_dimensions());
  }
  // Only whole tracks were removed; original rows and capture uniqueness remain
  // valid. Reuse the validated index rather than rebuilding it twice per revisit.
  draft->support.clear();
  for (const auto& [id, track] : index.landmarks) {
    draft->landmark_high_water = std::max(draft->landmark_high_water, id);
    if (!retired.count(id)) draft->support[id] = track.size();
  }
  for (auto it = draft->reliability.begin(); it != draft->reliability.end();)
    if (!draft->support.count(it->first))
      it = draft->reliability.erase(it);
    else
      ++it;
  detail::validate_bundle_values(*draft);
  if (draft->observations.size() + draft->observation_poses.size() + draft->reliability.size() + draft->maintenance_captures.size() > 16 * 1024 - 1)
    draft->encode();
  map_features_ = std::move(bank);
  observation_history_ = std::move(draft);
}
}  // namespace sapphire::backend::featmap
