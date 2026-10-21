#include "backend/maplayer/featmap/refinement.hpp"
#include <backend/maplayer/featmap/region_merge.hpp>
#include <backend/maplayer/featmap/merge.hpp>
#include <chrono>
#include <Eigen/Eigenvalues>

#include "backend/tools/logging.hpp"
#include "backend/maplayer/featmap/internal.hpp"

namespace sapphire::backend::featmap {
using namespace detail;
std::map<uint64_t, uint64_t> supported_region_correspondences(const LandmarkMap& target, const NodeObservations& target_history, const LandmarkMap& source,
                                                              const NodeObservations& source_history, const StateGroupf& target_from_source,
                                                              const std::vector<Match>& proposals) {
  if (target.descriptor_model_id() != source.descriptor_model_id() || target.descriptor_dimensions() != source.descriptor_dimensions()) throw MergeRejected("Region descriptor model mismatch");
  validate_local_poses(target_history);
  validate_local_poses(source_history);
  std::map<uint64_t, uint64_t> result;
  std::set<uint64_t> destinations;
  const StateGroupd source_from_target = StateGroupd(target_from_source).normalized().inverse();
  for (const auto& m : proposals) {
    const auto& p = target.points().at(m.landmark);
    const auto& q = source.points().at(m.feature);
    if (!p.geometry || !q.geometry) continue;
    if (p.appearance.descriptor.distance(q.appearance.descriptor) > (target.descriptor_model_id() ? .7f : 50.f)) continue;
    // Automatic previous-Node consolidation starts from the committed graph
    // placement. Descriptor agreement cannot authorize moving a distant region.
    // Check this new association against the committed summaries, not old pixels.
    try {
      const Vec3_t<double> point(p.position.x, p.position.y, p.position.z);
      const double error = p.geometry->evaluate(point).squaredNorm() +
          q.geometry->evaluate(source_from_target.transformPoint(point)).squaredNorm();
      if (!std::isfinite(error) || error > 7.81473) continue;
    } catch (const std::domain_error&) { continue; }
    if (!destinations.insert(p.id).second || !result.emplace(q.id, p.id).second) throw std::runtime_error("Nonunique region correspondence proposals");
  }
  return result;
}

RegionMergeResult merge_node_regions(const LandmarkMap& target, const NodeObservations& target_history, const LandmarkMap& source,
                                       const NodeObservations& source_history, const StateGroupf& target_from_source,
                                       const std::map<uint64_t, uint64_t>& correspondences, const camera::Perspective& source_camera, size_t minimum,
                                       const NodeBundleOptions& options) {
  (void)source_camera;  // Each summary retains its own optical reference.
  const auto start = std::chrono::steady_clock::now();
  const auto check = [](bool ok, const char* message) { if (!ok) throw MergeRejected(message); };
  check(target_history.reference_node > 0 && source_history.reference_node > 0 &&
        target_history.reference_node != source_history.reference_node, "Distinct region owners required");
  check(target_from_source.isValid(), "Invalid region alignment");
  check(target.vocabulary_hash() == source.vocabulary_hash(), "Region vocabulary mismatch");
  check(minimum >= 6 && correspondences.size() >= minimum && options.max_points >= minimum, "Insufficient region associations or BA budget");
  if (target.descriptor_model_id() != source.descriptor_model_id() || target.descriptor_dimensions() != source.descriptor_dimensions()) throw MergeRejected("Region descriptor model mismatch");
  validate_local_poses(target_history);
  validate_local_poses(source_history);
  // Capture ownership survives pixel pruning. Shared owners must never add
  // their summaries: that would count the same historical information twice.
  for (const auto& [id, pose] : source_history.observation_poses)
    check(!target_history.observation_poses.count(id), "Regions share an original capture");
  std::map<uint64_t, size_t> target_rows, source_rows;
  for (size_t i = 0; i < target.points().size(); ++i) target_rows.emplace(target.points()[i].id, i);
  for (size_t i = 0; i < source.points().size(); ++i) source_rows.emplace(source.points()[i].id, i);
  using BA = solver::BASolver;
  BA::Problem problem;
  problem.poses.push_back({source_history.reference_node, target_from_source, false, true});
  std::vector<std::pair<uint64_t, uint64_t>> selected;
  std::set<uint64_t> destinations;
  for (const auto& [from, to] : correspondences) {
    const auto source_row = source_rows.find(from), target_row = target_rows.find(to);
    check(source_row != source_rows.end() && target_row != target_rows.end() && destinations.insert(to).second,
          "Invalid or nonunique region association");
    const auto& a = target.points()[target_row->second];
    const auto& b = source.points()[source_row->second];
    check(a.geometry && b.geometry, "Region point lacks committed pixel summary");
    check(a.appearance.descriptor.distance(b.appearance.descriptor) <= (target.descriptor_model_id() ? .7f : 50.f), "Region descriptor mismatch");
    if (problem.points.size() >= options.max_points) continue;
    BA::Point point;
    point.id = to;
    point.initial = {a.position.x, a.position.y, a.position.z};
    point.history = a.geometry;
    point.source = BA::Point::Source{0, *b.geometry};
    problem.points.push_back(std::move(point));
    selected.emplace_back(from, to);
  }
  BA ba;
  BA::Options solve_options;
  solve_options.minimum_required_points = minimum;
  solve_options.max_iterations = options.max_iterations;
  const auto solved = ba.solve(problem, {}, solve_options, options.diagnostics);
  if (solved.status == BA::Status::InvalidProblem) throw std::logic_error("Invalid region summary assembly: " + solved.message);
  check(solved.status == BA::Status::Success, solved.message.c_str());
  const StateGroupd relative = StateGroupd(solved.poses[0]).normalized();
  RegionMergeResult result;
  result.refinement.status = solved.status;
  result.refinement.pose = solved.poses[0];
  if (solved.pose_covariances[0]) result.refinement.covariance = solved.pose_covariances[0]->matrix;
  result.refinement.optimized_points = problem.points.size();
  result.refinement.optimized_views = 1;
  result.refinement.measurements = 2 * problem.points.size();
  result.optimization_observer = source_history.reference_node;
  std::vector<Landmark> points = target.points();
  std::map<uint64_t, size_t> evaluated;
  for (size_t i = 0; i < selected.size(); ++i) evaluated.emplace(selected[i].first, i);
  std::map<uint64_t, uint64_t> accepted;
  for (const auto& [from, to] : correspondences) {
    auto& point = points[target_rows.at(to)];
    const auto& incoming = source.points()[source_rows.at(from)];
    const auto tested = evaluated.find(from);
    Vec3_t<double> position(point.position.x, point.position.y, point.position.z);
    if (tested != evaluated.end()) {
      const size_t i = tested->second;
      if (!solved.active_points[i]) continue;
      const auto& covariance = solved.point_covariances[i];
      const Eigen::SelfAdjointEigenSolver<Mat33_t<double>> spectrum(covariance, Eigen::EigenvaluesOnly);
      const double range2 = (solved.points[i] - point.geometry->reference.translationVector()).squaredNorm();
      if (spectrum.info() != Eigen::Success || !covariance.allFinite() || spectrum.eigenvalues().minCoeff() <= 0 ||
          9 * spectrum.eigenvalues().maxCoeff() >= range2) {
        ++result.refinement.deferred_depths;
        continue;  // Keep both point identities; no unobservable geometry is committed.
      }
      position = solved.points[i];
      point.position = {float(position.x()), float(position.y()), float(position.z())};
      point.depth_quality = float(std::max(0.0, 1 - 9 * spectrum.eigenvalues().maxCoeff() / range2));
      result.refinement.points.emplace(to, point.position);
      result.refinement.point_covariances.emplace(to, covariance);
      result.refinement.inliers.push_back(int(i));
    } else {
      // The finite point budget controls optimization only. Unoptimized pairs
      // must still pass the same new-association test at the accepted transform.
      double error;
      try {
        error = point.geometry->evaluate(position).squaredNorm() + incoming.geometry->evaluate(relative.inverse().transformPoint(position)).squaredNorm();
      } catch (const std::domain_error&) { continue; }
      if (!std::isfinite(error) || error > 7.81473) continue;
    }
    point.geometry = point.geometry->merged(*incoming.geometry, relative, position);
    accepted.emplace(from, to);
  }
  check(accepted.size() >= minimum, "Insufficient independently supported region associations after BA");
  result.bundle = target_history;
  result.bundle.landmark_high_water = std::max(target_history.landmark_high_water, source_history.landmark_high_water);
  for (const auto& point : target.points()) result.bundle.landmark_high_water = std::max(result.bundle.landmark_high_water, point.id);
  for (const auto& point : source.points()) result.bundle.landmark_high_water = std::max(result.bundle.landmark_high_water, point.id);
  for (const auto& [id, pose] : source_history.observation_poses) {
    const auto transformed = solved.poses[0].compose(pose);
    result.bundle.observation_poses.emplace(id, transformed);
    result.refinement.observation_poses.emplace(id, transformed);
  }
  result.source_to_retained = accepted;
  for (const auto& original : source.points()) {
    if (accepted.count(original.id)) continue;
    check(result.bundle.landmark_high_water < std::numeric_limits<uint64_t>::max(), "Point identity exhausted");
    auto point = original;
    point.id = ++result.bundle.landmark_high_water;
    result.source_to_retained.emplace(original.id, point.id);
    const auto position = relative.transformPoint(Vec3_t<double>(point.position.x, point.position.y, point.position.z));
    point.position = {float(position.x()), float(position.y()), float(position.z())};
    if (point.geometry) point.geometry = point.geometry->reanchored(relative);
    point.stereo = {};
    if (point.appearance.ray.pixel_angle > 0) {
      const auto& ray = original.appearance.ray;
      const auto origin = relative.transformPoint(Vec3_t<double>(ray.origin.x, ray.origin.y, ray.origin.z));
      const Vec3_t<double> direction = relative.rotationMatrix() * Vec3_t<double>(ray.direction.x, ray.direction.y, ray.direction.z);
      point.appearance.ray.origin = {float(origin.x()), float(origin.y()), float(origin.z())};
      point.appearance.ray.direction = {float(direction.x()), float(direction.y()), float(direction.z())};
    }
    points.push_back(std::move(point));
  }
  // This archive remains appearance/provenance data. It is not assembled into
  // the next BA problem and no accepted historical pixel is reclassified here.
  for (auto observation : source_history.observations) {
    observation.landmark = result.source_to_retained.at(observation.landmark);
    result.bundle.observations.push_back(std::move(observation));
  }
  result.bundle.maintenance_captures.insert(source_history.maintenance_captures.begin(), source_history.maintenance_captures.end());
  for (const auto& [id, state] : source_history.reliability)
    merge_reliability(result.bundle.reliability[result.source_to_retained.at(id)], state);
  update_representative_descriptors(points, result.bundle, result.bundle.observation_poses);
  LandmarkMap::retain_best(points, target.max_points());
  std::set<uint64_t> retained;
  for (const auto& point : points) retained.insert(point.id);
  auto& observations = result.bundle.observations;
  observations.erase(std::remove_if(observations.begin(), observations.end(), [&](const auto& o) { return !retained.count(o.landmark); }), observations.end());
  result.bundle.refresh_support();
  for (auto it = result.source_to_retained.begin(); it != result.source_to_retained.end();)
    if (!retained.count(it->second)) it = result.source_to_retained.erase(it); else ++it;
  result.features = LandmarkMap::create(target.vocabulary_hash(), std::move(points), target.max_points(), target.descriptor_model_id(), target.descriptor_dimensions());
  result.bundle.validate();
  result.refinement.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
  return result;
}
}  // namespace sapphire::backend::featmap
