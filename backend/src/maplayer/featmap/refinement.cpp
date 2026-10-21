#include "backend/maplayer/featmap/refinement.hpp"

#include <Eigen/Eigenvalues>
#include <chrono>
#include <unordered_map>

#include "backend/solver/triangulator.hpp"
#include "backend/tools/logging.hpp"

namespace sapphire::backend::featmap {
NodeBundleResult refine_node(const LandmarkMap& map, const NodeObservations& history, const std::map<int, StateGroupf>& boundaries,
                             const bow::FeatureBlock& query, const camera::Perspective& camera, const std::vector<Match>& matches,
                             const StateGroupf& initial, const std::vector<double>& variances, size_t minimum, bool fixed_query, bool joint_history,
                             int reference_node, const NodeBundleOptions& options, const Mat66_t<double>* fixed_query_covariance,
                             const std::map<int, StateGroupf>& query_from_members) {
  const auto start = std::chrono::steady_clock::now();
  NodeBundleResult result;
  solver::BASolver ba;
  solver::BASolver::Problem problem;
  std::vector<solver::BASolver::Observation> observations;
  solver::BASolver::Options solve_options;
  solve_options.minimum_required_points = minimum;
  solve_options.minimum_fixed_tracks = joint_history ? 6 : 0;
  solve_options.max_iterations = options.max_iterations;
  double solve_ms = 0;
  if (!initial.isValid() || minimum < (fixed_query ? 1U : 6U)) throw std::invalid_argument("Invalid submap BA request");
  if (!options.max_optimized_views || options.max_points < minimum || options.max_points > point_capacity)
    throw std::invalid_argument("Invalid local BA computation budget");
  if (!query_from_members.empty() && (joint_history || fixed_query_covariance))
    throw std::invalid_argument("Rigid node alignment cannot optimize member poses independently");
  if (joint_history && history.reference_node && reference_node && history.reference_node != reference_node)
    throw std::runtime_error("Node reference mismatch");
  if (joint_history && !reference_node)
    reference_node = history.reference_node ? history.reference_node : (boundaries.empty() ? 0 : boundaries.begin()->first);
  if (joint_history && !reference_node) throw std::runtime_error("Missing original submap reference");

  if (fixed_query_covariance && (!fixed_query || joint_history || !fixed_query_covariance->allFinite() ||
                                 !fixed_query_covariance->isApprox(fixed_query_covariance->transpose(), 1e-8) ||
                                 Eigen::LLT<Mat66_t<double>>(*fixed_query_covariance).info() != Eigen::Success))
    throw std::invalid_argument("Independent point refinement requires a fixed query and positive pose covariance");

  std::unordered_map<int, size_t> pose_rows;
  const bool summarized = !joint_history && query_from_members.empty();
  problem.poses.reserve(summarized ? 1 : boundaries.size() + 1);
  if (!summarized)
    for (const auto& [node, boundary] : boundaries) {
      if (query_from_members.count(node)) continue;
      const auto previous = history.observation_poses.find(node);
      auto pose = joint_history && previous != history.observation_poses.end() ? previous->second : boundary;
      if (joint_history && node == reference_node) pose = StateGroupf::identity();
      pose_rows.emplace(node, problem.poses.size());
      problem.poses.push_back({node, pose, true});
    }
  const size_t query_row = problem.poses.size();
  problem.poses.push_back({-1, initial, fixed_query, true});
  for (const auto& [node, local] : query_from_members) {
    if (!local.isValid()) throw std::invalid_argument("Invalid rigid source member placement");
    pose_rows.emplace(node, query_row);
  }
  // Source captures share one variable relative pose. Their measured pixels
  // and internal transforms remain fixed; only this temporary solver projection
  // composes the internal placement into each camera extrinsic.
  std::map<std::pair<int, const camera::StereoFrame*>, std::shared_ptr<const camera::StereoFrame>> member_frames;
  std::vector<camera::StereoObservation> member_measurements(query_from_members.empty() ? 0 : history.observations.size());
  // One pass over stored rows builds adjacency and covisibility together.
  // Only selected row references are assembled; pixels and calibration stay in their owners.
  const size_t absent = std::numeric_limits<size_t>::max();
  std::unordered_map<uint64_t, size_t> point_rows;
  point_rows.reserve(map.points().size());
  for (size_t i = 0; i < map.points().size(); ++i) point_rows.emplace(map.points()[i].id, i);
  std::vector<uint8_t> matched(map.points().size(), 0);
  for (const auto& m : matches) matched.at(m.landmark) = 1;
  std::vector<size_t> head(map.points().size(), absent), tail(head), count(map.points().size(), 0);
  const size_t history_rows = summarized ? 0 : history.observations.size();
  std::vector<size_t> next(history_rows, absent), observer(history_rows, absent), support(summarized ? 0 : boundaries.size(), 0);
  std::vector<uint8_t> fixed_support(map.points().size(), false), source_support(map.points().size(), false);
  if (summarized)
    for (size_t i = 0; i < map.points().size(); ++i) count[i] = map.points()[i].geometry.has_value();
  for (size_t row = 0; row < history_rows; ++row) {
    const auto& o = history.observations[row];
    const auto p = point_rows.find(o.landmark);
    const auto v = pose_rows.find(o.node);
    if (p == point_rows.end() || v == pose_rows.end() || !o.stereo.valid()) continue;
    const size_t i = p->second;
    if (head[i] == absent)
      head[i] = row;
    else
      next[tail[i]] = row;
    tail[i] = row;
    ++count[i];
    observer[row] = v->second;
    fixed_support[i] = fixed_support[i] || v->second != query_row;
    source_support[i] = source_support[i] || v->second == query_row;
    if (const auto member = query_from_members.find(o.node); member != query_from_members.end()) {
      auto& frame = member_frames[{o.node, o.stereo.frame.get()}];
      if (!frame) {
        frame = std::make_shared<const camera::StereoFrame>(member->second.compose(o.stereo.anchor_from_camera()), o.stereo.calibration(),
                                                            o.stereo.frame->pixels);
      }
      member_measurements[row] = {frame, o.stereo.row};
    }
    if (matched[i] && v->second != query_row) ++support[v->second];
  }
  if (joint_history) {
    std::vector<std::pair<size_t, int>> ranked;
    for (size_t i = 0; i < support.size(); ++i)
      if (problem.poses[i].id != reference_node && support[i] >= 6) ranked.emplace_back(support[i], problem.poses[i].id);
    std::sort(ranked.rbegin(), ranked.rend());
    size_t selected = fixed_query ? 0 : 1;
    for (const auto& [n, node] : ranked) {
      if (selected >= options.max_optimized_views) break;
      problem.poses[pose_rows.at(node)].fixed = false;
      ++selected;
    }
  }
  problem.points.reserve(options.max_points);
  const auto measurement_frame = camera::stereo_frame(camera, query.keypoints, query.points, variances, true);
  std::vector<camera::StereoObservation> current;
  // At most one generated observation per selected match. Reserve before publishing references;
  // history stays immutable and this bound prevents query storage from moving until solve returns.
  current.reserve(std::min(matches.size(), size_t(options.max_points)));
  std::vector<int> match_rows;
  std::vector<int> observation_captures;
  match_rows.reserve(options.max_points);
  observations.reserve(std::min(history_rows, size_t(options.max_points) * boundaries.size()) + options.max_points);
  observation_captures.reserve(observations.capacity());
  std::vector<size_t> seen(problem.poses.size(), absent);
  const auto add_point = [&](size_t row, int match) {
    const auto& source = map.points()[row];
    solver::BASolver::Point point;
    point.id = source.id;
    point.initial = Eigen::Vector3d(source.position.x, source.position.y, source.position.z);
    if (summarized) point.history = source.geometry;
    point.begin = observations.size();
    for (size_t r = head[row]; r != absent; r = next[r]) {
      if (observer[r] != query_row && seen[observer[r]] == problem.points.size())
        throw std::runtime_error("Duplicate source-submap landmark observation");
      seen[observer[r]] = problem.points.size();
      observations.push_back({observer[r], observer[r] == query_row ? member_measurements[r] : history.observations[r].stereo});
      observation_captures.push_back(history.observations[r].node);
    }
    if (match >= 0) {
      point.required = observations.size();
      point.minimum = summarized ? 1 : 2;
      observations.push_back({query_row, current.back()});
      observation_captures.push_back(-1);
    }
    point.end = observations.size();
    problem.points.push_back(point);
    match_rows.push_back(match);
  };
  for (size_t i = 0; i < matches.size(); ++i) {
    const auto& m = matches[i];
    const auto& k = query.keypoints.at(m.feature);
    if (k.octave < 0 || size_t(k.octave) >= variances.size() || !count.at(m.landmark) || (!query_from_members.empty() && !fixed_support[m.landmark]))
      continue;
    if (problem.points.size() >= options.max_points) break;
    const camera::StereoObservation o{measurement_frame, m.feature};
    current.push_back(o);
    add_point(m.landmark, int(i));
  }
  if (problem.points.size() < minimum) {
    result.failure = "Insufficient node pixel tracks";
    result.status = solver::BASolver::Status::InsufficientSupport;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return result;
  }
  if (joint_history || !query_from_members.empty())
    for (size_t i = 0; i < map.points().size() && problem.points.size() < options.max_points; ++i) {
      if (matched[i] || count[i] < 2) continue;
      if (!query_from_members.empty()) {
        if (fixed_support[i] && source_support[i]) add_point(i, -1);
        continue;
      }
      bool selected = false;
      for (size_t r = head[i]; r != absent && !selected; r = next[r])
        selected = !problem.poses[observer[r]].fixed || problem.poses[observer[r]].id == reference_node;
      if (selected) add_point(i, -1);
    }
  std::vector<uint8_t> observed(problem.poses.size(), false);
  for (const auto& o : observations) observed[o.pose] = true;
  for (size_t i = 0; i < query_row; ++i)
    if (observed[i]) {
      ++result.views;
      if (problem.poses[i].fixed)
        ++result.fixed_views;
      else
        ++result.optimized_views;
    }
  result.optimized_views += !fixed_query;
  result.measurements = observations.size();
  result.optimized_points = problem.points.size();
  const auto solve_start = options.profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  const auto solved = ba.solve(problem, observations, solve_options, options.diagnostics);
  if (options.profile) solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count();
  result.status = solved.status;
  if (solved.status == solver::BASolver::Status::InvalidProblem) throw std::logic_error("Invalid node BA assembly: " + solved.message);
  if (solved.status != solver::BASolver::Status::Success) {
    result.failure = solved.message;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return result;
  }
  result.pose = solved.poses[query_row];
  if (solved.pose_covariances[query_row]) result.covariance = solved.pose_covariances[query_row]->matrix;
  if (joint_history) {
    result.observation_poses[reference_node] = StateGroupf::identity();
    for (size_t i = 0; i < query_row; ++i)
      if (solved.active_poses[i] && !problem.poses[i].fixed) result.observation_poses[problem.poses[i].id] = solved.poses[i];
  }
  for (const auto& [node, local] : query_from_members) result.observation_poses[node] = result.pose.compose(local);
  for (size_t row : solved.rejected_observations) {
    if (observation_captures[row] == -1) continue;
    const auto point = std::upper_bound(problem.points.begin(), problem.points.end(), row, [](size_t r, const auto& p) { return r < p.end; });
    result.rejected_observations.emplace_back(observation_captures[row], point->id);
  }
  const Eigen::Vector3d origin = result.pose.compose(camera.cameraToBase()).translationVector().cast<double>();
  for (size_t i = 0; i < problem.points.size(); ++i) {
    const auto& p = problem.points[i];
    if (!solved.active_points[i]) continue;
    auto covariance = solved.point_covariances[i];
    if (fixed_query_covariance && p.required && solved.active_observations[*p.required]) {
      // x* minimizes pixel error with query pose held at the independently
      // accepted measurement. dx*/d(delta_pose) = -Hxx^-1 Hxp. Propagate the
      // supplied right-tangent [translation, rotation] uncertainty, including
      // camera extrinsics; do not treat the reference pose as an exact datum.
      const auto& measurement = observations[*p.required].measurement.get();
      const auto body_from_camera = measurement.anchor_from_camera();
      const auto camera_from_body_rotation = body_from_camera.rotationMatrix().cast<double>().transpose().eval();
      const auto body_point = solved.poses[query_row].inverse().transformPoint(solved.points[i].cast<float>()).cast<double>().eval();
      const Eigen::Vector3d optical = camera_from_body_rotation * (body_point - body_from_camera.translationVector().cast<double>());
      const auto& k = measurement.calibration();
      const double iz = 1 / optical.z();
      Mat33_t<double> pixel = Mat33_t<double>::Zero();
      pixel.row(0) << k[0] * iz, 0, -k[0] * optical.x() * iz * iz;
      if (measurement.pixel().y >= 0) pixel.row(1) << k[0] * iz, 0, (k[4] - k[0] * optical.x()) * iz * iz;
      pixel.row(2) << 0, k[1] * iz, -k[1] * optical.y() * iz * iz;
      pixel /= measurement.sigma();
      Mat33_t<double> skew;
      skew << 0, -body_point.z(), body_point.y(), body_point.z(), 0, -body_point.x(), -body_point.y(), body_point.x(), 0;
      Eigen::Matrix<double, 3, 6> pose_jacobian;
      pose_jacobian.leftCols<3>() = -pixel * camera_from_body_rotation;
      pose_jacobian.rightCols<3>() = pixel * camera_from_body_rotation * skew;
      const Mat33_t<double> point_jacobian = pixel * camera_from_body_rotation * solved.poses[query_row].rotationMatrix().cast<double>().transpose();
      const Eigen::Matrix<double, 3, 6> sensitivity = -covariance * point_jacobian.transpose() * pose_jacobian;
      covariance.noalias() += sensitivity * *fixed_query_covariance * sensitivity.transpose();
    }
    bool finite_depth = true;
    if (joint_history || fixed_query_covariance || !fixed_query) {
      bool parallax = p.history.has_value();  // The joint covariance below still has to constrain depth.
      for (size_t a = p.begin; a < p.end && !parallax; ++a) {
        const auto& x = observations[a];
        if (!solved.active_observations[a]) continue;
        parallax = solver::Triangulator::stereo_depth_support(x.measurement.get());
        for (size_t b = p.begin; b < a && !parallax; ++b) {
          const auto& y = observations[b];
          if (!solved.active_observations[b]) continue;
          parallax =
              solver::Triangulator::temporal_depth_support(x.measurement.get(), solved.poses[x.pose], y.measurement.get(), solved.poses[y.pose]);
        }
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance, Eigen::EigenvaluesOnly);
      const auto& xyz = solved.points[i];
      finite_depth = parallax && covariance.allFinite() && eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0 &&
                     9 * eig.eigenvalues().maxCoeff() < (xyz - origin).squaredNorm();
    }
    if (finite_depth) {
      result.points.emplace(p.id, cv::Point3f(solved.points[i].x(), solved.points[i].y(), solved.points[i].z()));
      result.point_covariances.emplace(p.id, covariance);
    } else
      ++result.deferred_depths;
    if (match_rows[i] >= 0) result.inliers.push_back(match_rows[i]);
  }
  result.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  if (options.profile)
    EXT_LOG_INFO("[submap_ba_profile] reference={} points={} measurements={} total_ms={:.6f} prepare_finish_ms={:.6f} solve_ms={:.6f}",
                 reference_node, result.optimized_points, result.measurements, result.elapsed_ms, result.elapsed_ms - solve_ms, solve_ms);
  return result;
}

solver::PoseResult refine_stereo_pose(const LandmarkMap& map, const bow::FeatureBlock& query, const camera::Perspective& camera,
                                      const std::vector<Match>& matches, const StateGroupf& initial, const std::vector<double>& variances,
                                      size_t minimum) {
  if (!initial.isValid() || !camera.isValidForReprojection() || !camera.cameraToBase().isValid() || minimum < 6)
    throw std::invalid_argument("Invalid stereo loop input");
  solver::PoseResult result;
  const auto start = std::chrono::steady_clock::now();
  const auto finish = [&]() {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return std::move(result);
  };
  if (!(camera.focalXBaseline() > 0)) {
    result.failure = "No query stereo calibration";
    return finish();
  }
  solver::BASolver ba;
  solver::BASolver::Problem problem;
  std::vector<solver::BASolver::Observation> observations;
  solver::BASolver::Options solve_options;
  problem.poses = {{0, StateGroupf::identity(), true}, {1, initial, false, true}};
  solve_options.minimum_required_points = minimum;
  solve_options.max_iterations = 30;
  problem.points.reserve(matches.size());
  observations.reserve(matches.size() * 2);
  const auto measurement_frame = camera::stereo_frame(camera, query.keypoints, query.points, variances, true);
  std::vector<camera::StereoObservation> current;
  // One append per match at most; published references stay valid through solve().
  current.reserve(matches.size());
  std::vector<int> indices;
  indices.reserve(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    const auto& match = matches[i];
    const auto& point = map.points().at(match.landmark);
    if (!point.geometry) continue;
    const auto& k = query.keypoints.at(match.feature);
    if (k.octave < 0 || size_t(k.octave) >= variances.size()) throw std::invalid_argument("Invalid stereo loop octave");
    const camera::StereoObservation o{measurement_frame, match.feature};
    const size_t begin = observations.size();
    current.push_back(o);
    observations.push_back({1, current.back()});
    solver::BASolver::Point p{point.id, Eigen::Vector3d(point.position.x, point.position.y, point.position.z), begin, begin + 1, begin, 1};
    p.history = point.geometry;
    problem.points.push_back(std::move(p));
    indices.push_back(i);
  }
  result.available = problem.points.size();
  if (result.available < minimum) {
    result.failure = "Insufficient committed point summaries";
    return finish();
  }
  const auto solved = ba.solve(problem, observations, solve_options);
  if (solved.status != solver::BASolver::Status::Success) {
    result.failure = solved.message;
    return finish();
  }
  result.pose = solved.poses[1];
  if (solved.pose_covariances[1]) result.covariance = solved.pose_covariances[1]->matrix;
  for (size_t i = 0; i < problem.points.size(); ++i)
    if (solved.active_points[i]) result.inliers.push_back(indices[i]);
  return finish();
}
}  // namespace sapphire::backend::featmap
