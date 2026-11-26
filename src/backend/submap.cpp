#include <sapphire/backend/submap.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <spdlog/spdlog.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sapphire/odometry/voxel_filter.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_map>

#include <sapphire/backend/submap_arxiv.hpp>
#include <sapphire/mapping/voxelmap.hpp>
#include <sapphire/utils.hpp>

namespace sapphire {
namespace {
using sapphire::BoundarySummary;
using sapphire::GaussianVoxelMap;
using sapphire::InertialFrame;
using Cloud = small_gicp::PointCloud;
using Tree = small_gicp::KdTree<Cloud>;
using Clock = std::chrono::steady_clock;

constexpr size_t min_frames = 8;
constexpr size_t max_frames = 20;
constexpr size_t cache_budget = 8 * 1024 * 1024;
constexpr double match_radius = 18.;
constexpr double departure_radius = 22.;
constexpr double deformation_budget_m = .10;

double elapsed(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }

gtsam::Pose3 toGtsam(const Isometry3d& pose) { return {gtsam::Rot3(pose.linear()), pose.translation()}; }
Isometry3d fromGtsam(const gtsam::Pose3& pose) {
  Isometry3d out = Isometry3d::Identity();
  out.linear() = pose.rotation().matrix();
  out.translation() = pose.translation();
  return out;
}
gtsam::Key frameKey(size_t submap, size_t frame) { return gtsam::Symbol('r', submap * max_frames + frame); }

struct Geometry {
  std::unique_ptr<GaussianVoxelMap> map;
  double score = 0.;
};
Geometry pack(const std::deque<InertialFrame>& frames, const std::vector<Isometry3d>& poses, size_t count, double range_squared) {
  auto map = std::make_unique<GaussianVoxelMap>();
  map->stamp = frames.front().frame.stamp;
  std::unordered_map<sapphire::VoxelKey, sapphire::Moments, sapphire::VoxelKeyHash> voxels;
  for (size_t i = 0; i < count; ++i) {
    const auto local = (poses.front().inverse() * poses[i]).matrix().cast<float>().eval();
    for (const auto& p : *frames[i].frame.cloud) {
      if (!p.getVector3fMap().allFinite() || p.getVector3fMap().squaredNorm() > range_squared) continue;
      const Eigen::Vector3f q = local.topLeftCorner<3, 3>() * p.getVector3fMap() + local.topRightCorner<3, 1>();
      voxels[sapphire::key(q)].add(q, i);
    }
  }
  map->reserve(voxels.size() / 4);
  Mat_66<double> information = Mat_66<double>::Zero();
  for (const auto& [key, stats] : voxels) {
    if (stats.frames < 2) continue;
    if (const auto voxel = stats.fit()) {
      map->insert(key, *voxel);
      const auto normal = voxel->plane.normal.cast<double>();
      const auto position = voxel->plane.center.cast<double>();
      Mat_61<double> j;
      j << position.cross(normal) / 10., normal;
      information.noalias() += j * j.transpose();
    }
  }
  const Eigen::SelfAdjointEigenSolver<Mat_66<double>> eig(information, Eigen::EigenvaluesOnly);
  const double score = eig.info() == Eigen::Success && eig.eigenvalues()[5] > 0. ? eig.eigenvalues()[0] / eig.eigenvalues()[5] : 0.;
  return {std::move(map), score};
}

struct MatchCloud {
  explicit MatchCloud(const GaussianVoxelMap& map) : cloud(std::make_shared<Cloud>()), normals(map.size()) {
    cloud->resize(map.size());
    for (size_t i = 0; i < map.size(); ++i) {
      const auto voxel = map.voxel(i);
      cloud->point(i) << voxel.plane.center.cast<double>(), 1.;
      normals[i] = voxel.plane.normal.cast<double>();
      const auto& a = voxel.gaussian.covariance;
      Eigen::Matrix3d cov;
      cov << a[0], a[1], a[2], a[1], a[3], a[4], a[2], a[4], a[5];
      cov.diagonal().array() += .0025;
      cloud->cov(i).setZero();
      cloud->cov(i).topLeftCorner<3, 3>() = cov;
    }
    tree = std::make_unique<Tree>(cloud);
  }
  std::shared_ptr<Cloud> cloud;
  std::unique_ptr<Tree> tree;
  std::vector<Eigen::Vector3d> normals;
  size_t accountedBytes() const { return 248 * cloud->size(); }
};

struct PlaneFit {
  Isometry3d pose = Isometry3d::Identity();
  Mat_66<double> info = Mat_66<double>::Zero();
  std::size_t inliers = 0;
  double overlap = 0.;
  double rms = 0.;
  bool converged = false;
};

struct Equations {
  Mat_66<double> h = Mat_66<double>::Zero();
  gtsam::Vector6 g = gtsam::Vector6::Zero();
  double cost = 0.;
  size_t support = 0;
  std::array<size_t, 2> directional{};
  std::array<double, 2> residual{};
};

Equations planeEquations(const MatchCloud& source, const MatchCloud& target, const Isometry3d& transform, double distance_sq = 1.) {
  Equations out;
  const auto increment = [&](const MatchCloud& from, const MatchCloud& to, const Isometry3d& mapping, bool reverse) {
    for (size_t i = 0; i < from.cloud->size(); ++i) {
      const Eigen::Vector3d point = from.cloud->point(i).head<3>();
      Eigen::Vector4d query;
      query << mapping * point, 1.;
      size_t closest;
      double distance2;
      if (!to.tree->nearest_neighbor_search(query, &closest, &distance2) || distance2 > distance_sq) continue;
      const auto& normal = to.normals[closest];
      if (std::abs(normal.dot(mapping.linear() * from.normals[i])) < .9) continue;
      const double residual = normal.dot(query.head<3>() - to.cloud->point(closest).head<3>());
      if (std::abs(residual) > .30) continue;
      gtsam::Vector6 j;
      if (reverse) {
        j << -query.head<3>().cross(normal), -normal;
      } else {
        const Eigen::Vector3d axis = mapping.linear().transpose() * normal;
        j << point.cross(axis), axis;
      }
      const double weight = .08 / std::max(.08, std::abs(residual));
      out.h.noalias() += weight * j * j.transpose();
      out.g.noalias() += weight * residual * j;
      out.cost += weight * residual * residual;
      ++out.support;
      ++out.directional[reverse];
      out.residual[reverse] += weight * residual * residual;
    }
  };
  increment(source, target, transform, false);
  increment(target, source, transform.inverse(), true);
  return out;
}

PlaneFit alignPlanes(const MatchCloud& source, const MatchCloud& target, const gtsam::Pose3& guess, double distance_sq = 1.) {
  PlaneFit fit;
  auto pose = guess;
  auto current = planeEquations(source, target, fromGtsam(pose), distance_sq);
  for (int iteration = 0; iteration < 10; ++iteration) {
    if (current.support < 120) break;
    const Eigen::LDLT<Mat_66<double>> factor(current.h);
    if (factor.info() != Eigen::Success || !factor.isPositive()) break;
    const gtsam::Vector6 step = factor.solve(-current.g);
    if (!step.allFinite() || step.head<3>().norm() > .15 || step.tail<3>().norm() > .8) break;
    if (step.head<3>().norm() < .005 && step.tail<3>().norm() < .01) {
      fit.converged = true;
      break;
    }
    const auto candidate = pose.retract(step);
    auto updated = planeEquations(source, target, fromGtsam(candidate), distance_sq);
    // Correspondence turnover must retain most of the geometric support.
    if (updated.support < .9 * current.support || updated.cost * current.support > 1.02 * current.cost * updated.support) break;
    pose = candidate;
    current = std::move(updated);
  }
  fit.pose = fromGtsam(pose);
  fit.info = current.h;
  fit.inliers = current.support;
  fit.overlap = double(current.support) / (source.cloud->size() + target.cloud->size());
  fit.rms = current.support ? std::sqrt(current.cost / current.support) : 0.;
  return fit;
}

std::optional<double> floorLevel(const MatchCloud& map, const Eigen::Matrix3d& rotation) {
  std::vector<double> floor;
  for (std::size_t i = 0; i < map.cloud->size(); ++i) {
    const Eigen::Vector3d p = rotation * map.cloud->point(i).head<3>();
    if (p.head<2>().squaredNorm() < 144. && std::abs(rotation.row(2).dot(map.normals[i])) > .85) floor.push_back(p.z());
  }
  if (floor.size() < 12) return {};
  const auto quartile = floor.begin() + floor.size() / 4;
  std::nth_element(floor.begin(), quartile, floor.end());
  return *quartile;
}

PlaneFit alignGravityPlanes(const MatchCloud& source, const MatchCloud& target, const Isometry3d& initial, const Isometry3d& source_anchor,
                            const Isometry3d& target_anchor) {
  auto start = initial;
  const auto source_floor = floorLevel(source, source_anchor.linear());
  const auto target_floor = floorLevel(target, target_anchor.linear());
  if (source_floor && target_floor) {
    Eigen::Vector3d delta = target_anchor.linear() * start.translation();
    delta.z() = *target_floor - *source_floor;
    start.translation() = target_anchor.linear().transpose() * delta;
  }
  Eigen::Matrix<double, 6, 4> basis = Eigen::Matrix<double, 6, 4>::Zero();
  basis.topLeftCorner<3, 1>() = source_anchor.linear().transpose() * Eigen::Vector3d::UnitZ();
  basis.bottomRightCorner<3, 3>().setIdentity();
  auto pose = toGtsam(start);
  for (int step = 0; step < 10; ++step) {
    const auto state = planeEquations(source, target, fromGtsam(pose), 1.5625);
    if (state.support < 120) break;
    const Eigen::Matrix4d h = basis.transpose() * state.h * basis;
    const Eigen::LDLT<Eigen::Matrix4d> llt(h);
    if (llt.info() != Eigen::Success || !llt.isPositive()) break;
    const Eigen::Vector4d delta = llt.solve(-basis.transpose() * state.g);
    if (!delta.allFinite() || std::abs(delta[0]) > .12 || delta.tail<3>().norm() > 1.) break;
    pose = pose.retract(basis * delta);
    if (delta.norm() < .012) break;
  }
  const auto measure = [&](const Isometry3d& value) {
    PlaneFit fit;
    fit.pose = value;
    const auto state = planeEquations(source, target, value, 1.5625);
    if (!state.directional[0] || !state.directional[1]) return fit;
    fit.info = state.h;
    fit.inliers = state.support;
    fit.overlap = std::min(double(state.directional[0]) / source.cloud->size(), double(state.directional[1]) / target.cloud->size());
    fit.rms = std::max(std::sqrt(state.residual[0] / state.directional[0]), std::sqrt(state.residual[1] / state.directional[1]));
    const auto delta = toGtsam(start).between(toGtsam(value));
    fit.converged = fit.overlap >= .53 && fit.rms <= .085 && delta.translation().norm() < 2.5 && gtsam::Rot3::Logmap(delta.rotation()).norm() < .18;
    return fit;
  };
  auto fit = measure(fromGtsam(pose));
  if (fit.inliers >= 120 && fit.overlap >= .30) {
    // The coarse pass supplies gravity alignment. Independently observed
    // planes can refine a small roll/pitch error in that initial alignment.
    const auto spatial = alignPlanes(source, target, pose, 1.5625);
    const auto correction = pose.between(toGtsam(spatial.pose));
    const Eigen::Vector3d turn = gtsam::Rot3::Logmap(correction.rotation());
    const Eigen::Vector3d vertical = basis.topLeftCorner<3, 1>();
    if (turn.cross(vertical).norm() < .06 && correction.translation().norm() < .5) {
      auto refined = measure(spatial.pose);
      if (refined.converged && refined.overlap >= .9 * fit.overlap && refined.rms < fit.rms) fit = std::move(refined);
    }
  }
  return fit;
}

struct SealCandidate {
  std::vector<Isometry3d> poses;
  std::vector<sapphire::InertialEstimate> nav;
  Geometry geometry;
  BoundarySummary boundary;
  sapphire::BaStats ba;
  size_t count = 0;
  bool certified = false;
};

struct SealedSubmap {
  Isometry3d anchor;
  std::vector<std::pair<double, Isometry3d>> local;
  bool departed = false;
  size_t cells = 0;
  bool retains_frames = false;
  std::size_t first_frame = 0;
};

}  // namespace

class Submap::Impl {
 public:
  Impl(SubmapArxiv& arxiv, const Config::MethodB& config)
      : arxiv_(arxiv),
        map_range_squared_(config.map_range_m * config.map_range_m),
        local_feedback_(config.local_feedback),
        local_feedback_ba_(config.local_feedback_ba) {}

  void build(InertialFrame&& frame) {
    if (!pending_.empty()) throw std::logic_error("cannot build over an active submap");
    append(std::move(frame));
  }

  bool update(InertialFrame&& frame) {
    if (pending_.empty()) {
      build(std::move(frame));
      return false;
    }
    if (pending_.size() > max_frames) throw std::logic_error("seal the full submap before updating it");
    append(std::move(frame));
    return pending_.size() == max_frames + 1;
  }

  void seal() {
    if (pending_.empty()) return;
    const bool has_next = pending_.size() == max_frames + 1;
    sealWindow(has_next ? max_frames : pending_.size(), !has_next);
  }

  void reset() {
    if (pending_.size() > 1) pending_.erase(pending_.begin(), pending_.end() - 1);
  }

  void clear() { pending_.clear(); }

  PoseGraphStats stats() const { return stats_; }
  LocalPlaneExchange& localPlanes() { return local_planes_; }

  PoseGraphSnapshot snapshot() const {
    PoseGraphSnapshot result;
    result.stats = stats_;
    for (const auto& [stamp, pose] : optimizedFrames()) result.optimized_poses.push_back(pose);
    result.loop_edges = loop_edges_;
    return result;
  }

  std::vector<std::pair<double, Isometry3d>> optimizedFrames() const {
    std::vector<std::pair<double, Isometry3d>> result;
    result.reserve(stats_.optimized_poses);
    const auto estimate = solver_.calculateEstimate();
    for (std::size_t id = 0; id < submaps_.size(); ++id) {
      const auto& submap = submaps_[id];
      const auto global = fromGtsam(estimate.at<gtsam::Pose3>(id));
      for (std::size_t i = 0; i < submap.local.size(); ++i) {
        const auto& [stamp, local] = submap.local[i];
        const Isometry3d pose = submap.retains_frames && i > 0 ? fromGtsam(estimate.at<gtsam::Pose3>(frameKey(id, i))) : global * local;
        result.emplace_back(stamp, pose);
      }
    }
    return result;
  }

  PointCloudPtr globalMap(double leaf) const {
    auto cloud = std::make_shared<PointCloud>();
    const auto estimate = solver_.calculateEstimate();
    for (std::size_t id = 0; id < submaps_.size(); ++id) {
      if (!submaps_[id].cells) continue;
      const auto pose = fromGtsam(estimate.at<gtsam::Pose3>(id));
      const auto geometry = arxiv_.read(id);
      for (std::size_t i = 0; i < geometry->size(); ++i) {
        Point point{};
        point.getVector3fMap() = (pose * geometry->voxel(i).plane.center.cast<double>()).cast<float>();
        point.data[3] = 1.f;
        point.timestamp = submaps_[id].local.front().first;
        cloud->push_back(point);
      }
    }
    return deterministicVoxelDownsample(*cloud, leaf);
  }

 private:
  void append(InertialFrame&& frame) {
    checkFrame(frame);
    if (local_feedback_) updateLocalPlanes(frame);
    pending_.push_back(std::move(frame));
  }

  // Recent planes stay in odom coordinates so the frontend can read them
  // independently of pose-graph corrections.
  void updateLocalPlanes(const InertialFrame& frame) {
    constexpr size_t stride = 2;
    constexpr size_t window = 8;
    constexpr size_t fixed_limit = 16;
    local_frames_.push_back(frame);
    if (++local_frames_since_publish_ < stride) return;
    local_frames_since_publish_ = 0;

    if (local_frames_.size() >= window) {
      if (local_feedback_ba_) optimizeLocalFrames();
      for (size_t i = 0; i < stride; ++i) {
        local_fixed_.push_back(std::move(local_frames_.front().frame));
        local_frames_.pop_front();
      }
      while (local_fixed_.size() > fixed_limit) local_fixed_.pop_front();
    }

    auto map = std::make_unique<GaussianVoxelMap>();
    map->center = frame.frame.odom.translation().cast<float>();
    map->stamp = frame.frame.stamp;
    std::unordered_map<VoxelKey, Moments, VoxelKeyHash> cells;
    size_t frame_id = 0;
    const auto add = [&](const MappingFrame& source) {
      const auto pose = source.odom.cast<float>();
      if ((pose.translation() - map->center).squaredNorm() > 30.f * 30.f) return;
      ++frame_id;
      for (const auto& point : *source.cloud) {
        const Eigen::Vector3f world = pose * point.getVector3fMap();
        if ((world - map->center).squaredNorm() <= 35.f * 35.f) cells[key(world)].add(world, frame_id);
      }
    };
    for (const auto& fixed : local_fixed_) add(fixed);
    if (local_fixed_.size() < 6) {
      for (const auto& recent : local_frames_) add(recent.frame);
    }
    map->reserve(cells.size() / 2);
    for (const auto& [cell, stats] : cells) {
      if (stats.frames < 2) continue;
      if (const auto plane = stats.fit()) map->insert(cell, *plane);
    }
    if (map->size() == 0) return;
    map->version = ++local_version_;
    spdlog::debug("[methodB] local_planes {} {:.9f} {}", map->version, map->stamp, map->size());
    local_planes_.publish(std::move(map));
  }

  void optimizeLocalFrames() {
    std::vector<Isometry3d> poses;
    std::vector<InertialEstimate> estimates;
    poses.reserve(local_frames_.size());
    estimates.reserve(local_frames_.size());
    for (const auto& recent : local_frames_) {
      poses.push_back(recent.frame.odom);
      estimates.push_back(recent.nav);
    }
    adjustInertial({local_frames_.begin(), local_frames_.end()}, {local_fixed_.begin(), local_fixed_.end()}, poses, estimates);
    for (size_t i = 0; i < local_frames_.size(); ++i) {
      local_frames_[i].frame.odom = poses[i];
      local_frames_[i].nav = estimates[i];
    }
  }

  void checkFrame(const InertialFrame& frame) const {
    if (!std::isfinite(frame.frame.stamp) || !frame.frame.odom.matrix().allFinite() || !frame.frame.cloud ||
        (!pending_.empty() && frame.frame.stamp <= pending_.back().frame.stamp)) {
      throw std::runtime_error("invalid or unordered mapping frame");
    }
  }

  SealCandidate evaluate(size_t count, bool final) {
    const size_t states = count + (final ? 0 : 1);
    std::vector<InertialFrame> frames(pending_.begin(), pending_.begin() + states);
    SealCandidate candidate;
    candidate.count = count;
    for (const auto& frame : frames) {
      candidate.poses.push_back(frame.frame.odom);
      candidate.nav.push_back(frame.nav);
    }
    if (frames.size() >= min_frames) candidate.ba = sapphire::adjustInertial(frames, {}, candidate.poses, candidate.nav, &candidate.boundary);
    const bool admits_compression = candidate.boundary.valid && candidate.boundary.interior_shift_m <= deformation_budget_m;
    if (!admits_compression && candidate.ba.updates) {
      // A failed boundary certificate falls back before packing. The
      // individual poses and geometry then describe exactly the same scans.
      for (std::size_t i = 0; i < frames.size(); ++i) {
        candidate.poses[i] = frames[i].frame.odom;
        candidate.nav[i] = frames[i].nav;
      }
      candidate.ba.updates = 0;
    }
    candidate.geometry = pack(pending_, candidate.poses, count, map_range_squared_);
    candidate.certified = admits_compression && candidate.geometry.score >= .008 && candidate.geometry.map->size() >= 160;
    return candidate;
  }

  void sealWindow(size_t count, bool final) {
    auto candidate = evaluate(count, final);
    auto& poses = candidate.poses;
    auto& nav = candidate.nav;
    auto& geometry = candidate.geometry;
    auto& boundary = candidate.boundary;
    const bool certified = candidate.certified;
    const size_t id = submaps_.size();
    const bool forced = !certified;
    geometry.map->version = id;
    const auto start = poses.front();
    SealedSubmap map{start, {}, false, geometry.map->size(), forced, stats_.optimized_poses};
    for (size_t i = 0; i < count; ++i) map.local.emplace_back(pending_[i].frame.stamp, start.inverse() * poses[i]);
    arxiv_.write(id, *geometry.map);

    gtsam::NonlinearFactorGraph odom;
    gtsam::Values initial;
    auto seed = toGtsam(start);
    if (id == 0) {
      odom.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(id, seed, gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
    } else {
      const auto relative = toGtsam(last_boundary_pose_.inverse() * start);
      odom.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(last_boundary_key_, id, relative,
                                                              gtsam::noiseModel::Gaussian::Covariance(last_edge_cov_));
      seed = solver_.calculateEstimate<gtsam::Pose3>(last_boundary_key_).compose(relative);
    }
    initial.insert(id, seed);
    gtsam::Key boundary_key = id;
    size_t boundary_index = 0;
    if (forced && count > 1) {
      // An uncertified block keeps its individual poses and odometry factors.
      // Its Gaussian block still supplies a proposal for loop registration.
      auto frame_seed = seed;
      const auto noise = gtsam::noiseModel::Gaussian::Covariance(relativeOdometryCovariance(poses, 0, 2));
      for (size_t i = 1; i < count; ++i) {
        const auto relative = toGtsam(poses[i - 1].inverse() * poses[i]);
        const auto key = frameKey(id, i);
        odom.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(boundary_key, key, relative, noise);
        frame_seed = frame_seed.compose(relative);
        initial.insert(key, frame_seed);
        boundary_key = key;
      }
      boundary_index = count - 1;
    }
    const auto start_update = Clock::now();
    solver_.update(odom, initial);
    stats_.isam_odometry_ms += elapsed(start_update);
    submaps_.push_back(std::move(map));
    stats_.optimized_poses += count;
    stats_.glc_summaries += certified;
    ++stats_.revision;
    if (geometry.map->size() >= 72) match(id, seed, *geometry.map);
    const auto proposals = arxiv_.indexAndQuery(*geometry.map, start);
    if (geometry.map->size() >= 72) globalMatch(id, *geometry.map, proposals, !final);
    stats_.solver_poses = solver_.getLinearizationPoint().size();
    stats_.solver_factors = solver_.getFactorsUnsafe().size();
    if (!final) {
      // Transport the same per-keyframe Pose3 noise used by Sapphire main
      // into the next anchor's right tangent before applying the BA summary.
      last_boundary_key_ = boundary_key;
      last_boundary_pose_ = poses[boundary_index];
      last_edge_cov_ = relativeOdometryCovariance(poses, boundary_index, poses.size());
      if (boundary.valid && certified) {
        const double marginal = boundary.covariance.bottomRightCorner<3, 3>().diagonal().maxCoeff();
        const double reference = last_edge_cov_.bottomRightCorner<3, 3>().diagonal().maxCoeff();
        last_edge_cov_ += (reference / std::max(reference, marginal)) * boundary.covariance;
      }
    }
    if (final) {
      clear();
    } else {
      pending_.back().frame.odom = poses[count];
      pending_.back().nav = nav[count];
      reset();
    }
  }

  const MatchCloud& hot(size_t id) {
    const auto hit = std::find_if(cache_.begin(), cache_.end(), [id](const auto& e) { return e.first == id; });
    if (hit != cache_.end()) {
      std::rotate(cache_.begin(), hit, std::next(hit));
      return *cache_.front().second;
    }
    const auto geometry = arxiv_.read(id);
    auto cloud = std::make_unique<MatchCloud>(*geometry);
    while (!cache_.empty() && cache_bytes_ + cloud->accountedBytes() > cache_budget) {
      cache_bytes_ -= cache_.back().second->accountedBytes();
      cache_.pop_back();
    }
    cache_bytes_ += cloud->accountedBytes();
    cache_.emplace_front(id, std::move(cloud));
    return *cache_.front().second;
  }

  void match(size_t id, const gtsam::Pose3& seed, const GaussianVoxelMap& source_map) {
    std::vector<std::pair<double, size_t>> ranked;
    const auto estimate = solver_.calculateEstimate();
    for (size_t old = 0; old + 1 < id; ++old) {
      const auto pose = estimate.at<gtsam::Pose3>(old);
      const double squared = (seed.translation() - pose.translation()).squaredNorm();
      if (squared > departure_radius * departure_radius) submaps_[old].departed = true;
      if (submaps_[old].departed && squared <= match_radius * match_radius && submaps_[old].cells >= 72 && submaps_[old].cells <= cache_budget / 248)
        ranked.emplace_back(squared, old);
    }
    std::sort(ranked.begin(), ranked.end());
    if (ranked.size() > 3) ranked.resize(3);
    if (ranked.empty()) return;
    const MatchCloud source(source_map);
    for (const auto& [distance, old] : ranked) {
      ++stats_.loop_candidates;
      const auto& target = hot(old);
      const auto guess = estimate.at<gtsam::Pose3>(old).between(estimate.at<gtsam::Pose3>(id));
      const auto fit = alignPlanes(source, target, guess);
      const double overlap = fit.overlap;
      bool accepted = false;
      double shift = 0., turn = 0.;
      if (fit.converged && overlap >= .45 && fit.inliers >= 60 && fit.pose.matrix().allFinite()) {
        const Eigen::SelfAdjointEigenSolver<Mat_66<double>> h((fit.info + fit.info.transpose()) * .5);
        const auto motion = toGtsam(fit.pose);
        const auto correction = guess.between(motion);
        shift = correction.translation().norm();
        turn = gtsam::Rot3::Logmap(correction.rotation()).norm();
        if (h.info() == Eigen::Success && h.eigenvalues()[0] > 1e-6 * h.eigenvalues()[5] && shift <= 3. && turn <= .24) {
          gtsam::Vector6 sigmas;
          sigmas << .09, .09, .09, .35, .35, .35;
          Mat_66<double> covariance = h.eigenvectors() *
                               (std::max(.0025, fit.rms * fit.rms) * double(fit.inliers) / 64. * h.eigenvalues().cwiseInverse()).asDiagonal() *
                               h.eigenvectors().transpose();
          covariance.diagonal().array() += sigmas.array().square();
          const auto noise = gtsam::noiseModel::Gaussian::Covariance(covariance);
          gtsam::NonlinearFactorGraph loop;
          loop.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(old, id, motion, noise);
          auto trial = solver_;
          trial.update(loop);
          trial.update();
          const auto optimized = trial.calculateEstimate();
          const double error = trial.getFactorsUnsafe().error(optimized);
          if (std::isfinite(error) && error <= solver_.getFactorsUnsafe().error(estimate) + 24.) {
            if (spdlog::should_log(spdlog::level::debug)) {
              const auto prior = estimate.at<gtsam::Pose3>(id);
              const auto posterior = optimized.at<gtsam::Pose3>(id);
              const auto turn = gtsam::Rot3::Logmap(posterior.rotation().compose(prior.rotation().inverse()));
              spdlog::debug("[methodB] edge local {} {} {:.9f} {:.6f} {:.6f} {:.6f}", id, old, submaps_[id].local.back().first, turn.x(), turn.y(),
                            turn.z());
            }
            solver_ = std::move(trial);
            loop_edges_.push_back({submaps_[id].first_frame, submaps_[old].first_frame});
            ++stats_.loop_edges;
            accepted = true;
          }
        }
      }
      if (!accepted) ++stats_.loop_registration_failures;
      if (accepted) break;
    }
  }

  void publishRevisit(std::size_t source, std::size_t target) {
    const auto& submap = submaps_[source];
    const auto& [stamp, relative] = submap.local.back();
    // The archive retrieval already requires leaving the target area. Keep
    // near-in-time associations available to PGO, without feeding them back.
    if (stamp - submaps_[target].local.back().first < 45.) return;

    const auto estimate = solver_.calculateEstimate();
    const std::size_t tail = submap.local.size() - 1;
    const Isometry3d source_pose = submap.retains_frames && tail > 0 ? fromGtsam(estimate.at<gtsam::Pose3>(frameKey(source, tail)))
                                                                     : fromGtsam(estimate.at<gtsam::Pose3>(source)) * relative;

    // Use the sealed tail as the center so the same loop selects the same
    // historical neighborhood regardless of the producer's scheduling.
    const Eigen::Vector3d center = source_pose.translation();
    std::vector<std::pair<double, std::size_t>> ranking;
    for (std::size_t id = 0; id < source; ++id) {
      const auto& historical = submaps_[id];
      if (id == target || historical.cells == 0 || historical.cells > cache_budget / 248 || stamp - historical.local.back().first < 45.) continue;
      const Isometry3d anchor = fromGtsam(estimate.at<gtsam::Pose3>(id));
      const auto middle = historical.local.size() / 2;
      const auto distance = [&](std::size_t i) { return (anchor * historical.local[i].second.translation() - center).squaredNorm(); };
      const double nearest = std::min({distance(0), distance(middle), distance(historical.local.size() - 1)});
      if (nearest <= 35. * 35.) ranking.emplace_back(nearest, id);
    }
    const auto count = std::min<std::size_t>(ranking.size(), 5);
    std::partial_sort(ranking.begin(), ranking.begin() + count, ranking.end());
    std::vector<RevisitBlock> blocks;
    blocks.reserve(1 + count);
    blocks.push_back({target, fromGtsam(estimate.at<gtsam::Pose3>(target))});
    for (std::size_t i = 0; i < count; ++i) {
      const auto id = ranking[i].second;
      blocks.push_back({id, fromGtsam(estimate.at<gtsam::Pose3>(id))});
    }
    arxiv_.publishRevisit(source, stamp, source_pose, blocks);
  }

  void globalMatch(std::size_t id, const GaussianVoxelMap& geometry, const std::vector<PlaceProposal>& proposals, bool live) {
    if (proposals.empty()) return;
    const MatchCloud source(geometry);
    for (const auto& proposal : proposals) {
      const auto target_id = proposal.target;
      if (submaps_[target_id].cells < 72 || submaps_[target_id].cells > cache_budget / 248) continue;
      ++stats_.loop_candidates;
      const auto& target = hot(target_id);
      const auto fit = alignGravityPlanes(source, target, proposal.relative, submaps_[id].anchor, submaps_[target_id].anchor);
      bool accepted = false;
      if (fit.converged) {
        const auto old = solver_.calculateEstimate();
        const auto measurement = toGtsam(fit.pose);
        gtsam::Vector6 sigmas;
        sigmas << .07, .07, .035, .22, .22, .35;
        gtsam::NonlinearFactorGraph update;
        update.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(target_id, id, measurement, gtsam::noiseModel::Diagonal::Sigmas(sigmas));
        try {
          auto trial = solver_;
          gtsam::ISAM2UpdateParams params;
          params.force_relinearize = true;
          trial.update(update, {}, params);
          for (int k = 0; k < 8; ++k) trial.update({}, {}, params);
          const auto result = trial.calculateEstimate();
          const double cost = trial.getFactorsUnsafe().error(result);
          if (std::isfinite(cost) && update.error(result) < 8. &&
              cost <= solver_.getFactorsUnsafe().error(old) + std::max(24., .04 * submaps_.size())) {
            if (spdlog::should_log(spdlog::level::debug)) {
              const auto prior = old.at<gtsam::Pose3>(id);
              const auto posterior = result.at<gtsam::Pose3>(id);
              const auto measured = old.at<gtsam::Pose3>(target_id).compose(measurement);
              const auto innovation = gtsam::Rot3::Logmap(measured.rotation().compose(prior.rotation().inverse()));
              const auto turn = gtsam::Rot3::Logmap(posterior.rotation().compose(prior.rotation().inverse()));
              spdlog::debug("[methodB] edge global {} {} {:.9f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}", id, target_id,
                            submaps_[id].local.back().first, turn.x(), turn.y(), turn.z(), innovation.x(), innovation.y(), innovation.z(),
                            fit.overlap, fit.rms);
            }
            solver_ = std::move(trial);
            loop_edges_.push_back({submaps_[id].first_frame, submaps_[target_id].first_frame});
            ++stats_.loop_edges;
            accepted = true;
          }
        } catch (const std::exception&) {
          // The committed graph is still usable after an invalid proposal.
        }
      }
      if (!accepted) ++stats_.loop_registration_failures;
      if (accepted) {
        if (live) publishRevisit(id, target_id);
        break;
      }
    }
  }

  SubmapArxiv& arxiv_;
  const double map_range_squared_;
  const bool local_feedback_, local_feedback_ba_;
  LocalPlaneExchange local_planes_;
  std::deque<InertialFrame> local_frames_;
  std::deque<MappingFrame> local_fixed_;
  size_t local_frames_since_publish_ = 0, local_version_ = 0;
  PoseGraphStats stats_;
  std::vector<PoseGraphEdge> loop_edges_;
  std::deque<InertialFrame> pending_;
  std::vector<SealedSubmap> submaps_;
  gtsam::Key last_boundary_key_ = 0;
  Isometry3d last_boundary_pose_ = Isometry3d::Identity();
  Mat_66<double> last_edge_cov_ = Mat_66<double>::Identity();
  gtsam::ISAM2 solver_;
  std::deque<std::pair<size_t, std::unique_ptr<MatchCloud>>> cache_;
  size_t cache_bytes_ = 0;
};

Submap::Submap(SubmapArxiv& arxiv, const Config::MethodB& config) : impl_(std::make_unique<Impl>(arxiv, config)) {}
Submap::~Submap() = default;
LocalPlaneExchange& Submap::localPlanes() { return impl_->localPlanes(); }
void Submap::build(InertialFrame frame) { impl_->build(std::move(frame)); }
bool Submap::update(InertialFrame frame) { return impl_->update(std::move(frame)); }
void Submap::seal() { impl_->seal(); }
void Submap::reset() { impl_->reset(); }
void Submap::clear() { impl_->clear(); }
PoseGraphStats Submap::stats() const { return impl_->stats(); }
PoseGraphSnapshot Submap::snapshot() const { return impl_->snapshot(); }
std::vector<std::pair<double, Isometry3d>> Submap::optimizedFrames() const { return impl_->optimizedFrames(); }
PointCloudPtr Submap::globalMap(double leaf) const { return impl_->globalMap(leaf); }

}  // namespace sapphire
