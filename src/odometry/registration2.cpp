#include <sapphire/odometry/registration2.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <sapphire/backend/submap_arxiv.hpp>
#include <sapphire/utils.hpp>
#include <spdlog/spdlog.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <numeric>

namespace sapphire {
namespace {
constexpr double kGyroNoise = .01;
constexpr double kAccelNoise = .01;
constexpr double kGyroBiasNoise = .0001;
constexpr double kAccelBiasNoise = .0001;
constexpr int kIterations = 4;
constexpr double kStopStep = .001;

Isometry3d toIsometry(const FilterPose& pose) {
  Isometry3d result = Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(pose.R_.cast<double>()).normalized().toRotationMatrix();
  result.translation() = pose.t_.cast<double>();
  return result;
}

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd) {
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  } else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];

  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}

inline bool compute_error(const std::array<double, 4>& abcd, const V3& point, const float length, scalar& error) {
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}

}  // namespace

Registration2::VoxelSampler::VoxelSampler(float leaf_size) : voxel_size_(leaf_size), inv_voxel_size_(1.0f / leaf_size) {
  dist2_.reserve(10000);
  points_.reserve(10000);
  voxel_map_.reserve(10000);
}

void Registration2::VoxelSampler::filter(const PointCloud& input, PointCloud& output) {
  voxel_map_.clear();
  dist2_.clear();
  points_.clear();

  for (const auto& pt : input.points) {
    Eigen::Vector3f pf = pt.getVector3fMap();
    Eigen::Vector3i idx = (pf * inv_voxel_size_).array().round().cast<int>();
    Eigen::Vector3f center = voxel_size_ * idx.cast<float>();
    float d2 = (pf - center).squaredNorm();

    idx += offset_;  // Avoid negative indices
    const std::size_t key = ((std::size_t(idx[2])) << 30) | ((std::size_t(idx[1])) << 15) | (std::size_t(idx[0]));

    auto it = voxel_map_.find(key);
    if (it == voxel_map_.end()) {
      voxel_map_.emplace(key, points_.size());
      points_.push_back(pt);
      dist2_.push_back(d2);
    } else if (d2 < dist2_[it->second]) {
      points_[it->second] = pt;
      dist2_[it->second] = d2;
    }
  }

  output.points.swap(points_);
  output.width = output.points.size();
  output.height = 1;
  output.is_dense = true;
  output.header = input.header;
}

Registration2::Registration2(const Config& config, RevisitExchange& exchange, LocalPlaneExchange& local_planes)
    : exchange_(exchange),
      local_planes_(local_planes),
      map_({.5f, static_cast<std::size_t>(config.methodB.voxel_capacity)}),
      sampler_(static_cast<float>(config.odometry.voxel_size)),
      lidar_to_imu_(config.extrinsics.T_imu_lidar.linear().cast<float>().eval(), config.extrinsics.T_imu_lidar.translation().cast<float>().eval()),
      gravity_(config.imu.init.gravity_mag) {}

bool Registration2::process(MeasureGroup scan) {
  measures_ = std::move(scan);
  return (this->*state_)();
}

bool Registration2::track() {
  deskew();
  sampler_.filter(*deskewed_, *sampled_);
  observe();
  insertScan();
  return true;
}

bool Registration2::initializeFilter() {
  for (auto& imu : measures_.imu) {
    imu_count_++;
    mean_gyro_ += (imu.gyr - mean_gyro_) / imu_count_;
    const V3 residual = imu.acc - mean_accel_;
    mean_accel_ += residual / imu_count_;
    accel_variation_ += residual.dot(imu.acc - mean_accel_);
  }

  /// 100 Hz for 1 second.
  if (imu_count_ < 50 || mean_accel_.squaredNorm() < 1.e-8f || measures_.imu.empty()) {
    return false;
  }

  V3 gravity = -mean_accel_ * gravity_ / mean_accel_.norm();
  V3 ref_gravity(0, 0, -gravity_);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix();

  M3 rot = R_yaw_inv * init_rot;

  iEskf::Options options;
  options.gyro_var_ = kGyroNoise;
  options.acce_var_ = kAccelNoise;
  options.bias_gyro_var_ = kGyroBiasNoise;
  options.bias_acce_var_ = kAccelBiasNoise;
  options.num_iterations_ = kIterations;
  options.quit_eps_ = kStopStep;

  float imu_scale = gravity_ / mean_accel_.norm();
  accel_scale_ = imu_scale;
  filter_.SetInitialConditions(options, mean_gyro_, V3::Zero(), imu_scale, ref_gravity);
  auto state = filter_.GetSysState();
  state.R = FilterRotation(rot);
  state.timestamp = measures_.imu.back().secs;
  filter_.SetX(state);
  initial_pose_ = filter_.GetSE3();
  state_ = &Registration2::initializeMap;
  return false;
}

bool Registration2::initializeMap() {
  std::size_t ptsize = measures_.lidar.pc->size();
  world_points_.resize(ptsize);

  const FilterPose transform = initial_pose_ * lidar_to_imu_;

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
      auto& point_pcl = measures_.lidar.pc->points[idx];
      V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
      world_points_[idx] = transform * point_body;
    }
  });

  map_.insert(world_points_);
  filter_.SetLastObsTime(measures_.lidar.end_time);

  if (++initial_scans_ == 4) {
    filter_.StartTracking(measures_.end_imu);
    state_ = &Registration2::track;
  }
  return false;
}

void Registration2::deskew() {
  imu_states_.clear();
  imu_states_.emplace_back(filter_.GetDynamicState());
  filter_.SetObsTime(measures_.lidar.end_time);
  for (const auto& imu : measures_.imu) {
    if (filter_.Predict(imu)) imu_states_.emplace_back(filter_.GetDynamicState());
  }
  if (filter_.GetTime() < measures_.lidar.end_time && filter_.Predict(measures_.end_imu)) {
    imu_states_.emplace_back(filter_.GetDynamicState());
  }

  const M3 TLI_R = lidar_to_imu_.R_;
  const V3 TLI_t = lidar_to_imu_.t_;
  const FilterPose T_end = filter_.GetSE3();
  const M3 R_inv = T_end.R_.transpose();
  const V3 T_end_t = T_end.t_;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  if (!deskewed_.unique()) deskewed_ = std::make_shared<PointCloud>();
  deskewed_->resize(ptsize);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t;
    V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
      auto& pt_full = deskewed_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full = pt;
      const double query_time = std::max(pt.timestamp, imu_states_.front().time);
      if (query_time > imu_states_.back().time || imu_states_.size() < 2) {
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = TLI_R * raw + TLI_t;
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = imu_states_.begin();
      for (auto iter = imu_states_.begin(); iter != imu_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter != imu_states_.end() && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double tau = query_time - match_iter->time;
      double s = tau / dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 p_i = p_h + v_h * tau + 0.5 * acc_t * tau * tau;
      V3 t_ei = p_i - T_end_t;
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}

struct ThreadACC {
  Mat_66<double> HTVH = Mat_66<double>::Zero();
  Vec_6<double> HTVr = Vec_6<double>::Zero();
  size_t matched = 0;
  ThreadACC() : HTVH(Mat_66<double>::Zero()), HTVr(Vec_6<double>::Zero()) {}
};

void Registration2::observe() {
  size_t ptsize = sampled_->size();

  auto& lengths = point_ranges_;
  auto& effect_mask = plane_mask_;
  auto& effect_knn_mask = neighbor_mask_;
  effect_mask.resize(ptsize);
  effect_knn_mask.resize(ptsize);
  planes_.resize(ptsize);
  match_indices_.resize(ptsize);
  scan_points_.resize(ptsize);
  lengths.resize(ptsize);

  search_count_ = ptsize;
  std::iota(match_indices_.begin(), match_indices_.begin() + ptsize, 0);

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& point_body_pcl = sampled_->points[i];
    scan_points_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    lengths[i] = scan_points_[i].norm();
  }

  map_.reset_max_group();
  int iter_num = 0;
  const auto odom = toIsometry(filter_.GetSE3());
  const auto* archive = updateRevisit(measures_.lidar.end_time, odom, scan_points_);
  const auto* published = local_planes_.read_only();
  const auto* local = published && (published->center - odom.translation().cast<float>()).squaredNorm() <= 15.f * 15.f
                          ? published : nullptr;
  const auto* plane_map = archive ? archive : local;
  const Eigen::Isometry3f target_from_odom = archive ? target_from_odom_ : Eigen::Isometry3f::Identity();
  std::vector<const VoxelPlane*> targets(plane_map ? ptsize : 0, nullptr);
  filter_.UpdateObserve([&, this](const iEskf::KFState& kf_state, Mat_66<scalar>& HTVH, Vec_6<scalar>& HTVr) {
    const FilterPose pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, search_count_), [&](const tbb::blocked_range<size_t>& r) {
      KNNHeapType top_K;
      auto& local_acc = tls_acc.local();
      for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
        int idx = match_indices_[r_s];
        V3& point_body = scan_points_[idx];
        V3 point_world = pose * point_body;

        V3 point_target = point_world;
        if (plane_map) point_target = target_from_odom * point_world;
        if (!need_converge || iter_num == 0) {
          const VoxelPlane* target = nullptr;
          if (plane_map) {
            target = plane_map->lookup(point_target);
            targets[idx] = target;
          }
          if (target) {
            auto& abcd = planes_[idx];
            const V3 normal = target_from_odom.linear().transpose() * target->normal;
            abcd = {normal.x(), normal.y(), normal.z(), target->normal.dot(target_from_odom.translation() - target->center)};
            effect_knn_mask[idx] = true;
            effect_mask[idx] = true;
          } else {
            top_K.reset();
            map_.getTopK(point_world, top_K);
            if (top_K.count < 4) {
              effect_mask[idx] = false;
              effect_knn_mask[idx] = false;
              continue;
            }
            effect_knn_mask[idx] = true;
            effect_mask[idx] = calc_plane_coeff(top_K.count, top_K.points_, planes_[idx]);
          }
        }

        if (!effect_mask[idx]) continue;

        auto& abcd = planes_[idx];
        scalar error;
        const auto* target = plane_map ? targets[idx] : nullptr;
        effect_mask[idx] = (!target || supported(*target, point_target)) && compute_error(abcd, point_world, lengths[idx], error);
        if (!effect_mask[idx]) continue;

        {
          V3d normvec(abcd[0], abcd[1], abcd[2]);
          V3d nb = R_transpose * normvec;
          V3d point_body_d = point_body.cast<double>();
          Vec_6<double> J;
          J.head<3>() = point_body_d.cross(nb);
          J.tail<3>() = normvec;

          ++local_acc.matched;
          const double weight = target ? std::min(1000., 1. / (target->sigma * target->sigma)) : 1000.;
          local_acc.HTVH += J * weight * J.transpose();
          local_acc.HTVr -= J * weight * error;
        }
      }
    });

    Mat_66<double> sum_HTVH = Mat_66<double>::Zero();
    Vec_6<double> sum_HTVr = Vec_6<double>::Zero();
    matches_ = 0;
    for (const auto& local_acc : tls_acc) {
      matches_ += local_acc.matched;
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if (need_converge) return;

    int _effect_knn_num = 0;
    for (size_t i = 0; i < search_count_; ++i) {
      int idx = match_indices_[i];
      if (!effect_knn_mask[idx]) continue;
      match_indices_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "search_count_: " << search_count_ << ", _effect_knn_num: " << _effect_knn_num;
    search_count_ = _effect_knn_num;

    iter_num++;
  });
  local_planes_.releaseCandidate();
}

void Registration2::insertScan() {
  const size_t ptsize = sampled_->size();
  if (ptsize == 0) return;

  const auto pose = filter_.GetSE3();
  world_points_.resize(ptsize);

  const auto R = pose.R_;
  const auto t = pose.t_;

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = scan_points_[i];
    world_points_[i] = R * pt + t;
  }

  map_.insert(world_points_);
}

namespace {
using Pose = Eigen::Isometry3d;

constexpr double max_delay = 30.;
constexpr std::size_t verification_points = 896;
constexpr double rotation_scale = 10.;
constexpr double max_refinement = .80;

struct Fit {
  Mat_66<double> hessian = Mat_66<double>::Zero();
  Mat_61<double> gradient = Mat_61<double>::Zero();
  std::size_t samples = 0, matches = 0;
  double cost = 0., inlier_cost = 0.;
  bool coverage() const { return matches >= std::max<std::size_t>(32, samples / 6); }
};

Fit score(const GaussianVoxelMap& map, const Pose& pose, const VV3& scan, bool held_out, bool coarse = false) {
  Fit fit;
  const auto count = std::min(scan.size(), verification_points);
  constexpr double missing_cost = .18 * .18;
  for (std::size_t i = held_out; i < count; i += 2) {
    ++fit.samples;
    const auto point = scan[i * scan.size() / count].cast<double>().eval();
    const auto world = (pose * point).eval();
    const auto* plane = coarse ? map.coarseLookup(world.cast<float>()) : map.lookup(world.cast<float>());
    if (!plane) {
      fit.cost += missing_cost;
      continue;
    }
    const auto normal = plane->normal.cast<double>().eval();
    const double residual = normal.dot(world - plane->center.cast<double>());
    fit.cost += residual * residual;
    fit.inlier_cost += residual * residual;
    ++fit.matches;
    if (!held_out) {
      const double weight = .06 / std::max(.06, std::abs(residual));
      Mat_61<double> jacobian;
      jacobian.head<3>() = point.cross(pose.linear().transpose() * normal) / rotation_scale;
      jacobian.tail<3>() = normal;
      fit.hessian.noalias() += weight * jacobian * jacobian.transpose();
      fit.gradient.noalias() += weight * residual * jacobian;
    }
  }
  return fit;
}

struct RevisitFit {
  Pose corrected;
  const char* reason;
  std::size_t matches = 0;
  double before = 0., after = 0.;
};

RevisitFit refine(const GaussianVoxelMap& map, const Pose& predicted, const VV3& scan) {
  Pose pose = predicted;
  for (int i = 0; i < 5; ++i) {
    const auto score_at_pose = score(map, pose, scan, false, i < 2);
    if (!score_at_pose.coverage()) return {pose, i < 2 ? "coarse_coverage" : "coverage", score_at_pose.matches};
    const Eigen::SelfAdjointEigenSolver<Mat_66<double>> eigen(score_at_pose.hessian, Eigen::EigenvaluesOnly);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[0] <= .001 * eigen.eigenvalues()[5]) return {pose, "degenerate"};
    const Mat_61<double> delta = score_at_pose.hessian.ldlt().solve(-score_at_pose.gradient);
    if (!delta.allFinite() || delta.tail<3>().norm() > .30 || delta.head<3>().norm() > .60) return {pose, "refinement"};
    pose.translation() += delta.tail<3>();
    const auto rotation = (delta.head<3>() / rotation_scale).eval();
    const double angle = rotation.norm();
    if (angle > 1e-12) pose.linear() *= Eigen::AngleAxisd(angle, rotation / angle).toRotationMatrix();
    if ((pose.translation() - predicted.translation()).norm() > max_refinement ||
        Eigen::AngleAxisd(predicted.linear().transpose() * pose.linear()).angle() > .14)
      return {pose, "refinement"};
    if (i >= 2 && delta.norm() < .004) break;
  }
  const auto before = score(map, predicted, scan, true);
  const auto after = score(map, pose, scan, true);
  const double a = std::sqrt(before.cost / std::max<std::size_t>(1, before.samples));
  const double b = std::sqrt(after.cost / std::max<std::size_t>(1, after.samples));
  if (!after.coverage() || after.cost > 1.01 * before.cost || after.inlier_cost > .10 * .10 * after.matches)
    return {pose, "validation", after.matches, a, b};
  return {pose, nullptr, after.matches, a, b};
}
}  // namespace

void Registration2::rememberKeyframe(double stamp, const Isometry3d& pose) { keyframes_.push_back({stamp, pose, std::nullopt}); }

const GaussianVoxelMap* Registration2::updateRevisit(double stamp, const Isometry3d& odom, const VV3& scan) {
  while (!keyframes_.empty() && keyframes_.front().stamp < stamp - max_delay) keyframes_.pop_front();
  if (!keyframes_.empty() && !keyframes_.back().next_scan && !scan.empty()) {
    // The first later scan is outside the sealed submap. Retain the same
    // points that the verification routine would sample from that scan.
    auto& observation = keyframes_.back().next_scan.emplace(VerificationScan{stamp, odom, {}});
    const auto count = std::min(scan.size(), verification_points);
    observation.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) observation.points.push_back(scan[i * scan.size() / count]);
  }

  const auto* candidate = exchange_.read_only();
  if (candidate && candidate->source_id > attempted_) {
    const auto anchor = std::lower_bound(keyframes_.begin(), keyframes_.end(), candidate->anchor_stamp,
                                         [](const Keyframe& frame, double time) { return frame.stamp < time; });
    if (anchor == keyframes_.end() || anchor->stamp != candidate->anchor_stamp) {
      attempted_ = candidate->source_id;
      spdlog::debug("[methodB] verify_rejected {} {} {:.9f} missing_anchor", candidate->source_id, candidate->target_id, stamp);
    } else if (anchor->next_scan) {
      attempted_ = candidate->source_id;
      verifyRevisit(*candidate, stamp, *anchor);
    }
  }
  exchange_.releaseCandidate();
  if (active_ && (target_from_odom_ * odom.translation().cast<float>() - active_->map->center).squaredNorm() > 30.f * 30.f) {
    active_ = nullptr;
    exchange_.setActive(nullptr);
  }
  return active_ ? active_->map.get() : nullptr;
}

void Registration2::verifyRevisit(const RevisitTarget& target, double stamp, const Keyframe& anchor) {
  const auto& observation = *anchor.next_scan;
  const Pose predicted = target.global_from_target.inverse() * target.global_from_source * anchor.odom_pose.inverse() * observation.prediction;
  const auto fit = refine(*target.map, predicted, observation.points);
  if (!fit.reason) {
    const Pose local_from_odom = fit.corrected * observation.prediction.inverse();
    const Pose global_from_odom = target.global_from_target * local_from_odom;
    if (spdlog::should_log(spdlog::level::debug)) {
      spdlog::debug("[methodB] verify_scan {} {} {:.9f}", target.source_id, target.target_id, observation.stamp);
      const Eigen::Quaterniond graph(target.global_from_source.linear() * anchor.odom_pose.linear().transpose());
      const Eigen::Quaterniond correction(global_from_odom.linear());
      const Eigen::Quaterniond verify(
          (target.global_from_target * fit.corrected * predicted.inverse() * target.global_from_target.inverse()).linear());
      spdlog::debug(
          "[methodB] verify {} {} {:.9f} {:.9f} {} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} "
          "{:.9f} {:.9f} {:.6f} {:.6f} {}",
          target.source_id, target.target_id, stamp, target.anchor_stamp, version_ + 1, correction.x(), correction.y(), correction.z(),
          correction.w(), graph.x(), graph.y(), graph.z(), graph.w(), verify.x(), verify.y(), verify.z(), verify.w(), fit.before, fit.after,
          fit.matches);
    }
    global_from_odom_ = global_from_odom;
    target_from_odom_ = local_from_odom.cast<float>();
    ++version_;
    active_ = &target;
    exchange_.setActive(active_);
  } else {
    spdlog::debug("[methodB] verify_rejected {} {} {:.9f} {}", target.source_id, target.target_id, stamp, fit.reason);
  }
}

}  // namespace sapphire
