#include "pipeline/frontend.hpp"
#include "slam.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace sapphire;

namespace {
StateGroup state(double t = 1) {
  StateGroup x;
  x.t = t;
  x.g = Eigen::Vector3d(0, 0, -9.81);
  x.cov.setIdentity();
  x.cov *= .01;
  return x;
}

PointCloudPtr planes(int axes = 3) {
  auto points = std::make_shared<PointCloud>();
  // Three separated planar patches; no association ambiguity or mixed voxels.
  for (int axis = 0; axis < axes; ++axis) {
    for (int i = 0; i < 9; ++i) for (int j = 0; j < 9; ++j) {
      pointVar p;
      p.pnt.setConstant(.5);
      p.pnt[axis] = 1.5 + axis;
      p.pnt[(axis + 1) % 3] = .18 + i * .08;
      p.pnt[(axis + 2) % 3] = .18 + j * .08;
      p.var = Eigen::Matrix3d::Identity() * .0001;
      points->push_back(p);
    }
  }
  return points;
}

void same(const StateGroup &a, const StateGroup &b) {
  assert(a.t == b.t);
  assert((a.R.array() == b.R.array()).all());
  assert((a.p.array() == b.p.array()).all());
  assert((a.v.array() == b.v.array()).all());
  assert((a.bg.array() == b.bg.array()).all());
  assert((a.ba.array() == b.ba.array()).all());
  assert((a.g.array() == b.g.array()).all());
  assert((a.cov.array() == b.cov.array()).all());
}

void seed(VoxelMap &map, const PointCloudPtr &points, int window,
          std::vector<StateGroup> &states, LidarFactor &factor) {
  vvec<double, 3> world;
  pvec_update(points, states.back(), world);
  map.cut_voxel_multi(points, 0, window, world);
  map.recut_multi(1, states, .0025, {1, 1, 1, 1});
  map.collect_factors(factor);
  // Matching-plane center/normal/covariance are refreshed by marginalization.
  // A zero-sized marginalization keeps this accepted frame in the window.
  map.marginalize(0, 1, 0, states);
}

std::deque<ImuMeas> interval(double begin, double end) {
  std::deque<ImuMeas> result;
  for (int i = 0; i <= 10; ++i) {
    ImuMeas imu;
    imu.timestamp = begin + (end - begin) * i / 10;
    imu.gyro.setZero();
    imu.accel = Eigen::Vector3d(.2, -.1, 9.81);
    result.push_back(imu);
  }
  return result;
}

void estimator_contract() {
  OdometryParameters odometry;
  LocalSubmapParameters local;
  local.thread_num = 1;
  tools::ParallelExecutor<> executor(local.thread_num);
  VoxelMap map(odometry, local, executor);
  LidarFactor factor(local.win_size);
  std::vector<StateGroup> states{state()};
  seed(map, planes(), local.win_size, states, factor);
  ESKF filter;
  auto predicted = state(1.1);
  predicted.p = Eigen::Vector3d(.02, -.015, .01);
  predicted.v = Eigen::Vector3d(.1, .2, .3);
  // Nonzero normal residual would alter both pose and covariance in old code.
  auto planar = planes(1);
  auto candidate = predicted;
  assert(!filter.observe_voxelmap(candidate, *planar, map));
  same(candidate, predicted);
  PointCloud empty;
  assert(!filter.observe_voxelmap(candidate, empty, map));
  same(candidate, predicted);
  auto invalid = planes();
  invalid->back().pnt.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!filter.observe_voxelmap(candidate, *invalid, map));
  same(candidate, predicted);
  auto bad_cov = predicted;
  bad_cov.cov(0, 0) = -1;
  const auto before_bad_cov = bad_cov;
  auto full = planes();
  assert(!filter.observe_voxelmap(bad_cov, *full, map));
  same(bad_cov, before_bad_cov);
  assert(filter.observe_voxelmap(candidate, *full, map));
  assert(candidate.p.norm() < predicted.p.norm() * .2);
  assert(candidate.cov.trace() < predicted.cov.trace());
  assert(candidate.t == predicted.t);
}
} // namespace

namespace sapphire {
struct SlamPipelineTestAccess {
  static void disabled_backend_drain(SlamPipeline &p) {
    // Invalid evidence must be drained without building a node or failing LIO
    // when mapping is disabled.
    {
      std::lock_guard<std::mutex> lock(p.marginal_mutex_);
      p.marginal_frames_.emplace_back(state(), GaussianCloud(1), std::numeric_limits<double>::quiet_NaN(), 0.0);
    }
    p.marginal_cv_.notify_one();
    p.shutdown();
    assert(!p.failed());
    assert(p.marginal_frames_.empty());
    assert(!p.producerFailure());
  }

  static void run(SlamPipeline &p, int &scans, int &odom) {
    p.shutdown(); // Exercise the production commit method with no live readers.
    auto &frontend = *p.frontend_;
    frontend.current_state_ = state();
    frontend.imu_estimator_.scale_gravity = 1;
    frontend.local_map_.states_.push_back(frontend.current_state_);
    frontend.local_map_.points_.push_back(planes());
    frontend.local_map_.times_.push_back(1);
    frontend.local_map_.journeys_.push_back(0);
    auto &factor = frontend.local_map_.lidar_factor_;
    seed(frontend.local_map_.geometry_, frontend.local_map_.points_.back(), frontend.local_map_.capacity(), frontend.local_map_.states_, factor);
    const auto map_size = frontend.local_map_.geometry_.size();
    const auto active_size = frontend.local_map_.geometry_.active_size();
    const auto factor_count = factor.plvec_voxels.size();
    for (int i = 1; i <= 2; ++i) {
      frontend.current_state_ = state(1 + i * .1);
      frontend.current_state_.p = Eigen::Vector3d(.02, -.015, .01);
      const auto prediction = frontend.current_state_;
      auto rejected = planes(1);
      auto distant = rejected->back();
      distant.pnt.setConstant(100); // Would create a new voxel if inserted.
      rejected->push_back(distant);
      assert(!frontend.update_lidar_window(rejected, interval(1 + (i - 1) * .1, 1 + i * .1), 0));
      same(frontend.current_state_, prediction);
      assert(frontend.local_map_.size() == 1 && frontend.local_map_.states_.size() == 1 && frontend.local_map_.points_.size() == 1);
      assert(frontend.local_map_.times_.size() == 1 && frontend.local_map_.journeys_.size() == 1);
      assert(frontend.local_map_.geometry_.size() == map_size && frontend.local_map_.geometry_.active_size() == active_size);
      assert(factor.plvec_voxels.size() == factor_count && frontend.local_map_.imu_factors_.empty());
      assert(scans == 0 && odom == 0);
      assert(frontend.local_map_.pending_imu_ && std::abs(frontend.local_map_.pending_imu_->dtime - i * .1) < 1e-12);
    }
    frontend.current_state_.t = 1.3;
    assert(frontend.update_lidar_window(planes(), interval(1.2, 1.3), 0));
    assert(frontend.local_map_.size() == 2 && frontend.local_map_.times_.back() == 1.3);
    // Observation insertion precedes BA; committed output belongs to run().
    assert(scans == 0 && odom == 0);
    assert(!frontend.local_map_.pending_imu_ && frontend.local_map_.imu_factors_.size() == 1);
    const auto &actual = *frontend.local_map_.imu_factors_.front();
    // Independent full-interval integration oracle, including covariance.
    std::deque<ImuMeas> all;
    for (int i = 0; i <= 30; ++i) {
      auto imu = interval(1, 1.1).front();
      imu.timestamp = 1 + i * .01;
      all.push_back(imu);
    }
    ImuFactor expected;
    expected.push_imu(all, 1, p.parameters_.local_submap);
    assert(std::abs(actual.dtime - .3) < 1e-12);
    assert((actual.p_delta - expected.p_delta).norm() < 1e-12);
    assert((actual.v_delta - expected.v_delta).norm() < 1e-12);
    assert((actual.R_delta - expected.R_delta).norm() < 1e-12);
    assert((actual.cov - expected.cov).norm() < 1e-12);
    assert((actual.v_delta - Eigen::Vector3d(.2, -.1, 9.81) * .3).norm() < 1e-12);
    assert((actual.p_delta - Eigen::Vector3d(.2, -.1, 9.81) * .045).norm() < 1e-12);
    frontend.current_state_.t = 1.4;
    assert(!frontend.update_lidar_window(planes(1), interval(1.3, 1.4), 0));
    assert(frontend.local_map_.pending_imu_);
    frontend.system_reset();
    assert(!frontend.local_map_.pending_imu_ && frontend.local_map_.imu_factors_.empty());
    assert(frontend.local_map_.size() == 0 && frontend.local_map_.states_.empty());
    // Failed initialization must release all borrowed input and integrator history.
    for (int i = 0; i < frontend.local_map_.capacity(); ++i) {
      MeasGroup raw;
      raw.lidar_begin_time = 2 + i * .1;
      raw.lidar_end_time = raw.lidar_begin_time + .1;
      raw.imu_buf = interval(raw.lidar_begin_time, raw.lidar_end_time);
      raw.lidar_cloud = std::make_shared<std::vector<LidarPoint>>();
      frontend.local_map_.insert_initial(state(raw.lidar_end_time), planes(), std::move(raw), 1);
    }
    assert(!frontend.local_map_.initialize(frontend.current_state_, frontend.extrinsic_, 1));
    assert(!frontend.local_map_.initial_measures_.empty());
    frontend.system_reset();
    assert(frontend.local_map_.size() == 0 && frontend.local_map_.initial_measures_.empty());
    assert(frontend.local_map_.imu_factors_.empty() && !frontend.local_map_.pending_imu_);
    assert((frontend.current_state_.g - Eigen::Vector3d(0, 0, -9.81)).norm() < 1e-12);

  }
};
} // namespace sapphire

int main() {
  estimator_contract();
  SapphireParameters parameters;
  parameters.pose_graph.enabled = false;
  parameters.local_submap.thread_num = 1;
  { SlamPipeline disabled(parameters); SlamPipelineTestAccess::disabled_backend_drain(disabled); }
  int scans = 0, odom = 0;
  SlamPipeline::Callbacks output;
  output.local_scan = [&](auto) { ++scans; };
  output.odom_state = [&](const auto &) { ++odom; };
  SlamPipeline pipeline(parameters, output);
  SlamPipelineTestAccess::run(pipeline, scans, odom);
  parameters.odometry.gicp_fallback.enabled = true;
  scans = odom = 0;
  { SlamPipeline fallback(parameters, output); SlamPipelineTestAccess::run(fallback, scans, odom); }
  std::cout << "PASS: candidate rollback, accepted correction, rejected map/window isolation, IMU continuity and reset\n";
}
