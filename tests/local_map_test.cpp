#include "maplayer/local_map.hpp"
#include "maplayer/node_builder.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <tuple>

using namespace sapphire;

namespace {
StateGroup pose(int frame) {
  StateGroup x;
  x.t = 1.0 + frame * .1;
  x.R = Eigen::AngleAxisd(.13, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  x.p = Eigen::Vector3d(.03, -.04, .02);
  x.g = Eigen::Vector3d(0, 0, -9.81);
  x.cov.setIdentity();
  x.cov *= .01;
  return x;
}

PointCloudPtr cloud() {
  auto points = std::make_shared<PointCloud>();
  for (int axis = 0; axis < 3; ++axis) {
    for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) {
      pointVar p;
      p.pnt.setConstant(.5);
      p.pnt[axis] = 1.5 + axis;
      p.pnt[(axis + 1) % 3] = .22 + i * .06;
      p.pnt[(axis + 2) % 3] = .22 + j * .06;
      p.var = Eigen::Matrix3d::Identity() * .0001;
      points->push_back(p);
    }
  }
  return points;
}

std::deque<ImuMeas> interval(int frame) {
  std::deque<ImuMeas> imus;
  for (int i = 0; i <= 10; ++i) {
    ImuMeas imu;
    imu.timestamp = pose(frame).t - .1 + i * .01;
    imu.gyro.setZero();
    imu.accel = Eigen::Vector3d(0, 0, 9.81);
    imus.push_back(imu);
  }
  return imus;
}

template<class F> void rejects(F &&f) {
  bool rejected = false;
  try { f(); } catch (const std::logic_error &) { rejected = true; }
  assert(rejected);
}

// Independent old runtime sequence: the refactor must preserve BA, frame
// retirement and odom-coordinate Gaussian moments, not merely vector lengths.
std::vector<LidarFrame> reference(const SapphireParameters &p, tools::ParallelExecutor<> &executor) {
  VoxelMap map(p.odometry, p.local_submap, executor);
  LidarFactor factor(p.local_submap.win_size);
  LI_BA_Optimizer optimizer(p.local_submap, executor);
  std::vector<StateGroup> states;
  std::deque<std::unique_ptr<ImuFactor>> imus;
  std::vector<LidarFrame> output;
  for (int i = 0; i < 8; ++i) {
    if (!states.empty()) {
      auto edge = std::make_unique<ImuFactor>(states.back().bg, states.back().ba);
      edge->push_imu(interval(i), 1, p.local_submap);
      imus.push_back(std::move(edge));
    }
    states.push_back(pose(i));
    auto points = cloud();
    vvec<double, 3> world;
    pvec_update(points, states.back(), world);
    factor.clear();
    factor.win_size = p.local_submap.win_size;
    map.cut_voxel_multi(points, states.size() - 1, p.local_submap.win_size, world);
    map.recut_multi(states.size(), states, p.odometry.min_eigen_value, p.local_submap.plane_eigen_value_thre_inv);
    map.collect_factors(factor);
    if (states.size() < static_cast<std::size_t>(p.local_submap.win_size)) continue;
    Eigen::MatrixXd hessian;
    optimizer.damping_iter(states, factor, imus, &hessian);
    auto frozen = map.marginalize(i, states.size(), 1, states);
    output.emplace_back(states.front(), std::move(frozen), states.front().t, i - p.local_submap.win_size + 1);
    states.erase(states.begin());
    imus.pop_front();
  }
  return output;
}

void same_cloud(GaussianCloud a, GaussianCloud b) {
  auto less = [](const GaussianPoint &a, const GaussianPoint &b) {
    return std::tie(a.voxel_key.x, a.voxel_key.y, a.voxel_key.z) < std::tie(b.voxel_key.x, b.voxel_key.y, b.voxel_key.z);
  };
  std::sort(a.begin(), a.end(), less);
  std::sort(b.begin(), b.end(), less);
  assert(a.size() == b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    assert(a[i].voxel_key == b[i].voxel_key && a[i].N == b[i].N && a[i].is_plane == b[i].is_plane);
    assert((a[i].mean - b[i].mean).norm() < 1e-6);
    assert((a[i].covariance - b[i].covariance).norm() < 1e-6);
  }
}

void rolling_window(SapphireParameters parameters, tools::ParallelExecutor<> &executor) {
  auto expected = reference(parameters, executor);
  LocalMap map(parameters, executor);
  NodeBuilder builder(100.0);
  rejects([&] { map.retire_oldest(0, true); });
  auto current = pose(0);
  rejects([&] { map.optimize(current); });
  std::size_t emitted = 0;
  for (int i = 0; i < 8; ++i) {
    current = pose(i);
    auto points = cloud();
    vvec<double, 3> world;
    pvec_update(points, current, world);
    map.append_imu(interval(i), 1);
    map.insert(current, points, i, world);
    if (!map.full()) continue;
    // A second live map must not reset the first map's rotating temporal slots,
    // root scale or depth, even when it is reset and destroyed between updates.
    {
      auto other_parameters = parameters;
      other_parameters.local_submap.win_size = 5;
      other_parameters.local_submap.max_layer = 1;
      other_parameters.odometry.voxel_size = 2;
      other_parameters.odometry.voxel_size_inv = .5;
      LocalMap other(other_parameters, executor);
      other.insert(current, points, i, world);
      assert(other.geometry().voxel_size() == 2);
      other.reset();
    }
    assert(map.geometry().voxel_size() == parameters.odometry.voxel_size);
    rejects([&] { map.retire_oldest(i, true); });
    map.optimize(current);
    const auto feedback = map.state(map.size() - 1);
    assert((current.p - feedback.p).norm() == 0 && (current.R - feedback.R).norm() == 0);
    auto frozen = map.retire_oldest(i, true);
    const auto &want = expected.at(emitted);
    assert(frozen.timestamp == want.timestamp && frozen.journey == want.journey);
    assert((frozen.x.p - want.x.p).norm() < 1e-12 && (frozen.x.R - want.x.R).norm() < 1e-12);
    same_cloud(frozen.gaussians, want.gaussians);
    int samples = 0;
    for (const auto &g : frozen.gaussians) samples += g.N;
    assert(samples == static_cast<int>(points->size())); // One frame, never the whole overlapping window.
    assert(!builder.push(std::move(frozen)));
    ++emitted;
    assert(map.retired_frames() == emitted && map.size() == map.capacity() - 1);
    assert(map.state(0).t == pose(emitted).t);
    rejects([&] { map.retire_oldest(i, true); });
  }
  assert(emitted == expected.size());
  // The remaining accepted window is sealed once, even without another scan.
  while (auto tail = map.seal_oldest()) {
    assert(tail->timestamp == pose(emitted).t);
    int samples = 0;
    for (const auto& g : tail->gaussians) samples += g.N;
    assert(samples == static_cast<int>(cloud()->size()));
    assert(!builder.push(std::move(*tail)));
    ++emitted;
  }
  assert(emitted == 8 && map.size() == 0 && !map.seal_oldest());
  auto node = builder.flush();
  assert(node);
  int samples = 0;
  for (const auto &g : *node->node->lidar_cloud()) samples += g.N;
  assert(samples == static_cast<int>(emitted * cloud()->size()));
  map.reset();
  assert(map.size() == 0 && map.retired_frames() == 0 && map.geometry().size() == 0);

  // Reset must allow a fresh startup, without the old pending IMU bridge.
  for (int i = 0; i < map.capacity(); ++i) {
    current = pose(i);
    auto points = cloud();
    vvec<double, 3> world;
    pvec_update(points, current, world);
    map.append_imu(interval(i), 1);
    map.insert(current, points, i, world);
  }
  map.optimize(current);
  assert(map.retire_oldest(0, false).gaussians.empty());
}
}  // namespace

int main() {
  SapphireParameters parameters;
  parameters.local_submap.win_size = 3;
  parameters.local_submap.thread_num = 1;
  tools::ParallelExecutor<> executor(1);
  rolling_window(parameters, executor);
  parameters.local_submap.win_size = 1;
  rejects([&] { LocalMap invalid(parameters, executor); });
  std::cout << "PASS: LocalMap preserves rolling BA, freezes each observation once, builds dense Nodes and resets coherently\n";
}
