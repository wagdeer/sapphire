#include "frontend/common/imu_estimator.hpp"

#include <cassert>
#include <iostream>
using namespace sapphire;
int main() {
  for (int count = 0; count <= 5; ++count) {
    ImuEstimator estimator;
    estimator.configure(SensorParameters{}, InitializerParameters{}, OdometryParameters{});
    estimator.init_flag = true;
    estimator.has_last_imu = true;
    estimator.last_pcl_end_time = 0;
    estimator.pcl_beg_time = 0;
    estimator.pcl_end_time = .01;
    estimator.last_imu.timestamp = 0;
    estimator.last_imu.gyro.setZero();
    estimator.last_imu.accel = Eigen::Vector3d(0, 0, 9.81);
    StateGroup state;
    state.g = Eigen::Vector3d(0, 0, -9.81);
    std::deque<ImuMeas> readings;
    for (int i = 1; i <= count; ++i) {
      auto reading = estimator.last_imu;
      reading.timestamp = .01 * i / count;
      readings.push_back(reading);
    }
    std::vector<LidarPoint> cloud(1);
    cloud[0].x = 1;
    cloud[0].time_offset = .01;
    const int result = estimator.process(state, cloud, readings);
    assert(result == (count >= 2));
    assert(state.t == (count >= 2 ? .01 : 0));
  }
  ImuEstimator initializer;
  initializer.configure(SensorParameters{}, InitializerParameters{}, OdometryParameters{});
  StateGroup state;
  std::vector<LidarPoint> cloud(1);
  std::deque<ImuMeas> empty;
  assert(initializer.process(state, cloud, empty) == 0);
  // A rejected scan must leave state, covariance and point coordinates intact.
  for (int reason = 0; reason < 4; ++reason) {
    ImuEstimator e;
    e.configure(SensorParameters{}, InitializerParameters{}, OdometryParameters{});
    e.init_flag = true;
    e.has_last_imu = true;
    e.last_pcl_end_time = 0;
    e.pcl_beg_time = 0;
    e.pcl_end_time = .01;
    e.last_imu.timestamp = 0;
    e.last_imu.gyro.setZero();
    e.last_imu.accel.setZero();
    std::deque<ImuMeas> imu(2, e.last_imu);
    imu[0].timestamp = .005;
    imu[1].timestamp = .01;
    std::vector<LidarPoint> points(1);
    points[0].x = 2;
    points[0].y = 3;
    points[0].z = 4;
    if (reason == 0) e.has_last_imu = false;
    if (reason == 1) e.pcl_end_time = 0;
    if (reason == 2) e.last_pcl_end_time = std::numeric_limits<double>::quiet_NaN();
    if (reason == 3) {
      e.last_imu.timestamp = .002;
    }
    StateGroup before;
    before.p = Eigen::Vector3d(1, 2, 3);
    before.v = Eigen::Vector3d(4, 5, 6);
    StateGroup after = before;
    assert(e.process(after, points, imu) == 0);
    assert((after - before).norm() == 0 && after.cov == before.cov && after.t == before.t);
    assert(points[0].x == 2 && points[0].y == 3 && points[0].z == 4);
    assert(imu[0].timestamp == .005 && imu[1].timestamp == .01);
  }
  // Initialization counts actual measurements and gives the same mean across batches.
  ImuEstimator one, batches;
  std::deque<ImuMeas> readings(30);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int i = 0; i < 30; ++i) {
    readings[i].timestamp = .005 * i;
    readings[i].accel = Eigen::Vector3d(i, i * i, 9.81);
    readings[i].gyro = readings[i].accel * .01;
    mean += readings[i].accel / 30.;
  }
  one.init(readings);
  batches.init({});
  assert(batches.init_num == 0 && !batches.has_last_imu);
  batches.init(std::deque<ImuMeas>(readings.begin(), readings.begin() + 7));
  batches.init(std::deque<ImuMeas>(readings.begin() + 7, readings.end()));
  assert(one.init_num == 30 && batches.init_num == 30);
  assert((one.mean_acc - mean).norm() < 1e-12);
  assert((one.mean_gyr - mean * .01).norm() < 1e-12);
  assert(one.mean_acc == batches.mean_acc && one.mean_gyr == batches.mean_gyr);
  // Resetting the count starts a fresh mean, even with stale accumulator values.
  one.init_num = 0;
  one.mean_gyr.setConstant(1e30);
  one.init(std::deque<ImuMeas>{readings.back()});
  assert(one.init_num == 1);
  assert(one.mean_acc == readings.back().accel && one.mean_gyr == readings.back().gyro);
  ImuEstimator threshold;
  threshold.configure(SensorParameters{}, InitializerParameters{}, OdometryParameters{});
  threshold.pcl_end_time = .145;
  auto first = std::deque<ImuMeas>(readings.begin(), readings.end() - 1);
  threshold.process(state, cloud, first);
  assert(!threshold.init_flag && threshold.init_num == 29);
  auto last = std::deque<ImuMeas>{readings.back()};
  threshold.process(state, cloud, last);
  assert(threshold.init_flag && threshold.init_num == 30);
  std::cout << "PASS: empty/one-sample failures cannot report success; two through five samples advance; empty initializer safe\n";
}
