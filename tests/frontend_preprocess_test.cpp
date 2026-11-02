#include "sensor/lidar_handler.hpp"
#include "sensor/lidar_sync.hpp"
#include "module/synchronizer.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

void dual_sync_contract() {
  using namespace sapphire;
  LidarSyncParameters config;
  config.front_time_offset = .012;
  config.rear_time_offset = -.002;
  config.extrinsic.linear() = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  config.extrinsic.translation() = Eigen::Vector3d(1, -2, 3);
  const auto now = LidarSync::Clock::now();
  const auto scan = [&](bool rear, double start, double duration = .2) {
    std::vector<TimedLidarPoint> points;
    for (int i = 0; i < 5; ++i)
      points.push_back({1, 2, 3, double(i + 1), start + duration * i / 4 -
                       (rear ? config.rear_time_offset : config.front_time_offset)});
    return points;
  };
  LidarSync sync(config);
  auto front = scan(false, 100), rear = scan(true, 100);
  std::swap(front[0], front[3]); // Point time order need not match wire order.
  assert(sync.push(false, front, "front", now));
  assert(sync.push(true, rear, "rear", now));
  for (int window = 0; window < 2; ++window) {
    const auto output = sync.pop(now);
    assert(output && std::abs(output->begin - (100 + .1 * window)) < 1e-8 && output->points.size() == 4);
    int fronts = 0, rears = 0;
    const auto transformed = config.extrinsic * Eigen::Vector3d(1, 2, 3);
    for (const auto &point : output->points) {
      assert(point.time_offset >= 0 && point.time_offset < .1);
      fronts += (Eigen::Vector3d(point.x, point.y, point.z) - Eigen::Vector3d(1, 2, 3)).norm() < 1e-5;
      rears += (Eigen::Vector3d(point.x, point.y, point.z) - transformed).norm() < 1e-5;
    }
    assert(fronts == 2 && rears == 2);
  }
  assert(!sync.pop(now)); // Last boundary point belongs to the next window.
  assert(!sync.push(false, front, "front", now) && sync.statistics().out_of_order == 1);
  sync.reset();
  assert(sync.buffered_front()==0 && sync.buffered_rear()==0);
  assert(sync.push(false, front, "front", now));
  assert(sync.push(true, rear, "rear", now));
  assert(sync.pop(now)); // Old sensor timestamps are valid in a new input epoch.
  assert(!sync.push(false, scan(false, 100.2), "changed", now));
  auto invalid = scan(false, 101);
  invalid[1].timestamp = std::numeric_limits<double>::quiet_NaN();
  config.point_filter_num = 2;
  LidarSync thinning(config);
  assert(!thinning.push(false, invalid, "front", now)); // Validate skipped times too.
  config.point_filter_num = 1;
  LidarSync timeout(config);
  assert(timeout.push(false, scan(false, 100), "front", now));
  assert(timeout.push(true, scan(true, 100), "rear", now));
  assert(!timeout.pop(now + std::chrono::seconds(1)) && timeout.statistics().timeout > 0);
  config.queue_size = 2;
  LidarSync bounded(config);
  for (int i = 0; i < 3; ++i) assert(bounded.push(false, scan(false, 100 + i * .2, .1), "front", now));
  assert(bounded.buffered_front() == 2 && bounded.statistics().overflow == 1);
  LidarSync coverage(config);
  assert(coverage.push(false, scan(false, 100), "front", now));
  assert(coverage.push(true, scan(true, 100, .05), "rear", now));
  assert(coverage.push(true, scan(true, 100.15, .05), "rear", now));
  assert(!coverage.pop(now) && coverage.statistics().coverage > 0);
  config.lidar_merge_hz = 0;
  LidarSync frame_windows(config);
  assert(frame_windows.push(false, scan(false, 100), "front", now));
  assert(frame_windows.push(true, scan(true, 100), "rear", now));
  const auto legacy = frame_windows.pop(now);
  assert(legacy && legacy->points.size() == 10 && std::abs(legacy->end - 100.2) < 1e-8);
}

int main() {
  dual_sync_contract();
  using namespace sapphire;
  SensorParameters config;
  config.point_filter_num = 2;
  config.blind_squared = 1.0;
  config.max_scan_duration = .2;
  // A filtered blind return still determines the legacy relative-time origin.
  std::array<std::array<double, 5>, 4> data{{
      {{.1, 0, 0, 2, 1000000}}, {{2, 0, 0, 3, 2000000}},
      {{3, 0, 0, 4, 3000000}}, {{4, 0, 0, 5, 4000000}}}};
  int reads = 0;
  const auto read = [&](std::size_t i, double &x, double &y, double &z, double &intensity, double &time) {
    ++reads;
    x = data[i][0]; y = data[i][1]; z = data[i][2]; intensity = data[i][3]; time = data[i][4];
    return true;
  };
  std::vector<LidarPoint> points;
  std::optional<double> end;
  double stamp = 0;
  LidarHandler relative(config);
  assert(relative.process(12, data.size(), read, {}, stamp, points, end));
  assert(reads == 2 && points.size() == 1 && stamp == 12 && !end);
  assert(points[0].x == 3 && std::abs(points[0].time_offset - .002) < 1e-8);

  config.lidar_type = 2;
  data[0][4] = 100.01; data[1][4] = 100.04;
  data[2][4] = 100.08; data[3][4] = 100.10;
  LidarHandler absolute(config);
  assert(absolute.process(999, data.size(), read, std::make_pair(100.01, 100.10), stamp, points, end));
  assert(stamp == 100.01 && end && *end == 100.10 && points.size() == 1);
  assert(std::abs(points[0].time_offset - .07) < 1e-7);
  // End time includes the skipped final return and is available to synchronization.
  Synchronizer sync(.2);
  assert(sync.push_lidar(stamp, points, end));
  assert(!absolute.process(999, data.size(), read, {}, stamp, points, end));
  assert(points.empty() && !end);
  assert(!absolute.process(999, data.size(), read, std::make_pair(100., 101.), stamp, points, end));
  // All native formats share filtering but keep their distinct clock contracts.
  std::vector<LivoxPoint> livox(4);
  const std::array<uint32_t, 4> offsets{{60000000, 0, 40000000, 100000000}};
  for (size_t i = 0; i < livox.size(); ++i) {
    livox[i].x = 3; livox[i].reflectivity = 42; livox[i].offset_time = offsets[i];
  }
  config.lidar_type = 0;
  LidarHandler livox_handler(config);
  assert(livox_handler.process(100, livox, stamp, points, end));
  assert(stamp == 100 && end && std::abs(*end - 100.1) < 1e-12 && points.size() == 2);
  assert(points[0].intensity == 42 && std::abs(points[0].time_offset - .06) < 1e-7);
  assert(!absolute.process(100, livox, stamp, points, end) && points.empty() && !end);

  std::vector<VelodynePoint> velodyne(4);
  for (size_t i = 0; i < velodyne.size(); ++i) {
    velodyne[i].x = 3; velodyne[i].time = offsets[i] * 1e-9 - .1;
  }
  config.lidar_type = 3;
  LidarHandler velodyne_handler(config);
  assert(velodyne_handler.process(100, velodyne, stamp, points, end));
  assert(std::abs(stamp - 99.9) < 1e-7 && end && *end == 100 && points.size() == 2);
  velodyne[1].time = std::numeric_limits<float>::quiet_NaN();
  assert(!velodyne_handler.process(100, velodyne, stamp, points, end) && points.empty() && !end);

  std::vector<RobosensePoint> robosense(4);
  std::vector<HesaiXtPoint> hesai(4);
  for (size_t i = 0; i < 4; ++i) {
    robosense[i].x = hesai[i].x = 3;
    robosense[i].timestamp = hesai[i].timestamp = 100 + offsets[i] * 1e-9;
  }
  config.lidar_type = 1;
  LidarHandler robosense_handler(config);
  assert(robosense_handler.process(999, robosense, stamp, points, end));
  assert(stamp == 100 && end && std::abs(*end - 100.1) < 1e-12 && points.size() == 2);
  assert(absolute.process(999, hesai, stamp, points, end));
  assert(stamp == 100 && end && std::abs(*end - 100.1) < 1e-12 && points.size() == 2);
  hesai[3].timestamp = 101;
  assert(!absolute.process(999, hesai, stamp, points, end) && points.empty() && !end);
  std::cout << "PASS: estimator-independent preprocessing, time origin, filtering, end-time and synchronization\n";
}
