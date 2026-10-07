#include "frontend/common/lidar_preprocess.hpp"
#include "frontend/common/synchronizer.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
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
  LidarProcessor relative(config);
  assert(relative.process(12, data.size(), read, {}, stamp, points, end));
  assert(reads == 2 && points.size() == 1 && stamp == 12 && !end);
  assert(points[0].x == 3 && std::abs(points[0].time_offset - .002) < 1e-8);

  config.lidar_type = 2;
  data[0][4] = 100.01; data[1][4] = 100.04;
  data[2][4] = 100.08; data[3][4] = 100.10;
  LidarProcessor absolute(config);
  assert(absolute.process(999, data.size(), read, std::make_pair(100.01, 100.10), stamp, points, end));
  assert(stamp == 100.01 && end && *end == 100.10 && points.size() == 1);
  assert(std::abs(points[0].time_offset - .07) < 1e-7);
  // End time includes the skipped final return and is available to synchronization.
  Synchronizer sync(.2);
  assert(sync.push_lidar(stamp, points, end));
  assert(!absolute.process(999, data.size(), read, {}, stamp, points, end));
  assert(points.empty() && !end);
  assert(!absolute.process(999, data.size(), read, std::make_pair(100., 101.), stamp, points, end));
  std::cout << "PASS: estimator-independent preprocessing, time origin, filtering, end-time and synchronization\n";
}
