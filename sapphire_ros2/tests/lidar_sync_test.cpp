#include "sapphire_ros2/lidar_sync.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <limits>
#include <set>
#include <stdexcept>

#include "pipeline.hpp"

namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
using Processor = sapphire_ros::LidarSync;
using Parameters = sapphire_ros::LidarSyncParameters;
using F = sensor_msgs::msg::PointField;

void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

template <class T>
void store(Cloud &m, size_t index, size_t offset, T value) {
  auto *target = m.data.data() + index / m.width * m.row_step + index % m.width * m.point_step + offset;
  std::memcpy(target, &value, sizeof(value));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  if (!m.is_bigendian) std::reverse(target, target + sizeof(value));
#else
  if (m.is_bigendian) std::reverse(target, target + sizeof(value));
#endif
}

Cloud::SharedPtr cloud(const std::vector<double> &times, bool rear = false, bool big = false, bool organized = false) {
  auto m = std::make_shared<Cloud>();
  m->header.frame_id = rear ? "rear" : "front";
  m->header.stamp.sec = 99;  // Deliberately unrelated: point timestamps are authoritative.
  m->height = organized ? 2 : 1;
  m->width = times.size() / m->height;
  m->point_step = 24;
  m->row_step = m->width * m->point_step + (organized ? 8 : 0);
  m->is_bigendian = big;
  m->data.resize(m->row_step * m->height, 0x5a);
  const std::vector<std::string> names{"x", "y", "z", "intensity", "timestamp"};
  for (size_t i = 0; i < names.size(); ++i) {
    F field;
    field.name = names[i];
    field.offset = i * 4;
    field.datatype = i == 4 ? F::FLOAT64 : F::FLOAT32;
    field.count = 1;
    m->fields.push_back(field);
  }
  for (size_t i = 0; i < times.size(); ++i) {
    store(*m, i, 0, rear ? 2.0F : 1.0F);
    store(*m, i, 4, 0.0F);
    store(*m, i, 8, 0.0F);
    store(*m, i, 12, rear ? 20.0F : 10.0F);
    store(*m, i, 16, times[i]);
  }
  return m;
}

Parameters legacy_parameters() {
  Parameters o;
  o.lidar_merge_hz = 0.0;
  return o;
}

void test_windows() {
  Parameters o = legacy_parameters();
  o.extrinsic.translation().x() = 1.0;
  Processor p(o);
  auto front = cloud({10.0, 10.05, 10.1});
  auto rear = cloud({9.95, 10.0, 10.05}, true);
  auto rear2 = cloud({10.06, 10.1, 10.15}, true);
  const auto original = rear2->data;
  check(p.push(false, front) && p.push(true, rear), "accept phase-shifted scans");
  check(!p.pop(), "wait for rear watermark even when headers match");
  check(p.push(true, rear2), "accept adjacent rear frame");
  auto first = p.pop();
  check(first && first->points.size() == 7 && first->begin == 10.0 && first->end == 10.1, "slice two rear frames into one front window");
  check(p.buffered_rear() == 1, "retain unused rear suffix");
  for (const auto &point : first->points) {
    check(point.time_offset >= 0 && point.time_offset <= 0.100001F, "correct merged offsets");
    if (point.intensity == 20) check(point.x == 3, "rear extrinsic applied exactly once");
  }
  check(rear2->data == original, "input ROS cloud remains unchanged");
  check(p.push(false, cloud({10.1, 10.15, 10.2})), "accept next primary window");
  check(p.push(true, cloud({10.16, 10.2, 10.25}, true)), "accept next rear frame");
  auto second = p.pop();
  check(second && second->points.size() == 5, "preserve rear suffix and avoid boundary duplicates");
  for (const auto &point : second->points) check(point.time_offset > 0, "already emitted boundary is excluded");
  check(p.statistics().emitted == 2, "emission statistics");
}

void test_validation_and_waits() {
  Processor p(legacy_parameters());
  auto bad = cloud({1.0, 1.1});
  bad->fields.back().offset = 22;
  check(!p.push(false, bad), "reject out-of-bounds field");
  bad = cloud({1.0, 1.1});
  bad->data.pop_back();
  check(!p.push(false, bad), "reject truncated cloud");
  bad = cloud({1.0, 1.1});
  bad->fields.back().datatype = F::FLOAT32;
  check(!p.push(false, bad), "reject ambiguous timestamp datatype");
  check(!p.push(false, cloud({1.0, std::numeric_limits<double>::quiet_NaN()})), "reject nonfinite time");
  check(!p.push(false, cloud({1.0, 100.0})), "reject incorrect units or excessive scan span");
  auto a = cloud({1.1, 1.0, 1.08, 1.02}, false, true, true);
  check(p.push(false, a) && p.push(true, cloud({1.0, 1.1}, true, true)), "big-endian padded organized clouds accepted");
  auto scan = p.pop();
  check(scan && scan->begin == 1 && scan->points.size() == 6, "unsorted timestamps scanned without relying on first point");
  check(!p.push(false, a), "duplicate frame rejected");
  check(!p.push(false, cloud({0.9, 1.0})), "out-of-order frame rejected");
  bad = cloud({1.2, 1.3});
  bad->header.frame_id = "changed_frame";
  check(!p.push(false, bad), "frame changes require reconfiguration");

  Parameters o = legacy_parameters();
  o.max_wait = 0.1;
  o.queue_size = 2;
  Processor timeout(o);
  const auto now = Processor::Clock::now();
  check(timeout.push(false, cloud({2.0, 2.1}), now), "queue primary");
  check(!timeout.pop(now + std::chrono::milliseconds(101)) && timeout.statistics().timeout == 1 && timeout.buffered_front() == 0,
        "missing rear stream expires even without more arrivals");
  for (int i = 0; i < 5; ++i) timeout.push(false, cloud({3.0 + i, 3.1 + i}), now);
  check(timeout.buffered_front() == 2 && timeout.statistics().overflow == 3, "bounded buffer under dead sensor");
  Processor gap(legacy_parameters());
  gap.push(false, cloud({5.0, 5.1}));
  gap.push(true, cloud({5.04, 5.12}, true));
  check(!gap.pop() && gap.statistics().coverage == 1, "reject uncovered primary prefix");
  Processor internal_gap(legacy_parameters());
  internal_gap.push(false, cloud({6.0, 6.2}));
  internal_gap.push(true, cloud({6.0, 6.05}, true));
  internal_gap.push(true, cloud({6.15, 6.2}, true));
  check(!internal_gap.pop() && internal_gap.statistics().coverage == 1, "reject missing rear frame in the middle");
  Processor empty(legacy_parameters());
  auto nan = cloud({7.0, 7.1}, true);
  store(*nan, 0, 0, std::numeric_limits<float>::quiet_NaN());
  store(*nan, 1, 0, std::numeric_limits<float>::quiet_NaN());
  empty.push(false, cloud({7.0, 7.1}));
  empty.push(true, nan);
  check(!empty.pop() && empty.statistics().empty == 1, "no silent single-sensor fallback");
}

void test_time_offset_and_end() {
  Parameters o = legacy_parameters();
  o.rear_time_offset = -0.05;
  o.point_filter_num = 2;
  Processor p(o);
  p.push(false, cloud({20.0, 20.1, 20.05}));
  p.push(true, cloud({20.05, 20.15, 20.1}, true));
  auto scan = p.pop();
  check(scan && std::abs(scan->end - 20.1) < 1e-8, "end time survives decimation and sensor clock offset");
  sapphire::Synchronizer sync(0.25);
  // A 5 Hz Airy frame must not suffer the old hardcoded 110 ms truncation.
  std::vector<sapphire::LidarPoint> points(2);
  points[0].x = points[1].x = 1;
  points[0].time_offset = 0.05;
  points[1].time_offset = 0.18;
  check(sync.push_lidar(30.0, points, 30.2), "explicit end accepted");
  for (int i = 0; i <= 25; ++i) {
    sapphire::ImuMeas imu;
    imu.timestamp = 30.0 + i * 0.01;
    imu.gyro.setZero();
    imu.accel.setZero();
    sync.push_imu(imu);
  }
  sapphire::MeasGroup group;
  check(sync.sync_packages(group) && group.lidar_cloud->size() == 2 && group.lidar_end_time == 30.2,
        "synchronizer preserves >110 ms points and true acquisition end");
  check(!sync.push_lidar(31.0, points, 31.1), "reject points past declared end");
  check(!sync.push_lidar(31.0, points, 31.5), "reject oversized explicit window");
  bool invalid = false;
  try {
    sapphire::Synchronizer bad(0);
  } catch (const std::invalid_argument &) {
    invalid = true;
  }
  check(invalid, "invalid duration rejected");
}

void test_moving_rig_deskew(double rate = 0.0) {
  Parameters o = legacy_parameters();
  o.lidar_merge_hz = rate;
  o.extrinsic.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();
  o.extrinsic.translation() << -0.3, 0.1, 0.5;
  Processor p(o);
  const std::vector<double> times{40.0, 40.025, 40.05, 40.075, 40.1};
  auto front = cloud(times);
  auto rear = cloud(times, true);
  const Eigen::Vector3d world_point(3, 2, 1);
  const Eigen::Vector3d velocity(1, 0.2, 0);
  const Eigen::Matrix3d R_imu_front = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d t_imu_front(0.2, -0.1, 0.05);
  auto raw_front = [&](double t) -> Eigen::Vector3d {
    const Eigen::Matrix3d R = Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return R_imu_front.transpose() * (R.transpose() * (world_point - velocity * t) - t_imu_front);
  };
  for (size_t i = 0; i < times.size(); ++i) {
    Eigen::Vector3d f = raw_front(times[i] - 40.0);
    Eigen::Vector3d r = o.extrinsic.inverse() * f;
    for (size_t k = 0; k < 3; ++k) {
      store(*front, i, k * 4, static_cast<float>(f[k]));
      store(*rear, i, k * 4, static_cast<float>(r[k]));
    }
  }
  p.push(false, front);
  p.push(true, rear);
  auto scan = p.pop();
  check(scan && scan->points.size() == (rate == 0.0 ? 10U : 4U), "overlap observations retained for common deskew");
  sapphire::ImuEstimator estimator;
  sapphire::SensorParameters sensor;
  estimator.configure(sensor, sapphire::InitializerParameters{}, sapphire::OdometryParameters{});
  estimator.Lid_rot_to_IMU = R_imu_front;
  estimator.Lid_offset_to_IMU = t_imu_front;
  estimator.has_last_imu = true;
  estimator.last_pcl_end_time = estimator.pcl_beg_time = scan->begin;
  estimator.pcl_end_time = scan->end;
  sapphire::StateGroup state;
  state.R.setIdentity();
  state.p.setZero();
  state.v = velocity;
  state.g.setZero();
  state.bg.setZero();
  state.ba.setZero();
  std::deque<sapphire::ImuMeas> imus;
  for (int i = 0; i <= std::llround((scan->end - scan->begin) * 100); ++i) {
    sapphire::ImuMeas imu;
    imu.timestamp = 40.0 + i * 0.01;
    imu.gyro << 0, 0, 1;
    imu.accel.setZero();
    imus.push_back(imu);
  }
  estimator.last_imu = imus.front();
  estimator.deskew(state, scan->points, imus);
  const Eigen::Vector3d expected = raw_front(scan->end - scan->begin);
  for (const auto &point : scan->points) {
    check((Eigen::Vector3d(point.x, point.y, point.z) - expected).norm() < 2e-6,
          "both lidars deskew to same static point under rotation, translation and nonidentity extrinsics");
  }
}
void test_fixed_conservation(double rate, bool reverse_arrival, bool unsorted, double epoch = 50.0) {
  Parameters o;
  o.lidar_merge_hz = rate;
  o.queue_size = 32;
  o.max_wait = 10;
  Processor p(o);
  // 10 Hz front packets, 20 Hz rear packets with 17 ms phase offset.
  // Geometry/IDs are unchanged whether messages arrive front-first or rear-first.
  std::vector<Cloud::SharedPtr> fronts, rears;
  for (int base = 0; base < 600; base += 100) {
    std::vector<double> times;
    for (int i = base; i < base + 100; ++i) times.push_back(epoch + i * 0.001);
    auto m = cloud(times);
    for (int j = 0; j < 100; ++j) store(*m, j, 12, static_cast<float>(base + j));
    fronts.push_back(m);
  }
  for (int base = 0; base < 600; base += 50) {
    std::vector<double> times;
    for (int i = base; i < base + 50; ++i) times.push_back(epoch + 0.017 + i * 0.001);
    auto m = cloud(times, true);
    for (int j = 0; j < 50; ++j) store(*m, j, 12, static_cast<float>(1000 + base + j));
    rears.push_back(m);
  }
  if (unsorted) {
    for (const auto &packets : {fronts, rears}) {
      for (const auto &m : packets) {
        for (size_t i = 0; i < m->width / 2; ++i) {
          auto a = m->data.begin() + i * m->point_step;
          auto b = m->data.begin() + (m->width - 1 - i) * m->point_step;
          std::swap_ranges(a, a + m->point_step, b);
        }
      }
    }
  }
  const auto now = Processor::Clock::now();
  const auto push_all = [&](bool rear) {
    for (const auto &m : rear ? rears : fronts) check(p.push(rear, m, now), "fixed stream packet accepted");
  };
  push_all(reverse_arrival);
  push_all(!reverse_arrival);
  std::set<int> ids;
  double first = -1, last = -1;
  size_t windows = 0;
  while (auto scan = p.pop(now)) {
    if (windows == 0)
      first = scan->begin;
    else
      check(std::abs(scan->begin - last) < 1e-6, "fixed windows contiguous");
    check(std::abs((scan->end - scan->begin) - 1.0 / rate) < 1e-6, "requested output period independent of packets");
    for (const auto &point : scan->points) {
      check(ids.insert(static_cast<int>(point.intensity)).second, "every raw point is emitted at most once");
      check(point.time_offset >= 0 && point.time_offset < 1.0 / rate + 1e-7, "half-open window offsets");
    }
    last = scan->end;
    ++windows;
  }
  check(windows >= 2, "several fixed windows emitted");
  const auto ns = [](double t) { return std::llround(static_cast<long double>(t) * 1000000000.0L); };
  std::set<int> expected;
  for (int i = 0; i < 600; ++i) {
    if (ns(epoch + i * 0.001) >= ns(first) && ns(epoch + i * 0.001) < ns(last)) expected.insert(i);
    if (ns(epoch + 0.017 + i * 0.001) >= ns(first) && ns(epoch + 0.017 + i * 0.001) < ns(last)) expected.insert(1000 + i);
  }
  check(ids == expected, "all eligible raw points are conserved across packet boundaries");
  check(p.statistics().dropped() == 0, "healthy fixed streams have no dropped windows");
  check(p.statistics().front_points + p.statistics().rear_points == ids.size(), "point accounting matches output");
}

void test_fixed_boundaries_and_limits() {
  Parameters o;
  o.lidar_merge_hz = 20;
  Processor p(o);
  // Keep multiple distinct rays with the same timestamp; no epsilon-sized blind stripe.
  const std::vector<double> times{60.0, 60.0499999, 60.05, 60.05, 60.0500001, 60.1};
  p.push(false, cloud(times));
  p.push(true, cloud(times, true));
  auto first = p.pop();
  auto second = p.pop();
  check(first && second && first->points.size() == 4 && second->points.size() == 6,
        "exact half-open membership preserves sub-microsecond and simultaneous rays");
  check(!p.pop(), "do not invent a complete window beyond the watermark");

  bool rejected = false;
  try {
    o.lidar_merge_hz = 50;
    Processor invalid(o);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "reject 50 Hz lidar updates with a configured 200 Hz IMU");
  o.imu_rate_hz = 500;
  Processor allowed(o);
  o.lidar_merge_hz = 2;
  rejected = false;
  try {
    Processor invalid(o);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "reject fusion windows exceeding configured scan duration");

  o = Parameters{};
  o.min_points = 3;
  Processor sparse(o);
  sparse.push(false, cloud({70.0, 70.02, 70.1}));
  sparse.push(true, cloud({70.0, 70.02, 70.1}, true));
  check(!sparse.pop() && sparse.statistics().empty == 1, "minimum points per sensor enforced");

  o = Parameters{};
  o.max_wait = 0.1;
  Processor timeout(o);
  const auto now = Processor::Clock::now();
  timeout.push(true, cloud({80.0, 80.1}, true), now);
  check(!timeout.pop(now + std::chrono::milliseconds(101)) && timeout.buffered_rear() == 0, "startup rear-only outage expires symmetrically");
  Processor gap(o);
  gap.push(false, cloud({90.0, 90.1}), now);
  gap.push(true, cloud({90.0, 90.1}, true), now);
  check(gap.pop().has_value(), "first fixed window");
  gap.push(false, cloud({36090.0, 36090.1}), now);
  gap.push(true, cloud({36090.0, 36090.1}, true), now);
  check(gap.pop().has_value() && gap.statistics().coverage > 300000, "large timestamp gaps skip in bounded work");
}

void test_imu_gate_unchanged() {
  sapphire::Synchronizer sync;
  std::vector<sapphire::LidarPoint> points(1);
  points.front().x = 1;
  for (int i = 0; i < 10; ++i) {
    sapphire::ImuMeas imu;
    imu.timestamp = 100.0 + i * 0.005;
    imu.gyro.setZero();
    imu.accel.setZero();
    check(sync.push_imu(imu), "IMU input accepted");
  }
  sapphire::MeasGroup group;
  check(sync.push_lidar(100, points, 100.016), "enqueue too-fast update");
  check(!sync.sync_packages(group) && group.imu_buf.size() == 4, "four new IMU samples still rejected");
  check(sync.push_lidar(100.016, points, 100.041), "enqueue admissible update");
  check(sync.sync_packages(group) && group.imu_buf.size() == 5, "five new IMU samples still accepted");
  sapphire::Synchronizer epoch_sync(0.1);
  check(epoch_sync.push_lidar(1700000000.05, points, 1700000000.15), "do not reject 100 ms windows due to epoch rounding");
  check(!epoch_sync.push_lidar(1700000001.05, points, 1700000001.15001), "duration tolerance does not admit genuinely oversized scans");
}
void test_single_formats() {
  sapphire::SensorParameters parameters;
  parameters.point_filter_num = 1;
  for (int type : {0, 1}) {
    parameters.lidar_type = type;
    sapphire_ros::LidarProcessor processor(parameters);
    auto message = cloud(type == 0 ? std::vector<double>{0, 50000000} : std::vector<double>{10, 10.05});
    message->header.stamp.sec = 10;
    double timestamp = 0;
    std::optional<double> end;
    std::vector<sapphire::LidarPoint> points;
    check(processor.process(message, timestamp, points, end), "single cloud decoded");
    check(points.size() == 2 && std::abs(timestamp - 10.0) < 1e-9, "single cloud timestamp");
    check(std::abs(points.back().time_offset - 0.05) < 1e-6, "LiDAR type selects nanoseconds or seconds");
  }
}
void test_hesai_input() {
  sapphire::SensorParameters parameters;
  parameters.lidar_type = 2;
  parameters.point_filter_num = 2;
  parameters.max_scan_duration = 0.11;
  sapphire_ros::LidarProcessor processor(parameters);
  for (bool big : {false, true}) {
    const double epoch = 1700000000.0;
    const std::vector<double> times{epoch + .06, epoch, epoch + .04, epoch + .10};
    auto message = cloud(times, false, big, true);
    // PCL-style padding / ring / timestamp offset, not our compact fixture layout.
    message->point_step = 48;
    message->row_step = message->width * 48 + 8;
    message->data.assign(message->row_step * message->height, 0x5a);
    message->fields[3].offset = 16;
    message->fields[4].offset = 24;
    F ring; ring.name = "ring"; ring.offset = 32; ring.count = 1; ring.datatype = F::UINT16;
    message->fields.push_back(ring);
    std::reverse(message->fields.begin(), message->fields.end());
    for (size_t i = 0; i < times.size(); ++i) {
      store(*message, i, 0, 1.0F); store(*message, i, 4, 0.0F); store(*message, i, 8, 0.0F);
      store(*message, i, 16, 12.0F); store(*message, i, 24, times[i]); store(*message, i, 32, uint16_t(i));
    }
    const auto original = message->data;
    double begin;
    std::optional<double> end;
    std::vector<sapphire::LidarPoint> points;
    check(processor.process(message, begin, points, end), "Hesai padded unsorted epoch cloud");
    check(begin == epoch && end && *end == times[3], "both excluded endpoints survive decimation");
    check(points.size() == 2 && std::abs(points[0].time_offset - .06) < 1e-6 &&
          std::abs(points[1].time_offset - .04) < 1e-6 && points[0].intensity == 12, "decode offsets and intensity");
    check(original == message->data, "input buffer unchanged");
    sapphire::Synchronizer sync(.11);
    check(sync.push_lidar(begin, points, end), "single strict output accepted by core");
    for (int i = 0; i <= 11; ++i) {
      sapphire::ImuMeas imu; imu.timestamp = epoch + i * .01;
      imu.gyro.setZero(); imu.accel.setZero(); check(sync.push_imu(imu), "IMU accepted");
    }
    sapphire::MeasGroup group;
    check(sync.sync_packages(group) && group.lidar_end_time == *end && group.lidar_cloud->size() == 2,
          "full declared end delivered without next LiDAR frame");
  }
  auto decode = [&](Cloud::SharedPtr m, bool expected) {
    double begin = 123; std::optional<double> end = 456;
    std::vector<sapphire::LidarPoint> points(1);
    check(processor.process(m, begin, points, end) == expected, "Hesai acceptance mismatch");
    if (!expected) check(points.empty() && !end, "failure clears reused output");
  };
  decode(nullptr, false);
  decode(cloud({1, 1}), false);
  decode(cloud({1, 1.2}), false);
  decode(cloud({-1, -.95}), false);
  decode(cloud({1.7e18, 1.7e18 + 1e8}), false);
  decode(cloud({1, std::numeric_limits<double>::quiet_NaN(), 1.05}), false);
  decode(cloud({1, std::numeric_limits<double>::infinity(), 1.05}), false);
  auto m = cloud({1, 1.05}); m->fields.back().datatype = F::FLOAT32; decode(m, false);
  m = cloud({1, 1.05}); m->fields.back().name = "time"; decode(m, false);
  m = cloud({1, 1.05}); m->fields.back().offset = 20; decode(m, false);
  m = cloud({1, 1.05}); m->data.pop_back(); decode(m, false);
  m = cloud({1, 1.05}); m->row_step = 1; decode(m, false);
  m = cloud({1, 1.05}); store(*m, 0, 0, 0.0F); decode(m, false);
  parameters.point_filter_num = 1;
  sapphire_ros::LidarProcessor unfiltered(parameters);
  m = cloud({1, 1.04, 1.10}); store(*m, 0, 0, 0.0F); store(*m, 2, 0, 0.0F);
  double begin; std::optional<double> end; std::vector<sapphire::LidarPoint> points;
  check(unfiltered.process(m, begin, points, end) && begin == 1 && *end == 1.10 &&
        points.size() == 1 && std::abs(points[0].time_offset - .04) < 1e-6, "blind endpoint removal preserves scan bounds");
  for (int invalid : {0, -1}) {
    parameters.point_filter_num = invalid;
    bool rejected = false;
    try { sapphire_ros::LidarProcessor bad(parameters); } catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "reject invalid direct decimation configuration");
  }
}

void test_lidar_type_config() {
  const auto path = std::filesystem::temp_directory_path() / ("sapphire-lidar-type-" + std::to_string(getpid()) + ".toml");
  for (const auto &value : {"\"livox\"", "\"airy\"", "\"hesai\"", "0", "1", "2", "3", "\"bad\""}) {
    { std::ofstream out(path); out << "[sensor]\nlidar_type = " << value << "\n"; }
    const std::string v(value);
    bool rejected = false;
    try {
      auto p = sapphire::load_parameters(path);
      const int expected = v == "\"livox\"" || v == "0" ? 0 : (v == "\"airy\"" || v == "1" ? 1 : 2);
      check(p.sensor.lidar_type == expected, "TOML LiDAR selector mapping");
    } catch (const std::invalid_argument &) { rejected = true; }
    check(rejected == (v == "3" || v == "\"bad\""), "TOML LiDAR selector validation");
  }
  std::filesystem::remove(path);
}
}  // namespace

int main() {
  try {
    test_windows();
    test_validation_and_waits();
    test_time_offset_and_end();
    test_moving_rig_deskew();
    test_moving_rig_deskew(20.0);
    for (double rate : {5.0, 10.0, 20.0, 30.0}) {
      test_fixed_conservation(rate, false, false);
      test_fixed_conservation(rate, true, true);
    }
    test_fixed_conservation(20, true, false, 1700000000.0);
    test_fixed_boundaries_and_limits();
    test_imu_gate_unchanged();
    test_single_formats();
    test_hesai_input();
    test_lidar_type_config();
    std::cout << "lidar: all synchronization, decoding and moving-rig tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
