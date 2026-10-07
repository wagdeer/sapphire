#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "lidar_preprocess.hpp"

namespace sapphire_ros {

struct LidarSyncParameters {
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();  // rear LiDAR -> front LiDAR
  double front_time_offset = 0.0;  // corrected time = device timestamp + offset, seconds
  double rear_time_offset = 0.0;
  double max_scan_duration = 0.25;
  double max_time_gap = 0.02;
  double max_wait = 0.3;
  double lidar_merge_hz = 10.0;  // 0: legacy front-frame windows; >0: symmetric fixed windows
  double imu_rate_hz = 200.0;    // Expected INPUT rate; does not configure the IMU hardware.
  size_t min_points = 1;  // Valid points per LiDAR per window.
  size_t queue_size = 8;
  int point_filter_num = 1;
  double blind_squared = 0.01;
};

// Single callback group owns this object. Queues hold the ROS message, not a PCL copy.
// Input messages may contribute to several output windows; both streams retain
// their unconsumed suffixes. Spatial overlap is not assumed. lidar_merge_hz=0
// keeps the earlier front-frame policy for compatibility and comparisons.
class LidarSync {
 public:
  using Clock = std::chrono::steady_clock;
  struct Scan {
    double begin = 0.0, end = 0.0;
    std::vector<sapphire::LidarPoint> points;
  };
  struct Statistics {
    size_t invalid = 0, out_of_order = 0, overflow = 0, timeout = 0, coverage = 0, empty = 0, emitted = 0;
    size_t front_points = 0, rear_points = 0;
    size_t dropped() const { return invalid + out_of_order + overflow + timeout + coverage + empty; }
  };

  explicit LidarSync(LidarSyncParameters parameters) : parameters_(std::move(parameters)) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (!positive(parameters_.max_scan_duration) || !positive(parameters_.max_time_gap) || !positive(parameters_.max_wait) || parameters_.queue_size < 2 ||
        parameters_.point_filter_num < 1 || !std::isfinite(parameters_.blind_squared) || parameters_.blind_squared < 0.0 ||
        !std::isfinite(parameters_.front_time_offset) || !std::isfinite(parameters_.rear_time_offset) || !parameters_.extrinsic.matrix().allFinite() ||
        !parameters_.extrinsic.linear().isUnitary(1e-6) || std::abs(parameters_.extrinsic.linear().determinant() - 1.0) > 1e-6) {
      throw std::invalid_argument("Invalid dual calibration, timing, queue or filtering parameters");
    }
    if (!positive(parameters_.imu_rate_hz) || !std::isfinite(parameters_.lidar_merge_hz) || parameters_.lidar_merge_hz < 0.0 ||
        parameters_.lidar_merge_hz > 100.0 || parameters_.min_points == 0) {
      throw std::invalid_argument("lidar.lidar_merge_hz must be in [0, 100]; min_points must be positive");
    }
    if (parameters_.lidar_merge_hz > 0.0) {
      if (parameters_.lidar_merge_hz * 5.0 > parameters_.imu_rate_hz) {
        throw std::invalid_argument("Fusion rate is too high: lidar_merge_hz must be <= imu_rate_hz / 5 (leave margin for jitter)");
      }
      if (1.0 / parameters_.lidar_merge_hz > parameters_.max_scan_duration) {
        throw std::invalid_argument("Fusion period exceeds sensor.max_scan_duration");
      }
      period_ns_ = std::llround(1e9 / parameters_.lidar_merge_hz);
    }
  }

  bool push(bool rear, sensor_msgs::msg::PointCloud2::ConstSharedPtr message, Clock::time_point now = Clock::now()) {
    Frame frame;
    frame.message = std::move(message);
    frame.arrival = now;
    frame.offset = rear ? parameters_.rear_time_offset : parameters_.front_time_offset;
    if (!inspect(frame)) {
      ++stats_.invalid;
      return false;
    }
    auto &last = rear ? last_rear_ : last_front_;
    auto &last_end = rear ? last_rear_end_ : last_front_end_;
    auto &frame_id = rear ? rear_frame_id_ : front_frame_id_;
    if (frame.message->header.frame_id.empty() || (!frame_id.empty() && frame.message->header.frame_id != frame_id)) {
      ++stats_.invalid;
      return false;
    }
    if (frame.begin <= last || frame.end <= last_end || (period_ns_ && std::isfinite(last_end) && to_ns(frame.begin) < to_ns(last_end))) {
      ++stats_.out_of_order;
      return false;
    }
    last = frame.begin;
    last_end = frame.end;
    if (period_ns_) prepare_order(frame);
    frame_id = frame.message->header.frame_id;
    auto &queue = rear ? rear_ : front_;
    if (queue.size() == parameters_.queue_size) {
      queue.pop_front();
      ++stats_.overflow;
    }
    queue.emplace_back(std::move(frame));
    return true;
  }

  std::optional<Scan> pop(Clock::time_point now = Clock::now()) { return period_ns_ ? pop_fixed(now) : pop_front_window(now); }

 private:
  std::optional<Scan> pop_front_window(Clock::time_point now) {
    while (!front_.empty()) {
      const Frame &front = front_.front();
      const double begin = std::max(front.begin, emitted_until_);
      if (front.end <= begin) {
        front_.pop_front();
        ++stats_.empty;
        continue;
      }
      while (!rear_.empty() && rear_.front().end < begin) {
        rear_.pop_front();
      }
      if (std::chrono::duration<double>(now - front.arrival).count() > parameters_.max_wait) {
        front_.pop_front();
        ++stats_.timeout;
        continue;
      }
      // Wait for the rear stream's acquisition watermark, not similar headers.
      if (rear_.empty() || rear_.back().end + kTimeEpsilon < front.end) {
        return std::nullopt;
      }
      double covered_until = begin;
      bool covered = rear_.front().begin <= begin + parameters_.max_time_gap;
      for (const Frame &rear : rear_) {
        if (rear.begin > front.end) break;
        if (rear.begin > covered_until + parameters_.max_time_gap) covered = false;
        covered_until = std::max(covered_until, rear.end);
      }
      if (!covered || covered_until + kTimeEpsilon < front.end) {
        front_.pop_front();
        ++stats_.coverage;
        continue;
      }
      Scan output;
      output.begin = begin;
      output.end = front.end;
      size_t capacity = front.count;
      for (const Frame &rear : rear_) {
        if (rear.begin > output.end) break;
        capacity += rear.count;
      }
      output.points.reserve(capacity / static_cast<size_t>(parameters_.point_filter_num) + 3);
      const size_t front_points = append(front, false, output, -std::numeric_limits<double>::infinity());
      size_t rear_points = 0;
      double rear_until = -std::numeric_limits<double>::infinity();
      for (const Frame &rear : rear_) {
        if (rear.begin > output.end) break;
        rear_points += append(rear, true, output, rear_until);
        rear_until = std::max(rear_until, rear.end);
      }
      front_.pop_front();
      if (front_points < parameters_.min_points || rear_points < parameters_.min_points) {
        ++stats_.empty;
        continue;  // Never quietly turn dual mode into a single-lidar scan.
      }
      emitted_until_ = output.end;
      while (!rear_.empty() && rear_.front().end <= emitted_until_) rear_.pop_front();
      ++stats_.emitted;
      stats_.front_points += front_points;
      stats_.rear_points += rear_points;
      return output;  // Synchronizer sorts once by the corrected point timestamps.
    }
    return std::nullopt;
  }

 public:
  const Statistics &statistics() const { return stats_; }
  size_t buffered_front() const { return front_.size(); }
  size_t buffered_rear() const { return rear_.size(); }

 private:
  // Sub-microsecond rounding is possible when adding offsets to epoch seconds.
  static constexpr double kTimeEpsilon = 1e-6;
  struct Frame {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    const sensor_msgs::msg::PointField *x = nullptr, *y = nullptr, *z = nullptr, *intensity = nullptr, *time = nullptr;
    size_t count = 0;
    double begin = std::numeric_limits<double>::infinity();
    double end = -std::numeric_limits<double>::infinity();
    double offset = 0.0;
    Clock::time_point arrival;
    // Only non-monotonic input needs an index array. XYZ always stays in the ROS buffer.
    std::vector<size_t> order;
    size_t cursor = 0;
    bool monotonic = true;
    const std::uint8_t *point(size_t i) const {
      return message->data.data() + (i / message->width) * message->row_step + (i % message->width) * message->point_step;
    }
    double stamp(size_t i) const { return detail::load_scalar<double>(point(i) + time->offset, message->is_bigendian) + offset; }
  };

  bool inspect(Frame &f) const {
    if (!f.message || !detail::valid_layout(*f.message)) return false;
    const auto &m = *f.message;
    f.x = detail::find_field(m, "x");
    f.y = detail::find_field(m, "y");
    f.z = detail::find_field(m, "z");
    f.intensity = detail::find_field(m, "intensity");
    f.time = detail::find_field(m, "timestamp");
    if (!detail::valid_scalar(f.x, m.point_step) || !detail::valid_scalar(f.y, m.point_step) || !detail::valid_scalar(f.z, m.point_step) ||
        !detail::valid_scalar(f.time, m.point_step) || f.time->datatype != sensor_msgs::msg::PointField::FLOAT64 ||
        (f.intensity && !detail::valid_scalar(f.intensity, m.point_step)))
      return false;
    f.count = static_cast<size_t>(m.width) * m.height;
    return detail::inspect_absolute_times(m, f.time, f.offset, parameters_.max_scan_duration,
                                          f.begin, f.end, f.monotonic);
  }

  static int64_t to_ns(double seconds) { return std::llround(static_cast<long double>(seconds) * 1000000000.0L); }
  static double to_seconds(int64_t ns) { return static_cast<double>(ns) * 1e-9; }

  size_t selected_count(const Frame &f) const { return (f.count - 1) / static_cast<size_t>(parameters_.point_filter_num) + 1; }
  size_t selected_index(const Frame &f, size_t position) const {
    return f.order.empty() ? position * static_cast<size_t>(parameters_.point_filter_num) : f.order[position];
  }
  void prepare_order(Frame &f) const {
    if (f.monotonic) return;
    f.order.resize(selected_count(f));
    for (size_t i = 0; i < f.order.size(); ++i) f.order[i] = i * static_cast<size_t>(parameters_.point_filter_num);
    std::stable_sort(f.order.begin(), f.order.end(), [&f](size_t a, size_t b) { return f.stamp(a) < f.stamp(b); });
  }

  bool covers(const std::deque<Frame> &queue, int64_t begin, int64_t end) const {
    int64_t covered = begin;
    const int64_t gap = to_ns(parameters_.max_time_gap);
    for (const Frame &f : queue) {
      const int64_t a = to_ns(f.begin), b = to_ns(f.end);
      if (b < begin) continue;
      if (a > covered + gap) return false;
      covered = std::max(covered, b);
      if (covered >= end) return true;
    }
    return false;
  }

  void retire_before(std::deque<Frame> &queue, int64_t begin) {
    while (!queue.empty() && to_ns(queue.front().end) < begin) queue.pop_front();
  }

  size_t append_fixed(std::deque<Frame> &queue, bool rear, int64_t begin, int64_t end, Scan &output) {
    const size_t before = output.points.size();
    for (Frame &f : queue) {
      if (to_ns(f.begin) >= end) break;
      while (f.cursor < selected_count(f)) {
        const size_t i = selected_index(f, f.cursor);
        const int64_t t = to_ns(f.stamp(i));
        if (t >= end) break;  // [begin, end): every raw observation belongs to exactly one window.
        ++f.cursor;
        if (t < begin) continue;
        append_point(f, i, rear, static_cast<float>((t - begin) * 1e-9), output.points);
      }
    }
    return output.points.size() - before;
  }

  std::optional<Scan> pop_fixed(Clock::time_point now) {
    if (!window_begin_ns_) {
      // No stream is the master, including during startup and sensor outages.
      for (auto *queue : {&front_, &rear_}) {
        while (!queue->empty() && std::chrono::duration<double>(now - queue->front().arrival).count() > parameters_.max_wait) {
          queue->pop_front();
          ++stats_.timeout;
        }
      }
      if (front_.empty() || rear_.empty()) return std::nullopt;
      window_begin_ns_ = std::max(to_ns(front_.front().begin), to_ns(rear_.front().begin));
    }
    while (true) {
      int64_t begin = *window_begin_ns_;
      retire_before(front_, begin);
      retire_before(rear_, begin);
      if (front_.empty() && rear_.empty()) return std::nullopt;
      // Fast-forward whole empty windows after an outage/overflow instead of
      // spinning once for every elapsed period (potentially hours in rosbag).
      int64_t earliest = begin;
      if (!front_.empty()) earliest = std::max(earliest, to_ns(front_.front().begin));
      if (!rear_.empty()) earliest = std::max(earliest, to_ns(rear_.front().begin));
      const int64_t skipped = (earliest - begin) / period_ns_;
      if (skipped > 0) {
        stats_.coverage += static_cast<size_t>(skipped);
        *window_begin_ns_ += skipped * period_ns_;
        continue;
      }
      const int64_t end = begin + period_ns_;
      const bool ready = !front_.empty() && !rear_.empty() && to_ns(front_.back().end) >= end && to_ns(rear_.back().end) >= end;
      auto arrival = Clock::time_point::max();
      for (const auto *queue : {&front_, &rear_}) {
        if (!queue->empty()) arrival = std::min(arrival, queue->front().arrival);
      }
      if (std::chrono::duration<double>(now - arrival).count() > parameters_.max_wait) {
        *window_begin_ns_ = end;
        ++stats_.timeout;
        continue;
      }
      if (!ready) return std::nullopt;
      if (!covers(front_, begin, end) || !covers(rear_, begin, end)) {
        *window_begin_ns_ = end;
        ++stats_.coverage;
        continue;
      }
      Scan output;
      output.begin = to_seconds(begin);
      output.end = to_seconds(end);
      // Reserve an estimate, not two complete scans for every tiny subframe.
      size_t capacity = 0;
      for (const auto *queue : {&front_, &rear_}) {
        for (const Frame &f : *queue) {
          if (to_ns(f.begin) >= end) break;
          const double fraction = std::min(1.0, static_cast<double>(period_ns_) / std::max(1.0, static_cast<double>(to_ns(f.end) - to_ns(f.begin))));
          capacity += static_cast<size_t>(std::ceil(selected_count(f) * fraction));
        }
      }
      output.points.reserve(capacity);
      const size_t front_points = append_fixed(front_, false, begin, end, output);
      const size_t rear_points = append_fixed(rear_, true, begin, end, output);
      *window_begin_ns_ = end;
      if (front_points < parameters_.min_points || rear_points < parameters_.min_points) {
        ++stats_.empty;
        continue;
      }
      ++stats_.emitted;
      stats_.front_points += front_points;
      stats_.rear_points += rear_points;
      return output;
    }
  }

  void append_point(const Frame &f, size_t i, bool rear, float offset, std::vector<sapphire::LidarPoint> &points) const {
    double x, y, z, intensity = 0.0;
    const auto *raw = f.point(i);
    detail::read_numeric(raw, f.x, f.message->is_bigendian, x);
    detail::read_numeric(raw, f.y, f.message->is_bigendian, y);
    detail::read_numeric(raw, f.z, f.message->is_bigendian, z);
    detail::read_numeric(raw, f.intensity, f.message->is_bigendian, intensity);
    Eigen::Vector3d point(x, y, z);
    if (!point.allFinite() || !std::isfinite(intensity) || point.squaredNorm() <= parameters_.blind_squared) return;
    if (rear) point = parameters_.extrinsic * point;
    sapphire::LidarPoint p;
    p.x = static_cast<float>(point.x());
    p.y = static_cast<float>(point.y());
    p.z = static_cast<float>(point.z());
    p.intensity = static_cast<float>(intensity);
    p.time_offset = offset;
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.intensity)) points.push_back(p);
  }

  size_t append(const Frame &f, bool rear, Scan &output, double previous_frame_end) const {
    const size_t before = output.points.size();
    for (size_t i = 0; i < f.count; i += static_cast<size_t>(parameters_.point_filter_num)) {
      const double t = f.stamp(i);
      if (t < output.begin - kTimeEpsilon || t > output.end + kTimeEpsilon || t <= emitted_until_ + kTimeEpsilon ||
          t <= previous_frame_end + kTimeEpsilon)
        continue;
      append_point(f, i, rear, static_cast<float>(std::clamp(t - output.begin, 0.0, output.end - output.begin)), output.points);
    }
    return output.points.size() - before;
  }

  LidarSyncParameters parameters_;
  Statistics stats_;
  std::deque<Frame> front_, rear_;
  std::string front_frame_id_, rear_frame_id_;
  double last_front_ = -std::numeric_limits<double>::infinity();
  double last_rear_ = -std::numeric_limits<double>::infinity();
  double last_front_end_ = -std::numeric_limits<double>::infinity();
  double last_rear_end_ = -std::numeric_limits<double>::infinity();
  double emitted_until_ = -std::numeric_limits<double>::infinity();
  int64_t period_ns_ = 0;
  std::optional<int64_t> window_begin_ns_;
};

}  // namespace sapphire_ros
