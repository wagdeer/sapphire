#pragma once
// NCLT packets are bucketed by sensor time before reaching the shared pipeline.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
inline double elapsed_ms(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }
struct __attribute__((packed)) Packet {
  uint16_t magic[4];
  uint32_t count;
  uint64_t stamp;
  uint32_t pad;
};
struct Hit {
  uint16_t x, y, z;
  uint8_t intensity, ring;
};
static_assert(sizeof(Packet) == 24 && sizeof(Hit) == 8);
struct RawPoint {
  float x, y, z, intensity;
  double time;
};
struct RawImu {
  double time = 0;
  std::array<double, 3> acc{}, gyro{};
};

class ImuReader {
 public:
  explicit ImuReader(const char* path) : input_(path) {
    if (!input_) throw std::runtime_error("open IMU");
    next();
  }
  void next() {
    std::string line;
    if (!std::getline(input_, line)) {
      stamp = -1;
      return;
    }
    char* end = nullptr;
    stamp = std::strtoll(line.c_str(), &end, 10);
    std::array<double, 9> values;
    for (double& v : values) {
      if (*end != ',') throw std::runtime_error("malformed IMU CSV");
      const char* start = end + 1;
      v = std::strtod(start, &end);
      if (start == end) throw std::runtime_error("malformed IMU value");
    }
    sample = {stamp * 1e-6, {values[3], values[4], values[5]}, {values[6], values[7], values[8]}};
  }
  int64_t stamp = -1;
  RawImu sample;

 private:
  std::ifstream input_;
};

/// Decode the same on-disk packets, 100 ms buckets and common 1–80 m input
/// range used in the earlier Sapphire runs. All disk decoding is untimed.
template <typename Runner>
void replay(Runner& runner, const char* hits_path, const char* imu_path, double offset, double duration) {
  std::ifstream input(hits_path, std::ios::binary);
  if (!input) throw std::runtime_error("open LiDAR packets");
  ImuReader imu(imu_path);
  Packet packet;
  std::vector<Hit> hits;
  std::vector<RawPoint> sweep;
  sweep.reserve(60000);
  int64_t origin = -1, start = -1, window = -1, last_hit = -1, last_imu = -1;
  size_t scans = 0;
  const auto begin = Clock::now();
  const char* pace_env = std::getenv("REPLAY_RATE");
  const double rate = pace_env ? std::stod(pace_env) : 0.;
  Clock::time_point first_scan;
  auto flush = [&] {
    if (sweep.size() < 1000 || imu.stamp < 0) return;
    if (rate > 0.) {
      if (scans == 0) first_scan = Clock::now();
      const auto elapsed = std::chrono::duration<double>(window * 0.1 / rate);
      std::this_thread::sleep_until(first_scan + std::chrono::duration_cast<Clock::duration>(elapsed));
    }
    while (imu.stamp >= 0) {
      const auto stamp = imu.stamp;
      if (stamp > last_imu) {
        runner.imu(imu.sample);
        last_imu = stamp;
      }
      imu.next();
      if (last_imu >= last_hit) break;
    }
    if (last_imu < last_hit) return;
    runner.lidar(sweep);
    if (++scans % 1000 == 0) std::cerr << "scans=" << scans << " wall_ms=" << elapsed_ms(begin) << '\n';
  };
  while (input.read(reinterpret_cast<char*>(&packet), sizeof packet)) {
    if (packet.magic[0] != 0xad9c || packet.magic[3] != 0xad9c || !packet.count || packet.count > 100000)
      throw std::runtime_error("invalid Velodyne packet");
    const int64_t stamp = packet.stamp;
    if (origin < 0) {
      origin = stamp;
      start = origin + std::llround(offset * 1e6);
      if (offset > 0)
        while (imu.stamp >= 0 && imu.stamp < start - 50000) imu.next();
    }
    if (stamp < start) {
      input.seekg(packet.count * sizeof(Hit), std::ios::cur);
      continue;
    }
    if (stamp - start > std::llround(duration * 1e6)) break;
    const auto new_window = (stamp - start) / 100000;
    if (new_window != window) {
      flush();
      sweep.clear();
      window = new_window;
    }
    hits.resize(packet.count);
    if (!input.read(reinterpret_cast<char*>(hits.data()), hits.size() * sizeof(Hit))) throw std::runtime_error("truncated packet");
    for (const auto& hit : hits) {
      const float x = .005f * hit.x - 100.f, y = .005f * hit.y - 100.f, z = .005f * hit.z - 100.f;
      const float squared = x * x + y * y + z * z;
      if (squared >= 1.f && squared <= 6400.f) sweep.push_back({x, y, z, float(hit.intensity), stamp * 1e-6});
    }
    last_hit = stamp;
  }
  if (last_hit >= 0 && !sweep.empty() && last_hit * 1e-6 - sweep.front().time >= .07) flush();
  std::cerr << "finished scans=" << scans << " wall_ms=" << elapsed_ms(begin) << '\n';
}
