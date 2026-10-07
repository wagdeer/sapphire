// Offline sensor-time replay of the production pipeline, with completion-based
// backpressure. No ROS transport/arrival-latency claim; all scans are consumed.
#include <sys/resource.h>

#include <fstream>
#include <iomanip>
#include <iostream>

#include "pipeline.hpp"
using namespace sapphire;
template <class T>
T read(std::istream& f) {
  T x{};
  f.read(reinterpret_cast<char*>(&x), sizeof(x));
  if (!f) throw std::runtime_error("truncated input");
  return x;
}
int main(int argc, char** argv) {
  try {
    if (argc < 4) {
      std::cerr << "frontend_replay DATA_DIR CONFIG OUTPUT [duration_s]\n";
      return 2;
    }
    const std::filesystem::path data(argv[1]), out(argv[3]);
    if (std::filesystem::exists(out)) throw std::runtime_error("output exists; choose a new run directory");
    std::filesystem::create_directories(out);
    const double duration = argc > 4 ? std::stod(argv[4]) : 1e9;
    auto p = load_parameters(argv[2]);
    p.general.save_path = (out / "maps").string();
    p.general.save_map = 0;
    p.pose_graph.enabled = false;
    p.pose_graph.scene_refresh = false;
    p.pose_graph.visual.enabled = false;
    p.navi_map.enabled = false;
    std::ofstream odom(out / "odom.csv"), observations(out / "observations.csv"), timing(out / "timing.csv"), map(out / "map.bin", std::ios::binary);
    odom << std::setprecision(17) << "stamp_s,x,y,z,qx,qy,qz,qw,vx,vy,vz\n";
    observations << std::setprecision(17) << "stamp_s,accepted,ms,x,y,z,vx,vy,vz\n";
    timing << std::setprecision(17) << "stamp_s,ms\n";
    std::mutex mutex;
    std::condition_variable cv;
    double done = -1;
    std::size_t outputs = 0, accepted = 0, rejected = 0, completed = 0, scans = 0, imu_count = 0, map_points = 0;
    OutputSink sink;
    sink.odom_state = [&](const StateGroup& s) {
      Eigen::Quaterniond q(s.R);
      odom << s.t << ',' << s.p.x() << ',' << s.p.y() << ',' << s.p.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
           << s.v.x() << ',' << s.v.y() << ',' << s.v.z() << '\n';
      ++outputs;
    };
    sink.lidar_update = [&](const StateGroup& s, bool ok, double ms) {
      accepted += ok;
      rejected += !ok;
      observations << s.t << ',' << ok << ',' << ms << ',' << s.p.x() << ',' << s.p.y() << ',' << s.p.z() << ',' << s.v.x() << ',' << s.v.y() << ','
                   << s.v.z() << '\n';
    };
    sink.scan_processed = [&](double stamp, double ms) {
      timing << stamp << ',' << ms << '\n';
      {
        std::lock_guard<std::mutex> lock(mutex);
        done = stamp;
        ++completed;
      }
      cv.notify_all();
    };
    sink.local_map = [&](auto cloud) {
      for (const auto& q : *cloud) {
        float v[3] = {float(q.x()), float(q.y()), float(q.z())};
        map.write(reinterpret_cast<char*>(v), sizeof(v));
        ++map_points;
      }
    };
    const auto start = std::chrono::steady_clock::now();
    SlamPipeline pipeline(p, sink);
    std::ifstream imufile(data / "imu.bin", std::ios::binary), lidar(data / "lidar.bin", std::ios::binary);
    if (!imufile || !lidar) throw std::runtime_error("missing input");
    double imu_tip = -1, first = -1;
    while (lidar.peek() != EOF) {
      const double begin = read<double>(lidar), end = read<double>(lidar);
      const auto n = read<uint32_t>(lidar);
      if (first < 0) first = begin;
      if (begin - first > duration) break;
      if (n > 16 * 1024 * 1024 / sizeof(LidarPoint)) throw std::runtime_error("oversized input scan");
      std::vector<LidarPoint> cloud(n);
      for (auto& v : cloud) {
        v.x = read<float>(lidar);
        v.y = read<float>(lidar);
        v.z = read<float>(lidar);
        v.intensity = read<float>(lidar);
        v.time_offset = read<float>(lidar);
      }
      while (imu_tip <= end && imufile.peek() != EOF) {
        ImuMeas m;
        m.timestamp = read<double>(imufile);
        for (int j = 0; j < 3; ++j) m.gyro[j] = read<double>(imufile);
        for (int j = 0; j < 3; ++j) m.accel[j] = read<double>(imufile);
        if (!pipeline.push_imu(m)) throw std::runtime_error("IMU input refused");
        imu_tip = m.timestamp;
        ++imu_count;
      }
      if (imu_tip <= end) throw std::runtime_error("missing tail IMU coverage");
      if (!pipeline.push_lidar(begin, std::move(cloud), end)) throw std::runtime_error("LiDAR input refused");
      ++scans;
      std::unique_lock<std::mutex> lock(mutex);
      if (!cv.wait_for(lock, std::chrono::seconds(30), [&] { return done >= end; })) throw std::runtime_error("scan completion timeout");
      if (scans % 500 == 0) std::cout << "processed=" << scans << " sensor_s=" << end - first << std::endl;
    }
    pipeline.shutdown();
    if (auto failure = pipeline.producerFailure()) std::rethrow_exception(failure);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    rusage u{};
    getrusage(RUSAGE_SELF, &u);
    std::ofstream summary(out / "run.json");
    summary << "{\"frontend\":\"" << p.odometry.frontend << "\",\"scans\":" << scans << ",\"completed\":" << completed << ",\"odom\":" << outputs
            << ",\"imu\":" << imu_count << ",\"accepted\":" << accepted << ",\"rejected\":" << rejected << ",\"wall_s\":" << elapsed
            << ",\"cpu_s\":" << u.ru_utime.tv_sec + u.ru_utime.tv_usec / 1e6 + u.ru_stime.tv_sec + u.ru_stime.tv_usec / 1e6
            << ",\"peak_rss_kb\":" << u.ru_maxrss << ",\"map_points\":" << map_points << "}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
