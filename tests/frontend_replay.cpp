// Offline sensor-time replay of the production pipeline, with completion-based
// backpressure. No ROS transport/arrival-latency claim; all scans are consumed.
#include <sys/resource.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <iostream>

#include "slam.hpp"
#include "sensor/lidar_handler.hpp"
#include <set>
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
    if (argc < 5) {
      std::cerr << "frontend_replay DATA_DIR CONFIG SENSOR_CONFIG OUTPUT [duration_s] [--backend] [--no-map] [--raw]\n";
      return 2;
    }
    const std::filesystem::path data(argv[1]), out(argv[4]);
    if (std::filesystem::exists(out)) throw std::runtime_error("output exists; choose a new run directory");
    std::filesystem::create_directories(out);
    const double duration = argc > 5 ? std::stod(argv[5]) : 1e9;
    const auto sensors = load_sensor_parameters(argv[3]);
    if (sensors.lidars.size() != 1) throw std::invalid_argument("Offline replay takes one already-normalized LiDAR stream");
    auto p = load_parameters(argv[2], sensors.lidars.front());
    p.general.save_path = (out / "maps").string();
    p.recovery.input_timeout_sec = 0; // Offline pacing is not live sensor silence.
    p.pose_graph.enabled = false;
    const LidarHandler handler(p.sensor);
    bool raw_input = false;
    bool write_map = true;
    for (int i=6;i<argc;++i) {
      const std::string flag(argv[i]);
      if (flag=="--backend") p.pose_graph.enabled=true;
      else if (flag=="--no-map") write_map=false;
      else if (flag=="--raw") raw_input=true;
      else throw std::invalid_argument("unknown replay flag: " + flag);
    }

    std::ofstream odom, map;
    std::uint64_t generation = 0;
    const auto open_segment = [&](std::uint64_t value) {
      if (odom.is_open()) odom.close();
      if (map.is_open()) map.close();
      generation = value;
      const auto segment = out / ("session_" + std::to_string(value));
      std::filesystem::create_directories(segment);
      odom.open(segment / "odom.csv", std::ios::app);
      map.open(segment / "map.bin", std::ios::binary | std::ios::app);
      if (odom.tellp() == 0) odom << std::setprecision(17) << "stamp_s,x,y,z,qx,qy,qz,qw,vx,vy,vz\n";
      odom << std::setprecision(17);
    };
    open_segment(0);
    std::ofstream observations(out / "observations.csv"), timing(out / "timing.csv");
    std::ofstream events(out / "events.csv"), recovery(out / "recovery.csv");
    events << std::setprecision(17) << "sequence,generation,stamp_s,disposition,reason\n";
    recovery << "generation,phase,reason,resets,initialization_attempts\n";
    observations << std::setprecision(17) << "stamp_s,accepted,ms,x,y,z,vx,vy,vz\n";
    timing << std::setprecision(17) << "stamp_s,ms,completed_monotonic_s\n";
    std::mutex mutex;
    std::ofstream resources(out / "resources.csv");
    resources << std::setprecision(17) << "stamp_s,rss_kb\n";
    const long page_kb = sysconf(_SC_PAGESIZE) / 1024;
    std::condition_variable cv;
    std::set<std::uint64_t> done;
    std::size_t tracked = 0;
    std::size_t outputs = 0, accepted = 0, rejected = 0, completed = 0, scans = 0, imu_count = 0, map_points = 0;
    SlamPipeline::Callbacks sink;
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
    sink.recovery_changed = [&](const RecoveryProgress& status) {
      if (status.phase == TrackingPhase::Resetting && status.generation != generation) open_segment(status.generation);
      recovery << status.generation << ',' << static_cast<int>(status.phase) << ',' << recoveryReasonName(status.reason)
               << ',' << status.resets << ',' << status.initialization_attempts << std::endl;
      cv.notify_all();
    };
    sink.scan_finished = [&](const ScanResult& result) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        events << result.sequence << ',' << result.generation << ',' << result.stamp << ',' << static_cast<int>(result.disposition)
               << ',' << recoveryReasonName(result.reason) << std::endl;
        if (!done.insert(result.sequence).second) std::terminate();
        ++completed;
        tracked += result.disposition == ScanDisposition::Tracked;
      }
      cv.notify_all();
    };
    sink.scan_processed = [&](double stamp, double ms) {
      timespec completed_time{};clock_gettime(CLOCK_MONOTONIC,&completed_time);
      timing << stamp << ',' << ms << ',' << completed_time.tv_sec+completed_time.tv_nsec*1e-9 << '\n';
      std::ifstream statm("/proc/self/statm");
      std::size_t virtual_pages = 0, resident_pages = 0;
      if (statm >> virtual_pages >> resident_pages) resources << stamp << ',' << resident_pages * page_kb << '\n';
    };
    if (write_map) sink.local_map = [&](auto cloud) {
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
    std::uint64_t input_epoch = 0;
    std::optional<ImuMeas> lookahead;
    const auto wait_recovery = [&] {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
      for (;;) {
        if (auto error = pipeline.producerFailure()) std::rethrow_exception(error);
        const auto status = pipeline.recoveryProgress();
        if (!status.pending && status.phase != TrackingPhase::Resetting) {
          if (status.input_epoch != input_epoch) {
            input_epoch = status.input_epoch;
            // Reset discards the input queue, including the one IMU sampled
            // beyond the previous scan. Re-admit that real lookahead sample.
            if (lookahead) pipeline.push_imu(*lookahead);
          }
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("recovery barrier timeout");
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, std::chrono::milliseconds(20));
      }
    };
    double imu_tip = -1, first = -1;
    while (lidar.peek() != EOF) {
      wait_recovery();
      const double header = read<double>(lidar), recorded_end = read<double>(lidar);
      const auto n = read<uint32_t>(lidar);
      if (n > 16 * 1024 * 1024 / sizeof(LidarPoint)) throw std::runtime_error("oversized input scan");
      std::vector<LidarPoint> cloud(n);
      std::vector<VelodynePoint> raw;
      if (raw_input) raw.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        auto& v = cloud[i];
        v.x = read<float>(lidar); v.y = read<float>(lidar); v.z = read<float>(lidar);
        v.intensity = read<float>(lidar); v.time_offset = read<float>(lidar);
        if (raw_input) { auto& p = raw[i]; p.x=v.x; p.y=v.y; p.z=v.z; p.intensity=v.intensity; p.time=v.time_offset; }
      }
      double begin = header;
      std::optional<double> scan_end = recorded_end;
      bool valid = true;
      if (raw_input) valid = handler.process(header, raw, begin, cloud, scan_end) && scan_end.has_value();
      begin += sensors.lidars.front().time_offset;
      const double end = scan_end.value_or(recorded_end) + sensors.lidars.front().time_offset;
      if (first < 0) first = begin;
      if (begin - first > duration) break;
      while (imu_tip <= end && imufile.peek() != EOF) {
        ImuMeas m;
        m.timestamp = read<double>(imufile);
        for (int j=0;j<3;++j) m.gyro[j]=read<double>(imufile);
        for (int j=0;j<3;++j) m.accel[j]=read<double>(imufile);
        if (!pipeline.push_imu(m)) {
          wait_recovery();
          if (std::isfinite(m.timestamp) && m.timestamp >= 0 && m.gyro.allFinite() && m.accel.allFinite()) {
            pipeline.push_imu(m); // Re-seed a valid clock sample after clearing old history.
            wait_recovery();
          }
        }
        if (std::isfinite(m.timestamp) && m.timestamp>=0 && m.gyro.allFinite() && m.accel.allFinite()) lookahead=m;
        imu_tip = m.timestamp;
        ++imu_count;
      }
      const auto admission = !valid ? pipeline.rejectLidar(end, RecoveryReason::InvalidLidar) :
          imu_tip <= end ? pipeline.rejectLidar(end, RecoveryReason::ImuCoverage) :
          pipeline.push_lidar(begin, std::move(cloud), end);
      ++scans;
      {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, std::chrono::seconds(30), [&] { return done.count(admission.sequence) != 0; }))
          throw std::runtime_error("scan completion timeout");
        done.erase(admission.sequence);
      }
      wait_recovery();
      if (scans % 500 == 0) std::cout << "processed=" << scans << " sensor_s=" << end-first << " generation=" << generation << std::endl;
    }
    const auto recovery_progress = pipeline.recoveryProgress();
    const auto backend_progress=pipeline.drain();
    pipeline.close();
    if (auto failure = pipeline.producerFailure()) std::rethrow_exception(failure);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    rusage u{};
    getrusage(RUSAGE_SELF, &u);
    std::ofstream summary(out / "run.json");
    summary << "{\"frontend\":\"" << p.odometry.frontend << "\",\"scans\":" << scans << ",\"completed\":" << completed << ",\"odom\":" << outputs
            << ",\"tracked\":" << tracked << ",\"resets\":" << recovery_progress.resets << ",\"imu\":" << imu_count << ",\"accepted\":" << accepted << ",\"rejected\":" << rejected << ",\"wall_s\":" << elapsed
            << ",\"cpu_s\":" << u.ru_utime.tv_sec + u.ru_utime.tv_usec / 1e6 + u.ru_stime.tv_sec + u.ru_stime.tv_usec / 1e6
            << ",\"peak_rss_kb\":" << u.ru_maxrss << ",\"backend_nodes\":" << backend_progress.completed << ",\"map_points\":" << map_points << "}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
