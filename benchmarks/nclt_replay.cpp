#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <sys/resource.h>
#include <tbb/global_control.h>

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sapphire/config.hpp>
#include <sapphire/odometry/pipeline.hpp>
#include <stdexcept>

#include <sapphire/benchmarks/nclt_input.hpp>

namespace {

void recordMemory(const std::filesystem::path& directory, const char* phase) {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::ifstream status("/proc/self/status");
  std::string line;
  long resident = 0;
  while (std::getline(status, line)) {
    if (line.compare(0, 6, "VmRSS:") != 0) continue;
    resident = std::stol(line.substr(6));
    break;
  }
  std::ofstream csv(directory / "memory.csv", std::ios::app);
  csv << phase << ',' << resident << ',' << usage.ru_maxrss << '\n';
}

class ExportLog : public spdlog::sinks::base_sink<std::mutex> {
 public:
  explicit ExportLog(std::filesystem::path directory) : directory_(std::move(directory)), revisits_(directory_ / "revisits.log") {}

 private:
  void sink_it_(const spdlog::details::log_msg& message) override {
    const std::string_view text(message.payload.data(), message.payload.size());
    if (text == "[methodB] map_export_begin") recordMemory(directory_, "map_begin");
    if (text == "[methodB] map_export_end") recordMemory(directory_, "map_end");
    constexpr std::string_view prefix = "[methodB] ";
    if (text.substr(0, prefix.size()) == prefix) revisits_ << text << '\n';
  }
  void flush_() override {}
  std::filesystem::path directory_;
  std::ofstream revisits_;
};

template <typename Predicate>
void waitFor(Predicate ready) {
  const auto deadline = Clock::now() + std::chrono::minutes(45);
  while (!ready()) {
    if (Clock::now() >= deadline) throw std::runtime_error("mapping output timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

class Runner {
 public:
  Runner(const sapphire::Config& config, const std::filesystem::path& directory)
      : config_(config), directory_(directory), pipeline_(config), online_(directory_ / "online.tum"), timings_(directory_ / "events.csv") {
    if (!online_ || !timings_) throw std::runtime_error("cannot open benchmark outputs");
    online_ << std::setprecision(17);
    timings_ << std::setprecision(17) << "stamp,ms,fresh,converged\n";
    imu_from_reference_.translation() << .11, .18, .71;
  }

  void imu(const RawImu& imu) {
    pipeline_.pushImu({imu.time, Eigen::Map<const Eigen::Vector3d>(imu.acc.data()), Eigen::Map<const Eigen::Vector3d>(imu.gyro.data())});
  }

  void lidar(const std::vector<RawPoint>& points) {
    const auto start = Clock::now();
    auto scan = std::make_shared<sapphire::PointCloud>();
    scan->reserve(points.size());
    for (const auto& input : points) {
      sapphire::Point point{};
      point.getVector3fMap() << input.x, input.y, input.z;
      point.data[3] = 1.f;
      point.intensity = input.intensity;
      point.timestamp = input.time;
      scan->push_back(point);
    }
    pipeline_.pushLidar(points.front().time, scan);
    const auto pose = pipeline_.latestResult();
    const auto ms = elapsed_ms(start);
    const bool fresh = pipeline_.initialized() && pose.stamp > last_ && pose.stamp > 0.;
    timings_ << points.front().time << ',' << ms << ',' << fresh << ',' << (fresh && pose.converged) << '\n';
    if (fresh) {
      last_ = pose.stamp;
      writePose(online_, pose.stamp, pose.T_world_lidar);
    }
  }

  void finish() {
    recordMemory(directory_, "online_end");
    if (config_.methodB.enabled) {
      pipeline_.requestPoseGraphSnapshot();
      pipeline_.requestPoseGraphMap();
      pipeline_.finishMapping();
    } else {
      if (!pipeline_.poseGraphSnapshot().stats.keyframes) {
        throw std::runtime_error("no mapping keyframes after replay");
      }
      // A snapshot request can be serviced using a batch that was copied
      // before the last keyframe arrived. Request it after the graph catches up.
      waitFor([&] {
        const auto stats = pipeline_.poseGraphSnapshot().stats;
        return stats.optimized_poses == stats.keyframes;
      });
      pipeline_.requestPoseGraphSnapshot();
      waitFor([&] {
        const auto snapshot = pipeline_.poseGraphSnapshot();
        return snapshot.optimized_poses.size() == snapshot.stats.keyframes;
      });
      recordMemory(directory_, "map_begin");
      pipeline_.requestPoseGraphMap();
      waitFor([&] { return pipeline_.latestPoseGraphMap() != nullptr; });
      recordMemory(directory_, "map_end");
      pipeline_.finishMapping();
    }
    const auto snapshot = pipeline_.poseGraphSnapshot();
    const auto map = pipeline_.latestPoseGraphMap();
    const auto trajectory = pipeline_.mapTrajectory();
    if (!map || trajectory.samples.empty()) throw std::runtime_error("missing benchmark map or trajectory");
    std::ofstream final(directory_ / "mapped.tum");
    final << std::setprecision(17);
    for (const auto& pose : trajectory.samples) writePose(final, pose.stamp, pose.T_map_lidar);
    recordMemory(directory_, "trajectory_end");
    std::ofstream summary(directory_ / "summary.csv");
    summary << "keyframes,optimized,loops,dropped,points,poses\n"
            << snapshot.stats.keyframes << ',' << snapshot.stats.optimized_poses << ',' << snapshot.stats.loop_edges << ','
            << snapshot.stats.dropped_frames << ',' << map->size() << ',' << trajectory.samples.size() << '\n';
    if (!final || !summary) throw std::runtime_error("cannot finish benchmark outputs");
  }

 private:
  void writePose(std::ostream& output, double stamp, const sapphire::Isometry3d& lidar) const {
    const sapphire::Isometry3d body = lidar * config_.extrinsics.T_imu_lidar.inverse() * imu_from_reference_;
    const Eigen::Quaterniond q(body.linear());
    output << stamp << ' ' << body.translation().transpose() << ' ' << q.coeffs().transpose() << '\n';
  }

  sapphire::Config config_;
  std::filesystem::path directory_;
  sapphire::OdometryPipeline pipeline_;
  std::ofstream online_, timings_;
  sapphire::Isometry3d imu_from_reference_ = sapphire::Isometry3d::Identity();
  double last_ = -1.;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 7) throw std::invalid_argument("usage: sapphire_nclt_replay main|methodB profile.toml hits.bin imu.csv output duration_s");
    const std::string_view mode(argv[1]);
    if (mode != "main" && mode != "methodB") throw std::invalid_argument("expected main or methodB");
    const std::filesystem::path output(argv[5]);
    if (!std::filesystem::create_directory(output)) throw std::runtime_error("the output directory must be new");
    std::ofstream(output / "memory.csv") << "phase,rss_kib,peak_kib\n";
    auto config = sapphire::loadConfig(argv[2]);
    if (config.methodB.enabled != (mode == "methodB")) throw std::invalid_argument("replay mode does not match methodB.enabled");
    if (config.methodB.enabled) config.methodB.map_directory = (output / "submaps").string();
    config.output.trajectory_dir.clear();
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("benchmark", std::make_shared<ExportLog>(output)));
    spdlog::set_level(spdlog::level::debug);
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 2);
    Runner runner(config, output);
    replay(runner, argv[3], argv[4], 0., std::stod(argv[6]));
    runner.finish();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
