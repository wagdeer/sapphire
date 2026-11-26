#include <chrono>
#include <filesystem>
#include <sapphire/odometry/pipeline.hpp>
#include <stdexcept>

namespace {

struct TemporaryMap {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
                               ("sapphire-pipeline2-imu-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ~TemporaryMap() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

sapphire::PointCloudConstPtr scan(double end) {
  auto points = std::make_shared<sapphire::PointCloud>();
  for (int axis = 0; axis < 3; ++axis) {
    for (int i = 0; i < 8; ++i) {
      for (int j = 0; j < 8; ++j) {
        sapphire::Point p{};
        auto xyz = p.getVector3fMap();
        xyz[axis] = 4.f;
        xyz[(axis + 1) % 3] = i * .5f;
        xyz[(axis + 2) % 3] = j * .5f;
        p.data[3] = 1.f;
        p.timestamp = end - .001 * (7 - j);
        points->push_back(p);
      }
    }
  }
  return points;
}

}  // namespace

int main() {
  TemporaryMap map;
  sapphire::Config config;
  config.pgo.enabled = true;
  config.methodB.enabled = true;
  config.methodB.map_directory = map.path.string();
  config.imu.init.gravity_mag = 9.8;
  sapphire::OdometryPipeline pipeline(config);
  std::size_t tracked = 0;
  int imu_index = 0;
  double last_imu = 0.;
  for (int i = 0; i < 24; ++i) {
    const double end = .095 + .1 * i;
    while (last_imu < end) {
      last_imu = .003 + .01 * imu_index++;
      pipeline.pushImu({last_imu, {0., 0., 9.8}, Eigen::Vector3d::Zero()});
    }
    pipeline.pushLidar(end - .007, scan(end));
    const auto pose = pipeline.latestResult();
    if (!pose.converged) continue;
    ++tracked;
    if (pose.stamp != end || !pose.T_world_lidar.matrix().allFinite()) {
      throw std::runtime_error("LiDAR state must be at the scan end");
    }
    const auto propagated = pipeline.latestPropagatedResult();
    if (!propagated || propagated->stamp != last_imu) {
      throw std::runtime_error("the raw IMU sample after the scan must still reach the IMU-rate state");
    }
  }
  if (tracked < 10) throw std::runtime_error("pipeline2 failed to start tracking");
  pipeline.finishMapping();
}
