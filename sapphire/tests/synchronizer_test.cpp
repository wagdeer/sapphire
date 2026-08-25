#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "pipeline.hpp"

namespace {

void check(bool condition, const char *message) {
  if (condition) {
    return;
  }
  std::cerr << "check failed: " << message << '\n';
  std::exit(1);
}

ImuMeas imu(double timestamp) {
  ImuMeas value;
  value.timestamp = timestamp;
  value.gyro.setZero();
  value.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
  return value;
}

LidarPoint point(float time_offset) {
  LidarPoint value;
  value.x = 1.0F;
  value.y = 2.0F;
  value.z = 3.0F;
  value.intensity = 4.0F;
  value.time_offset = time_offset;
  return value;
}

}  // namespace

int main() {
  const sapphire::SapphireParameters file_parameters = sapphire::load_parameters(std::string(SAPPHIRE_SOURCE_DIR) + "/config/mid360.toml");
  check(file_parameters.sensor.point_filter_num == 1, "TOML sensor parameters");
  check(!file_parameters.pose_graph.enabled, "TOML pose graph parameters");

  sapphire::SapphireParameters parameters;
  parameters.sensor.blind = 0.2;
  parameters.local_submap.plane_eigen_value_thre = {2.0, 4.0, 5.0, 10.0};
  sapphire::validate_parameters(parameters);
  check(std::abs(parameters.sensor.blind_squared - 0.04) < 1e-12, "blind squared");
  check(std::abs(parameters.odometry.down_size_inv - 10.0) < 1e-12, "down size inverse");
  check(std::abs(parameters.local_submap.plane_eigen_value_thre_inv[1] - 0.25) < 1e-12, "plane threshold inverse");

  sapphire::Synchronizer synchronizer;
  check(synchronizer.push_imu(imu(0.90)), "first imu accepted");
  check(!synchronizer.push_imu(imu(0.90)), "duplicate imu rejected");
  check(!synchronizer.push_imu(imu(0.80)), "reverse imu rejected");
  check(synchronizer.push_imu(imu(0.95)), "second imu accepted");
  check(synchronizer.push_imu(imu(1.00)), "third imu accepted");
  check(synchronizer.push_imu(imu(1.03)), "fourth imu accepted");
  check(synchronizer.push_imu(imu(1.06)), "fifth imu accepted");
  check(synchronizer.push_imu(imu(1.12)), "terminal imu accepted");

  std::vector<LidarPoint> cloud{point(0.12F), point(0.08F), point(0.01F)};
  check(synchronizer.push_lidar(1.0, cloud), "lidar accepted");
  check(!synchronizer.push_lidar(1.0, cloud), "duplicate lidar rejected");
  check(!synchronizer.push_lidar(0.9, cloud), "reverse lidar rejected");

  MeasGroup measures;
  check(synchronizer.sync_packages(measures), "package synchronized");
  check(measures.imu_buf.size() > 4, "enough imu samples");
  check(measures.lidar_cloud->size() == 2, "long point clipped");
  check(measures.lidar_cloud->front().time_offset == 0.01F, "cloud sorted by offset");

  synchronizer.stop_accepting();
  check(!synchronizer.push_imu(imu(2.0)), "imu rejected after stop");
  check(!synchronizer.push_lidar(2.0, cloud), "lidar rejected after stop");
  return 0;
}
