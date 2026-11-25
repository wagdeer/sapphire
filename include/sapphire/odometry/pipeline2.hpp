#pragma once

#include <memory>
#include <optional>
#include <sapphire/odometry/imu_init.hpp>
#include <sapphire/types.hpp>

namespace sapphire {

class PoseGraphBackend;

class Pipeline2 {
 public:
  Pipeline2(const Config& config, PoseGraphBackend& backend);
  ~Pipeline2();

  void pushImu(const ImuData& imu);
  void pushLidar(const PointCloudConstPtr& points);
  bool initialized() const;
  const ImuInitializer::Result& imuInitResult() const;
  std::optional<OdomFrameSample> propagated() const;
  OdomFrameSample latest() const;
  PointCloudConstPtr deskewed(const Isometry3d& T_map_odom) const;
  Eigen::Vector3d accelBias() const;
  Eigen::Vector3d gyroBias() const;
  std::size_t mapRevision() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
