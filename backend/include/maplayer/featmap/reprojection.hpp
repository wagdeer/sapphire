#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <backend/sensor/camera/perspective.hpp>

namespace sapphire::backend::featmap::geometry {
struct CameraPose {
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  explicit CameraPose(const StateGroupf& pose)
      : rotation(Eigen::Quaterniond(pose.rotationMatrix().cast<double>()).normalized().toRotationMatrix()),
        translation(pose.translationVector().cast<double>()) {}
  CameraPose(const CameraPose& body, const CameraPose& sensor)
      : rotation(body.rotation * sensor.rotation), translation(body.translation + body.rotation * sensor.translation) {}
};
inline bool inlier(const camera::StereoObservation& o, const CameraPose& camera, const Eigen::Vector3d& point) {
  const double error = camera::Perspective::squaredPixelError(camera.rotation, camera.translation, point, o.calibration(), {o.pixel().x, o.pixel().y, o.pixel().z});
  return std::isfinite(error) && error <= (o.pixel().y < 0 ? 5.99146 : 7.81473) * o.sigma() * o.sigma();
}
inline bool inlier(const camera::StereoObservation& o, const StateGroupf& boundary, const Eigen::Vector3d& point) {
  return inlier(o, CameraPose(CameraPose(boundary), CameraPose(o.anchor_from_camera())), point);
}
// One pass over immutable observations: each camera pose is calculated once.
// Value entries replace per-camera shared_ptr/calibration allocations.
struct ObservationProjection {
  struct Sensor {
    const camera::StereoObservation* source;
    CameraPose extrinsic;
    std::map<const StateGroupf*, CameraPose> fixed;
  };
  std::vector<Sensor> sensors;
  bool inlier(const camera::StereoObservation& o, const StateGroupf& boundary, const Eigen::Vector3d& point) {
    if (!point.allFinite()) return false;
    Sensor* sensor = nullptr;
    for (auto& s : sensors)
      if (s.source->calibration() == o.calibration() &&
          (s.source->anchor_from_camera().rotationMatrix().array() == o.anchor_from_camera().rotationMatrix().array()).all() &&
          (s.source->anchor_from_camera().translationVector().array() == o.anchor_from_camera().translationVector().array()).all()) {
        sensor = &s;
        break;
      }
    if (!sensor) {
      sensors.push_back({&o, CameraPose(o.anchor_from_camera()), {}});
      sensor = &sensors.back();
    }
    auto it = sensor->fixed.find(&boundary);
    if (it == sensor->fixed.end()) it = sensor->fixed.emplace(&boundary, CameraPose(CameraPose(boundary), sensor->extrinsic)).first;
    return geometry::inlier(o, it->second, point);
  }
};
}
