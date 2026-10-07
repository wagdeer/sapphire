#pragma once

#include <variant>
#include "common/camera/kb4.hpp"
#include "common/camera/pinhole.hpp"

namespace sapphire {
// Runtime selection for configured optical streams. Each concrete model owns its
// calibration and geometry; UCM remains an explicit, separately included model.
class CameraModel final {
 public:
  explicit CameraModel(const CameraParameters &calibration);
  const cv::Mat &matrix() const {
    return std::visit([](const auto &model) -> const cv::Mat & { return model.matrix(); }, model_);
  }
  const std::vector<float> &distortion() const {
    return std::visit([](const auto &model) -> const std::vector<float> & { return model.distortion(); }, model_);
  }
  std::optional<Eigen::Vector2f> project(const Eigen::Vector3f &point_camera) const {
    return std::visit([&](const auto &model) { return model.project(point_camera); }, model_);
  }
  std::optional<Eigen::Vector3f> unproject(cv::Point2f raw) const {
    return std::visit([&](const auto &model) { return model.unproject(raw); }, model_);
  }
  std::optional<Eigen::Vector2f> bearingToRectified(const Eigen::Vector3f &point_camera) const {
    return std::visit([&](const auto &model) { return model.bearingToRectified(point_camera); }, model_);
  }
  cv::Point2f distortNormalized(cv::Point2f ray) const {
    return std::visit([&](const auto &model) { return model.distortNormalized(ray); }, model_);
  }
  void undistortPixels(const std::vector<cv::Point2f> &raw,
                       std::vector<cv::Point2f> &out, bool rectified = false) const {
    std::visit([&](const auto &model) { model.undistortPixels(raw, out, rectified); }, model_);
  }
  void undistortImage(const cv::Mat &raw, cv::Mat &rectified) const {
    std::visit([&](const auto &model) { model.undistortImage(raw, rectified); }, model_);
  }
  Eigen::Isometry3f cameraToImu() const {
    return std::visit([](const auto &model) { return model.cameraToImu(); }, model_);
  }
 private:
  std::variant<PinholeCameraModel, Kb4CameraModel> model_;
};
}  // namespace sapphire
