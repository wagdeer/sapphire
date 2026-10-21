#include "backend/sensor/lidar/input.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "backend/sensor/camera/utils.hpp"

namespace sapphire {
namespace {
std::optional<Eigen::Isometry3d> capturePose(const OdomPoses& poses, double stamp) {
  if (poses.empty() || stamp < poses.front().timestamp || stamp > poses.back().timestamp) return {};
  const auto hi = std::lower_bound(poses.begin(), poses.end(), stamp, [](const auto& p, double t) { return p.timestamp < t; });
  if (hi == poses.begin() || hi->timestamp == stamp) return hi->T_odom_base;
  const auto lo = std::prev(hi);
  const double alpha = (stamp - lo->timestamp) / (hi->timestamp - lo->timestamp);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(lo->T_odom_base.linear()).slerp(alpha, Eigen::Quaterniond(hi->T_odom_base.linear())).toRotationMatrix();
  result.translation() = (1 - alpha) * lo->T_odom_base.translation() + alpha * hi->T_odom_base.translation();
  return result;
}
Eigen::Isometry3d extrinsic(const CameraParameters& c) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    t.translation()[r] = c.camera_to_imu_translation.at(r);
    for (int k = 0; k < 3; ++k) t.linear()(r, k) = c.camera_to_imu_rotation.at(3 * r + k);
  }
  return t;
}
cv::Mat intrinsic(const CameraParameters& c) {
  return (cv::Mat_<double>(3, 3) << c.intrinsics[0], 0, c.intrinsics[2], 0, c.intrinsics[1], c.intrinsics[3], 0, 0, 1);
}
cv::Mat rectify(const VisualFrame& image, const CameraParameters& c, const cv::Mat& rotation, const cv::Mat& projection) {
  auto raw = cv::imdecode(image.image_png(), cv::IMREAD_GRAYSCALE);
  cv::Mat mx, my, out;
  if (c.distortion_model == "equidistant")
    cv::fisheye::initUndistortRectifyMap(intrinsic(c), cv::Mat(c.distortion), rotation, projection, raw.size(), CV_32FC1, mx, my);
  else
    cv::initUndistortRectifyMap(intrinsic(c), cv::Mat(c.distortion), rotation, projection, raw.size(), CV_32FC1, mx, my);
  cv::remap(raw, out, mx, my, cv::INTER_LINEAR);
  return out;
}
backend::camera::Perspective model(const cv::Mat& k, const Eigen::Isometry3d& t, cv::Size size, double baseline = 0) {
  return {k.at<double>(0, 0),
          k.at<double>(1, 1),
          k.at<double>(0, 2),
          k.at<double>(1, 2),
          backend::StateGroupf(Eigen::Matrix4f(t.matrix().cast<float>())),
          size,
          baseline};
}
}  // namespace
backend::SensorData makeBackendInput(const SubmapFrame& submap, const VisualLoopParameters& config) {
  backend::SensorData result;
  result.setStamp(submap.timestamp());
  result.lidar_cloud = submap.cloud();
  if (!config.enabled) return result;
  std::vector<const VisualFrame*> selected(config.cameraCount());
  std::vector<Eigen::Isometry3d> camera_poses(config.cameraCount(), Eigen::Isometry3d::Identity());
  for (const auto& image : submap.visual_frames()) {
    if (image.camera_id() >= selected.size()) continue;
    const auto& c = config.camera(image.camera_id());
    const auto pose = capturePose(submap.odom_poses(), image.timestamp() + c.time_offset);
    if (!pose) continue;  // Never borrow an anchor pose for an unsynchronized image.
    selected[image.camera_id()] = &image;
    camera_poses[image.camera_id()] = submap.T_odom_base().inverse() * (*pose) * extrinsic(c);
  }
  if (config.mode == "stereo" && selected[0] && selected[1]) {
    const auto& l = config.left;
    const auto& r = config.right;
    if (std::abs(selected[0]->timestamp() + l.time_offset - selected[1]->timestamp() - r.time_offset) <= .005 &&
        l.distortion_model == r.distortion_model) {
      const Eigen::Isometry3d right_from_left = camera_poses[1].inverse() * camera_poses[0];
      cv::Mat rotation(3, 3, CV_64F), translation(3, 1, CV_64F), rl, rr, pl, pr, q;
      for (int i = 0; i < 3; ++i) {
        translation.at<double>(i) = right_from_left.translation()[i];
        for (int j = 0; j < 3; ++j) rotation.at<double>(i, j) = right_from_left.linear()(i, j);
      }
      const cv::Size size(l.width, l.height);
      if (l.distortion_model == "equidistant")
        cv::fisheye::stereoRectify(intrinsic(l), cv::Mat(l.distortion), intrinsic(r), cv::Mat(r.distortion), size, rotation, translation, rl, rr, pl,
                                   pr, q, cv::CALIB_ZERO_DISPARITY, size, 0, 1);
      else
        cv::stereoRectify(intrinsic(l), cv::Mat(l.distortion), intrinsic(r), cv::Mat(r.distortion), size, rotation, translation, rl, rr, pl, pr, q,
                          cv::CALIB_ZERO_DISPARITY, 0, size);
      // Native sparse stereo supports horizontal, positive disparity only.
      if (pr.at<double>(0, 3) < 0 && std::abs(pr.at<double>(1, 3)) < 1e-6) {
        Eigen::Isometry3d raw_from_rect = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) raw_from_rect.linear()(i, j) = rl.at<double>(j, i);
        result.setImageData(rectify(*selected[0], l, rl, pl), {}, model(pl, camera_poses[0] * raw_from_rect, size, -pr.at<double>(0, 3)));
        result.set_stereo_right(rectify(*selected[1], r, rr, pr));
        result.visual_camera_ids = {0};
        return result;
      }
    }
  }
  std::vector<cv::Mat> images;
  std::vector<backend::camera::Perspective> cameras;
  for (size_t i = 0; i < selected.size(); ++i)
    if (selected[i]) {
      const auto& c = config.camera(i);
      const auto k = intrinsic(c);
      images.push_back(rectify(*selected[i], c, cv::Mat::eye(3, 3, CV_64F), k));
      cameras.push_back(model(k, camera_poses[i], {c.width, c.height}));
      result.visual_camera_ids.push_back(i);
    }
  if (!images.empty()) result.setImageData(backend::camera::makeCameraMosaic(images), {}, std::move(cameras));
  return result;
}
}  // namespace sapphire
