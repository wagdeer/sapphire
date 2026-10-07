#include "backend/visual/solver/visual_pnp.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

namespace sapphire {
namespace {
bool positive(double x) { return std::isfinite(x) && x > 0; }
bool validPose(const Eigen::Isometry3d &pose) {
  return pose.matrix().allFinite() &&
      (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
      std::abs(pose.linear().determinant() - 1) < 1e-6 &&
      (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
}
}  // namespace

VisualPnpResult estimateVisualPnpSeed(const mapping::scene::FeatureMap &target,
    const features::FeatureBlock &query, const std::vector<mapping::scene::Match> &matches,
    const CameraParameters &camera, const Eigen::Isometry3d &T_query_camera,
    const VisualPnpParameters &p) {
  if (camera.width <= 0 || camera.height <= 0 || camera.width > 4096 || camera.height > 4096 ||
      camera.intrinsics.size() != 4 || !positive(camera.intrinsics[0]) || !positive(camera.intrinsics[1]) ||
      !std::all_of(camera.intrinsics.begin(), camera.intrinsics.end(), [](double x) { return std::isfinite(x); }) ||
      !validPose(T_query_camera) || p.max_pairs < 6 || p.max_pairs > features::FeatureBlock::max_features ||
      p.ransac_iterations < 1 || p.ransac_iterations > 500 || p.refinement_iterations < 1 || p.refinement_iterations > 30 ||
      p.min_inliers < 6 || std::size_t(p.min_inliers) > p.max_pairs ||
      !positive(p.min_inlier_fraction) || p.min_inlier_fraction > 1 ||
      !positive(p.max_reprojection_px) || p.max_reprojection_px > 10 ||
      p.min_occupied_cells < 1 || p.min_occupied_cells > 12 ||
      !positive(p.min_second_axis_ratio) || p.min_second_axis_ratio > 1)
    throw std::invalid_argument("Invalid visual PnP calibration, pose or budget");
  VisualPnpResult result;
  if (matches.size() > p.max_pairs) { result.status = VisualPnpStatus::pair_budget; return result; }
  query.validate();
  std::vector<bool> used_target(target.points().size()), used_query(query.keypoints.size());
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> pixels;
  std::vector<mapping::scene::Match> metric_matches;
  for (const auto &m : matches) {
    if (m.landmark >= target.points().size() || m.feature >= query.keypoints.size() ||
        m.appearance >= target.points()[m.landmark].appearance_count || m.distance < 0 || m.distance > 256 ||
        used_target[m.landmark] || used_query[m.feature])
      throw std::invalid_argument("Visual PnP requires valid one-to-one exact-snapshot matches");
    used_target[m.landmark] = used_query[m.feature] = true;
    const auto &landmark = target.points()[m.landmark];
    const auto &pixel = query.keypoints[m.feature].pt;
    if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) throw std::invalid_argument("Nonfinite visual PnP pixel");
    if (!landmark.has_position) continue;
    const auto &point = landmark.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::invalid_argument("Nonfinite visual PnP geometry");
    // Rectification can move observed pixels outside the calibrated rectangle.
    // They cannot supply this utility's image-coverage evidence.
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= camera.width || pixel.y >= camera.height) continue;
    object.emplace_back(point.x, point.y, point.z);
    pixels.emplace_back(pixel.x, pixel.y);
    metric_matches.push_back(m);
  }
  result.metric_pairs = object.size();
  if (object.size() < std::size_t(p.min_inliers)) return result;
  const cv::Mat K = (cv::Mat_<double>(3, 3) << camera.intrinsics[0], 0, camera.intrinsics[2],
      0, camera.intrinsics[1], camera.intrinsics[3], 0, 0, 1);
  cv::Mat rvec, tvec, consensus;
  result.status = VisualPnpStatus::no_consensus;
  if (!cv::solvePnPRansac(object, pixels, K, cv::noArray(), rvec, tvec, false,
          p.ransac_iterations, float(p.max_reprojection_px), .999, consensus, cv::SOLVEPNP_EPNP) ||
      consensus.total() < std::size_t(p.min_inliers) ||
      double(consensus.total()) / object.size() < p.min_inlier_fraction) return result;
  std::vector<cv::Point3d> refine_object;
  std::vector<cv::Point2d> refine_pixels;
  for (int i = 0; i < consensus.rows; ++i) {
    const auto row = std::size_t(consensus.at<int>(i));
    refine_object.push_back(object.at(row)); refine_pixels.push_back(pixels.at(row));
  }
  cv::solvePnPRefineLM(refine_object, refine_pixels, K, cv::noArray(), rvec, tvec,
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, p.refinement_iterations, 1e-6));
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Isometry3d T_camera_target = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    T_camera_target.translation()[r] = tvec.at<double>(r);
    for (int c = 0; c < 3; ++c) T_camera_target.linear()(r, c) = R.at<double>(r, c);
  }
  result.status = VisualPnpStatus::insufficient_support;
  if (!validPose(T_camera_target)) return result;
  std::array<bool, 12> occupied{};
  std::vector<Eigen::Vector3d> inlier_points;
  std::vector<mapping::scene::Match> inliers;
  double squared_error = 0;
  for (std::size_t i = 0; i < object.size(); ++i) {
    const Eigen::Vector3d point(object[i].x, object[i].y, object[i].z);
    const Eigen::Vector3d pc = T_camera_target * point;
    if (!pc.allFinite() || pc.z() <= 1e-6) continue;
    const double dx = camera.intrinsics[0] * pc.x() / pc.z() + camera.intrinsics[2] - pixels[i].x;
    const double dy = camera.intrinsics[1] * pc.y() / pc.z() + camera.intrinsics[3] - pixels[i].y;
    const double error = dx*dx + dy*dy;
    if (!std::isfinite(error) || error > p.max_reprojection_px*p.max_reprojection_px) continue;
    squared_error += error;
    inliers.push_back(metric_matches[i]); inlier_points.push_back(point);
    const int x = std::min(3, int(pixels[i].x * 4 / camera.width));
    const int y = std::min(2, int(pixels[i].y * 3 / camera.height));
    occupied[y*4+x] = true;
  }
  result.occupied_cells = int(std::count(occupied.begin(), occupied.end(), true));
  if (inliers.size() < std::size_t(p.min_inliers) || double(inliers.size()) / object.size() < p.min_inlier_fraction ||
      result.occupied_cells < p.min_occupied_cells) return result;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto &point : inlier_points) center += point;
  center /= inlier_points.size();
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const auto &point : inlier_points) scatter.noalias() += (point - center) * (point - center).transpose();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(scatter);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() || eigen.eigenvalues()[2] <= 1e-12 ||
      eigen.eigenvalues()[1] < p.min_second_axis_ratio * eigen.eigenvalues()[2]) return result;
  const Eigen::Isometry3d seed = T_camera_target.inverse() * T_query_camera.inverse();
  if (!validPose(seed)) return result;
  result.status = VisualPnpStatus::seed;
  result.T_target_query = seed;
  result.rms_reprojection_px = std::sqrt(squared_error / inliers.size());
  result.inliers = std::move(inliers);
  return result;
}
}  // namespace sapphire
