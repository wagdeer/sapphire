#include <algorithm>
#include <cmath>
#include <memory>

#include "CAPE.h"
#include "backend/grid/flat_ground.hpp"

namespace sapphire::mapping {
void collectGroundCapePatches(const GaussianCloud &points, const Eigen::Isometry3d &world_from_body, float tolerance,
                              std::vector<GroundPatch> &patches) {
  // A bounded XY lattice organizes actual laser centroids for CAPE. Empty cells
  // remain empty; no image, camera calibration, depth input or surface filling.
  constexpr int width = 80, block = 4;
  constexpr double resolution = 1.0, half_extent = width * resolution / 2;
  thread_local CAPE fitter(width, width, block, block, false, .965925826f, 50.f);
  thread_local Eigen::MatrixXf cloud(width * width, 3);
  thread_local cv::Mat labels(width, width, CV_8UC1);
  thread_local std::vector<int> selected(width * width);
  std::fill(selected.begin(), selected.end(), -1);
  cloud.setZero();
  labels.setTo(0);
  const Eigen::Vector3d origin = world_from_body.translation();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto &point = points[i];
    if (!point.valid()) continue;
    const Eigen::Vector3d world = world_from_body * point.mean.cast<double>();
    const Eigen::Vector3d local(world.x() - origin.x(), origin.y() - world.y(), origin.z() - world.z());
    if (local.z() <= .1 || local.z() > 5 || std::abs(local.x()) >= half_extent || std::abs(local.y()) >= half_extent) continue;
    const int x = int((local.x() + half_extent) / resolution), y = int((local.y() + half_extent) / resolution);
    const int cell = y * width + x;
    const int row = ((y / block) * (width / block) + x / block) * block * block + (y % block) * block + x % block;
    // Nearest surface from above: obstacles must not be replaced by floor behind them.
    if (selected[cell] >= 0 && cloud(row, 2) <= 1000 * local.z()) continue;
    selected[cell] = static_cast<int>(i);
    cloud.row(row) = (1000 * local.transpose()).cast<float>();
  }
  std::vector<PlaneSeg> planes;
  std::vector<CylinderSeg> cylinders;
  int plane_count = 0, cylinder_count = 0;
  fitter.process(cloud, plane_count, cylinder_count, labels, planes, cylinders);
  if (planes.size() > 255) return;
  std::vector<bool> horizontal(planes.size() + 1, false);
  for (std::size_t i = 0; i < planes.size(); ++i) horizontal[i + 1] = std::abs(planes[i].normal[2]) >= .965925826;
  for (int y = 0; y < width; ++y)
    for (int x = 0; x < width; ++x) {
      const auto label = labels.at<std::uint8_t>(y, x);
      const int index = selected[y * width + x];
      if (index < 0 || label >= horizontal.size() || !horizontal[label]) continue;
      const auto &point = points[index];
      const Eigen::Vector3d world = world_from_body * point.mean.cast<double>();
      const Eigen::Matrix3d covariance = world_from_body.linear() * point.covariance.cast<double>() * world_from_body.linear().transpose();
      if (covariance(2, 2) > tolerance * tolerance) continue;
      patches.push_back({world.cast<float>(), covariance.topLeftCorner<2, 2>().cast<float>()});
    }
}
}  // namespace sapphire::mapping
