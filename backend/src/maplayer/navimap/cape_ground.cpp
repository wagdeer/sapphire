#include <cmath>
#include <backend/common/depth_frame.hpp>
#include <backend/maplayer/navimap/flat_ground.hpp>
#include <limits>
#include <memory>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "CAPE.h"
namespace sapphire::backend {
void collectGroundCapePatches(const DepthFrame& f, const StateGroupf& transform, float bodyZ, float tolerance, std::vector<GroundImagePatch>& patches) {
  if (!f.organized() || !transform.isValid() || !std::isfinite(bodyZ) || !std::isfinite(tolerance) || tolerance <= 0)
    throw std::invalid_argument("CAPE ground fitting requires organized depth, valid pose and positive tolerance");
  constexpr std::size_t block = 20;
  // Pad partial cells with invalid depth. Never crop the last rows/columns or
  // change cell size with resolution; neither should alter observation semantics.
  const std::size_t width = ((f.width + block - 1) / block) * block;
  const std::size_t height = ((f.height + block - 1) / block) * block;
  if (width > 65535 || height > 65535 || width * height > std::size_t(std::numeric_limits<int>::max()))
    throw std::invalid_argument("CAPE input dimensions exceed its indexing limits");
  thread_local std::unique_ptr<CAPE> fitter;
  thread_local Eigen::MatrixXf cloud;
  thread_local cv::Mat labels;
  thread_local std::vector<PlaneSeg> planes;
  thread_local std::vector<CylinderSeg> cylinders;
  thread_local std::vector<unsigned char> selected, horizontal;
  if (labels.rows != int(height) || labels.cols != int(width)) {
    fitter = std::make_unique<CAPE>(height, width, block, block, false, .965925826f, 50.f);
    labels.create(height, width, CV_8UC1);
    cloud.resize(width * height, 3);
  }
  cloud.setZero();
  labels.setTo(0);
  planes.clear();
  cylinders.clear();
  for (std::size_t y = 0; y < f.height; ++y)
    for (std::size_t x = 0; x < f.width; ++x) {
      const auto i = y * f.width + x;
      if (!(f.points.z[i] > 0) || !std::isfinite(f.points.z[i]) || !std::isfinite(f.points.x[i]) ||
          !std::isfinite(f.points.y[i]))
        continue;
      const auto j = ((y / block) * (width / block) + x / block) * block * block + (y % block) * block + x % block;
      cloud(j, 0) = 1000 * f.points.x[i];
      cloud(j, 1) = 1000 * f.points.y[i];
      cloud(j, 2) = 1000 * f.points.z[i];
    }
  int nrPlanes = 0, nrCylinders = 0;
  fitter->process(cloud, nrPlanes, nrCylinders, labels, planes, cylinders);
  const Eigen::Matrix3d rotation = transform.rotationMatrix().cast<double>();
  const Eigen::Vector3d translation = transform.translationVector().cast<double>();
  horizontal.assign(planes.size() + 1, 0);
  for (std::size_t i = 0; i < planes.size(); ++i) {
    const auto& p = planes[i];
    const Eigen::Vector3d n(p.normal[0], p.normal[1], p.normal[2]);
    horizontal[i + 1] = std::abs((rotation * n).z()) >= .965925826;
  }
  selected.resize(f.size());
  // Upstream's segmentation image uses 8-bit labels. Ambiguous overflow must
  // not become false ground evidence.
  if (planes.size() > 255) return;
  for (std::size_t y = 0; y < f.height; ++y)
    for (std::size_t x = 0; x < f.width; ++x) {
      const auto label = labels.ptr<unsigned char>(y)[x];
      selected[y * f.width + x] = label < horizontal.size() && horizontal[label];
    }
  for (std::size_t v = 0; v < f.height; v += 8)
    for (std::size_t u = 0; u < f.width; u += 8) {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      Eigen::Matrix3d sq = Eigen::Matrix3d::Zero();
      std::size_t n = 0;
      const auto endV = std::min(v + 8, f.height), endU = std::min(u + 8, f.width);
      for (auto y = v; y < endV; ++y)
        for (auto x = u; x < endU; ++x) {
          const auto i = y * f.width + x;
          if (!selected[i]) continue;
          const Eigen::Vector3d p(f.points.x[i], f.points.y[i], f.points.z[i]);
          if (!p.allFinite()) continue;
          sum += p;
          sq.noalias() += p * p.transpose();
          ++n;
        }
      if (n < 4 || n * 2 < (endV - v) * (endU - u)) continue;
      const Eigen::Vector3d mean = sum / n, world = rotation * mean + translation;
      const Eigen::Matrix3d covariance = rotation * (sq / n - mean * mean.transpose()) * rotation.transpose();
      if (world.z() >= bodyZ || covariance(2, 2) > tolerance * tolerance) continue;
      patches.push_back({world.cast<float>(), covariance.topLeftCorner<2, 2>().cast<float>()});
    }
}
}  // namespace sapphire::backend
