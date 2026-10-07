#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

#include "common/common.hpp"

namespace sapphire {

struct AABB {
  AABB() = default;

  AABB(const Eigen::Vector3f &minimum, const Eigen::Vector3f &maximum) : minimum(minimum), maximum(maximum) {}

  explicit AABB(const vvec<float, 3> &cloud) {
    for (const Eigen::Vector3f &point : cloud) {
      include(point);
    }
  }

  explicit AABB(const GaussianCloud &cloud) {
    for (const GaussianPoint &point : cloud) {
      if (!point.mean.allFinite() || !std::isfinite(point.radius) || point.radius < 0.0) {
        continue;
      }
      const Eigen::Vector3f mean = point.mean.cast<float>();
      const Eigen::Vector3f margin = Eigen::Vector3f::Constant(static_cast<float>(point.radius));
      include(mean - margin);
      include(mean + margin);
    }
  }

  Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
  Eigen::Vector3f maximum = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());

  bool valid() const noexcept { return minimum.allFinite() && maximum.allFinite() && (minimum.array() <= maximum.array()).all(); }

  Eigen::Vector3f center() const noexcept { return 0.5F * (minimum + maximum); }

  Eigen::Vector3f extent() const noexcept { return maximum - minimum; }

  AABB intersection(const AABB &other) const noexcept {
    if (!valid() || !other.valid()) {
      return {};
    }
    const Eigen::Vector3f overlap_minimum = minimum.cwiseMax(other.minimum);
    const Eigen::Vector3f overlap_maximum = maximum.cwiseMin(other.maximum);
    if ((overlap_minimum.array() > overlap_maximum.array()).any()) {
      return {};
    }
    return {overlap_minimum, overlap_maximum};
  }

  float xyArea() const noexcept {
    if (!valid()) {
      return 0.0F;
    }
    const Eigen::Vector2f xy_extent = extent().head<2>();
    const float area = xy_extent.x() * xy_extent.y();
    return std::isfinite(area) && area > 0.0F ? area : 0.0F;
  }

  /// Fraction of this box's XY area covered by another box.
  float xyOverlapRatio(const AABB &other) const noexcept {
    const float area = xyArea();
    if (area == 0.0F || !other.valid()) {
      return 0.0F;
    }
    const Eigen::Vector2f overlap_minimum = minimum.head<2>().cwiseMax(other.minimum.head<2>());
    const Eigen::Vector2f overlap_maximum = maximum.head<2>().cwiseMin(other.maximum.head<2>());
    const Eigen::Vector2f overlap_extent = (overlap_maximum - overlap_minimum).cwiseMax(Eigen::Vector2f::Zero());
    return overlap_extent.x() * overlap_extent.y() / area;
  }

  void include(const Eigen::Vector3f &point) {
    if (!point.allFinite()) {
      return;
    }
    minimum = minimum.cwiseMin(point);
    maximum = maximum.cwiseMax(point);
  }

  AABB expanded(const Eigen::Vector3f &margin) const {
    if (!valid() || !margin.allFinite() || (margin.array() < 0.0F).any()) {
      return {};
    }
    return {minimum - margin, maximum + margin};
  }

  AABB transform(const Eigen::Isometry3f &pose) const {
    if (!valid() || !pose.matrix().allFinite()) {
      return {};
    }
    const Eigen::Vector3f transformed_center = pose * center();
    const Eigen::Vector3f transformed_half_extent = pose.linear().cwiseAbs() * (0.5F * extent());
    return {transformed_center - transformed_half_extent, transformed_center + transformed_half_extent};
  }
};

}  // namespace sapphire
