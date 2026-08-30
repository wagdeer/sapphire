#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unsupported/Eigen/Splines>
#include <utility>
#include <vector>

#include "common.hpp"

namespace sapphire {

struct TrajectorySample {
  double timestamp = -1.0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float qx = 0.0F;
  float qy = 0.0F;
  float qz = 0.0F;
  float qw = 1.0F;
  float distance = 0.0F;
};

struct BSplinePath {
  int degree = 0;
  std::vector<float> knots;
  vvec<float, 3> control_points;

  bool valid() const noexcept {
    if (degree < 1 || degree > 3 || control_points.size() < static_cast<std::size_t>(degree + 1) ||
        knots.size() != control_points.size() + static_cast<std::size_t>(degree + 1)) {
      return false;
    }
    if (std::abs(knots.front()) > 1e-6F || std::abs(knots.back() - 1.0F) > 1e-6F) {
      return false;
    }
    for (std::size_t index = 1; index < knots.size(); ++index) {
      if (!std::isfinite(knots[index]) || knots[index] < knots[index - 1]) {
        return false;
      }
    }
    return std::all_of(control_points.begin(), control_points.end(), [](const Eigen::Vector3f &point) { return point.allFinite(); });
  }

  Eigen::Vector3f evaluate(float parameter) const;
  std::pair<Eigen::Vector3f, float> closest_point(const Eigen::Vector3f &point, int coarse_samples = 32, int newton_iterations = 8) const;
  vvec<float, 3> sample_by_arc_length(float spacing, int length_samples = 128) const;
};

struct NavigationPath {
  std::vector<TrajectorySample> samples;
  BSplinePath spline;
};

namespace navigation_detail {

using EigenSpline3d = Eigen::Spline<double, 3, Eigen::Dynamic>;

inline EigenSpline3d make_eigen_spline(const BSplinePath &path) {
  Eigen::RowVectorXd knots(static_cast<Eigen::Index>(path.knots.size()));
  for (std::size_t index = 0; index < path.knots.size(); ++index) {
    knots(static_cast<Eigen::Index>(index)) = path.knots[index];
  }
  Eigen::Matrix<double, 3, Eigen::Dynamic> controls(3, static_cast<Eigen::Index>(path.control_points.size()));
  for (std::size_t index = 0; index < path.control_points.size(); ++index) {
    controls.col(static_cast<Eigen::Index>(index)) = path.control_points[index].cast<double>();
  }
  return {knots, controls};
}

inline vvec<double, 3> downsample_positions(const std::vector<TrajectorySample> &samples, double minimum_distance) {
  vvec<double, 3> positions;
  if (samples.empty()) {
    return positions;
  }
  positions.emplace_back(samples.front().x, samples.front().y, samples.front().z);
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const Eigen::Vector3d point(samples[index].x, samples[index].y, samples[index].z);
    if ((point - positions.back()).norm() > minimum_distance) {
      positions.push_back(point);
    }
  }
  const Eigen::Vector3d end(samples.back().x, samples.back().y, samples.back().z);
  if ((end - positions.back()).norm() > 1e-9) {
    positions.push_back(end);
  }
  return positions;
}

inline vvec<double, 3> smooth_positions(const vvec<double, 3> &input, double lambda) {
  if (input.size() < 3 || lambda <= 0.0) {
    return input;
  }
  const Eigen::Index count = static_cast<Eigen::Index>(input.size());
  Eigen::SparseMatrix<double> system(count, count);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(count) * 10);
  for (Eigen::Index index = 0; index < count; ++index) {
    triplets.emplace_back(index, index, 1.0);
  }
  for (Eigen::Index index = 1; index + 1 < count; ++index) {
    triplets.emplace_back(index - 1, index - 1, lambda);
    triplets.emplace_back(index - 1, index, -2.0 * lambda);
    triplets.emplace_back(index - 1, index + 1, lambda);
    triplets.emplace_back(index, index - 1, -2.0 * lambda);
    triplets.emplace_back(index, index, 4.0 * lambda);
    triplets.emplace_back(index, index + 1, -2.0 * lambda);
    triplets.emplace_back(index + 1, index - 1, lambda);
    triplets.emplace_back(index + 1, index, -2.0 * lambda);
    triplets.emplace_back(index + 1, index + 1, lambda);
  }
  system.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(system);
  if (solver.info() != Eigen::Success) {
    return input;
  }
  vvec<double, 3> output(input.size(), Eigen::Vector3d::Zero());
  for (Eigen::Index dimension = 0; dimension < 3; ++dimension) {
    Eigen::VectorXd values(count);
    for (Eigen::Index index = 0; index < count; ++index) {
      values(index) = input[static_cast<std::size_t>(index)](dimension);
    }
    const Eigen::VectorXd smoothed = solver.solve(values);
    if (solver.info() != Eigen::Success || !smoothed.allFinite()) {
      return input;
    }
    for (Eigen::Index index = 0; index < count; ++index) {
      output[static_cast<std::size_t>(index)](dimension) = smoothed(index);
    }
  }
  return output;
}

}  // namespace navigation_detail

inline BSplinePath fit_navigation_spline(const std::vector<TrajectorySample> &samples, double minimum_distance = 0.5, double smoothing_lambda = 0.1) {
  vvec<double, 3> positions = navigation_detail::downsample_positions(samples, minimum_distance);
  if (positions.size() < 2) {
    return {};
  }
  positions = navigation_detail::smooth_positions(positions, smoothing_lambda);

  const int degree = std::min(3, static_cast<int>(positions.size()) - 1);
  Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, static_cast<Eigen::Index>(positions.size()));
  for (std::size_t index = 0; index < positions.size(); ++index) {
    points.col(static_cast<Eigen::Index>(index)) = positions[index];
  }
  const navigation_detail::EigenSpline3d spline = Eigen::SplineFitting<navigation_detail::EigenSpline3d>::Interpolate(points, degree);

  BSplinePath path;
  path.degree = degree;
  path.knots.reserve(static_cast<std::size_t>(spline.knots().size()));
  for (Eigen::Index index = 0; index < spline.knots().size(); ++index) {
    path.knots.push_back(static_cast<float>(spline.knots()(index)));
  }
  path.control_points.reserve(static_cast<std::size_t>(spline.ctrls().cols()));
  for (Eigen::Index index = 0; index < spline.ctrls().cols(); ++index) {
    path.control_points.push_back(spline.ctrls().col(index).cast<float>());
  }
  return path;
}

inline Eigen::Vector3f BSplinePath::evaluate(float parameter) const {
  if (!valid()) {
    return Eigen::Vector3f::Zero();
  }
  const double clamped = std::clamp(static_cast<double>(parameter), 0.0, 1.0);
  return navigation_detail::make_eigen_spline(*this)(clamped).matrix().cast<float>();
}

inline std::pair<Eigen::Vector3f, float> BSplinePath::closest_point(const Eigen::Vector3f &point, int coarse_samples, int newton_iterations) const {
  if (!valid() || coarse_samples < 1 || newton_iterations < 0) {
    return {Eigen::Vector3f::Zero(), 0.0F};
  }
  const navigation_detail::EigenSpline3d spline = navigation_detail::make_eigen_spline(*this);
  const Eigen::Vector3d query = point.cast<double>();
  double best_parameter = 0.0;
  double best_distance_squared = (spline(0.0).matrix() - query).squaredNorm();
  for (int index = 1; index <= coarse_samples; ++index) {
    const double parameter = static_cast<double>(index) / static_cast<double>(coarse_samples);
    const double distance_squared = (spline(parameter).matrix() - query).squaredNorm();
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_parameter = parameter;
    }
  }

  for (int iteration = 0; iteration < newton_iterations; ++iteration) {
    const Eigen::Matrix<double, 3, Eigen::Dynamic> derivatives = spline.derivatives(best_parameter, 2);
    const Eigen::Vector3d position = derivatives.col(0);
    const Eigen::Vector3d first = derivatives.col(1);
    Eigen::Vector3d second = Eigen::Vector3d::Zero();
    if (degree >= 2) {
      second = derivatives.col(2);
    }
    const Eigen::Vector3d difference = position - query;
    const double gradient = difference.dot(first);
    const double curvature = first.squaredNorm() + difference.dot(second);
    if (std::abs(curvature) < 1e-12) {
      break;
    }
    best_parameter = std::clamp(best_parameter - gradient / curvature, 0.0, 1.0);
  }
  return {spline(best_parameter).matrix().cast<float>(), static_cast<float>(best_parameter)};
}

inline vvec<float, 3> BSplinePath::sample_by_arc_length(float spacing, int length_samples) const {
  vvec<float, 3> output;
  if (!valid() || spacing <= 0.0F || length_samples < 1) {
    return output;
  }
  std::vector<double> lengths(static_cast<std::size_t>(length_samples) + 1, 0.0);
  Eigen::Vector3f previous = evaluate(0.0F);
  for (int index = 1; index <= length_samples; ++index) {
    const Eigen::Vector3f point = evaluate(static_cast<float>(index) / static_cast<float>(length_samples));
    lengths[static_cast<std::size_t>(index)] = lengths[static_cast<std::size_t>(index - 1)] + (point - previous).norm();
    previous = point;
  }

  output.push_back(evaluate(0.0F));
  if (lengths.back() >= spacing) {
    std::size_t segment = 1;
    for (double target = spacing; target < lengths.back(); target += spacing) {
      while (segment < lengths.size() && lengths[segment] < target) {
        ++segment;
      }
      if (segment >= lengths.size()) {
        break;
      }
      const double segment_length = lengths[segment] - lengths[segment - 1];
      const double ratio = segment_length <= 1e-12 ? 0.0 : (target - lengths[segment - 1]) / segment_length;
      const double parameter = (static_cast<double>(segment - 1) + ratio) / static_cast<double>(length_samples);
      output.push_back(evaluate(static_cast<float>(parameter)));
    }
  }
  output.push_back(evaluate(1.0F));
  return output;
}

}  // namespace sapphire
