#include "backend/common/depth_frame.hpp"

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "backend/tools/parallel_executor.hpp"

namespace sapphire::backend {
namespace {
struct DepthProjection {
  const void* data;
  std::size_t stride_bytes, width, height;
  int decimation;
  bool millimeters;
  float cx, cy, inverse_fx, inverse_fy, minimum, maximum;
};

template <class Sample, bool Contiguous, bool Bounded, class Kernel>
void project(const DepthProjection& p, float* x, float* y, float* z, Kernel, std::size_t begin, std::size_t end) {
  using Packet = typename Kernel::template Packet<float>;
  constexpr std::size_t width = Packet::width;
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  const Packet cx(p.cx), inverse_fx(p.inverse_fx);
  const Packet zero(0.f), minimum(p.minimum), maximum(Bounded ? p.maximum : std::numeric_limits<float>::max()), invalids(invalid);
  const Packet offsets = Packet::iota();
  for (std::size_t row = begin; row < end; ++row) {
    const auto* input = reinterpret_cast<const Sample*>(static_cast<const std::uint8_t*>(p.data) + row * p.decimation * p.stride_bytes);
    const auto sample = [&](std::size_t column) {
      const Sample raw = input[Contiguous ? column : column * p.decimation];
      float depth;
      if constexpr (std::is_same_v<Sample, std::uint16_t>)
        depth = raw != 65535 ? float(raw) * .001f : 0.f;
      else
        depth = raw;
      const bool valid = std::isfinite(depth) && depth > 0.f && depth >= p.minimum && (!Bounded || depth <= p.maximum);
      return valid ? depth : invalid;
    };
    const float normalized_y = (float(row) - p.cy) * p.inverse_fy;
    const Packet py(normalized_y);
    std::size_t column = 0;
    for (; column + width <= p.width; column += width) {
      const Packet raw_depth = [&] {
        if constexpr (Contiguous && std::is_same_v<Sample, float>) {
          return Packet::load(input + column);
        } else {
          alignas(32) float depths[width];
          for (std::size_t lane = 0; lane < width; ++lane) {
            const auto raw = input[Contiguous ? column + lane : (column + lane) * p.decimation];
            if constexpr (std::is_same_v<Sample, std::uint16_t>)
              depths[lane] = float(raw) * .001f;
            else
              depths[lane] = raw;
          }
          return Packet::load(depths);
        }
      }();
      // Ordered comparisons reject NaNs and infinities without per-lane
      // branches. Keep the uint16 sentinel out before emitting optical Z.
      auto valid = (raw_depth > zero) & (raw_depth >= minimum) & (raw_depth <= maximum);
      if constexpr (std::is_same_v<Sample, std::uint16_t>) valid = valid & (raw_depth < Packet(65535.f * .001f));
      const Packet depth = select(valid, raw_depth, invalids);
      const Packet px = ((Packet(float(column)) + offsets) - cx) * inverse_fx;
      (px * depth).store(x + row * p.width + column);
      (py * depth).store(y + row * p.width + column);
      depth.store(z + row * p.width + column);
    }
    for (; column < p.width; ++column) {
      const float depth = sample(column);
      const std::size_t i = row * p.width + column;
      x[i] = ((float(column) - p.cx) * p.inverse_fx) * depth;
      y[i] = normalized_y * depth;
      z[i] = depth;
    }
  }
}

template <class Sample, class Kernel>
void dispatch(const DepthProjection& p, float* x, float* y, float* z, Kernel kernel, std::size_t begin, std::size_t end) {
  if (p.decimation == 1) {
    if (p.maximum > 0)
      project<Sample, true, true>(p, x, y, z, kernel, begin, end);
    else
      project<Sample, true, false>(p, x, y, z, kernel, begin, end);
  } else {
    if (p.maximum > 0)
      project<Sample, false, true>(p, x, y, z, kernel, begin, end);
    else
      project<Sample, false, false>(p, x, y, z, kernel, begin, end);
  }
}
template <class Kernel>
void project_depth(const DepthProjection& p, float* x, float* y, float* z, Kernel kernel, std::size_t begin, std::size_t end) {
  if (p.millimeters)
    dispatch<std::uint16_t>(p, x, y, z, kernel, begin, end);
  else
    dispatch<float>(p, x, y, z, kernel, begin, end);
}

template <class Execute>
DepthFrame from_depth(const cv::Mat& depth, const camera::Perspective& inputPerspective, const DepthProjectionOptions& options, Execute execute) {
  if (depth.empty() || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
    throw std::invalid_argument("DepthFrame requires CV_16UC1 or CV_32FC1 depth");
  }
  if (!inputPerspective.isValidForProjection() || inputPerspective.cameraToBase().isNull() || inputPerspective.imageSize() != depth.size() ||
      options.decimation <= 0 || depth.cols % options.decimation != 0 || depth.rows % options.decimation != 0 || !std::isfinite(options.minDepth) ||
      !std::isfinite(options.maxDepth) || options.minDepth < 0.0f || (options.maxDepth > 0.0f && options.maxDepth < options.minDepth)) {
    throw std::invalid_argument("Invalid DepthFrame projection parameters");
  }

  DepthFrame output;
  output.width = static_cast<std::size_t>(depth.cols / options.decimation);
  output.height = static_cast<std::size_t>(depth.rows / options.decimation);
  const float inverseDecimation = 1.0f / static_cast<float>(options.decimation);
  output.perspective = inputPerspective.scaled(inverseDecimation);
  output.localTransform = inputPerspective.cameraToBase();
  output.points.resize(output.width * output.height);

  const DepthProjection projection{depth.data,
                                   depth.step,
                                   output.width,
                                   output.height,
                                   options.decimation,
                                   depth.type() == CV_16UC1,
                                   float(output.perspective.cx()),
                                   float(output.perspective.cy()),
                                   float(output.perspective.fxInv()),
                                   float(output.perspective.fyInv()),
                                   options.minDepth,
                                   options.maxDepth};
  execute(output.height, [&](auto kernel, std::size_t, std::size_t begin, std::size_t end) {
    project_depth(projection, output.points.x.data(), output.points.y.data(), output.points.z.data(), kernel, begin, end);
  });
  return output;
}
}  // namespace

DepthFrame DepthFrame::fromDepth(const cv::Mat& depth, const camera::Perspective& perspective, const DepthProjectionOptions& options) {
  return from_depth(depth, perspective, options, [](std::size_t rows, auto task) { task(simd::Kernel<>{}, 0, 0, rows); });
}

template <class ISA_TYPE>
DepthFrame DepthFrame::fromDepth(const cv::Mat& depth, const camera::Perspective& perspective, const DepthProjectionOptions& options,
                                 tools::ParallelExecutor<ISA_TYPE>& executor) {
  return from_depth(depth, perspective, options, [&](std::size_t rows, auto task) { executor.parallel_for(rows, task); });
}
template DepthFrame DepthFrame::fromDepth<simd::Native>(const cv::Mat&, const camera::Perspective&, const DepthProjectionOptions&,
                                                        tools::ParallelExecutor<simd::Native>&);
template DepthFrame DepthFrame::fromDepth<simd::Scalar>(const cv::Mat&, const camera::Perspective&, const DepthProjectionOptions&,
                                                        tools::ParallelExecutor<simd::Scalar>&);

bool DepthFrame::organized() const noexcept { return width > 0U && height > 0U && width * height == size(); }

std::size_t DepthFrame::pixelIndex(std::size_t column, std::size_t row) const {
  if (column >= width || row >= height) {
    throw std::out_of_range("DepthFrame pixel is outside the organized dimensions");
  }
  return row * width + column;
}

cv::Point3f DepthFrame::viewPoint() const noexcept {
  const Eigen::Vector3f translation = localTransform.translationVector();
  return cv::Point3f(translation.x(), translation.y(), translation.z());
}

}  // namespace sapphire::backend
