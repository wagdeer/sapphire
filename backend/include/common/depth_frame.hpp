#pragma once

#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <type_traits>

#include "backend/sensor/camera/perspective.hpp"
#include "backend/tools/simd.hpp"

namespace sapphire::backend {

namespace tools {
template <class ISA_TYPE>
class ParallelExecutor;
}

struct DepthProjectionOptions {
  int decimation = 1;
  float minDepth = 0.0f;
  float maxDepth = 0.0f;
};

struct DepthFrame {
  DepthFrame() = default;
  DepthFrame(const DepthFrame&) = delete;
  DepthFrame& operator=(const DepthFrame&) = delete;
  DepthFrame(DepthFrame&&) noexcept = default;
  DepthFrame& operator=(DepthFrame&&) noexcept = default;

  simd::SoA3<float> points;
  std::size_t width = 0;
  std::size_t height = 0;
  camera::Perspective perspective;
  StateGroupf localTransform = StateGroupf::identity();

  static DepthFrame fromDepth(const cv::Mat& depth, const camera::Perspective& perspective, const DepthProjectionOptions& options);

  template <class ISA_TYPE>
  static DepthFrame fromDepth(const cv::Mat& depth, const camera::Perspective& perspective, const DepthProjectionOptions& options,
                              tools::ParallelExecutor<ISA_TYPE>& executor);

  std::size_t size() const noexcept { return points.size(); }
  bool organized() const noexcept;
  std::size_t pixelIndex(std::size_t column, std::size_t row) const;
  cv::Point3f viewPoint() const noexcept;
};

static_assert(!std::is_polymorphic_v<DepthFrame>);

}  // namespace sapphire::backend
