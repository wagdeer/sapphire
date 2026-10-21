#pragma once

#include <cmath>
#include <backend/common/common_type.hpp>
#include <backend/tools/simd_sse_neon.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <stdexcept>

namespace sapphire::backend::feature {

// Dispatch once at module entry, never inside the pairwise distance loop.
template <class Fn>
decltype(auto) dispatch_descriptor(int dimensions, Fn&& fn) {
  switch (dimensions) {
    case 32: return fn(Binary_U8{});
    case 64: return fn(Float64_F32{});
    case 256: return fn(Float_F32{});
    default: throw std::invalid_argument("Unsupported descriptor dimensions");
  }
}

template <class Format>
class descriptor_row {
 public:
  using scalar = typename descriptor_traits<Format>::scalar;
  explicit descriptor_row(const scalar* values) noexcept : values_(values) {}
  const scalar* data() const noexcept { return values_; }

 private:
  const scalar* values_;
};

// Validate once when entering a typed module, not for each descriptor pair.
// Borrow arbitrary OpenCV row strides; do not repack or convert the matrix.
template <class Format>
class descriptor_matrix {
 public:
  explicit descriptor_matrix(const cv::Mat& values) : values_(values) {
    if (!values.empty() && (values.type() != descriptor_traits<Format>::cv_type || values.cols != descriptor_traits<Format>::dimensions))
      throw std::invalid_argument("Descriptor matrix does not match compiled format");
  }
  descriptor_row<Format> row(int index) const noexcept {
    return descriptor_row<Format>(reinterpret_cast<const typename descriptor_traits<Format>::scalar*>(values_.data + size_t(index) * values_.step[0]));
  }
  int rows() const noexcept { return values_.rows; }

 private:
  const cv::Mat& values_;
};

template <class Format>
class descriptor_distance;
template <>
class descriptor_distance<Binary_U8> {
 public:
  explicit descriptor_distance(descriptor_row<Binary_U8> query) noexcept : query_(query.data()) {}
  int distance(descriptor_row<Binary_U8> candidate) const noexcept { return query_(candidate.data()); }
  int score(descriptor_row<Binary_U8> candidate) const noexcept { return distance(candidate); }
  static float distance_from_score(float score) noexcept { return score; }

 private:
  simd::Hamming256<simd::Native> query_;
};

template <class Format>
class descriptor_distance {
 public:
  static_assert(descriptor_traits<Format>::metric == DistanceMetric::l2);
  using scalar = typename descriptor_traits<Format>::scalar;
  explicit descriptor_distance(descriptor_row<Format> query) noexcept : query_(query.data()) {}
  float squared_distance(descriptor_row<Format> candidate) const noexcept {
    if constexpr (std::is_same_v<scalar, float>) {
      return cv::hal::normL2Sqr_(query_, candidate.data(), descriptor_traits<Format>::dimensions);
    } else {
      float sum = 0;
      for (int i = 0; i < descriptor_traits<Format>::dimensions; ++i) {
        const float delta = float(query_[i]) - float(candidate.data()[i]);
        sum += delta * delta;
      }
      return sum;
    }
  }
  // Search orders squared scores; convert only the final neighbors to L2.
  float score(descriptor_row<Format> candidate) const noexcept { return squared_distance(candidate); }
  static float distance_from_score(float score) noexcept { return std::sqrt(score); }
  float distance(descriptor_row<Format> candidate) const noexcept { return distance_from_score(score(candidate)); }
 private:
  const scalar* query_;
};

static_assert(sizeof(descriptor_storage<Binary_U8>) == 32);
static_assert(sizeof(descriptor_storage<Float_F16>) == 512);
static_assert(sizeof(descriptor_storage<Float_F32>) == 1024);
static_assert(sizeof(descriptor_row<Float_F32>) == sizeof(const float*));
static_assert(!std::is_constructible_v<descriptor_distance<Binary_U8>, descriptor_row<Float_F32>>);
static_assert(!std::is_constructible_v<descriptor_distance<Float_F16>, descriptor_row<Float_F32>>);

}  // namespace sapphire::backend::feature
