#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <opencv2/core/mat.hpp>

#include "backend/tools/simd_sse_neon.hpp"

namespace sapphire::backend::feature::gaussian_detail {

#if defined(SIMD_NEON)

inline constexpr std::array<float, 7> kernel{{
    0x1.1f5f62p-4F,
    0x1.0c70fcp-3F,
    0x1.869472p-3F,
    0x1.ba95cp-3F,
    0x1.869472p-3F,
    0x1.0c70fcp-3F,
    0x1.1f5f62p-4F,
}};

struct FloatBlock16 {
  float32x4_t values_0;
  float32x4_t values_1;
  float32x4_t values_2;
  float32x4_t values_3;
};

inline FloatBlock16 convert_u8x16_to_f32(const uint8x16_t values) {
  const uint16x8_t values_low = vmovl_u8(vget_low_u8(values));
  const uint16x8_t values_high = vmovl_u8(vget_high_u8(values));
  return {
      vcvtq_f32_u32(vmovl_u16(vget_low_u16(values_low))),
      vcvtq_f32_u32(vmovl_u16(vget_high_u16(values_low))),
      vcvtq_f32_u32(vmovl_u16(vget_low_u16(values_high))),
      vcvtq_f32_u32(vmovl_u16(vget_high_u16(values_high))),
  };
}

inline FloatBlock16 load_symmetric_pair_u8x16_as_f32(const std::uint8_t* center, const int distance) {
  const uint8x16_t left = vld1q_u8(center - distance);
  const uint8x16_t right = vld1q_u8(center + distance);
  const uint16x8_t pair_low = vaddl_u8(vget_low_u8(left), vget_low_u8(right));
  const uint16x8_t pair_high = vaddl_u8(vget_high_u8(left), vget_high_u8(right));
  return {
      vcvtq_f32_u32(vmovl_u16(vget_low_u16(pair_low))),
      vcvtq_f32_u32(vmovl_u16(vget_high_u16(pair_low))),
      vcvtq_f32_u32(vmovl_u16(vget_low_u16(pair_high))),
      vcvtq_f32_u32(vmovl_u16(vget_high_u16(pair_high))),
  };
}

inline void accumulate_symmetric_pair(const std::uint8_t* center, const int distance, const float coefficient, FloatBlock16& sums) {
  const FloatBlock16 pair = load_symmetric_pair_u8x16_as_f32(center, distance);
  sums.values_0 = vfmaq_n_f32(sums.values_0, pair.values_0, coefficient);
  sums.values_1 = vfmaq_n_f32(sums.values_1, pair.values_1, coefficient);
  sums.values_2 = vfmaq_n_f32(sums.values_2, pair.values_2, coefficient);
  sums.values_3 = vfmaq_n_f32(sums.values_3, pair.values_3, coefficient);
}

inline void filter_horizontal_16(const std::uint8_t* input, float* output, const int x) {
  const std::uint8_t* center = input + x;
  const FloatBlock16 center_values = convert_u8x16_to_f32(vld1q_u8(center));
  FloatBlock16 sums{
      vmulq_n_f32(center_values.values_0, kernel[3]),
      vmulq_n_f32(center_values.values_1, kernel[3]),
      vmulq_n_f32(center_values.values_2, kernel[3]),
      vmulq_n_f32(center_values.values_3, kernel[3]),
  };
  accumulate_symmetric_pair(center, 1, kernel[4], sums);
  accumulate_symmetric_pair(center, 2, kernel[5], sums);
  accumulate_symmetric_pair(center, 3, kernel[6], sums);
  vst1q_f32(output + x, sums.values_0);
  vst1q_f32(output + x + 4, sums.values_1);
  vst1q_f32(output + x + 8, sums.values_2);
  vst1q_f32(output + x + 12, sums.values_3);
}

inline void filter_horizontal_row(const std::uint8_t* input, float* output, const int width) {
  int x = 0;
  for (; x + 16 <= width; x += 16) {
    filter_horizontal_16(input, output, x);
  }
  for (; x < width; ++x) {
    float sum = kernel[3] * input[x];
    for (int distance = 1; distance <= 3; ++distance) {
      sum += kernel[static_cast<std::size_t>(3 + distance)] * (static_cast<float>(input[x - distance]) + static_cast<float>(input[x + distance]));
    }
    output[x] = sum;
  }
}

inline float32x4_t filter_vertical_4(const std::array<const float*, 7>& rows, const int x) {
  float32x4_t sum = vmulq_n_f32(vld1q_f32(rows[3] + x), kernel[3]);
  for (int distance = 1; distance <= 3; ++distance) {
    const float32x4_t pair = vaddq_f32(vld1q_f32(rows[3 - distance] + x), vld1q_f32(rows[3 + distance] + x));
    sum = vfmaq_n_f32(sum, pair, kernel[static_cast<std::size_t>(3 + distance)]);
  }
  return sum;
}

inline void blur_7x7_sigma_2(const cv::Mat& source, cv::Mat& destination, cv::Mat& horizontal_row_ring) {
  CV_Assert(source.type() == CV_8UC1);
  cv::Size whole_size;
  cv::Point offset;
  source.locateROI(whole_size, offset);
  CV_Assert(offset.x >= 3 && offset.y >= 3);
  CV_Assert(offset.x + source.cols + 3 <= whole_size.width);
  CV_Assert(offset.y + source.rows + 3 <= whole_size.height);

  constexpr int ring_row_count = 7;
  horizontal_row_ring.create(ring_row_count, source.cols, CV_32FC1);
  destination.create(source.size(), CV_8UC1);
  for (int source_y = -3; source_y <= 3; ++source_y) {
    const auto* input = source.data + static_cast<std::ptrdiff_t>(source_y) * static_cast<std::ptrdiff_t>(source.step);
    filter_horizontal_row(input, horizontal_row_ring.ptr<float>(source_y + 3), source.cols);
  }

  int oldest_ring_row = 0;
  for (int y = 0; y < source.rows; ++y) {
    std::array<const float*, ring_row_count> horizontal_rows{};
    int ring_row = oldest_ring_row;
    for (auto& horizontal_row : horizontal_rows) {
      horizontal_row = horizontal_row_ring.ptr<float>(ring_row);
      if (++ring_row == ring_row_count) {
        ring_row = 0;
      }
    }

    auto* output = destination.ptr<std::uint8_t>(y);
    int x = 0;
    for (; x + 8 <= source.cols; x += 8) {
      const int32x4_t low = vcvtnq_s32_f32(filter_vertical_4(horizontal_rows, x));
      const int32x4_t high = vcvtnq_s32_f32(filter_vertical_4(horizontal_rows, x + 4));
      vst1_u8(output + x, vqmovun_s16(vcombine_s16(vqmovn_s32(low), vqmovn_s32(high))));
    }
    for (; x < source.cols; ++x) {
      float sum = kernel[3] * horizontal_rows[3][x];
      for (int distance = 1; distance <= 3; ++distance) {
        sum += kernel[static_cast<std::size_t>(3 + distance)] * (horizontal_rows[3 + distance][x] + horizontal_rows[3 - distance][x]);
      }
      output[x] = cv::saturate_cast<std::uint8_t>(sum);
    }

    if (y + 1 < source.rows) {
      const int next_source_y = y + 4;
      const auto* next_input = source.data + static_cast<std::ptrdiff_t>(next_source_y) * static_cast<std::ptrdiff_t>(source.step);
      filter_horizontal_row(next_input, horizontal_row_ring.ptr<float>(oldest_ring_row), source.cols);
    }
    if (++oldest_ring_row == ring_row_count) {
      oldest_ring_row = 0;
    }
  }
}

// Q15 quantization of the OpenCV 7x7 sigma=2 kernel. The center coefficient
// is adjusted by one so the integer coefficients sum exactly to 32768.
inline constexpr std::array<std::uint16_t, 7> kernel_q15{{
    2299,
    4295,
    6249,
    7082,
    6249,
    4295,
    2299,
}};

inline void filter_horizontal_fixed_8(const std::uint8_t* input, std::uint16_t* output, const int x) {
  const auto* center = input + x;
  const uint16x8_t center_values = vmovl_u8(vld1_u8(center));
  uint32x4_t sum_low = vmull_n_u16(vget_low_u16(center_values), kernel_q15[3]);
  uint32x4_t sum_high = vmull_n_u16(vget_high_u16(center_values), kernel_q15[3]);
  for (int distance = 1; distance <= 3; ++distance) {
    const uint16x8_t pair = vaddl_u8(vld1_u8(center - distance), vld1_u8(center + distance));
    const std::uint16_t coefficient = kernel_q15[static_cast<std::size_t>(3 + distance)];
    sum_low = vmlal_n_u16(sum_low, vget_low_u16(pair), coefficient);
    sum_high = vmlal_n_u16(sum_high, vget_high_u16(pair), coefficient);
  }
  vst1q_u16(output + x, vcombine_u16(vrshrn_n_u32(sum_low, 7), vrshrn_n_u32(sum_high, 7)));
}

inline void filter_horizontal_fixed_row(const std::uint8_t* input, std::uint16_t* output, const int width) {
  int x = 0;
  for (; x + 8 <= width; x += 8) {
    filter_horizontal_fixed_8(input, output, x);
  }
  for (; x < width; ++x) {
    std::uint32_t sum = kernel_q15[3] * static_cast<std::uint32_t>(input[x]);
    for (int distance = 1; distance <= 3; ++distance) {
      sum += kernel_q15[static_cast<std::size_t>(3 + distance)] *
             (static_cast<std::uint32_t>(input[x - distance]) + static_cast<std::uint32_t>(input[x + distance]));
    }
    output[x] = static_cast<std::uint16_t>((sum + 64U) >> 7U);
  }
}

inline uint32x4_t filter_vertical_fixed_4(const std::array<const std::uint16_t*, 7>& rows, const int x) {
  uint32x4_t sum = vmull_n_u16(vld1_u16(rows[3] + x), kernel_q15[3]);
  for (int distance = 1; distance <= 3; ++distance) {
    const uint32x4_t pair = vaddl_u16(vld1_u16(rows[3 - distance] + x), vld1_u16(rows[3 + distance] + x));
    sum = vmlaq_n_u32(sum, pair, kernel_q15[static_cast<std::size_t>(3 + distance)]);
  }
  return sum;
}

// Experimental fixed-point A/B path. Horizontal values retain eight
// fractional bits in a uint16 ring, and the vertical pass returns uint8.
inline void blur_7x7_sigma_2_fixed_point(const cv::Mat& source, cv::Mat& destination, cv::Mat& horizontal_row_ring) {
  CV_Assert(source.type() == CV_8UC1);
  cv::Size whole_size;
  cv::Point offset;
  source.locateROI(whole_size, offset);
  CV_Assert(offset.x >= 3 && offset.y >= 3);
  CV_Assert(offset.x + source.cols + 3 <= whole_size.width);
  CV_Assert(offset.y + source.rows + 3 <= whole_size.height);

  constexpr int ring_row_count = 7;
  horizontal_row_ring.create(ring_row_count, source.cols, CV_16UC1);
  destination.create(source.size(), CV_8UC1);
  for (int source_y = -3; source_y <= 3; ++source_y) {
    const auto* input = source.data + static_cast<std::ptrdiff_t>(source_y) * static_cast<std::ptrdiff_t>(source.step);
    filter_horizontal_fixed_row(input, horizontal_row_ring.ptr<std::uint16_t>(source_y + 3), source.cols);
  }

  int oldest_ring_row = 0;
  for (int y = 0; y < source.rows; ++y) {
    std::array<const std::uint16_t*, ring_row_count> horizontal_rows{};
    int ring_row = oldest_ring_row;
    for (auto& horizontal_row : horizontal_rows) {
      horizontal_row = horizontal_row_ring.ptr<std::uint16_t>(ring_row);
      if (++ring_row == ring_row_count) {
        ring_row = 0;
      }
    }

    auto* output = destination.ptr<std::uint8_t>(y);
    int x = 0;
    for (; x + 8 <= source.cols; x += 8) {
      const uint32x4_t low = vrshrq_n_u32(filter_vertical_fixed_4(horizontal_rows, x), 23);
      const uint32x4_t high = vrshrq_n_u32(filter_vertical_fixed_4(horizontal_rows, x + 4), 23);
      const uint16x8_t narrowed = vcombine_u16(vqmovn_u32(low), vqmovn_u32(high));
      vst1_u8(output + x, vqmovn_u16(narrowed));
    }
    for (; x < source.cols; ++x) {
      std::uint64_t sum = kernel_q15[3] * static_cast<std::uint64_t>(horizontal_rows[3][x]);
      for (int distance = 1; distance <= 3; ++distance) {
        sum += kernel_q15[static_cast<std::size_t>(3 + distance)] *
               (static_cast<std::uint64_t>(horizontal_rows[3 - distance][x]) + static_cast<std::uint64_t>(horizontal_rows[3 + distance][x]));
      }
      output[x] = static_cast<std::uint8_t>(std::min<std::uint64_t>(255U, (sum + (1ULL << 22U)) >> 23U));
    }

    if (y + 1 < source.rows) {
      const int next_source_y = y + 4;
      const auto* next_input = source.data + static_cast<std::ptrdiff_t>(next_source_y) * static_cast<std::ptrdiff_t>(source.step);
      filter_horizontal_fixed_row(next_input, horizontal_row_ring.ptr<std::uint16_t>(oldest_ring_row), source.cols);
    }
    if (++oldest_ring_row == ring_row_count) {
      oldest_ring_row = 0;
    }
  }
}

#endif

}  // namespace sapphire::backend::feature::gaussian_detail
