#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

#include "backend/tools/simd_sse_neon.hpp"

namespace sapphire::backend::feature {

namespace pyramid_detail {

inline constexpr std::uint16_t resize_coefficient_scale = 1U << 11U;
inline constexpr unsigned int resize_result_shift = 22U;

struct ResizeWorkspace {
  cv::Size source_size;
  cv::Size destination_size;
  std::vector<int> x_offsets;
  std::vector<int> y_offsets;
  std::vector<std::uint16_t> x_weights;
  std::vector<std::uint16_t> y_weights;
  std::vector<int> vector_block_bases;
  std::vector<std::array<std::uint8_t, 8>> vector_block_indices;
  std::size_t vectorized_width = 0;
};

inline void fill_reflect101_border(cv::Mat& level, cv::Mat& buffer, const int padding) {
  CV_Assert(level.type() == CV_8UC1 && buffer.type() == CV_8UC1);
  CV_Assert(padding >= 0);
  CV_Assert(buffer.rows == level.rows + 2 * padding);
  CV_Assert(buffer.cols == level.cols + 2 * padding);
  cv::Size whole_size;
  cv::Point offset;
  level.locateROI(whole_size, offset);
  CV_Assert(whole_size == buffer.size());
  CV_Assert(offset.x == padding && offset.y == padding);
  if (padding == 0) {
    return;
  }

  // The direct path writes only the border. Keep a general fallback for
  // unusually tiny pyramid levels where REFLECT_101 repeats more than once.
  if (level.cols <= padding || level.rows <= padding) {
    const cv::Mat source = level.clone();
    cv::copyMakeBorder(source, buffer, padding, padding, padding, padding, cv::BORDER_REFLECT_101 | cv::BORDER_ISOLATED);
    return;
  }

  for (int y = 0; y < level.rows; ++y) {
    auto* row = level.ptr<std::uint8_t>(y);
    for (int distance = 1; distance <= padding; ++distance) {
      row[-distance] = row[distance];
      row[level.cols - 1 + distance] = row[level.cols - 1 - distance];
    }
  }

  const std::size_t row_bytes = static_cast<std::size_t>(buffer.cols);
  for (int distance = 1; distance <= padding; ++distance) {
    std::memcpy(buffer.ptr<std::uint8_t>(padding - distance), buffer.ptr<std::uint8_t>(padding + distance), row_bytes);
    std::memcpy(buffer.ptr<std::uint8_t>(padding + level.rows - 1 + distance), buffer.ptr<std::uint8_t>(padding + level.rows - 1 - distance), row_bytes);
  }
}

#if defined(SIMD_NEON)
inline void prepare_resize_workspace(const cv::Size& source_size, const cv::Size& destination_size, ResizeWorkspace& workspace) {
  if (workspace.source_size == source_size && workspace.destination_size == destination_size) {
    return;
  }

  workspace.source_size = source_size;
  workspace.destination_size = destination_size;
  workspace.x_offsets.resize(destination_size.width);
  workspace.y_offsets.resize(destination_size.height);
  workspace.x_weights.resize(destination_size.width);
  workspace.y_weights.resize(destination_size.height);

  const auto prepare_axis = [](const int source_length, const int destination_length, std::vector<int>& offsets, std::vector<std::uint16_t>& weights) {
    const double scale = static_cast<double>(source_length) / destination_length;
    for (int destination_index = 0; destination_index < destination_length; ++destination_index) {
      const double source_position = (destination_index + 0.5) * scale - 0.5;
      int source_index = static_cast<int>(std::floor(source_position));
      double fraction = source_position - source_index;
      if (source_length <= 1 || source_index < 0) {
        source_index = 0;
        fraction = 0.0;
      } else if (source_index >= source_length - 1) {
        source_index = source_length - 2;
        fraction = 1.0;
      }
      offsets[destination_index] = source_index;
      weights[destination_index] =
          static_cast<std::uint16_t>(std::clamp(cvRound(fraction * resize_coefficient_scale), 0, static_cast<int>(resize_coefficient_scale)));
    }
  };

  prepare_axis(source_size.width, destination_size.width, workspace.x_offsets, workspace.x_weights);
  prepare_axis(source_size.height, destination_size.height, workspace.y_offsets, workspace.y_weights);

  workspace.vector_block_bases.clear();
  workspace.vector_block_indices.clear();
  workspace.vectorized_width = 0;
  for (int destination_x = 0; destination_x + 8 <= destination_size.width; destination_x += 8) {
    const int source_base = workspace.x_offsets[destination_x];
    const int source_last = workspace.x_offsets[destination_x + 7] + 1;
    if (source_base + 15 >= source_size.width || source_last - source_base >= 16) {
      break;
    }
    std::array<std::uint8_t, 8> indices{};
    for (int lane = 0; lane < 8; ++lane) {
      indices[lane] = static_cast<std::uint8_t>(workspace.x_offsets[destination_x + lane] - source_base);
    }
    workspace.vector_block_bases.push_back(source_base);
    workspace.vector_block_indices.push_back(indices);
    workspace.vectorized_width += 8;
  }
}

inline void resize_rows_neon(const cv::Mat& source, cv::Mat& destination, const ResizeWorkspace& workspace) {
  const uint16x8_t coefficient_scale = vdupq_n_u16(resize_coefficient_scale);
  for (int destination_y = 0; destination_y < destination.rows; ++destination_y) {
    const int source_y = workspace.y_offsets[destination_y];
    const std::uint32_t y_weight = workspace.y_weights[destination_y];
    const std::uint32_t inverse_y_weight = resize_coefficient_scale - y_weight;
    const auto* top_row = source.ptr<std::uint8_t>(source_y);
    const auto* bottom_row = source.ptr<std::uint8_t>(std::min(source_y + 1, source.rows - 1));
    auto* destination_row = destination.ptr<std::uint8_t>(destination_y);

    std::size_t block_index = 0;
    for (; block_index < workspace.vector_block_bases.size(); ++block_index) {
      const int destination_x = static_cast<int>(block_index * 8U);
      const int source_x = workspace.vector_block_bases[block_index];
      const uint8x8_t indices = vld1_u8(workspace.vector_block_indices[block_index].data());
      const uint8x8_t right_indices = vadd_u8(indices, vdup_n_u8(1));
      const uint8x16_t top_table = vld1q_u8(top_row + source_x);
      const uint8x16_t bottom_table = vld1q_u8(bottom_row + source_x);
      const uint16x8_t top_left = vmovl_u8(vqtbl1_u8(top_table, indices));
      const uint16x8_t top_right = vmovl_u8(vqtbl1_u8(top_table, right_indices));
      const uint16x8_t bottom_left = vmovl_u8(vqtbl1_u8(bottom_table, indices));
      const uint16x8_t bottom_right = vmovl_u8(vqtbl1_u8(bottom_table, right_indices));
      const uint16x8_t x_weight = vld1q_u16(workspace.x_weights.data() + destination_x);
      const uint16x8_t inverse_x_weight = vsubq_u16(coefficient_scale, x_weight);

      const uint32x4_t top_low = vmlal_u16(vmull_u16(vget_low_u16(top_left), vget_low_u16(inverse_x_weight)), vget_low_u16(top_right), vget_low_u16(x_weight));
      const uint32x4_t top_high =
          vmlal_u16(vmull_u16(vget_high_u16(top_left), vget_high_u16(inverse_x_weight)), vget_high_u16(top_right), vget_high_u16(x_weight));
      const uint32x4_t bottom_low =
          vmlal_u16(vmull_u16(vget_low_u16(bottom_left), vget_low_u16(inverse_x_weight)), vget_low_u16(bottom_right), vget_low_u16(x_weight));
      const uint32x4_t bottom_high =
          vmlal_u16(vmull_u16(vget_high_u16(bottom_left), vget_high_u16(inverse_x_weight)), vget_high_u16(bottom_right), vget_high_u16(x_weight));

      const uint32x4_t result_low = vrshrq_n_u32(vaddq_u32(vmulq_n_u32(top_low, inverse_y_weight), vmulq_n_u32(bottom_low, y_weight)), resize_result_shift);
      const uint32x4_t result_high = vrshrq_n_u32(vaddq_u32(vmulq_n_u32(top_high, inverse_y_weight), vmulq_n_u32(bottom_high, y_weight)), resize_result_shift);
      const uint16x8_t result_u16 = vcombine_u16(vqmovn_u32(result_low), vqmovn_u32(result_high));
      vst1_u8(destination_row + destination_x, vqmovn_u16(result_u16));
    }

    for (int destination_x = static_cast<int>(workspace.vectorized_width); destination_x < destination.cols; ++destination_x) {
      const int source_x = workspace.x_offsets[destination_x];
      const std::uint32_t x_weight = workspace.x_weights[destination_x];
      const std::uint32_t inverse_x_weight = resize_coefficient_scale - x_weight;
      const std::uint32_t top_value = top_row[source_x] * inverse_x_weight + top_row[std::min(source_x + 1, source.cols - 1)] * x_weight;
      const std::uint32_t bottom_value = bottom_row[source_x] * inverse_x_weight + bottom_row[std::min(source_x + 1, source.cols - 1)] * x_weight;
      const std::uint64_t value = static_cast<std::uint64_t>(top_value) * inverse_y_weight + static_cast<std::uint64_t>(bottom_value) * y_weight;
      destination_row[destination_x] = static_cast<std::uint8_t>((value + (1ULL << (resize_result_shift - 1U))) >> resize_result_shift);
    }
  }
}
#endif

inline void resize_level(const cv::Mat& source, cv::Mat& destination, ResizeWorkspace& workspace) {
  CV_Assert(source.type() == CV_8UC1);
  CV_Assert(destination.cols > 0 && destination.rows > 0);
#if defined(SIMD_NEON)
  prepare_resize_workspace(source.size(), destination.size(), workspace);
  resize_rows_neon(source, destination, workspace);
#else
  static_cast<void>(workspace);
  cv::resize(source, destination, destination.size(), 0.0, 0.0, cv::INTER_LINEAR);
#endif
}

}  // namespace pyramid_detail

class Pyramid {
 public:
  Pyramid(const std::size_t num_levels, const int padding)
      : padding_(padding),
        levels_(num_levels),
        buffers_(num_levels),
        blurred_levels_(num_levels),
        gaussian_workspaces_(num_levels),
        level_states_(num_levels, level_state_t::uninitialized),
        resize_workspaces_(num_levels) {
    if (padding < 0) {
      throw std::invalid_argument("pyramid padding must be non-negative");
    }
  }

  void compute(const cv::Mat& image, const std::vector<float>& inverse_scale_factors) {
    CV_Assert(image.type() == CV_8UC1);
    CV_Assert(levels_.size() == inverse_scale_factors.size());
    std::fill(level_states_.begin(), level_states_.end(), level_state_t::uninitialized);
    for (std::size_t level = 0; level < levels_.size(); ++level) {
      const float inverse_scale = inverse_scale_factors[level];
      const cv::Size size(cvRound(static_cast<float>(image.cols) * inverse_scale), cvRound(static_cast<float>(image.rows) * inverse_scale));
      auto& buffer = buffers_[level];
      buffer.create(size.height + 2 * padding_, size.width + 2 * padding_, image.type());
      levels_[level] = buffer(cv::Rect(padding_, padding_, size.width, size.height));
      if (level == 0) {
        image.copyTo(levels_[level]);
        pyramid_detail::fill_reflect101_border(levels_[level], buffer, padding_);
        level_states_[level] = level_state_t::padded;
        continue;
      }
      pyramid_detail::resize_level(levels_[level - 1], levels_[level], resize_workspaces_[level]);
      pyramid_detail::fill_reflect101_border(levels_[level], buffer, padding_);
      level_states_[level] = level_state_t::padded;
    }
  }

  void blur_level(std::size_t level);

  cv::Mat& at(const std::size_t level) { return levels_.at(level); }

  const cv::Mat& at(const std::size_t level) const { return levels_.at(level); }

  const std::vector<cv::Mat>& levels() const { return levels_; }

 private:
  enum class level_state_t : std::uint8_t {
    uninitialized,
    padded,
    blurred,
  };

  int padding_;
  std::vector<cv::Mat> levels_;
  std::vector<cv::Mat> buffers_;
  std::vector<cv::Mat> blurred_levels_;
  std::vector<cv::Mat> gaussian_workspaces_;
  std::vector<level_state_t> level_states_;
  std::vector<pyramid_detail::ResizeWorkspace> resize_workspaces_;
};

}  // namespace sapphire::backend::feature
