#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "backend/visual/feature/fast_score.hpp"

namespace sapphire::visual::fast_detail {

inline int positive_modulo_3(const int value) {
  const int remainder = value % 3;
  return remainder < 0 ? remainder + 3 : remainder;
}

template <typename Emit>
void detect_9_16_cell_nms(const cv::Mat& image, int threshold, std::array<std::vector<std::uint8_t>, 3>& score_rows,
                          std::array<std::vector<int>, 3>& corner_positions, const cv::Rect& valid_centers, const int cell_width, const int cell_height,
                          const int num_cell_columns, const Emit& emit) {
  CV_Assert(image.type() == CV_8UC1);
  CV_Assert(cell_width > 0 && cell_height > 0 && num_cell_columns > 0);
  CV_Assert(valid_centers.x >= 3 && valid_centers.y >= 3);
  CV_Assert(valid_centers.x + valid_centers.width <= image.cols - 3);
  CV_Assert(valid_centers.y + valid_centers.height <= image.rows - 3);
  if (image.cols < 7 || image.rows < 7) {
    return;
  }

  threshold = std::clamp(threshold, 0, 255);
  for (auto& scores : score_rows) {
    scores.assign(static_cast<std::size_t>(image.cols), 0U);
  }
  for (auto& positions : corner_positions) {
    positions.clear();
    if (positions.capacity() < static_cast<std::size_t>(image.cols)) {
      positions.reserve(static_cast<std::size_t>(image.cols));
    }
  }
  const auto offsets = make_offsets(static_cast<int>(image.step1()));

  std::array<std::uint8_t, 512> threshold_table{};
  for (int difference = -255; difference <= 255; ++difference) {
    threshold_table[static_cast<std::size_t>(difference + 255)] = static_cast<std::uint8_t>(difference < -threshold ? 1 : difference > threshold ? 2 : 0);
  }

#if CV_SIMD128
  const cv::v_uint8x16 delta = cv::v_setall_u8(0x80);
  const cv::v_uint8x16 threshold_vector = cv::v_setall_u8(static_cast<uchar>(threshold));
  const cv::v_uint8x16 contiguous_required = cv::v_setall_u8(8);
#endif

  for (int row = 3; row < image.rows - 2; ++row) {
    const int current_index = (row - 3) % 3;
    auto& current_scores = score_rows[current_index];
    auto& current_positions = corner_positions[current_index];
    std::fill(current_scores.begin(), current_scores.end(), 0U);
    current_positions.clear();

    if (row < image.rows - 3) {
      const std::uint8_t* pixel = image.ptr<std::uint8_t>(row) + 3;
      int column = 3;

#if CV_SIMD128
      for (; column < image.cols - 16 - 3; column += 16, pixel += 16) {
        const cv::v_uint8x16 center = cv::v_load(pixel);
        const cv::v_int8x16 upper_threshold = cv::v_reinterpret_as_s8((center + threshold_vector) ^ delta);
        const cv::v_int8x16 lower_threshold = cv::v_reinterpret_as_s8((center - threshold_vector) ^ delta);

        const cv::v_int8x16 cardinal_0 = cv::v_reinterpret_as_s8(cv::v_sub_wrap(cv::v_load(pixel + offsets[0]), delta));
        const cv::v_int8x16 cardinal_1 = cv::v_reinterpret_as_s8(cv::v_sub_wrap(cv::v_load(pixel + offsets[4]), delta));
        const cv::v_int8x16 cardinal_2 = cv::v_reinterpret_as_s8(cv::v_sub_wrap(cv::v_load(pixel + offsets[8]), delta));
        const cv::v_int8x16 cardinal_3 = cv::v_reinterpret_as_s8(cv::v_sub_wrap(cv::v_load(pixel + offsets[12]), delta));

        cv::v_int8x16 darker = (upper_threshold < cardinal_0) & (upper_threshold < cardinal_1);
        cv::v_int8x16 brighter = (cardinal_0 < lower_threshold) & (cardinal_1 < lower_threshold);
        darker = darker | ((upper_threshold < cardinal_1) & (upper_threshold < cardinal_2));
        brighter = brighter | ((cardinal_1 < lower_threshold) & (cardinal_2 < lower_threshold));
        darker = darker | ((upper_threshold < cardinal_2) & (upper_threshold < cardinal_3));
        brighter = brighter | ((cardinal_2 < lower_threshold) & (cardinal_3 < lower_threshold));
        darker = darker | ((upper_threshold < cardinal_3) & (upper_threshold < cardinal_0));
        brighter = brighter | ((cardinal_3 < lower_threshold) & (cardinal_0 < lower_threshold));
        cv::v_int8x16 possible = darker | brighter;

        if (!cv::v_check_any(possible)) {
          continue;
        }
        if (!cv::v_check_any(cv::v_combine_low(possible, possible))) {
          column -= 8;
          pixel -= 8;
          continue;
        }

        cv::v_int8x16 darker_run = cv::v_setzero_s8();
        cv::v_int8x16 brighter_run = cv::v_setzero_s8();
        cv::v_uint8x16 maximum_darker = cv::v_setzero_u8();
        cv::v_uint8x16 maximum_brighter = cv::v_setzero_u8();
        for (int circle = 0; circle < 25; ++circle) {
          const cv::v_int8x16 sample = cv::v_reinterpret_as_s8(cv::v_load(pixel + offsets[circle]) ^ delta);
          darker = upper_threshold < sample;
          brighter = sample < lower_threshold;
          darker_run = cv::v_sub_wrap(darker_run, darker) & darker;
          brighter_run = cv::v_sub_wrap(brighter_run, brighter) & brighter;
          maximum_darker = cv::v_max(maximum_darker, cv::v_reinterpret_as_u8(darker_run));
          maximum_brighter = cv::v_max(maximum_brighter, cv::v_reinterpret_as_u8(brighter_run));
        }

        const cv::v_uint8x16 is_corner = contiguous_required < cv::v_max(maximum_darker, maximum_brighter);
        unsigned int mask = cv::v_signmask(cv::v_reinterpret_as_s8(is_corner));
        for (int lane = 0; mask != 0U && lane < 16; ++lane, mask >>= 1U) {
          if ((mask & 1U) == 0U) {
            continue;
          }
          const int candidate_column = column + lane;
          current_positions.push_back(candidate_column);
          current_scores[candidate_column] = static_cast<std::uint8_t>(score(pixel + lane, offsets, threshold));
        }
      }
#endif

      for (; column < image.cols - 3; ++column, ++pixel) {
        const int center = pixel[0];
        const std::uint8_t* table = threshold_table.data() - center + 255;
        int comparison = table[pixel[offsets[0]]] | table[pixel[offsets[8]]];
        if (comparison == 0) {
          continue;
        }
        comparison &= table[pixel[offsets[2]]] | table[pixel[offsets[10]]];
        comparison &= table[pixel[offsets[4]]] | table[pixel[offsets[12]]];
        comparison &= table[pixel[offsets[6]]] | table[pixel[offsets[14]]];
        if (comparison == 0) {
          continue;
        }
        comparison &= table[pixel[offsets[1]]] | table[pixel[offsets[9]]];
        comparison &= table[pixel[offsets[3]]] | table[pixel[offsets[11]]];
        comparison &= table[pixel[offsets[5]]] | table[pixel[offsets[13]]];
        comparison &= table[pixel[offsets[7]]] | table[pixel[offsets[15]]];

        bool is_corner = false;
        if ((comparison & 1) != 0) {
          const int limit = center - threshold;
          int contiguous = 0;
          for (int circle = 0; circle < 25; ++circle) {
            if (pixel[offsets[circle]] < limit) {
              if (++contiguous > 8) {
                is_corner = true;
                break;
              }
            } else {
              contiguous = 0;
            }
          }
        }
        if (!is_corner && (comparison & 2) != 0) {
          const int limit = center + threshold;
          int contiguous = 0;
          for (int circle = 0; circle < 25; ++circle) {
            if (pixel[offsets[circle]] > limit) {
              if (++contiguous > 8) {
                is_corner = true;
                break;
              }
            } else {
              contiguous = 0;
            }
          }
        }
        if (is_corner) {
          current_positions.push_back(column);
          current_scores[column] = static_cast<std::uint8_t>(score(pixel, offsets, threshold));
        }
      }
    }

    if (row == 3) {
      continue;
    }

    const int candidate_row = row - 1;
    const int middle_index = positive_modulo_3(row - 4);
    const int upper_index = positive_modulo_3(row - 5);
    const auto& middle_scores = score_rows[middle_index];
    const auto& upper_scores = score_rows[upper_index];
    const auto& positions = corner_positions[middle_index];

    for (const int column : positions) {
      if (!valid_centers.contains(cv::Point(column, candidate_row))) {
        continue;
      }
      const int score_value = middle_scores[column];
      const int cell_column = (column - valid_centers.x) / cell_width;
      const int cell_row = (candidate_row - valid_centers.y) / cell_height;
      const int candidate_cell = cell_row * num_cell_columns + cell_column;
      const int first_x = std::max(column - 1, valid_centers.x + cell_column * cell_width);
      const int last_x = std::min(column + 1, std::min(valid_centers.x + (cell_column + 1) * cell_width, valid_centers.x + valid_centers.width) - 1);
      const int first_y = std::max(candidate_row - 1, valid_centers.y + cell_row * cell_height);
      const int last_y = std::min(candidate_row + 1, std::min(valid_centers.y + (cell_row + 1) * cell_height, valid_centers.y + valid_centers.height) - 1);
      bool is_maximum = true;
      for (int neighbor_row = first_y; neighbor_row <= last_y && is_maximum; ++neighbor_row) {
        const int y_offset = neighbor_row - candidate_row;
        const auto& neighbor_scores = y_offset < 0 ? upper_scores : y_offset == 0 ? middle_scores : current_scores;
        for (int neighbor_column = first_x; neighbor_column <= last_x; ++neighbor_column) {
          if (neighbor_column == column && y_offset == 0) {
            continue;
          }
          if (score_value <= neighbor_scores[neighbor_column]) {
            is_maximum = false;
            break;
          }
        }
      }
      if (is_maximum) {
        emit(column, candidate_row, score_value, candidate_cell);
      }
    }
  }
}

}  // namespace sapphire::visual::fast_detail
