#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "tools/simd_sse_neon.h"

namespace sapphire::visual::fast_detail {

using offsets_t = std::array<int, 25>;

inline offsets_t make_offsets(const int row_stride) {
  static constexpr std::array<std::array<int, 2>, 16> circle{{
      {{0, 3}},
      {{1, 3}},
      {{2, 2}},
      {{3, 1}},
      {{3, 0}},
      {{3, -1}},
      {{2, -2}},
      {{1, -3}},
      {{0, -3}},
      {{-1, -3}},
      {{-2, -2}},
      {{-3, -1}},
      {{-3, 0}},
      {{-3, 1}},
      {{-2, 2}},
      {{-1, 3}},
  }};
  offsets_t offsets{};
  for (std::size_t index = 0; index < circle.size(); ++index) {
    offsets[index] = circle[index][0] + circle[index][1] * row_stride;
  }
  for (std::size_t index = circle.size(); index < offsets.size(); ++index) {
    offsets[index] = offsets[index - circle.size()];
  }
  return offsets;
}

inline std::array<std::int16_t, 25> load_differences(const std::uint8_t* center, const offsets_t& offsets) {
  std::array<std::int16_t, 25> differences{};
  for (std::size_t index = 0; index < differences.size(); ++index) {
    differences[index] = static_cast<std::int16_t>(static_cast<int>(center[0]) - static_cast<int>(center[offsets[index]]));
  }
  return differences;
}

inline int score_scalar(const std::uint8_t* center, const offsets_t& offsets, const int initial_threshold) {
  constexpr int pattern_size = 16;
  const auto differences = load_differences(center, offsets);

  int darker_score = initial_threshold;
  for (int index = 0; index < pattern_size; index += 2) {
    int score = std::min(static_cast<int>(differences[index + 1]), static_cast<int>(differences[index + 2]));
    score = std::min(score, static_cast<int>(differences[index + 3]));
    if (score <= darker_score) {
      continue;
    }
    score = std::min(score, static_cast<int>(differences[index + 4]));
    score = std::min(score, static_cast<int>(differences[index + 5]));
    score = std::min(score, static_cast<int>(differences[index + 6]));
    score = std::min(score, static_cast<int>(differences[index + 7]));
    score = std::min(score, static_cast<int>(differences[index + 8]));
    darker_score = std::max(darker_score, std::min(score, static_cast<int>(differences[index])));
    darker_score = std::max(darker_score, std::min(score, static_cast<int>(differences[index + 9])));
  }

  int brighter_score = -darker_score;
  for (int index = 0; index < pattern_size; index += 2) {
    int score = std::max(static_cast<int>(differences[index + 1]), static_cast<int>(differences[index + 2]));
    score = std::max(score, static_cast<int>(differences[index + 3]));
    score = std::max(score, static_cast<int>(differences[index + 4]));
    score = std::max(score, static_cast<int>(differences[index + 5]));
    if (score >= brighter_score) {
      continue;
    }
    score = std::max(score, static_cast<int>(differences[index + 6]));
    score = std::max(score, static_cast<int>(differences[index + 7]));
    score = std::max(score, static_cast<int>(differences[index + 8]));
    brighter_score = std::min(brighter_score, std::max(score, static_cast<int>(differences[index])));
    brighter_score = std::min(brighter_score, std::max(score, static_cast<int>(differences[index + 9])));
  }
  return -brighter_score - 1;
}

#if defined(SIMD_NEON)
inline int score_neon(const std::uint8_t* center, const offsets_t& offsets) {
  const auto differences = load_differences(center, offsets);
  int16x8_t maximum_dark = vdupq_n_s16(-1000);
  int16x8_t minimum_bright = vdupq_n_s16(1000);

  for (int index = 0; index < 16; index += 8) {
    int16x8_t value = vld1q_s16(differences.data() + index + 1);
    const int16x8_t next = vld1q_s16(differences.data() + index + 2);
    int16x8_t minimum = vminq_s16(value, next);
    int16x8_t maximum = vmaxq_s16(value, next);
    for (int offset = 3; offset <= 8; ++offset) {
      value = vld1q_s16(differences.data() + index + offset);
      minimum = vminq_s16(minimum, value);
      maximum = vmaxq_s16(maximum, value);
    }

    value = vld1q_s16(differences.data() + index);
    maximum_dark = vmaxq_s16(maximum_dark, vminq_s16(minimum, value));
    minimum_bright = vminq_s16(minimum_bright, vmaxq_s16(maximum, value));
    value = vld1q_s16(differences.data() + index + 9);
    maximum_dark = vmaxq_s16(maximum_dark, vminq_s16(minimum, value));
    minimum_bright = vminq_s16(minimum_bright, vmaxq_s16(maximum, value));
  }

  const int16x8_t score = vmaxq_s16(maximum_dark, vnegq_s16(minimum_bright));
  return static_cast<int>(vmaxvq_s16(score)) - 1;
}
#elif defined(SIMD_SSE2)
inline int score_sse2(const std::uint8_t* center, const offsets_t& offsets) {
  const auto differences = load_differences(center, offsets);
  __m128i maximum_dark = _mm_set1_epi16(-1000);
  __m128i minimum_bright = _mm_set1_epi16(1000);

  for (int index = 0; index < 16; index += 8) {
    __m128i value = _mm_loadu_si128(reinterpret_cast<const __m128i*>(differences.data() + index + 1));
    const __m128i next = _mm_loadu_si128(reinterpret_cast<const __m128i*>(differences.data() + index + 2));
    __m128i minimum = _mm_min_epi16(value, next);
    __m128i maximum = _mm_max_epi16(value, next);
    for (int offset = 3; offset <= 8; ++offset) {
      value = _mm_loadu_si128(reinterpret_cast<const __m128i*>(differences.data() + index + offset));
      minimum = _mm_min_epi16(minimum, value);
      maximum = _mm_max_epi16(maximum, value);
    }

    value = _mm_loadu_si128(reinterpret_cast<const __m128i*>(differences.data() + index));
    maximum_dark = _mm_max_epi16(maximum_dark, _mm_min_epi16(minimum, value));
    minimum_bright = _mm_min_epi16(minimum_bright, _mm_max_epi16(maximum, value));
    value = _mm_loadu_si128(reinterpret_cast<const __m128i*>(differences.data() + index + 9));
    maximum_dark = _mm_max_epi16(maximum_dark, _mm_min_epi16(minimum, value));
    minimum_bright = _mm_min_epi16(minimum_bright, _mm_max_epi16(maximum, value));
  }

  __m128i score = _mm_max_epi16(maximum_dark, _mm_sub_epi16(_mm_setzero_si128(), minimum_bright));
  score = _mm_max_epi16(score, _mm_srli_si128(score, 8));
  score = _mm_max_epi16(score, _mm_srli_si128(score, 4));
  score = _mm_max_epi16(score, _mm_srli_si128(score, 2));
  return static_cast<int>(_mm_extract_epi16(score, 0)) - 1;
}
#endif

inline int score(const std::uint8_t* center, const offsets_t& offsets, const int initial_threshold) {
#if defined(SIMD_NEON)
  static_cast<void>(initial_threshold);
  return score_neon(center, offsets);
#elif defined(SIMD_SSE2)
  static_cast<void>(initial_threshold);
  return score_sse2(center, offsets);
#else
  return score_scalar(center, offsets, initial_threshold);
#endif
}

}  // namespace sapphire::visual::fast_detail
