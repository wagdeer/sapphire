#pragma once
#include <opencv2/core/mat.hpp>
#include "tools/simd_sse_neon.h"

namespace sapphire {
// Exact 2x2 area reduction for the admitted CV_8UC1 camera frames. A new buffer
// preserves input ownership/stride; non-2x resizing remains the caller's choice.
inline cv::Mat halfGrayArea(const cv::Mat &source) {
  CV_Assert(source.type() == CV_8UC1 && !source.empty() && source.cols % 2 == 0 && source.rows % 2 == 0);
  cv::Mat result(source.rows / 2, source.cols / 2, CV_8UC1);
  for (int y = 0; y < result.rows; ++y) {
    const auto *a = source.ptr<std::uint8_t>(2 * y), *b = source.ptr<std::uint8_t>(2 * y + 1);
    auto *out = result.ptr<std::uint8_t>(y);
    int x = 0;
#if defined(SIMD_SSE2)
    const auto zero = _mm_setzero_si128(), one = _mm_set1_epi16(1), rounding = _mm_set1_epi32(2);
    for (; x + 16 <= source.cols; x += 16) {
      const auto av = _mm_loadu_si128(reinterpret_cast<const __m128i *>(a + x));
      const auto bv = _mm_loadu_si128(reinterpret_cast<const __m128i *>(b + x));
      const auto low = _mm_add_epi16(_mm_unpacklo_epi8(av, zero), _mm_unpacklo_epi8(bv, zero));
      const auto high = _mm_add_epi16(_mm_unpackhi_epi8(av, zero), _mm_unpackhi_epi8(bv, zero));
      const auto sums0 = _mm_srli_epi32(_mm_add_epi32(_mm_madd_epi16(low, one), rounding), 2);
      const auto sums1 = _mm_srli_epi32(_mm_add_epi32(_mm_madd_epi16(high, one), rounding), 2);
      const auto pixels = _mm_packus_epi16(_mm_packs_epi32(sums0, sums1), zero);
      _mm_storel_epi64(reinterpret_cast<__m128i *>(out + x / 2), pixels);
    }
#elif defined(SIMD_NEON)
    for (; x + 16 <= source.cols; x += 16) {
      const auto av = vld1q_u8(a + x), bv = vld1q_u8(b + x);
      const auto low = vpaddlq_u16(vaddl_u8(vget_low_u8(av), vget_low_u8(bv)));
      const auto high = vpaddlq_u16(vaddl_u8(vget_high_u8(av), vget_high_u8(bv)));
      vst1_u8(out + x / 2, vmovn_u16(vcombine_u16(vrshrn_n_u32(low, 2), vrshrn_n_u32(high, 2))));
    }
#endif
    for (; x < source.cols; x += 2) out[x / 2] = (int(a[x]) + a[x + 1] + b[x] + b[x + 1] + 2) >> 2;
  }
  return result;
}
}  // namespace sapphire
