#include <backend/registration/descriptor/orb.hpp>
#include <backend/registration/fast_orb.hpp>
#include <backend/tools/simd_sse_neon.hpp>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "backend/registration/descriptor/orb_rotated_pattern.h"

namespace sapphire::backend::feature {
namespace {
// Circular intensity-centroid support for the fixed 31-pixel ORB patch.
// These are the original rounded extents with symmetric boundary correction;
// immutable geometry is shared instead of rebuilt in each image workspace.
constexpr std::array<int, 16> orientation_extents{15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};
static_assert(orientation_extents.size() == OrbDescriptor::patch_size / 2 + 1);

int descriptor_angle_index(const float angle) {
  if (!std::isfinite(angle)) {
    return 0;
  }
  int index = static_cast<int>(angle) % 360;
  if (index < 0) {
    index += 360;
  }
  return index;
}

[[maybe_unused]] void compute_fast_orb_scalar(const cv::KeyPoint& keypoint, const cv::Mat& image, uchar* descriptor) {
  const auto& pattern = detail::rotated_orb_patterns[descriptor_angle_index(keypoint.angle)];
  const uchar* center = &image.at<uchar>(cvRound(keypoint.pt.y), cvRound(keypoint.pt.x));
  const int step = static_cast<int>(image.step1());

  for (std::size_t byte = 0; byte < 32U; ++byte) {
    int samples[16];
    const std::size_t pattern_offset = byte * 16U;
    for (std::size_t point = 0; point < 16U; ++point) {
      const auto& offset = pattern[pattern_offset + point];
      samples[point] = center[static_cast<int>(offset.row) * step + static_cast<int>(offset.column)];
    }
    int value = samples[0] < samples[1];
    value |= (samples[2] < samples[3]) << 1;
    value |= (samples[4] < samples[5]) << 2;
    value |= (samples[6] < samples[7]) << 3;
    value |= (samples[8] < samples[9]) << 4;
    value |= (samples[10] < samples[11]) << 5;
    value |= (samples[12] < samples[13]) << 6;
    value |= (samples[14] < samples[15]) << 7;
    descriptor[byte] = static_cast<uchar>(value);
  }
}

#if defined(SIMD_SSE2)
void compute_fast_orb_sse2(const cv::KeyPoint& keypoint, const cv::Mat& image, uchar* descriptor) {
  const auto& pattern = detail::rotated_orb_patterns[descriptor_angle_index(keypoint.angle)];
  const int row = cvRound(keypoint.pt.y);
  const int column = cvRound(keypoint.pt.x);
  const int step = static_cast<int>(image.step1());
  const uchar* center = image.data + row * step + column;
  const __m128i low_byte_mask = _mm_set1_epi16(0x00FF);
  const __m128i zero = _mm_setzero_si128();

  for (std::size_t byte = 0; byte < 32U; ++byte) {
    alignas(16) uchar samples[16];
    const std::size_t pattern_offset = byte * 16U;
    for (std::size_t point = 0; point < 16U; ++point) {
      const auto& offset = pattern[pattern_offset + point];
      samples[point] = center[static_cast<int>(offset.row) * step + static_cast<int>(offset.column)];
    }

    const __m128i sample_values = _mm_load_si128(reinterpret_cast<const __m128i*>(samples));
    const __m128i even_samples = _mm_and_si128(sample_values, low_byte_mask);
    const __m128i odd_samples = _mm_srli_epi16(sample_values, 8);
    const __m128i comparisons = _mm_cmplt_epi16(even_samples, odd_samples);
    const __m128i packed_comparisons = _mm_packs_epi16(comparisons, zero);
    descriptor[byte] = static_cast<uchar>(_mm_movemask_epi8(packed_comparisons) & 0xFF);
  }
}
#endif

#if defined(SIMD_NEON)
alignas(16) constexpr std::array<std::int16_t, 8> ic_lane_offsets{0, 1, 2, 3, 4, 5, 6, 7};
alignas(8) constexpr std::array<std::uint8_t, 8> descriptor_bit_weights{1, 2, 4, 8, 16, 32, 64, 128};

void accumulate_ic_moment_neon(const uchar* pixels, const int first_u, int32x4_t& m_10) {
  const int16x8_t u = vaddq_s16(vdupq_n_s16(static_cast<std::int16_t>(first_u)), vld1q_s16(ic_lane_offsets.data()));
  const int16x8_t values = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(pixels)));
  m_10 = vmlal_s16(m_10, vget_low_s16(values), vget_low_s16(u));
  m_10 = vmlal_s16(m_10, vget_high_s16(values), vget_high_s16(u));
}

void accumulate_ic_moments_neon(const uchar* plus, const uchar* minus, const int first_u, const int v, int32x4_t& m_01, int32x4_t& m_10) {
  const uint8x8_t plus_values = vld1_u8(plus);
  const uint8x8_t minus_values = vld1_u8(minus);
  const int16x8_t difference = vreinterpretq_s16_u16(vsubl_u8(plus_values, minus_values));
  const int16x8_t sum = vreinterpretq_s16_u16(vaddl_u8(plus_values, minus_values));
  const int16x8_t u = vaddq_s16(vdupq_n_s16(static_cast<std::int16_t>(first_u)), vld1q_s16(ic_lane_offsets.data()));

  m_01 = vmlal_n_s16(m_01, vget_low_s16(difference), static_cast<std::int16_t>(v));
  m_01 = vmlal_n_s16(m_01, vget_high_s16(difference), static_cast<std::int16_t>(v));
  m_10 = vmlal_s16(m_10, vget_low_s16(sum), vget_low_s16(u));
  m_10 = vmlal_s16(m_10, vget_high_s16(sum), vget_high_s16(u));
}

float ic_angle_neon(const cv::Mat& image, const cv::Point2f& point, const std::array<int, OrbDescriptor::patch_size / 2 + 1>& u_max) {
  const uchar* const center = &image.at<uchar>(cvRound(point.y), cvRound(point.x));
  const int step = static_cast<int>(image.step1());
  int32x4_t m_01_vector = vdupq_n_s32(0);
  int32x4_t m_10_vector = vdupq_n_s32(0);
  int m_01_scalar = 0;
  int m_10_scalar = 0;

  int u = -OrbDescriptor::patch_size / 2;
  for (; u + 7 <= OrbDescriptor::patch_size / 2; u += 8) {
    accumulate_ic_moment_neon(center + u, u, m_10_vector);
  }
  for (; u <= OrbDescriptor::patch_size / 2; ++u) {
    m_10_scalar += u * center[u];
  }

  for (int v = 1; v <= OrbDescriptor::patch_size / 2; ++v) {
    const int d = u_max[static_cast<std::size_t>(v)];
    u = -d;
    for (; u + 7 <= d; u += 8) {
      accumulate_ic_moments_neon(center + v * step + u, center - v * step + u, u, v, m_01_vector, m_10_vector);
    }
    for (; u <= d; ++u) {
      const int val_plus = center[u + v * step];
      const int val_minus = center[u - v * step];
      m_01_scalar += v * (val_plus - val_minus);
      m_10_scalar += u * (val_plus + val_minus);
    }
  }

  const int m_01 = m_01_scalar + vaddvq_s32(m_01_vector);
  const int m_10 = m_10_scalar + vaddvq_s32(m_10_vector);
  return cv::fastAtan2(static_cast<float>(m_01), static_cast<float>(m_10));
}

void compute_fast_orb_neon(const cv::KeyPoint& keypoint, const cv::Mat& image, uchar* descriptor) {
  const auto& pattern = detail::rotated_orb_patterns[descriptor_angle_index(keypoint.angle)];
  const int row = cvRound(keypoint.pt.y);
  const int column = cvRound(keypoint.pt.x);
  const int step = static_cast<int>(image.step1());
  const uchar* center = image.data + row * step + column;
  const uint8x8_t bit_weights = vld1_u8(descriptor_bit_weights.data());

  for (std::size_t byte = 0; byte < 32U; ++byte) {
    alignas(16) uchar samples[16];
    const std::size_t pattern_offset = byte * 16U;
    for (std::size_t point = 0; point < 16U; ++point) {
      const auto& offset = pattern[pattern_offset + point];
      samples[point] = center[static_cast<int>(offset.row) * step + static_cast<int>(offset.column)];
    }
    const uint8x16_t sample_values = vld1q_u8(samples);
    const uint8x8x2_t pairs = vuzp_u8(vget_low_u8(sample_values), vget_high_u8(sample_values));
    const uint8x8_t comparisons = vclt_u8(pairs.val[0], pairs.val[1]);
    descriptor[byte] = vaddv_u8(vand_u8(comparisons, bit_weights));
  }
}
#endif

}  // namespace

float OrbDescriptor::orientation(const cv::Mat& image, const cv::Point2f& point) {
#if defined(SIMD_NEON)
  return ic_angle_neon(image, point, orientation_extents);
#else
  int m_01 = 0, m_10 = 0;

  const uchar* const center = &image.at<uchar>(cvRound(point.y), cvRound(point.x));

  for (int u = -patch_size / 2; u <= patch_size / 2; ++u) {
    m_10 += u * center[u];
  }

  const auto step = static_cast<int>(image.step1());
  for (int v = 1; v <= patch_size / 2; ++v) {
    int v_sum = 0;
    const int d = orientation_extents.at(v);
    for (int u = -d; u <= d; ++u) {
      const int val_plus = center[u + v * step];
      const int val_minus = center[u - v * step];
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  return cv::fastAtan2(m_01, m_10);
#endif
}

void OrbDescriptor::write_descriptor(const cv::KeyPoint& keypoint, const cv::Mat& image, uchar* descriptor) {
#if defined(SIMD_NEON)
  compute_fast_orb_neon(keypoint, image, descriptor);
#elif defined(SIMD_SSE2)
  compute_fast_orb_sse2(keypoint, image, descriptor);
#else
  compute_fast_orb_scalar(keypoint, image, descriptor);
#endif
}

void OrbDescriptor::compute(const cv::_InputArray& in_image, std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors, const cv::Mat& mask, cv::Mat* descriptor_storage) {
  if (in_image.empty() || keypts.empty()) {
    if (in_image.empty()) {
      keypts.clear();
    }
    out_descriptors.release();
    orb_.pyramid_ready_for_descriptors_ = false;
    return;
  }

  const auto image = in_image.getMat();
  assert(image.type() == CV_8UC1);
  if (!orb_.can_reuse_pyramid(image)) {
    orb_.prepare_pyramid(image);
  }

  orb_.prepare_masks(mask, image.size());
  auto& filtered_keypoints = filtered_keypoints_;
  auto& local_keypoints = local_keypoints_;
  auto& levels = descriptor_levels_;
  auto& populated_levels = populated_levels_;
  filtered_keypoints.clear(); local_keypoints.clear(); levels.clear();
  populated_levels.assign(orb_.config_.orb_pyramid_levels, false);
  filtered_keypoints.reserve(keypts.size());
  local_keypoints.reserve(keypts.size());
  levels.reserve(keypts.size());

  for (const auto& keypoint : keypts) {
    if (keypoint.octave < 0 || keypoint.octave >= orb_.config_.orb_pyramid_levels) {
      continue;
    }

    const unsigned int level = static_cast<unsigned int>(keypoint.octave);
    cv::KeyPoint local_keypoint = keypoint;
    local_keypoint.pt *= orb_.inv_scale_factors_.at(level);
    const cv::Mat& level_image = orb_.image_pyramid_.at(level);
    if (local_keypoint.pt.x < static_cast<float>(support_radius) || local_keypoint.pt.y < static_cast<float>(support_radius) ||
        local_keypoint.pt.x >= static_cast<float>(level_image.cols - support_radius) ||
        local_keypoint.pt.y >= static_cast<float>(level_image.rows - support_radius)) {
      continue;
    }

    if (!orb_.level_masks_.empty() && orb_.level_masks_[level].at<uint8_t>(cvRound(local_keypoint.pt.y), cvRound(local_keypoint.pt.x))) continue;
    local_keypoint.angle = orientation(level_image, local_keypoint.pt);
    cv::KeyPoint output_keypoint = keypoint;
    output_keypoint.angle = local_keypoint.angle;
    filtered_keypoints.push_back(output_keypoint);
    local_keypoints.push_back(local_keypoint);
    levels.push_back(level);
    populated_levels[level] = true;
  }

  keypts.swap(filtered_keypoints);
  if (keypts.empty() || !out_descriptors.needed()) {
    out_descriptors.release();
    orb_.pyramid_ready_for_descriptors_ = false;
    return;
  }

  for (int level = 0; level < orb_.config_.orb_pyramid_levels; ++level) {
    if (populated_levels[level]) {
      orb_.image_pyramid_.blur_level(level);
    }
  }

  if (descriptor_storage) {
    if (descriptor_storage->rows < int(keypts.size()) || descriptor_storage->cols != 32 || descriptor_storage->type() != CV_8U)
      descriptor_storage->create(int(keypts.size()), 32, CV_8U);
    out_descriptors.assign(descriptor_storage->rowRange(0, int(keypts.size())));
  } else out_descriptors.create(static_cast<int>(keypts.size()), 32, CV_8U);
  cv::Mat descriptors = out_descriptors.getMat();
  for (std::size_t index = 0; index < local_keypoints.size(); ++index) {
    write_descriptor(local_keypoints[index], orb_.image_pyramid_.at(levels[index]), descriptors.ptr<uchar>(static_cast<int>(index)));
  }
  orb_.pyramid_ready_for_descriptors_ = false;
}
}  // namespace sapphire::backend::feature
