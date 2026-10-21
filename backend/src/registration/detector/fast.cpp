#include <backend/registration/detector/fast.hpp>
#include <backend/registration/descriptor/orb.hpp>
#include <backend/registration/fast_orb.hpp>
#include <backend/tools/simd_sse_neon.hpp>
#include <limits>
#include <backend/registration/detector/fast/fast_9_16_detector.h>
#include <backend/registration/detector/fast/fast_9_16_score.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
namespace sapphire::backend::feature {
namespace {
constexpr unsigned int harris_block_size = 7;

float harris_response(const int a, const int b, const int c, const float harris_k, const float scale_sq_sq) {
  const float trace = static_cast<float>(a + b);
  return (static_cast<float>(a) * static_cast<float>(b) - static_cast<float>(c) * static_cast<float>(c) - harris_k * trace * trace) * scale_sq_sq;
}

bool harris_patch_is_inside(const cv::Mat& image, const int x0, const int y0) {
  return x0 >= 1 && y0 >= 1 && x0 + static_cast<int>(harris_block_size) < image.cols && y0 + static_cast<int>(harris_block_size) < image.rows;
}

[[maybe_unused]] void score_by_harris_scalar(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Point2i& keypoint_offset,
                                             const float harris_k) {
  const int step = static_cast<int>(image.step1());
  const int radius = static_cast<int>(harris_block_size / 2);
  const float scale = 1.0F / static_cast<float>(4U * harris_block_size * 255U);
  const float scale_sq = scale * scale;
  const float scale_sq_sq = scale_sq * scale_sq;
  std::array<int, harris_block_size * harris_block_size> offsets{};
  for (unsigned int row = 0; row < harris_block_size; ++row) {
    for (unsigned int column = 0; column < harris_block_size; ++column) {
      offsets[row * harris_block_size + column] = static_cast<int>(row) * step + static_cast<int>(column);
    }
  }

  const uchar* image_data = image.ptr<uchar>();
  for (auto& keypoint : keypoints) {
    const int x0 = cvRound(keypoint.pt.x) + keypoint_offset.x - radius;
    const int y0 = cvRound(keypoint.pt.y) + keypoint_offset.y - radius;
    if (!harris_patch_is_inside(image, x0, y0)) {
      keypoint.response = std::numeric_limits<float>::lowest();
      continue;
    }

    const uchar* patch = image_data + y0 * step + x0;
    int a = 0;
    int b = 0;
    int c = 0;
    for (unsigned int index = 0; index < harris_block_size * harris_block_size; ++index) {
      const uchar* pixel = patch + offsets[index];
      const int ix = (pixel[1] - pixel[-1]) * 2 + (pixel[-step + 1] - pixel[-step - 1]) + (pixel[step + 1] - pixel[step - 1]);
      const int iy = (pixel[step] - pixel[-step]) * 2 + (pixel[step - 1] - pixel[-step - 1]) + (pixel[step + 1] - pixel[-step + 1]);
      a += ix * ix;
      b += iy * iy;
      c += ix * iy;
    }
    keypoint.response = harris_response(a, b, c, harris_k, scale_sq_sq);
  }
}

#if defined(SIMD_SSE2)
struct HarrisRowSse2 {
  __m128i left;
  __m128i middle;
};

HarrisRowSse2 load_harris_row_sse2(const uchar* row) {
  const __m128i zero = _mm_setzero_si128();
  return {_mm_unpacklo_epi8(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row - 1)), zero),
          _mm_unpacklo_epi8(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(row)), zero)};
}

void accumulate_harris_row_7_sse2(const HarrisRowSse2& top, const HarrisRowSse2& middle, const HarrisRowSse2& bottom, __m128i& a, __m128i& b, __m128i& c) {
  const __m128i top_right = _mm_srli_si128(top.middle, 2);
  const __m128i middle_right = _mm_srli_si128(middle.middle, 2);
  const __m128i bottom_right = _mm_srli_si128(bottom.middle, 2);

  __m128i ix = _mm_add_epi16(_mm_add_epi16(_mm_sub_epi16(top_right, top.left), _mm_sub_epi16(bottom_right, bottom.left)),
                             _mm_slli_epi16(_mm_sub_epi16(middle_right, middle.left), 1));
  __m128i iy = _mm_add_epi16(_mm_add_epi16(_mm_sub_epi16(bottom.left, top.left), _mm_sub_epi16(bottom_right, top_right)),
                             _mm_slli_epi16(_mm_sub_epi16(bottom.middle, top.middle), 1));

  const __m128i valid_lanes = _mm_set_epi16(0, -1, -1, -1, -1, -1, -1, -1);
  ix = _mm_and_si128(ix, valid_lanes);
  iy = _mm_and_si128(iy, valid_lanes);
  a = _mm_add_epi32(a, _mm_madd_epi16(ix, ix));
  b = _mm_add_epi32(b, _mm_madd_epi16(iy, iy));
  c = _mm_add_epi32(c, _mm_madd_epi16(ix, iy));
}

int horizontal_sum_int32_sse2(__m128i value) {
  value = _mm_add_epi32(value, _mm_shuffle_epi32(value, _MM_SHUFFLE(1, 0, 3, 2)));
  value = _mm_add_epi32(value, _mm_shuffle_epi32(value, _MM_SHUFFLE(2, 3, 0, 1)));
  return _mm_cvtsi128_si32(value);
}

void score_by_harris_sse2_7(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Point2i& keypoint_offset, const float harris_k) {
  constexpr int radius = harris_block_size / 2;
  const int step = static_cast<int>(image.step1());
  const float scale = 1.0F / static_cast<float>(4U * harris_block_size * 255U);
  const float scale_sq = scale * scale;
  const float scale_sq_sq = scale_sq * scale_sq;
  const uchar* image_data = image.ptr<uchar>();

  for (auto& keypoint : keypoints) {
    const int x0 = cvRound(keypoint.pt.x) + keypoint_offset.x - radius;
    const int y0 = cvRound(keypoint.pt.y) + keypoint_offset.y - radius;
    if (!harris_patch_is_inside(image, x0, y0)) {
      keypoint.response = std::numeric_limits<float>::lowest();
      continue;
    }

    const uchar* patch = image_data + y0 * step + x0;
    HarrisRowSse2 top = load_harris_row_sse2(patch - step);
    HarrisRowSse2 middle = load_harris_row_sse2(patch);
    __m128i a = _mm_setzero_si128();
    __m128i b = _mm_setzero_si128();
    __m128i c = _mm_setzero_si128();
    for (unsigned int row = 0; row < harris_block_size; ++row, patch += step) {
      const HarrisRowSse2 bottom = load_harris_row_sse2(patch + step);
      accumulate_harris_row_7_sse2(top, middle, bottom, a, b, c);
      top = middle;
      middle = bottom;
    }
    keypoint.response = harris_response(horizontal_sum_int32_sse2(a), horizontal_sum_int32_sse2(b), horizontal_sum_int32_sse2(c), harris_k, scale_sq_sq);
  }
}
#endif

#if defined(SIMD_NEON)
void accumulate_harris_row_7(const uint8x16_t top_row, const uint8x16_t middle_row, const uint8x16_t bottom_row, int32x4_t& a, int32x4_t& b, int32x4_t& c) {
  const uint8x16_t zero = vdupq_n_u8(0);
  const int16x8_t vertical_left = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(bottom_row), vget_low_u8(top_row)));
  const int16x8_t vertical_middle = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(vextq_u8(bottom_row, zero, 1)), vget_low_u8(vextq_u8(top_row, zero, 1))));
  const int16x8_t vertical_right = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(vextq_u8(bottom_row, zero, 2)), vget_low_u8(vextq_u8(top_row, zero, 2))));
  int16x8_t iy = vaddq_s16(vaddq_s16(vertical_left, vertical_right), vmulq_n_s16(vertical_middle, 2));

  const int16x8_t horizontal_top = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(vextq_u8(top_row, zero, 2)), vget_low_u8(top_row)));
  const int16x8_t horizontal_middle = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(vextq_u8(middle_row, zero, 2)), vget_low_u8(middle_row)));
  const int16x8_t horizontal_bottom = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(vextq_u8(bottom_row, zero, 2)), vget_low_u8(bottom_row)));
  int16x8_t ix = vaddq_s16(vaddq_s16(horizontal_top, horizontal_bottom), vmulq_n_s16(horizontal_middle, 2));

  ix = vsetq_lane_s16(0, ix, 7);
  iy = vsetq_lane_s16(0, iy, 7);
  a = vaddq_s32(a, vaddq_s32(vmull_s16(vget_low_s16(ix), vget_low_s16(ix)), vmull_s16(vget_high_s16(ix), vget_high_s16(ix))));
  b = vaddq_s32(b, vaddq_s32(vmull_s16(vget_low_s16(iy), vget_low_s16(iy)), vmull_s16(vget_high_s16(iy), vget_high_s16(iy))));
  c = vaddq_s32(c, vaddq_s32(vmull_s16(vget_low_s16(ix), vget_low_s16(iy)), vmull_s16(vget_high_s16(ix), vget_high_s16(iy))));
}

void score_by_harris_neon_7(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Point2i& keypoint_offset, const float harris_k) {
  constexpr int radius = harris_block_size / 2;
  const int step = static_cast<int>(image.step1());
  const float scale = 1.0F / static_cast<float>(4U * harris_block_size * 255U);
  const float scale_sq = scale * scale;
  const float scale_sq_sq = scale_sq * scale_sq;
  const uchar* image_data = image.ptr<uchar>();

  for (auto& keypoint : keypoints) {
    const int x0 = cvRound(keypoint.pt.x) + keypoint_offset.x - radius;
    const int y0 = cvRound(keypoint.pt.y) + keypoint_offset.y - radius;
    if (!harris_patch_is_inside(image, x0, y0)) {
      keypoint.response = std::numeric_limits<float>::lowest();
      continue;
    }

    const uchar* patch = image_data + y0 * step + x0;
    uint8x16_t top_row = vld1q_u8(patch - step - 1);
    uint8x16_t middle_row = vld1q_u8(patch - 1);
    int32x4_t a = vdupq_n_s32(0);
    int32x4_t b = vdupq_n_s32(0);
    int32x4_t c = vdupq_n_s32(0);
    for (unsigned int row = 0; row < harris_block_size; ++row, patch += step) {
      const uint8x16_t bottom_row = vld1q_u8(patch + step - 1);
      accumulate_harris_row_7(top_row, middle_row, bottom_row, a, b, c);
      top_row = middle_row;
      middle_row = bottom_row;
    }
    keypoint.response = harris_response(vaddvq_s32(a), vaddvq_s32(b), vaddvq_s32(c), harris_k, scale_sq_sq);
  }
}
#endif

void score_by_harris(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Point2i& keypoint_offset, const float harris_k) {
  CV_Assert(image.type() == CV_8UC1);
#if defined(SIMD_NEON)
  score_by_harris_neon_7(image, keypoints, keypoint_offset, harris_k);
#elif defined(SIMD_SSE2)
  score_by_harris_sse2_7(image, keypoints, keypoint_offset, harris_k);
#else
  score_by_harris_scalar(image, keypoints, keypoint_offset, harris_k);
#endif
}

bool is_cell_local_fast_maximum(const cv::Mat& scores, const cv::Rect& core, const int x, const int y) {
  const int score = scores.ptr<std::uint8_t>(y)[x];
  const int right = core.x + core.width - 1;
  const int bottom = core.y + core.height - 1;
  if (x > core.x && x < right && y > core.y && y < bottom) {
    const auto* previous = scores.ptr<std::uint8_t>(y - 1);
    const auto* current = scores.ptr<std::uint8_t>(y);
    const auto* next = scores.ptr<std::uint8_t>(y + 1);
    return score > previous[x - 1] && score > previous[x] && score > previous[x + 1] && score > current[x - 1] && score > current[x + 1] &&
           score > next[x - 1] && score > next[x] && score > next[x + 1];
  }
  const int first_y = std::max(y - 1, core.y);
  const int last_y = std::min(y + 1, bottom);
  const int first_x = std::max(x - 1, core.x);
  const int last_x = std::min(x + 1, right);
  for (int neighbor_y = first_y; neighbor_y <= last_y; ++neighbor_y) {
    const auto* row = scores.ptr<std::uint8_t>(neighbor_y);
    for (int neighbor_x = first_x; neighbor_x <= last_x; ++neighbor_x) {
      if ((neighbor_x != x || neighbor_y != y) && score <= row[neighbor_x]) {
        return false;
      }
    }
  }
  return true;
}
}
FastDetector::FastDetector(FastOrb& orb)
    : orb_(orb), features_per_level_(orb.config_.orb_pyramid_levels),
      level_workspaces_(orb.config_.orb_pyramid_levels), all_keypoints_workspace_(orb.config_.orb_pyramid_levels) {
  const unsigned int feature_budget = std::max(1, orb_.config_.max_features);
  if (orb_.config_.orb_pyramid_levels == 1) {
    features_per_level_[0] = feature_budget;
    return;
  }

  const float factor = 1.0F / orb_.config_.orb_pyramid_scale;
  float desired_features = feature_budget * (1.0F - factor) /
                           (1.0F - static_cast<float>(std::pow(static_cast<double>(factor), static_cast<double>(orb_.config_.orb_pyramid_levels))));
  unsigned int allocated_features = 0;
  for (int level = 0; level + 1 < orb_.config_.orb_pyramid_levels; ++level) {
    features_per_level_[level] = static_cast<unsigned int>(std::max(cvRound(desired_features), 0));
    allocated_features += features_per_level_[level];
    desired_features *= factor;
  }
  features_per_level_.back() = allocated_features < feature_budget ? feature_budget - allocated_features : 0;
}

void FastDetector::detect(const cv::_InputArray& in_image, std::vector<cv::KeyPoint>& keypts, const cv::Mat& mask) {
  if (in_image.empty()) {
    keypts.clear();
    orb_.pyramid_ready_for_descriptors_ = false;
    return;
  }

  const auto image = in_image.getMat();
  assert(image.type() == CV_8UC1);

  orb_.prepare_pyramid(image);
  orb_.prepare_masks(mask, image.size());
  auto& all_keypts = all_keypoints_workspace_;
  compute_harris_keypoints(all_keypts);

  keypts.clear();
  compute_orientations(all_keypts);
  std::size_t num_keypoints = 0;
  for (int level = 0; level < orb_.config_.orb_pyramid_levels; ++level) {
    correct_keypoint_scale(all_keypts[level], level);
    num_keypoints += all_keypts[level].size();
  }

  keypts.reserve(num_keypoints);
  for (int level = 0; level < orb_.config_.orb_pyramid_levels; ++level) {
    auto& keypts_at_level = all_keypts.at(level);
    keypts.insert(keypts.end(), keypts_at_level.begin(), keypts_at_level.end());
  }
}

void FastDetector::compute_harris_keypoints(std::vector<std::vector<cv::KeyPoint>>& all_keypts) const {
  all_keypts.resize(orb_.config_.orb_pyramid_levels);
  for (auto& keypoints : all_keypts) {
    keypoints.clear();
  }

  constexpr int cell_width = 25;
  constexpr int cell_height = 25;
  constexpr int fast_border = 3;
  const auto process_level = [&](const unsigned int level) {
    auto& workspace = level_workspaces_.at(level);
    auto& level_keypoints = all_keypts.at(level);
    const cv::Mat& level_image = orb_.image_pyramid_.at(level);
    const auto excluded = [&](int x, int y) {
      return !orb_.level_masks_.empty() && orb_.level_masks_[level].at<uint8_t>(y, x) != 0;
    };
    const unsigned int desired_features = features_per_level_.at(level);
    constexpr int minimum_pyramid_extent = 2 * static_cast<int>(OrbDescriptor::support_radius);
    if (desired_features == 0 || level_image.cols <= minimum_pyramid_extent || level_image.rows <= minimum_pyramid_extent) {
      return;
    }

    const int min_border_x = OrbDescriptor::support_radius;
    const int min_border_y = OrbDescriptor::support_radius;
    const int max_border_x = level_image.cols - OrbDescriptor::support_radius;
    const int max_border_y = level_image.rows - OrbDescriptor::support_radius;
    const int width = max_border_x - min_border_x;
    const int height = max_border_y - min_border_y;
    const int num_cols = static_cast<int>(std::ceil(static_cast<float>(width) / cell_width));
    const int num_rows = static_cast<int>(std::ceil(static_cast<float>(height) / cell_height));
    const int num_cells = num_cols * num_rows;
    const int features_per_cell = static_cast<int>(std::ceil(static_cast<float>(desired_features) / static_cast<float>(num_cells)));

    workspace.cell_keypoints.resize(num_cells);
    workspace.cell_bounds.resize(num_cells);
    workspace.cell_to_retain.assign(num_cells, 0);
    workspace.cell_total.assign(num_cells, 0);
    workspace.cell_exhausted.assign(num_cells, 0);
    for (int cell_index = 0; cell_index < num_cells; ++cell_index) {
      auto& cell_keypoints = workspace.cell_keypoints[cell_index];
      cell_keypoints.clear();
      cell_keypoints.reserve(static_cast<std::size_t>(features_per_cell) * 5U);
      workspace.cell_bounds[cell_index] = cv::Rect();
    }

    for (int row = 0; row < num_rows; ++row) {
      const int initial_y = min_border_y + row * cell_height - fast_border;
      int cell_roi_height = cell_height + 2 * fast_border;
      if (row == num_rows - 1) {
        cell_roi_height = max_border_y + fast_border - initial_y;
      }
      if (cell_roi_height <= 0) {
        continue;
      }
      for (int column = 0; column < num_cols; ++column) {
        const int initial_x = min_border_x + column * cell_width - fast_border;
        int cell_roi_width = cell_width + 2 * fast_border;
        if (column == num_cols - 1) {
          cell_roi_width = max_border_x + fast_border - initial_x;
        }
        if (cell_roi_width <= 0) {
          continue;
        }

        const int cell_index = row * num_cols + column;
        workspace.cell_bounds[cell_index] = cv::Rect(initial_x, initial_y, cell_roi_width, cell_roi_height);
      }
    }

    const cv::Rect fast_detection_roi(min_border_x - fast_border, min_border_y - fast_border, width + 2 * fast_border, height + 2 * fast_border);
    const int high_fast_threshold = orb_.config_.fast_threshold;
    const int low_fast_threshold = orb_.config_.fast_secondary_threshold;
    auto& fast_scores = workspace.harris_fast_scores;
    const auto fast_offsets = fast_detail::make_offsets(static_cast<int>(level_image.step));

#if defined(SIMD_NEON)
    const auto emit_primary_keypoint = [&](const int local_x, const int local_y, const int score, const int cell_index) {
      const cv::Rect& bounds = workspace.cell_bounds[cell_index];
      const int x = local_x + fast_detection_roi.x;
      const int y = local_y + fast_detection_roi.y;
      if (excluded(x, y)) return;
      cv::KeyPoint keypoint(static_cast<float>(x - bounds.x), static_cast<float>(y - bounds.y), 7.0F, -1.0F, static_cast<float>(score));
      workspace.cell_keypoints[cell_index].push_back(keypoint);
    };
    fast_detail::detect_9_16_cell_nms(level_image(fast_detection_roi), high_fast_threshold, workspace.primary_fast_score_rows,
                                      workspace.primary_fast_corner_positions, cv::Rect(fast_border, fast_border, width, height), cell_width, cell_height,
                                      num_cols, emit_primary_keypoint);
#else
    auto& raw_keypoints = workspace.harris_raw_keypoints;
    raw_keypoints.clear();
    cv::FAST(level_image(fast_detection_roi), raw_keypoints, high_fast_threshold, false, cv::FastFeatureDetector::TYPE_9_16);
    fast_scores.create(level_image.size(), CV_8UC1);
    fast_scores.setTo(0);
    for (auto keypoint : raw_keypoints) {
      keypoint.pt.x += static_cast<float>(fast_detection_roi.x);
      keypoint.pt.y += static_cast<float>(fast_detection_roi.y);
      const int x = cvRound(keypoint.pt.x);
      const int y = cvRound(keypoint.pt.y);
      if (x < min_border_x || x >= max_border_x || y < min_border_y || y >= max_border_y) {
        continue;
      }
      if (excluded(x, y)) continue;
      const int column = (x - min_border_x) / cell_width;
      const int row = (y - min_border_y) / cell_height;
      const int cell_index = row * num_cols + column;
      const cv::Rect& bounds = workspace.cell_bounds[cell_index];
      const int score = fast_detail::score(level_image.ptr<std::uint8_t>(y) + x, fast_offsets, high_fast_threshold);
      fast_scores.ptr<std::uint8_t>(y)[x] = static_cast<std::uint8_t>(score);
      keypoint.pt.x -= static_cast<float>(bounds.x);
      keypoint.pt.y -= static_cast<float>(bounds.y);
      keypoint.response = static_cast<float>(score);
      workspace.cell_keypoints[cell_index].push_back(keypoint);
    }
#endif

    const auto apply_fast_nms = [&](std::vector<cv::KeyPoint>& cell_keypoints, const cv::Rect& bounds, const cv::Rect& core) {
      cell_keypoints.erase(std::remove_if(cell_keypoints.begin(), cell_keypoints.end(),
                                          [&](const cv::KeyPoint& keypoint) {
                                            const int x = cvRound(keypoint.pt.x) + bounds.x;
                                            const int y = cvRound(keypoint.pt.y) + bounds.y;
                                            return !is_cell_local_fast_maximum(fast_scores, core, x, y);
                                          }),
                           cell_keypoints.end());
    };

    int num_exhausted_cells = 0;
    int num_to_redistribute = 0;
    for (int cell_index = 0; cell_index < num_cells; ++cell_index) {
      const cv::Rect& bounds = workspace.cell_bounds[cell_index];
      const cv::Rect core(bounds.x + fast_border, bounds.y + fast_border, bounds.width - 2 * fast_border, bounds.height - 2 * fast_border);
      auto& cell_keypoints = workspace.cell_keypoints[cell_index];

#if !defined(SIMD_NEON)
      apply_fast_nms(cell_keypoints, bounds, core);
#endif
      if (static_cast<int>(cell_keypoints.size()) < features_per_cell && orb_.config_.fast_secondary_detection_enabled && low_fast_threshold < high_fast_threshold) {
        auto& fallback_keypoints = workspace.harris_fallback_keypoints;
        fallback_keypoints.clear();
        cv::FAST(level_image(bounds), fallback_keypoints, low_fast_threshold, false, cv::FastFeatureDetector::TYPE_9_16);
        cell_keypoints.clear();
        cell_keypoints.reserve(fallback_keypoints.size());
        fast_scores.create(level_image.size(), CV_8UC1);
        fast_scores(bounds).setTo(0);
        for (auto keypoint : fallback_keypoints) {
          const int x = cvRound(keypoint.pt.x) + bounds.x;
          const int y = cvRound(keypoint.pt.y) + bounds.y;
          if (!core.contains(cv::Point(x, y)) || excluded(x, y)) {
            continue;
          }
          const int score = fast_detail::score(level_image.ptr<std::uint8_t>(y) + x, fast_offsets, low_fast_threshold);
          fast_scores.ptr<std::uint8_t>(y)[x] = static_cast<std::uint8_t>(score);
          keypoint.response = static_cast<float>(score);
          cell_keypoints.push_back(keypoint);
        }
        apply_fast_nms(cell_keypoints, bounds, core);
      }

      score_by_harris(level_image, cell_keypoints, bounds.tl(), orb_.config_.orb_harris_k);

      const int num_keys = static_cast<int>(cell_keypoints.size());
      workspace.cell_total[cell_index] = num_keys;
      if (num_keys > features_per_cell) {
        workspace.cell_to_retain[cell_index] = features_per_cell;
      } else {
        workspace.cell_to_retain[cell_index] = num_keys;
        num_to_redistribute += features_per_cell - num_keys;
        workspace.cell_exhausted[cell_index] = 1;
        ++num_exhausted_cells;
      }
    }

    while (num_to_redistribute > 0 && num_exhausted_cells < num_cells) {
      const int new_features_per_cell =
          features_per_cell + static_cast<int>(std::ceil(static_cast<float>(num_to_redistribute) / static_cast<float>(num_cells - num_exhausted_cells)));
      num_to_redistribute = 0;
      for (int cell_index = 0; cell_index < num_cells; ++cell_index) {
        if (workspace.cell_exhausted[cell_index] != 0) {
          continue;
        }
        if (workspace.cell_total[cell_index] > new_features_per_cell) {
          workspace.cell_to_retain[cell_index] = new_features_per_cell;
        } else {
          workspace.cell_to_retain[cell_index] = workspace.cell_total[cell_index];
          num_to_redistribute += new_features_per_cell - workspace.cell_total[cell_index];
          workspace.cell_exhausted[cell_index] = 1;
          ++num_exhausted_cells;
        }
      }
    }

    level_keypoints.reserve(static_cast<std::size_t>(desired_features) * 2U);
    const unsigned int scaled_patch_size = static_cast<unsigned int>(OrbDescriptor::patch_size * orb_.scale_factors_.at(level));
    for (int cell_index = 0; cell_index < num_cells; ++cell_index) {
      auto& cell_keypoints = workspace.cell_keypoints[cell_index];
      const int num_to_retain = workspace.cell_to_retain[cell_index];
      if (num_to_retain <= 0) {
        cell_keypoints.clear();
        continue;
      }
      cv::KeyPointsFilter::retainBest(cell_keypoints, num_to_retain);
      if (static_cast<int>(cell_keypoints.size()) > num_to_retain) {
        cell_keypoints.resize(num_to_retain);
      }
      const cv::Point offset = workspace.cell_bounds[cell_index].tl();
      for (auto& keypoint : cell_keypoints) {
        keypoint.pt.x += static_cast<float>(offset.x);
        keypoint.pt.y += static_cast<float>(offset.y);
        keypoint.octave = static_cast<int>(level);
        keypoint.size = static_cast<float>(scaled_patch_size);
        level_keypoints.push_back(keypoint);
      }
    }
    if (level_keypoints.size() > desired_features) {
      cv::KeyPointsFilter::retainBest(level_keypoints, static_cast<int>(desired_features));
      level_keypoints.resize(desired_features);
    }
  };

  for (int level = 0; level < orb_.config_.orb_pyramid_levels; ++level) {
    process_level(level);
  }
}

void FastDetector::compute_orientations(std::vector<std::vector<cv::KeyPoint>>& all_keypts) const {
  for (std::size_t level = 0; level < all_keypts.size(); ++level) {
    auto& keypts_at_level = all_keypts[level];
    const cv::Mat& level_image = orb_.image_pyramid_.at(level);
    for (auto& keypoint : keypts_at_level) {
      keypoint.angle = OrbDescriptor::orientation(level_image, keypoint.pt);
    }
  }
}

void FastDetector::correct_keypoint_scale(std::vector<cv::KeyPoint>& keypts_at_level, const unsigned int level) const {
  if (level == 0) {
    return;
  }
  const float scale_at_level = orb_.scale_factors_.at(level);
  for (auto& keypt_at_level : keypts_at_level) {
    keypt_at_level.pt *= scale_at_level;
  }
}
}
