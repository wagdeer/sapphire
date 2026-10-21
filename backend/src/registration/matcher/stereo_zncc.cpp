#include <backend/registration/matcher/stereo_zncc.hpp>
#include <backend/sensor/camera/utils.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace sapphire::backend {
namespace {
// Frozen by the RK3588 stereo study: 9x9 ZNCC, 1/4-resolution global search,
// +/-3 refinement at 1/2 and full resolution, and a reverse search. A 0.05
// margin excludes other peaks outside +/-2 pixels at the coarse level.
constexpr int radius = 4;
constexpr float minimum_correlation = .85f, uniqueness_margin = .05f;
struct Patch {
  std::array<uint8_t, 81> data;
  int sum = 0, square_sum = 0;
  Patch(const cv::Mat& image, int x, int y) {
    int i = 0;
    for (int v = -radius; v <= radius; ++v)
      for (int u = -radius; u <= radius; ++u) {
        const int a = image.at<uint8_t>(y + v, x + u);
        data[i++] = a;
        sum += a;
        square_sum += a * a;
      }
  }
  float correlation(const cv::Mat& image, int x, int y) const {
    int total = 0, squares = 0, dot = 0, i = 0;
    for (int v = -radius; v <= radius; ++v) {
      const auto* b = image.ptr<uint8_t>(y + v) + x - radius;
      int u = 0;
#if defined(__aarch64__)
      const auto av = vld1_u8(data.data() + i), bv = vld1_u8(b);
      total += vaddlv_u8(bv);
      squares += vaddlvq_u16(vmull_u8(bv, bv));
      dot += vaddlvq_u16(vmull_u8(av, bv));
      u = 8;
#endif
      for (; u < 9; ++u) {
        total += b[u];
        squares += b[u] * b[u];
        dot += data[i + u] * b[u];
      }
      i += 9;
    }
    const double a_variance = 81.0 * square_sum - double(sum) * sum;
    const double b_variance = 81.0 * squares - double(total) * total;
    if (a_variance < 81 * 81 * 4 || b_variance < 81 * 81 * 4) return -1;
    return float((81.0 * dot - double(sum) * total) / std::sqrt(a_variance * b_variance));
  }
};
using Pyramid = std::array<cv::Mat, 3>;
bool excluded(const cv::Mat& mask, int x, int y) { return !mask.empty() && mask.at<uint8_t>(y, x) != 0; }

// Conservative mask propagation covers pyrDown's 5x5 support and then the
// whole correlation patch. A masked pixel cannot leak through a coarse match.
void prepare_masks(const cv::Mat& source, const Pyramid& images, Pyramid& output, cv::Mat& scratch) {
  if (source.empty()) {
    for (auto& mask : output) mask.release();
    return;
  }
  source.copyTo(output[0]);
  const auto filter = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
  for (int level = 1; level < 3; ++level) {
    cv::dilate(output[level - 1], scratch, filter, {-1, -1}, 1, cv::BORDER_CONSTANT, cv::Scalar(255));
    output[level].create(images[level].size(), CV_8U);
    for (int y = 0; y < output[level].rows; ++y)
      for (int x = 0; x < output[level].cols; ++x) output[level].at<uint8_t>(y, x) = scratch.at<uint8_t>(2 * y, 2 * x);
  }
  const auto support = cv::getStructuringElement(cv::MORPH_RECT, {9, 9});
  for (auto& mask : output) cv::dilate(mask, mask, support, {-1, -1}, 1, cv::BORDER_CONSTANT, cv::Scalar(255));
}
float search(const cv::Mat& a, const cv::Mat& b, const cv::Mat& am, const cv::Mat& bm,
             int x, int y, int low, int high, bool unique, std::vector<float>& scores) {
  if (x < 5 || y < 5 || x >= a.cols - 5 || y >= a.rows - 5 || excluded(am, x, y)) return NAN;
  low = std::max(5, low);
  high = std::min(b.cols - 6, high);
  if (high - low < 2) return NAN;
  const Patch patch(a, x, y);
  scores.resize(high - low + 1);
  float best = -2;
  int best_x = -1;
  for (int rx = low; rx <= high; ++rx) {
    const float value = excluded(bm, rx, y) ? -1 : patch.correlation(b, rx, y);
    scores[rx - low] = value;
    if (value > best) { best = value; best_x = rx; }
  }
  if (best < minimum_correlation || best_x == low || best_x == high) return NAN;
  // All three interpolation samples must have valid support and texture.
  const float previous = scores[best_x - low - 1], next = scores[best_x - low + 1];
  if (previous <= -1 || next <= -1) return NAN;
  if (unique) {
    float second = -1;
    for (int rx = low; rx <= high; ++rx)
      if (std::abs(rx - best_x) > 2) second = std::max(second, scores[rx - low]);
    if (best - second < uniqueness_margin) return NAN;
  }
  const float curvature = previous + next - 2 * best;
  if (!(curvature < -.0001f)) return NAN;
  const float delta = (previous - next) / (2 * curvature);
  if (!std::isfinite(delta) || std::abs(delta) > 1) return NAN;
  return best_x + delta;
}
float directional(const Pyramid& a, const Pyramid& b, const Pyramid& am, const Pyramid& bm,
                  int px, int py, int direction, float max_disparity, std::vector<float>& scores) {
  float found = NAN;
  for (int level = 2; level >= 0; --level) {
    const float scale = float(1 << level);
    const int x = cvRound(px / scale), y = cvRound(py / scale);
    int low, high;
    if (level == 2) {
      // Include a neighboring sample beyond each disparity endpoint so small
      // positive disparities can be interpolated instead of hitting a boundary.
      low = direction < 0 ? int(x - max_disparity / scale - 1) : x - 1;
      high = direction < 0 ? x + 1 : int(x + max_disparity / scale + 1);
    } else { low = cvRound(found * 2) - 3; high = cvRound(found * 2) + 3; }
    found = search(a[level], b[level], am[level], bm[level], x, y, low, high, level == 2, scores);
    if (!std::isfinite(found)) return NAN;
  }
  return found;
}
}  // namespace

std::vector<cv::Point3f> StereoZncc::match(const cv::Mat& left, const cv::Mat& right,
    const std::vector<cv::KeyPoint>& points, const camera::Perspective& camera, const cv::Mat& left_mask, const cv::Mat& right_mask) {
  std::vector<cv::Point3f> output(points.size(), {NAN, NAN, NAN});
  if (points.empty()) return output;
  if (left.empty() || left.type() != CV_8U || right.type() != CV_8U || left.size() != right.size() ||
      left.size() != camera.imageSize() || !camera.isValidForReprojection() || !camera.cameraToBase().isValid() ||
      !(camera.focalXBaseline() > 0) || !std::isfinite(camera.focalXBaseline()) || points.size() > 8192)
    throw std::invalid_argument("Invalid epipolar stereo images, features or calibration");
  camera::validateImageMask(left_mask, left.size());
  camera::validateImageMask(right_mask, right.size());
  Pyramid a, b;
  // Retain scratch allocations, never retain the caller's full-resolution frame.
  a[0] = left; b[0] = right;
  for (int level = 1; level < 3; ++level) {
    cv::pyrDown(a[level - 1], epipolar_left_[level]);
    cv::pyrDown(b[level - 1], epipolar_right_[level]);
    a[level] = epipolar_left_[level]; b[level] = epipolar_right_[level];
  }
  prepare_masks(left_mask, a, epipolar_left_mask_, epipolar_mask_workspace_);
  prepare_masks(right_mask, b, epipolar_right_mask_, epipolar_mask_workspace_);
  const float maximum = float(std::min(double(left.cols), camera.fx()));
  for (size_t i = 0; i < points.size(); ++i) {
    const auto p = points[i].pt;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 5 || p.y < 5 || p.x >= left.cols - 5 || p.y >= left.rows - 5) continue;
    const int x = cvRound(p.x), y = cvRound(p.y);
    const float rx = directional(a, b, epipolar_left_mask_, epipolar_right_mask_, x, y, -1, maximum, epipolar_scores_);
    if (!std::isfinite(rx)) continue;
    const float back = directional(b, a, epipolar_right_mask_, epipolar_left_mask_, cvRound(rx), y, 1, maximum, epipolar_scores_);
    if (!std::isfinite(back) || std::abs(back - x) > 1) continue;
    // Both patches use the same fractional offset. Integer centering cancels
    // rather than adding the left detector's fractional pixel to disparity.
    const float disparity = x - rx;
    if (!(disparity > .5f && disparity < maximum)) continue;
    const auto optical = camera.backProject(p, camera.focalXBaseline() / disparity);
    if (!optical) continue;
    const auto body = camera.cameraToBase().transformPoint({float(optical->x), float(optical->y), float(optical->z)});
    if (body.allFinite()) output[i] = {body.x(), body.y(), body.z()};
  }
  return output;
}
}  // namespace sapphire::backend
