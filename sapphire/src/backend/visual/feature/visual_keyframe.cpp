#include "backend/visual/feature/visual_keyframe.hpp"
#include "backend/visual/feature/fast_detector.hpp"
#include "common/camera/camera.hpp"
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace sapphire {
namespace {
int cell(cv::Point2f p, cv::Size size) { return std::clamp(int(p.y * 6 / size.height), 0, 5) * 8 + std::clamp(int(p.x * 8 / size.width), 0, 7); }
bool inside(cv::Point2f p, cv::Size size) {
  return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 20 && p.y >= 20 && p.x < size.width - 20 && p.y < size.height - 20;
}
double coverage(const std::vector<cv::Point2f> &p, cv::Size size) {
  std::array<bool, 48> occupied{};
  for (auto x : p) occupied[cell(x, size)] = true;
  return double(std::count(occupied.begin(), occupied.end(), true)) / 48;
}
}  // namespace
void replenishVisualCorners(const cv::Mat &image, std::vector<cv::Point2f> &points, int limit) {
  std::array<int, 48> counts{};
  cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
  for (auto p : points) {
    ++counts[cell(p, image.size())];
    cv::circle(mask, p, 5, cv::Scalar(0), -1);
  }
  const int quota = (limit + 47) / 48;
  std::array<std::vector<std::uint8_t>, 3> scores;
  std::array<std::vector<int>, 3> corners;
  for (int k = 0; k < 48 && points.size() < std::size_t(limit); ++k) {
    if (counts[k] >= quota) continue;
    const int x0 = (k % 8) * image.cols / 8, y0 = (k / 8) * image.rows / 6;
    const cv::Rect roi(x0, y0, ((k % 8) + 1) * image.cols / 8 - x0, ((k / 8) + 1) * image.rows / 6 - y0);
    std::vector<cv::KeyPoint> fresh;
    if (roi.width < 7 || roi.height < 7) continue;
    visual::fast_detail::detect_9_16_cell_nms(image(roi), 15, scores, corners,
        cv::Rect(3, 3, roi.width - 6, roi.height - 6), roi.width, roi.height, 1,
        [&](int x, int y, int score, int) { fresh.emplace_back(float(x), float(y), 7.F, -1.F, float(score)); });
    std::stable_sort(fresh.begin(), fresh.end(), [](const auto &a, const auto &b) { return a.response > b.response; });
    for (const auto &key : fresh) {
      cv::Point2f p = key.pt + cv::Point2f(x0, y0);
      if (!inside(p, image.size()) || !mask.at<uchar>(cvRound(p.y), cvRound(p.x))) continue;
      points.push_back(p);
      cv::circle(mask, p, 5, cv::Scalar(0), -1);
      if (++counts[k] >= quota || points.size() >= std::size_t(limit)) break;
    }
  }
}
VisualKeyframeSelector::Decision VisualKeyframeSelector::evaluate(const ImageMeas &image, const std::optional<Eigen::Isometry3d> &pose) {
  const auto start = std::chrono::steady_clock::now();
  Decision d;
  if (!std::isfinite(image.timestamp) || image.timestamp <= last_time_ || image.camera_id != camera_id_ || image.gray.type() != CV_8UC1 ||
      image.gray.cols != config_.camera(camera_id_).width || image.gray.rows != config_.camera(camera_id_).height)
    throw std::invalid_argument("Invalid keyframe input");
  if (pose && (!pose->matrix().allFinite() || !pose->linear().isUnitary(1e-6) || std::abs(pose->linear().determinant() - 1) > 1e-6 ||
               (pose->matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9))
    throw std::invalid_argument("Invalid keyframe LIO pose");
  d.gap = last_time_ >= 0 && image.timestamp - last_time_ > .3;
  if (d.gap) {
    points_.clear();
    ids_.clear();
    pyramid_.clear();
  }  // keep accepted reference/time across tracking gaps
  std::vector<cv::Mat> current;
  cv::buildOpticalFlowPyramid(image.gray, current, {21, 21}, 3);
  if (!points_.empty()) {
    std::vector<cv::Point2f> next = points_, back = points_;
    std::vector<uchar> ok, reverse;
    std::vector<float> error, error_back;
    const auto term = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, .01);
    cv::calcOpticalFlowPyrLK(pyramid_, current, points_, next, ok, error, {21, 21}, 3, term, cv::OPTFLOW_USE_INITIAL_FLOW);
    cv::calcOpticalFlowPyrLK(current, pyramid_, next, back, reverse, error_back, {21, 21}, 3, term, cv::OPTFLOW_USE_INITIAL_FLOW);
    std::size_t write = 0;
    cv::Mat spacing(image.gray.size(), CV_8UC1, cv::Scalar(255));
    for (std::size_t i = 0; i < points_.size(); ++i)
      if (ok[i] && reverse[i] && inside(next[i], image.gray.size()) && error[i] <= 30 && error_back[i] <= 30 &&
          cv::norm(back[i] - points_[i]) <= .75 && spacing.at<uchar>(cvRound(next[i].y), cvRound(next[i].x))) {
        cv::circle(spacing, next[i], 5, cv::Scalar(0), -1);
        points_[write] = next[i];
        ids_[write] = ids_[i];
        ++write;
      }
    points_.resize(write);
    ids_.resize(write);
  }
  replenishVisualCorners(image.gray, points_, config_.max_features);
  while (ids_.size() < points_.size()) {
    if (next_id_ == UINT64_MAX) throw std::overflow_error("Visual track ID exhausted");
    ids_.push_back(next_id_++);
  }
  pyramid_ = std::move(current);
  last_time_ = image.timestamp;
  current_pose_ = pose;
  d.tracks = points_.size();
  d.coverage = coverage(points_, image.gray.size());
  for (auto id : ids_) d.shared += reference_.count(id);
  d.renewal = d.tracks ? 1. - double(d.shared) / d.tracks : 0;
  d.usable = d.tracks >= 40 && d.coverage >= config_.keyframe_min_coverage;
  d.seed = reference_time_ < 0;
  if (reference_pose_ && pose) {
    const Eigen::Isometry3d delta = reference_pose_->inverse() * (*pose);
    d.translation_m = delta.translation().norm();
    d.rotation_deg = Eigen::AngleAxisd(delta.linear()).angle() * 180. / std::acos(-1.);
    d.translation = d.translation_m > config_.keyframe_translation_m;
    d.rotation = d.rotation_deg > config_.keyframe_rotation_deg;
  }
  d.visual = d.renewal >= config_.keyframe_renewal;
  d.selected = d.usable && pose &&
               (d.seed || (image.timestamp - reference_time_ >= config_.keyframe_min_interval && (d.visual || d.translation || d.rotation)));
  selected_ = d.selected;
  d.flow_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return d;
}
void VisualKeyframeSelector::accept() {
  if (!selected_) throw std::logic_error("Cannot accept a non-keyframe");
  reference_.clear();
  reference_.insert(ids_.begin(), ids_.end());
  reference_time_ = last_time_;
  reference_pose_ = current_pose_;
  selected_ = false;
}
void VisualKeyframeSelector::reset() {
  points_.clear();
  ids_.clear();
  pyramid_.clear();
  reference_.clear();
  reference_pose_.reset();
  current_pose_.reset();
  last_time_ = reference_time_ = -1;
  selected_ = false;
}
void VisualKeyframeSelector::beginSubmap() {
  reference_.clear(); reference_pose_.reset(); reference_time_=-1; selected_=false;
}
VisualFrame makeImageAttribute(const ImageMeas &image, const CameraParameters &camera, const Eigen::Isometry3d &pose) {
  if(image.gray.type()!=CV_8UC1 || image.gray.cols!=camera.width || image.gray.rows!=camera.height ||
     image.gray.total()>VisualFrame::max_image_pixels) throw std::invalid_argument("Invalid image attribute input");
  cv::Mat rectified;
  const CameraModel model(camera);
  model.undistortImage(image.gray, rectified);
  const auto &K = model.matrix();
  std::vector<std::uint8_t> bytes;
  if(!cv::imencode(".png",rectified,bytes,{cv::IMWRITE_PNG_COMPRESSION,1})) throw std::runtime_error("Image attribute encoding failed");
  VisualFrame frame(image.timestamp,image.camera_id);
  frame.set_observation_pose(pose);
  frame.set_projection({camera.width,camera.height,{K.at<float>(0,0),K.at<float>(1,1),K.at<float>(0,2),K.at<float>(1,2)}});
  frame.set_image_png(std::move(bytes));
  return frame;
}
}  // namespace sapphire
