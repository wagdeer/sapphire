#include "backend/visual/feature/visual_features.hpp"
#include "backend/visual/solver/visual_stereo.hpp"

#include <chrono>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "common/camera/camera.hpp"

namespace sapphire {
namespace {
constexpr double sync_tolerance = .002, max_error = 1.5;
bool rigid(const Eigen::Isometry3d &t) {
  return t.matrix().allFinite() && (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(t.linear().determinant() - 1) < 1e-6 && (t.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
}
Eigen::Vector2d ray(const VisualPoint &p, const CameraParameters &c) {
  return {(p.pixel().x() - c.intrinsics[2]) / c.intrinsics[0], (p.pixel().y() - c.intrinsics[3]) / c.intrinsics[1]};
}
bool epipolar(const Eigen::Vector2d &l, const Eigen::Vector2d &r, const Eigen::Isometry3d &t, const CameraParameters &lc,
              const CameraParameters &rc) {
  const Eigen::Vector3d a(l.x(), l.y(), 1), b(r.x(), r.y(), 1);
  const Eigen::Vector3d line_l = t.translation().cross(t.linear() * b);
  const Eigen::Vector3d line_r = t.linear().transpose() * a.cross(t.translation());
  const double e = std::abs(a.dot(line_l));
  const double dl = std::hypot(line_l.x() / lc.intrinsics[0], line_l.y() / lc.intrinsics[1]);
  const double dr = std::hypot(line_r.x() / rc.intrinsics[0], line_r.y() / rc.intrinsics[1]);
  return dl > 1e-12 && dr > 1e-12 && e <= max_error * dl && e <= max_error * dr;
}
std::vector<cv::DMatch> mutual(const cv::Mat &a, const cv::Mat &b, const cv::Mat &mask = {}) {
  if (a.rows < 2 || b.rows < 2) return {};
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> ab, ba;
  matcher.knnMatch(a, b, ab, 2, mask);
  cv::Mat reverse;
  if (!mask.empty()) cv::transpose(mask, reverse);
  matcher.knnMatch(b, a, ba, 2, reverse);
  const auto good = [](const auto &v) { return v.size() == 2 && v[0].distance <= 64 && v[0].distance < .8f * v[1].distance; };
  std::vector<cv::DMatch> result;
  for (const auto &v : ab)
    if (good(v)) {
      const auto &m = v[0];
      const auto &back = ba.at(m.trainIdx);
      if (good(back) && back[0].trainIdx == m.queryIdx) result.push_back(m);
    }
  return result;
}
int cell(const VisualPoint &p, const CameraParameters &c) {
  return std::clamp(int(p.pixel().y() * 6 / c.height), 0, 5) * 8 + std::clamp(int(p.pixel().x() * 8 / c.width), 0, 7);
}
bool usable(const VisualFrame &f, const CameraParameters &c) {
  std::array<bool, 48> cells{};
  for (const auto &p : f.points()) cells[cell(p, c)] = true;
  return f.points().size() >= 40 && std::count(cells.begin(), cells.end(), true) >= 17;
}
VisualFrame copy(const VisualFrame &f) {
  VisualFrame result(f.timestamp(), f.camera_id());
  result.update_features(f.points(), f.descriptors().clone());
  if (f.projection()) result.set_projection(*f.projection());
  if (f.T_odom_camera()) result.set_observation_pose(*f.T_odom_camera());
  return result;
}
}  // namespace
std::optional<VisualGeometry> triangulateStereo(const Eigen::Vector2d &l, const Eigen::Vector2d &r, const Eigen::Isometry3d &t,
                                                const CameraParameters &lc, const CameraParameters &rc) {
  if (!l.allFinite() || !r.allFinite() || !rigid(t) || t.translation().norm() < 1e-4) return {};
  if (lc.intrinsics.size() != 4 || rc.intrinsics.size() != 4) return {};
  for (const auto *c : {&lc, &rc})
    for (double v : c->intrinsics)
      if (!std::isfinite(v)) return {};
  if (lc.intrinsics[0] <= 0 || lc.intrinsics[1] <= 0 || rc.intrinsics[0] <= 0 || rc.intrinsics[1] <= 0) return {};
  if (!epipolar(l, r, t, lc, rc)) return {};
  const Eigen::Vector3d a = Eigen::Vector3d(l.x(), l.y(), 1).normalized();
  const Eigen::Vector3d b = t.linear() * Eigen::Vector3d(r.x(), r.y(), 1).normalized();
  const double cosine = std::clamp(a.dot(b), -1., 1.);
  const double sine = a.cross(b).norm(), angle = std::atan2(sine, cosine), det = sine * sine;
  if (cosine <= 0 || det <= 1e-16) return {};  // numerical parallelism, not a distance cap
  const double at = a.dot(t.translation()), bt = b.dot(t.translation());
  const double u = (at - cosine * bt) / det, v = (cosine * at - bt) / det;
  if (u <= 0 || v <= 0) return {};
  const Eigen::Vector3d xyz = .5 * (u * a + t.translation() + v * b), other = t.inverse() * xyz;
  if (!xyz.allFinite() || !other.allFinite() || xyz.z() <= 0 || other.z() <= 0) return {};
  const double e0 = ((xyz.head<2>() / xyz.z() - l).cwiseProduct(Eigen::Vector2d(lc.intrinsics[0], lc.intrinsics[1]))).norm();
  const double e1 = ((other.head<2>() / other.z() - r).cwiseProduct(Eigen::Vector2d(rc.intrinsics[0], rc.intrinsics[1]))).norm();
  if (std::max(e0, e1) > max_error) return {};
  return VisualGeometry{xyz, angle, std::max(e0, e1), 2, VisualDepthSource::Stereo};
}
StereoVisualProcessor::StereoVisualProcessor(VisualLoopParameters c) : config_(std::move(c)), selector_(config_) {
  SapphireParameters p;
  p.pose_graph.visual = config_;
  validate_parameters(p);
  if (!config_.enabled || !config_.stereo_depth) throw std::invalid_argument("Stereo processor requires explicit stereo depth");
  // Matching, rectification and archived pixel projection share float calibration.
  for (auto *camera : {&config_.left, &config_.right}) {
    for (auto &v : camera->intrinsics) v = static_cast<float>(v);
    for (auto &v : camera->distortion) v = static_cast<float>(v);
  }
  T_left_right_ = (CameraModel(config_.left).cameraToImu().inverse() * CameraModel(config_.right).cameraToImu()).cast<double>();
  prepareRectification();
}
void StereoVisualProcessor::prepareRectification() {
  const auto &l = config_.left, &r = config_.right;
  if (l.width != r.width || l.height != r.height || l.distortion_model != r.distortion_model)
    throw std::invalid_argument("Stereo rectification requires matching image sizes and distortion models");
  const Eigen::Isometry3d right_left = T_left_right_.inverse();
  cv::Mat R(3, 3, CV_64F), t(3, 1, CV_64F), rotations[2], projections[2], Q;
  for (int i = 0; i < 3; ++i) {
    t.at<double>(i) = right_left.translation()[i];
    for (int j = 0; j < 3; ++j) R.at<double>(i, j) = right_left.linear()(i, j);
  }
  const cv::Size size(l.width, l.height);
  const CameraModel left_model(l), right_model(r);
  if (l.distortion_model == "equidistant")
    cv::fisheye::stereoRectify(left_model.matrix(), left_model.distortion(), right_model.matrix(), right_model.distortion(), size, R, t, rotations[0], rotations[1],
                               projections[0], projections[1], Q, cv::fisheye::CALIB_ZERO_DISPARITY, size, 0., 1.);
  else
    cv::stereoRectify(left_model.matrix(), left_model.distortion(), right_model.matrix(), right_model.distortion(), size, R, t, rotations[0], rotations[1],
                      projections[0], projections[1], Q, cv::CALIB_ZERO_DISPARITY, 0., size);
  if (!cv::checkRange(projections[0]) || !cv::checkRange(projections[1]) || std::abs(projections[1].at<double>(1, 3)) > 1e-8 ||
      std::abs(projections[1].at<double>(0, 3)) < 1e-8)
    throw std::invalid_argument("Stereo requires a finite horizontal rectification baseline");
  const auto &P = projections[0];
  rectified_intrinsics_ = {P.at<double>(0, 0), P.at<double>(1, 1), P.at<double>(0, 2), P.at<double>(1, 2)};
  if (rectified_intrinsics_[0] <= 0 || rectified_intrinsics_[1] <= 0) throw std::invalid_argument("Invalid rectified stereo focal length");
  disparity_sign_ = projections[1].at<double>(0, 3) < 0 ? 1. : -1.;  // camera names do not determine physical ordering
  for (int side = 0; side < 2; ++side) {
    const auto &c = config_.camera(side);
    const CameraModel model(c);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) rectification_[side](i, j) = rotations[side].at<double>(i, j);
    if (c.distortion_model == "equidistant")
      cv::fisheye::initUndistortRectifyMap(model.matrix(), model.distortion(), rotations[side], projections[side], size, CV_32FC1, map_x_[side],
                                           map_y_[side]);
    else
      cv::initUndistortRectifyMap(model.matrix(), model.distortion(), rotations[side], projections[side], size, CV_32FC1, map_x_[side],
                                  map_y_[side]);
    cv::Mat mask(size, CV_8UC1, cv::Scalar(255));
    cv::remap(mask, valid_patch_[side], map_x_[side], map_y_[side], cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::threshold(valid_patch_[side], valid_patch_[side], 254, 255, cv::THRESH_BINARY);
    cv::erode(valid_patch_[side], valid_patch_[side], cv::Mat::ones(11, 11, CV_8UC1));
  }
}
void StereoVisualProcessor::beginSubmap() {
  reference_[0].reset();
  reference_[1].reset();
  novelty_since_ = -1;
}
void StereoVisualProcessor::reset() {
  beginSubmap();
  selector_.reset();
  pending_[0].clear();
  pending_[1].clear();
  last_ = {-1, -1};
}
std::optional<StereoVisualProcessor::Result> StereoVisualProcessor::push(ImageMeas image, std::optional<Eigen::Isometry3d> pose) {
  const auto id = image.camera_id;
  if (id > 1 || !std::isfinite(image.timestamp) || image.timestamp < 0) throw std::invalid_argument("Invalid stereo observation");
  const auto &c = config_.camera(id);
  if (image.gray.type() != CV_8UC1 || image.gray.cols != c.width || image.gray.rows != c.height || image.timestamp <= last_[id] ||
      (pose && !rigid(*pose)))
    throw std::invalid_argument("Invalid stereo image/time/pose");
  last_[id] = image.timestamp;
  auto &q = pending_[id];
  if (q.size() == 4) {
    q.pop_front();
    ++dropped_;
  }
  q.push_back({std::move(image), std::move(pose)});
  while (!pending_[0].empty() && !pending_[1].empty()) {
    const double delta = pending_[0].front().image.timestamp - pending_[1].front().image.timestamp;
    if (std::abs(delta) <= sync_tolerance) {
      auto l = std::move(pending_[0].front()), r = std::move(pending_[1].front());
      pending_[0].pop_front();
      pending_[1].pop_front();
      return process(std::move(l), std::move(r));
    }
    pending_[delta < 0 ? 0 : 1].pop_front();
    ++dropped_;
  }
  return {};
}
StereoVisualProcessor::Result StereoVisualProcessor::process(Pending left, Pending right) {
  const auto start = std::chrono::steady_clock::now();
  Result out{{VisualFrame(left.image.timestamp,0),VisualFrame(right.image.timestamp,1)}};
  if (config_.keyframe_selection) {
    out.selection = selector_.evaluate(left.image, right.pose ? left.pose : std::nullopt);
    out.keyframe = out.selection.selected;
    out.usable = out.selection.usable;
    if (!out.keyframe) {
      out.total_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      return out; // No ORB, stereo matching, archive or retrieval for ordinary frames.
    }
  }
  const auto descriptor_start = std::chrono::steady_clock::now();
  // Detection runs only after the flow/LIO gate. The same ORB rows serve stereo,
  // scene-change detection and retrieval; flow tracks only select keyframes.
  out.frames[0] = extractLoopFeatures(left.image, config_);
  out.frames[1] = extractLoopFeatures(right.image, config_);
  out.descriptor_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-descriptor_start).count();
  const auto stereo_start = std::chrono::steady_clock::now();
  for (int side = 0; side < 2; ++side) {
    const auto &c = config_.camera(side);
    out.frames[side].set_projection({c.width, c.height, {c.intrinsics[0], c.intrinsics[1], c.intrinsics[2], c.intrinsics[3]}});
  }
  if (left.pose) out.frames[0].set_observation_pose(*left.pose);
  if (right.pose) out.frames[1].set_observation_pose(*right.pose);
  auto lp = out.frames[0].points(), rp = out.frames[1].points();
  // Each ray/epipolar line depends on one feature, not on a feature pair.
  // Cache these once; retain exactly the symmetric calibrated pixel gate.
  std::vector<Eigen::Vector3d> left_rays, right_lines;
  std::vector<double> left_line_norm, right_line_norm;
  left_rays.reserve(lp.size());
  right_lines.reserve(rp.size());
  left_line_norm.reserve(rp.size());
  right_line_norm.reserve(lp.size());
  for (const auto &point : lp) {
    const auto n = ray(point, config_.left);
    const Eigen::Vector3d a(n.x(), n.y(), 1);
    const Eigen::Vector3d line = T_left_right_.linear().transpose() * a.cross(T_left_right_.translation());
    left_rays.push_back(a);
    right_line_norm.push_back(std::hypot(line.x() / config_.right.intrinsics[0], line.y() / config_.right.intrinsics[1]));
  }
  for (const auto &point : rp) {
    const auto n = ray(point, config_.right);
    const Eigen::Vector3d line = T_left_right_.translation().cross(T_left_right_.linear() * Eigen::Vector3d(n.x(), n.y(), 1));
    right_lines.push_back(line);
    left_line_norm.push_back(std::hypot(line.x() / config_.left.intrinsics[0], line.y() / config_.left.intrinsics[1]));
  }
  cv::Mat mask(int(lp.size()), int(rp.size()), CV_8UC1, cv::Scalar(0));
  for (int i = 0; i < int(lp.size()); ++i) {
    if (rp.empty() || right_line_norm[i] <= 1e-12) continue;
    auto *row = mask.ptr<uchar>(i);
    for (int j = 0; j < int(rp.size()); ++j) {
      if (std::abs(lp[i].level() - rp[j].level()) > 1 || left_line_norm[j] <= 1e-12) continue;
      const double e = std::abs(left_rays[i].dot(right_lines[j]));
      if (e <= max_error * left_line_norm[j] && e <= max_error * right_line_norm[i]) row[j] = 255;
    }
  }
  auto matches = mutual(out.frames[0].descriptors(), out.frames[1].descriptors(), mask);
  out.matches = matches.size();
  std::array<cv::Mat, 2> rectified;
  cv::remap(left.image.gray, rectified[0], map_x_[0], map_y_[0], cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  cv::remap(right.image.gray, rectified[1], map_x_[1], map_y_[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  const auto &K = rectified_intrinsics_;
  const auto project = [&](const VisualPoint &p, int side) {
    const auto n = ray(p, config_.camera(side));
    const Eigen::Vector3d v = rectification_[side] * Eigen::Vector3d(n.x(), n.y(), 1);
    if (v.z() <= 0) return Eigen::Vector2d(NAN, NAN);
    return Eigen::Vector2d(K[0] * v.x() / v.z() + K[2], K[1] * v.y() / v.z() + K[3]);
  };
  const auto valid = [&](int side, int x, int y) {
    return x >= 5 && y >= 5 && x + 5 < rectified[side].cols && y + 5 < rectified[side].rows && valid_patch_[side].at<uchar>(y, x) == 255;
  };
  const auto cost = [&](int lx, int ly, int rx) {
    const int center = int(rectified[0].at<uchar>(ly, lx)) - int(rectified[1].at<uchar>(ly, rx));
    int value = 0;
    for (int dy = -5; dy <= 5; ++dy) {
      const auto *l = rectified[0].ptr<uchar>(ly + dy) + lx - 5;
      const auto *r = rectified[1].ptr<uchar>(ly + dy) + rx - 5;
      for (int dx = 0; dx < 11; ++dx) value += std::abs(int(l[dx]) - int(r[dx]) - center);
    }
    return value;
  };
  struct Refined {
    cv::DMatch match;
    VisualGeometry geometry;
    int cost;
  };
  std::vector<Refined> candidates;
  candidates.reserve(matches.size());
  const Eigen::Isometry3d T_right_left = T_left_right_.inverse();
  for (const auto &m : matches) {
    const auto lxy = project(lp[m.queryIdx], 0), rxy = project(rp[m.trainIdx], 1);
    if (!lxy.allFinite() || !rxy.allFinite() || std::abs(lxy.y() - rxy.y()) > 2.) continue;
    const int lx = cvRound(lxy.x()), ly = cvRound(lxy.y()), rx = cvRound(rxy.x());
    if (!valid(0, lx, ly)) continue;
    std::array<int, 11> costs;
    costs.fill(std::numeric_limits<int>::max());
    for (int offset = -5; offset <= 5; ++offset)
      if (valid(1, rx + offset, ly)) costs[offset + 5] = cost(lx, ly, rx + offset);
    const int best = std::min_element(costs.begin(), costs.end()) - costs.begin();
    if (best == 0 || best == 10 || costs[best - 1] == INT_MAX || costs[best + 1] == INT_MAX) continue;
    const double curvature = double(costs[best - 1]) + costs[best + 1] - 2. * costs[best];
    if (curvature <= 0) continue;
    const double delta = (double(costs[best - 1]) - costs[best + 1]) / (2. * curvature);
    if (!std::isfinite(delta) || std::abs(delta) > 1.) continue;
    const double right_x = rx + best - 5 + delta + (lxy.x() - lx);
    const double disparity = disparity_sign_ * (lxy.x() - right_x);
    if (disparity < .01 || disparity >= K[0]) continue;  // disparity-only support; no distance ceiling
    Eigen::Vector3d refined_ray = rectification_[1].transpose() * Eigen::Vector3d((right_x - K[2]) / K[0], (lxy.y() - K[3]) / K[1], 1.);
    if (refined_ray.z() <= 0) continue;
    auto g =
        triangulateStereo(ray(lp[m.queryIdx], config_.left), refined_ray.head<2>() / refined_ray.z(), T_left_right_, config_.left, config_.right);
    if (!g) continue;
    const Eigen::Vector3d p_right = T_right_left * g->position_camera;
    const auto original = ray(rp[m.trainIdx], config_.right);
    const double error =
        ((p_right.head<2>() / p_right.z() - original).cwiseProduct(Eigen::Vector2d(config_.right.intrinsics[0], config_.right.intrinsics[1]))).norm();
    if (error > max_error) continue;
    g->max_reprojection_px = std::max(g->max_reprojection_px, error);
    candidates.push_back({m, *g, costs[best]});
  }
  std::vector<int> patch_costs;
  for (const auto &c : candidates) patch_costs.push_back(c.cost);
  int median_cost = 0;
  if (!patch_costs.empty()) {
    auto mid = patch_costs.begin() + patch_costs.size() / 2;
    std::nth_element(patch_costs.begin(), mid, patch_costs.end());
    median_cost = *mid;
  }
  for (const auto &c : candidates) {
    if (c.cost > 2 * median_cost) continue;
    if (next_id_ == UINT64_MAX) throw std::overflow_error("Stereo feature ID exhausted");
    auto other = c.geometry;
    other.position_camera = T_right_left * other.position_camera;
    lp[c.match.queryIdx].set_tracking(next_id_, c.geometry);
    rp[c.match.trainIdx].set_tracking(next_id_, other);
    ++next_id_;
    ++out.metric;
  }
  auto ld = out.frames[0].descriptors(), rd = out.frames[1].descriptors();
  out.frames[0].update_features(std::move(lp), std::move(ld));
  out.frames[1].update_features(std::move(rp), std::move(rd));
  out.stereo_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-stereo_start).count();
  if (config_.keyframe_selection) selector_.accept();
  out.usable = usable(out.frames[0], config_.left) && usable(out.frames[1], config_.right);
  if (out.usable) {
    if (!reference_[0]) {
      reference_[0] = copy(out.frames[0]);
      reference_[1] = copy(out.frames[1]);
    }
    bool changed = true;
    for (int side = 0; side < 2; ++side) {
      const auto &ref = *reference_[side];
      auto keep = mutual(ref.descriptors(), out.frames[side].descriptors());
      std::array<int, 48> total{}, retained{};
      for (const auto &point : ref.points()) ++total[cell(point, config_.camera(side))];
      for (const auto &m : keep) ++retained[cell(ref.points()[m.queryIdx], config_.camera(side))];
      double spatial = 0;
      int occupied = 0;
      for (int k = 0; k < 48; ++k)
        if (total[k]) {
          spatial += double(retained[k]) / total[k];
          ++occupied;
        }
      changed &= keep.size() < .45 * ref.points().size() && spatial / std::max(1, occupied) < .5;
    }
    const double stamp = left.image.timestamp;
    if (changed) {
      if (novelty_since_ < 0) novelty_since_ = stamp;
      out.submap_candidate = stamp - reference_[0]->timestamp() >= 1. && stamp - novelty_since_ >= .5;
    } else
      novelty_since_ = -1;
  } else
    novelty_since_ = -1;
  out.total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return out;
}
}  // namespace sapphire
