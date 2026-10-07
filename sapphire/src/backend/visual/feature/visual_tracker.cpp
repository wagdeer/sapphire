#include "backend/visual/feature/visual_tracker.hpp"
#include "common/camera/camera.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <unordered_map>
#include <unordered_set>

namespace sapphire {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
bool validPose(const Eigen::Isometry3d &pose) {
  return pose.matrix().allFinite() &&
      (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
      std::abs(pose.linear().determinant() - 1) < 1e-6 &&
      (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
}
double median(std::vector<double> values) {
  if (values.empty()) return 0;
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}
bool positive(double value) { return std::isfinite(value) && value > 0; }
bool validGeometryParameters(const VisualTrackingParameters &p) {
  return positive(p.min_baseline_m) && positive(p.min_parallax_rad) && p.min_parallax_rad < 1.5707963267948966 &&
      positive(p.max_reprojection_px) && positive(p.max_depth_m);
}
// Intensity centroid on the same tracked patch; no second point detector.
float orientation(const cv::Mat &image, cv::Point2f pixel) {
  const int x = cvRound(pixel.x), y = cvRound(pixel.y);
  double mx = 0, my = 0;
  for (int v = -15; v <= 15; ++v) {
    const int radius = int(std::floor(std::sqrt(225 - v * v)));
    for (int u = -radius; u <= radius; ++u) {
      const double intensity = image.at<std::uint8_t>(y + v, x + u);
      mx += u * intensity;
      my += v * intensity;
    }
  }
  return cv::fastAtan2(float(my), float(mx));
}
}  // namespace

std::optional<Eigen::Isometry3d> interpolateVisualCameraPose(double image_time,
    double before_time, const Eigen::Isometry3d &before_body,
    double after_time, const Eigen::Isometry3d &after_body,
    const Eigen::Isometry3d &T_body_camera, double max_gap_s) {
  if (!std::isfinite(image_time) || !std::isfinite(before_time) || !std::isfinite(after_time) ||
      !positive(max_gap_s) || after_time <= before_time || after_time - before_time > max_gap_s ||
      image_time < before_time || image_time > after_time || !validPose(before_body) ||
      !validPose(after_body) || !validPose(T_body_camera)) return {};
  const double u = (image_time - before_time) / (after_time - before_time);
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
  body.translation() = (1 - u) * before_body.translation() + u * after_body.translation();
  body.linear() = Eigen::Quaterniond(before_body.linear()).slerp(u, Eigen::Quaterniond(after_body.linear())).toRotationMatrix();
  return body * T_body_camera;
}

std::optional<VisualGeometry> triangulateVisualTrack(const std::array<VisualRayObservation, 3> &obs,
    double fx, double fy, const VisualTrackingParameters &p) {
  if (!positive(fx) || !positive(fy) || !validGeometryParameters(p)) return {};
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(obs[i].timestamp) || !validPose(obs[i].T_odom_camera) || !obs[i].normalized_pixel.allFinite() ||
        (i && obs[i].timestamp <= obs[i - 1].timestamp)) return {};
  const auto &first = obs.front();
  const auto &last = obs.back();
  // Solve in the first camera frame, avoiding subtraction of large world points.
  const Eigen::Isometry3d T_first_last = first.T_odom_camera.inverse() * last.T_odom_camera;
  const Eigen::Vector3d baseline = T_first_last.translation();
  if (baseline.norm() < p.min_baseline_m) return {};
  const Eigen::Vector3d a = Eigen::Vector3d(first.normalized_pixel.x(), first.normalized_pixel.y(), 1).normalized();
  const Eigen::Vector3d b = T_first_last.linear() * Eigen::Vector3d(last.normalized_pixel.x(), last.normalized_pixel.y(), 1).normalized();
  const double cosine = std::clamp(a.dot(b), -1.0, 1.0);
  const double angle = std::acos(cosine), determinant = 1 - cosine * cosine;
  if (angle < p.min_parallax_rad || cosine <= 0 || determinant < 1e-10) return {};
  const double a_base = a.dot(baseline), b_base = b.dot(baseline);
  const double first_distance = (a_base - cosine * b_base) / determinant;
  const double last_distance = (cosine * a_base - b_base) / determinant;
  if (first_distance <= 0 || last_distance <= 0) return {};
  const Eigen::Vector3d point_first = .5 * (first_distance * a + baseline + last_distance * b);
  double max_error = 0;
  Eigen::Vector3d point_current;
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d point = (obs[i].T_odom_camera.inverse() * first.T_odom_camera) * point_first;
    if (!point.allFinite() || point.z() <= 0 || point.norm() > p.max_depth_m) return {};
    const Eigen::Vector2d error = (point.head<2>() / point.z() - obs[i].normalized_pixel).cwiseProduct(Eigen::Vector2d(fx, fy));
    max_error = std::max(max_error, error.norm());
    if (max_error > p.max_reprojection_px) return {};
    if (i == 2) point_current = point;
  }
  return VisualGeometry{point_current, angle, max_error, 3};
}

class VisualTracker::Impl {
 public:
  struct Track {
    std::uint64_t id = 0;
    cv::Point2f pixel;
    int age = 1;
    std::optional<VisualRayObservation> first, previous;
    std::optional<VisualGeometry> geometry;
    std::uint64_t anchor_id = 0;
    std::size_t anchor_row = 0;
    int confirmations = 0;
  };
  struct Anchor { std::uint64_t id; std::shared_ptr<VisualFrame> frame; };
  struct ReferencePoint { int cell; cv::Point2f pixel; };
  struct Reference {
    double timestamp = -1;
    std::unordered_map<std::uint64_t, ReferencePoint> points;
    std::vector<int> cell_counts;
  };
  Impl(CameraParameters camera, std::size_t camera_id, VisualTrackingParameters params)
      : model(camera), camera(std::move(camera)), camera_id(camera_id), p(params) {
    if (this->camera.width < 64 || this->camera.height < 64 || this->camera.width > 4096 || this->camera.height > 4096 ||
        this->camera.intrinsics.size() != 4 || !positive(this->camera.intrinsics[0]) || !positive(this->camera.intrinsics[1]) ||
        !std::all_of(this->camera.intrinsics.begin(), this->camera.intrinsics.end(), [](double x) { return std::isfinite(x); }) ||
        !valid_camera_distortion(this->camera) ||
        !std::all_of(this->camera.distortion.begin(), this->camera.distortion.end(), [](double x) { return std::isfinite(x); }) ||
        p.max_features < 1 || p.max_features > 4096 || p.grid_cols < 1 || p.grid_cols > 32 || p.grid_rows < 1 || p.grid_rows > 32 ||
        p.pyramid_levels < 0 || p.pyramid_levels > 5 || p.min_tracks < 1 || p.min_tracks > p.max_features || p.mature_age < 2 ||
        p.mature_age > 100 || !positive(p.max_gap_s) || !positive(p.min_feature_distance_px) ||
        p.min_feature_distance_px > std::min(this->camera.width, this->camera.height) ||
        !positive(p.max_forward_backward_px) || !positive(p.max_patch_error) || !positive(p.min_coverage) || p.min_coverage > 1 ||
        !positive(p.renewal_threshold) || p.renewal_threshold > 1 || !positive(p.min_keyframe_interval_s) ||
        !positive(p.observation_interval_s) || p.observation_interval_s < p.min_keyframe_interval_s ||
        !positive(p.observation_displacement_px) || !positive(p.min_submap_interval_s) || !positive(p.submap_displacement_px) ||
        !validGeometryParameters(p) || p.max_anchors < 1 || p.max_anchors > 6 || p.max_match_visits < 1 || p.max_match_visits > 65536 ||
        p.min_anchor_matches < 3 || p.min_anchor_matches > p.max_features || !positive(p.projection_radius_px) || p.projection_radius_px > 32 ||
        !positive(p.max_anchor_reprojection_px) || p.max_anchor_reprojection_px > p.projection_radius_px ||
        p.max_hamming_distance < 0 || p.max_hamming_distance > 128 || !positive(p.match_ratio) || p.match_ratio >= 1 ||
        !positive(p.min_affiliated_fraction) || p.min_affiliated_fraction > 1 || !positive(p.novelty_duration_s) ||
        !positive(p.recovery_duration_s)) throw std::invalid_argument("Invalid visual tracker parameters");
    // Freeze the same float intrinsics used by the camera model into observations.
    for (auto &v : this->camera.intrinsics) v = static_cast<float>(v);
    orb = cv::ORB::create(p.max_features, 1.2f, 1, border, 0, 2, cv::ORB::HARRIS_SCORE, 31);
  }
  int cell(cv::Point2f pixel) const {
    return std::clamp(int(pixel.y * p.grid_rows / camera.height), 0, p.grid_rows - 1) * p.grid_cols +
           std::clamp(int(pixel.x * p.grid_cols / camera.width), 0, p.grid_cols - 1);
  }
  bool inside(cv::Point2f pixel) const {
    return std::isfinite(pixel.x) && std::isfinite(pixel.y) && pixel.x >= border && pixel.y >= border &&
        pixel.x < camera.width - border && pixel.y < camera.height - border;
  }
  void resetTracking() {
    tracks.clear(); previous_pyramid.clear();
    last_timestamp = -1; keyframe_reference = {}; submap_reference = {}; latest_usable = false;
    novelty_since = usable_since = -1; recovering = true;
    // IDs are not reused within this object's lifetime, including gap/reset.
  }
  void reset() { resetTracking(); anchors.clear(); last_description.reset(); submap_since = -1; recovering = false; }

  std::shared_ptr<VisualFrame> copyFrame(const VisualFrame &frame) const {
    auto copy = std::make_shared<VisualFrame>(frame.timestamp(), frame.camera_id());
    copy->update_features(frame.points(), frame.descriptors().clone());
    if (frame.T_odom_camera()) copy->set_observation_pose(*frame.T_odom_camera());
    if (frame.projection()) copy->set_projection(*frame.projection());
    return copy;
  }
  void retain(const std::shared_ptr<VisualFrame> &frame) {
    if (!frame || anchors.size() >= std::size_t(p.max_anchors)) return;
    if (next_anchor_id == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Visual anchor ID exhausted");
    anchors.push_back({next_anchor_id++, frame});
    if (submap_since < 0) submap_since = frame->timestamp();
  }
  void acceptSubmap() {
    if (!latest_usable || !last_description || last_description->timestamp() != last_timestamp)
      throw std::logic_error("Submap reference requires a current usable described observation");
    anchors.clear(); retain(last_description); submap_since = last_timestamp;
    submap_reference = reference(); novelty_since = -1;
    for (auto &track : tracks) { track.anchor_id = 0; track.confirmations = 0; }
  }

  // Receive every posed observation independently of descriptor cadence. Add
  // geometry only from the same uninterrupted segment; never splice across gaps.
  void enrichAnchors(const Eigen::Isometry3d &pose) {
    std::unordered_map<std::uint64_t, const VisualGeometry *> observed;
    for (const auto &track : tracks) if (track.geometry) observed.emplace(track.id, &*track.geometry);
    if (observed.empty()) return;
    for (auto &anchor : anchors) {
      if (!anchor.frame->T_odom_camera()) continue;
      std::vector<VisualPoint> points;
      const Eigen::Isometry3d T_anchor_current = anchor.frame->T_odom_camera()->inverse() * pose;
      for (std::size_t i = 0; i < anchor.frame->points().size(); ++i) {
        const auto &original = anchor.frame->points()[i];
        const auto it = observed.find(original.track_id());
        if (original.geometry() || it == observed.end()) continue;
        auto geometry = *it->second;
        geometry.position_camera = T_anchor_current * geometry.position_camera;
        const auto &pc = geometry.position_camera;
        if (pc.z() <= 0) continue;
        const Eigen::Vector2d pixel(camera.intrinsics[0] * pc.x() / pc.z() + camera.intrinsics[2],
            camera.intrinsics[1] * pc.y() / pc.z() + camera.intrinsics[3]);
        const double anchor_error = (pixel - original.pixel().cast<double>()).norm();
        if (!pixel.allFinite() || anchor_error > p.max_anchor_reprojection_px) continue;
        geometry.max_reprojection_px = std::max(geometry.max_reprojection_px, anchor_error);
        if (points.empty()) points = anchor.frame->points();
        points[i].set_tracking(original.track_id(), geometry);
      }
      if (!points.empty()) anchor.frame->update_features(std::move(points), anchor.frame->descriptors());
    }
  }
  struct Projection { std::size_t anchor, row; cv::Point2f raw; Eigen::Vector2d rectified; };
  std::optional<Projection> project(std::size_t a, std::size_t row, const Eigen::Isometry3d &pose) const {
    const auto &frame = *anchors[a].frame;
    const auto &point = frame.points()[row];
    if (!frame.T_odom_camera() || !point.geometry()) return {};
    const Eigen::Isometry3d T_current_anchor = pose.inverse() * *frame.T_odom_camera();
    const auto &source = point.geometry()->position_camera;
    const Eigen::Vector3d pc = T_current_anchor * source;
    if (!pc.allFinite() || pc.z() <= 0 || pc.norm() > p.max_depth_m) return {};
    const double scale = pc.norm() / source.norm();
    if (scale < .5 || scale > 2 || (T_current_anchor.linear() * source).normalized().dot(pc.normalized()) < .7) return {};
    const Eigen::Vector3f local_point = pc.cast<float>();
    const auto raw = model.project(local_point);
    const auto rectified = model.bearingToRectified(local_point);
    if (!raw || !rectified) return {};
    Projection result{a, row, cv::Point2f(raw->x(), raw->y()), rectified->cast<double>()};
    if (!inside(result.raw) || !result.rectified.allFinite()) return {};
    return result;
  }
  std::vector<Projection> projections(const std::optional<Eigen::Isometry3d> &pose, VisualTrackingStats &stats) const {
    std::vector<Projection> out;
    if (!pose) return out;
    for (std::size_t a = 0; a < anchors.size(); ++a) for (std::size_t row = 0; row < anchors[a].frame->points().size(); ++row) {
      ++stats.projection_tests;
      auto prediction = project(a, row, *pose);
      if (prediction) out.push_back(*prediction);
    }
    stats.projected = out.size();
    return out;
  }
  void affiliate(Result &result, const std::vector<Projection> &predictions, const std::vector<cv::Point2f> &normalized,
      const std::vector<std::vector<std::size_t>> &grid, const std::vector<int> &mature_counts) {
    auto &stats = result.stats;
    std::vector<std::vector<bool>> occupied;
    std::unordered_set<std::uint64_t> claimed_landmarks;
    for (const auto &a : anchors) occupied.emplace_back(a.frame->points().size(), false);
    std::unordered_map<std::uint64_t, std::size_t> by_id;
    for (std::size_t i = 0; i < tracks.size(); ++i) by_id.emplace(tracks[i].id, i);
    std::vector<std::vector<const Projection *>> projected_rows;
    for (const auto &a : anchors) projected_rows.emplace_back(a.frame->points().size(), nullptr);
    for (const auto &prediction : predictions) projected_rows[prediction.anchor][prediction.row] = &prediction;
    auto validReprojection = [&](std::size_t i, const Projection *prediction) {
      if (!prediction) return false;
      const Eigen::Vector2d pixel(normalized[i].x * camera.intrinsics[0] + camera.intrinsics[2],
          normalized[i].y * camera.intrinsics[1] + camera.intrinsics[3]);
      return pixel.allFinite() && (pixel - prediction->rectified).norm() <= p.max_anchor_reprojection_px;
    };
    // Continuous segment identity is stronger than a fresh descriptor match.
    // A retained 2D point may supply affiliation without pretending to have depth.
    for (std::size_t a = 0; a < anchors.size(); ++a) for (std::size_t row = 0; row < anchors[a].frame->points().size(); ++row) {
      const auto it = by_id.find(anchors[a].frame->points()[row].track_id());
      if (it == by_id.end()) continue;
      auto &track = tracks[it->second];
      if (!track.anchor_id) { track.anchor_id = anchors[a].id; track.anchor_row = row; track.confirmations = 2; }
    }
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      auto &track = tracks[i];
      if (!track.anchor_id) continue;
      auto anchor = std::find_if(anchors.begin(), anchors.end(), [&](const Anchor &a) { return a.id == track.anchor_id; });
      bool valid = anchor != anchors.end() && track.anchor_row < anchor->frame->points().size();
      std::size_t a = std::size_t(anchor - anchors.begin());
      if (valid) {
        const auto source_id = anchor->frame->points()[track.anchor_row].track_id();
        const bool continuous = source_id == track.id;
        valid = !occupied[a][track.anchor_row] && !claimed_landmarks.count(source_id) &&
            (continuous || validReprojection(i, projected_rows[a][track.anchor_row]));
        if (valid) claimed_landmarks.insert(source_id);
      }
      if (!valid) { track.anchor_id = 0; track.confirmations = 0; }
      else { occupied[a][track.anchor_row] = true; track.confirmations = std::min(2, track.confirmations + 1); }
    }
    if (result.keyframe) {
      std::unordered_map<std::uint64_t, int> descriptor_rows;
      for (std::size_t i = 0; i < result.keyframe->points().size(); ++i)
        descriptor_rows.emplace(result.keyframe->points()[i].track_id(), int(i));
      struct Match { std::size_t track, anchor, row; int distance; };
      std::vector<Match> matches;
      const double radius2 = p.projection_radius_px * p.projection_radius_px;
      for (const auto &prediction : predictions) {
        if (occupied[prediction.anchor][prediction.row] ||
            claimed_landmarks.count(anchors[prediction.anchor].frame->points()[prediction.row].track_id())) continue;
        const int x0 = std::max(0, int((prediction.raw.x - p.projection_radius_px) * p.grid_cols / camera.width));
        const int x1 = std::min(p.grid_cols - 1, int((prediction.raw.x + p.projection_radius_px) * p.grid_cols / camera.width));
        const int y0 = std::max(0, int((prediction.raw.y - p.projection_radius_px) * p.grid_rows / camera.height));
        const int y1 = std::min(p.grid_rows - 1, int((prediction.raw.y + p.projection_radius_px) * p.grid_rows / camera.height));
        int best = 257, second = 257; std::size_t index = 0;
        for (int y = y0; y <= y1 && !stats.match_budget_exhausted; ++y)
          for (int x = x0; x <= x1 && !stats.match_budget_exhausted; ++x) for (const auto i : grid[y * p.grid_cols + x]) {
            if (stats.match_visits == std::size_t(p.max_match_visits)) { stats.match_budget_exhausted = true; break; }
            ++stats.match_visits;
            const auto delta = tracks[i].pixel - prediction.raw;
            if (tracks[i].anchor_id || delta.dot(delta) > radius2) continue;
            const auto desc = descriptor_rows.find(tracks[i].id);
            if (desc == descriptor_rows.end() || !validReprojection(i, &prediction)) continue;
            const int distance = int(cv::norm(result.keyframe->descriptors().row(desc->second),
                anchors[prediction.anchor].frame->descriptors().row(int(prediction.row)), cv::NORM_HAMMING));
            if (distance < best) { second = best; best = distance; index = i; }
            else second = std::min(second, distance);
          }
        if (stats.match_budget_exhausted) break;
        if (best <= p.max_hamming_distance && best < p.match_ratio * second)
          matches.push_back({index, prediction.anchor, prediction.row, best});
      }
      // Ambiguity in the reverse direction matters as well: competing nearby
      // landmark identities cannot both claim the same current image feature.
      // The same temporal feature retained in two anchors is one identity.
      std::map<std::pair<std::size_t, std::uint64_t>, Match> unique;
      for (const auto &m : matches) {
        const auto key = std::make_pair(m.track, anchors[m.anchor].frame->points()[m.row].track_id());
        const auto it = unique.find(key);
        if (it == unique.end() || m.distance < it->second.distance) unique.insert_or_assign(key, m);
      }
      matches.clear();
      for (const auto &entry : unique) matches.push_back(entry.second);
      std::vector<int> best(tracks.size(), 257), second(tracks.size(), 257);
      for (const auto &m : matches) {
        if (m.distance < best[m.track]) { second[m.track] = best[m.track]; best[m.track] = m.distance; }
        else second[m.track] = std::min(second[m.track], m.distance);
      }
      std::sort(matches.begin(), matches.end(), [](const Match &a, const Match &b) {
        if (a.distance != b.distance) return a.distance < b.distance;
        if (a.anchor != b.anchor) return a.anchor < b.anchor;
        return a.row < b.row;
      });
      if (!stats.match_budget_exhausted) for (const auto &m : matches) {
        auto &track = tracks[m.track];
        const auto source_id = anchors[m.anchor].frame->points()[m.row].track_id();
        if (track.anchor_id || occupied[m.anchor][m.row] || claimed_landmarks.count(source_id) ||
            !(m.distance < p.match_ratio * second[m.track])) continue;
        track.anchor_id = anchors[m.anchor].id; track.anchor_row = m.row; track.confirmations = 1;
        occupied[m.anchor][m.row] = true;
        claimed_landmarks.insert(source_id);
      }
    }
    std::vector<int> supported(mature_counts.size()), anchor_support(anchors.size());
    for (const auto &track : tracks) if (track.age >= p.mature_age && track.anchor_id && track.confirmations >= 2) {
      auto a = std::find_if(anchors.begin(), anchors.end(), [&](const Anchor &anchor) { return anchor.id == track.anchor_id; });
      if (a == anchors.end()) continue;
      const auto landmark = a->frame->points()[track.anchor_row].track_id();
      result.associations.push_back({track.id, a->id, landmark});
      ++stats.associated; ++supported[cell(track.pixel)]; ++anchor_support[a - anchors.begin()];
      if (landmark != track.id) ++stats.reacquired;
    }
    int cells = 0;
    for (std::size_t i = 0; i < mature_counts.size(); ++i) if (mature_counts[i]) {
      stats.affiliation_coverage += double(supported[i]) / mature_counts[i]; ++cells;
    }
    stats.affiliation_coverage /= std::max(1, cells);
    stats.unexplained_fraction = cells ? 1 - stats.affiliation_coverage : 0;
    for (std::size_t a = 0; a < anchors.size(); ++a) if (std::size_t(anchor_support[a]) > stats.best_anchor_matches) {
      stats.best_anchor_matches = anchor_support[a]; stats.reference_anchor = anchors[a].id;
    }
    stats.affiliated = stats.usable && stats.associated >= std::size_t(p.min_anchor_matches) &&
        stats.affiliation_coverage >= p.min_affiliated_fraction;
  }
  Reference reference() const {
    Reference result;
    result.timestamp = last_timestamp;
    result.cell_counts.assign(p.grid_cols * p.grid_rows, 0);
    for (const auto &track : tracks) if (track.age >= p.mature_age) {
      const int c = cell(track.pixel);
      result.points.emplace(track.id, ReferencePoint{c, track.pixel});
      ++result.cell_counts[c];
    }
    return result;
  }
  void measure(const Reference &ref, VisualTrackingStats &stats) const {
    stats.reference_tracks = ref.points.size();
    if (ref.points.empty()) return;
    std::vector<int> retained(ref.cell_counts.size());
    std::vector<double> displacement;
    for (const auto &track : tracks) if (track.age >= p.mature_age) {
      const auto it = ref.points.find(track.id);
      if (it != ref.points.end()) {
        ++stats.shared_tracks;
        ++retained[it->second.cell];
        displacement.push_back(cv::norm(track.pixel - it->second.pixel));
      }
    }
    stats.reference_retention = double(stats.shared_tracks) / ref.points.size();
    stats.renewal = stats.mature_tracks ? 1 - double(stats.shared_tracks) / stats.mature_tracks : 0;
    int occupied = 0;
    for (std::size_t i = 0; i < retained.size(); ++i) if (ref.cell_counts[i]) {
      stats.spatial_retention += double(retained[i]) / ref.cell_counts[i];
      ++occupied;
    }
    stats.spatial_retention /= std::max(1, occupied);
    stats.reference_displacement_px = median(std::move(displacement));
  }
  Result process(const ImageMeas &image, const std::optional<Eigen::Isometry3d> &pose) {
    if (image.camera_id != camera_id || image.gray.type() != CV_8UC1 || image.gray.cols != camera.width || image.gray.rows != camera.height ||
        !std::isfinite(image.timestamp) || image.timestamp < 0 || image.timestamp <= last_timestamp || (pose && !validPose(*pose)))
      throw std::invalid_argument("Invalid/out-of-order visual tracker observation");
    struct FailureReset {
      Impl &owner;
      bool complete = false;
      ~FailureReset() { if (!complete) owner.reset(); }
    } failure_reset{*this};
    const auto start = Clock::now();
    Result result;
    auto &stats = result.stats;
    if (last_timestamp >= 0 && image.timestamp - last_timestamp > p.max_gap_s) {
      resetTracking(); stats.reset_by_gap = true;
    }
    last_timestamp = image.timestamp;
    std::vector<cv::Mat> pyramid;
    // tryReuseInputImage=false: asynchronous caller may reuse its image buffer.
    cv::buildOpticalFlowPyramid(image.gray, pyramid, cv::Size(21, 21), p.pyramid_levels, true,
        cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, false);
    std::vector<int> counts(p.grid_cols * p.grid_rows), mature_counts(counts.size());
    std::vector<std::vector<std::size_t>> grid(counts.size());
    const auto predictions = projections(pose, stats);
    std::vector<int> predicted_cells(counts.size());
    for (const auto &prediction : predictions) ++predicted_cells[cell(prediction.raw)];
    const int per_cell = (p.max_features + int(counts.size()) - 1) / int(counts.size());
    if (!tracks.empty()) {
      std::vector<cv::Point2f> previous, current, backward;
      for (const auto &track : tracks) previous.push_back(track.pixel);
      std::vector<uchar> forward_status, backward_status;
      std::vector<float> forward_error, backward_error;
      cv::calcOpticalFlowPyrLK(previous_pyramid, pyramid, previous, current, forward_status, forward_error,
          cv::Size(21, 21), p.pyramid_levels);
      // Failed forward points are replaced before backward LK; status still rejects them.
      for (std::size_t i = 0; i < current.size(); ++i)
        if (!forward_status[i] || !inside(current[i])) { forward_status[i] = 0; current[i] = previous[i]; }
      cv::calcOpticalFlowPyrLK(pyramid, previous_pyramid, current, backward, backward_status, backward_error,
          cv::Size(21, 21), p.pyramid_levels);
      std::vector<Track> kept;
      kept.reserve(tracks.size());
      // Existing order favors old tracks; new detections are appended.
      for (std::size_t i = 0; i < tracks.size(); ++i) {
        if (!forward_status[i] || !backward_status[i] || !inside(backward[i]) ||
            !std::isfinite(forward_error[i]) || forward_error[i] > p.max_patch_error ||
            cv::norm(backward[i] - previous[i]) > p.max_forward_backward_px) {
          ++stats.flow_rejected; continue;
        }
        auto &track = tracks[i];
        track.pixel = current[i];
        track.age = std::min(track.age + 1, 1000000);
        track.geometry.reset();
        ++counts[cell(track.pixel)];
        if (track.age >= p.mature_age) { ++mature_counts[cell(track.pixel)]; ++stats.mature_tracks; }
        grid[cell(track.pixel)].push_back(kept.size());
        kept.push_back(std::move(track));
      }
      tracks = std::move(kept);
      if (tracks.empty()) {
        stats.reference_lost = !anchors.empty();
        submap_reference = {}; keyframe_reference = {};
        recovering = true; usable_since = novelty_since = -1;
      }
    }
    // Coverage is derived from the same admission counters used by replenishment.
    // Newly detected points do not inflate mature coverage or novelty.
    if (tracks.size() < std::size_t(p.max_features)) {
      cv::Mat mask(image.gray.size(), CV_8UC1, cv::Scalar(0));
      mask(cv::Rect(border, border, camera.width - 2 * border, camera.height - 2 * border)).setTo(255);
      for (const auto &track : tracks) cv::circle(mask, track.pixel, int(std::ceil(p.min_feature_distance_px)), cv::Scalar(0), -1);
      // Grid quotas govern new detections only. Pruning live tracks when camera
      // motion moves them into a full cell would manufacture feature renewal.
      std::vector<cv::Point2f> detected;
      cv::goodFeaturesToTrack(image.gray, detected, p.max_features * 3, .01, p.min_feature_distance_px, mask);
      // Predicted cells are visited first, without reserving observed coverage or
      // masking failed predictions. All detections return to the same pool.
      std::stable_partition(detected.begin(), detected.end(), [&](cv::Point2f pixel) { return predicted_cells[cell(pixel)] > 0; });
      // Guarantee the ordinary coverage quota first. Spend otherwise unused
      // capacity on textured cells with a bounded, softer quota; empty sky/road
      // cells must not reserve slots forever. Existing tracks are never pruned.
      std::vector<bool> admitted(detected.size(), false);
      for (int pass = 0; pass < 2; ++pass) for (std::size_t candidate = 0; candidate < detected.size(); ++candidate) {
        if (admitted[candidate]) continue;
        const auto &pixel = detected[candidate];
        if (tracks.size() >= std::size_t(p.max_features)) break;
        const int c = cell(pixel);
        if (!inside(pixel) || counts[c] >= per_cell * (pass == 0 ? 1 : 3)) continue;
        if (next_id == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Visual track ID exhausted");
        Track track; track.id = next_id++; track.pixel = pixel;
        grid[c].push_back(tracks.size());
        tracks.push_back(std::move(track));
        admitted[candidate] = true;
        ++counts[c]; ++stats.new_tracks;
      }
    }
    stats.tracks = tracks.size();
    stats.coverage = double(std::count_if(mature_counts.begin(), mature_counts.end(), [](int n) { return n > 0; })) / counts.size();
    stats.all_point_coverage = double(std::count_if(counts.begin(), counts.end(), [](int n) { return n > 0; })) / counts.size();
    stats.usable = stats.mature_tracks >= std::size_t(p.min_tracks) && stats.coverage >= p.min_coverage;
    latest_usable = stats.usable;
    std::vector<cv::Point2f> pixels, normalized;
    for (const auto &track : tracks) pixels.push_back(track.pixel);
    model.undistortPixels( pixels, normalized);
    if (pose) for (std::size_t i = 0; i < tracks.size(); ++i) {
      auto &track = tracks[i];
      const auto &pixel = normalized[i];
      VisualRayObservation observation{image.timestamp, *pose, Eigen::Vector2d(pixel.x, pixel.y)};
      if (!observation.normalized_pixel.allFinite()) { track.first.reset(); track.previous.reset(); continue; }
      if (track.first && track.previous && track.first->timestamp < track.previous->timestamp) {
        track.geometry = triangulateVisualTrack({*track.first, *track.previous, observation}, camera.intrinsics[0], camera.intrinsics[1], p);
        if (track.geometry) ++stats.triangulated;
      }
      if (!track.first) track.first = observation;
      track.previous = observation;
    }
    measure(submap_reference, stats);
    stats.raw_submap_candidate = stats.usable && !submap_reference.points.empty() &&
        image.timestamp - submap_reference.timestamp >= p.min_submap_interval_s &&
        ((stats.renewal >= p.renewal_threshold && stats.spatial_retention <= 1 - p.renewal_threshold) ||
         stats.reference_displacement_px >= p.submap_displacement_px);
    VisualTrackingStats keyframe_stats;
    keyframe_stats.mature_tracks = stats.mature_tracks;
    measure(keyframe_reference, keyframe_stats);
    const double since_keyframe = image.timestamp - keyframe_reference.timestamp;
    // A newly recovered but stationary view still needs a decision after its
    // confirmation interval. Motion-only descriptor scheduling would starve it.
    const bool decision_due = novelty_since >= 0 && image.timestamp - novelty_since >= p.novelty_duration_s;
    const bool recovery_due = recovering && usable_since >= 0 && image.timestamp - usable_since >= p.recovery_duration_s;
    const bool describe = stats.usable && (keyframe_reference.points.empty() ||
        (since_keyframe >= p.min_keyframe_interval_s && (decision_due || recovery_due || keyframe_stats.renewal >= p.renewal_threshold ||
        (since_keyframe >= p.observation_interval_s && keyframe_stats.reference_displacement_px >= p.observation_displacement_px))));
    stats.tracking_ms = elapsed(start);
    if (describe) {
      const auto descriptor_start = Clock::now();
      std::vector<cv::KeyPoint> keypoints;
      for (std::size_t i = 0; i < tracks.size(); ++i) if (tracks[i].age >= p.mature_age)
        keypoints.emplace_back(tracks[i].pixel, 31, orientation(image.gray, tracks[i].pixel), 1, 0, int(i));
      cv::Mat descriptors;
      orb->compute(image.gray, keypoints, descriptors);
      std::vector<VisualPoint> points;
      cv::Mat accepted_descriptors;
      for (std::size_t i = 0; i < keypoints.size(); ++i) {
        const auto index = std::size_t(keypoints[i].class_id);
        if (index >= tracks.size()) throw std::logic_error("ORB lost visual track row identity");
        const auto pixel = normalized[index];
        const Eigen::Vector2f rectified(float(pixel.x * camera.intrinsics[0] + camera.intrinsics[2]),
            float(pixel.y * camera.intrinsics[1] + camera.intrinsics[3]));
        if (!rectified.allFinite() || rectified.x() < 0 || rectified.y() < 0 || rectified.x() >= camera.width || rectified.y() >= camera.height) continue;
        VisualPoint point(rectified, 0, 1);
        point.set_tracking(tracks[index].id, tracks[index].geometry);
        points.push_back(std::move(point));
        accepted_descriptors.push_back(descriptors.row(int(i)));
      }
      if (points.size() >= std::size_t(p.min_tracks)) {
        VisualFrame frame(image.timestamp, camera_id);
        frame.update_features(std::move(points), std::move(accepted_descriptors));
        frame.set_projection({camera.width, camera.height,
            {camera.intrinsics[0], camera.intrinsics[1], camera.intrinsics[2], camera.intrinsics[3]}});
        if (pose) frame.set_observation_pose(*pose);
        result.keyframe.emplace(std::move(frame));
        keyframe_reference = reference();
      }
      stats.descriptor_ms = elapsed(descriptor_start);
    }
    const auto affiliation_start = Clock::now();
    if (pose) enrichAnchors(*pose);
    if (result.keyframe) {
      last_description = copyFrame(*result.keyframe);
    }
    affiliate(result, predictions, normalized, grid, mature_counts);
    if (!stats.usable) usable_since = -1;
    else if (usable_since < 0) usable_since = image.timestamp;
    if (stats.affiliated || (usable_since >= 0 && image.timestamp - usable_since >= p.recovery_duration_s)) recovering = false;
    stats.recovering = recovering;
    const bool pending_reassociation = std::count_if(tracks.begin(), tracks.end(), [](const Track &t) {
      return t.anchor_id && t.confirmations == 1;
    }) >= p.min_anchor_matches;
    const bool novelty = stats.usable && !stats.affiliated && !recovering && !pending_reassociation && !stats.match_budget_exhausted;
    if (!novelty) novelty_since = -1;
    else if (novelty_since < 0) novelty_since = image.timestamp;
    if (result.keyframe && anchors.empty()) {
      retain(last_description); stats.internal_keyframe = true; novelty_since = -1;
    } else if (result.keyframe && novelty_since >= 0 && image.timestamp - novelty_since >= p.novelty_duration_s) {
      if (anchors.size() < std::size_t(p.max_anchors)) {
        retain(last_description); stats.internal_keyframe = true; novelty_since = -1;
      } else if (image.timestamp - submap_since >= p.min_submap_interval_s) stats.submap_candidate = true;
    }
    stats.anchors = anchors.size();
    for (const auto &anchor : anchors) stats.anchor_points += anchor.frame->points().size();
    stats.affiliation_ms = elapsed(affiliation_start);
    previous_pyramid = std::move(pyramid);
    failure_reset.complete = true;
    return result;
  }
  CameraModel model;
  CameraParameters camera;
  std::size_t camera_id;
  VisualTrackingParameters p;
  static constexpr int border = 24;
  cv::Ptr<cv::ORB> orb;
  std::vector<cv::Mat> previous_pyramid;
  std::vector<Track> tracks;
  Reference keyframe_reference, submap_reference;
  double last_timestamp = -1;
  std::uint64_t next_id = 1;
  bool latest_usable = false;
  std::vector<Anchor> anchors;
  std::shared_ptr<VisualFrame> last_description;
  std::uint64_t next_anchor_id = 1;
  double submap_since = -1, novelty_since = -1, usable_since = -1;
  bool recovering = false;
};

VisualTracker::VisualTracker(CameraParameters camera, std::size_t camera_id, VisualTrackingParameters parameters)
    : impl_(std::make_unique<Impl>(std::move(camera), camera_id, parameters)) {}
VisualTracker::~VisualTracker() = default;
VisualTracker::Result VisualTracker::process(const ImageMeas &image, std::optional<Eigen::Isometry3d> pose) { return impl_->process(image, pose); }
void VisualTracker::reset() { impl_->reset(); }
void VisualTracker::beginSubmap() {
  impl_->anchors.clear(); impl_->last_description.reset();
  impl_->keyframe_reference = {}; impl_->submap_reference = {};
  impl_->submap_since = impl_->novelty_since = -1;
  for (auto &track : impl_->tracks) { track.anchor_id = 0; track.confirmations = 0; }
}
void VisualTracker::acceptSubmapReference() {
  impl_->acceptSubmap();
}
std::vector<VisualFrame> VisualTracker::copySubmapEvidence() const {
  std::vector<VisualFrame> result;
  result.reserve(impl_->anchors.size());
  for (const auto &anchor : impl_->anchors) result.push_back(std::move(*impl_->copyFrame(*anchor.frame)));
  return result;
}
}  // namespace sapphire
