#include "backend/visual/feature/visual_tracker.hpp"

#include <iostream>
#include <map>
#include <set>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

using namespace sapphire;
namespace {
void check(bool condition, const char *message) { if (!condition) throw std::runtime_error(message); }
CameraParameters camera() {
  CameraParameters c; c.width = 640; c.height = 480; c.intrinsics = {400, 410, 320, 240}; return c;
}
std::array<VisualRayObservation, 3> observations(const Eigen::Vector3d &point, const Eigen::Isometry3d &world) {
  std::array<VisualRayObservation, 3> out;
  Eigen::Isometry3d body_camera = Eigen::Isometry3d::Identity();
  body_camera.linear() = Eigen::AngleAxisd(.2, Eigen::Vector3d(1, 2, -1).normalized()).toRotationMatrix();
  body_camera.translation() = Eigen::Vector3d(.1, -.2, .07);
  for (int i = 0; i < 3; ++i) {
    Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
    body.translation() = Eigen::Vector3d(.2 * i, .04 * i, -.02 * i);
    body.linear() = Eigen::AngleAxisd(.02 * i, Eigen::Vector3d::UnitY()).toRotationMatrix();
    out[i].timestamp = i + 1;
    out[i].T_odom_camera = world * body * body_camera;
    const Eigen::Vector3d pc = out[i].T_odom_camera.inverse() * (world * point);
    out[i].normalized_pixel = pc.head<2>() / pc.z();
  }
  return out;
}
void poseInterpolationTest() {
  Eigen::Isometry3d before = Eigen::Isometry3d::Identity(), after = before, camera = before;
  before.linear() = Eigen::AngleAxisd(.2, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  before.translation() = Eigen::Vector3d(4, -2, 3);
  after.linear() = before.linear() * Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  after.translation() = Eigen::Vector3d(6, -1, 2);
  camera.linear() = Eigen::AngleAxisd(.7, Eigen::Vector3d(3, -2, 1).normalized()).toRotationMatrix();
  camera.translation() = Eigen::Vector3d(.3, -.1, .2);
  auto actual = interpolateVisualCameraPose(1.05, 1, before, 1.2, after, camera);
  Eigen::Isometry3d expected = before;
  expected.linear() = before.linear() * Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitY()).toRotationMatrix();
  expected.translation() = .75 * before.translation() + .25 * after.translation();
  check(actual && (actual->matrix() - (expected * camera).matrix()).norm() < 1e-10,
        "image-time pose interpolates original body motion then composes optical extrinsic");
  check(!interpolateVisualCameraPose(.99, 1, before, 1.2, after, camera) &&
        !interpolateVisualCameraPose(1.21, 1, before, 1.2, after, camera), "visual pose never extrapolates");
  check(!interpolateVisualCameraPose(1.1, 1, before, 1.3, after, camera), "large pose gap cannot produce metric evidence");
  check(!interpolateVisualCameraPose(1, 1, before, 1, after, camera), "duplicate pose stamps cannot bracket");
}
void geometryTest() {
  VisualTrackingParameters p;
  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.linear() = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, -2, 3).normalized()).toRotationMatrix();
  world.translation() = Eigen::Vector3d(10, -30, 5);
  const Eigen::Vector3d point(.4, -.3, 5);
  auto obs = observations(point, world);
  const auto result = triangulateVisualTrack(obs, 400, 410, p);
  check(bool(result), "known asymmetric camera/body/world poses triangulate");
  check((result->position_camera - obs.back().T_odom_camera.inverse() * (world * point)).norm() < 1e-9,
        "output is current optical camera metres, not world/body/first camera");
  auto bad = obs; bad[1].normalized_pixel.y() += .1;
  check(!triangulateVisualTrack(bad, 400, 410, p), "independent middle-view reprojection rejects false correspondence");
  bad = obs; bad[1].timestamp = bad[0].timestamp;
  check(!triangulateVisualTrack(bad, 400, 410, p), "duplicate observation cannot count as third view");
  bad = observations(Eigen::Vector3d(.4, -.3, -5), world);
  check(!triangulateVisualTrack(bad, 400, 410, p), "negative depth rejected");
  bad = observations(Eigen::Vector3d(.4, -.3, 5000), world);
  check(!triangulateVisualTrack(bad, 400, 410, p), "vanishing parallax rejected");
  bad = obs; bad[2].T_odom_camera.linear()(0, 0) += .1;
  check(!triangulateVisualTrack(bad, 400, 410, p), "invalid metric pose rejected");
  for (int i = 0; i < 3; ++i) {
    bad[i].T_odom_camera = Eigen::Isometry3d::Identity();
    bad[i].T_odom_camera.linear() = Eigen::AngleAxisd(.03 * i, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d pc = bad[i].T_odom_camera.inverse() * point;
    bad[i].normalized_pixel = pc.head<2>() / pc.z(); bad[i].timestamp = i;
  }
  check(!triangulateVisualTrack(bad, 400, 410, p), "pure rotation cannot fabricate depth");
  auto scaled = obs;
  for (auto &o : scaled) o.T_odom_camera.translation() *= 2;
  const auto twice = triangulateVisualTrack(scaled, 400, 410, p);
  check(twice && (twice->position_camera - 2 * result->position_camera).norm() < 1e-8, "metric scale comes from supplied motion");
}
cv::Mat texture() {
  cv::Mat image(480, 640, CV_8UC1); cv::RNG rng(49321); rng.fill(image, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(image, image, cv::Size(3, 3), .6); return image;
}
void trackerTest() {
  auto c = camera(); c.intrinsics[1] = 400;
  VisualTrackingParameters p; p.observation_interval_s = .2; p.min_keyframe_interval_s = .1;
  p.observation_displacement_px = 4; p.submap_displacement_px = 25; p.min_submap_interval_s = .3;
  VisualTracker tracker(c, 0, p);
  const cv::Mat original = texture();
  std::map<std::uint64_t, Eigen::Vector2f> first_points;
  int frames = 0, geometry = 0, candidates = 0;
  double worst_depth_error = 0;
  for (int i = 0; i < 22; ++i) {
    cv::Mat image;
    const cv::Mat translation = (cv::Mat_<double>(2, 3) << 1, 0, -3 * i, 0, 1, 0);
    cv::warpAffine(original, image, translation, original.size());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); pose.translation().x() = .03 * i;
    auto result = tracker.process(ImageMeas(i * .05, image), pose);
    check(result.stats.tracks <= std::size_t(p.max_features), "feature budget bounded");
    if (i == 0) check(result.stats.all_point_coverage > .8 && result.stats.coverage == 0 && !result.keyframe,
                      "replenishment coverage available immediately; immature points do not inflate reliable coverage");
    candidates += result.stats.raw_submap_candidate;
    check(!result.stats.submap_candidate, "continuous explained view does not require a new submap for raw displacement alone");
    if (result.keyframe) {
      ++frames;
      const auto &frame = *result.keyframe;
      check(frame.T_odom_camera() && frame.descriptors().rows == int(frame.points().size()), "pose and descriptors row aligned");
      int shared = 0;
      for (const auto &pt : frame.points()) {
        check(pt.track_id() && pt.level() == 0, "single-scale supplied-point descriptor explicitly labelled");
        if (frames == 1) first_points.emplace(pt.track_id(), pt.pixel());
        else if (first_points.count(pt.track_id())) {
          ++shared;
          const float expected_x = first_points.at(pt.track_id()).x() - 3.f * (i - 2);
          check(std::abs(expected_x - pt.pixel().x()) < .8, "descriptor point keeps temporal feature ID and coordinate");
        }
        if (pt.geometry()) {
          ++geometry;
          worst_depth_error = std::max(worst_depth_error, std::abs(pt.geometry()->position_camera.z() - 4));
        }
      }
      if (frames == 1) tracker.acceptSubmapReference();
      else check(shared > 200, "one feature population survives across selected descriptor frames");
    }
  }
  check(frames >= 3 && frames < 12 && geometry > 300 && worst_depth_error < .12, "flow plus temporal geometry works without per-frame descriptions");
  check(candidates > 0, "view displacement can request submap while intermediate observations preserve reference");
  auto evidence = tracker.copySubmapEvidence();
  check(!evidence.empty() && evidence.size() <= std::size_t(p.max_anchors), "representative export is bounded");
  check(evidence.front().projection() && evidence.front().projection()->width == c.width &&
      evidence.front().projection()->height == c.height && evidence.front().projection()->intrinsics ==
      std::array<double,4>{c.intrinsics[0],c.intrinsics[1],c.intrinsics[2],c.intrinsics[3]},
      "copied/enriched tracker evidence retains original rectified projection");
  int enriched = 0;
  for (const auto &frame : evidence) {
    check(bool(frame.T_odom_camera()), "export preserves original image-time pose");
    for (const auto &point : frame.points()) if (point.geometry()) {
      ++enriched;
      const auto &pc = point.geometry()->position_camera;
      const Eigen::Vector2d projected(c.intrinsics[0]*pc.x()/pc.z()+c.intrinsics[2], c.intrinsics[1]*pc.y()/pc.z()+c.intrinsics[3]);
      check((projected-point.pixel().cast<double>()).norm() < 3, "exported geometry belongs to its retained optical camera");
    }
  }
  check(enriched > 100, "early representative gains later same-track triangulation");
  const auto original_byte = tracker.copySubmapEvidence().front().descriptors().at<std::uint8_t>(0, 0);
  // A cv::Mat header can create a mutable alias even from a const public view.
  cv::Mat exported_alias = evidence.front().descriptors();
  exported_alias.at<std::uint8_t>(0, 0) ^= 255;
  check(tracker.copySubmapEvidence().front().descriptors().at<std::uint8_t>(0, 0) == original_byte,
        "exported descriptor mutation cannot modify retained anchor");
  // A resource boundary can occur between descriptions, without resetting flow.
  tracker.beginSubmap();
  check(tracker.copySubmapEvidence().empty(), "new submap exports no previous coverage");
  int no_pose_descriptions = 0, recovered_geometry = 0;
  for (int i = 22; i < 33; ++i) {
    cv::Mat image;
    const cv::Mat translation = (cv::Mat_<double>(2, 3) << 1, 0, -3 * i, 0, 1, 0);
    cv::warpAffine(original, image, translation, original.size());
    std::optional<Eigen::Isometry3d> pose;
    if (i >= 27) { pose = Eigen::Isometry3d::Identity(); pose->translation().x() = .03 * i; }
    auto r = tracker.process(ImageMeas(i * .05, image), pose);
    if (i == 22) {
      check(r.keyframe && r.stats.mature_tracks > 200 && !r.stats.reset_by_gap,
            "resource cut preserves mature continuous tracks and describes fresh coverage");
      check(r.associations.empty(), "old submap anchor IDs do not survive boundary");
      const auto exported = tracker.copySubmapEvidence();
      check(exported.size() == 1 && exported.front().timestamp() == i * .05,
            "first current observation seeds the new submap rather than reusing old image");
    }
    if (!pose && r.keyframe) {
      ++no_pose_descriptions;
      check(!r.keyframe->T_odom_camera(), "missing image-time pose is explicit");
      for (const auto &point : r.keyframe->points()) check(!point.geometry(), "no stale previous-camera XYZ on pose-less observation");
    }
    recovered_geometry += r.stats.triangulated;
  }
  check(no_pose_descriptions > 0 && recovered_geometry > 100, "2D-only interval does not fabricate depth and metric support can recover");
  bool refused = false;
  try { tracker.process(ImageMeas(.5, original)); } catch (const std::invalid_argument &) { refused = true; }
  check(refused, "out of order rejected before state mutation");
  auto gap = tracker.process(ImageMeas(2., original));
  check(gap.stats.reset_by_gap && !gap.stats.submap_candidate && !gap.keyframe, "gap resets visual reference, requires new temporal support");
  auto blank = tracker.process(ImageMeas(2.05, cv::Mat::zeros(original.size(), CV_8UC1)));
  check(!blank.stats.usable && !blank.stats.submap_candidate && !blank.keyframe, "blackout cannot create empty submap request");
  for (int i = 0; i < 4; ++i) {
    auto recovery = tracker.process(ImageMeas(2.1 + .05 * i, original));
    check(!recovery.stats.submap_candidate && recovery.stats.triangulated == 0, "recovery is not novelty and missing metric pose yields no depth");
  }
  tracker.reset();
  check(!tracker.process(ImageMeas(0, original)).keyframe, "explicit reset permits new time domain and requires track maturity");
  std::cout << "translation keyframes=" << frames << " geometry=" << geometry << " max_depth_error_m=" << worst_depth_error << '\n';
}
void intermediateGeometryTest() {
  auto c = camera(); VisualTrackingParameters p;
  p.min_keyframe_interval_s = p.observation_interval_s = 1;
  VisualTracker tracker(c, 0, p);
  const auto original = texture();
  int descriptions = 0, geometry = 0;
  for (int i = 0; i <= 6; ++i) {
    const double x = i <= 2 ? 0 : .04 * (i - 2);
    const cv::Mat warp = (cv::Mat_<double>(2, 3) << 1, 0, -c.intrinsics[0] * x / 4, 0, 1, 0);
    cv::Mat image; cv::warpAffine(original, image, warp, original.size());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); pose.translation().x() = x;
    auto result = tracker.process(ImageMeas(i * .05, image), pose);
    descriptions += bool(result.keyframe);
    geometry += result.stats.triangulated;
    if (i == 2) {
      check(result.keyframe && tracker.copySubmapEvidence().size() == 1, "stationary initial anchor is described");
      const auto initial = tracker.copySubmapEvidence();
      for (const auto &point : initial.front().points())
        check(!point.geometry(), "stationary anchor starts without depth");
    }
    if (i > 2) check(!result.keyframe, "intermediate geometry does not force another description");
  }
  const auto evidence = tracker.copySubmapEvidence();
  check(descriptions == 1 && geometry > 100 && evidence.size() == 1, "geometry matures between sparse descriptions");
  check(evidence.front().projection() && evidence.front().projection()->width == c.width &&
      evidence.front().projection()->height == c.height && evidence.front().projection()->intrinsics ==
      std::array<double,4>{c.intrinsics[0],c.intrinsics[1],c.intrinsics[2],c.intrinsics[3]},
      "copied/enriched tracker evidence retains original rectified projection");
  int enriched = 0;
  for (const auto &point : evidence.front().points()) if (point.geometry()) {
    ++enriched; const auto &pc = point.geometry()->position_camera;
    check(std::abs(pc.z() - 4) < .12, "intermediate geometry retains metric depth");
    const Eigen::Vector2d projected(c.intrinsics[0]*pc.x()/pc.z()+c.intrinsics[2], c.intrinsics[1]*pc.y()/pc.z()+c.intrinsics[3]);
    check((projected - point.pixel().cast<double>()).norm() < .1, "intermediate geometry is expressed in the original anchor camera");
  }
  check(enriched > 100, "retained anchor must receive valid depth even when no descriptor frame is selected");
}
void spatialBudgetTest() {
  auto c = camera(); VisualTrackingParameters p;
  cv::Mat band = cv::Mat::zeros(c.height, c.width, CV_8UC1);
  const auto pattern = texture();
  pattern(cv::Rect(24, 32, c.width-48, 168)).copyTo(band(cv::Rect(24, 32, c.width-48, 168)));
  VisualTracker tracker(c, 0, p);
  for (int i=0; i<3; ++i) {
    auto r=tracker.process(ImageMeas(i*.05,band));
    check(r.stats.tracks <= std::size_t(p.max_features), "coverage redistribution cannot increase total point budget");
    if (i==0) check(r.stats.tracks > 450 && r.stats.all_point_coverage >= .375,
        "textured band can use spare capacity after distributing features across cells");
    if (r.keyframe) {
      std::vector<int> cells(p.grid_cols*p.grid_rows);
      std::set<std::pair<float,float>> pixels;
      for (const auto &pt:r.keyframe->points()) {
        ++cells[int(pt.pixel().y()*p.grid_rows/c.height)*p.grid_cols+int(pt.pixel().x()*p.grid_cols/c.width)];
        check(pixels.emplace(pt.pixel().x(),pt.pixel().y()).second, "two replenishment passes cannot admit one detection twice");
      }
      for(int count:cells) check(count<=3*((p.max_features+p.grid_cols*p.grid_rows-1)/(p.grid_cols*p.grid_rows)),
          "new static features obey the softer per-cell cap");
    }
  }
  cv::Mat patch=cv::Mat::zeros(c.height,c.width,CV_8UC1);
  pattern(cv::Rect(240,160,80,80)).copyTo(patch(cv::Rect(240,160,80,80)));
  VisualTracker foreground(c,0,p);
  for(int i=0;i<10;++i) {
    auto r=foreground.process(ImageMeas(i*.05,patch));
    check(!r.stats.usable && !r.keyframe && !r.stats.submap_candidate,
        "a dense local foreground patch cannot stand in for image-wide mature support");
  }
}
void staticAndOwnershipTest() {
  VisualTracker tracker(camera(), 1);
  cv::Mat original = texture();
  int frames = 0;
  for (int i = 0; i < 60; ++i) {
    cv::Mat reused = original.clone();
    ImageMeas image(i * .05, reused); image.camera_id = 1;
    auto result = tracker.process(image);
    reused.setTo(0);  // Must not mutate the retained previous pyramid.
    frames += bool(result.keyframe);
    check(!result.stats.submap_candidate && result.stats.triangulated == 0, "static/missing-pose frames do not generate fake novelty/depth");
    if (i > 2) check(result.stats.mature_tracks > 400 && result.stats.new_tracks == 0, "owned image and stable IDs avoid repeated detection population");
  }
  check(frames == 1, "static stream computes descriptors once");
}
void rotationTest() {
  auto c = camera(); c.intrinsics[1] = 400;
  VisualTrackingParameters p; p.observation_interval_s = .2; p.min_keyframe_interval_s = .1;
  p.observation_displacement_px = 3;
  VisualTracker tracker(c, 0, p);
  const cv::Mat original = texture();
  std::map<std::uint64_t, cv::Mat> descriptors;
  int frames = 0, compared = 0;
  double hamming_sum = 0;
  for (int i = 0; i < 12; ++i) {
    cv::Mat rotated;
    const cv::Mat affine = cv::getRotationMatrix2D(cv::Point2f(320, 240), double(i), 1);
    cv::warpAffine(original, rotated, affine, original.size());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    // Pixel y points down; OpenCV image rotation corresponds to inverse camera Rz.
    pose.linear() = Eigen::AngleAxisd(i * CV_PI / 180, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto result = tracker.process(ImageMeas(i * .05, rotated), pose);
    check(result.stats.triangulated == 0, "image rotation with zero baseline never reports metric points");
    if (result.keyframe) {
      ++frames;
      for (std::size_t j = 0; j < result.keyframe->points().size(); ++j) {
        const auto id = result.keyframe->points()[j].track_id();
        const auto row = result.keyframe->descriptors().row(int(j));
        if (frames == 1) descriptors.emplace(id, row.clone());
        else if (descriptors.count(id)) { hamming_sum += cv::norm(row, descriptors.at(id), cv::NORM_HAMMING); ++compared; }
      }
    }
  }
  check(frames > 1 && compared > 200 && hamming_sum / compared < 60, "provided tracked points have useful refreshed descriptor orientation");
  std::cout << "rotation shared_descriptor_pairs=" << compared << " mean_hamming=" << hamming_sum / compared << '\n';
}
void affiliationTest(int visit_budget, bool wrong_pose) {
  auto c = camera(); c.intrinsics[1] = 400;
  VisualTrackingParameters p; p.min_keyframe_interval_s = .1; p.observation_interval_s = .2;
  p.observation_displacement_px = 3; p.max_match_visits = visit_budget;
  VisualTracker tracker(c, 0, p);
  const cv::Mat original = texture();
  cv::Mat last;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 16; ++i) {
    const cv::Mat affine = (cv::Mat_<double>(2, 3) << 1, 0, -3*i, 0, 1, 0);
    cv::warpAffine(original, last, affine, original.size());
    pose.translation().x() = .03*i;
    auto r = tracker.process(ImageMeas(.05*i, last), pose);
    if (i == 2) tracker.acceptSubmapReference();
    check(r.stats.anchors <= std::size_t(p.max_anchors), "anchor count bounded");
  }
  auto black = tracker.process(ImageMeas(.8, cv::Mat::zeros(last.size(), CV_8UC1)), pose);
  check(black.stats.anchors > 0 && !black.stats.submap_candidate, "blackout preserves local coverage without empty cut");
  if (wrong_pose) pose.translation().y() += 1;
  std::size_t recovered = 0; bool budget_exhausted = false; std::size_t visits = 0;
  for (int i = 0; i < 8; ++i) {
    auto r = tracker.process(ImageMeas(.85 + .05*i, last), pose);
    recovered = std::max(recovered, r.stats.reacquired);
    visits = std::max(visits, r.stats.match_visits);
    budget_exhausted |= r.stats.match_budget_exhausted;
    check(r.stats.match_visits <= std::size_t(visit_budget), "projection lookup visits bounded");
    check(r.stats.projection_tests <= std::size_t(p.max_anchors*p.max_features), "no history-size projection traversal");
    check(r.stats.triangulated == 0, "restored anchor association does not fabricate new-segment depth");
    check(!r.stats.submap_candidate, "recovery does not cut before affiliation is resolved");
    if (i < 3) check(r.stats.reacquired == 0, "tentative descriptor match waits for later observation");
    std::map<std::uint64_t, int> identities;
    for (const auto &a : r.associations) check(++identities[a.landmark_track_id] == 1, "one landmark cannot claim two current points");
  }
  std::cout << "reacquisition budget=" << visit_budget << " wrong_pose=" << wrong_pose << " recovered=" << recovered
            << " visits=" << visits << " exhausted=" << budget_exhausted << '\n';
  if (visit_budget == 1) check(budget_exhausted && !recovered, "exhausted match evidence is not promoted");
  else if (wrong_pose) check(recovered == 0, "appearance alone cannot override incompatible visual reprojection");
  else check(recovered > 100, "lost tracks reattach to existing visual landmarks after blackout");
  tracker.reset();
  check(tracker.process(ImageMeas(0, last), pose).stats.anchors == 0, "explicit domain reset clears historical local anchors");
}
void anchorCapacityTest() {
  auto c = camera();
  c.width = 320; c.height = 240; c.intrinsics = {220, 220, 160, 120};
  VisualTrackingParameters p; p.max_features = 180; p.grid_cols = 4; p.grid_rows = 3;
  p.min_keyframe_interval_s = .1; p.observation_interval_s = .2; p.novelty_duration_s = .15;
  p.recovery_duration_s = .2; p.min_submap_interval_s = .1;
  VisualTracker tracker(c, 0, p);
  double stamp = 0; bool boundary = false;
  std::size_t last_count = 0;
  for (int scene = 0; scene < 9; ++scene) {
    cv::Mat view(c.height, c.width, CV_8UC1);
    cv::RNG rng(311 + scene); rng.fill(view, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(view, view, cv::Size(3,3), .6);
    tracker.process(ImageMeas(stamp, cv::Mat::zeros(view.size(), CV_8UC1))); stamp += .05;
    for (int i = 0; i < 18; ++i) {
      auto r = tracker.process(ImageMeas(stamp, view)); stamp += .05;
      check(r.stats.anchors <= std::size_t(p.max_anchors) && r.stats.anchor_points <= std::size_t(p.max_anchors*p.max_features),
            "ignored boundary requests cannot grow or rotate the local coverage cache");
      check(r.stats.anchors >= last_count, "anchors are retained until explicit boundary acknowledgement");
      last_count = r.stats.anchors;
      boundary |= r.stats.submap_candidate;
      if (scene == 1) check(!r.stats.submap_candidate, "a second representative image belongs inside the same submap");
      if (scene == 8 && r.stats.submap_candidate) {
        tracker.acceptSubmapReference();
        auto after = tracker.process(ImageMeas(stamp, view)); stamp += .05;
        check(after.stats.anchors == 1 && after.stats.affiliated, "acknowledgement seeds fresh bounded coverage from current observation");
        return;
      }
    }
  }
  check(last_count == std::size_t(p.max_anchors) && boundary, "new stationary views eventually exceed representative budget");
  throw std::runtime_error("last stationary new view did not produce an actionable boundary");
}
}  // namespace
int main() {
  try {
    cv::setNumThreads(1);
    poseInterpolationTest(); geometryTest(); trackerTest(); intermediateGeometryTest(); spatialBudgetTest(); staticAndOwnershipTest(); rotationTest();
    affiliationTest(16384, false); affiliationTest(1, false); affiliationTest(16384, true);
    anchorCapacityTest();
    std::cout << "visual tracker tests passed\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
