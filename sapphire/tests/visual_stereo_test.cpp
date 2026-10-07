#include "backend/visual/solver/visual_stereo.hpp"

#include <zlib.h>

#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "backend/storage/visual_observation_archive.hpp"
#include "common/camera/camera.hpp"
using namespace sapphire;
void check(bool b, const char *s) {
  if (!b) throw std::runtime_error(s);
}
CameraParameters camera() {
  CameraParameters c;
  c.width = 360;
  c.height = 270;
  c.intrinsics = {180, 180, 180, 135};
  c.camera_to_imu_rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  c.camera_to_imu_translation = {0, 0, 0};
  return c;
}
VisualLoopParameters config() {
  VisualLoopParameters c;
  c.enabled = true;
  c.stereo_depth = true;
  c.mode = "stereo";
  c.max_features = 500;
  c.left = camera();
  c.right = c.left;
  c.right.camera_to_imu_translation = {.12, 0, 0};
  return c;
}
ImageMeas image(cv::Mat m, int cam, double time) {
  ImageMeas result(time, std::move(m));
  result.camera_id = cam;
  return result;
}
void geometry() {
  auto c = camera();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = (Eigen::AngleAxisd(.07, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.04, Eigen::Vector3d::UnitY())).toRotationMatrix();
  t.translation() = Eigen::Vector3d(.17, -.023, .011);
  for (const auto &p : {Eigen::Vector3d(.4, -.3, 3), Eigen::Vector3d(-1, .8, 5), Eigen::Vector3d(.2, .3, 1)}) {
    const Eigen::Vector3d right = t.inverse() * p;
    auto l = p.head<2>() / p.z();
    auto r = right.head<2>() / right.z();
    auto g = triangulateStereo(l, r, t, c, c);
    check(g && (g->position_camera - p).norm() < 1e-9, "asymmetric calibrated stereo exact metric depth");
    check(g->observations == 2 && g->source == VisualDepthSource::Stereo, "truthful simultaneous two-camera provenance");
    auto reversed = triangulateStereo(r, l, t.inverse(), c, c);
    check(reversed && (reversed->position_camera - right).norm() < 1e-9, "swapped camera direction exact");
    check(!triangulateStereo(l, r + Eigen::Vector2d(0, .2), t, c, c), "epipolar mismatch rejects");
    check(!triangulateStereo(l, r, Eigen::Isometry3d::Identity(), c, c), "zero baseline rejects");
  }
  // Independent fisheye projection -> shared unprojection -> stereo depth.
  auto fish = c;
  fish.distortion_model = "equidistant";
  fish.distortion = {-.04, .01, -.002, .0003};
  const Eigen::Vector3d expected(.8, -.3, 3), other = t.inverse() * expected;
  const cv::Mat fish_K = (cv::Mat_<double>(3,3) << fish.intrinsics[0],0,fish.intrinsics[2],0,fish.intrinsics[1],fish.intrinsics[3],0,0,1);
  std::array<Eigen::Vector2d, 2> normalized;
  for (int i = 0; i < 2; ++i) {
    const auto xyz = i ? other : expected;
    std::vector<cv::Point3d> object{{xyz.x(), xyz.y(), xyz.z()}};
    std::vector<cv::Point2d> raw;
    cv::fisheye::projectPoints(object, raw, cv::Vec3d(), cv::Vec3d(), fish_K, fish.distortion);
    std::vector<cv::Point2f> rays;
    CameraModel(fish).undistortPixels( {cv::Point2f(raw[0])}, rays);
    normalized[i] = {rays[0].x, rays[0].y};
  }
  auto fisheye_depth = triangulateStereo(normalized[0], normalized[1], t, fish, fish);
  check(fisheye_depth && (fisheye_depth->position_camera - expected).norm() < 1e-4,
        "fisheye rays have one correct undistortion and baseline direction");
  t = Eigen::Isometry3d::Identity();
  t.translation().x() = .12;
  check(!triangulateStereo({0, 0}, {.04, 0}, t, c, c), "negative depth rejects");
  check(!triangulateStereo({0, 0}, {0, 0}, t, c, c), "parallel rays reject");
  auto far = triangulateStereo({0, 0}, {-.12 / 300., 0}, t, c, c);
  check(far && std::abs(far->position_camera.z() - 300) < 1e-4, "far stereo point is not rejected by a metric distance cap");
  auto bad = t;
  bad.linear()(0, 0) = 2;
  check(!triangulateStereo({0, 0}, {-.04, 0}, bad, c, c), "nonrigid calibration rejects");
}
std::array<cv::Mat, 2> texture() {
  cv::Mat l(270, 360, CV_8UC1);
  cv::RNG rng(818);
  rng.fill(l, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(l, l, {3, 3}, .6);
  cv::Mat r;
  cv::Mat affine = (cv::Mat_<double>(2, 3) << 1, 0, -7.2, 0, 1, 0);
  cv::warpAffine(l, r, affine, l.size());
  return {l, r};
}
void archive(const VisualFrame &frame) {
  auto bytes = database_detail::packVisualObservation(frame);
  check(bytes[4] == 3, "stereo writes explicit VOM3");
  auto f = database_detail::readVisualObservation(frame.timestamp(), frame.camera_id(), frame.points().size(), bytes.data(), bytes.size(),
                                                  frame.descriptors().clone());
  check(database_detail::packVisualObservation(f) == bytes, "stereo evidence byte-exact roundtrip");
  int metrics = 0;
  for (const auto &p : f.points())
    if (p.geometry()) {
      ++metrics;
      check(p.geometry()->source == VisualDepthSource::Stereo && p.geometry()->observations == 2, "source and observation count survive archive");
    }
  check(metrics > 10, "archived real stereo support");
  auto rejects = [&](std::vector<std::uint8_t> b) {
    const auto crc = crc32(0, b.data(), b.size() - 4);
    for (int k = 0; k < 4; ++k) b[b.size() - 4 + k] = crc >> (k * 8);
    bool fail = false;
    try {
      database_detail::readVisualObservation(frame.timestamp(), frame.camera_id(), frame.points().size(), b.data(), b.size(),
                                             frame.descriptors().clone());
    } catch (const std::exception &) {
      fail = true;
    }
    check(fail, "invalid stereo archive rejects");
  };
  auto bad = bytes;
  bad[4] = 2;
  rejects(bad);  // v2 cannot interpret stereo tag
  for (std::size_t i = 0; i < f.points().size(); ++i)
    if (f.points()[i].geometry()) {
      bad = bytes;
      bad[184 + i * 72 + 68] = 3;
      rejects(bad);
      break;  // stereo must have 2 views
    }
  VisualPoint invalid({1, 1}, 0, 1);
  VisualGeometry g;
  g.position_camera = {0, 0, 2};
  g.parallax_rad = .05;
  g.observations = 2;
  bool failed = false;
  try {
    invalid.set_tracking(1, g);
  } catch (const std::invalid_argument &) {
    failed = true;
  }
  check(failed, "temporal evidence still requires >=3 views");
}
void processor() {
  auto c = config();
  StereoVisualProcessor stereo(c);
  auto pics = texture();
  check(!stereo.push(image(pics[0], 0, 1.)) && stereo.pending() == 1, "left only has no depth");
  auto result = stereo.push(image(pics[1], 1, 1.000005));
  check(bool(result) && result->metric > 10, "paired real pixel matching triangulates");
  for (const auto &f : result->frames) {
    check(f.points().size() <= 500, "per-camera feature cap");
    archive(f);
  }
  std::vector<double> depths;
  for (const auto &p : result->frames[0].points())
    if (p.geometry()) depths.push_back(p.geometry()->position_camera.z());
  std::sort(depths.begin(), depths.end());
  check(std::abs(depths[depths.size() / 2] - 3) < .5, "known 7.2px disparity gives 3m depth");
  std::cout << "SYNTHETIC matches=" << result->matches << " metric=" << result->metric << " median_depth=" << depths[depths.size() / 2] << '\n';
  Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
  world.linear() = Eigen::AngleAxisd(.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  world.translation() = Eigen::Vector3d(10, -4, 3);
  Eigen::Isometry3d baseline = Eigen::Isometry3d::Identity();
  baseline.translation().x() = .12;
  stereo.push(image(pics[0], 0, 1.5), world);
  auto posed = stereo.push(image(pics[1], 1, 1.500005), world * baseline);
  check(posed && posed->metric == result->metric && !posed->submap_candidate, "world anchoring cannot change stereo depth or stationary reference");
  for (std::size_t i = 0; i < result->frames[0].points().size(); ++i) {
    const auto &a = result->frames[0].points()[i].geometry();
    const auto &b = posed->frames[0].points()[i].geometry();
    check(bool(a) == bool(b) && (!a || (a->position_camera - b->position_camera).norm() == 0), "depth is independent of LIO world pose");
  }
  archive(posed->frames[0]);
  check(!stereo.push(image(pics[0], 0, 2.)) && !stereo.push(image(pics[1], 1, 2.01)), "asynchronous cameras never form depth");
  check(stereo.dropped() > 0, "stale unmatched frame counted");
  stereo.reset();
  for (int i = 0; i < 20; ++i) check(!stereo.push(image(pics[0], 0, 3. + i * .1)), "no peer no temporal fallback");
  check(stereo.pending() == 4, "pending camera queue bounded");
  stereo.reset();
  check(stereo.pending() == 0, "reset clears pending pair");
  // Physical left/right can be opposite to topic names (as in the Hilti rig).
  auto reverse_config = c;
  reverse_config.keyframe_selection = true;
  reverse_config.right.camera_to_imu_translation[0] = -.12;
  StereoVisualProcessor reverse_stereo(reverse_config);
  reverse_stereo.push(image(pics[1], 0, 1.),Eigen::Isometry3d::Identity());
  auto reverse_result = reverse_stereo.push(image(pics[0], 1, 1.),Eigen::Isometry3d::Identity());
  check(reverse_result && reverse_result->metric > 10, "negative physical baseline uses the correct disparity sign");
  std::vector<double> reverse_depths;
  for (const auto &point : reverse_result->frames[0].points())
    if (point.geometry()) reverse_depths.push_back(point.geometry()->position_camera.z());
  std::sort(reverse_depths.begin(), reverse_depths.end());
  check(std::abs(reverse_depths[reverse_depths.size() / 2] - 3.) < .5, "swapped images still give positive 3m depth");

  // Nonparallel cameras: image warp is an independent plane homography oracle.
  auto tilted_config = c;
  tilted_config.keyframe_selection = true;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(.02, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(.01, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) tilted_config.right.camera_to_imu_rotation[i * 3 + j] = rotation(i, j);
  tilted_config.right.camera_to_imu_translation = {.12, .015, .005};
  Eigen::Isometry3d lr = Eigen::Isometry3d::Identity();
  lr.linear() = rotation;
  lr.translation() = Eigen::Vector3d(.12, .015, .005);
  const auto rl = lr.inverse();
  Eigen::Matrix3d hom = rl.linear() + rl.translation() * Eigen::RowVector3d(0, 0, 1) / 3.;
  cv::Mat H(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) H.at<double>(i, j) = hom(i, j);
  // Independent double homography oracle; do not mix the runtime float K with H.
  const auto &intrinsics = c.left.intrinsics;
  const cv::Mat K = (cv::Mat_<double>(3,3) << intrinsics[0],0,intrinsics[2],0,intrinsics[1],intrinsics[3],0,0,1);
  H = K * H * K.inv();
  cv::Mat tilted_right;
  cv::warpPerspective(pics[0], tilted_right, H, pics[0].size());
  StereoVisualProcessor tilted(tilted_config);
  tilted.push(image(pics[0], 0, 1.),Eigen::Isometry3d::Identity());
  auto tilted_result = tilted.push(image(tilted_right, 1, 1.),lr);
  check(tilted_result && tilted_result->metric > 10, "rotated calibrated cameras support sparse patch stereo");
  std::vector<double> tilted_depth;
  for (const auto &point : tilted_result->frames[0].points())
    if (point.geometry()) tilted_depth.push_back(point.geometry()->position_camera.z());
  std::sort(tilted_depth.begin(), tilted_depth.end());
  check(std::abs(tilted_depth[tilted_depth.size() / 2] - 3) < .3, "rectified patch depth returns to original optical-camera coordinates");
  cv::Mat black(270, 360, CV_8UC1, cv::Scalar(0));
  stereo.push(image(pics[0], 0, 10));
  auto empty = stereo.push(image(black, 1, 10));
  check(empty && empty->metric == 0, "textureless peer gives no geometry");
}
void realImages(const char *profile, const char *directory) {
  auto p = load_parameters(profile);
  p.pose_graph.visual.keyframe_selection = false; // isolated stereo depth, not a temporal cadence fixture
  StereoVisualProcessor stereo(p.pose_graph.visual);
  std::size_t total = 0, frames = 0;
  for (int i = 0; i < 24; ++i) {
    for (int cam = 0; cam < 2; ++cam) {
      auto raw = cv::imread(std::string(directory) + "/cam" + std::to_string(cam) + "-" + std::to_string(i) + ".pgm", 0);
      check(!raw.empty(), "real image exists");
      cv::Mat small;
      cv::resize(raw, small, {360, 270}, 0, 0, cv::INTER_AREA);
      auto result = stereo.push(image(small, cam, 1. + i * .075));
      if (result) {
        total += result->metric;
        ++frames;
        check(result->frames[0].points().size() <= 500 && result->frames[1].points().size() <= 500, "real image cap");
      }
    }
  }
  check(frames == 24 && total > 0, "real Hilti produces paired stereo support");
  std::cout << "HILTI pairs=" << frames << " metric_total=" << total << '\n';
  p.pose_graph.visual.keyframe_selection=true;
  StereoVisualProcessor gated(p.pose_graph.visual);
  std::size_t selected=0,ordinary=0,metric=0;
  for(int i=0;i<24;++i) for(int cam=0;cam<2;++cam) {
    auto raw=cv::imread(std::string(directory)+"/cam"+std::to_string(cam)+"-"+std::to_string(i)+".pgm",0);
    cv::Mat small;cv::resize(raw,small,{360,270},0,0,cv::INTER_AREA);
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=.06*i;
    auto result=gated.push(image(small,cam,1.+i*.1),pose);
    if(!result) continue;
    if(result->keyframe) {++selected;metric+=result->metric;}
    else {++ordinary;check(result->descriptor_ms==0 && result->stereo_ms==0,"real non-keyframes never execute descriptors/stereo");}
    std::cout<<"HILTI_GATE frame="<<i<<" tracks="<<result->selection.tracks<<" coverage="<<result->selection.coverage<<" selected="<<result->keyframe<<" metric="<<result->metric<<'\n';
  }
  check(selected>0 && ordinary>selected && metric>0,"real-image synthetic-pose fixture selects sparse keyframes with depth");
  std::cout<<"HILTI_GATE selected="<<selected<<" ordinary="<<ordinary<<" metric="<<metric<<'\n';
}
void keyframeGate() {
  auto c=config();c.keyframe_selection=true;
  auto pics=texture();
  Eigen::Isometry3d origin=Eigen::Isometry3d::Identity(), moved=origin;
  VisualKeyframeSelector selector(c);
  auto seed=selector.evaluate(image(pics[0],0,1.),origin);
  check(seed.selected && seed.seed && seed.tracks<=500,"healthy posed frame seeds keyframe selector");selector.accept();
  moved.translation().x()=.6;
  auto early=selector.evaluate(image(pics[0],0,1.1),moved);
  check(!early.selected && early.translation,"minimum interval suppresses motion candidate");
  for(int i=2;i<10;++i) selector.evaluate(image(pics[0],0,1.+i*.1),moved);
  auto motion=selector.evaluate(image(pics[0],0,2.),moved);
  check(motion.selected && motion.translation && !motion.visual && std::abs(motion.translation_m-.6)<1e-9,
        "rejected candidates preserve accepted reference and LIO alone can select");selector.accept();
  for(int i=1;i<=12;++i) {
    auto still=selector.evaluate(image(pics[0],0,2.+i*.1),moved);
    check(!still.selected,"stationary retained tracks do not periodically produce descriptors");
  }
  auto turn=moved;turn.linear()=Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto rotated=selector.evaluate(image(pics[0],0,3.3),turn);
  check(rotated.selected && rotated.rotation && !rotated.translation,"rotation is independently measured from accepted LIO camera pose");selector.accept();
  cv::Mat black(pics[0].size(),CV_8UC1,cv::Scalar(0));
  auto dark=selector.evaluate(image(black,0,4.5),origin);
  check(!dark.selected && !dark.usable,"motion cannot bypass low visual quality");
  auto recovered=selector.evaluate(image(pics[0],0,4.6),turn);
  check(recovered.selected && recovered.visual && !recovered.translation && !recovered.rotation,
        "newly replenished IDs after tracking loss trigger renewal without erasing accepted pose");
  selector.reset();
  auto unposed=selector.evaluate(image(pics[0],0,1.),{});
  check(!unposed.selected && unposed.usable,"missing LIO pose permits tracking but prevents archival keyframe");

  StereoVisualProcessor stereo(c);
  const auto pair=[&](double stamp,const Eigen::Isometry3d &pose) {
    stereo.push(image(pics[0],0,stamp),pose);
    auto right=pose;right.translation()+=pose.linear()*Eigen::Vector3d(.12,0,0);
    return stereo.push(image(pics[1],1,stamp),right);
  };
  auto first=pair(1.,origin);
  check(first && first->keyframe && first->metric>10 && first->frames[0].has_features(),"selected keyframes have real stereo descriptors/depth");
  std::vector<double> selected_depths;
  for(const auto &point:first->frames[0].points()) if(point.geometry()) selected_depths.push_back(point.geometry()->position_camera.z());
  std::sort(selected_depths.begin(),selected_depths.end());
  check(std::abs(selected_depths[selected_depths.size()/2]-3)<.15,"ORB keyframe preserves known stereo depth");
  archive(first->frames[0]);
  stereo.beginSubmap(); // producer/resource cuts cannot create another visual keyframe
  for(int i=1;i<=12;++i) {
    auto ordinary=pair(1.+i*.1,origin);
    check(ordinary && !ordinary->keyframe && ordinary->matches==0 && ordinary->metric==0 && ordinary->descriptor_ms==0 && ordinary->stereo_ms==0 &&
          !ordinary->frames[0].has_features() && !ordinary->frames[1].has_features(),"non-keyframe exits before ORB, stereo and archival payload");
  }
  auto next=pair(2.3,moved);
  check(next && next->keyframe && next->selection.translation,"integration selects moved keyframe after producer submap cut");
  std::cout<<"KEYFRAME_GATE PASS first_metric="<<first->metric<<" tracks="<<first->selection.tracks<<'\n';
}
int main(int argc, char **argv) try {
  geometry();
  processor();
  keyframeGate();
  if (argc == 3) realImages(argv[1], argv[2]);
  std::cout << "STEREO PASS\n";
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
