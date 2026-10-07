#include "backend/storage/visual_observation_archive.hpp"
#include "backend/visual/visual_loop.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include <cstring>
#include <iostream>
#include <zlib.h>
using namespace sapphire;
void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
void checksum(std::vector<std::uint8_t> &bytes) {
  const auto value = crc32(0, bytes.data(), bytes.size() - 4);
  for (int i = 0; i < 4; ++i) bytes[bytes.size() - 4 + i] = value >> (8 * i);
}
int main() try {
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  anchor.linear() = (Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitY())).toRotationMatrix();
  anchor.translation() = Eigen::Vector3d(12, -5, 2);
  Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
  camera.linear() = Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitX()).toRotationMatrix();
  camera.translation() = Eigen::Vector3d(.35, -.1, .2);
  const Eigen::Vector3d xyz(.4, -.3, 4.5);
  std::array<VisualRayObservation, 3> observations;
  for (int i = 0; i < 3; ++i) {
    auto pose = anchor * camera;
    pose.translation() += anchor.linear() * Eigen::Vector3d((i - 2) * .15, .02 * (i - 2), 0);
    const Eigen::Vector3d pc = pose.inverse() * (anchor * camera * xyz);
    observations[i] = {1.0 + .1 * i, pose, pc.head<2>() / pc.z()};
  }
  auto geometry = triangulateVisualTrack(observations, 300, 300, {});
  check(geometry && (geometry->position_camera - xyz).norm() < 1e-9, "independent visual ray triangulation");
  VisualFrame frame(1.2, 1);
  frame.set_observation_pose(anchor * camera);
  VisualPoint point({float(320 + 300 * xyz.x()/xyz.z()), float(240 + 300 * xyz.y()/xyz.z())}, 2, .7);
  point.set_tracking(0x123456789abcdefULL, geometry);
  cv::Mat descriptors(3, 32, CV_8UC1); cv::RNG random(201); random.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  std::vector<VisualPoint> points{point, VisualPoint({10, 20}, 0, .5), point};
  // Duplicate descriptor stays the first appearance-only observation: no borrowed depth.
  descriptors.row(1).copyTo(descriptors.row(2));
  frame.update_features(std::move(points), descriptors);
  const auto payload = database_detail::packVisualObservation(frame);
  check(payload.size() == 148 + 3 * 72 && payload[0] == 'V' && payload[1] == 'O', "versioned fixed layout");
  auto decoded = database_detail::readVisualObservation(1.2, 1, 3, payload.data(), payload.size(), descriptors.clone());
  check(decoded.camera_id() == 1 && decoded.points()[0].track_id() == point.track_id() &&
      (*decoded.T_odom_camera()).matrix() == (*frame.T_odom_camera()).matrix(), "pose and full-width track ID roundtrip");
  check(database_detail::packVisualObservation(decoded) == payload, "byte-exact evidence roundtrip");
  check(!decoded.projection(), "v1 never invents a projection");
  std::vector<VisualFrame> frames; frames.push_back(std::move(decoded));
  const auto scene = buildVisualScene(frames, anchor);
  check(scene->points().size() == 2 && scene->points()[0].has_position && !scene->points()[1].has_position, "mixed geometry and deterministic deduplication");
  const auto &v = scene->points()[0].position;
  check((Eigen::Vector3d(v.x,v.y,v.z) - camera * xyz).norm() < 1e-6, "metric point is in original submap anchor");
  check((Eigen::Vector3d(v.x,v.y,v.z) - anchor * camera * xyz).norm() > 10, "world-frame oracle discriminates");
  VisualFrame legacy(2);
  legacy.update_features({VisualPoint({12,13},1,.3)}, descriptors.row(0).clone());
  auto old = database_detail::packVisualObservation(legacy);
  check(old.size() == 16, "legacy layout preserved");
  auto old_frame = database_detail::readVisualObservation(2,0,1,old.data(),old.size(),legacy.descriptors().clone());
  check(!old_frame.T_odom_camera() && !old_frame.points()[0].geometry() && !old_frame.points()[0].track_id(), "legacy never acquires geometry");
  auto rejects = [&](std::vector<std::uint8_t> bytes, bool update_checksum) {
    if (update_checksum) checksum(bytes);
    bool rejected = false;
    try { (void)database_detail::readVisualObservation(1.2,1,3,bytes.data(),bytes.size(),descriptors.clone()); }
    catch (const std::exception &) { rejected = true; }
    check(rejected, "corrupt/unsupported evidence rejected");
  };
  auto bad = payload; bad.pop_back(); rejects(bad,false);
  bad = payload; bad[150] ^= 1; rejects(bad,false);
  bad = payload; bad[4] = 2; rejects(bad,true); // version
  bad = payload; bad[12] = 2; rejects(bad,true); // pose flag
  bad = payload; bad[8] = 4; rejects(bad,true); // count
  bad = payload; bad[144 + 24] = 2; rejects(bad,true); // geometry flag
  bad = payload; std::fill(bad.begin()+144+16,bad.begin()+144+24,0); rejects(bad,true); // metric without track
  bad = payload; const double nan = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(bad.data()+144+28,&nan,8); rejects(bad,true);
  bad = payload; const double negative = -1;
  std::memcpy(bad.data()+144+28+16,&negative,8); rejects(bad,true); // negative optical depth
  bad = payload; std::fill(bad.begin()+144+68,bad.begin()+144+72,0); rejects(bad,true); // observation count
  bad = payload; const double nonrigid = 2; std::memcpy(bad.data()+16,&nonrigid,8); rejects(bad,true); // nonrigid pose
  frame.set_projection({640,480,{300,310,320,240}});
  const auto v2 = database_detail::packVisualObservation(frame);
  check(v2.size() == 188 + 3*72 && v2[4] == 2, "projection uses version2 layout");
  auto projected = database_detail::readVisualObservation(1.2,1,3,v2.data(),v2.size(),descriptors.clone());
  check(projected.projection() && projected.projection()->width == 640 && projected.projection()->height == 480 &&
        projected.projection()->intrinsics == frame.projection()->intrinsics &&
        database_detail::packVisualObservation(projected) == v2, "frozen projection byte-exact roundtrip");
  bad = v2; bad[4] = 3; rejects(bad,true);
  bad = v2; bad[12] = 1; rejects(bad,true); // version2 requires projection
  bad = v2; std::fill(bad.begin()+144,bad.begin()+148,0); rejects(bad,true);
  bad = v2; std::memcpy(bad.data()+152,&negative,8); rejects(bad,true); // focal length
  bad = v2; std::memcpy(bad.data()+176,&nan,8); rejects(bad,true); // principal point
  std::cout << "visual archive: versioned/legacy evidence, corruption gates and asymmetric visual-only scene geometry passed\n";
} catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
