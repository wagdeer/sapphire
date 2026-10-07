#include "backend/visual/solver/visual_pnp.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>

using namespace sapphire;
namespace {
void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
template<class F> void invalid(F f) {
  bool caught = false;
  try { f(); } catch (const std::invalid_argument &) { caught = true; }
  check(caught, "malformed contract must throw");
}
struct Fixture {
  CameraParameters camera;
  Eigen::Isometry3d T_target_query = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_query_camera = Eigen::Isometry3d::Identity();
  std::shared_ptr<const mapping::scene::FeatureMap> target;
  std::shared_ptr<const features::FeatureBlock> query;
  std::vector<mapping::scene::Match> matches;
  Fixture(bool quantized = false, int geometry = 0, bool appearance_only = false, bool outliers = false) {
    camera.width = 640; camera.height = 480; camera.intrinsics = {420, 430, 318, 239};
    // Pixels below already rectified: nonzero raw calibration must not be reapplied.
    camera.distortion = {-.2, .03, .001, -.002, 0};
    T_target_query.linear() = Eigen::AngleAxisd(.43, Eigen::Vector3d(1, -2, 3).normalized()).toRotationMatrix();
    T_target_query.translation() = Eigen::Vector3d(2.4, -1.3, .7);
    T_query_camera.linear() = Eigen::AngleAxisd(-.31, Eigen::Vector3d(-2, 1, 1).normalized()).toRotationMatrix();
    T_query_camera.translation() = Eigen::Vector3d(.37, -.24, .15);
    std::vector<mapping::scene::Landmark> landmarks;
    std::vector<cv::KeyPoint> pixels;
    std::vector<std::uint32_t> nodes;
    std::vector<int> words;
    cv::Mat descriptors(120, 32, CV_8UC1);
    cv::RNG rng(72351); rng.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
    for (int i = 0; i < 120; ++i) {
      double u = 40 + (i % 12) * 50, v = 40 + (i / 12) * 44;
      if (geometry == 2) { u = 290 + i%12; v = 210 + i/12; } // confined image support
      if (geometry == 3) { u = 70 + i*4; v = 70 + i*2.7; } // collinear XYZ at constant depth
      const double depth = geometry == 1 || geometry == 3 ? 6 : rng.uniform(4., 11.);
      const Eigen::Vector3d pc((u-318)*depth/420, (v-239)*depth/430, depth);
      const Eigen::Vector3d pt = T_target_query * T_query_camera * pc;
      mapping::scene::Landmark point;
      point.id = 300 + i; point.has_position = !appearance_only;
      point.position = cv::Point3f(pt.x(), pt.y(), pt.z()); point.appearance_count = 1;
      point.appearances[0].quality = .8f;
      std::copy_n(descriptors.ptr<std::uint8_t>(i), 32, point.appearances[0].descriptor.begin());
      point.appearances[0].tree_node = (i*7)%11 + 1; point.appearances[0].word_id = i%11;
      landmarks.push_back(point);
      if (outliers && i%4 == 0) { u = rng.uniform(20., 620.); v = rng.uniform(20., 460.); }
      else if (outliers) { u += rng.gaussian(.2); v += rng.gaussian(.2); }
      pixels.emplace_back(cv::Point2f(u, v), 31);
      nodes.push_back(point.appearances[0].tree_node); words.push_back(i%11);
    }
    target = mapping::scene::FeatureMap::create(quantized ? 51 : 0, std::move(landmarks), 120);
    query = quantized ? features::FeatureBlock::create(51, nodes, words, pixels, {}, descriptors) :
        features::FeatureBlock::create_raw(pixels, {}, descriptors);
    for (std::uint32_t row = 0; row < 120; ++row) matches.push_back({query->feature_ids[row], row, 0, 0});
  }
  VisualPnpResult run(VisualPnpParameters p = {}) const {
    return estimateVisualPnpSeed(*target, *query, matches, camera, T_query_camera, p);
  }
};
void seedCases() {
  for (bool quantized : {false, true}) for (bool noisy : {false, true}) {
    Fixture f(quantized, 0, false, noisy);
    auto r = f.run();
    check(r.status == VisualPnpStatus::seed && r.T_target_query, "metric seed recovered");
    const double error = (r.T_target_query->matrix() - f.T_target_query.matrix()).norm();
    check(error < (noisy ? .015 : 1e-5), "asymmetric target-from-query direction and image-time pose");
    check((r.T_target_query->inverse().matrix() - f.T_target_query.matrix()).norm() > 1, "inverse oracle discriminates");
    check(r.inliers.size() >= (noisy ? 88u : 120u) && r.occupied_cells >= 10, "broad refined consensus");
    for (auto m : r.inliers) {
      check(m.landmark == f.query->feature_ids[m.feature], "inlier row indexes exact sorted query");
      if (noisy) check(m.landmark%4 != 0, "gross outliers not retained");
    }
    std::cout << "seed quantized=" << quantized << " noisy=" << noisy << " error=" << error
              << " inliers=" << r.inliers.size() << " rms=" << r.rms_reprojection_px << '\n';
  }
  Fixture fisheye;
  fisheye.camera.distortion_model = "equidistant";
  fisheye.camera.distortion = {-.04, -.009, .009, -.004};
  const auto fisheye_seed = fisheye.run();
  check(fisheye_seed.T_target_query && (fisheye_seed.T_target_query->matrix()-fisheye.T_target_query.matrix()).norm() < 1e-5,
        "already rectified equidistant observations are not undistorted again by PnP");
  Fixture plane(false, 1);
  auto r = plane.run();
  check(r.T_target_query && (r.T_target_query->matrix()-plane.T_target_query.matrix()).norm() < 1e-4,
        "spatially broad planar support may supply a seed, not loop acceptance");
}
void rejectCases() {
  Fixture f;
  auto wrong = f.matches;
  std::vector<std::uint32_t> permutation(120); std::iota(permutation.begin(), permutation.end(), 0);
  std::mt19937 rng(52); std::shuffle(permutation.begin(), permutation.end(), rng);
  for (std::size_t i = 0; i < wrong.size(); ++i) wrong[i].landmark = permutation[i];
  auto r = estimateVisualPnpSeed(*f.target, *f.query, wrong, f.camera, f.T_query_camera);
  check(!r.T_target_query && r.inliers.empty(), "unrelated correspondence candidate has no seed");
  check(Fixture(false, 0, true).run().status == VisualPnpStatus::insufficient_geometry, "legacy appearance has no invented metric seed");
  check(!Fixture(false, 2).run().T_target_query, "localized image patch rejected");
  VisualPnpParameters p; p.min_occupied_cells = 1;
  check(!Fixture(false, 3).run(p).T_target_query, "collinear geometry rejected independently of image coverage");
  p = {}; p.max_pairs = 100;
  r = f.run(p);
  check(r.status == VisualPnpStatus::pair_budget && !r.T_target_query && !r.metric_pairs, "pair exhaustion has no partial result");
  auto malformed = f.matches; malformed.back().feature = malformed.front().feature;
  invalid([&] { estimateVisualPnpSeed(*f.target, *f.query, malformed, f.camera, f.T_query_camera); });
  malformed = f.matches; malformed.back().landmark = 120;
  invalid([&] { estimateVisualPnpSeed(*f.target, *f.query, malformed, f.camera, f.T_query_camera); });
  malformed = f.matches; malformed.back().appearance = 1;
  invalid([&] { estimateVisualPnpSeed(*f.target, *f.query, malformed, f.camera, f.T_query_camera); });
  auto bad_pose = f.T_query_camera; bad_pose.linear()(0, 0) += .1;
  invalid([&] { estimateVisualPnpSeed(*f.target, *f.query, f.matches, f.camera, bad_pose); });
  p = {}; p.max_reprojection_px = NAN;
  invalid([&] { f.run(p); });
}
}  // namespace
int main() {
  try { cv::setNumThreads(1); seedCases(); rejectCases(); std::cout << "visual PnP tests passed\n"; return 0; }
  catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
