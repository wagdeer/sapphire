#include "backend/registration/revisit_coverage.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace sapphire;
namespace {
void check(bool good, const char *message) { if (!good) throw std::runtime_error(message); }
template <class F> void invalid(F f) {
  bool rejected = false; try { f(); } catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "invalid model/numerics must be explicit errors");
}
GaussianCloud cloud() {
  GaussianCloud points;
  for (int i = 0; i < 300; ++i) {
    GaussianPoint p; p.N = 20;
    p.mean = Eigen::Vector3f((i % 15) * .61f - 4.01f, ((i / 15) % 10) * .73f - 3.02f, (i / 150) * 1.3f + .04f);
    p.covariance = Eigen::Vector3f(.001f, .01f, .04f).asDiagonal(); p.regularize();
    // Deliberately unrelated to mean: frozen voxel identities belong to producer frames.
    p.voxel_key.x = i * 971; p.voxel_key.y = -i * 11;
    points.push_back(p);
  }
  return points;
}
Eigen::Isometry3d relative() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(.31, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.23, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(7.3, -2.9, 1.2); return T;
}
GaussianCloud transform(const GaussianCloud &points, const Eigen::Isometry3d &T) {
  GaussianCloud out; out.reserve(points.size());
  for (const auto &p : points) out.push_back(p.transformed(T.linear(), T.translation()));
  return out;
}
const RevisitSupport &complete(const RevisitCoverage &r) {
  check(r.status == RevisitCoverageStatus::Complete && bool(r.support), "complete support expected"); return *r.support;
}
} // namespace
int main() try {
  const auto target = cloud(); const auto T = relative(); const auto query = transform(target, T.inverse());
  auto full = measureRevisitCoverage(query, target, T);
  check(complete(full).pairs == target.size() && full.support->query_fraction == 1 && full.support->target_fraction == 1,
        "asymmetric pose and covariance direction, ignoring producer voxel keys");
  check(full.support->supported_target_cells == full.support->target_cells, "complete spatial support");
  auto reverse = measureRevisitCoverage(target, query, T.inverse());
  check(complete(reverse).pairs == full.support->pairs, "inverse relation preserves complete support");
  auto wrong = measureRevisitCoverage(query, target, T.inverse());
  check(complete(wrong).target_fraction < .1, "wrong transform cannot appear redundant");
  GaussianCloud partial(query.begin(), query.begin() + 60);
  auto crop = measureRevisitCoverage(partial, target, T);
  check(complete(crop).query_fraction == 1 && std::abs(crop.support->target_fraction - .2) < 1e-12,
        "excellent one-sided registration is not complete target coverage");
  check(crop.support->supported_target_cells < crop.support->target_cells, "partial view leaves old cells unsupported");
  auto duplicates = partial;
  for (int i = 0; i < 4; ++i) duplicates.insert(duplicates.end(), partial.begin(), partial.end());
  auto density = measureRevisitCoverage(duplicates, target, T);
  check(complete(density).pairs == 60 && density.support->target_fraction == .2 && density.support->query_fraction == .2,
        "duplicate density cannot impersonate wider support");
  auto changed = query;
  for (std::size_t i = 0; i < 30; ++i) changed[i].mean.x() += 30;
  auto change = measureRevisitCoverage(changed, target, T);
  check(complete(change).pairs == 270 && change.support->query_fraction == .9 && change.support->target_fraction == .9,
        "changed structure is unsupported in both directions, not a static-stability prerequisite");
  auto incompatible = target;
  for (auto &point : incompatible) point.covariance = Eigen::Vector3f(.04f, .01f, .001f).asDiagonal();
  auto shape = measureRevisitCoverage(incompatible, target, Eigen::Isometry3d::Identity());
  check(complete(shape).pairs == 0, "coincident means with perpendicular thin distributions are not equivalent");
  auto empty = measureRevisitCoverage({}, target, T);
  check(empty.status == RevisitCoverageStatus::Empty && !empty.support, "missing evidence is not zero-overlap completion");
  RevisitCoverageOptions small; small.max_points = 100;
  auto input = measureRevisitCoverage(query, target, T, small);
  check(input.status == RevisitCoverageStatus::InputBudget && !input.support && input.comparisons == 0, "input budget precedes large work");
  small = {}; small.max_comparisons = 50;
  auto work = measureRevisitCoverage(query, target, T, small);
  check(work.status == RevisitCoverageStatus::ComparisonBudget && work.comparisons == 50 && !work.support,
        "budget exhaustion never publishes a flattering partial fraction");
  invalid([&] { auto bad = target; bad[0].mean.x() = std::numeric_limits<float>::quiet_NaN(); measureRevisitCoverage(bad, target, T); });
  invalid([&] { auto bad = target; bad[0].covariance(0, 0) = -1; measureRevisitCoverage(bad, target, T); });
  invalid([&] { auto bad = target; bad[0].covariance(0, 1) = 1; measureRevisitCoverage(bad, target, T); });
  invalid([&] { auto bad = target; bad[0].N = 0; measureRevisitCoverage(bad, target, T); });
  invalid([&] { auto bad = T; bad.linear()(0, 0) += 1; measureRevisitCoverage(query, target, bad); });
  invalid([&] { auto bad = T; bad.matrix()(3, 0) = 1; measureRevisitCoverage(query, target, bad); });
  invalid([&] { auto bad = T; bad.translation().x() = 1e30; measureRevisitCoverage(query, target, bad); });
  invalid([&] { auto bad = RevisitCoverageOptions{}; bad.mean_distance = 0; measureRevisitCoverage(query, target, T, bad); });
  // Dense adversarial cells remain bounded even though all means are coincident.
  GaussianCloud dense(512, target[0]);
  auto bounded = measureRevisitCoverage(dense, dense, Eigen::Isometry3d::Identity());
  check(bounded.status == RevisitCoverageStatus::ComparisonBudget && bounded.comparisons == 200000 && !bounded.support,
        "dense cells cannot bypass comparison bound");
  GaussianCloud large; large.reserve(32768);
  for (int i = 0; i < 32768; ++i) {
    auto p = target[0]; p.mean = Eigen::Vector3f((i % 128) * .7f, ((i / 128) % 128) * .7f, (i / 16384) * .7f);
    large.push_back(p);
  }
  auto sparse = measureRevisitCoverage(large, large, Eigen::Isometry3d::Identity());
  check(complete(sparse).pairs == large.size() && sparse.comparisons == large.size(),
        "default input envelope handles sparse full-size revisit without quadratic search");
  // Deterministic repeated refresh input: count evidence only, no pretend DB compaction.
  const auto begin = std::chrono::steady_clock::now();
  std::size_t pairs = 0;
  for (int i = 0; i < 100; ++i) {
    auto pose = T; pose.translation() += Eigen::Vector3d(.003 * i, -.001 * i, .0007 * i);
    auto current = transform(target, pose.inverse());
    auto result = measureRevisitCoverage(current, target, pose);
    check(complete(result).pairs == 300, "repeated anchored observations remain fully supported"); pairs += result.support->pairs;
  }
  std::cout << "PASS revisit coverage: complete=300/300 partial=60/300 duplicates=60/300 changed=270/300 incompatible=0/300; repeated_pairs="
            << pairs << " elapsed_ms=" << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count() << '\n';
} catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
