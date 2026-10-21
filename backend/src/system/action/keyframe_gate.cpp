#include <algorithm>
#include <array>
#include <backend/system/action/keyframe_gate.hpp>
#include <backend/registration/matcher/projection_match.hpp>
#include <backend/solver/triangulator.hpp>

namespace sapphire::backend {
KeyframeGate::Geometry KeyframeGate::assess(const VisualFeatures& reference, const camera::Perspective& reference_camera,
                                            const VisualFeatures& query, const camera::Perspective& camera,
                                            const StateGroupf& query_from_reference, const std::vector<double>& variances, const MatchOptions& options) {
  Geometry out;
  if (reference.keypoints.empty() || reference.descriptors.empty() || reference.points.size() != reference.keypoints.size() ||
      !reference_camera.isValidForReprojection() || !(reference_camera.focalXBaseline() > 0) ||
      !reference_camera.cameraToBase().isValid()) return out;
  out.image_features = query.keypoints.size();
  if (query.keypoints.empty() || query.descriptors.empty() || query.points.size() != query.keypoints.size() || !camera.isValidForReprojection() ||
      !(camera.focalXBaseline() > 0) || camera.imageSize().width <= 0 || camera.imageSize().height <= 0 || !camera.cameraToBase().isValid() || !query_from_reference.isValid())
    return out;
  std::array<size_t, 16> measured{}, supported{};
  std::vector<uint8_t> eligible(query.keypoints.size());
  const auto cell = [&](const cv::KeyPoint& k) {
    return std::min(3, int(k.pt.y * 4 / camera.imageSize().height)) * 4 + std::min(3, int(k.pt.x * 4 / camera.imageSize().width));
  };
  for (size_t i = 0; i < query.keypoints.size(); ++i) {
    const auto& k = query.keypoints[i];
    if (!std::isfinite(k.pt.x) || !std::isfinite(k.pt.y) || !camera.isInImage(k.pt.x, k.pt.y) || k.octave < 0 || size_t(k.octave) >= variances.size())
      return out;
    // Use exactly the feature-map insertion contract, not all ORB pixels:
    // untriangulated rows cannot become new map content. Low stereo support
    // always keeps a node; uncovered insertable content is checked per region.
    eligible[i] = solver::Triangulator::stereo_seed_valid(camera::stereo_calibration(camera), camera::stereo_pixel(camera, k, query.points[i], std::sqrt(variances[k.octave])));
    out.features += eligible[i];
    measured[cell(k)] += eligible[i];
  }
  out.checked = true;
  if (out.features < options.min_matches) return out;
  const auto matches = featmap::match_fixed_pose(reference, reference_camera, query, camera, query_from_reference, variances, options);
  out.comparisons = matches.comparisons;
  out.spatial_visits = matches.spatial_visits;
  out.inconsistent_depth = matches.inconsistent_depth;
  if (matches.status == featmap::MatchStatus::budget_exceeded) return out;
  for (const auto& m : matches.pairs)
    if (eligible[m.feature]) {
      ++supported[cell(query.keypoints[m.feature])];
      ++out.matches;
    }
  for (size_t i = 0; i < measured.size(); ++i) {
    out.supported_cells += supported[i] > 0;
    out.new_cells += measured[i] > 0 && supported[i] == 0;
  }
  // Query coverage describes redundant insertable measurements, not landmark
  // maturity. New image regions and independently associated conflicting depth
  // both veto absorption; ordinary descriptor ambiguity only reduces support.
  out.redundant = !out.inconsistent_depth && out.matches >= options.min_matches && out.matches * 10 >= out.features * 9 && out.supported_cells >= 6 && out.new_cells == 0;
  return out;
}
}  // namespace sapphire::backend
