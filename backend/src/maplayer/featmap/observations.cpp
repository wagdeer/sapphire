#include <algorithm>
#include <backend/maplayer/featmap/observations.hpp>
#include <stdexcept>

#include "backend/maplayer/featmap/validation.hpp"

namespace sapphire::backend::featmap {
namespace detail {

void validate_local_poses(const NodeObservations& bundle) {
  for (const auto& [id, p] : bundle.observation_poses)
    if (id <= 0 || !p.isValid() || !p.translationVector().allFinite() || !p.rotationMatrix().allFinite() ||
        !(p.rotationMatrix().transpose() * p.rotationMatrix()).isApprox(Eigen::Matrix3f::Identity(), 1e-4))
      throw std::runtime_error("Invalid submap-local pose provenance");
  if (!bundle.observation_poses.empty()) {
    const auto ref = bundle.observation_poses.find(bundle.reference_node);
    if (bundle.reference_node <= 0 || ref == bundle.observation_poses.end() || ref->second.translationDistance(StateGroupf::identity()) > 1e-6 ||
        !ref->second.rotationMatrix().isApprox(Eigen::Matrix3f::Identity(), 1e-6))
      throw std::runtime_error("Invalid submap-local datum");
  }
}
void validate_bundle_values(const NodeObservations& bundle) {
  const auto model = bundle.observations.empty() ? 0 : bundle.observations.front().appearance.descriptor.model_id();
  const int dimensions = bundle.observations.empty() ? 32 : bundle.observations.front().appearance.descriptor.dimensions();
  for (const auto& o : bundle.observations) {
    if (o.appearance.descriptor.model_id() != model || o.appearance.descriptor.dimensions() != dimensions)
      throw std::runtime_error("Mixed observation descriptor models");
    if (o.appearance.descriptor.model_id() != 0 && (o.appearance.word_id != -1 || o.appearance.tree_node != bow::FeatureBlock::invalid_node_id))
      throw std::runtime_error("Learned observation has binary vocabulary provenance");
    if (!o.stereo.valid()) throw std::runtime_error("Invalid submap measurement provenance");
    const auto& coefficients = o.stereo.anchor_from_camera().coefficients();
    if (!std::all_of(coefficients.begin(), coefficients.end(), [](float x) { return std::isfinite(x); }) ||
        !std::all_of(o.stereo.calibration().begin(), o.stereo.calibration().end(), [](float x) { return std::isfinite(x); }))
      throw std::runtime_error("Nonfinite observation camera model");
  }
  for (int capture : bundle.maintenance_captures)
    if (!bundle.observation_poses.count(capture)) throw std::runtime_error("Reliability capture lacks placement");
  for (const auto& [id, state] : bundle.reliability) {
    if (!id || state.trials.size() > 8) throw std::runtime_error("Invalid landmark reliability history");
    std::set<int> captures;
    for (const auto& [capture, hit] : state.trials)
      if (!bundle.observation_poses.count(capture) || !captures.insert(capture).second)
        throw std::runtime_error("Invalid repeated reliability trial");
  }
  validate_local_poses(bundle);
}
static std::map<uint64_t, uint32_t> observation_support(const NodeObservations& bundle) {
  // Validation needs uniqueness and support counts, not two nested lookup
  // trees containing every observation. Sort compact keys in contiguous buffers.
  std::vector<std::pair<uint64_t, int>> observations;
  std::vector<std::pair<int, uint32_t>> pixels;
  observations.reserve(bundle.observations.size());
  pixels.reserve(bundle.observations.size());
  for (const auto& o : bundle.observations) {
    if (o.node <= 0 || !o.landmark) throw std::runtime_error("Invalid submap observation identity");
    observations.emplace_back(o.landmark, o.node);
    if (o.feature_id != std::numeric_limits<uint32_t>::max()) pixels.emplace_back(o.node, o.feature_id);
  }
  std::sort(observations.begin(), observations.end());
  std::sort(pixels.begin(), pixels.end());
  if (std::adjacent_find(observations.begin(), observations.end()) != observations.end())
    throw std::runtime_error("Duplicate source-submap landmark observation");
  if (std::adjacent_find(pixels.begin(), pixels.end()) != pixels.end()) throw std::runtime_error("One source pixel assigned to multiple landmarks");
  std::map<uint64_t, uint32_t> support;
  for (size_t first = 0; first < observations.size();) {
    size_t end = first + 1;
    while (end < observations.size() && observations[end].first == observations[first].first) ++end;
    support.emplace_hint(support.end(), observations[first].first, uint32_t(end - first));
    first = end;
  }
  return support;
}
std::map<uint64_t, uint32_t> validate_bundle_structure(const NodeObservations& bundle) {
  auto support = observation_support(bundle);
  validate_bundle_values(bundle);
  for (const auto& [id, state] : bundle.reliability)
    if (!support.count(id)) throw std::runtime_error("Orphan landmark reliability history");
  return support;
}
}  // namespace detail
using detail::validate_bundle_structure;

NodeObservations::Index NodeObservations::index() const {
  NodeObservations::Index result;
  std::set<std::pair<int, uint32_t>> pixels;
  for (size_t index = 0; index < observations.size(); ++index) {
    const auto& o = observations[index];
    if (o.node <= 0 || !o.landmark) throw std::runtime_error("Invalid submap observation identity");
    auto& frame = result.frames[o.node];
    auto& track = result.landmarks[o.landmark];
    if (!frame.emplace(o.landmark, index).second || !track.emplace(o.node, index).second)
      throw std::runtime_error("Duplicate source-submap landmark observation");
    if (o.feature_id != std::numeric_limits<uint32_t>::max() && !pixels.emplace(o.node, o.feature_id).second)
      throw std::runtime_error("One source pixel assigned to multiple landmarks");
  }
  return result;
}
void NodeObservations::refresh_support() {
  auto current = detail::observation_support(*this);
  for (auto it = reliability.begin(); it != reliability.end();)
    if (!current.count(it->first))
      it = reliability.erase(it);
    else
      ++it;
  support = std::move(current);
  if (!support.empty()) landmark_high_water = std::max(landmark_high_water, support.rbegin()->first);
}
void NodeObservations::reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected) {
  const std::set<std::pair<int, uint64_t>> keys(rejected.begin(), rejected.end());
  observations.erase(std::remove_if(observations.begin(), observations.end(),
                                    [&](const auto& o) {
                                      return keys.count({o.node, o.landmark});
                                    }),
                     observations.end());
  refresh_support();
}
size_t NodeObservations::memory_size() const {
  size_t appearance_bytes = 0;
  for (const auto& o : observations) appearance_bytes += o.appearance.descriptor.dynamic_bytes();
  std::vector<const camera::StereoFrame*> frames;
  frames.reserve(observations.size());
  for (const auto& o : observations) frames.push_back(o.stereo.frame.get());
  size_t reliability_bytes = maintenance_captures.size() * (sizeof(int) + 3 * sizeof(void*));
  for (const auto& [id, state] : reliability)
    reliability_bytes +=
        sizeof(decltype(reliability)::value_type) + 3 * sizeof(void*) + state.trials.capacity() * sizeof(decltype(state.trials)::value_type);
  return appearance_bytes + camera::stereo_frame_memory(std::move(frames)) + sizeof(*this) + reliability_bytes +
         observations.capacity() * sizeof(Observation) + support.size() * (sizeof(decltype(support)::value_type) + 3 * sizeof(void*)) +
         observation_poses.size() * (sizeof(decltype(observation_poses)::value_type) + 3 * sizeof(void*));
}
void NodeObservations::validate() const {
  validate_bundle_structure(*this);
  // v7 binary records fit in 1024 bytes; v8 learned records fit in 2048,
  // including 9-byte CBOR scalar encodings and the 1024-byte descriptor.
  // Reserve another 1024 bytes for map keys/headers. This is only a fast proof
  // of fitting the limit: larger drafts use the exact encoder, never a stricter
  // approximate rejection. Keep this bound in sync with the archive schema.
  const bool stores_model =
      std::any_of(observations.begin(), observations.end(), [](const auto& o) { return o.appearance.descriptor.model_id() != 0; });
  if (observations.size() + observation_poses.size() + reliability.size() + maintenance_captures.size() >
      size_t(stores_model ? 8 * 1024 - 1 : 16 * 1024 - 1))
    encode();
}
void NodeObservations::prune_redundant_views(int anchor, const LandmarkMap& features, float pyramid_scale, const std::set<int>& protected_observers) {
  if (!std::isfinite(pyramid_scale) || pyramid_scale < 1) throw std::invalid_argument("Invalid submap feature scale");
  std::set<uint64_t> ids;
  for (const auto& p : features.points()) ids.insert(p.id);
  std::map<int, std::map<uint64_t, size_t>> sources;
  for (size_t row = 0; row < observations.size(); ++row) {
    const auto& o = observations[row];
    if (ids.count(o.landmark)) sources[o.node].emplace(o.landmark, row);
  }
  // Cull a redundant original source only when >=90% of its
  // points must have >=3 other observers at an equal/better octave (+1).
  // The caller protects active observers explicitly. Numeric capture IDs do
  // not encode spatial adjacency or observation age after region consolidation.
  auto protected_nodes = protected_observers;
  protected_nodes.insert(anchor);
  for (auto it = sources.begin(); it != sources.end();) {
    if (protected_nodes.count(it->first)) {
      ++it;
      continue;
    }
    size_t redundant = 0;
    bool orphan = false;
    for (const auto& [point, row] : it->second) {
      const auto& o = observations[row];
      size_t observers = 0, better = 0;
      for (const auto& [node, track] : sources) {
        if (node == it->first) continue;
        const auto other = track.find(point);
        if (other == track.end()) continue;
        ++observers;
        if (observations[other->second].stereo.sigma() <= o.stereo.sigma() * pyramid_scale + 1e-6f) ++better;
        // Three better observers prove both redundancy and non-orphan status.
        if (better == 3) break;
      }
      orphan |= observers == 0;
      redundant += better >= 3;
    }
    // Preserve sole observers of unseen static points independently of the
    // number of poses selected for the next local optimization.
    if (!orphan && !it->second.empty() && redundant * 10 >= it->second.size() * 9)
      it = sources.erase(it);
    else
      ++it;
  }
  std::vector<NodeObservations::Observation> retained;
  retained.reserve(observations.size());
  for (const auto& [node, track] : sources) {
    for (const auto& [id, row] : track) retained.push_back(observations[row]);
  }
  observations = std::move(retained);
  for (auto it = support.begin(); it != support.end();)
    if (!ids.count(it->first))
      it = support.erase(it);
    else
      ++it;
  // Capture placement outlives its bounded BA pixels: navigation and graph endpoints still use it.
  if (!observation_poses.empty() && !observation_poses.count(reference_node)) throw std::runtime_error("Node datum observations were lost");
  refresh_support();
}
}  // namespace sapphire::backend::featmap
