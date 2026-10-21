#include <backend/maplayer/featmap.hpp>
#include "backend/maplayer/featmap/internal.hpp"
#include <stdexcept>

namespace sapphire::backend::featmap {
const NodeObservations& FeatureMap::history() const {
  if (!observation_history_) throw std::logic_error("Node observation history is not loaded");
  return *observation_history_;
}
NodeObservations& FeatureMap::edit_history() {
  history();
  if (!observation_history_.unique()) observation_history_ = std::make_shared<NodeObservations>(*observation_history_);
  return *observation_history_;
}
void FeatureMap::set_content(std::shared_ptr<const LandmarkMap> features, NodeObservations history) {
  auto observations = std::make_shared<NodeObservations>(std::move(history));
  if (!features) throw std::invalid_argument("Missing map features");
  for (const auto& o : observations->observations)
    if (o.appearance.descriptor.model_id() != features->descriptor_model_id() || o.appearance.descriptor.dimensions() != features->descriptor_dimensions())
      throw std::invalid_argument("Map/history descriptor model mismatch");
  map_features_ = std::move(features);
  observation_history_ = std::move(observations);
}
void FeatureMap::retire_unsupported(const std::set<uint64_t>& previously_observed) {
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  set_points(detail::retire_unsupported(*map_features_, edit_history(), previously_observed));
}
void FeatureMap::prune_views(int anchor, float pyramid_scale, const std::set<int>& protected_observers) {
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  edit_history().prune_redundant_views(anchor, *map_features_, pyramid_scale, protected_observers);
}
void FeatureMap::update_descriptors(const std::map<int, StateGroupf>& boundaries) {
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  set_points(representative_descriptors(*map_features_, history(), boundaries));
}
void FeatureMap::reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected) {
  if (rejected.empty()) return;
  if (!map_features_) throw std::logic_error("Node points are not loaded");
  auto history = this->history();
  std::set<uint64_t> previously_observed;
  for (const auto& observation : history.observations) previously_observed.insert(observation.landmark);
  history.reject_observations(rejected);
  // A public owner edit must be a publishable history, not an intermediate BA
  // draft. In particular, it cannot silently discard a still-needed datum.
  history.validate();
  auto points = map_features_->points();
  detail::retire_unsupported_points(points, history, previously_observed);
  detail::update_representative_descriptors(points, history, history.observation_poses);
  auto features = LandmarkMap::create(map_features_->vocabulary_hash(), std::move(points), map_features_->max_points(), map_features_->descriptor_model_id(), map_features_->descriptor_dimensions());
  set_content(std::move(features), std::move(history));
}
}  // namespace sapphire::backend::featmap
