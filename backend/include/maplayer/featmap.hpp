#pragma once
#include <backend/maplayer/featmap/observations.hpp>

namespace sapphire::backend::featmap {
struct Match;

// Feature layer of one Node. An empty resident cache is valid: its summary
// stays on the Node and archives stay in the database. Copies share immutable points
// and detach observation history only when a draft changes it.
class FeatureMap {
 public:
  const VisualFrame& reference_frame() const { return reference_frame_; }
  void set_reference_frame(VisualFrame frame) {
    if (frame.shared_features() != loop_features_) loop_features_.reset();
    reference_frame_ = std::move(frame);
  }
  const std::shared_ptr<const bow::FeatureBlock>& loop_features() const { return loop_features_; }
  void set_loop_features(std::shared_ptr<const bow::FeatureBlock> features) {
    reference_frame_.share_features(features);
    loop_features_ = std::move(features);
  }
  void clear_reference_features() { set_loop_features({}); }
  const std::vector<GlobalDescriptor>& global_descriptors() const { return global_descriptors_; }
  void set_global_descriptors(std::vector<GlobalDescriptor> descriptors) { global_descriptors_ = std::move(descriptors); }
  const std::shared_ptr<const LandmarkMap>& landmarks() const { return map_features_; }
  bool has_history() const { return bool(observation_history_); }
  const NodeObservations& history() const;
  void clear() { map_features_.reset(); observation_history_.reset(); }
  void set_landmarks(std::shared_ptr<const LandmarkMap> points) {
    map_features_ = std::move(points);
    observation_history_.reset();
  }
  void set_content(std::shared_ptr<const LandmarkMap> points, NodeObservations history);
  size_t memory_size() const {
    size_t global_bytes = global_descriptors_.capacity() * sizeof(GlobalDescriptor);
    for (const auto& descriptor : global_descriptors_) {
      global_bytes += descriptor.data().total() * descriptor.data().elemSize() + descriptor.info().total() * descriptor.info().elemSize();
    }
    const size_t index_bytes = loop_features_ ? loop_features_->memory_size() - loop_features_->VisualFeatures::memory_size() : 0;
    return global_bytes + reference_frame_.getMemoryUsed() - sizeof(VisualFrame) + index_bytes + (map_features_ ? map_features_->memory_size() : 0) + (observation_history_ ? observation_history_->memory_size() : 0);
  }
  void reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected);
  void replace_landmark(uint64_t removed, uint64_t retained);
  void retire_unsupported(const std::set<uint64_t>& previously_observed);
  void fuse_landmarks(const VisualFrame&, const std::shared_ptr<const bow::FeatureBlock>&, const StateGroupf& query_from_map, const std::vector<Match>& seeds);
  // Only call for a geometrically accepted revisit with a completed association search.
  void maintain_landmarks(const VisualFrame&, const std::shared_ptr<const bow::FeatureBlock>&, int capture, const StateGroupf& query_from_map,
                          const std::vector<double>& pixel_variances, const std::map<uint64_t, uint32_t>& verified_rows = {});
  void prune_views(int anchor, float pyramid_scale, const std::set<int>& protected_observers);
  void update_descriptors(const std::map<int, StateGroupf>& poses);

 private:
  VisualFrame reference_frame_;
  std::shared_ptr<const bow::FeatureBlock> loop_features_;
  std::vector<GlobalDescriptor> global_descriptors_;
  NodeObservations& edit_history();
  void set_points(std::shared_ptr<const LandmarkMap> points) { map_features_ = std::move(points); }
  std::shared_ptr<const LandmarkMap> map_features_;
  std::shared_ptr<NodeObservations> observation_history_;
};
}  // namespace sapphire::backend::featmap
