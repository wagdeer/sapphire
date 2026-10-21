#pragma once
#include <backend/registration/utils/rknn_models.hpp>
#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/topograph.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/maplayer/topograph/link.hpp>
#include <backend/maplayer/topograph/visit_history.hpp>
#include <backend/system/action/visual_registration.hpp>
#include <unordered_map>


namespace sapphire::backend {
class Memory;
struct Relocalization {
  std::vector<int> ids, predicted;
  std::vector<int> visit_predictions;
  bool visits_reordered = false;
  std::vector<int> scene_candidates;
  std::vector<float> scene_candidate_scores;  // Cosine evidence aligned with the posterior-ranked appearance shortlist.
  double semantic_seconds = 0;
  std::vector<float> scores;
  std::vector<float> raw_scene_scores;  // ID-aligned cosine evidence before likelihood adjustment.
  std::map<int, float> posterior;
  std::set<int> loaded, protected_ids;
  PlaceHypothesis hypothesis;
  int qualified_id = 0;  // appearance nominated; not geometric or graph acceptance
  int geometry_verified_id = 0;
  int committed_id = 0;
  int retrieval_id = 0, protected_global = 0, protected_local = 0, protected_local_limit = 0;
  float ratio = 0;
  bool rejected = false;
  double database_seconds = 0, likelihood_seconds = 0, posterior_seconds = 0;
  double selection_seconds = 0, validation_seconds = 0;
};

class Relocalizer final {
 public:
  Relocalizer(Memory& memory, const ParametersMap& parameters = {});
  Relocalizer(Memory& memory, const RelocalizerParams& retrieval, const RegistrationParams& registration, const FeatureParams& feature, const PreprocessParams& preprocess, float protected_local_ratio, bool metric_enabled, std::shared_ptr<feature::RknnModels> models = {});
  ~Relocalizer();
  void reset();
  void restore_visits();
  void save_visits() const;
  void record_visit(int query, int session, double stamp, int mapped_place, int committed_loop);
  void prioritize_prefetch(int session, double stamp, std::vector<int>& predicted) const;
  size_t visit_count() const { return visits_.size(); }
  Relocalization recognize(const Node&, PlaceHypothesis previous, const std::map<int, StateGroupf>& poses);
  void retrieve_neighbors(const Node&, const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints, bool memory_limited,
                          bool any_transferred, Relocalization&);
  static void adjust_likelihood(const std::vector<int>& ids, std::vector<float>& scores,
                                int virtual_place_ratio = sapphire::backend::RelocalizerParams{}.virtual_place_likelihood_method);
  size_t memory_size() const;
  bool inverted() const { return config_.invert_registration; }
  const VisualRegistration& geometry() const { return geometry_; }
  // cold_only filters before ranking, so resident leaders do not consume I/O slots.
  std::vector<int> candidates(const Node& query, size_t limit, bool cold_only = false) const;
  // Pose/appearance metadata only: no graph paths or feature-bank loads.
  std::vector<int> spatial_candidates(const Node& query, const std::map<int, StateGroupf>& poses,
                                      const Relocalization& recognition, const std::set<int>& excluded,
                                      size_t* insufficient_support = nullptr) const;
  // BoW ids must be strictly ascending. Scores stay aligned with caller IDs.
  void score(const Node* query, const std::vector<int>& ids, std::vector<float>& scores) const;
  Registration verify(const Node& source, const Node& target, StateGroupf guess, RegistrationTrace* trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;
  Registration verify(int source, int target, StateGroupf guess, RegistrationTrace* trace = nullptr, std::optional<uint64_t> pnp_seed = std::nullopt) const;

 private:
  std::vector<int> visit_predictions(int session, double stamp) const;
  RelocalizerParams config_;
  RegistrationParams lidar_registration_;
  VisitHistory visits_;
  Memory& memory_;
  float _localImmunizationRatio = sapphire::backend::MemoryParams{}.protected_local_ratio;
  bool _rgbdSlamMode = sapphire::backend::TopographParams{}.metric_enabled;
  VisualRegistration geometry_;
  mutable std::unordered_map<int, size_t> score_positions_;
};
}  // namespace sapphire::backend
