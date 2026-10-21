#ifndef BAYESFILTER_H_
#define BAYESFILTER_H_
#include <backend/common/common_type.hpp>
#include <backend/params/topograph.hpp>

#include <list>
#include <opencv2/core/core.hpp>
#include <set>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>


namespace sapphire::backend {

class Topograph;
class BayesFilter {
 public:
  /**
   * @brief Constructs a Bayes filter with default or custom parameters.
   * @param parameters Optional parameter map (Bayes group keys). Defaults are used for missing keys.
   */
  BayesFilter(const ParametersMap& parameters = ParametersMap());
  explicit BayesFilter(const TopographParams& parameters);
  virtual ~BayesFilter();

  /**
   * @brief Runs one Bayes filter iteration (prediction + update).
   *
   * Given parallel ordered id/likelihood vectors, computes and stores the normalized posterior.
   * Sparse prediction columns are generated or updated from supplied adjacency using @p ids.
   *
   * @param adjacency Resident connectivity snapshot used by the transition model.
   * @param ids Strictly ascending node ids.
   * @param likelihood Observation likelihood aligned one-to-one with @p ids.
   * @return Reference to the internal posterior map (id → probability). On error (empty likelihood or invalid prediction model), returns the unchanged posterior.
   */
  const std::map<int, float>& computePosterior(std::map<int, std::vector<int>> adjacency, const std::set<int>& excluded, const std::vector<int>& ids,
                                               const std::vector<float>& likelihood);

  /**
   * @brief Clears posterior, sparse prediction columns and cached neighbor indices.
   */
  void reset();
  // Owner-thread relation edits invalidate old reachable neighborhoods before mutation.
  void relations_changed(int source) noexcept;
  const std::map<int, float>& computePosterior(const Topograph& graph, const std::set<int>& excluded,
                                              const std::vector<int>& ids, const std::vector<float>& likelihood);

  // Called at the owning graph transaction's commit boundary, before another
  // observation is processed. Fold removed into retained without another Bayes
  // update. The retained place must remain in the next candidate ID set.
  // Returns whether removed was in the posterior (STM nodes often are not).
  // This does not merge Memory, the retrieval index, or the database.
  bool merge_places(int retained, int removed);

  // Discard graph-dependent caches while preserving the current belief. Normal
  // Changed adjacency invalidates only affected neighborhoods.
  void invalidate_prediction();

  /**
   * @brief Returns the current posterior probability map.
   * @return Map of node id to normalized posterior probability. This is the probability to be at the given location.
   */
  const std::map<int, float>& getPosterior() const { return _posterior; }

  /**
   * @brief Returns the virtual place prior threshold.
   * @return Value in [0, 1] used when building the virtual place row of the prediction matrix.
   */
  float getVirtualPlacePrior() const { return _virtualPlacePrior; }

  /**
   * @brief Returns the loop-closure prediction model as a vector of values.
   * @return Vector in the format `{Vp, Lc, l1, l2, l3, ...}`.
   */
  const std::vector<double>& getPredictionLC() const;

  /**
   * @brief Builds or updates the prediction (transition) matrix for the given node ids.
   *
   * Rows and columns correspond to @p ids. Supplied neighbor links fill
   * transition probabilities according to @ref getPredictionLC(). When @p ids match the
   * current prediction keys and adjacency, the cache may be reused
   * unless full prediction updates are requested.
   *
   * @param adjacency Resident connectivity snapshot used by the transition model.
   * @param ids Ordered list of node ids (often includes the virtual-place ID as first element).
   * @return Square CV_32FC1 matrix of size ids.size() × ids.size().
   */
  cv::Mat generatePrediction(std::map<int, std::vector<int>> adjacency, const std::set<int>& excluded, const std::vector<int>& ids);
  // Read-only next-observation prediction. Does not advance or renormalize belief.
  std::vector<int> rank_next_places(size_t limit) const;
  size_t neighbor_computations() const { return neighbor_computations_; }

  /**
   * @brief Estimates memory usage of this object and its internal containers.
   * @return Approximate memory footprint in bytes.
   */
  unsigned long getMemoryUsed() const;

 private:
  friend class Topograph;
  void configure(const TopographParams& parameters);
  struct PredictionColumn {
    float background = 0.0f;
    float toVirtual = 0.0f;
    std::uint64_t backgroundGeneration = 0U;
    std::vector<std::pair<std::uint32_t, float>> overrides;
  };

  /**
   * @brief Incrementally updates the prediction matrix when ids are added or removed.
   */
  void updatePrediction(std::map<int, std::vector<int>> adjacency, const std::set<int>& excluded, const std::vector<int>& ids);
  void clear_prediction_columns();
  void updateColumns(const std::set<int>& excluded, const std::vector<int>& ids, const Topograph* relations = nullptr);
  const std::map<int, float>& updateObservation(const std::vector<int>& ids, const std::vector<float>& likelihood);
  using Adjacency = std::map<int, std::vector<int>>;
  Adjacency graph_snapshot_;
  mutable size_t neighbor_computations_ = 0;
  bool lookahead_ready_ = false;
  std::map<int, int> graph_distances(int origin, const Topograph* relations) const;

  PredictionColumn buildPredictionColumn(unsigned int column, const std::map<int, int>& neighbors, const std::set<int>& excluded,
                                         const std::unordered_map<int, int>& idToIndex, unsigned int count, bool virtualPlaceUsed) const;

  /**
   * @brief Realigns the posterior map with the current set of likelihood ids.
   */
  void updatePosterior(const std::vector<int>& likelihoodIds);

 private:
  std::map<int, float> _posterior;                     ///< Current posterior (node id → probability).
  std::set<int> excluded_snapshot_;
  std::vector<int> _predictionIds;
  std::vector<std::uint64_t> _predictionRowGenerations;
  std::vector<PredictionColumn> _predictionColumns;
  std::uint64_t _predictionGeneration = 0U;
  std::vector<float> _posteriorWorkspace;
  std::vector<double> _priorWorkspace;
  std::vector<double> _backgroundWorkspace;
  float _virtualPlacePrior;                            ///< Prior for virtual place transitions.
  std::vector<double> _predictionLC;                   ///< Model `{Vp, Lc, l1, l2, ...}`.
  bool _fullPredictionUpdate;                          ///< If true, rebuild the full prediction matrix each time.
  float _totalPredictionLCValues;                      ///< Sum of all values in _predictionLC.
  float _predictionEpsilon;                            ///< Minimum non-zero probability in the model.
  std::map<int, std::map<int, int> > _neighborsIndex;  ///< Cached neighbor margins per node id.
};

}  // namespace sapphire::backend

#endif /* BAYESFILTER_H_ */
