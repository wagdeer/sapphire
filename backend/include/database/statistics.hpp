#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <backend/common/state_group.hpp>
#include <backend/maplayer/topograph/link.hpp>
#include <backend/maplayer/node.hpp>
#include <backend/params/statistics.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <optional>
#include <vector>

namespace sapphire::backend {

#define RTABMAP_STATS(PREFIX, NAME, UNIT)                                                                                  \
 public:                                                                                                                   \
  static std::string k##PREFIX##NAME() { return #PREFIX "/" #NAME "/" #UNIT; }                                             \
                                                                                                                           \
 private:                                                                                                                  \
  class Dummy##PREFIX##NAME {                                                                                              \
   public:                                                                                                                 \
    Dummy##PREFIX##NAME() {                                                                                                \
      if (!_defaultDataInitialized) _defaultData.insert(std::pair<std::string, float>(#PREFIX "/" #NAME "/" #UNIT, 0.0f)); \
    }                                                                                                                      \
  };                                                                                                                       \
  Dummy##PREFIX##NAME dummy##PREFIX##NAME

/**
 * @class Statistics
 * @brief Collects and manages runtime statistics for RTAB-Map
 *
 * The Statistics class provides a comprehensive system for collecting, storing, and managing
 * various runtime statistics about RTAB-Map's operation. Statistics are organized into groups
 * such as:
 * - **Loop**: Loop closure detection, hypothesis validation, optimization errors
 * - **Memory**: Working memory size, database usage, node management
 * - **Timing**: Performance measurements for various operations
 * - **Keypoint**: Visual word dictionary and feature statistics
 *
 * Statistics are stored in a map with keys in the format "Group/Name/Unit" (e.g., "Timing/Total time/ms").
 * This hierarchical naming allows for easy categorization and plotting.
 *
 * The class supports two modes:
 * - **Basic mode** (`extended() == false`): Only stores loop closure and last node ID fields
 * - **Extended mode** (`extended() == true`): Stores all available statistics including poses,
 *   constraints, likelihoods, posteriors, and detailed timing information
 *
 * Statistics can be serialized to/from strings for storage in databases or transmission over networks.
 *
 * @note Statistics are typically created by RTAB-Map's core components (Mapping, Memory, etc.)
 *       and can be retrieved for analysis, visualization, or debugging purposes.
 *
 */
class Statistics {
  RTABMAP_STATS(Loop, Id, );  // Accepted appearance loop detection
  RTABMAP_STATS(Loop, RejectedHypothesis, );
  RTABMAP_STATS(Loop, Accepted_hypothesis_id, );
  RTABMAP_STATS(Loop, Highest_hypothesis_id, );
  RTABMAP_STATS(Loop, Known_place_probability, );  // 1-P(virtual), not the best-place posterior
  RTABMAP_STATS(Loop, Vp_hypothesis, );
  RTABMAP_STATS(Loop, Reactivate_id, );
  RTABMAP_STATS(Loop, Hypothesis_ratio, );
  RTABMAP_STATS(Loop, Hypothesis_reactivated, );
  RTABMAP_STATS(Loop, Map_id, );
  RTABMAP_STATS(Loop, Bow_comparison_bound, );
  RTABMAP_STATS(Loop, Bow_comparisons, );
  RTABMAP_STATS(Loop, Visual_words, );
  RTABMAP_STATS(Loop, Visual_inliers, );
  RTABMAP_STATS(Loop, Visual_inliers_ratio, );
  RTABMAP_STATS(Loop, Visual_matches, );
  RTABMAP_STATS(Loop, Visual_variance, );
  RTABMAP_STATS(Loop, Distance_since_last_closure, m);
  RTABMAP_STATS(Loop, Last_id, );
  RTABMAP_STATS(Loop, Optimization_max_error, m);
  RTABMAP_STATS(Loop, Optimization_max_error_ratio, );
  RTABMAP_STATS(Loop, Optimization_max_ang_error, deg);
  RTABMAP_STATS(Loop, Optimization_max_ang_error_ratio, );
  RTABMAP_STATS(Loop, Optimization_error, );
  RTABMAP_STATS(Loop, Optimization_iterations, );
  RTABMAP_STATS(Loop, Optimization_max_error_from_id, );
  RTABMAP_STATS(Loop, Optimization_max_error_to_id, );
  RTABMAP_STATS(Loop, Optimization_max_ang_error_from_id, );
  RTABMAP_STATS(Loop, Optimization_max_ang_error_to_id, );
  RTABMAP_STATS(Loop, Optimization_max_error_removed_from_id, );
  RTABMAP_STATS(Loop, Optimization_max_error_removed_to_id, );
  RTABMAP_STATS(Loop, Optimization_max_error_removed_count, );
  RTABMAP_STATS(Loop, Linear_variance, );
  RTABMAP_STATS(Loop, Angular_variance, );
  RTABMAP_STATS(Loop, Visual_inliers_mean_dist, m);
  RTABMAP_STATS(Loop, Visual_inliers_distribution, );
  RTABMAP_STATS(Loop, Proximity_links_cleared, );
  // Odom correction
  RTABMAP_STATS(Loop, Odom_correction_norm, m);
  RTABMAP_STATS(Loop, Odom_correction_angle, deg);
  RTABMAP_STATS(Loop, Odom_correction_x, m);
  RTABMAP_STATS(Loop, Odom_correction_y, m);
  RTABMAP_STATS(Loop, Odom_correction_z, m);
  RTABMAP_STATS(Loop, Odom_correction_roll, deg);
  RTABMAP_STATS(Loop, Odom_correction_pitch, deg);
  RTABMAP_STATS(Loop, Odom_correction_yaw, deg);
  // Map to Odom
  RTABMAP_STATS(Loop, MapToOdom_norm, m);
  RTABMAP_STATS(Loop, MapToOdom_angle, deg);
  RTABMAP_STATS(Loop, MapToOdom_x, m);
  RTABMAP_STATS(Loop, MapToOdom_y, m);
  RTABMAP_STATS(Loop, MapToOdom_z, m);
  RTABMAP_STATS(Loop, MapToOdom_roll, deg);
  RTABMAP_STATS(Loop, MapToOdom_pitch, deg);
  RTABMAP_STATS(Loop, MapToOdom_yaw, deg);
  // Map to Base
  RTABMAP_STATS(Loop, MapToBase_x, m);
  RTABMAP_STATS(Loop, MapToBase_y, m);
  RTABMAP_STATS(Loop, MapToBase_z, m);
  RTABMAP_STATS(Loop, MapToBase_roll, deg);
  RTABMAP_STATS(Loop, MapToBase_pitch, deg);
  RTABMAP_STATS(Loop, MapToBase_yaw, deg);
  RTABMAP_STATS(Loop, MapToBase_lin_std, m);
  RTABMAP_STATS(Loop, MapToBase_lin_var, m2);

  RTABMAP_STATS(Memory, Working_memory_size, );
  RTABMAP_STATS(Memory, Working_memory_inter_size, );
  RTABMAP_STATS(Memory, Short_time_memory_size, );
  RTABMAP_STATS(Memory, Short_time_memory_inter_size, );
  RTABMAP_STATS(Memory, Database_memory_used, MB);
  RTABMAP_STATS(Memory, Nodes_removed, );
  RTABMAP_STATS(Memory, Immunized_globally, );
  RTABMAP_STATS(Memory, Immunized_locally, );
  RTABMAP_STATS(Memory, Immunized_locally_max, );
  RTABMAP_STATS(Memory, Nodes_retrieved, );
  RTABMAP_STATS(Memory, Images_buffered, );
  RTABMAP_STATS(Memory, Rehearsal_sim, );
  RTABMAP_STATS(Memory, Rehearsal_id, );
  RTABMAP_STATS(Memory, Rehearsal_merged, );
  RTABMAP_STATS(Memory, Local_graph_size, );
  RTABMAP_STATS(Memory, Odometry_variance_ang, );
  RTABMAP_STATS(Memory, Odometry_variance_lin, );
  RTABMAP_STATS(Memory, Distance_travelled, m);
  RTABMAP_STATS(Memory, RAM_usage, MB);
  RTABMAP_STATS(Memory, RAM_estimated, MB);
  RTABMAP_STATS(Memory, Triangulated_points, );
  RTABMAP_STATS(Memory, Closest_node_distance, m);
  RTABMAP_STATS(Memory, Closest_node_angle, rad);

  RTABMAP_STATS(Timing, Memory_update, ms);
  RTABMAP_STATS(Timing, Cleaning_neighbors, ms);
  RTABMAP_STATS(Timing, Reactivation, ms);
  RTABMAP_STATS(Timing, Add_loop_closure_link, ms);
  RTABMAP_STATS(Timing, Map_optimization, ms);
  RTABMAP_STATS(Timing, Likelihood_computation, ms);
  RTABMAP_STATS(Timing, Posterior_computation, ms);
  RTABMAP_STATS(Timing, Hypotheses_creation, ms);
  RTABMAP_STATS(Timing, Hypotheses_validation, ms);
  RTABMAP_STATS(Timing, Statistics_creation, ms);
  RTABMAP_STATS(Timing, Memory_cleanup, ms);
  RTABMAP_STATS(Timing, Total, ms);
  RTABMAP_STATS(Timing, Forgetting, ms);
  RTABMAP_STATS(Timing, Joining_trash, ms);
  RTABMAP_STATS(Timing, Emptying_trash, ms);
  RTABMAP_STATS(Timing, Finalizing_statistics, ms);
  RTABMAP_STATS(Timing, RAM_estimation, ms);

  RTABMAP_STATS(TimingMem, Pre_update, ms);
  RTABMAP_STATS(TimingMem, Node_creation, ms);
  RTABMAP_STATS(TimingMem, Rehearsal, ms);
  RTABMAP_STATS(TimingMem, Keypoints_detection, ms);
  RTABMAP_STATS(TimingMem, Descriptors_extraction, ms);
  RTABMAP_STATS(TimingMem, Keypoints_3D_motion, ms);
  RTABMAP_STATS(TimingMem, Add_new_words, ms);
  RTABMAP_STATS(TimingMem, Compressing_data, ms);
  RTABMAP_STATS(TimingMem, Post_decimation, ms);
  RTABMAP_STATS(TimingMem, Occupancy_grid, ms);
  RTABMAP_STATS(TimingGrid, Projection, ms);
  RTABMAP_STATS(TimingGrid, Filtering, ms);
  RTABMAP_STATS(TimingGrid, Surface_classification, ms);
  RTABMAP_STATS(TimingGrid, Ground_refinement, ms);
  RTABMAP_STATS(TimingGrid, Transform_merge, ms);
  RTABMAP_STATS(TimingGrid, Voxelization, ms);
  RTABMAP_STATS(TimingGrid, Ray_tracing, ms);
  RTABMAP_STATS(Grid, Projected_points, count);
  RTABMAP_STATS(Grid, Valid_points, count);
  RTABMAP_STATS(Grid, Output_cells, count);

  RTABMAP_STATS(Keypoint, Dictionary_size, words);
  RTABMAP_STATS(Keypoint, Current_frame, words);
  RTABMAP_STATS(Keypoint, Vocabulary_words, words);
  RTABMAP_STATS(Keypoint, Vocabulary_memory_usage, KB);

 public:
  enum OptimizationTrigger : std::uint32_t {
    kOptimizationGlobalLoop = 1U << 0U,
    kOptimizationReducedNodes = 1U << 2U,
    kOptimizationPosePrior = 1U << 3U,
    kOptimizationGravity = 1U << 4U,
    kOptimizationRetrievedNodes = 1U << 7U
  };

  struct GraphOptimizationInfo {
    std::uint32_t triggerMask = 0U;
    int nodeId = 0;
    std::size_t poseCount = 0U;
    std::size_t edgeCount = 0U;
    int iterations = 0;
    double error = 0.0;
    double totalMs = 0.0;

    [[nodiscard]] bool executed() const noexcept { return triggerMask != 0U; }
  };

  /**
   * @brief Returns the default statistics data map
   *
   * Returns a map containing all predefined statistics with their default values (0.0).
   * This map is initialized when the first Statistics object is constructed.
   *
   * @return Const reference to the default statistics data map
   */
  static const std::map<std::string, float>& defaultData();

  /**
   * @brief Serializes a statistics data map to a string
   *
   * Converts a statistics data map into a serialized string format suitable for storage
   * or transmission. The format is: "key1:value1;key2:value2;..." where values are
   * formatted as numbers with dots (not commas) as decimal separators (independent of the system's locale).
   *
   * @param data The statistics data map to serialize
   * @return Serialized string representation of the data
   *
   * @note Empty maps result in empty strings
   * @see deserializeData()
   */
  static std::string serializeData(const std::map<std::string, float>& data);

  /**
   * @brief Deserializes a statistics data map from a string
   *
   * Parses a serialized statistics string back into a data map. The string format
   * should be: "key1:value1;key2:value2;..." as produced by serializeData().
   *
   * @param data The serialized string to parse
   * @return Deserialized statistics data map
   *
   * @note Invalid entries (missing colons, malformed values) are silently skipped
   * @see serializeData()
   */
  static std::map<std::string, float> deserializeData(const std::string& data);

 public:
  /**
   * @brief Default constructor
   *
   * Creates a new Statistics object in basic mode (extended = false).
   * Initializes all ID fields to 0 or -1, and timestamp to 0.0.
   *
   * @note The first Statistics object constructed will initialize the default data map.
   */
  Statistics();

  /**
   * @brief Virtual destructor
   */
  virtual ~Statistics();

  /**
   * @brief Adds a statistic value to the data map
   *
   * Adds or updates a statistic in the internal data map. The name should follow
   * the format "Group/Name/Unit" (e.g., "Timing/Total time/ms").
   *
   * @param name The statistic name in the format "Group/Name/Unit"
   * @param value The statistic value to store
   *
   * @note If a statistic with the same name already exists, it will be overwritten.
   * @note Use the static methods generated by RTABMAP_STATS() to get standardized names:
   * @code
   * statistics.addStatistic(Statistics::kTimingTotal(), 500.0f);
   * @endcode
   */
  void addStatistic(const std::string& name, float value);

  // setters

  /**
   * @brief Sets whether extended statistics mode is enabled
   *
   * In basic mode (extended = false), only loop closure and last node ID fields are filled.
   * In extended mode (extended = true), all available statistics including poses, constraints,
   * likelihoods, posteriors, and detailed timing information are stored.
   *
   * @param extended True to enable extended mode, false for basic mode
   */
  void setExtended(bool extended) { _extended = extended; }

  /**
   * @brief Sets the reference image ID (current/last processed node ID)
   * @param id The node ID
   */
  void setRefImageId(int id) { _refImageId = id; }

  /**
   * @brief Sets the reference image map ID
   * @param id The map ID associated with the reference image
   */
  void setRefImageMapId(int id) { _refImageMapId = id; }

  /**
   * @brief Sets the loop closure detection ID
   * @param id The node ID where a loop closure was detected (0 if none)
   */
  void setLoopClosureId(int id) { _loopClosureId = id; }

  /**
   * @brief Sets the loop closure map ID
   * @param id The map ID associated with the loop closure
   */
  void setLoopClosureMapId(int id) { _loopClosureMapId = id; }

  /**
   * @brief Sets the timestamp for these statistics
   * @param stamp The timestamp (typically in seconds since epoch)
   */
  void setStamp(double stamp) { _stamp = stamp; }

  /**
   * @brief Adds node data to the statistics
   *
   * Adds a node to the internal nodes data map. Multiple nodes
   * can be stored, indexed by their ID.
   *
   * @param data The node to add
   *
   * @note If a node with the same ID already exists, it will be overwritten.
   */
  void addNodeData(const Node& data) { (_nodesData).insert_or_assign(data.id(), data); }

  /**
   * @brief Sets all node data at once
   * @param data Map of node IDs to Node objects
   */
  void setNodesData(const std::map<int, Node>& data) { _nodesData = data; }

  /**
   * @brief Sets the pose graph (node poses)
   *
   * Sets the complete pose graph, mapping node IDs to their StateGroupf poses.
   * Used in extended mode for visualization and analysis.
   *
   * @param poses Map of node IDs to their poses
   */
  void setPoses(const std::map<int, StateGroupf>& poses) { _poses = poses; }

  /**
   * @brief Sets the constraint graph (links between nodes)
   *
   * Sets the complete constraint graph, mapping node IDs to their Link constraints.
   * Used in extended mode for visualization and analysis.
   *
   * @param constraints Multimap of node IDs to their links (a node can have multiple links)
   */
  void setConstraints(const std::multimap<int, Link>& constraints) { _constraints = constraints; }

  /**
   * @brief Sets the map correction transform
   *
   * The map correction transform represents the transformation from the map fixed frame
   * to the odometry fixed frame. This transform is typically updated after graph optimization
   * to transform the odometry pose in map frame.
   *
   * @param mapCorrection The pose of odometry frame in map coordinate system
   */
  void setMapCorrection(const StateGroupf& mapCorrection) { _mapCorrection = mapCorrection; }

  /**
   * @brief Sets the loop closure transform
   *
   * The transform between the current pose and the loop closure pose.
   *
   * @param loopClosureTransform The loop closure transform
   */
  void setLoopClosureTransform(const StateGroupf& loopClosureTransform) { _loopClosureTransform = loopClosureTransform; }

  /**
   * @brief Sets the map-correction covariance matrix.
   *
   * @param covariance Uncertainty of the current map correction (typically 6x6).
   */
  void setMapCorrectionCovariance(const std::optional<Mat66_t<double>>& covariance) { _mapCorrectionCovariance = covariance; }

  /**
   * @brief Sets node labels
   * @param labels Map of node IDs to their string labels
   */
  void setLabels(const std::map<int, std::string>& labels) { _labels = labels; }

  /**
   * @brief Sets the reduced IDs mapping
   *
   * Maps original node IDs to reduced IDs, used for memory optimization.
   *
   * @param reducedIds Map of original IDs to reduced IDs
   */
  void setReducedIds(const std::map<int, int>& reducedIds) { _reducedIds = reducedIds; }

  /**
   * @brief Sets the working memory state
   *
   * The working memory state is a vector of node IDs currently in working memory.
   *
   * @param state Vector of node IDs in working memory
   */
  void setWmState(const std::vector<int>& state) { _wmState = state; }

  void setGraphOptimizationInfo(const GraphOptimizationInfo& info) { _graphOptimization = info; }

  // getters

  /**
   * @brief Returns whether extended statistics mode is enabled
   * @return True if extended mode, false if basic mode
   */
  bool extended() const { return _extended; }

  /**
   * @brief Returns the reference image ID
   * @return The node ID (0 if not set)
   */
  int refImageId() const { return _refImageId; }

  /**
   * @brief Returns the reference image map ID
   * @return The map ID (-1 if not set)
   */
  int refImageMapId() const { return _refImageMapId; }

  /**
   * @brief Returns the loop closure detection ID
   * @return The node ID where loop closure was detected (0 if none)
   */
  int loopClosureId() const { return _loopClosureId; }

  /**
   * @brief Returns the loop closure map ID
   * @return The map ID associated with the loop closure (-1 if not set)
   */
  int loopClosureMapId() const { return _loopClosureMapId; }

  /**
   * @brief Returns the timestamp
   * @return The timestamp (0.0 if not set)
   */
  double stamp() const { return _stamp; }

  /**
   * @brief Returns the last node data
   *
   * Returns the most recently added node (by ID). If no nodes are stored,
   * returns a dummy empty node.
   *
   * @return Reference to the last node data
   */
  const Node& getLastNodeData() const { return _nodesData.empty() ? _dummyEmptyData : _nodesData.rbegin()->second; }

  /**
   * @brief Returns all node data
   * @return Const reference to the map of node IDs to Node objects
   */
  const std::map<int, Node>& getNodesData() const { return _nodesData; }

  /**
   * @brief Returns the pose graph
   * @return Const reference to the map of node IDs to poses
   */
  const std::map<int, StateGroupf>& poses() const { return _poses; }

  /**
   * @brief Returns the constraint graph
   * @return Const reference to the multimap of node IDs to links
   */
  const std::multimap<int, Link>& constraints() const { return _constraints; }

  /**
   * @brief Returns the map correction transform
   *
   * Returns the transform from the map fixed frame to the odometry fixed frame.
   *
   * @return Const reference to the map correction transform
   */
  const StateGroupf& mapCorrection() const { return _mapCorrection; }

  /**
   * @brief Returns the loop closure transform
   * @return Const reference to the loop closure transform
   */
  const StateGroupf& loopClosureTransform() const { return _loopClosureTransform; }

  /**
   * @brief Returns the map-correction covariance matrix.
   * @return Const reference to the covariance matrix (may be empty).
   */
  const std::optional<Mat66_t<double>>& mapCorrectionCovariance() const { return _mapCorrectionCovariance; }

  /**
   * @brief Returns node labels
   * @return Const reference to the map of node IDs to labels
   */
  const std::map<int, std::string>& labels() const { return _labels; }

  /**
   * @brief Returns the reduced IDs mapping
   * @return Const reference to the map of original IDs to reduced IDs
   */
  const std::map<int, int>& reducedIds() const { return _reducedIds; }

  /**
   * @brief Returns the working memory state
   * @return Const reference to the vector of node IDs in working memory
   */
  const std::vector<int>& wmState() const { return _wmState; }

  const GraphOptimizationInfo& graphOptimization() const { return _graphOptimization; }

  /**
   * @brief Returns the statistics data map
   *
   * Returns the map containing all plottable statistics in the format "Group/Name/Unit" -> value.
   * This is the main data structure for storing numeric statistics.
   *
   * @return Const reference to the statistics data map
   *
   * @note Use addStatistic() to add values to this map
   * @note Use serializeData() to convert this map to a string for storage
   */
  const std::map<std::string, float>& data() const { return _data; }

 private:
  bool _extended;  ///< Extended mode flag: false = only loop closure and last node ID, true = all statistics

  int _refImageId;        ///< Reference image ID (current/last processed node)
  int _refImageMapId;     ///< Reference image map ID
  int _loopClosureId;     ///< Loop closure detection ID (0 if none)
  int _loopClosureMapId;  ///< Loop closure map ID
  double _stamp;          ///< Timestamp for these statistics

  std::map<int, Node> _nodesData;  ///< Map of node IDs to Node objects
  Node _dummyEmptyData;            ///< Dummy empty node returned when no nodes are stored

  std::map<int, StateGroupf> _poses;      ///< Pose graph: node IDs to poses
  std::multimap<int, Link> _constraints;  ///< Constraint graph: node IDs to links (multimap allows multiple links per node)
  StateGroupf _mapCorrection;             ///< StateGroupf from map fixed frame to odometry fixed frame (typically updated after optimization)
  StateGroupf _loopClosureTransform;      ///< Loop closure transform
  std::optional<Mat66_t<double>> _mapCorrectionCovariance;  ///< Map-correction covariance matrix

  std::map<int, std::string> _labels;  ///< Node labels

  std::map<int, int> _reducedIds;  ///< Mapping of original IDs to reduced/compressed IDs

  std::vector<int> _wmState;  ///< Working memory state (vector of node IDs in working memory)
  GraphOptimizationInfo _graphOptimization;

  /**
   * @brief Statistics data map (plottable statistics)
   *
   * Format: {"Group/Name/Unit", value}
   * Example: {"Timing/Total time/ms", 500.0f}
   *
   * All plottable numeric statistics are stored in this map with hierarchical names
   * that allow for easy categorization and visualization.
   */
  std::map<std::string, float> _data;

  static std::map<std::string, float> _defaultData;  ///< Static default data map (initialized on first Statistics construction)
  static bool _defaultDataInitialized;               ///< Flag indicating if default data has been initialized
                                                     // end extended data
};

// A bounded stream owned independently from the mapping algorithm. I/O errors
// propagate to Task; enabled logging must not silently lose its output.
class StatisticsLog final {
 public:
  void open(const std::string& directory, const StatisticsParams&, bool overwrite);
  void write(const Statistics&);
  void close();

 private:
  std::ofstream stream_;
  bool flush_each_record_ = false;
};

}  // end namespace sapphire::backend

#endif /* STATISTICS_H_ */
