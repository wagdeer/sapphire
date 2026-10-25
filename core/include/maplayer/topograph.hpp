#pragma once
#include <common/common_type.hpp>
#include <maplayer/topograph/root_frame.hpp>
#include <maplayer/topograph/link.hpp>
#include <parameters.hpp>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <set>

namespace sapphire {

class Memory;
class Mapping;
class Node;
// Nodes, their graph connectivity and metric placement.
// Node links are authoritative; numerical factors and estimates belong to the solver.
// Memory controls resident/cold transfers, not the relation model. The online
// Mapping owns this object and lends it to Memory for that storage lifecycle.
class Topograph final {
 public:
  Topograph() = default;
  ~Topograph();
  Topograph(const Topograph&) = delete;
  Topograph& operator=(const Topograph&) = delete;
  int getLastLoopClosureId() const { return _lastLoopClosureId; }
  const std::vector<double>& getOdomMaxInf() const { return _odomMaxInf; }
  bool isGraphReduced() const { return parameters_.reduce_graph; }
  const Node* getNode(int id) const;
  std::map<int, int> place_distances(int origin, unsigned max_hops) const;
  std::map<int, std::vector<int>> resident_adjacency() const;
  // Residency changes are graph edits too: preserve old-neighborhood invalidation.
  void admit(Node* node, bool restored = false);
  void release(int id);
  void apply_links(const std::vector<Link>& added, const std::vector<std::pair<int, int>>& removed);
  void prepare_links(const std::vector<Link>&, const std::vector<std::pair<int, int>>&, std::map<int, Node>&, int& last_loop) const;
  void apply_links(std::map<int, Node>&, int last_loop) noexcept;
  std::map<int, float> getNeighborsIdRadius(int nodeId, float radius, const std::map<int, SE3f>& poses, int maxGraphDepth) const;
  std::set<int> metric_component(int root, const std::set<int>& ids, const std::function<std::multimap<int, Link>(int)>& load_links) const;
  int reduceNode(int id, float maxDistance, int direction);
  using ColdLinks = std::function<void(int, int&, std::multimap<int, Link>&)>;
  std::map<int, int> getNeighborsId(int id, int depth, int maxChecked, bool increment, bool ignoreLoops, bool ignoreIntermediate,
                                    const std::set<int>& nodes, double* seconds, const ColdLinks& load) const;
  void metric_constraints(const std::set<int>&, std::map<int, SE3f>&, std::multimap<int, Link>&, bool include_intermediate,
                          const ColdLinks&) const;
  void detach(Node& node);

  void updateLink(const Link& link);
  void removeLink(int from, int to);

  const std::map<int, RootFrame>& root_frames() const { return root_frames_; }
  bool persistent_root_frames() const { return persistent_root_frames_; }
  uint64_t topology_revision() const { return topology_revision_; }
  const RootFrame* rootFrame(const std::map<int, SE3f>& poses) const;
  void validateFramePose(std::map<int, SE3f>& poses) const;
  std::string root_frame_id() const;
  std::map<std::string, SE3f> root_frame_aliases() const;
  void select_world_component(const std::set<int>& connected);

  void track_observation(const Node&, const Mat66_t<double>&);
  void remove_reduced(const std::map<int, int>& reduced);
  // Returns a component requiring cold connectivity resolution by the caller, or zero.
  int publication_component(bool use_previous, bool solver_enabled);
  void publish_current(const std::map<int, int>& reduced, const std::list<int>& removed);
  void accept_explicit_solution(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& constraints, int last_id,
                                bool priors_ignored);

  int committed_loop() const { return committed_loop_; }

  void commit_loop(int place) noexcept { committed_loop_ = place; }
  void start_session();
  // Consume an already validated solver projection. No allocations at commit.
  void commit_solution(std::map<int, SE3f>& poses, const SE3f& correction,
                       const std::optional<Mat66_t<double>>& covariance, const SE3f& mapped) noexcept;
  // No allocation: applied in the same commit as contraction of Node links.
  void commit_contraction(int retained, int removed) noexcept;
  static void erase_graph_edge(Node&, Node&, int& last_loop);
  static void insert_graph_edge(Node&, Node&, const Link&, bool reduce, int& last_loop);

  const std::map<int, Node*>& nodes() const { return _nodes; }

  std::set<int> take_pose_changes() {
    std::set<int> out;
    out.swap(changed_poses_);
    return out;
  }
  void record_pose_changes(std::set<int>& changes) noexcept { changed_poses_.merge(changes); }

  bool links_changed() const { return _linksChanged; }
  void restore_frames(bool persistent, std::map<int, RootFrame> frames);
  const RootFrame& restore_frame(const RootFrame&);

  void clear_residents();
  void reset_storage();
  void remap_last_loop(int removed, int retained);
  const SE3f& current_odom_pose() const { return odom_from_active_anchor_; }
  void reset_odometry() { odom_from_active_anchor_ = {}; }
  void make_intermediate(Node&);

  std::map<int, Node> prepare_cold_link(const Link&, bool update) const;
  void commit_cold_link(std::map<int, Node>&) noexcept;

  // New observation only; restoring a stored node must not synthesize constraints.
  void admit_observation(Node*, Node* previous, const Mat66_t<double>& covariance);

  const std::map<int, SE3f>& poses() const { return _optimizedPoses; }
  const std::map<int, SE3f>& stored_poses() const { return stored_poses_; }
  const std::map<int, SE3f>& world_poses() const { return world_poses_; }
  const SE3f& world_last_pose() const { return world_last_pose_; }
  const SE3f& correction() const { return _mapCorrection; }
  const SE3f& last_mapped_pose() const { return _lastMappedPose; }
  const std::optional<Mat66_t<double>>& correction_covariance() const { return _mapCorrectionCovariance; }
  const std::pair<int, int>& rejected_edge() const { return _lastRejectedLoopClosureIds; }
  bool aligned() const { return map_aligned_; }
  int active_anchor() const { return active_frame_anchor_; }
  void record_rejected_edge(const std::pair<int, int>& edge) { _lastRejectedLoopClosureIds = edge; }
  void restore_solution(std::map<int, SE3f>&, const SE3f&);
  void activate_restored_world(bool use_previous);
  void clear_projection();
  void reset_projection();
  void clear_publication();
  void end_observations();
  void begin_mapping(bool new_session);
  void remove_reduced_world(const std::map<int, int>&);
  void prepare_archive(const std::set<int>& surviving, bool no_current_odometry);
  void finish_archive(const std::set<int>& surviving, const std::map<int, int>& reduced);
  void remove_last_projection(int last_id);
  void retain_component(const std::map<int, int>& ids);
  void remember_solution(const std::map<int, SE3f>& poses);
  void apply_offline_solution(const std::map<int, SE3f>&, bool correct_odometry);

 private:
  friend class Memory;
  void configure(const TopographParams& parameters) {
    parameters.validate();
    parameters_ = parameters;
  }

  Node* mutable_node(int id) const {
    auto it = _nodes.find(id);
    return it == _nodes.end() ? nullptr : it->second;
  }
  std::optional<std::pair<Link, Link>> odometry_links(const Node&, const Node*, bool, const Mat66_t<double>&, bool) const;
  void connect_odometry(Node&, Node&, const Mat66_t<double>&, const std::pair<Link, Link>*);
  static bool canBeReduced(const Link&, float, int);
  TopographParams parameters_;
  std::map<int, RootFrame> root_frames_;
  bool persistent_root_frames_ = false;
  // Nodes whose optimized placement changed; their local Gaussian geometry is unchanged.
  std::set<int> changed_poses_;
  int _lastLoopClosureId = 0;
  bool _linksChanged = false;  // False by default, become true when links are modified.
  std::uint64_t topology_revision_ = 1;
  std::vector<double> _odomMaxInf;
  std::map<int, Node*> _nodes;
  SE3f odom_from_active_anchor_;
  int committed_loop_ = 0;
  int active_frame_anchor_ = 0;
  std::map<int, SE3f> _optimizedPoses;
  // Independent components retain separate coordinate origins in persistent snapshots.
  std::map<int, SE3f> stored_poses_;
  // Only the selected connected component is exposed for world publication.
  std::map<int, SE3f> world_poses_;
  SE3f world_last_pose_;
  bool map_aligned_ = false;
  SE3f _mapCorrection = SE3f::identity();
  SE3f _lastMappedPose;                              ///< Last pose of getLocalOptimizedPoses().
  std::optional<Mat66_t<double>> _mapCorrectionCovariance;  ///< Uncertainty of the current map correction.
  std::pair<int, int> _lastRejectedLoopClosureIds;
};

/**
 * @namespace sapphire::graph
 * @brief Graph of Node nodes: poses, inter-submap links, and path planning.
 * Gaussian geometry remains inside each node;
 * this graph only places and connects whole submaps.
 *
 * Functions operate on maps of node ids to @ref SE3f poses and
 * @ref Link constraints (typically stored as `std::multimap<int, Link>` keyed by
 * the source node id).
 *
 * Main groups:
 * - **Validation:** @ref computeMaxGraphErrors()
 * - **Links:** @ref findLink(), @ref findLinks(),
 *   @ref filterDuplicateLinks()
 * - **Spatial queries:** @ref findNearestNode(), @ref findNearestNodes(),
 *   @ref radiusPosesFiltering()
 * - **Planning:** @ref computePath(), @ref computePathLength()
 */
namespace graph {
// Connected component with the existing traversal order and pose propagation.
// Solver policy is supplied by the caller; connectivity owns no optimizer state.
void getConnectedGraph(int fromId, const std::map<int, SE3f>& posesIn, const std::multimap<int, Link>& linksIn,
                       std::map<int, SE3f>& posesOut, std::multimap<int, Link>& linksOut, bool priors_ignored = false);

/**
 * @brief Largest pose-graph constraint violations after optimization.
 *
 * For each non-self-referenced link (`from != to`), compares the relative pose implied by @p poses to the
 * link measurement and tracks the worst linear/angular error and error/std ratios.
 */
struct MaxGraphErrors {
  float linear = -1.0f;        ///< Absolute linear error (m) of the worst link.
  float angular = -1.0f;       ///< Absolute angular error (rad) of the worst link.
  float linearRatio = -1.0f;   ///< linear / sqrt(trans variance) of the worst link.
  float angularRatio = -1.0f;  ///< angular / sqrt(rot variance) of the worst link.
  Link linearLink;             ///< Link with largest @ref linearRatio.
  Link angularLink;            ///< Link with largest @ref angularRatio.
};

/**
 * @brief Finds the worst pose-graph constraint residuals after optimization.
 *
 * Iterates over @p links and, for each non-self-referenced edge (`from != to`):
 * 1. Looks up `T_from` and `T_to` in @p poses (returns default @ref MaxGraphErrors if
 *    any endpoint pose is missing, null, or not invertible).
 * 2. Builds the relative pose implied by the optimized poses:
 *    - Normal link: `t = T_from⁻¹ · T_to`
 * 3. Compares `t` to the link transform:
 *    - **Linear error:** max |Δx|, |Δy|, and |Δz| (z ignored when @p for3DoF is true).
 *    - **Angular error:** full 3D angle between `t` and the link, or yaw-only if @p for3DoF;
 * 4. Normalizes by link uncertainty: `error / sqrt(variance)` using the link information
 *    matrix (largest diagonal variance for translation/rotation).
 *
 * The returned @ref MaxGraphErrors holds the link with the highest linear and angular
 * *ratios* (not necessarily the largest absolute error).
 *
 * @param poses Optimized node poses (must contain every `from` and `to` id used).
 * @param links Graph constraints (typically `std::multimap<int, Link>` keyed by `from`).
 * @param for3DoF If true, linear error uses x/y only and angular error compares yaw only.
 * @return @ref MaxGraphErrors; fields stay `-1` when no valid link was checked or on early abort.
 */
MaxGraphErrors computeMaxGraphErrors(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& links);

/**
 * @brief Maximum information-matrix diagonal over odometry neighbor links.
 *
 * Scans @p links of type @ref Link::kOdometry and,
 * for each dof (x, y, z, roll, pitch, yaw), keeps the largest diagonal entry of
 * the 6×6 information matrix.
 *
 * @param links Graph constraints (multimap keyed by source id).
 * @return Six maximum information values, or an empty vector if no neighbor links exist.
 */
std::vector<double> getMaxOdomInf(const std::multimap<int, Link>& links);

/**
 * @brief Finds the first link from @p from to @p to in a multimap keyed by source id.
 *
 * Iterates all entries with key @p from and matches the destination (and optionally
 * @p type). When @p checkBothWays is true, also searches key @p to for a link back
 * to @p from.
 *
 * @param links Link multimap (`key` = source node id).
 * @param from Source node id.
 * @param to Destination node id.
 * @param checkBothWays If true, also match `to → from`.
 * @param type Required link type, or std::nullopt to accept any type.
 * @return Iterator to the link, or `links.end()` if not found.
 */
std::multimap<int, Link>::iterator findLink(std::multimap<int, Link>& links, int from, int to, bool checkBothWays = true,
                                            std::optional<Link::Type> type = std::nullopt);
/** @overload
 *  `std::multimap<int, std::pair<int, Link::Type>>`. */
std::multimap<int, std::pair<int, Link::Type>>::iterator findLink(std::multimap<int, std::pair<int, Link::Type>>& links, int from, int to,
                                                                  bool checkBothWays = true, std::optional<Link::Type> type = std::nullopt);
/** @overload
 *  `std::multimap<int, int>`. */
std::multimap<int, int>::iterator findLink(std::multimap<int, int>& links, int from, int to, bool checkBothWays = true);
/** @overload
 *  Const `std::multimap<int, Link>`. */
std::multimap<int, Link>::const_iterator findLink(const std::multimap<int, Link>& links, int from, int to, bool checkBothWays = true,
                                                  std::optional<Link::Type> type = std::nullopt);
/** @overload
 *  Const `std::multimap<int, std::pair<int, Link::Type>>`. */
std::multimap<int, std::pair<int, Link::Type>>::const_iterator findLink(const std::multimap<int, std::pair<int, Link::Type>>& links, int from, int to,
                                                                        bool checkBothWays = true, std::optional<Link::Type> type = std::nullopt);
/** @overload
 *  Const `std::multimap<int, int>`. */
std::multimap<int, int>::const_iterator findLink(const std::multimap<int, int>& links, int from, int to, bool checkBothWays = true);

/**
 * @brief Lists all links incident on node @p from.
 *
 * Outgoing links (`link.from() == from`) are returned as stored; for incoming links
 * (`link.to() == from`), the inverse link is returned so the pose of @p from is always
 * the source frame.
 *
 * @param links Graph constraints.
 * @param from Node id to query.
 * @return Incident links (may be empty).
 */
std::list<Link> findLinks(const std::multimap<int, Link>& links, int from);

/**
 * @brief Removes duplicate undirected links.
 *
 * Keeps the first occurrence of each `(from, to)` or `(to, from)` pair with the same
 * @ref Link::Type (see @ref findLink() with @p checkBothWays).
 *
 * @param links Input link multimap.
 * @return Copy without duplicates.
 */
std::multimap<int, Link> filterDuplicateLinks(const std::multimap<int, Link>& links);

/**
 * @brief Subsamples poses that are spatially (and optionally angularly) redundant.
 *
 * For each pose not yet processed, finds all poses within @p radius (KD-tree). When
 * @p angle &gt; 0, only poses whose +X axis differs by at most @p angle (rad) are grouped.
 * From each group, keeps one pose: the latest in map order if @p keepLatest, otherwise
 * the earliest. The first and last poses of the input map are always kept.
 *
 * @param poses Input trajectory (map iteration order defines “latest/oldest”).
 * @param radius Clustering radius (m); if `≤ 0` or fewer than three poses, returns @p poses unchanged.
 * @param angle Max heading difference within a cluster (rad); `0` ignores orientation.
 * @param keepLatest If true, keep the latest pose per cluster; otherwise the earliest.
 * @return Subsampled poses.
 */
std::map<int, SE3f> radiusPosesFiltering(const std::map<int, SE3f>& poses, float radius, float angle, bool keepLatest = true);

/**
 * @brief Radius-neighbor clustering of poses.
 *
 * For each pose, inserts `(queryId, neighborId)` into the output for every other pose
 * within @p radius (and within @p angle of the query heading when @p angle &gt; 0).
 *
 * @param poses Input poses.
 * @param radius Search radius (m); no pairs if `≤ 0` or fewer than two poses.
 * @param angle Max heading difference (rad); `0` ignores orientation.
 * @return Multimap of pose id → neighbor id (both ids from @p poses).
 */
std::multimap<int, int> radiusPosesClustering(const std::map<int, SE3f>& poses, float radius, float angle);

/**
 * @brief A* shortest path on a pose graph with Euclidean edge costs.
 *
 * Edge cost between adjacent nodes is the Euclidean distance between their poses in
 * @p poses. Uses `costSoFar + distToEnd` where `distToEnd` is the distance to the goal pose.
 *
 * @param poses Node id → pose (must contain every node reached by @p links).
 * @param links Directed edges (`from` → `to`) keyed by source id.
 * @param from Start node id.
 * @param to Goal node id.
 * @param updateNewCosts If true, use a multimap queue that can decrease keys when a shorter path is found.
 * @return Ordered path from @p from to @p to (inclusive) with poses; empty if unreachable.
 */
std::list<std::pair<int, SE3f>> computePath(const std::map<int, SE3f>& poses, const std::multimap<int, int>& links, int from, int to,
                                                   bool updateNewCosts = false);

/**
 * @brief Dijkstra shortest path on link constraints.
 *
 * Explores outgoing links keyed by `link.from()`. Edge cost is `1` when
 * @p useSameCostForAllLinks is true, otherwise the translation norm of the link transform.
 *
 * @param links Constraints keyed by source node id.
 * @param from Start node id.
 * @param to Goal node id.
 * @param updateNewCosts If true, allow cost improvements on open nodes.
 * @param useSameCostForAllLinks If true, unit edge cost; else use the link translation norm.
 * @return Node ids from @p from to @p to (inclusive); empty if unreachable.
 */
std::list<int> computePath(const std::multimap<int, Link>& links, int from, int to, bool updateNewCosts = false, bool useSameCostForAllLinks = false);

/**
 * @brief Id of the nearest pose to @p targetPose.
 *
 * Wrapper around @ref findNearestNodes() with `radius=0`, `k=1` (1-NN in 3D).
 *
 * @param poses Nodes to search.
 * @param targetPose Query position (x, y, z only; orientation is not used).
 * @param distance If not null, set to the squared Euclidean distance of the match.
 * @return Closest node id, or `0` if @p poses is empty.
 */
int findNearestNode(const std::map<int, SE3f>& poses, const SE3f& targetPose, float* distance = 0);

/**
 * @brief Spatial neighbors of a node (KD-tree radius or k-NN search).
 *
 * @p nodeId is removed from the search set. Requires `radius &gt; 0` or `k &gt; 0`.
 * When `radius &gt; 0`, returns all poses within @p radius (up to @p k if `k &gt; 0`).
 * When `radius == 0`, returns the @p k nearest neighbors.
 *
 * @param nodeId Query node (must exist in @p poses); excluded from results.
 * @param poses Candidate poses.
 * @param radius Search radius (m).
 * @param angle Max +X axis angle difference (rad); `0` ignores heading.
 * @param k Max neighbors (`0` = all within radius).
 * @return Neighbor id → squared Euclidean distance.
 */
std::map<int, float> findNearestNodes(int nodeId, const std::map<int, SE3f>& poses, float radius, float angle = 0.0f, int k = 0);
/**
 * @brief Spatial neighbors of a pose (KD-tree radius or k-NN search).
 * @param targetPose Query pose (position used; orientation used when @p angle &gt; 0).
 * @param poses Candidate poses (not modified).
 * @param radius Search radius (m).
 * @param angle Max +X axis angle difference (rad); `0` ignores heading.
 * @param k Max neighbors (`0` = all within radius).
 * @return Neighbor id → squared Euclidean distance.
 */
std::map<int, float> findNearestNodes(const SE3f& targetPose, const std::map<int, SE3f>& poses, float radius, float angle = 0.0f,
                                      int k = 0);
/**
 * @brief Like the node-id overload of @ref findNearestNodes()
 * but returns full @ref SE3f values.
 */
std::map<int, SE3f> findNearestPoses(int nodeId, const std::map<int, SE3f>& poses, float radius, float angle = 0.0f, int k = 0);
/** @overload
 *  Query by @ref SE3f instead of node id. */
std::map<int, SE3f> findNearestPoses(const SE3f& targetPose, const std::map<int, SE3f>& poses, float radius, float angle = 0.0f,
                                            int k = 0);

/**
 * @brief Path length along an ordered list of poses.
 *
 * Sums translation distances for consecutive entries.
 *
 * @param path Ordered `(nodeId, pose)` pairs.
 * @return Total length (m), or `0` if fewer than two poses.
 */
float computePathLength(const std::vector<std::pair<int, SE3f>>& path);

/**
 * @brief Path length in map iteration order.
 *
 * Sums distances between consecutive poses in ascending map key order (does not verify
 * that entries form a connected path in the graph).
 *
 * @param path Poses keyed by node id (sorted by key).
 * @return Total length (m), or `0` if fewer than two poses.
 */
float computePathLength(const std::map<int, SE3f>& path);

/**
 * @brief Axis-aligned bounding box of pose positions.
 *
 * @param poses Input poses (no effect if empty).
 * @param min Output minimum (x, y, z) in meters.
 * @param max Output maximum (x, y, z) in meters.
 */
void computeMinMax(const std::map<int, SE3f>& poses, cv::Vec3f& min, cv::Vec3f& max);

} /* namespace graph */

} /* namespace sapphire */
