#pragma once
#include <backend/common/common_type.hpp>
#include <backend/common/odom_observation.hpp>
#include <backend/common/root_frame.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/maplayer/node.hpp>
#include <backend/io/runtime_resources.hpp>
#include <backend/system/action/build_node.hpp>
#include <backend/params/archive.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/navimap.hpp>
#include <backend/params/node.hpp>
#include <backend/params/pgo.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/topograph.hpp>
#include <backend/registration/retrieval/faiss_cosine_index.hpp>
#include <backend/registration/retrieval/fbow.hpp>
#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <optional>
#include <set>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "backend/common/sensor_data.hpp"
#include "backend/common/state_group.hpp"
#include "backend/database/feature_prefetch.hpp"
#include "backend/database/sensor_archive.hpp"
#include "backend/maplayer/topograph/link.hpp"

namespace sapphire::backend {

class Node;
class DBDriver;
class FbowIndex;
namespace featmap {
struct MergeDraft;
}
class Statistics;
namespace tools {
template <class ISA_TYPE>
class ParallelExecutor;
}

class LocalMapCollection;
class LocalMapMerge;
struct LocalMap;

class Memory {
 public:
  /** @brief First valid node id assigned to a new node (positive integer). */
  static const int kIdStart;
  /** @brief Reserved id for the "virtual place" used by the Bayes filter (negative). */
  static const int kIdVirtual;
  /** @brief Sentinel value indicating an invalid node id (zero). */
  static const int kIdInvalid;

 public:
  Memory(const ParametersMap &parameters = ParametersMap(), Topograph *graph = nullptr, RuntimeResources resources = RuntimeResources::defaults());
  Topograph &topograph() { return graph_; }
  const Topograph &topograph() const { return graph_; }
  virtual ~Memory();

  /** @return Effective startup parameters for this instance. */
  virtual const ParametersMap &getParameters() const { return parameters_; }
  const ArchiveParams &archive_params() const { return archive_params_; }
  const TopographParams &topograph_params() const { return topograph_params_; }
  const NavimapParams &navimap_params() const { return navimap_params_; }
  const PGOParams &pose_graph_params() const { return pose_graph_params_; }
  // Override only the next saved Info.parameters record (offline maintenance).
  // Preserves historical names/values without applying the current catalog.
  // Replaces the complete record; tool-only overrides cannot leak into it.
  void setParameterRecord(const ParametersMap &parameters);
  // One owner-thread admission: reserve identity, prepare privately outside
  // Memory, then publish the ready content and maintain residency.
  void begin_update() { _nodesAdded = 0; }
  int reserve_node_id(int requested);
  bool empty() const { return graph_.nodes().empty(); }
  int current_map_id() const { return _idMapCount; }
  size_t mean_feature_count() const;
  Node *admit_node(std::unique_ptr<Node>, SensorRecord, const Mat66_t<double> &covariance, const std::vector<float> &velocity = {});
  std::map<int, int> maintain_residency();
  bool apply_rehearsal(int old_id, int new_id, bool full_merge, bool intermediate_merge);
  /**
   * @brief Opens or creates a database and loads existing state into WM.
   *
   * @param dbUrl Path to the database file (empty for in-memory).
   * @param dbOverwritten If true, deletes the existing file before opening.
   * @return True on success, false if the database could not be opened.
   */
  bool init(const std::string &dbUrl, bool dbOverwritten = false);
  /**
   * @brief Flushes pending data and closes the database connection.
   *
   * @param databaseSaved If true, persists STM/WM nodes and statistics before closing.
   *        If false, in-memory state is discarded.
   * @param ouputDatabasePath If non-empty, the database is copied to this path on close. If a
   *        database on disk was initially created/loaded on a different path, it will be updated with latest
   *        changes and renamed to the output path.
   */
  void close(bool databaseSaved = true, const std::string &ouputDatabasePath = "");
  /**
   * @brief Starts a new map id, breaking mapping-session continuity.
   *
   * @param reducedIds If non-null, populated with id remappings produced by graph reduction
   *        triggered by the new map.
   * @return The new map id (auto-incremented).
   */
  int incrementMapId(std::map<int, int> *reducedIds = 0);
  /**
   * @brief Refreshes the age of @p nodeId in working memory, marking it as recent.
   *
   * Used to keep loop-closure hypotheses active so they are not transferred to LTM
   * during the next @ref forget() call.
   */
  void updateAge(int nodeId);

  /**
   * @brief Transfers oldest nodes from WM to LTM to respect memory and/or time limits.
   *
   * At least one more node than the count added/retrieved in the previous
   * iteration is transferred, regardless of visual-word count.
   *
   * In both regimes, candidate selection (see @c getRemovableNodes()) honors
   * @p ignoredIds, skips intermediate nodes, and excludes WM nodes linked to STM (to
   * preserve rehearsal). Intermediate (weight==-1) nodes linked to a transferred
   * node are dragged out with it.
   *
   * @param ignoredIds Nodes that must not be transferred (e.g. STM, retrieved ids, on the planned path).
   * @return Ids of nodes moved to LTM, in transfer order.
   */
  std::list<int> forget(const std::set<int> &ignoredIds = std::set<int>());
  // Global lightweight retrieval is independent of STM/WM node residency.
  const std::map<int, uint32_t> &retrieval_mass() const { return retrieval_mass_; }
  std::vector<int> global_candidates(const Node &query, size_t limit, std::vector<float> *scores = nullptr, bool cold_only = false) const;
  std::vector<float> global_scores(const Node &query, const std::vector<int> &ids) const;
  bool uses_scene_index() const { return scene_faiss_; }
  bool uses_raw_features() const { return scene_faiss_; }
  std::shared_ptr<const bow::FeatureBlock> frame_features_for(const Node &snapshot) const;
  void load_calibration(int id, std::vector<camera::Perspective> &cameras) const;
  size_t retrieval_document_count() const { return retrieval_mass_.size(); }
  size_t working_memory_bytes() const;
  bool working_memory_over_budget(unsigned int node_limit) const;
  std::list<int> trim_working_memory(unsigned int node_limit, const std::set<int> &protected_ids = {});
  unsigned int working_memory_byte_limit() const { return memory_params_.resident_budget_bytes; }
  /**
   * @brief Reloads nodes from LTM into WM.
   *
   * @param ids Candidate ids; those already in WM/STM are ignored.
   * @param maxLoaded Cap on successfully loaded non-intermediate nodes (0 = unlimited).
   * @param timeDbAccess Output: total reactivation time, including index activation (seconds).
   * @return Ids effectively brought back to WM.
   */
  std::set<int> reactivateNodes(const std::list<int> &ids, unsigned int maxLoaded, double &timeDbAccess);

  /**
   * @brief Drops the last node if flagged as bad.
   * @return Id of the removed node, or 0 if none was removed.
   */
  int cleanup();
  /** @brief Persists @p statistics to the database; @p saveWMState records the WM id list. */
  void saveStatistics(const Statistics &statistics, bool saveWMState);
  // Queue anchors and their dependent products together. A nonzero result is
  // acceptance only; DBDriver::writeCommitted reports durable completion.
  uint64_t save_keyframe(int node_id, const std::vector<OdomObservation> &trajectory, const std::map<int, cv::Mat> &localmaps,
                         const Statistics *statistics = nullptr, bool save_wm = false);
  /** @brief Stores a preview image (typically a thumbnail of the last frame) in the database. */
  void savePreviewImage(const cv::Mat &image) const;
  /** @brief Loads the preview image previously written by @ref savePreviewImage(). */
  cv::Mat loadPreviewImage() const;
  /** @brief Persists an optimized pose graph and its compatibility pose metadata. */

  const RootFrame &create_root_frame(int anchor, const StateGroupf &pose);

  void saveOptimizedPoses(const std::map<int, StateGroupf> &optimizedPoses, const StateGroupf &lastPose) const;
  /** @brief Loads optimized poses previously written by @ref saveOptimizedPoses(). */
  std::map<int, StateGroupf> loadOptimizedPoses(StateGroupf *lastPose) const;
  /** @brief Persists a 2D occupancy grid (origin @p xMin, @p yMin and resolution @p cellSize). */
  void save_localmap(int anchor_id, const cv::Mat &archive) const;
  void attach_navimap(std::shared_ptr<const LocalMap> map) const;
  std::map<int, cv::Mat> load_localmaps() const;
  void save_trajectory(const std::vector<OdomObservation> &observations) const;
  std::vector<OdomObservation> load_recent_trajectory(int limit) const;
  int trajectory_high_water() const;
  void save2DMap(const cv::Mat &map, float xMin, float yMin, float cellSize) const;
  /** @brief Loads the 2D occupancy grid previously written by @ref save2DMap(). */
  cv::Mat load2DMap(float &xMin, float &yMin, float &cellSize) const;
  /** @brief Forces the database driver to flush any queued node/word saves. */
  void emptyTrash();
  /** @brief Blocks until the asynchronous database write thread has finished pending work. */
  void joinTrashThread();
  void checkpoint_nodes();
  /**
   * @brief Adds a graph link between two nodes.
   * @param link Link to add (type, transform, covariance).
   * @param addInDatabase If true, the link is also added when one of the ids is only in LTM.
   * @return True if the link was added, false on conflict or missing nodes.
   */
  bool addLink(const Link &link, bool addInDatabase = false);

  /** @brief Replaces an existing link with @p link (same endpoints and type). */
  void updateLink(const Link &link, bool updateInDatabase = false);
  std::shared_ptr<const featmap::LandmarkMap> map_features_for(int id, const Node *snapshot = nullptr) const;
  void prefetch_map_features(const std::vector<int> &ranked);
  bool predictive_prefetch_enabled() const { return memory_params_.predictive_prefetch_enabled; }
  size_t feature_cache_capacity() const { return memory_params_.feature_cache_nodes; }

  // Cold-aware graph reads: Memory supplies storage, Topograph performs traversal.
  std::set<int> metric_component(int root) const;
  std::map<int, int> getNeighborsId(int nodeId, int maxGraphDepth, int maxCheckedInDatabase = -1, bool incrementMarginOnLoop = false,
                                    bool ignoreLoopIds = false, bool ignoreIntermediateNodes = false, const std::set<int> &nodesSet = std::set<int>(),
                                    double *dbAccessTime = 0) const;
  /** @brief Marks @p locationId as intermediate (weight = -1); excludes it from loop closure. */
  void convertToIntermediate(int locationId);
  /**
   * @brief Removes @p locationId from WM/STM and the database.
   * @param locationId Id of the node to delete.
   * @param deletedWords Optional output: words whose reference count dropped to zero.
   * @param keepLinkedInDb If true, the location keeps its links, weight and label and
   *                       stays part of the graph in the database. If false (default),
   *                       it is unlinked first -- links removed on both sides, weight
   *                       invalidated, label cleared -- so at best it is kept as
   *                       history only and no longer appears in
   *                       @ref getAllNodeIds(). An unlinked location is written
   *                       to the database only if it was already saved there or if
   *                       @ref "archive.keep_unlinked_nodes" is true (default);
   *                       otherwise it is discarded outright.
   */
  void deleteLocation(int locationId, std::list<int> *deletedWords = 0, bool keepLinkedInDb = false);
  /** @brief Forces @p locationId to be flushed to the database. */
  void saveLocationData(int locationId);
  /** @brief Removes any link between @p idA and @p idB (both directions). */
  /** @brief Strips raw images, user data and/or occupancy grid from @p id to save memory (RAM). This doesn't clear any compressed data.*/
  void removeRawData(int id, bool image = true, bool userData = true, bool occupancyGrid = true);
  /**
   * @brief Merges @p id with a close neighbor (graph reduction).
   * @param id Node to reduce.
   * @param maxDistance Maximum distance to a neighbor to allow the merge (meters).
   * @param keepLinkedInDb Same meaning as in @ref deleteLocation(): if true, the merged
   *                       node keeps its links, weight and label and stays part of the
   *                       graph in the database. If false (default), it is unlinked
   *                       first -- links removed on both sides, weight invalidated,
   *                       label cleared -- so at best it is kept as history only and no
   *                       longer appears in @ref getAllNodeIds(). An unlinked node
   *                       is written to the database only if it was already saved there
   *                       or if @ref "archive.keep_unlinked_nodes" is true
   *                       (default); otherwise it is discarded outright.
   * @param direction Restrict merge target: 0=any, 1=previous neighbor, 2=next neighbor.
   * @return Id of the node @p id was merged into, or 0 if no reduction was performed.
   */
  int reduceNode(int id, float maxDistance = 0.0f, bool keepLinkedInDb = false, int direction = 0);
  /**
   * @brief Enables a dummy visual word dictionary (no descriptors kept, word ids only).
   * @note Must be called before @ref init(); ignored (with an error logged) once the
   *       database driver is set.
   * @note Silently disabled by @ref init() when the database holds no dictionary,
   *       since there would be nothing to skip loading, and the memory would
   *       otherwise be left unable to add new nodes.
   */
  /** @return Working memory as { node id, age } (does not include STM). */
  const std::map<int, double> &getWorkingMem() const { return _workingMem; }
  /**
   * @brief Returns the number of nodes in working memory, excluding the virtual place.
   * @param ignoreIntermediateNodes If true, intermediate nodes (weight = -1) are not counted.
   */
  size_t getWorkingMemSize(bool ignoreIntermediateNodes = false) const;
  /** @return Number of intermediate nodes (weight = -1) currently in working memory. */
  int getWorkingMemIntermediateNodesCount() const { return _workingMemIntermediateNodesCount; }
  /** @return Number of intermediate nodes (weight = -1) currently in short-term memory. */
  int getStMemIntermediateNodesCount() const { return _stMemIntermediateNodesCount; }
  /** @return Set of node ids currently in short-term memory. */
  const std::set<int> &getStMem() const { return _stMem; }
  /** @return Configured maximum STM size (@ref "memory.recent_nodes"). */
  int getMaxStMemSize() const { return memory_params_.recent_nodes; }
  /** @brief Returns neighbor (sequential) links of @p nodeId; @p lookInDatabase also checks LTM. */
  std::multimap<int, Link> getNeighborLinks(int nodeId, bool lookInDatabase = false) const;
  /** @brief Returns loop-closure links of @p nodeId; @p lookInDatabase also checks LTM. */
  std::multimap<int, Link> getLoopClosureLinks(int nodeId, bool lookInDatabase = false) const;
  /**
   * @brief Returns all links of @p nodeId (neighbor, loop, prior, gravity, ...).
   * @param nodeId Source node id.
   * @param lookInDatabase Also query LTM for links.
   */
  std::multimap<int, Link> getLinks(int nodeId, bool lookInDatabase = false) const;
  /** @brief Returns links of every node; @p ignoreNullLinks drops empty placeholders. */
  std::multimap<int, Link> getAllLinks(bool lookInDatabase, bool ignoreNullLinks = true) const;
  /** @return True if raw binary image data is kept in memory after compression. */
  bool isBinDataKept() const { return archive_params_.keep_sensor_data; }
  bool keeps_depth_nodes() const { return archive_params_.keep_intermediate_data && navimap_params_.frame_grid_enabled; }
  /** @return Similarity threshold used by rehearsal (@ref "node.rehearsal_similarity"). */
  float getSimilarityThreshold() const { return node_params_.rehearsal_similarity; }
  /** @return Map from node id to weight (rehearsal accumulation count) for WM and STM. */
  std::map<int, int> getWeights() const;
  /** @return Id of the most recently added node, or 0 if none. */
  int getLastNodeId() const;
  StateGroupf current_odom_pose() const;
  bool submap_refresh_enabled() const { return node_params_.refresh_enabled; }
  const NodeParams &node_params() const { return node_params_; }
  const FeatureParams &feature_params() const { return feature_params_; }
  featmap::NodeBundleOptions local_ba_options() const {
    featmap::NodeBundleOptions options;
    options.max_iterations = node_params_.local_ba.max_iterations;
    options.max_points = node_params_.local_ba.max_optimized_points;
    options.profile = node_params_.profile_enabled;
    return options;
  }

  bool has_database() const { return bool(_dbDriver); }

  cv::Mat load_observation_archive(int id) const;
  // Private owner drafts. The caller persists them before non-throwing apply;
  // preparing a draft never publishes node or index state.
  struct FeatureUpdate {
    int retained = 0, source = 0, region = 0;
    uint64_t topology = 0;
    std::unique_ptr<Node> replacement;
    std::map<int, int> members;
    std::map<int, StateGroupf> local_poses;
    std::set<int> changed_captures;
    std::map<int, Node> link_nodes;
    int last_loop = 0;
    std::map<int, uint32_t> masses;
    std::optional<FbowIndex::ReferenceUpdate> references;
  };
  FeatureUpdate prepare_feature_content(int retained, int source, std::unique_ptr<Node> replacement, bool consolidate,
                                        const std::vector<Link> &added = {}, const std::vector<std::pair<int, int>> &removed = {}, int region = 0);
  bool can_apply(const FeatureUpdate &) const;
  void apply_feature_content(FeatureUpdate &&) noexcept;

  struct NodeUpdate {
    int retained = 0, removed = 0;
    uint64_t topology = 0;
    StateGroupf odom_from_anchor;
    std::map<int, std::unique_ptr<Node>> nodes;
    std::map<int, std::string> labels;
    std::map<int, int> members;
    std::map<int, double> working;
    std::deque<int> recent;
    std::optional<FbowIndex::ReferenceUpdate> references;
  };
  NodeUpdate prepare_node_merge(int retained, int removed, const featmap::MergeDraft &, std::shared_ptr<const LocalMap>, const StateGroupf &,
                                const std::multimap<int, Link> &constraints, const std::vector<Link> &added,
                                const std::vector<std::pair<int, int>> &removed_links);
  bool can_apply(const NodeUpdate &) const;
  void apply_submap_merge(NodeUpdate &&) noexcept;
  const std::shared_ptr<DBDriver> &database() const { return _dbDriver; }

  /**
   * @brief Returns the most recent WM node.
   * @param ignoreIntermediateNodes If true, skips weight==-1 placeholder nodes.
   */
  const Node *getLastWorkingNode(bool ignoreIntermediateNodes) const;
  /** @return Node id labeled @p label, or 0 if not found. */
  int getNodeIdByLabel(const std::string &label, bool lookInDatabase = true) const;
  /** @return Map from node id to non-empty label (including STM+WM+LTM). */
  const std::map<int, std::string> &getAllLabels() const { return _labels; }

  /** @return True if every persisted node (in LTM) is currently loaded in WM/STM. */
  bool allNodesInWM() const { return _allNodesInWM; }

  /** @return On-disk database size in bytes. */
  int getDatabaseMemoryUsed() const;
  /** @return Schema version of the open database (e.g. "0.20.0"). */
  std::string getDatabaseVersion() const;
  /** @return File path of the open database (empty if in-memory). */
  std::string getDatabaseUrl() const;
  /** @return Last @ref emptyTrash() flush time in seconds. */
  double getDbSavingTime() const;
  /** @return Map id of node @p id; @p lookInDatabase also queries LTM. */
  int getMapId(int id, bool lookInDatabase = false) const;
  /** @return Odometry pose stored with @p nodeId (null if unknown). */
  StateGroupf getOdomPose(int nodeId, bool lookInDatabase = false) const;
  /**
   * @brief Reads metadata of @p nodeId (no images/words).
   *
   * @param nodeId Node id.
   * @param odomPose Output odometry pose (null if not set).
   * @param mapId Output map id.
   * @param weight Output rehearsal weight.
   * @param label Output label (empty if none).
   * @param stamp Output timestamp (seconds, epoch).
   * @param velocity Output 6-vector velocity (empty if not set).
   * @param sensors Output environmental sensor readings.
   * @param lookInDatabase Also query LTM.
   * @return True if @p nodeId was found.
   */
  bool getNodeInfo(int nodeId, StateGroupf &odomPose, int &mapId, int &weight, std::string &label, double &stamp, std::vector<float> &velocity,
                   EnvSensors &sensors, bool lookInDatabase = false) const;
  /** @return Compressed image blob for @p nodeId (empty if not stored). */
  cv::Mat getImageCompressed(int nodeId) const;
  /**
   * @brief Loads sensor data of @p locationId from WM or LTM.
   * @param images Include compressed RGB/depth images.
   * @param userData Include user data blob.
   * @param occupancyGrid Include occupancy grid cells.
   */
  std::shared_ptr<const LocalGrid> local_grid(int nodeId) const;
  SensorRecord getNodeData(int locationId, bool images, bool userData, bool occupancyGrid) const;
  // Preserve typed feature blocks (raw ORB, grouped frame, or submap bank) on export.
  void copyNodeFeatures(int nodeId, Node &output, bool localFeatures, bool globalDescriptors) const;
  /** @brief Loads camera calibration stored with @p nodeId. */
  void getNodeCalibration(int nodeId, std::vector<camera::Perspective> &models) const;
  /**
   * @brief Returns all node ids in WM, STM and LTM.
   * @param ignoreChildren If true, nodes not linked to graph anymore are excluded
   */
  std::set<int> getAllNodeIds(bool ignoreChildren = true) const;
  /** @return True if mapping data has changed and must be persisted. */
  bool memoryChanged() const { return _memoryChanged; }

  /** @return Node with @p id in WM/STM, or null if not loaded. */
  const Node *getNode(int id) const;
  /** @return True if @p nodeId is in short-term memory. */
  bool isInSTM(int nodeId) const { return _stMem.find(nodeId) != _stMem.end(); }
  /** @return True if @p nodeId is in working memory. */
  bool isInWM(int nodeId) const { return _workingMem.find(nodeId) != _workingMem.end(); }
  /** @return True if @p nodeId is not in STM/WM, so when it is in LTM or non-existing. */
  bool isInLTM(int nodeId) const { return !this->isInSTM(nodeId) && !this->isInWM(nodeId); }
  /** @return True if node ids are auto-generated, false if taken from sensor data id. */
  bool isIDsGenerated() const { return node_params_.generate_node_ids; }

  /**
   * @return True if the external odometry pose orientation is used as the source
   *         of each new node's gravity link.
   *
   * When enabled, every new node gets a self-loop @ref Link::kGravity holding the
   * rotation of the odometry pose passed to @ref update(). This assumes odometry is
   * already gravity-aligned. When disabled, no gravity link is added.
   *
   * Gravity links are consumed by graph optimization only when
   * @ref "pose_graph.gravity_sigma" is non-zero.
   *
   * @see "topograph.use_odometry_gravity"
   * @see "pose_graph.gravity_sigma"
   */

  /** @return Approximate RAM usage of the in-memory state, in bytes. */
  unsigned long getMemoryUsed() const;  // Bytes

  /** @return Visual word dictionary used for tf-idf likelihood and feature matching. */
  // Null in Faiss mode: no vocabulary or inverted postings are allocated.
  const FbowIndex *fbow_index() const;

  /**
   * @brief Extracts a sub-graph (poses + links) for a set of node ids.
   *
   * Used by graph optimization callers to retrieve constraints for a region of interest.
   *
   * @param ids Ids to include.
   * @param poses Output: odometry poses for @p ids.
   * @param links Output: links between the nodes.
   * @param lookInDatabase If true, fetch missing data from LTM.
   */
  void getMetricConstraints(const std::set<int> &ids, std::map<int, StateGroupf> &poses, std::multimap<int, Link> &links,
                            bool lookInDatabase = false);

 private:
  void addNodeToStm(Node *node, const Mat66_t<double> &covariance, const std::pair<Link, Link> *neighbor);
  void clear();
  void loadDataFromDb();
  void moveToTrash(Node *s, bool keepLinkedToGraph = true, std::list<int> *deletedWords = 0, bool committed = false);

  void moveNodeToWMFromSTM(int id, int *reducedTo = 0);
  void addNodeToWmFromLTM(Node *node);
  Node *_getNode(int id) const;
  std::list<Node *> getRemovableNodes(int count, const std::set<int> &ignoredIds = std::set<int>(), bool budget_pressure = false);
  std::list<int> transfer_to_ltm(const std::vector<int> &ids);
  void index_retrieval_summary(const Node &node);
  std::map<int, uint32_t> retrieval_mass_;
  bool scene_faiss_ = false;
  int getNextId();

  const std::map<int, Node *> &getNodes() const { return graph_.nodes(); }

  void copyData(const Node *from, Node *to);

  // keypoint stuff
  void disableWordsRef(int nodeId);
  void enableWordsRef(const std::list<int> &nodeIds);

 private:
  const RuntimeResources resources_;
  std::unique_ptr<Topograph> owned_graph_;  // Standalone Memory clients own their domain here.
  Topograph &graph_;

 protected:
  /** @brief Database driver owning the persistent storage (created by @ref init()). */
  std::shared_ptr<DBDriver> _dbDriver;

 private:
  const ParametersMap parameters_;  // Fixed startup configuration; database records never reconfigure it.
  const NodeParams node_params_;
  const ArchiveParams archive_params_;
  const TopographParams topograph_params_;
  const MemoryParams memory_params_;
  const NavimapParams navimap_params_;
  const PGOParams pose_graph_params_;
  const RegistrationParams registration_params_;
  std::optional<ParametersMap> parameter_record_;
  ParametersMap parameterRecord() const;
  const FeatureParams feature_params_;
  mutable feature::FaissCosineIndex global_references_;
  bool use_previous_map_ = true;
  std::unique_ptr<featmap::PrefetchCache> submap_prefetch_;
  std::deque<int> recent_feature_nodes_;

  int _idCount;
  int _idMapCount;
  Node *_lastNode;
  // Current visit can be anchored to an old canonical submap after fusion.
  StateGroupf latest_odom_pose_;
  bool session_has_observations_ = false;
  bool _memoryChanged;  // Tracks admitted or changed map content.
  int _nodesAdded;
  int _workingMemIntermediateNodesCount;  // number of nodes with weight==-1 currently in _workingMem
  int _stMemIntermediateNodesCount;       // number of nodes with weight==-1 currently in _stMem
  bool _allNodesInWM;

  // Input/archive residency is independent of the map layers and their snapshots.
  std::map<int, SensorRecord> sensor_records_;
  const SensorRecord &cached_sensor_record(int id) const;
  bool queue_node(const Node &node) const;
  std::set<int> _stMem;               // id
  std::map<int, double> _workingMem;  // id,age
  std::map<int, std::string> _labels;

  // Keypoint stuff
  std::unique_ptr<FbowIndex> _vwd;
};

}  // namespace sapphire::backend
