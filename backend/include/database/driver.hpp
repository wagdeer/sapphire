#pragma once

#include <atomic>
#include <backend/common/common_type.hpp>
#include <backend/common/odom_observation.hpp>
#include <backend/common/root_frame.hpp>
#include <backend/common/state_group.hpp>
#include <backend/database/statistics.hpp>
#include <backend/registration/descriptor/feature_block.hpp>
#include <backend/maplayer/topograph/link.hpp>
#include <backend/params/database.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <optional>
#include <set>
#include <string>
#include <thread>

#include "backend/database/sensor_archive.hpp"
#include "backend/tools/shared_queue.hpp"

namespace sapphire::backend {

class Node;
struct LocalMap;
namespace featmap {
class LandmarkMap;
}

class DBDriver {
 public:
  struct NodeCommit {
    int target = 0, source = 0, region = 0;
    cv::Mat bundle, expected_target, expected_source, expected_region;
    std::map<int, StateGroupf> poses;
    StateGroupf last;
  };
  // One ordered, indivisible persistence unit. Snapshots and cv::Mat buffers
  // transferred here must no longer be modified by the submitting thread.
  struct WriteBatch {
    std::vector<std::shared_ptr<Node>> nodes;
    std::map<int, SensorRecord> records;
    std::vector<OdomObservation> trajectory;
    std::map<int, cv::Mat> localmaps;
    std::optional<Statistics> statistics;
    std::optional<NodeCommit> submap;
    bool saveWmState = false;
    uint64_t epoch = 0;
  };
  // Returns a submission sequence (not a durability acknowledgement), or 0
  // on rejection. Batches are never replaced/coalesced by node ID.
  uint64_t asyncWrite(WriteBatch batch);
  // Sequence acknowledgements are valid only for the current connection.
  bool writeCommitted(uint64_t sequence) const;

  struct WorkerMetrics {
    size_t pendingItems = 0;
    size_t pendingBytes = 0;
    size_t highWaterItems = 0;
    size_t highWaterBytes = 0;
    uint64_t producerWaitCount = 0;
    uint64_t producerWaitNanoseconds = 0;
    size_t lastBatchItems = 0;
    size_t lastBatchBytes = 0;
    size_t maxBatchItems = 0;
    size_t maxBatchBytes = 0;
    uint64_t acceptedSequence = 0;
    uint64_t completedSequence = 0;
    bool failed = false;
    double oldestAgeMs = 0;
    double lastWriteMs = 0;
    double maxWriteMs = 0;
  };

  /**
   * @brief Factory: returns a SQLite database driver (@ref DBDriverSqlite3).
   * @param parameters Optional driver parameters.
   * @return New driver instance; caller owns the pointer.
   */
  static DBDriver *create(const ParametersMap &parameters = ParametersMap());

 public:
  virtual ~DBDriver();

  /**
   * @brief True when no database file URL was set at open time.
   * @note @ref DBDriverSqlite3 overrides this when @ref "database.sqlite.in_memory" is enabled.
   */
  virtual bool isInMemory() const { return _url.empty(); }
  /** @return Database file path last passed to @ref openConnection(). */
  const std::string &getUrl() const { return _url; }

  /**
   * @brief Queue a node for deferred save; ownership is transferred.
   *
   * @note Only the *compressed* sensor buffers are written
   * from the separate SensorRecord argument. Image/depth raw buffers are ignored;
   * user data is encoded on demand. Memory prepares the archive before saving.
   * Direct driver callers should call encodeSensorData(record)
   * or restore already encoded bytes with archivedSensorData().
   */
  void asyncSave(Node *s, std::optional<SensorRecord> data = std::nullopt);
  /** @brief Ownership-safe enqueue. False means the worker rejected the item. */
  bool asyncSave(std::unique_ptr<Node> s, std::optional<SensorRecord> data = std::nullopt);
  /**
   * @brief Flush queued nodes to the database.
   * @param async If true, signal the background thread instead of flushing synchronously.
   */
  void emptyTrashes(bool async = false);
  double getEmptyTrashesTime() const { return _emptyTrashesTime.load(); }
  WorkerMetrics getWorkerMetrics() const;
  bool isWorkerHealthy() const;
  std::string getWorkerError() const;
  // Notification only: never close/join the driver from this callback.
  void setWorkerFailureCallback(std::function<void(const std::string &)> callback);

  /**
   * @name Direct database updates
   * @brief Write node or link data immediately (not via the async trash).
   * @{
   */
  /** @brief Insert or replace a link in the database. */
  void addLink(const Link &link);
  void write_link_pair(const Link &, bool from_cold, bool to_cold, bool update);
  /** @brief Remove a link between two nodes. */
  void removeLink(int from, int to);
  /** @brief Update an existing link in the database. */
  void updateLink(const Link &link);
  /** @brief Update camera calibration stored for a node. */
  void updateCalibration(int nodeId, const std::vector<camera::Perspective> &models);
  /**
   * @brief Update the depth image stored for a node.
   * @param image Raw @c CV_16UC1 millimeter depth image.
   * The image is always RVL-compressed before writing.
   */
  void updateDepthImage(int nodeId, const cv::Mat &image);
  /** @} */

 public:
  /**
   * @name Session exports and map artifacts
   * @brief Statistics, preview image, optimized poses and 2D map.
   * @{*/
  struct RunInfo {
    int lastNode = 0, processMemoryUsed = -1, databaseMemoryUsed = 0, dictionarySize = 0;
    ParametersMap parameters;
  };
  // Finalization barrier: atomically persist resident nodes, their exact working
  // set and run metadata. Caller retains exclusive node ownership until return.
  // Empty nodes explicitly saves an empty set; it does not select all history.
  void saveWorkingMemory(const std::vector<Node *> &nodes, const std::map<int, SensorRecord> &records, const RunInfo &info);
  void addInfoAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize,
                       const ParametersMap &parameters) const;
  /**
   * @brief Append a @ref Statistics record for a processed node.
   * @param statistics Metrics for @ref Statistics::refImageId() (insert skipped if id &le; 0 or data empty).
   * @param saveWmState If true and schema &ge; 0.16.2, also store compressed working-memory state.
   */
  void addStatistics(const Statistics &statistics, bool saveWmState) const;
  /**
   * @brief Save the map preview thumbnail in the Admin table.
   * @param image Raw image or JPEG-compressed blob (@c CV_8UC1, single row); empty clears it.
   * @note Requires database schema &ge; 0.12.0.
   */
  void savePreviewImage(const cv::Mat &image) const;
  /** @brief Load and uncompress the map preview thumbnail from the Admin table. */
  cv::Mat loadPreviewImage() const;
  /** @brief Persist graph-optimized poses and the latest corrected mapping pose. */
  std::map<int, RootFrame> load_root_frames() const;
  RootFrame create_root_frame(int anchor, const StateGroupf &pose);
  void saveOptimizedPoses(const std::map<int, StateGroupf> &optimizedPoses, const StateGroupf &lastPose);
  std::map<int, StateGroupf> loadOptimizedPoses(StateGroupf *lastPose = 0) const;
  /** @brief Save the assembled 2D occupancy grid and its origin metadata. */
  void save_localmap(int anchor_id, const cv::Mat &archive);
  uint64_t write_epoch() const { return write_epoch_.load(); }
  std::set<int> submap_neighbors(int id) const;
  // The mapping owner submits all snapshots synchronously. Drain them, then commit
  // the complete contraction. Old snapshots cannot overwrite the committed epoch.
  void merge_submap(int retained, int removed, const std::vector<Node *> &nodes, const cv::Mat &archive, const std::map<int, StateGroupf> &poses,
                    const StateGroupf &last_pose, const StateGroupf &retained_from_removed);

  cv::Mat load_submap_bundle(int anchor) const;
  std::map<int, int> submap_members() const;
  void refresh_submap(Node &target, int source, const cv::Mat &bundle);
  void commit_submap(Node &target, int source, const cv::Mat &bundle, const std::vector<Node *> &links, const std::map<int, StateGroupf> &poses,
                     const StateGroupf &last, const cv::Mat &expected_target = {}, const cv::Mat &expected_source = {}, int region = 0,
                     const cv::Mat &expected_region = {});
  // Ordered submission, not a durability acknowledgement. Advances the snapshot
  // epoch on acceptance; the existing writer reports any later SQL failure.
  uint64_t submit_submap(const Node &target, int source, const cv::Mat &bundle, const std::vector<Node *> &links,
                         const std::map<int, StateGroupf> &poses, const StateGroupf &last, const cv::Mat &expected_target = {},
                         const cv::Mat &expected_source = {}, int region = 0, const cv::Mat &expected_region = {});
  void consolidate_submap(Node &target, int source, const cv::Mat &bundle, const cv::Mat &expected_target, const cv::Mat &expected_source);
  std::map<int, cv::Mat> load_localmaps() const;
  std::vector<int> localmap_ids() const;
  // Streams retrieval summaries only; no descriptors, poses, links or map payloads.
  using RetrievalVisitor = std::function<void(const Node &)>;
  void visit_retrieval_summaries(const RetrievalVisitor &visitor) const;
  void visit_global_descriptors(const RetrievalVisitor &visitor) const;
  cv::Mat load_localmap(int anchor_id) const;
  void save_trajectory(const std::vector<OdomObservation> &observations);
  std::vector<OdomObservation> load_recent_trajectory(int limit) const;
  // Offline consumers require the complete journal, not the bounded UI tail.
  std::vector<OdomObservation> load_trajectory() const;
  int trajectory_high_water() const;
  void save2DMap(const cv::Mat &map, float xMin, float yMin, float cellSize) const;
  /** @brief Load the 2D map and fill origin/cell size outputs. */
  cv::Mat load2DMap(float &xMin, float &yMin, float &cellSize) const;
  /** @} */

 public:
  /**
   * @name Connection and database introspection
   * @brief Mutex-protected wrappers around protected \c *Query() methods.
   * @{*/
  /** @brief Open or create the database at @p url (empty @p url uses an in-memory database). */
  bool openConnection(const std::string &url, bool overwritten = false, bool readOnly = false);
  /** @brief Close the connection; optionally flush trashes and save in-memory DB to @p outputUrl. */
  void closeConnection(bool save = true, const std::string &outputUrl = "");
  /** @return True if a database connection is active. */
  bool isConnected() const;
  unsigned long getMemoryUsed() const;  // In bytes
  /** @return Schema version string stored in the database (e.g. "0.21.0"). */
  std::string getDatabaseVersion() const;
  long getNodesMemoryUsed() const;
  long getLinksMemoryUsed() const;
  long getImagesMemoryUsed() const;
  long getDepthImagesMemoryUsed() const;
  long getCalibrationsMemoryUsed() const;
  long getGridsMemoryUsed() const;
  long getUserDataMemoryUsed() const;
  long getFeaturesMemoryUsed() const;
  long getStatisticsMemoryUsed() const;
  int getLastNodesSize() const;  // Last committed working-memory snapshot, excluding pending writes.
  int getTotalNodesSize() const;
  ParametersMap getLastParameters() const;
  std::map<std::string, float> getStatistics(int nodeId, double &stamp, std::vector<int> *wmState = 0) const;
  std::map<int, std::pair<std::map<std::string, float>, double>> getAllStatistics() const;
  std::map<int, std::vector<int>> getAllStatisticsWmStates() const;

  /** @} */

  /**
   * @brief Run a SQL statement that does not return result rows.
   *
   * Executes @p sql on the open database (e.g. @c INSERT, @c UPDATE, @c DELETE,
   * @c CREATE, @c PRAGMA). Thread-safe wrapper around the backend's SQL execution.
   *
   * For reads, use the dedicated @ref loadNode(), @ref loadLinks() and related
   * query methods instead of raw SQL.
   *
   * @param sql Complete SQL statement (SQLite syntax for @ref DBDriverSqlite3).
   * @note The connection must be open (@ref isConnected()). On failure the driver
   *       aborts with an assertion (SQLite backend).
   * @warning @p sql is passed verbatim; sanitize any user-controlled values before calling.
   */
  void executeNoResult(const std::string &sql) const;
  // Optional, bounded advisory route snapshot. Older maps have no such table.
  void save_visit_history(const std::string &archive);
  std::string load_visit_history() const;

  /**
   * @name Load and query
   * @brief Load nodes, links and node metadata from the database.
   * @{*/

  // Load objects
  void loadLastNodes(std::list<Node *> &nodes, bool loadWordIdsOnly = false) const;  // returned nodes must be freed after usage
  /** @brief Load one node by id; caller must delete the returned pointer. */
  Node *loadNode(
      int id,
      bool *loadedFromTrash = 0);  // returned node must be freed after usage, call loadNodes() instead if more than one node should be loaded
  void loadNodes(const std::list<int> &ids, std::list<Node *> &nodes, std::set<int> *loadedFromTrash = 0,
                 bool loadWordIdsOnly = false);  // returned nodes must be freed after usage
  // Specific queries...
  void getNodeData(int nodeId, SensorRecord &data, bool images = true, bool userData = true, bool occupancyGrid = true) const;
  bool getCalibration(int nodeId, std::vector<camera::Perspective> &models) const;
  bool getNodeInfo(int nodeId, StateGroupf &pose, int &mapId, int &weight, std::string &label, double &stamp, std::vector<float> &velocity,
                   EnvSensors &sensors) const;
  std::shared_ptr<const bow::FeatureBlock> get_loop_features(int node_id) const;
  std::shared_ptr<const featmap::LandmarkMap> get_map_features(int node_id) const;

  void getLocalFeatures(int nodeId, std::multimap<int, int> &words, std::vector<cv::KeyPoint> &keypoints, std::vector<cv::Point3f> &points,
                        cv::Mat &descriptors) const;
  /** @brief Replace output with outgoing links, optionally selecting an exact type.
   */
  void loadLinks(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const;
  void getWeight(int nodeId, int &weight) const;
  // Exact last committed working-memory snapshot; ordinary node writes do not change it.
  void getLastNodeIds(std::set<int> &ids) const;
  void getAllNodeIds(std::set<int> &ids, bool ignoreChildren = false, bool ignoreBadNodes = false, bool ignoreIntermediateNodes = false) const;
  void getAllOdomPoses(std::map<int, StateGroupf> &poses, bool ignoreChildren = false, bool ignoreIntermediateNodes = false) const;
  void getAllLinks(std::multimap<int, Link> &links, bool ignoreNullLinks = true) const;
  void getLastNodeId(int &id) const;
  void getLastMapId(int &mapId) const;
  void getInvertedIndexNi(int nodeId, int &ni) const;
  // Raw detections nominate retrieval candidates, never metric connections.
  void getNodeIdByLabel(const std::string &label, int &id) const;
  void getAllLabels(std::map<int, std::string> &labels) const;
  /** @} */

 protected:
  /**
   * @brief Protected constructor for subclasses.
   */
  DBDriver(const ParametersMap &parameters = ParametersMap());
  explicit DBDriver(const DatabaseParams &parameters);

  /**
   * @name Backend implementation (subclass responsibility)
   * @brief Pure virtual SQL/backend hooks invoked by public wrappers above.
   * @{*/
  virtual bool connectDatabaseQuery(const std::string &url, bool overwritten = false, bool readOnly = false) = 0;
  virtual void disconnectDatabaseQuery(bool save = true, const std::string &outputUrl = "") = 0;
  virtual bool isConnectedQuery() const = 0;
  // Thread-safe accounting only: must not access the transaction connection.
  virtual unsigned long getMemoryUsedQuery() const = 0;  // In bytes
  virtual long getNodesMemoryUsedQuery() const = 0;
  virtual long getLinksMemoryUsedQuery() const = 0;
  virtual long getImagesMemoryUsedQuery() const = 0;
  virtual long getDepthImagesMemoryUsedQuery() const = 0;
  virtual long getCalibrationsMemoryUsedQuery() const = 0;
  virtual long getGridsMemoryUsedQuery() const = 0;
  virtual long getUserDataMemoryUsedQuery() const = 0;
  virtual long getFeaturesMemoryUsedQuery() const = 0;
  virtual long getStatisticsMemoryUsedQuery() const = 0;
  virtual int getLastNodesSizeQuery() const = 0;
  virtual int getTotalNodesSizeQuery() const = 0;
  virtual ParametersMap getLastParametersQuery() const = 0;
  virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double &stamp, std::vector<int> *wmState) const = 0;
  virtual std::map<int, std::pair<std::map<std::string, float>, double>> getAllStatisticsQuery() const = 0;
  virtual std::map<int, std::vector<int>> getAllStatisticsWmStatesQuery() const = 0;

  virtual void executeNoResultQuery(const std::string &sql) const = 0;
  virtual void save_visit_history_query(const std::string &) const { throw std::runtime_error("Visit storage unsupported"); }
  virtual std::string load_visit_history_query() const { return {}; }

  virtual void getWeightQuery(int nodeId, int &weight) const = 0;

  virtual void saveQuery(const std::list<Node *> &nodes, const std::map<int, SensorRecord> &records) = 0;
  virtual void updateQuery(const std::list<Node *> &nodes) const = 0;
  virtual void saveWorkingMemoryQuery(const std::vector<Node *> &nodes, const RunInfo &info) const = 0;
  virtual void addInfoAfterRunQuery(int size, const RunInfo &info) const = 0;

  virtual void addLinkQuery(const Link &link) const = 0;
  virtual void updateLinkQuery(const Link &link) const = 0;

  virtual void updateCalibrationQuery(int nodeId, const std::vector<camera::Perspective> &models) const = 0;

  virtual void updateDepthImageQuery(int nodeId, const cv::Mat &image) const = 0;

  virtual void addStatisticsQuery(const Statistics &statistics, bool saveWmState) const = 0;
  virtual void savePreviewImageQuery(const cv::Mat &image) const = 0;
  virtual cv::Mat loadPreviewImageQuery() const = 0;
  virtual std::map<int, RootFrame> load_root_frames_query() const = 0;
  virtual RootFrame create_root_frame_query(int anchor, const StateGroupf &pose) = 0;
  virtual void saveOptimizedPosesQuery(const std::map<int, StateGroupf> &optimizedPoses, const StateGroupf &lastPose) const = 0;
  virtual std::map<int, StateGroupf> loadOptimizedPosesQuery(StateGroupf *lastPose = 0) const = 0;
  virtual void save_trajectory_query(const std::vector<OdomObservation> &) const = 0;
  virtual std::vector<OdomObservation> load_recent_trajectory_query(int limit) const = 0;
  virtual int trajectory_high_water_query() const = 0;
  virtual void save_localmap_query(int anchor_id, const cv::Mat &archive) const = 0;
  virtual std::set<int> submap_neighbors_query(int) const { return {}; }
  virtual void merge_submap_query(int, int, const std::vector<Node *> &, const cv::Mat &, const std::map<int, StateGroupf> &, const StateGroupf &,
                                  const StateGroupf &) {
    throw std::runtime_error("Node transactions are unsupported by this database");
  }
  virtual cv::Mat load_submap_bundle_query(int) const { return {}; }
  virtual std::map<int, int> submap_members_query() const { return {}; }
  virtual void refresh_submap_query(Node &, int, const cv::Mat &) { throw std::runtime_error("Node refresh unsupported"); }
  virtual void consolidate_submap_query(Node &, int, const cv::Mat &, const cv::Mat &, const cv::Mat &) {
    throw std::runtime_error("Mature submap consolidation unsupported");
  }
  virtual std::map<int, cv::Mat> load_localmaps_query() const = 0;
  virtual std::vector<int> localmap_ids_query() const = 0;
  virtual void visit_retrieval_summaries_query(const RetrievalVisitor &visitor) const = 0;
  virtual void visit_global_descriptors_query(const RetrievalVisitor &) const {}
  virtual cv::Mat load_localmap_query(int anchor_id) const = 0;
  virtual void save2DMapQuery(const cv::Mat &map, float xMin, float yMin, float cellSize) const = 0;
  virtual cv::Mat load2DMapQuery(float &xMin, float &yMin, float &cellSize) const = 0;

  // Load objects
  virtual void loadLastNodesQuery(std::list<Node *> &nodes, bool loadWordIdsOnly) const = 0;
  virtual void loadNodesQuery(const std::list<int> &ids, std::list<Node *> &nodes, bool loadWordIdsOnly) const = 0;
  virtual void loadLinksQuery(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const = 0;

  virtual void loadNodeDataQuery(std::list<SensorRecord *> &records, bool images = true, bool userData = true, bool occupancyGrid = true) const = 0;
  virtual bool getCalibrationQuery(int nodeId, std::vector<camera::Perspective> &models) const = 0;
  virtual bool getNodeInfoQuery(int nodeId, StateGroupf &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                std::vector<float> &velocity, EnvSensors &sensors) const = 0;
  virtual std::shared_ptr<const bow::FeatureBlock> load_loop_features(int node_id) const { return {}; }
  virtual std::shared_ptr<const featmap::LandmarkMap> load_map_features(int node_id) const { return {}; }
  virtual void getLocalFeaturesQuery(int nodeId, std::multimap<int, int> &words, std::vector<cv::KeyPoint> &keypoints,
                                     std::vector<cv::Point3f> &points, cv::Mat &descriptors) const = 0;
  virtual void getLastNodeIdsQuery(std::set<int> &ids) const = 0;
  virtual void getAllNodeIdsQuery(std::set<int> &ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const = 0;
  virtual void getAllOdomPosesQuery(std::map<int, StateGroupf> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const = 0;
  virtual void getAllLinksQuery(std::multimap<int, Link> &links, bool ignoreNullLinks) const = 0;
  virtual void getLastIdQuery(const std::string &tableName, int &id, const std::string &fieldName = "id") const = 0;
  virtual void getInvertedIndexNiQuery(int nodeId, int &ni) const = 0;
  virtual void getNodeIdByLabelQuery(const std::string &label, int &id) const = 0;
  virtual void getAllLabelsQuery(std::map<int, std::string> &labels) const = 0;
  /** @brief Test seam called by the worker immediately before a batch transaction. */
  virtual void beforeAsyncBatchWrite() {}
  /** @} */

 private:
  struct WriteEntry;
  struct WorkerCommand;
  enum class WorkerState { kStopped, kRunning, kDraining, kDiscarding, kFailed };

  void beginTransactionLocked() const;
  void commitLocked() const;
  void rollbackLocked() const noexcept;
  void saveOrUpdateLocked(const std::vector<Node *> &nodes, const std::map<int, SensorRecord> &records);
  void writeSubmapLocked(const NodeCommit &commit, const std::vector<Node *> &nodes);

  uint64_t enqueueWrite(WriteBatch batch);
  void waitForAnchors(const std::set<int> &anchors);
  std::vector<OdomObservation> visibleTrajectory(int limit) const;
  bool hasCapacityLocked(size_t itemDelta, size_t byteDelta) const;
  void markSequenceCompletedLocked(uint64_t sequence);
  bool pushWorkerCommand(std::unique_ptr<WorkerCommand> command);
  void startWorkerLocked();
  void workerLoop();
  void monitorWrites();
  void stopMonitor();
  bool processBatch(bool draining);
  void discardPendingLocked();
  void failWorker(const std::string &message);
  const Node *findVisibleNodeLocked(int id) const;
  bool hasPendingInsertLocked(int id) const;
  const SensorRecord *findVisibleRecordLocked(int id) const;
  std::map<int, const Node *> visibleNodesLocked() const;

 private:
  mutable std::mutex submission_mutex_;
  std::atomic<uint64_t> write_epoch_{0};
  mutable std::mutex _lifecycleMutex;
  mutable std::mutex _pendingMutex;
  mutable std::mutex _dbSafeAccessMutex;
  std::condition_variable _pendingChanged;
  std::condition_variable _sequenceChanged;
  std::map<uint64_t, std::unique_ptr<WriteEntry>> _pendingWrites;
  std::map<uint64_t, std::unique_ptr<WriteEntry>> _inFlightWrites;
  std::set<uint64_t> _completedOutOfOrder;
  std::unique_ptr<tools::SharedQueue<std::unique_ptr<WorkerCommand>>> _workerCommands;
  std::thread _worker;
  std::thread _monitor;
  bool _monitorStop = false;                                  // _pendingMutex
  std::function<void(const std::string &)> _failureCallback;  // _lifecycleMutex
  WorkerState _workerState;
  std::atomic<bool> _accepting;
  std::atomic<bool> _workerFailed;
  std::atomic<bool> _discardRequested;
  std::atomic<bool> _workNotificationPending;
  std::string _workerError;
  size_t _pendingItems;
  size_t _pendingBytes;
  size_t _highWaterItems;
  size_t _highWaterBytes;
  uint64_t _producerWaitCount;
  uint64_t _producerWaitNanoseconds;
  size_t _lastBatchItems;
  size_t _lastBatchBytes;
  size_t _maxBatchItemsObserved;
  size_t _maxBatchBytesObserved;
  uint64_t _acceptedSequence;
  uint64_t _completedSequence;
  uint64_t _connectionSequenceBase = 0;
  // Highest pending anchor needed by a synchronous dependent write. Protected
  // by _pendingMutex; its batch must not wait for the ordinary batching timer.
  uint64_t _requiredSequence = 0;
  size_t _maxPendingItems;
  size_t _maxPendingBytes;
  std::chrono::milliseconds _producerWait;
  std::chrono::milliseconds _commitTimeout;
  double _lastWriteMs = 0, _maxWriteMs = 0;  // _pendingMutex
  size_t _maxBatchItems;
  size_t _maxBatchBytes;
  std::chrono::milliseconds _maxBatchLatency;
  std::atomic<double> _emptyTrashesTime;
  std::string _url;
};

}  // namespace sapphire::backend
