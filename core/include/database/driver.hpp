#pragma once

#include <atomic>
#include <common/common_type.hpp>
#include <database/trajectory.hpp>
#include <maplayer/topograph/root_frame.hpp>
#include <common/se3_state.hpp>
#include <database/statistics.hpp>
#include <maplayer/topograph/link.hpp>
#include <parameters.hpp>
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

#include "tools/shared_queue.hpp"

namespace sapphire {

class Node;
class DBDriver {
 public:
  struct WriteBatch {
    std::vector<std::shared_ptr<Node>> nodes;
    std::vector<OdomObservation> trajectory;
    std::optional<Statistics> statistics;
    std::optional<std::map<int, SE3f>> optimized_poses;
    SE3f last_pose;
    bool saveWmState = false;
    uint64_t epoch = 0;
  };
  uint64_t asyncWrite(WriteBatch batch);
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

  static DBDriver *create(const DatabaseParams &parameters = {});

 public:
  virtual ~DBDriver();

  virtual bool isInMemory() const { return _url.empty(); }

  const std::string &getUrl() const { return _url; }

  void asyncSave(Node *s);

  bool asyncSave(std::unique_ptr<Node> s);

  void emptyTrashes(bool async = false);
  double getEmptyTrashesTime() const { return _emptyTrashesTime.load(); }
  WorkerMetrics getWorkerMetrics() const;
  bool isWorkerHealthy() const;
  std::string getWorkerError() const;
  void setWorkerFailureCallback(std::function<void(const std::string &)> callback);

  void addLink(const Link &link);
  void write_link_pair(const Link &, bool from_cold, bool to_cold, bool update);

  void removeLink(int from, int to);

  void updateLink(const Link &link);

 public:
  struct RunInfo {
    int lastNode = 0, processMemoryUsed = -1, databaseMemoryUsed = 0;
    ParametersMap parameters;
  };
  void saveWorkingMemory(const std::vector<Node *> &nodes, const RunInfo &info);
  void addInfoAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, const ParametersMap &parameters) const;

  void addStatistics(const Statistics &statistics, bool saveWmState) const;

  std::map<int, RootFrame> load_root_frames() const;
  RootFrame create_root_frame(int anchor, const SE3f &pose);
  void saveOptimizedPoses(const std::map<int, SE3f> &optimizedPoses, const SE3f &lastPose);
  std::map<int, SE3f> loadOptimizedPoses(SE3f *lastPose = 0) const;

  uint64_t write_epoch() const { return write_epoch_.load(); }

  void save_trajectory(const std::vector<OdomObservation> &observations);
  std::vector<OdomObservation> load_recent_trajectory(int limit) const;
  std::vector<OdomObservation> load_trajectory() const;
  int trajectory_high_water() const;

 public:
  bool openConnection(const std::string &url, bool overwritten = false, bool readOnly = false);

  void closeConnection(bool save = true, const std::string &outputUrl = "");

  bool isConnected() const;
  unsigned long getMemoryUsed() const;  // In bytes

  std::string getDatabaseVersion() const;
  long getNodesMemoryUsed() const;
  long getLinksMemoryUsed() const;


  long getStatisticsMemoryUsed() const;
  int getLastNodesSize() const;  // Last committed working-memory snapshot, excluding pending writes.
  int getTotalNodesSize() const;
  ParametersMap getLastParameters() const;
  std::map<std::string, float> getStatistics(int nodeId, double &stamp, std::vector<int> *wmState = 0) const;
  std::map<int, std::pair<std::map<std::string, float>, double>> getAllStatistics() const;
  std::map<int, std::vector<int>> getAllStatisticsWmStates() const;

  void executeNoResult(const std::string &sql) const;
  void loadLastNodes(std::list<Node *> &nodes) const;  // returned nodes must be freed after usage

  Node *loadNode(
      int id,
      bool *loadedFromTrash = 0);  // returned node must be freed after usage, call loadNodes() instead if more than one node should be loaded
  void loadNodes(const std::list<int> &ids, std::list<Node *> &nodes,
                 std::set<int> *loadedFromTrash = 0);  // returned nodes must be freed after usage

  bool getNodeInfo(int nodeId, SE3f &pose, int &mapId, int &weight, std::string &label, double &stamp, std::vector<float> &velocity) const;
  // Latest visible target payload size, without decoding persisted Gaussian data.
  // Missing node -> nullopt; node with no geometry -> 0.
  std::optional<GaussianCloudSize> node_geometry_size(int id) const;

  void loadLinks(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const;
  void getWeight(int nodeId, int &weight) const;
  void getLastNodeIds(std::set<int> &ids) const;
  void getAllNodeIds(std::set<int> &ids, bool ignoreChildren = false, bool ignoreBadNodes = false, bool ignoreIntermediateNodes = false) const;
  void getAllOdomPoses(std::map<int, SE3f> &poses, bool ignoreChildren = false, bool ignoreIntermediateNodes = false) const;
  void getAllLinks(std::multimap<int, Link> &links, bool ignoreNullLinks = true) const;
  void getLastNodeId(int &id) const;
  void getLastMapId(int &mapId) const;
  void getNodeIdByLabel(const std::string &label, int &id) const;
  void getAllLabels(std::map<int, std::string> &labels) const;

 protected:
  DBDriver(const ParametersMap &parameters = ParametersMap());
  explicit DBDriver(const DatabaseParams &parameters);

  virtual bool connectDatabaseQuery(const std::string &url, bool overwritten = false, bool readOnly = false) = 0;
  virtual void disconnectDatabaseQuery(bool save = true, const std::string &outputUrl = "") = 0;
  virtual bool isConnectedQuery() const = 0;
  virtual unsigned long getMemoryUsedQuery() const = 0;  // In bytes
  virtual long getNodesMemoryUsedQuery() const = 0;
  virtual long getLinksMemoryUsedQuery() const = 0;


  virtual long getStatisticsMemoryUsedQuery() const = 0;
  virtual int getLastNodesSizeQuery() const = 0;
  virtual int getTotalNodesSizeQuery() const = 0;
  virtual ParametersMap getLastParametersQuery() const = 0;
  virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double &stamp, std::vector<int> *wmState) const = 0;
  virtual std::map<int, std::pair<std::map<std::string, float>, double>> getAllStatisticsQuery() const = 0;
  virtual std::map<int, std::vector<int>> getAllStatisticsWmStatesQuery() const = 0;

  virtual void executeNoResultQuery(const std::string &sql) const = 0;

  virtual void getWeightQuery(int nodeId, int &weight) const = 0;

  virtual void saveQuery(const std::list<Node *> &nodes) = 0;
  virtual void updateQuery(const std::list<Node *> &nodes) const = 0;
  virtual void saveWorkingMemoryQuery(const std::vector<Node *> &nodes, const RunInfo &info) const = 0;
  virtual void addInfoAfterRunQuery(int size, const RunInfo &info) const = 0;

  virtual void addLinkQuery(const Link &link) const = 0;
  virtual void updateLinkQuery(const Link &link) const = 0;

  virtual void addStatisticsQuery(const Statistics &statistics, bool saveWmState) const = 0;

  virtual std::map<int, RootFrame> load_root_frames_query() const = 0;
  virtual RootFrame create_root_frame_query(int anchor, const SE3f &pose) = 0;
  virtual void saveOptimizedPosesQuery(const std::map<int, SE3f> &optimizedPoses, const SE3f &lastPose) const = 0;
  virtual std::map<int, SE3f> loadOptimizedPosesQuery(SE3f *lastPose = 0) const = 0;
  virtual void save_trajectory_query(const std::vector<OdomObservation> &) const = 0;
  virtual std::vector<OdomObservation> load_recent_trajectory_query(int limit) const = 0;
  virtual int trajectory_high_water_query() const = 0;
  virtual void loadLastNodesQuery(std::list<Node *> &nodes) const = 0;
  virtual void loadNodesQuery(const std::list<int> &ids, std::list<Node *> &nodes) const = 0;
  virtual std::optional<GaussianCloudSize> node_geometry_size_query(int id) const = 0;
  virtual void loadLinksQuery(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const = 0;


  virtual bool getNodeInfoQuery(int nodeId, SE3f &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                std::vector<float> &velocity) const = 0;

  virtual void getLastNodeIdsQuery(std::set<int> &ids) const = 0;
  virtual void getAllNodeIdsQuery(std::set<int> &ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const = 0;
  virtual void getAllOdomPosesQuery(std::map<int, SE3f> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const = 0;
  virtual void getAllLinksQuery(std::multimap<int, Link> &links, bool ignoreNullLinks) const = 0;
  virtual void getLastIdQuery(const std::string &tableName, int &id, const std::string &fieldName = "id") const = 0;

  virtual void getNodeIdByLabelQuery(const std::string &label, int &id) const = 0;
  virtual void getAllLabelsQuery(std::map<int, std::string> &labels) const = 0;

  virtual void beforeAsyncBatchWrite() {}

 private:
  struct WriteEntry;
  struct WorkerCommand;
  enum class WorkerState { kStopped, kRunning, kDraining, kDiscarding, kFailed };

  void beginTransactionLocked() const;
  void commitLocked() const;
  void rollbackLocked() const noexcept;
  void saveOrUpdateLocked(const std::vector<Node *> &nodes);

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

}  // namespace sapphire
