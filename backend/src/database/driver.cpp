#include "backend/database/driver.hpp"

#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/parameters.hpp>
#include <backend/params/database.hpp>
#include <backend/tools/associative.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <stdexcept>

#include "backend/database/sensor_archive.hpp"
#include "backend/database/sqlite.hpp"
#include "backend/maplayer/node.hpp"

namespace sapphire::backend {

struct DBDriver::WriteEntry {
  WriteEntry(WriteBatch value, uint64_t sequenceIn, size_t bytesIn)
      : batch(std::move(value)), sequence(sequenceIn), bytes(bytesIn), enqueued(std::chrono::steady_clock::now()) {}
  WriteBatch batch;
  uint64_t sequence;
  size_t bytes;
  std::chrono::steady_clock::time_point enqueued;
};

struct DBDriver::WorkerCommand {
  enum class Type { kWork, kFlush, kDrain, kDiscard };
  WorkerCommand(Type typeIn, uint64_t sequenceIn = 0) : type(typeIn), sequence(sequenceIn) {}
  Type type;
  uint64_t sequence;
};

DBDriver* DBDriver::create(const ParametersMap& parameters) { return new DBDriverSqlite3(parameters); }

DBDriver::DBDriver(const ParametersMap& parameters) : DBDriver(DatabaseParams{}.withOverrides(parameters)) {}

DBDriver::DBDriver(const DatabaseParams& parameters)
    : _workerState(WorkerState::kStopped),
      _accepting(false),
      _workerFailed(false),
      _discardRequested(false),
      _workNotificationPending(false),
      _pendingItems(0),
      _pendingBytes(0),
      _highWaterItems(0),
      _highWaterBytes(0),
      _producerWaitCount(0),
      _producerWaitNanoseconds(0),
      _lastBatchItems(0),
      _lastBatchBytes(0),
      _maxBatchItemsObserved(0),
      _maxBatchBytesObserved(0),
      _acceptedSequence(0),
      _completedSequence(0),
      _maxPendingItems(parameters.writer_max_items),
      _maxPendingBytes(parameters.writer_max_bytes),
      _producerWait(parameters.writer_producer_wait_ms),
      _commitTimeout(parameters.writer_commit_timeout_ms),
      _maxBatchItems(parameters.writer_batch_max_items),
      _maxBatchBytes(parameters.writer_batch_max_bytes),
      _maxBatchLatency(parameters.writer_batch_max_latency_ms),
      _emptyTrashesTime(0) {
  parameters.validate();
}

DBDriver::~DBDriver() {
  stopMonitor();
  _accepting.store(false);
  _discardRequested.store(true);
  if (_workerCommands) {
    _workerCommands->push(std::make_unique<WorkerCommand>(WorkerCommand::Type::kDiscard));
  }
  _pendingChanged.notify_all();
  if (_worker.joinable()) {
    _worker.join();
  }
}

void DBDriver::closeConnection(bool save, const std::string& outputUrl) {
  {
    std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
    {
      std::lock_guard<std::mutex> pendingLock(_pendingMutex);
      _accepting.store(false);
    }
    if (_worker.joinable()) {
      if (save && !_workerFailed.load()) {
        _workerState = WorkerState::kDraining;
        pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kDrain));
      } else {
        _workerState = WorkerState::kDiscarding;
        _discardRequested.store(true);
        pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kDiscard));
      }
    }
  }
  _pendingChanged.notify_all();

  if (_worker.joinable()) {
    _worker.join();
  }
  stopMonitor();
  if (_workerCommands) {
    _workerCommands->close();
    _workerCommands.reset();
  }

  if (!save || _workerFailed.load()) {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    discardPendingLocked();
  }

  {
    std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
    if (this->isConnectedQuery()) {
      this->disconnectDatabaseQuery(save, outputUrl);
    }
  }

  {
    std::lock_guard<std::mutex> lock(_lifecycleMutex);
    _failureCallback = {};
  }
  if (save && _workerFailed.load()) {
    throw std::runtime_error("Database writes did not complete: " + getWorkerError());
  }

  std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
  if (_workerState != WorkerState::kFailed) {
    _workerState = WorkerState::kStopped;
  }
}

bool DBDriver::openConnection(const std::string& url, bool overwritten, bool readOnly) {
  EXT_LOG_DEBUG("");
  std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
  if (_worker.joinable()) {
    EXT_LOG_ERROR("Cannot open a database while its worker is still active");
    return false;
  }
  _url = url;
  {
    std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
    if (!this->connectDatabaseQuery(url, overwritten, readOnly)) {
      return false;
    }
  }
  _workerError.clear();
  _workerFailed.store(false);
  _discardRequested.store(false);
  _workerState = WorkerState::kRunning;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    _connectionSequenceBase = _acceptedSequence;
    _accepting.store(true);
  }
  startWorkerLocked();
  return true;
}

bool DBDriver::isConnected() const {
  bool r;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  r = isConnectedQuery();
  databaseLock.unlock();
  return r;
}

// In bytes
unsigned long DBDriver::getMemoryUsed() const {
  // This is diagnostic accounting, not a database-content query. The driver
  // publishes it safely without borrowing the transaction connection.
  return getMemoryUsedQuery();
}

long DBDriver::getNodesMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getNodesMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getLinksMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getLinksMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getImagesMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getImagesMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getDepthImagesMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getDepthImagesMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getCalibrationsMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getCalibrationsMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getGridsMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getGridsMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getUserDataMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getUserDataMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getFeaturesMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getFeaturesMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
long DBDriver::getStatisticsMemoryUsed() const {
  long bytes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  bytes = getStatisticsMemoryUsedQuery();
  databaseLock.unlock();
  return bytes;
}
int DBDriver::getLastNodesSize() const {
  int nodes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  nodes = getLastNodesSizeQuery();
  databaseLock.unlock();
  return nodes;
}
int DBDriver::getTotalNodesSize() const {
  int nodes;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  nodes = getTotalNodesSizeQuery();
  databaseLock.unlock();
  return nodes;
}
ParametersMap DBDriver::getLastParameters() const {
  ParametersMap parameters;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  parameters = getLastParametersQuery();
  databaseLock.unlock();
  return parameters;
}

std::map<std::string, float> DBDriver::getStatistics(int nodeId, double& stamp, std::vector<int>* wmState) const {
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
    for (auto it = queue->rbegin(); it != queue->rend(); ++it) {
      const auto& batch = it->second->batch;
      if (batch.statistics && batch.statistics->refImageId() == nodeId && !batch.statistics->data().empty()) {
        stamp = batch.statistics->stamp();
        if (wmState) *wmState = batch.saveWmState ? batch.statistics->wmState() : std::vector<int>{};
        return batch.statistics->data();
      }
    }
  return getStatisticsQuery(nodeId, stamp, wmState);
}

std::map<int, std::pair<std::map<std::string, float>, double>> DBDriver::getAllStatistics() const {
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  auto result = getAllStatisticsQuery();
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue) {
      const auto& stats = entry.second->batch.statistics;
      if (stats && stats->refImageId() > 0 && !stats->data().empty())
        result.insert_or_assign(stats->refImageId(), std::make_pair(stats->data(), stats->stamp()));
    }
  return result;
}

std::map<int, std::vector<int>> DBDriver::getAllStatisticsWmStates() const {
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  auto result = getAllStatisticsWmStatesQuery();
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue) {
      const auto& batch = entry.second->batch;
      if (batch.statistics && batch.statistics->refImageId() > 0 && !batch.statistics->data().empty()) {
        result.erase(batch.statistics->refImageId());
        if (batch.saveWmState && !batch.statistics->wmState().empty()) result.emplace(batch.statistics->refImageId(), batch.statistics->wmState());
      }
    }
  return result;
}

std::string DBDriver::getDatabaseVersion() const { return "0"; }

void DBDriver::beginTransactionLocked() const {
  EXT_LOG_DEBUG("");
  this->executeNoResultQuery("BEGIN TRANSACTION;");
}

void DBDriver::commitLocked() const {
  EXT_LOG_DEBUG("");
  this->executeNoResultQuery("COMMIT;");
}

void DBDriver::rollbackLocked() const noexcept {
  try {
    this->executeNoResultQuery("ROLLBACK;");
  } catch (...) {
  }
}

void DBDriver::executeNoResult(const std::string& sql) const {
  std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->executeNoResultQuery(sql);
}

void DBDriver::emptyTrashes(bool async) {
  const auto started = std::chrono::steady_clock::now();
  uint64_t target = 0;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    target = _acceptedSequence;
    // A synchronous drain is a persistent demand, not only a wake-up. The
    // notification may arrive while the writer is executing an earlier batch.
    if (!async) _requiredSequence = std::max(_requiredSequence, target);
  }

  if (async) {
    if (!_workNotificationPending.exchange(true) && !pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kWork))) {
      _workNotificationPending.store(false);
    }
    _pendingChanged.notify_all();
    return;
  }

  if (target != 0 && _worker.joinable() && !_workerFailed.load()) {
    pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kFlush, target));
    _pendingChanged.notify_all();
    std::unique_lock<std::mutex> pendingLock(_pendingMutex);
    _sequenceChanged.wait(pendingLock, [this, target] { return _completedSequence >= target || _workerFailed.load() || _discardRequested.load(); });
  }

  _emptyTrashesTime.store(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
  EXT_LOG_DEBUG("Total time emptying trashes = {:.6f}s...", _emptyTrashesTime.load());
}

void DBDriver::asyncSave(Node* s, std::optional<SensorRecord> data) { asyncSave(std::unique_ptr<Node>(s), std::move(data)); }

bool DBDriver::asyncSave(std::unique_ptr<Node> node, std::optional<SensorRecord> data) {
  if (!node) return true;
  WriteBatch batch;
  batch.epoch = node->write_epoch;
  if (data) {
    if (data->observation.id() != 0 && data->observation.id() != node->id()) throw std::invalid_argument("Sensor archive/node ID mismatch");
    data->observation.setId(node->id());
    batch.records.emplace(node->id(), std::move(*data));
  }
  batch.nodes.emplace_back(std::move(node));
  return asyncWrite(std::move(batch)) != 0;
}

uint64_t DBDriver::asyncWrite(WriteBatch batch) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  if (batch.submap) throw std::invalid_argument("Use submit_submap for graph transactions");
  if (batch.epoch != write_epoch()) return 0;
  std::set<int> ids;
  for (const auto& node : batch.nodes) {
    if (!node || node->id() <= 0 || !ids.insert(node->id()).second) throw std::invalid_argument("Invalid or duplicate node in write batch");
    if (node->write_epoch != batch.epoch) return 0;
  }
  return enqueueWrite(std::move(batch));
}

bool DBDriver::writeCommitted(uint64_t sequence) const {
  std::lock_guard<std::mutex> pending(_pendingMutex);
  // Discard advances the drain sequence for wakeups; it is not a commit.
  return sequence > _connectionSequenceBase && sequence <= _completedSequence && !_workerFailed.load() && !_discardRequested.load();
}

std::set<int> DBDriver::submap_neighbors(int id) const {
  std::set<int> result;
  {
    std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
    result = submap_neighbors_query(id);
  }
  std::lock_guard<std::mutex> pending(_pendingMutex);
  for (const auto& entry : visibleNodesLocked())
    if (entry.first != id && entry.second->hasLink(id)) result.insert(entry.first);
  return result;
}

cv::Mat DBDriver::load_submap_bundle(int anchor) const {
  {
    std::lock_guard<std::mutex> pending(_pendingMutex);
    for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
      for (auto it = queue->rbegin(); it != queue->rend(); ++it) {
        const auto& commit = it->second->batch.submap;
        if (!commit) continue;
        if (commit->target == anchor) return commit->bundle.clone();
        if (commit->region == anchor || (!commit->expected_source.empty() && commit->source == anchor)) return {};
      }
  }
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  return load_submap_bundle_query(anchor);
}
std::map<int, int> DBDriver::submap_members() const {
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  auto members = submap_members_query();
  std::lock_guard<std::mutex> pending(_pendingMutex);
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue) {
      const auto& commit = entry.second->batch.submap;
      if (!commit) continue;
      for (auto& [capture, owner] : members)
        if (owner == commit->region || (!commit->expected_source.empty() && owner == commit->source)) owner = commit->target;
      members[commit->source] = commit->target;
      if (commit->region) members[commit->region] = commit->target;
    }
  return members;
}
void DBDriver::refresh_submap(Node& target, int source, const cv::Mat& bundle) {
  if (target.id() <= 0 || source <= 0 || target.id() == source || bundle.empty()) throw std::invalid_argument("Invalid submap refresh");
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Writer failed before submap refresh");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  beginTransactionLocked();
  try {
    refresh_submap_query(target, source, bundle);
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
  ++write_epoch_;
}

void DBDriver::commit_submap(Node& target, int source, const cv::Mat& bundle, const std::vector<Node*>& nodes,
                             const std::map<int, StateGroupf>& poses, const StateGroupf& last, const cv::Mat& expected_target,
                             const cv::Mat& expected_source, int region, const cv::Mat& expected_region) {
  if (bundle.empty() || poses.empty() || target.id() == source) throw std::invalid_argument("Incomplete fused graph commit");
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Writer failed before fused graph commit");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  beginTransactionLocked();
  try {
    if (expected_source.empty())
      refresh_submap_query(target, source, bundle);
    else
      consolidate_submap_query(target, source, bundle, expected_target, expected_source);
    // These are prepared capture records, not rewritten odometry or redirected
    // measurements. Their inverse storage copies remain paired in one transaction.
    std::list<Node*> changed(nodes.begin(), nodes.end());
    Node link_target(target);
    link_target.mark_features_saved();  // content/history already written above; links still dirty
    if (region) consolidate_submap_query(link_target, region, bundle, bundle, expected_region);
    changed.push_back(&link_target);
    updateQuery(changed);
    saveOptimizedPosesQuery(poses, last);
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
  ++write_epoch_;
}

uint64_t DBDriver::submit_submap(const Node& target, int source, const cv::Mat& bundle, const std::vector<Node*>& links,
                                 const std::map<int, StateGroupf>& poses, const StateGroupf& last, const cv::Mat& expected_target,
                                 const cv::Mat& expected_source, int region, const cv::Mat& expected_region) {
  if (target.id() <= 0 || source <= 0 || target.id() == source || bundle.empty() || poses.empty() ||
      (region && (region == target.id() || region == source || expected_region.empty())) || (!expected_source.empty() && expected_target.empty()))
    throw std::invalid_argument("Incomplete fused graph submission");
  std::lock_guard<std::mutex> submission(submission_mutex_);
  WriteBatch batch;
  if (target.write_epoch != write_epoch()) throw std::invalid_argument("Stale fused graph target");
  batch.epoch = write_epoch() + 1;
  batch.submap.emplace(NodeCommit{target.id(), source, region, bundle.clone(), expected_target.clone(), expected_source.clone(),
                                  expected_region.clone(), poses, last});
  std::set<int> ids{target.id()};
  for (const auto* node : links) {
    if (!node || node->id() <= 0 || node->write_epoch != write_epoch() || !ids.insert(node->id()).second)
      throw std::invalid_argument("Invalid fused graph node");
    batch.nodes.push_back(std::make_shared<Node>(*node));
  }
  batch.nodes.push_back(std::make_shared<Node>(target));
  for (auto& node : batch.nodes) {
    node->setSaved(true);
    node->write_epoch = batch.epoch;
  }
  const auto sequence = enqueueWrite(std::move(batch));
  if (sequence) ++write_epoch_;
  return sequence;
}

void DBDriver::writeSubmapLocked(const NodeCommit& commit, const std::vector<Node*>& nodes) {
  auto target = std::find_if(nodes.begin(), nodes.end(), [&](const auto* node) { return node->id() == commit.target; });
  if (target == nodes.end()) throw std::logic_error("Missing fused graph target");
  if (commit.expected_source.empty())
    refresh_submap_query(**target, commit.source, commit.bundle);
  else
    consolidate_submap_query(**target, commit.source, commit.bundle, commit.expected_target, commit.expected_source);
  (*target)->mark_features_saved();
  if (commit.region) consolidate_submap_query(**target, commit.region, commit.bundle, commit.bundle, commit.expected_region);
  updateQuery(std::list<Node*>(nodes.begin(), nodes.end()));
  auto atlas = loadOptimizedPosesQuery(nullptr);
  for (const auto& [id, pose] : commit.poses) atlas.insert_or_assign(id, pose);
  saveOptimizedPosesQuery(atlas, commit.last);
}

void DBDriver::consolidate_submap(Node& target, int source, const cv::Mat& bundle, const cv::Mat& expected_target, const cv::Mat& expected_source) {
  if (target.id() <= 0 || source <= 0 || target.id() == source || bundle.empty() || expected_target.empty() || expected_source.empty())
    throw std::invalid_argument("Invalid mature submap consolidation");
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Writer failed before mature submap consolidation");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  beginTransactionLocked();
  try {
    consolidate_submap_query(target, source, bundle, expected_target, expected_source);
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
  ++write_epoch_;
}

void DBDriver::merge_submap(int retained, int removed, const std::vector<Node*>& nodes, const cv::Mat& archive,
                            const std::map<int, StateGroupf>& poses, const StateGroupf& last_pose, const StateGroupf& retained_from_removed) {
  if (retained <= 0 || removed <= 0 || retained == removed || nodes.empty() || poses.count(removed) || !retained_from_removed.isValid())
    throw std::invalid_argument("Invalid submap transaction");
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Database writer failed before submap transaction");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  beginTransactionLocked();
  try {
    merge_submap_query(retained, removed, nodes, archive, poses, last_pose, retained_from_removed);
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
  ++write_epoch_;
}

void DBDriver::saveOrUpdateLocked(const std::vector<Node*>& nodes, const std::map<int, SensorRecord>& records) {
  EXT_LOG_DEBUG("");
  std::list<Node*> toSave;
  std::list<Node*> toUpdate;
  if (this->isConnectedQuery() && nodes.size()) {
    for (std::vector<Node*>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
      if ((*i)->isSaved()) {
        toUpdate.push_back(*i);
      } else {
        toSave.push_back(*i);
      }
    }

    if (toUpdate.size()) {
      this->updateQuery(toUpdate);
    }
    if (toSave.size()) {
      this->saveQuery(toSave, records);
    }
  }
}

bool DBDriver::hasCapacityLocked(size_t itemDelta, size_t byteDelta) const {
  const bool itemCapacity = _maxPendingItems == 0 || _pendingItems + itemDelta <= _maxPendingItems;
  const bool byteCapacity = _maxPendingBytes == 0 || _pendingBytes + byteDelta <= _maxPendingBytes;
  return itemCapacity && byteCapacity;
}

void DBDriver::markSequenceCompletedLocked(uint64_t sequence) {
  if (sequence <= _completedSequence) {
    return;
  }
  _completedOutOfOrder.insert(sequence);
  while (!_completedOutOfOrder.empty() && *_completedOutOfOrder.begin() == _completedSequence + 1) {
    _completedSequence = *_completedOutOfOrder.begin();
    _completedOutOfOrder.erase(_completedOutOfOrder.begin());
  }
}

uint64_t DBDriver::enqueueWrite(WriteBatch batch) {
  size_t bytes = sizeof(WriteEntry) + batch.trajectory.capacity() * sizeof(OdomObservation);
  for (const auto& node : batch.nodes) bytes += node->getMemoryUsed();
  for (const auto& record : batch.records) bytes += record.second.memoryUsed();
  for (const auto& archive : batch.localmaps) bytes += archive.second.total() * archive.second.elemSize();
  if (batch.submap) {
    const auto& commit = *batch.submap;
    for (const auto* archive : {&commit.bundle, &commit.expected_target, &commit.expected_source, &commit.expected_region})
      bytes += archive->total() * archive->elemSize();
    bytes += commit.poses.size() * (sizeof(decltype(commit.poses)::value_type) + 3 * sizeof(void*));
  }
  if (batch.statistics) {
    bytes += batch.statistics->wmState().size() * sizeof(int);
    for (const auto& value : batch.statistics->data()) bytes += sizeof(value) + value.first.capacity() + 3 * sizeof(void*);
  }
  if (_maxPendingBytes && bytes > _maxPendingBytes) {
    failWorker("Database batch exceeds capacity: bytes=" + std::to_string(bytes) + " capacity_bytes=" + std::to_string(_maxPendingBytes));
    return 0;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + _producerWait;
  std::unique_lock<std::mutex> lock(_pendingMutex);
  if (!_accepting.load() || _workerFailed.load()) return 0;
  if (!hasCapacityLocked(1, bytes)) {
    if (_producerWait.count() == 0) {
      const auto message = "Database queue capacity exceeded: pending_items=" + std::to_string(_pendingItems) +
                           " capacity_items=" + std::to_string(_maxPendingItems) + " pending_bytes=" + std::to_string(_pendingBytes) +
                           " incoming_bytes=" + std::to_string(bytes) + " capacity_bytes=" + std::to_string(_maxPendingBytes);
      lock.unlock();
      failWorker(message);
      return 0;
    }
    ++_producerWaitCount;
    // Wake the writer now, but never replace a bounded wait with a synchronous drain.
    _requiredSequence = _acceptedSequence;
    _pendingChanged.notify_all();
    const bool ready =
        _pendingChanged.wait_until(lock, deadline, [&] { return !_accepting.load() || _workerFailed.load() || hasCapacityLocked(1, bytes); });
    _producerWaitNanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - started).count();
    if (!ready) {
      const auto message = "Database capacity wait expired: timeout_ms=" + std::to_string(_producerWait.count()) +
                           " pending_items=" + std::to_string(_pendingItems) + " pending_bytes=" + std::to_string(_pendingBytes) +
                           " high_water_items=" + std::to_string(_highWaterItems) + " high_water_bytes=" + std::to_string(_highWaterBytes);
      lock.unlock();
      failWorker(message);
      return 0;
    }
  }
  if (!_accepting.load() || _workerFailed.load()) return 0;
  for (const auto& node : batch.nodes)
    if (findVisibleNodeLocked(node->id())) node->setSaved(true);
  const uint64_t sequence = ++_acceptedSequence;
  ++_pendingItems;
  _pendingBytes += bytes;
  _pendingWrites.emplace(sequence, std::make_unique<WriteEntry>(std::move(batch), sequence, bytes));
  _highWaterItems = std::max(_highWaterItems, _pendingItems);
  _highWaterBytes = std::max(_highWaterBytes, _pendingBytes);
  lock.unlock();
  _pendingChanged.notify_all();
  if (!_workNotificationPending.exchange(true) && !pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kWork))) {
    _workNotificationPending.store(false);
    failWorker("Database worker command queue closed while accepting a batch");
    return 0;
  }
  return sequence;
}

bool DBDriver::pushWorkerCommand(std::unique_ptr<WorkerCommand> command) { return _workerCommands && _workerCommands->push(std::move(command)); }

void DBDriver::startWorkerLocked() {
  _workerCommands = std::make_unique<tools::SharedQueue<std::unique_ptr<WorkerCommand>>>();
  _workNotificationPending.store(false);
  {
    std::lock_guard<std::mutex> lock(_pendingMutex);
    _monitorStop = false;
  }
  _worker = std::thread(&DBDriver::workerLoop, this);
  _monitor = std::thread(&DBDriver::monitorWrites, this);
}

void DBDriver::workerLoop() {
  try {
    std::unique_ptr<WorkerCommand> command;
    while (_workerCommands->wait_pop(command)) {
      if (_workerFailed.load()) break;
      if (_discardRequested.load() || command->type == WorkerCommand::Type::kDiscard) {
        std::lock_guard<std::mutex> pendingLock(_pendingMutex);
        discardPendingLocked();
        break;
      }
      if (command->type == WorkerCommand::Type::kDrain) {
        while (processBatch(true)) {
        }
        break;
      }
      if (command->type == WorkerCommand::Type::kFlush) {
        for (;;) {
          {
            std::lock_guard<std::mutex> pendingLock(_pendingMutex);
            if (_completedSequence >= command->sequence) {
              break;
            }
          }
          if (!processBatch(true)) {
            break;
          }
        }
      } else {
        _workNotificationPending.store(false);
        processBatch(false);
        if (_workerFailed.load()) break;
        bool morePending = false;
        {
          std::lock_guard<std::mutex> pendingLock(_pendingMutex);
          morePending = !_pendingWrites.empty();
        }
        if (morePending && !_workNotificationPending.exchange(true)) {
          pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kWork));
        }
      }
    }
  } catch (const std::exception& exception) {
    failWorker(exception.what());
  } catch (...) {
    failWorker("Unknown database worker exception");
  }

  {
    std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
    if (_workerState != WorkerState::kFailed) {
      _workerState = WorkerState::kStopped;
    }
  }
  _pendingChanged.notify_all();
  _sequenceChanged.notify_all();
}

bool DBDriver::processBatch(bool draining) {
  std::vector<WriteEntry*> entries;
  size_t batchBytes = 0;
  {
    std::unique_lock<std::mutex> pendingLock(_pendingMutex);
    if (_pendingWrites.empty()) return false;
    const auto& first = *_pendingWrites.begin()->second;
    if (!draining && first.sequence > _requiredSequence && !_discardRequested.load() && _maxBatchLatency.count() > 0 &&
        _pendingItems < _maxBatchItems && (_maxBatchBytes == 0 || _pendingBytes < _maxBatchBytes)) {
      const auto deadline = first.enqueued + _maxBatchLatency;
#ifdef EXTERIOR_TSAN_ENABLED
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining > std::chrono::steady_clock::duration::zero())
        _pendingChanged.wait_until(pendingLock, std::chrono::system_clock::now() + remaining);
#else
      _pendingChanged.wait_until(pendingLock, deadline);
#endif
    }
    if (_workerFailed.load()) return false;
    if (_discardRequested.load()) {
      discardPendingLocked();
      return false;
    }
    while (!_pendingWrites.empty() && entries.size() < _maxBatchItems) {
      auto next = _pendingWrites.begin();
      if (_maxBatchBytes && !entries.empty() && batchBytes + next->second->bytes > _maxBatchBytes) break;
      batchBytes += next->second->bytes;
      entries.push_back(next->second.get());
      _inFlightWrites.insert(_pendingWrites.extract(next));
    }
  }
  const auto writeStarted = std::chrono::steady_clock::now();
  beforeAsyncBatchWrite();
  if (_workerFailed.load()) return false;
  if (_discardRequested.load()) {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    discardPendingLocked();
    return false;
  }
  {
    std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
    bool transactionStarted = false;
    try {
      if (!isConnectedQuery()) throw std::runtime_error("Database disconnected with pending writes");
      beginTransactionLocked();
      transactionStarted = true;
      for (const auto* entry : entries) {
        const auto& batch = entry->batch;
        // SQL updates may mutate their arguments; keep the visible snapshots immutable.
        std::vector<std::unique_ptr<Node>> copies;
        std::vector<Node*> nodes;
        for (const auto& node : batch.nodes) {
          copies.push_back(std::make_unique<Node>(*node));
          nodes.push_back(copies.back().get());
        }
        if (batch.submap)
          writeSubmapLocked(*batch.submap, nodes);
        else
          saveOrUpdateLocked(nodes, batch.records);
        for (const auto& archive : batch.localmaps) save_localmap_query(archive.first, archive.second);
        if (!batch.trajectory.empty()) save_trajectory_query(batch.trajectory);
        if (batch.statistics) addStatisticsQuery(*batch.statistics, batch.saveWmState);
      }
      if (_workerFailed.load()) throw std::runtime_error("Database transaction cancelled after writer failure");
      commitLocked();
      transactionStarted = false;
    } catch (...) {
      if (transactionStarted) rollbackLocked();
      throw;
    }
  }
  bool expired = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const auto completed = std::chrono::steady_clock::now();
    for (const auto* entry : entries) {
      expired |= completed - entry->enqueued >= _commitTimeout;
      const auto sequence = entry->sequence;
      markSequenceCompletedLocked(sequence);
      _pendingBytes -= entry->bytes;
      --_pendingItems;
      _inFlightWrites.erase(sequence);
    }
    _lastWriteMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - writeStarted).count();
    _maxWriteMs = std::max(_maxWriteMs, _lastWriteMs);
    _lastBatchItems = entries.size();
    _lastBatchBytes = batchBytes;
    _maxBatchItemsObserved = std::max(_maxBatchItemsObserved, _lastBatchItems);
    _maxBatchBytesObserved = std::max(_maxBatchBytesObserved, _lastBatchBytes);
  }
  _pendingChanged.notify_all();
  _sequenceChanged.notify_all();
  if (expired)
    failWorker("Database commit completed after deadline: timeout_ms=" + std::to_string(_commitTimeout.count()) +
               " write_ms=" + std::to_string(_lastWriteMs));
  return !entries.empty();
}

void DBDriver::discardPendingLocked() {
  _pendingWrites.clear();
  _inFlightWrites.clear();
  _pendingItems = 0;
  _pendingBytes = 0;
  _completedSequence = _acceptedSequence;
  _completedOutOfOrder.clear();
  _pendingChanged.notify_all();
  _sequenceChanged.notify_all();
}

void DBDriver::failWorker(const std::string& message) {
  std::function<void(const std::string&)> notify;
  {
    std::lock_guard<std::mutex> lock(_lifecycleMutex);
    if (_workerFailed.load()) return;  // Preserve the first cause.
    _workerError = message;
    _workerState = WorkerState::kFailed;
    _workerFailed.store(true);
    _accepting.store(false);
    notify = _failureCallback;
  }
  _pendingChanged.notify_all();
  _sequenceChanged.notify_all();
  EXT_LOG_ERROR("Database worker failed: {}", message.c_str());
  pushWorkerCommand(std::make_unique<WorkerCommand>(WorkerCommand::Type::kWork));
  if (notify) {
    try {
      notify(message);
    } catch (...) {
      EXT_LOG_ERROR("Database failure observer threw; writer remains failed");
    }
  }
}

void DBDriver::setWorkerFailureCallback(std::function<void(const std::string&)> callback) {
  std::string error;
  {
    std::lock_guard<std::mutex> lock(_lifecycleMutex);
    _failureCallback = callback;
    if (_workerFailed.load()) error = _workerError;
  }
  if (callback && !error.empty()) callback(error);
}

void DBDriver::stopMonitor() {
  {
    std::lock_guard<std::mutex> lock(_pendingMutex);
    _monitorStop = true;
  }
  _pendingChanged.notify_all();
  if (_monitor.joinable()) _monitor.join();
}

void DBDriver::monitorWrites() {
  std::unique_lock<std::mutex> lock(_pendingMutex);
  while (!_monitorStop && !_workerFailed.load()) {
    const WriteEntry* oldest = nullptr;
    for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
      if (!queue->empty() && (!oldest || queue->begin()->second->sequence < oldest->sequence)) oldest = queue->begin()->second.get();
    if (!oldest) {
      _pendingChanged.wait(lock);
      continue;
    }
    const auto deadline = oldest->enqueued + _commitTimeout;
    const auto now = std::chrono::steady_clock::now();
    if (now < deadline) {
      _pendingChanged.wait_until(lock, deadline);
      continue;
    }
    const auto message = "Database commit deadline expired: sequence=" + std::to_string(oldest->sequence) +
                         " age_ms=" + std::to_string(std::chrono::duration<double, std::milli>(now - oldest->enqueued).count()) +
                         " timeout_ms=" + std::to_string(_commitTimeout.count()) + " pending_items=" + std::to_string(_pendingItems) +
                         " pending_bytes=" + std::to_string(_pendingBytes) + " in_flight_items=" + std::to_string(_inFlightWrites.size()) +
                         " high_water_items=" + std::to_string(_highWaterItems) + " high_water_bytes=" + std::to_string(_highWaterBytes) +
                         " last_write_ms=" + std::to_string(_lastWriteMs);
    lock.unlock();
    failWorker(message);
    return;
  }
}

DBDriver::WorkerMetrics DBDriver::getWorkerMetrics() const {
  WorkerMetrics metrics;
  std::lock_guard<std::mutex> pendingLock(_pendingMutex);
  metrics.pendingItems = _pendingItems;
  metrics.pendingBytes = _pendingBytes;
  metrics.highWaterItems = _highWaterItems;
  metrics.highWaterBytes = _highWaterBytes;
  metrics.producerWaitCount = _producerWaitCount;
  metrics.producerWaitNanoseconds = _producerWaitNanoseconds;
  metrics.lastBatchItems = _lastBatchItems;
  metrics.lastBatchBytes = _lastBatchBytes;
  metrics.maxBatchItems = _maxBatchItemsObserved;
  metrics.maxBatchBytes = _maxBatchBytesObserved;
  metrics.acceptedSequence = _acceptedSequence;
  metrics.completedSequence = _completedSequence;
  metrics.failed = _workerFailed.load();
  metrics.lastWriteMs = _lastWriteMs;
  metrics.maxWriteMs = _maxWriteMs;
  const auto now = std::chrono::steady_clock::now();
  for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
    if (!queue->empty())
      metrics.oldestAgeMs = std::max(metrics.oldestAgeMs, std::chrono::duration<double, std::milli>(now - queue->begin()->second->enqueued).count());
  return metrics;
}

bool DBDriver::isWorkerHealthy() const { return !_workerFailed.load(); }

std::string DBDriver::getWorkerError() const {
  std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
  return _workerError;
}

const Node* DBDriver::findVisibleNodeLocked(int id) const {
  for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
    for (auto it = queue->rbegin(); it != queue->rend(); ++it)
      for (const auto& node : it->second->batch.nodes)
        if (node->id() == id) return node.get();
  return nullptr;
}

bool DBDriver::hasPendingInsertLocked(int id) const {
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue)
      for (const auto& node : entry.second->batch.nodes)
        if (node->id() == id && !node->isSaved()) return true;
  return false;
}

std::map<int, const Node*> DBDriver::visibleNodesLocked() const {
  std::map<int, const Node*> result;
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue)
      for (const auto& node : entry.second->batch.nodes) result.insert_or_assign(node->id(), node.get());
  return result;
}

void DBDriver::write_link_pair(const Link& link, bool from_cold, bool to_cold, bool update) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Writer failed before graph edit");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  beginTransactionLocked();
  try {
    if (from_cold) {
      if (update)
        updateLinkQuery(link);
      else
        addLinkQuery(link);
    }
    if (to_cold) {
      if (update)
        updateLinkQuery(link.inverse());
      else
        addLinkQuery(link.inverse());
    }
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
  ++write_epoch_;
}
void DBDriver::addLink(const Link& link) {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->addLinkQuery(link);
  databaseLock.unlock();
}
void DBDriver::removeLink(int from, int to) {
  this->executeNoResult(fmt::format("DELETE FROM Link WHERE from_id={:d} and to_id={:d}", from, to).c_str());
}
void DBDriver::updateLink(const Link& link) {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->updateLinkQuery(link);
  databaseLock.unlock();
}
void DBDriver::updateCalibration(int nodeId, const std::vector<camera::Perspective>& models) {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->updateCalibrationQuery(nodeId, models);
  databaseLock.unlock();
}

void DBDriver::updateDepthImage(int nodeId, const cv::Mat& image) {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->updateDepthImageQuery(nodeId, image);
  databaseLock.unlock();
}

void DBDriver::loadLastNodes(std::list<Node*>& nodes, bool loadWordIdsOnly) const {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  this->loadLastNodesQuery(nodes, loadWordIdsOnly);
  for (auto* node : nodes) node->write_epoch = write_epoch();
}

Node* DBDriver::loadNode(int id, bool* loadedFromTrash) {
  std::list<int> ids;
  ids.push_back(id);
  std::list<Node*> nodes;
  std::set<int> loadedFromTrashSet;
  loadNodes(ids, nodes, &loadedFromTrashSet);
  if (loadedFromTrash && loadedFromTrashSet.size()) {
    *loadedFromTrash = true;
  }
  if (!nodes.empty()) {
    return nodes.front();
  }
  return 0;
}
void DBDriver::loadNodes(const std::list<int>& signIds, std::list<Node*>& nodes, std::set<int>* loadedFromTrash, bool loadWordIdsOnly) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  EXT_LOG_DEBUG("");
  std::list<int> ids = signIds;
  std::list<Node*> loaded;
  std::set<int> fromTrash;
  try {
    {
      std::lock_guard<std::mutex> pendingLock(_pendingMutex);
      for (std::list<int>::iterator iter = ids.begin(); iter != ids.end();) {
        const Node* visible = findVisibleNodeLocked(*iter);
        if (visible) {
          auto copy = std::make_unique<Node>(*visible);
          // The queued original owns the initial INSERT. A returned snapshot
          // can only submit subsequent updates, including after that INSERT finishes.
          copy->setSaved(true);
          loaded.push_back(copy.release());
          fromTrash.insert(*iter);
          iter = ids.erase(iter);
        } else {
          ++iter;
        }
      }
    }
    EXT_LOG_DEBUG("");
    if (ids.size()) {
      std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
      this->loadNodesQuery(ids, loaded, loadWordIdsOnly);
    }
  } catch (...) {
    for (auto* node : loaded) delete node;
    throw;
  }
  for (auto* node : loaded) node->write_epoch = write_epoch();
  nodes.splice(nodes.end(), loaded);
  if (loadedFromTrash) loadedFromTrash->insert(fromTrash.begin(), fromTrash.end());
}

const SensorRecord* DBDriver::findVisibleRecordLocked(int id) const {
  for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
    for (auto it = queue->rbegin(); it != queue->rend(); ++it) {
      const auto found = it->second->batch.records.find(id);
      if (found != it->second->batch.records.end()) return &found->second;
    }
  return nullptr;
}

void DBDriver::getNodeData(int nodeId, SensorRecord& data, bool images, bool userData, bool occupancyGrid) const {
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const auto* node = findVisibleNodeLocked(nodeId);
    const auto* record = findVisibleRecordLocked(nodeId);
    if (record && node &&
        (hasPendingInsertLocked(nodeId) || ((!images || !record->archive.image.empty()) && (!userData || !record->archive.user.empty()) &&
                                            (!occupancyGrid || record->gridCellSize() != 0.0f)))) {
      data = *record;
      selectSensorData(data, images, userData, occupancyGrid);
      return;
    }
  }
  data = {};
  data.observation.setId(nodeId);
  std::list<SensorRecord*> records{&data};
  std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
  loadNodeDataQuery(records, images, userData, occupancyGrid);
  StateGroupf pose;
  int mapId, weight;
  std::string label;
  double stamp = 0;
  std::vector<float> velocity;
  EnvSensors sensors;
  if (getNodeInfoQuery(nodeId, pose, mapId, weight, label, stamp, velocity, sensors)) {
    data.observation.setStamp(stamp);
    data.observation.setEnvSensors(sensors);
  }
}

bool DBDriver::getCalibration(int nodeId, std::vector<camera::Perspective>& models) const {
  EXT_LOG_DEBUG("");
  bool found = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* visible = findVisibleNodeLocked(nodeId);
    if (visible) {
      models = visible->visualFrame().perspectives();
      if (models.empty()) {
        if (const auto* record = findVisibleRecordLocked(nodeId)) models = record->observation.perspectives();
      }
      found = !models.empty() || !visible->isSaved();
    }
  }

  if (!found) {
    std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
    found = this->getCalibrationQuery(nodeId, models);
    databaseLock.unlock();
  }
  return found;
}

bool DBDriver::getNodeInfo(int nodeId, StateGroupf& pose, int& mapId, int& weight, std::string& label, double& stamp, std::vector<float>& velocity,
                           EnvSensors& sensors) const {
  bool found = false;
  bool loadSensors = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* visible = findVisibleNodeLocked(nodeId);
    if (visible) {
      pose = visible->getPose();
      mapId = visible->mapId();
      weight = visible->getWeight();
      label = visible->getLabel();
      stamp = visible->getStamp();
      velocity = visible->getVelocity();
      if (const auto* record = findVisibleRecordLocked(nodeId))
        sensors = record->observation.envSensors();
      else {
        sensors.clear();
        loadSensors = true;
      }
      found = true;
    }
  }

  if (!found) {
    std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
    found = this->getNodeInfoQuery(nodeId, pose, mapId, weight, label, stamp, velocity, sensors);
    databaseLock.unlock();
  }
  if (loadSensors) {
    std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
    StateGroupf oldPose;
    int oldMap, oldWeight;
    std::string oldLabel;
    double oldStamp;
    std::vector<float> oldVelocity;
    getNodeInfoQuery(nodeId, oldPose, oldMap, oldWeight, oldLabel, oldStamp, oldVelocity, sensors);
  }
  return found;
}

std::shared_ptr<const bow::FeatureBlock> DBDriver::get_loop_features(int node_id) const {
  {
    std::lock_guard<std::mutex> lock(_pendingMutex);
    const auto* node = findVisibleNodeLocked(node_id);
    if (node && node->has_map_features()) return {};
    if (node && node->loop_features()) return node->loop_features();
  }
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  return load_loop_features(node_id);
}

std::shared_ptr<const featmap::LandmarkMap> DBDriver::get_map_features(int node_id) const {
  {
    std::lock_guard<std::mutex> lock(_pendingMutex);
    const auto* node = findVisibleNodeLocked(node_id);
    if (node && node->map_features()) return node->map_features();
  }
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  return load_map_features(node_id);
}

void DBDriver::getLocalFeatures(int nodeId, std::multimap<int, int>& words, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point3f>& points,
                                cv::Mat& descriptors) const {
  bool found = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* s = findVisibleNodeLocked(nodeId);
    if (s) {
      EXT_ASSERT(s != 0);
      found = true;
      if (!s->getWords().empty()) {
        words = s->getWords();
        if (s->getWordsKpts().empty()) {
          found = false;  // Force checking the database in case the local features were not loaded in RAM
        } else {
          words = s->getWords();
          keypoints = s->getWordsKpts();
          points = s->getWords3();
          descriptors = s->getWordsDescriptors().clone();
        }
      }
    }
  }

  if (!found) {
    std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
    getLocalFeaturesQuery(nodeId, words, keypoints, points, descriptors);
  }
}

void DBDriver::loadLinks(int nodeId, std::multimap<int, Link>& links, std::optional<Link::Type> type) const {
  if (type && !Link::isConstraintType(*type)) throw std::invalid_argument("Invalid link query type");
  links.clear();
  bool found = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* s = findVisibleNodeLocked(nodeId);
    if (s) {
      EXT_ASSERT(s != 0);
      for (std::map<int, Link>::const_iterator nIter = s->getLinks().begin(); nIter != s->getLinks().end(); ++nIter) {
        if (!type || nIter->second.type() == *type) {
          links.insert(*nIter);
        }
      }

      found = true;
    }
  }

  if (!found) {
    std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
    this->loadLinksQuery(nodeId, links, type);
    databaseLock.unlock();
  }
}

void DBDriver::getWeight(int nodeId, int& weight) const {
  bool found = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* visible = findVisibleNodeLocked(nodeId);
    if (visible) {
      weight = visible->getWeight();
      found = true;
    }
  }

  if (!found) {
    std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
    this->getWeightQuery(nodeId, weight);
    databaseLock.unlock();
  }
}

void DBDriver::getLastNodeIds(std::set<int>& ids) const {
  std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getLastNodeIdsQuery(ids);
}

void DBDriver::getAllNodeIds(std::set<int>& ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const {
  std::map<int, bool> visibleIds;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    for (const auto& entry : visible) {
      bool hasNeighbors = !ignoreChildren;
      if (ignoreChildren) {
        for (std::map<int, Link>::const_iterator nIter = entry.second->getLinks().begin(); nIter != entry.second->getLinks().end(); ++nIter) {
          if (nIter->second.type() == Link::kOdometry) {
            hasNeighbors = true;
            break;
          }
        }
      }
      visibleIds[entry.first] =
          hasNeighbors && (!ignoreBadNodes || entry.second->feature_count() != 0) && (!ignoreIntermediateNodes || entry.second->getWeight() != -1);
    }
  }

  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getAllNodeIdsQuery(ids, ignoreChildren, ignoreBadNodes, ignoreIntermediateNodes);
  databaseLock.unlock();

  for (const auto& entry : visibleIds) {
    ids.erase(entry.first);
    if (entry.second) {
      ids.insert(entry.first);
    }
  }
}

void DBDriver::getAllOdomPoses(std::map<int, StateGroupf>& poses, bool ignoreChildren, bool ignoreIntermediateNodes) const {
  std::set<int> visibleIds;
  std::map<int, StateGroupf> visiblePoses;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    for (const auto& entry : visible) {
      visibleIds.insert(entry.first);
      bool hasNeighbors = !ignoreChildren;
      if (ignoreChildren) {
        for (std::map<int, Link>::const_iterator nIter = entry.second->getLinks().begin(); nIter != entry.second->getLinks().end(); ++nIter) {
          if (nIter->second.type() == Link::kOdometry) {
            hasNeighbors = true;
            break;
          }
        }
      }
      if (hasNeighbors && (!ignoreIntermediateNodes || entry.second->getWeight() != -1)) {
        visiblePoses.insert(std::make_pair(entry.first, entry.second->getPose()));
      }
    }
  }

  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getAllOdomPosesQuery(poses, ignoreChildren, ignoreIntermediateNodes);
  databaseLock.unlock();

  for (int id : visibleIds) {
    poses.erase(id);
  }
  poses.insert(visiblePoses.begin(), visiblePoses.end());
}

void DBDriver::getAllLinks(std::multimap<int, Link>& links, bool ignoreNullLinks) const {
  std::map<int, std::vector<Link>> visibleLinks;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    for (const auto& entry : visible) {
      std::vector<Link>& nodeLinks = visibleLinks[entry.first];
      for (std::map<int, Link>::const_iterator jter = entry.second->getLinks().begin(); jter != entry.second->getLinks().end(); ++jter) {
        if (!ignoreNullLinks || jter->second.isValid()) {
          nodeLinks.push_back(jter->second);
        }
      }
    }
  }

  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getAllLinksQuery(links, ignoreNullLinks);
  databaseLock.unlock();

  for (const auto& entry : visibleLinks) {
    links.erase(entry.first);
    for (const Link& link : entry.second) {
      links.insert(std::make_pair(entry.first, link));
    }
  }
}

void DBDriver::getLastNodeId(int& id) const {
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    if (!visible.empty()) {
      id = std::max(id, visible.rbegin()->first);
    }
  }

  int databaseId = 0;
  int statisticsId = 0;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getLastIdQuery("Node", databaseId);
  this->getLastIdQuery("Statistics", statisticsId);
  databaseLock.unlock();
  id = std::max(id, std::max(databaseId, statisticsId));
}

void DBDriver::getLastMapId(int& mapId) const {
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    for (const auto& entry : visible) {
      mapId = std::max(mapId, entry.second->mapId());
    }
  }

  int databaseMapId = 0;
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getLastIdQuery("Node", databaseMapId, "map_id");
  databaseLock.unlock();
  mapId = std::max(mapId, databaseMapId);
}

void DBDriver::getInvertedIndexNi(int nodeId, int& ni) const {
  bool found = false;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const Node* visible = findVisibleNodeLocked(nodeId);
    if (visible) {
      ni = visible->retrieval_mass();
      found = true;
    }
  }

  if (!found) {
    std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
    this->getInvertedIndexNiQuery(nodeId, ni);
    databaseLock.unlock();
  }
}

void DBDriver::getNodeIdByLabel(const std::string& label, int& id) const {
  if (!label.empty()) {
    int idFound = 0;
    std::set<int> visibleIds;
    {
      std::lock_guard<std::mutex> pendingLock(_pendingMutex);
      const std::map<int, const Node*> visible = visibleNodesLocked();
      for (const auto& entry : visible) {
        visibleIds.insert(entry.first);
        if (entry.second->getLabel() == label) {
          idFound = entry.first;
          break;
        }
      }
    }

    // then look in the database
    if (idFound == 0) {
      std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
      this->getNodeIdByLabelQuery(label, idFound);
      databaseLock.unlock();
      id = visibleIds.find(idFound) == visibleIds.end() ? idFound : 0;
    } else {
      id = idFound;
    }
  } else {
    EXT_LOG_WARN("Can't search with an empty label!");
  }
}

void DBDriver::getAllLabels(std::map<int, std::string>& labels) const {
  std::set<int> visibleIds;
  std::map<int, std::string> visibleLabels;
  {
    std::lock_guard<std::mutex> pendingLock(_pendingMutex);
    const std::map<int, const Node*> visible = visibleNodesLocked();
    for (const auto& entry : visible) {
      visibleIds.insert(entry.first);
      if (!entry.second->getLabel().empty()) {
        visibleLabels.insert(std::make_pair(entry.first, entry.second->getLabel()));
      }
    }
  }

  // then look in the database
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  this->getAllLabelsQuery(labels);
  databaseLock.unlock();
  for (int id : visibleIds) {
    labels.erase(id);
  }
  labels.insert(visibleLabels.begin(), visibleLabels.end());
}

void DBDriver::saveWorkingMemory(const std::vector<Node*>& nodes, const std::map<int, SensorRecord>& records, const RunInfo& info) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (_workerFailed.load()) throw std::runtime_error("Database writer failed before working-memory finalization");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  if (!isConnectedQuery()) throw std::runtime_error("Cannot finalize working memory without a database");
  beginTransactionLocked();
  try {
    saveOrUpdateLocked(nodes, records);
    saveWorkingMemoryQuery(nodes, info);
    commitLocked();
  } catch (...) {
    rollbackLocked();
    throw;
  }
}

void DBDriver::addInfoAfterRun(int stMemSize, int lastSignAdded, int processMemUsed, int databaseMemUsed, int dictionarySize,
                               const ParametersMap& parameters) const {
  std::lock_guard<std::mutex> databaseLock(_dbSafeAccessMutex);
  if (isConnectedQuery()) addInfoAfterRunQuery(stMemSize, {lastSignAdded, processMemUsed, databaseMemUsed, dictionarySize, parameters});
}

void DBDriver::addStatistics(const Statistics& statistics, bool saveWmState) const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  addStatisticsQuery(statistics, saveWmState);
}

void DBDriver::savePreviewImage(const cv::Mat& image) const {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  savePreviewImageQuery(image);
  databaseLock.unlock();
}

cv::Mat DBDriver::loadPreviewImage() const {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  cv::Mat image = loadPreviewImageQuery();
  databaseLock.unlock();
  return image;
}

std::map<int, RootFrame> DBDriver::load_root_frames() const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  return load_root_frames_query();
}
RootFrame DBDriver::create_root_frame(int anchor, const StateGroupf& pose) {
  emptyTrashes(false);  // Establishment is once per component, never per frame.
  if (!isWorkerHealthy()) throw std::runtime_error("Cannot establish frame after database worker failure");
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  return create_root_frame_query(anchor, pose);
}

void DBDriver::saveOptimizedPoses(const std::map<int, StateGroupf>& optimizedPoses, const StateGroupf& lastPose) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (!isWorkerHealthy()) throw std::runtime_error("Writer failed before final optimized poses: " + getWorkerError());
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  saveOptimizedPosesQuery(optimizedPoses, lastPose);
  databaseLock.unlock();
}

std::map<int, StateGroupf> DBDriver::loadOptimizedPoses(StateGroupf* lastPose) const {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  std::map<int, StateGroupf> poses = loadOptimizedPosesQuery(lastPose);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue) {
      const auto& commit = entry.second->batch.submap;
      if (!commit) continue;
      for (const auto& [id, pose] : commit->poses) poses.insert_or_assign(id, pose);
      if (lastPose) *lastPose = commit->last;
    }
  return poses;
}

void DBDriver::waitForAnchors(const std::set<int>& anchors) {
  std::unique_lock<std::mutex> pending(_pendingMutex);
  _sequenceChanged.wait(pending, [&] {
    if (_workerFailed.load() || _discardRequested.load()) return true;
    bool ready = true;
    for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
      for (const auto& entry : *queue) {
        const auto& batch = entry.second->batch;
        bool dependent = false;
        for (const auto& node : batch.nodes) dependent |= anchors.count(node->id()) != 0;
        for (const auto& archive : batch.localmaps) dependent |= anchors.count(archive.first) != 0;
        for (const auto& row : batch.trajectory) dependent |= anchors.count(row.anchor_id) != 0;
        if (dependent) {
          ready = false;
          _requiredSequence = std::max(_requiredSequence, entry.first);
        }
      }
    if (!ready) _pendingChanged.notify_all();
    return ready;
  });
  if (_workerFailed.load() || _discardRequested.load()) throw std::runtime_error("Dependent database write did not complete");
}

void DBDriver::save_trajectory(const std::vector<OdomObservation>& observations) {
  if (observations.empty()) return;
  std::lock_guard<std::mutex> submission(submission_mutex_);
  std::set<int> anchors;
  for (const auto& row : observations) anchors.insert(row.anchor_id);
  waitForAnchors(anchors);
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  save_trajectory_query(observations);
}
std::vector<OdomObservation> DBDriver::visibleTrajectory(int limit) const {
  // DB -> pending lock order also covers the commit/queue-retirement gap.
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  std::map<int, OdomObservation> rows;
  for (const auto& row : load_recent_trajectory_query(limit)) rows.emplace(row.sequence, row);
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue)
      for (const auto& row : entry.second->batch.trajectory) rows.insert_or_assign(row.sequence, row);
  while (rows.size() > size_t(limit)) rows.erase(rows.begin());
  std::vector<OdomObservation> result;
  result.reserve(rows.size());
  for (const auto& row : rows) result.push_back(row.second);
  return result;
}
std::vector<OdomObservation> DBDriver::load_recent_trajectory(int limit) const {
  if (limit <= 0 || limit > 2048) throw std::invalid_argument("Invalid trajectory display limit");
  return visibleTrajectory(limit);
}
std::vector<OdomObservation> DBDriver::load_trajectory() const { return visibleTrajectory(std::numeric_limits<int>::max()); }
int DBDriver::trajectory_high_water() const {
  const auto rows = visibleTrajectory(1);
  return rows.empty() ? 0 : rows.back().sequence;
}

void DBDriver::save_visit_history(const std::string& archive) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  emptyTrashes(false);
  if (!isWorkerHealthy()) throw std::runtime_error("Writer failed before visit-history save");
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  save_visit_history_query(archive);
}

std::string DBDriver::load_visit_history() const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  return load_visit_history_query();
}

void DBDriver::save_localmap(int anchor_id, const cv::Mat& archive) {
  std::lock_guard<std::mutex> submission(submission_mutex_);
  waitForAnchors({anchor_id});
  std::lock_guard<std::mutex> database(_dbSafeAccessMutex);
  save_localmap_query(anchor_id, archive);
}

std::map<int, cv::Mat> DBDriver::load_localmaps() const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  auto result = load_localmaps_query();
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue)
      for (const auto& archive : entry.second->batch.localmaps) result.insert_or_assign(archive.first, archive.second);
  return result;
}
std::vector<int> DBDriver::localmap_ids() const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  const auto committed = localmap_ids_query();
  std::set<int> ids(committed.begin(), committed.end());
  for (const auto* queue : {&_inFlightWrites, &_pendingWrites})
    for (const auto& entry : *queue)
      for (const auto& archive : entry.second->batch.localmaps) ids.insert(archive.first);
  return {ids.begin(), ids.end()};
}
void DBDriver::visit_retrieval_summaries(const RetrievalVisitor& visitor) const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  visit_retrieval_summaries_query(visitor);
}
void DBDriver::visit_global_descriptors(const RetrievalVisitor& visitor) const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  visit_global_descriptors_query(visitor);
}
cv::Mat DBDriver::load_localmap(int anchor_id) const {
  std::lock_guard<std::mutex> lock(_dbSafeAccessMutex);
  std::lock_guard<std::mutex> pending(_pendingMutex);
  for (const auto* queue : {&_pendingWrites, &_inFlightWrites})
    for (auto it = queue->rbegin(); it != queue->rend(); ++it) {
      const auto found = it->second->batch.localmaps.find(anchor_id);
      if (found != it->second->batch.localmaps.end()) return found->second;
    }
  return load_localmap_query(anchor_id);
}

void DBDriver::save2DMap(const cv::Mat& map, float xMin, float yMin, float cellSize) const {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  save2DMapQuery(map, xMin, yMin, cellSize);
  databaseLock.unlock();
}

cv::Mat DBDriver::load2DMap(float& xMin, float& yMin, float& cellSize) const {
  std::unique_lock<std::mutex> databaseLock(_dbSafeAccessMutex);
  cv::Mat map = load2DMapQuery(xMin, yMin, cellSize);
  databaseLock.unlock();
  return map;
}

}  // namespace sapphire::backend
