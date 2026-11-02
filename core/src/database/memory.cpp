#include "database/memory.hpp"

#include <algorithm>
#include <common/common_type.hpp>
#include <parameters.hpp>
#include <tools/associative.hpp>
#include <tools/logging.hpp>
#include <tools/process.hpp>
#include <tools/timer.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "common/se3_state.hpp"
#include "database/driver.hpp"
#include "database/statistics.hpp"
#include "maplayer/topograph/node.hpp"
#include "maplayer/topograph.hpp"

namespace sapphire {

Memory::Memory(const MappingParameters& parameters, Topograph* graph)
    : owned_graph_(graph ? nullptr : std::make_unique<Topograph>()),
      graph_(graph ? *graph : *owned_graph_),
      _dbDriver(0),
      parameters_(parameters),
      _idCount(kIdStart),
      _idMapCount(kIdStart),
      _lastNode(0),
      _memoryChanged(false),
      _nodesAdded(0),
      _workingMemIntermediateNodesCount(0) {
  parameters_.validate();
  graph_.configure(parameters_.topograph);
}

bool Memory::init(const std::string& dbUrl, bool dbOverwritten) {
  if (_dbDriver) close(!dbOverwritten);
  parameter_record_.reset();
  use_previous_map_ = parameters_.topograph.use_previous_map;
  clear();
  _dbDriver.reset(DBDriver::create(parameters_.database));

  bool success = true;
  if (_dbDriver) {
    success = false;
    if (_dbDriver->openConnection(dbUrl, dbOverwritten, false)) {
      success = true;
    }
  }

  const bool persistent = parameters_.topograph.persistent_root_frames;
  graph_.restore_frames(persistent, success ? _dbDriver->load_root_frames() : std::map<int, RootFrame>{});
  loadDataFromDb();

  return success;
}

void Memory::loadDataFromDb() {
  if (_dbDriver && _dbDriver->isConnected()) {
    const bool loadAllNodesInWM = parameters_.memory.load_all_nodes_on_start;

    // Load the last working memory...
    std::list<Node*> dbNodes;

    if (!use_previous_map_) {
      // Keep global ID counters, but do not load old graph candidates.
    } else if (loadAllNodesInWM) {
      EXT_LOG_DEBUG("Loading all nodes to WM...");
      std::set<int> ids;
      _dbDriver->getAllNodeIds(ids, true);
      _dbDriver->loadNodes(std::list<int>(ids.begin(), ids.end()), dbNodes);
    } else {
      EXT_LOG_DEBUG("Loading last nodes to WM...");
      // load previous session working memory
      _dbDriver->loadLastNodes(dbNodes);
    }
    for (std::list<Node*>::reverse_iterator iter = dbNodes.rbegin(); iter != dbNodes.rend(); ++iter) {
      // ignore bad nodes
      if (!((*iter)->isBadNode() && parameters_.node.ignore_bad_frames)) {
        // insert all in WM
        // Note: it doesn't make sense to keep last STM nodes
        //       of the last session in the new STM because they can be
        //       only linked with the ones of the current session by
        //       global loop closures.
        graph_.admit(*iter, true);
        _workingMem.insert(std::make_pair((*iter)->id(), tools::steady_time_seconds()));
        if ((*iter)->getWeight() == -1) {
          ++_workingMemIntermediateNodesCount;
        }
      } else {
        delete *iter;
      }
    }

    // Assign the last node
    if (_stMem.size() > 0) {
      _lastNode = sapphire::tools::value_or(graph_.nodes(), *_stMem.rbegin(), (Node*)0);
    } else if (_workingMem.size() > 0) {
      _lastNode = sapphire::tools::value_or(graph_.nodes(), _workingMem.rbegin()->first, (Node*)0);
    }

    // Last id
    _dbDriver->getLastNodeId(_idCount);
    _idMapCount = -1;
    _dbDriver->getLastMapId(_idMapCount);
    ++_idMapCount;

    // A new empty database gets an initial record. Merely opening an existing
    // empty map must not overwrite its history with maintenance-tool settings.
    if (_lastNode == 0 && _dbDriver->getLastParameters().empty()) {
      _dbDriver->addInfoAfterRun(0, 0, 0, 0, parameterRecord());
    }
  } else {
    _idCount = kIdStart;
    _idMapCount = kIdStart;
  }

  _workingMem.insert(std::make_pair(kIdVirtual, 0));

  EXT_LOG_DEBUG("ids start with {:d}", _idCount + 1);
  EXT_LOG_DEBUG("map ids start with {:d}", _idMapCount);
}

void Memory::close(bool databaseSaved, const std::string& ouputDatabasePath) {
  EXT_LOG_DEBUG("databaseSaved={:d}", databaseSaved ? 1 : 0);

  bool databaseNameChanged = false;
  if (databaseSaved && _dbDriver) {
    databaseNameChanged =
        ouputDatabasePath.size() && _dbDriver->getUrl().size() && _dbDriver->getUrl().compare(ouputDatabasePath) != 0 ? true : false;
  }

  // Residency can change through eviction/reactivation without changing graph
  // content. Compare explicit sets so a saved close still records that change.
  bool workingMemoryChanged = false;
  if (databaseSaved && _dbDriver && use_previous_map_) {
    std::set<int> resident, saved;
    for (const auto& [id, node] : graph_.nodes())
      if (id > 0) resident.insert(id);
    _dbDriver->getLastNodeIds(saved);
    workingMemoryChanged = resident != saved;
  }
  EXT_LOG_DEBUG("_memoryChanged={:d} graph_.links_changed()={:d} databaseNameChanged={:d}", _memoryChanged ? 1 : 0, graph_.links_changed() ? 1 : 0,
                databaseNameChanged ? 1 : 0);

  if (!databaseSaved || (!_memoryChanged && !graph_.links_changed() && !databaseNameChanged && !workingMemoryChanged)) {
    EXT_LOG_DEBUG("No changes added to database.");
    if (_dbDriver) {
      if (_memoryChanged || graph_.links_changed() || databaseNameChanged) {
        EXT_LOG_DEBUG("[storage] Closing without saving pending changes: nodes={} links={} name={}", _memoryChanged ? "true" : "false",
                      graph_.links_changed() ? "true" : "false", databaseNameChanged ? "true" : "false");
      }
      if (databaseSaved) {
        _dbDriver->emptyTrashes(false);
        if (!_dbDriver->isWorkerHealthy()) throw std::runtime_error("Database writes did not complete: " + _dbDriver->getWorkerError());
      }
      _dbDriver->closeConnection(false);
      _dbDriver.reset();
    }
    this->clear();
  } else {
    EXT_LOG_DEBUG("Saving memory...");
    this->clear();
    if (_dbDriver) {
      _dbDriver->emptyTrashes();
      _dbDriver->closeConnection(true, ouputDatabasePath);
      _dbDriver.reset();
    }
  }
}

Memory::~Memory() {
  try {
    close();
  } catch (const std::exception& error) {
    EXT_LOG_ERROR("Memory destruction: {}", error.what());
    try {
      close(false);
    } catch (const std::exception& cleanup) {
      EXT_LOG_ERROR("Memory cleanup: {}", cleanup.what());
    }
  }

  if (_dbDriver) {
    EXT_LOG_WARN("Please call Memory::close() before");
  }
}

void Memory::setParameterRecord(const ParametersMap& parameters) {
  if (!_dbDriver) throw std::logic_error("Open a database before setting its parameter record");
  Parameters::validateRecord(parameters);
  parameter_record_ = parameters;
}

ParametersMap Memory::parameterRecord() const { return parameter_record_ ? *parameter_record_ : parameters_.parameters(); }

int Memory::reserve_node_id(int requested) {
  int id = requested;
  if (parameters_.node.generate_node_ids) {
    id = this->getNextId();
  } else {
    if (id <= 0) {
      EXT_LOG_ERROR(
          "Received node ID is null. "
          "Please set parameter node.generate_node_ids to \"true\" or "
          "make sure the input source provides node ids (seq).");
      return 0;
    } else if (id > _idCount) {
      _idCount = id;
    } else {
      EXT_LOG_ERROR(
          "Id of acquired node ({:d}) is smaller than the last in memory ({:d}). "
          "Please set parameter node.generate_node_ids to \"true\" or "
          "make sure the input source provides node ids (seq) over the last in "
          "memory, which is {:d}.",
          id, _idCount, _idCount);
      return 0;
    }
  }

  return id;
}

Node* Memory::admit_node(std::unique_ptr<Node> s, const Mat66_t<double>& covariance) {
  if (!s || s->mapId() != -1 || s->isSaved() || !s->getLinks().empty() ||
      !s->getPose().isValid() || !std::isfinite(s->getStamp()))
    throw std::invalid_argument("Admission requires a detached node with a valid odometry anchor");
  const int id = reserve_node_id(s->id());
  if (!id) return nullptr;
  s->assignIdentity(id, current_map_id());
  _nodesAdded = 0;
  // Storage assigns identity and residency; Topograph initializes graph evidence.
  _stMem.insert(s->id());
  try {
    graph_.admit_observation(s.get(), _lastNode, covariance);
  } catch (...) {
    _stMem.erase(s->id());
    throw;
  }
  _lastNode = s.release();
  ++_nodesAdded;
  _memoryChanged = true;
  _lastNode->write_epoch = _dbDriver ? _dbDriver->write_epoch() : 0;
  return _lastNode;
}

std::vector<int> Memory::promote_recent_nodes(bool all) {
  size_t observations = 0;
  for (int id : _stMem)
    if (getNode(id)->getWeight() >= 0) ++observations;
  std::vector<int> promoted;
  while (!_stMem.empty() && (all || (parameters_.memory.recent_nodes > 0 && observations > parameters_.memory.recent_nodes))) {
    const int id = *_stMem.begin();
    if (getNode(id)->getWeight() >= 0) --observations;
    promoted.push_back(id);
    moveNodeToWMFromSTM(id);
  }
  return promoted;
}

void Memory::addNodeToWmFromLTM(Node* node) {
  if (node) {
    EXT_LOG_DEBUG("Inserting node {:d} in WM...", node->id());
    _workingMem.insert(std::make_pair(node->id(), tools::steady_time_seconds()));
    if (node->getWeight() == -1) {
      ++_workingMemIntermediateNodesCount;
    }
    graph_.admit(node, true);
    ++_nodesAdded;
  } else {
    EXT_LOG_ERROR("[memory] Cannot complete update: current node is missing");
  }
}

void Memory::moveNodeToWMFromSTM(int id) {
  EXT_ASSERT(_stMem.count(id));
  _workingMem.emplace(id, tools::steady_time_seconds());
  if (getNode(id)->getWeight() == -1) ++_workingMemIntermediateNodesCount;
  _stMem.erase(id);
}

const Node* Memory::getNode(int id) const { return _getNode(id); }

Node* Memory::_getNode(int id) const { return sapphire::tools::value_or(graph_.nodes(), id, (Node*)0); }

size_t Memory::getWorkingMemSize(bool ignoreIntermediateNodes) const {
  // The virtual place is absent before init() and after close(). Do not
  // underflow size_t when reporting an empty working memory.
  const size_t nodes = _workingMem.size() - _workingMem.count(kIdVirtual);
  return ignoreIntermediateNodes ? nodes - std::min(nodes, size_t(_workingMemIntermediateNodesCount)) : nodes;
}

std::multimap<int, Link> Memory::getLinks(int nodeId, bool lookInDatabase) const {
  std::multimap<int, Link> links;
  if (nodeId > 0) {
    Node* s = sapphire::tools::value_or(graph_.nodes(), nodeId, (Node*)0);
    if (s) {
      links = s->getLinks();

    } else if (lookInDatabase && _dbDriver) {
      _dbDriver->loadLinks(nodeId, links, std::nullopt);
    } else {
      EXT_LOG_WARN("Cannot find node {:d} in memory", nodeId);
    }
  }
  return links;
}

std::multimap<int, Link> Memory::getAllLinks(bool lookInDatabase, bool ignoreNullLinks) const {
  std::multimap<int, Link> links;

  if (lookInDatabase && _dbDriver) {
    _dbDriver->getAllLinks(links, ignoreNullLinks);
  }

  for (std::map<int, Node*>::const_iterator iter = graph_.nodes().begin(); iter != graph_.nodes().end(); ++iter) {
    links.erase(iter->first);
    for (std::map<int, Link>::const_iterator jter = iter->second->getLinks().begin(); jter != iter->second->getLinks().end(); ++jter) {
      if (!ignoreNullLinks || jter->second.isValid()) {
        links.insert(std::make_pair(iter->first, jter->second));
      }
    }
  }

  return links;
}

int Memory::getNextId() {
  if (_idCount == std::numeric_limits<int>::max()) throw std::overflow_error("Node IDs exhausted");
  return ++_idCount;
}

int Memory::incrementMapId() {
  if (!graph_.current_odom_pose().isNull()) {
    promote_recent_nodes(true);
    graph_.reset_odometry();
    return ++_idMapCount;
  }
  return _idMapCount;
}

void Memory::updateAge(int nodeId) {
  std::map<int, double>::iterator iter = _workingMem.find(nodeId);
  if (iter != _workingMem.end()) {
    iter->second = tools::steady_time_seconds();
  }
}

std::set<int> Memory::getAllNodeIds(bool ignoreChildren) const {
  std::set<int> ids;
  if (_dbDriver) {
    _dbDriver->getAllNodeIds(ids, ignoreChildren);
  }
  for (std::map<int, Node*>::const_iterator iter = graph_.nodes().begin(); iter != graph_.nodes().end(); ++iter) {
    ids.insert(iter->first);
  }
  return ids;
}

void Memory::clear() {
  EXT_LOG_DEBUG("");

  // empty the STM
  while (_stMem.size()) {
    moveNodeToWMFromSTM(*_stMem.begin());
  }
  if (_stMem.size() != 0) {
    EXT_LOG_ERROR("_stMem must be empty here, size={:d}", (int)_stMem.size());
  }
  _stMem.clear();

  if (_dbDriver) {
    // Resolve unsaved rejected nodes before taking the final resident set: their
    // removal can change neighbors. Historical nodes remain in the database.
    std::vector<int> rejected;
    for (const auto& [id, node] : graph_.nodes())
      if (id > 0 && !node->isSaved() && node->isBadNode() && parameters_.node.ignore_bad_frames && node->getWeight() != -1) rejected.push_back(id);
    for (int id : rejected) moveToTrash(_getNode(id));
    _dbDriver->emptyTrashes();
    std::vector<Node*> residents;
    for (const auto& [id, node] : graph_.nodes())
      if (id > 0) residents.push_back(node);
    DBDriver::RunInfo info;
    info.lastNode = _lastNode ? _lastNode->id() : 0;
    std::error_code rssError;
    const auto rss = tools::resident_set_size(rssError);
    info.processMemoryUsed = rss && !rssError ? static_cast<int>(*rss) : -1;
    info.databaseMemoryUsed = _dbDriver->getMemoryUsed();
    info.parameters = parameterRecord();
    // Stop already owns these nodes. Commit directly under the database barrier,
    // without copying the entire working set into another background queue.
    _dbDriver->saveWorkingMemory(residents, info);
  }

  // Get the tree root (parents)
  if (!_dbDriver) {
    // We are not saving to database anyway, just delete now.
    graph_.clear_residents();
    _workingMem.clear();

  } else {
    std::map<int, Node*> mem = graph_.nodes();
    for (std::map<int, Node*>::iterator i = mem.begin(); i != mem.end(); ++i) {
      if (i->second) {
        // EXT_LOG_DEBUG("deleting from the working and the short-term memory: %d", i->first);
        this->moveToTrash(i->second, true, true);
      }
    }
  }

  if (_workingMem.size() != 0 && !(_workingMem.size() == 1 && _workingMem.begin()->first == kIdVirtual)) {
    EXT_LOG_ERROR("_workingMem must be empty here, size={:d}", (int)_workingMem.size());
  }
  _workingMem.clear();
  _workingMemIntermediateNodesCount = 0;
  if (graph_.nodes().size() != 0) {
    EXT_LOG_ERROR("graph_.nodes() must be empty here, size={:d}", (int)graph_.nodes().size());
  }
  graph_.clear_residents();

  EXT_LOG_DEBUG("");
  // Wait until the db trash has finished cleaning the memory
  if (_dbDriver) {
    _dbDriver->emptyTrashes();
  }
  EXT_LOG_DEBUG("");
  _lastNode = 0;
  graph_.reset_odometry();

  _nodesAdded = 0;
  _idCount = kIdStart;
  _idMapCount = kIdStart;
  _memoryChanged = false;

  if (_dbDriver) {
    _dbDriver->emptyTrashes();
  }
  graph_.reset_storage();
  EXT_LOG_DEBUG("");
}

std::list<int> Memory::transfer_to_ltm(const std::vector<int>& ids) {
  if (!_dbDriver || ids.empty()) return {};
  // Preserve every source until its snapshot has been committed. Batch flush once,
  // never delete a live node after a rejected asynchronous enqueue.
  for (int id : ids) {
    const auto* source = getNode(id);
    if (!source) throw std::runtime_error("Expired WM eviction plan");
    if (source->write_epoch != _dbDriver->write_epoch()) throw std::runtime_error("Expired WM eviction epoch");
  }
  for (int id : ids) {
    const auto* source = getNode(id);
    // A clean, restored submap has no update for the writer.
    // isSaved() alone is NOT a durability acknowledgement. The drain below
    // still waits for any previously accepted insert/update before release.
    const bool clean_submap = source->isSaved() && !source->isModified();
    if (clean_submap) continue;
    if (!queue_node(*source)) throw std::runtime_error("WM transfer rejected by database worker");
  }
  _dbDriver->emptyTrashes();
  if (_dbDriver->getWorkerMetrics().failed) throw std::runtime_error("WM transfer persistence failed");
  std::list<int> transferred;
  for (int id : ids) {
    moveToTrash(_getNode(id), true, true);
    transferred.push_back(id);
  }
  return transferred;
}
std::list<int> Memory::forget(const std::set<int>& ignoredIds) {
  const auto candidates = getRemovableNodes(std::max(1, _nodesAdded + 1), ignoredIds);
  std::vector<int> plan;
  for (const auto* node : candidates) plan.push_back(node->id());
  return transfer_to_ltm(plan);
}
size_t Memory::working_memory_bytes() const {
  size_t bytes = 0;
  for (const auto& [id, age] : _workingMem)
    if (id > 0) bytes += graph_.nodes().at(id)->getMemoryUsed();
  return bytes;
}
bool Memory::working_memory_over_budget(unsigned int node_limit) const {
  return (node_limit && getWorkingMemSize(true) > node_limit) ||
         (parameters_.memory.resident_budget_bytes && working_memory_bytes() > parameters_.memory.resident_budget_bytes);
}
std::list<int> Memory::trim_working_memory(unsigned int node_limit, const std::set<int>& protected_ids) {
  if (!_dbDriver || !working_memory_over_budget(node_limit)) return {};
  size_t nodes = getWorkingMemSize(true), bytes = working_memory_bytes();
  // A low-water target avoids repeated load/evict oscillation at the limit.
  const size_t target_nodes = node_limit ? std::max(size_t(1), size_t(node_limit) * 9 / 10) : SIZE_MAX;
  const size_t target_bytes = parameters_.memory.resident_budget_bytes ? size_t(parameters_.memory.resident_budget_bytes) * 9 / 10 : SIZE_MAX;
  const auto candidates = getRemovableNodes(std::numeric_limits<int>::max(), protected_ids, true);
  std::vector<int> plan;
  for (const auto* node : candidates) {
    if (nodes <= target_nodes && bytes <= target_bytes) break;
    plan.push_back(node->id());
    bytes -= std::min(bytes, size_t(node->getMemoryUsed()));
    if (node->getWeight() >= 0) --nodes;
  }
  auto result = transfer_to_ltm(plan);
  if ((node_limit && nodes > node_limit) || (parameters_.memory.resident_budget_bytes && bytes > parameters_.memory.resident_budget_bytes))
    EXT_LOG_WARN("WM budget constrained by protected nodes: nodes={}, bytes={}, transferred={}", nodes, bytes, result.size());
  return result;
}

uint64_t Memory::save_keyframe(int node_id, const std::vector<OdomObservation>& trajectory, const Statistics* statistics, bool save_wm, bool save_graph) {
  if (!_dbDriver) return 0;
  DBDriver::WriteBatch batch;
  batch.epoch = _dbDriver->write_epoch();
  batch.trajectory = trajectory;
  std::set<int> anchors;
  if (node_id > 0) anchors.insert(node_id);
  for (const auto& row : trajectory) anchors.insert(row.anchor_id);
  if (save_graph) {
    for (const auto& [id, node] : graph_.nodes()) if (!node->isSaved() || node->isModified()) anchors.insert(id);
    batch.optimized_poses = graph_.stored_poses();
    for (const auto& entry : graph_.world_poses()) batch.optimized_poses->insert_or_assign(entry.first, entry.second);
    for (const auto& entry : graph_.poses()) batch.optimized_poses->insert_or_assign(entry.first, entry.second);
    batch.last_pose = graph_.last_mapped_pose();
  }
  for (int id : anchors) {
    if (const auto* node = _getNode(id); node && (!node->isSaved() || (save_graph && node->isModified()))) {
      batch.nodes.push_back(std::make_shared<Node>(*node));
    }
  }
  if (statistics) {
    batch.statistics = *statistics;
    if (save_wm) {
      std::vector<int> residents;
      residents.reserve(graph_.nodes().size());
      for (const auto& [id, node] : graph_.nodes())
        if (id > 0) residents.push_back(id);
      batch.statistics->setWmState(residents);
    }
    batch.saveWmState = save_wm;
  }
  const auto sequence = _dbDriver->asyncWrite(std::move(batch));
  if (!sequence) throw std::runtime_error("Cannot submit keyframe persistence: " + _dbDriver->getWorkerError());
  for (int id : anchors) {
    if (auto* node = _getNode(id); node && (!node->isSaved() || save_graph)) {
      if (save_graph) node->setModified(false);
      node->setSaved(true);  // Snapshot is now owned and visible through the writer.
    }
  }
  if (save_graph) joinTrashThread();
  return sequence;
}

const RootFrame& Memory::create_root_frame(int anchor, const SE3f& pose) {
  if (!graph_.persistent_root_frames() || !_dbDriver) throw std::logic_error("Persistent map frames are disabled");
  if (graph_.root_frames().count(anchor)) return graph_.root_frames().at(anchor);
  saveLocationData(anchor);
  const auto frame = _dbDriver->create_root_frame(anchor, pose);
  return graph_.restore_frame(frame);
}

void Memory::saveOptimizedPoses(const std::map<int, SE3f>& optimizedPoses, const SE3f& lastPose) const {
  if (_dbDriver) {
    _dbDriver->saveOptimizedPoses(optimizedPoses, lastPose);
  }
}

std::map<int, SE3f> Memory::loadOptimizedPoses(SE3f* lastPose) const {
  if (_dbDriver) {
    auto poses = _dbDriver->loadOptimizedPoses(lastPose);
    // World poses also belong to cold nodes. RAM residency is not a validity
    // condition for a persisted coordinate frame; only deleted IDs are stale.
    std::set<int> ids;
    _dbDriver->getAllNodeIds(ids, false);
    for (auto it = poses.lower_bound(1); it != poses.end();) {
      if (!ids.count(it->first)) {
        EXT_LOG_WARN("Removing stale optimized pose for deleted node {:d}", it->first);
        it = poses.erase(it);
      } else
        ++it;
    }
    return poses;
  }
  return std::map<int, SE3f>();
}

int Memory::trajectory_high_water() const { return _dbDriver ? _dbDriver->trajectory_high_water() : 0; }

void Memory::emptyTrash() {
  if (_dbDriver) {
    _dbDriver->emptyTrashes(true);
  }
}

void Memory::checkpoint_nodes() {
  if (!_dbDriver) return;
  std::vector<Node*> dirty;
  for (const auto& [id, node] : graph_.nodes())
    if (id > 0 && (!node->isSaved() || node->isModified())) {
      if (!queue_node(*node)) throw std::runtime_error("Cannot queue dirty graph node");
      dirty.push_back(node);
    }
  joinTrashThread();
  for (auto* node : dirty) {
    node->setSaved(true);
    node->setModified(false);
  }
}

void Memory::joinTrashThread() {
  if (_dbDriver) {
    EXT_LOG_DEBUG("");
    _dbDriver->emptyTrashes();
    EXT_LOG_DEBUG("");
  }
}

class WeightAgeIdKey {
 public:
  WeightAgeIdKey(int w, double a, int i) : weight(w), age(a), id(i) {}
  bool operator<(const WeightAgeIdKey& k) const {
    if (weight < k.weight) {
      return true;
    } else if (weight == k.weight) {
      if (age < k.age) {
        return true;
      } else if (age == k.age) {
        if (id < k.id) {
          return true;
        }
      }
    }
    return false;
  }
  int weight;
  double age;
  int id;
};

std::list<Node*> Memory::getRemovableNodes(int count, const std::set<int>& ignoredIds, bool budget_pressure) {
  // EXT_LOG_DEBUG("");
  std::list<Node*> removableNodes;
  std::map<WeightAgeIdKey, Node*> weightAgeIdMap;

  size_t workingMemSize = this->getWorkingMemSize(true);

  // Find the last index to check...
  EXT_LOG_DEBUG("mem.size()={:d}, ignoredIds.size()={:d}", (int)workingMemSize, (int)ignoredIds.size());

  if (workingMemSize > 0 || (budget_pressure && getWorkingMemSize(false) > 0)) {
    // A revisited canonical submap can have a small ID. Protect recently used
    // working-memory entries by their access time, never by their numeric key.
    std::set<int> recent;
    if (!budget_pressure && graph_.getLastLoopClosureId() > 0) {
      std::vector<std::pair<double, int>> visits;
      for (const auto& entry : _workingMem)
        if (entry.first > 0) visits.emplace_back(entry.second, entry.first);
      const size_t budget = std::min(visits.size(), size_t(std::max(0, int(parameters_.memory.protected_recent_ratio * workingMemSize))));
      std::partial_sort(visits.begin(), visits.begin() + budget, visits.end(), std::greater<std::pair<double, int>>{});
      for (size_t i = 0; i < budget; ++i) recent.insert(visits[i].second);
    }

    // Ignore neighbor of the last location in STM (for neighbor links redirection issue during Rehearsal).
    Node* lastInSTM = 0;
    if (_stMem.size()) {
      lastInSTM = graph_.nodes().at(*_stMem.begin());
    }

    for (std::map<int, double>::const_iterator memIter = _workingMem.begin(); memIter != _workingMem.end(); ++memIter) {
      if ((_lastNode && memIter->first == _lastNode->id()) || recent.count(memIter->first) || memIter->first == graph_.getLastLoopClosureId()) {
        // ignore recent memory
      } else if (memIter->first > 0 && ignoredIds.find(memIter->first) == ignoredIds.end() && (!lastInSTM || !lastInSTM->hasLink(memIter->first))) {
        Node* s = this->_getNode(memIter->first);
        if (s) {
          // Links must not be in STM to be removable, rehearsal issue
          bool foundInSTM = false;
          for (std::map<int, Link>::const_iterator iter = s->getLinks().begin(); iter != s->getLinks().end(); ++iter) {
            if (_stMem.find(iter->first) != _stMem.end()) {
              EXT_LOG_DEBUG("Ignored {:d} because it has a link ({:d}) to STM", s->id(), iter->first);
              foundInSTM = true;
              break;
            }
          }
          if (!foundInSTM && (s->getWeight() >= 0 || (budget_pressure && s->getWeight() == -1))) {
            // Less weighted node priority to be transferred
            // Ignore intermediate nodes
            weightAgeIdMap.insert(
                std::make_pair(WeightAgeIdKey(budget_pressure ? 0 : s->getWeight(),
                                              !budget_pressure && parameters_.memory.eviction_order_by_id ? 0.0 : memIter->second, s->id()),
                               s));
          }
        } else {
          EXT_LOG_ERROR("[memory] Working-memory node {} has no resident node during eviction", memIter->first);
        }
      } else {
        // EXT_LOG_DEBUG("Ignoring id %d", memIter->first);
      }
    }

    for (const auto& entry : weightAgeIdMap) {
      removableNodes.push_back(entry.second);
      if (removableNodes.size() >= static_cast<unsigned>(count)) break;
    }
  } else {
    EXT_LOG_DEBUG("[memory] No removable nodes available");
  }
  return removableNodes;
}

void Memory::moveToTrash(Node* s, bool keepLinkedToGraph, bool committed) {
  // EXT_LOG_DEBUG("id=%d", s?s->id():0);
  if (s) {
    if (graph_.root_frames().count(s->id()) && !keepLinkedToGraph) throw std::runtime_error("Cannot delete a component datum");
    if (s->getWeight() == -1 && isInWM(s->id())) --_workingMemIntermediateNodesCount;

    // it is a bad node (not saved), remove links!
    if (keepLinkedToGraph && !s->isSaved() && s->isBadNode() && parameters_.node.ignore_bad_frames && s->getWeight() != -1) {
      keepLinkedToGraph = false;
    }

    // If not saved to database
    if (!keepLinkedToGraph) {
      EXT_ASSERT_MSG(this->isInSTM(s->id()) || this->isInWM(s->id()),
                     "Deleting location ({:d}) outside the "
                     "WM/STM is not implemented! STM size={:d} WM size={:d}",
                     s->id(), this->getStMem().size(), this->getWorkingMem().size());
      graph_.detach(*s);
    }

    _workingMem.erase(s->id());
    _stMem.erase(s->id());
    graph_.release(s->id());
    if (_nodesAdded > 0) {
      --_nodesAdded;
    }

    if (_lastNode == s) {
      _lastNode = 0;
      if (_stMem.size()) {
        _lastNode = this->_getNode(*_stMem.rbegin());
      } else if (_workingMem.size()) {
        _lastNode = this->_getNode(_workingMem.rbegin()->first);
      }
    }

    if ((parameters_.archive.keep_unlinked_nodes || keepLinkedToGraph || s->isSaved()) && _dbDriver && s->id() > 0) {
      if (committed)
        delete s;
      else
        _dbDriver->asyncSave(s);
    } else {
      delete s;
    }
  }
}

int Memory::getLastNodeId() const { return _lastNode ? _lastNode->id() : _idCount; }

const Node* Memory::getLastWorkingNode(bool ignoreIntermediateNodes) const {
  if (ignoreIntermediateNodes && _lastNode && _lastNode->getWeight() == -1) {
    for (std::map<int, Node*>::const_reverse_iterator iter = graph_.nodes().rbegin(); iter != graph_.nodes().rend(); ++iter) {
      if (iter->second->getWeight() != -1) {
        return iter->second;
      }
    }
    return 0;
  }
  return _lastNode;
}

void Memory::deleteLocation(int locationId, bool keepLinkedInDb) {
  EXT_LOG_DEBUG("Deleting location {:d} (keepLinkedInDb={})", locationId, keepLinkedInDb ? "true" : "false");
  Node* location = _getNode(locationId);
  if (location) {
    this->moveToTrash(location, keepLinkedInDb);
    _memoryChanged = true;
  } else {
    EXT_LOG_WARN("Location {:d} has not been found in STM/WM, cannot delete it.", locationId);
  }
}

void Memory::saveLocationData(int id) {
  auto* node = _getNode(id);
  if (!node || !_dbDriver || id <= 0 || node->isSaved()) return;
  if (!queue_node(*node)) throw std::runtime_error("Cannot queue node persistence");
  node->setSaved(true);
}

// compute transform fromId -> toId

bool Memory::addLink(const Link& link, bool addInDatabase) {
  EXT_ASSERT(Link::isConstraintType(link.type()));
  if (getNode(link.from()) && getNode(link.to())) {
    graph_.apply_links({link}, {});
    return true;
  }
  if (!addInDatabase || !_dbDriver) return false;
  auto drafts = graph_.prepare_cold_link(link, false);
  _dbDriver->write_link_pair(link, !getNode(link.from()), !getNode(link.to()), false);
  graph_.commit_cold_link(drafts);
  // The DB fence rejects pre-edit archive snapshots. Refresh the live owners,
  // exactly as for content/contraction commits; cold drafts remain on disk.
  for (const auto& [id, node] : graph_.nodes()) node->write_epoch = _dbDriver->write_epoch();
  return true;
}

SE3f Memory::getOdomPose(int nodeId, bool lookInDatabase) const {
  if (const auto* node = getNode(nodeId)) return node->getPose();
  SE3f pose;
  int mapId, weight;
  std::string label;
  double stamp;
  std::vector<float> velocity;

  getNodeInfo(nodeId, pose, mapId, weight, label, stamp, velocity, lookInDatabase);
  return pose;
}

bool Memory::getNodeInfo(int nodeId, SE3f& odomPose, int& mapId, int& weight, std::string& label, double& stamp, std::vector<float>& velocity,
                         bool lookInDatabase) const {
  const Node* s = this->getNode(nodeId);
  if (s) {
    odomPose = s->getPose();
    mapId = s->mapId();
    weight = s->getWeight();
    label = std::string(s->getLabel());
    stamp = s->getStamp();
    velocity = std::vector<float>(s->getVelocity());
    return true;
  } else if (lookInDatabase && _dbDriver) {
    return _dbDriver->getNodeInfo(nodeId, odomPose, mapId, weight, label, stamp, velocity);
  }
  return false;
}

bool Memory::queue_node(const Node& node) const {
  return _dbDriver->asyncSave(std::make_unique<Node>(node));
}

std::set<int> Memory::reactivateNodes(const std::list<int>& ids, unsigned int maxLoaded, double& timeDbAccess) {
  tools::Timer timer;
  std::vector<int> pending;
  std::set<int> unique;
  for (int id : ids) {
    if (id <= 0 || !unique.insert(id).second) continue;
    if (getNode(id)) {
      updateAge(id);
      continue;
    }
    pending.push_back(id);
  }
  std::set<int> totalLoaded;
  size_t cursor = 0, ordinary_loaded = 0;
  // The quota counts successful non-intermediate loads, never requested IDs.
  // Missing/intermediate records consume neither the quota nor a recursive call.
  while (_dbDriver && cursor < pending.size() && (!maxLoaded || ordinary_loaded < maxLoaded)) {
    const size_t count = maxLoaded ? std::min(pending.size() - cursor, size_t(maxLoaded) - ordinary_loaded) : pending.size() - cursor;
    std::list<int> batch(pending.begin() + cursor, pending.begin() + cursor + count);
    cursor += count;
    std::list<Node*> reactivatedSigns;
    _dbDriver->loadNodes(batch, reactivatedSigns);
    std::list<int> idsLoaded;
    for (auto i = reactivatedSigns.begin(); i != reactivatedSigns.end(); ++i) {
      if ((*i)->getWeight() < -1) {
        delete *i;
        continue;
      }

      idsLoaded.push_back((*i)->id());
      totalLoaded.insert((*i)->id());
      if ((*i)->getWeight() >= 0) ++ordinary_loaded;
      // append to working memory
      this->addNodeToWmFromLTM(*i);
      // Pending DB snapshots may still carry a block even for a summary-only request.
    }
  }
  timeDbAccess = timer.elapsed_seconds();
  return totalLoaded;
}

// returns all non-null poses and links
// if lookInDatabase is false, intermediate nodes are ignored and new neighbor links between non-intermediate nodes are returned
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void Memory::getMetricConstraints(const std::set<int>& ids, std::map<int, SE3f>& poses, std::multimap<int, Link>& links, bool lookInDatabase) {
  EXT_LOG_DEBUG("");
  for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter) {
    SE3f pose = getOdomPose(*iter, lookInDatabase);
    if (!pose.isNull()) {
      poses.insert(std::make_pair(*iter, pose));
    }
  }

  graph_.metric_constraints(ids, poses, links, lookInDatabase,
                            [this, lookInDatabase](int id, int&, std::multimap<int, Link>& result) { result = getLinks(id, lookInDatabase); });
}

std::set<int> Memory::metric_component(int root) const {
  return graph_.metric_component(root, getAllNodeIds(), [this](int id) { return getLinks(id, true); });
}
std::map<int, int> Memory::getNeighborsId(int nodeId, int depth, int maxChecked, bool increment, bool ignoreLoops, bool ignoreIntermediate,
                                          const std::set<int>& nodes, double* seconds) const {
  Topograph::ColdLinks load;
  if (_dbDriver && maxChecked != 0)
    load = [this](int id, int& weight, std::multimap<int, Link>& links) {
      _dbDriver->getWeight(id, weight);
      _dbDriver->loadLinks(id, links, std::nullopt);
    };
  return graph_.getNeighborsId(nodeId, depth, maxChecked, increment, ignoreLoops, ignoreIntermediate, nodes, seconds, load);
}
}  // namespace sapphire
