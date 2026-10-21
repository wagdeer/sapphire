#include "backend/memory.hpp"

#include <algorithm>
#include <array>
#include <backend/common/common_type.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/parameters.hpp>
#include <backend/params/archive.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/navimap.hpp>
#include <backend/params/node.hpp>
#include <backend/params/pgo.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/topograph.hpp>
#include <backend/maplayer/featmap/merge.hpp>
#include <backend/tools/associative.hpp>
#include <backend/tools/filesystem.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/process.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/timer.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "backend/common/state_group.hpp"
#include "backend/database/compression.hpp"
#include "backend/database/driver.hpp"
#include "backend/database/sensor_archive.hpp"
#include "backend/database/statistics.hpp"
#include "backend/maplayer/topograph.hpp"
#include "backend/maplayer/node.hpp"
#include "backend/parameters.hpp"
#include "backend/registration/retrieval/fbow.hpp"
#include "backend/tools/seal_trace.hpp"

namespace sapphire::backend {

namespace {
// Accepted-edge bookkeeping is shared by resident updates and private submap
// drafts. Deleting an accepted closure is not an inverse of a trial insertion.

}  // namespace

const int Memory::kIdStart = 0;
const int Memory::kIdVirtual = -1;
const int Memory::kIdInvalid = 0;

Memory::Memory(const ParametersMap& parameters, Topograph* graph, RuntimeResources resources)
    : resources_(std::move(resources)),
      owned_graph_(graph ? nullptr : std::make_unique<Topograph>()),
      graph_(graph ? *graph : *owned_graph_),
      _dbDriver(0),
      parameters_(LoadParameters::from_map(parameters)),
      node_params_(NodeParams{}.withOverrides(parameters_)),
      archive_params_(ArchiveParams{}.withOverrides(parameters_)),
      topograph_params_(TopographParams{}.withOverrides(parameters_)),
      memory_params_(MemoryParams{}.withOverrides(parameters_)),
      navimap_params_(NavimapParams{}.withOverrides(parameters_)),
      pose_graph_params_(PGOParams{}.withOverrides(parameters_)),
      registration_params_(RegistrationParams{}.withOverrides(parameters_)),
      feature_params_(FeatureParams{}.withOverrides(parameters_)),
      _idCount(kIdStart),
      _idMapCount(kIdStart),
      _lastNode(0),
      _memoryChanged(false),
      _nodesAdded(0),
      _workingMemIntermediateNodesCount(0),
      _stMemIntermediateNodesCount(0),
      _allNodesInWM(true) {
  graph_.configure(topograph_params_);
  scene_faiss_ = feature_params_.uses_xfeat();
  if (feature_params_.enabled && !scene_faiss_)
    _vwd = resources_.prepared_vocabulary ? std::make_unique<FbowIndex>(resources_.prepared_vocabulary)
                                          : std::make_unique<FbowIndex>(resources_.vocabulary_path, resources_.vocabulary_hash);
}

bool Memory::init(const std::string& dbUrl, bool dbOverwritten) {
  if (_dbDriver) close(!dbOverwritten);
  parameter_record_.reset();
  use_previous_map_ = topograph_params_.use_previous_map;
  clear();
  _dbDriver.reset(DBDriver::create(parameters_));

  bool success = true;
  if (_dbDriver) {
    success = false;
    if (_dbDriver->openConnection(dbUrl, dbOverwritten, false)) {
      success = true;
    }
  }

  const bool persistent = topograph_params_.persistent_root_frames;
  graph_.restore_frames(persistent, success ? _dbDriver->load_root_frames() : std::map<int, RootFrame>{});
  loadDataFromDb();

  return success;
}

void Memory::loadDataFromDb() {
  if (_dbDriver && _dbDriver->isConnected()) {
    const bool loadAllNodesInWM = memory_params_.load_all_nodes_on_start;

    // Load the last working memory...
    std::list<Node*> dbNodes;

    if (!use_previous_map_) {
      // Keep global ID counters, but do not load old graph/BoW candidates.
    } else if (loadAllNodesInWM) {
      EXT_LOG_DEBUG("Loading all nodes to WM...");
      std::set<int> ids;
      _dbDriver->getAllNodeIds(ids, true);
      _dbDriver->loadNodes(std::list<int>(ids.begin(), ids.end()), dbNodes, 0, true);
    } else {
      EXT_LOG_DEBUG("Loading last nodes to WM...");
      // load previous session working memory
      _dbDriver->loadLastNodes(dbNodes, true);
    }
    for (std::list<Node*>::reverse_iterator iter = dbNodes.rbegin(); iter != dbNodes.rend(); ++iter) {
      // ignore bad nodes
      if (!((*iter)->isBadNode() && node_params_.ignore_bad_frames)) {
        // insert all in WM
        // Note: it doesn't make sense to keep last STM images
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

    // Get labels
    EXT_LOG_DEBUG("Get labels");
    _dbDriver->getAllLabels(_labels);

    EXT_LOG_DEBUG("Check if all nodes are in Working Memory");
    for (std::map<int, Node*>::const_iterator iter = graph_.nodes().begin(); iter != graph_.nodes().end() && _allNodesInWM; ++iter) {
      for (std::map<int, Link>::const_iterator jter = iter->second->getLinks().begin(); jter != iter->second->getLinks().end(); ++jter) {
        if (graph_.nodes().find(jter->first) == graph_.nodes().end()) {
          _allNodesInWM = false;
          break;
        }
      }
    }
    EXT_LOG_DEBUG("allNodesInWM()={}", _allNodesInWM ? "true" : "false");

    // Assign the last node
    if (_stMem.size() > 0) {
      _lastNode = sapphire::backend::tools::value_or(graph_.nodes(), *_stMem.rbegin(), (Node*)0);
    } else if (_workingMem.size() > 0) {
      _lastNode = sapphire::backend::tools::value_or(graph_.nodes(), _workingMem.rbegin()->first, (Node*)0);
    }

    // Last id
    _dbDriver->getLastNodeId(_idCount);
    _idMapCount = -1;
    _dbDriver->getLastMapId(_idMapCount);
    ++_idMapCount;

    // The FBoW tree is immutable and loaded from configuration. Runtime
    // postings are reconstructed directly from the loaded nodes.
    EXT_LOG_DEBUG("Adding word references...");
    tools::Timer timer;
    graph_.restore_members(_dbDriver->submap_members());
    std::set<int> placement_owners;
    for (const auto& [capture, owner] : graph_.members()) placement_owners.insert(owner);
    for (int owner : placement_owners) {
      auto archive = _dbDriver->load_submap_bundle(owner);
      if (!archive.empty()) graph_.restore_local_poses(owner, featmap::NodeObservations::decode(archive));
    }
    // Rebuild the global index without loading cold graph nodes or feature blocks.
    if (use_previous_map_) _dbDriver->visit_retrieval_summaries([&](const Node& summary) { index_retrieval_summary(summary); });
    if (use_previous_map_ && scene_faiss_)
      _dbDriver->visit_global_descriptors([&](const Node& summary) {
        if (!graph_.is_submap_member(summary.id())) global_references_.set(summary.id(), summary.featmap().global_descriptors());
      });
    for (const auto& [id, mass] : retrieval_mass_)
      if (!graph_.nodes().count(id)) {
        _allNodesInWM = false;
        break;
      }
    // Enable loaded nodes
    const std::map<int, Node*>& nodes = this->getNodes();
    for (std::map<int, Node*>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
      Node* s = this->_getNode(i->first);
      EXT_ASSERT(s != 0);
      if (_vwd && s->has_natural_features() && s->vocabulary_hash() != _vwd->getFBoWVocabularyHash())
        throw std::runtime_error("Database vocabulary hash does not match configured vocabulary");

      if (s->feature_count()) {
        index_retrieval_summary(*s);
        s->setEnabled(true);
      }
    }
    if (memory_params_.preload_visual_features) {
      for (auto entry = graph_.nodes().rbegin();
           entry != graph_.nodes().rend() && recent_feature_nodes_.size() < static_cast<size_t>(memory_params_.feature_cache_nodes); ++entry) {
        if (entry->second->has_map_features()) {
          auto map = _dbDriver->get_map_features(entry->first);
          if (map) {
            entry->second->cache_map_features(std::move(map));
            recent_feature_nodes_.push_front(entry->first);
          }
          continue;
        }
        auto block = _dbDriver->get_loop_features(entry->first);
        if (block) {
          entry->second->set_loop_features(std::move(block));
          recent_feature_nodes_.push_front(entry->first);
        }
      }
    }
    EXT_LOG_DEBUG("Total word references added = {:d} (in {:.6f} s)", (_vwd ? _vwd->getTotalActiveReferences() : 0), timer.lap_seconds());

    // A new empty database gets an initial record. Merely opening an existing
    // empty map must not overwrite its history with maintenance-tool settings.
    if (_lastNode == 0 && _dbDriver->getLastParameters().empty()) {
      _dbDriver->addInfoAfterRun(0, 0, 0, 0, 0, parameterRecord());
    }
  } else {
    _idCount = kIdStart;
    _idMapCount = kIdStart;
  }

  // Reuse decoded demand reads even when predictive prefetch is disabled.
  // The cache owns immutable, incarnation-checked snapshots and starts no thread until planned.
  if (_dbDriver) {
    auto* driver = _dbDriver.get();
    submap_prefetch_ =
        std::make_unique<featmap::PrefetchCache>(memory_params_.feature_cache_nodes, [driver](int id) { return driver->get_map_features(id); });
  }
  _workingMem.insert(std::make_pair(kIdVirtual, 0));

  EXT_LOG_DEBUG("ids start with {:d}", _idCount + 1);
  EXT_LOG_DEBUG("map ids start with {:d}", _idMapCount);
}

void Memory::close(bool databaseSaved, const std::string& ouputDatabasePath) {
  submap_prefetch_.reset();  // Join outstanding reads before closing the driver.
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

ParametersMap Memory::parameterRecord() const { return parameter_record_ ? *parameter_record_ : parameters_; }

int Memory::reserve_node_id(int requested) {
  int id = requested;
  if (node_params_.generate_node_ids) {
    id = this->getNextId();
  } else {
    if (id <= 0) {
      EXT_LOG_ERROR(
          "Received image ID is null. "
          "Please set parameter node.generate_node_ids to \"true\" or "
          "make sure the input source provides image ids (seq).");
      return 0;
    } else if (id > _idCount) {
      _idCount = id;
    } else {
      EXT_LOG_ERROR(
          "Id of acquired image ({:d}) is smaller than the last in memory ({:d}). "
          "Please set parameter node.generate_node_ids to \"true\" or "
          "make sure the input source provides image ids (seq) over the last in "
          "memory, which is {:d}.",
          id, _idCount, _idCount);
      return 0;
    }
  }

  return id;
}

size_t Memory::mean_feature_count() const {
  if (retrieval_mass_.empty()) return 0;
  if (_vwd) return _vwd->getTotalActiveReferences() / retrieval_mass_.size();
  uint64_t count = 0;
  for (const auto& [node, n] : retrieval_mass_) count += n;
  return count / retrieval_mass_.size();
}

Node* Memory::admit_node(std::unique_ptr<Node> s, SensorRecord record, const Mat66_t<double>& covariance, const std::vector<float>& velocity) {
  if (!s || s->id() != _idCount || graph_.nodes().count(s->id()) || s->mapId() != _idMapCount)
    throw std::invalid_argument("Expired or duplicate node admission");
  // Validate and invert before publishing any archive or vocabulary entry.
  auto neighbor = graph_.odometry_links(*s, _lastNode, session_has_observations_, covariance, topograph_params_.ignore_odometry_cross_covariance);
  const int id = s->id();

  // Register the final representation once: frame frequencies for appearance-
  // only nodes, fixed landmark votes for metric submaps. Preparation above has
  // no transient frame postings to remove or replace.
  if (_vwd && s->loop_features()) {
    for (const auto& word : s->retrieval_words())
      if (word.word_id > 0) _vwd->add_word_references(word.word_id, id, word.count);
    s->setEnabled(true);
  }
  sensor_records_.insert_or_assign(id, std::move(record));
  if (velocity.size() == 6) s->setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
  addNodeToStm(s.get(), covariance, neighbor ? &*neighbor : nullptr);
  _lastNode = s.release();
  graph_.begin_odometry(_lastNode->getPose());
  latest_odom_pose_ = _lastNode->getPose();
  session_has_observations_ = true;
  _lastNode->write_epoch = _dbDriver ? _dbDriver->write_epoch() : 0;
  return _lastNode;
}

std::map<int, int> Memory::maintain_residency() {
  int notIntermediateNodesCount = 0;
  for (std::set<int>::iterator iter = _stMem.begin(); iter != _stMem.end(); ++iter) {
    const Node* s = this->getNode(*iter);
    EXT_ASSERT(s != 0);
    if (s->getWeight() >= 0) {
      ++notIntermediateNodesCount;
    }
  }
  std::map<int, int> reducedIds;
  while (_stMem.size() && memory_params_.recent_nodes > 0 && static_cast<unsigned int>(notIntermediateNodesCount) > memory_params_.recent_nodes) {
    int id = *_stMem.begin();
    Node* s = this->_getNode(id);
    EXT_ASSERT(s != 0);
    if (s->getWeight() >= 0) {
      --notIntermediateNodesCount;
    }

    int reducedTo = 0;
    moveNodeToWMFromSTM(id, &reducedTo);

    if (reducedTo > 0) {
      reducedIds.insert(std::make_pair(id, reducedTo));
    }
  }

  if (!_memoryChanged) {
    _memoryChanged = true;
  }

  return reducedIds;
}

void Memory::addNodeToStm(Node* node, const Mat66_t<double>& covariance, const std::pair<Link, Link>* neighbor) {
  tools::Timer timer;
  // add node on top of the short-term memory
  if (node) {
    EXT_LOG_DEBUG("adding {:d} (pose={})", node->id(), node->getPose().debugString().c_str());
    // Update neighbors
    if (_lastNode) {
      if (session_has_observations_) {
        graph_.connect_odometry(*node, *_lastNode, covariance, neighbor);
        if (!_stMem.empty()) EXT_LOG_DEBUG("Min STM id = {:d}", *_stMem.begin());
      } else {
        EXT_LOG_DEBUG("Ignoring neighbor link between {:d} and {:d} because they are not in the same map! ({:d} vs {:d})", _lastNode->id(),
                      node->id(), _lastNode->mapId(), node->mapId());

        if (topograph_params_.automatic_map_labels) {
          // Tag the first node of the map
          std::string tag = fmt::format("map{:d}", node->mapId());
          if (getNodeIdByLabel(tag, false) == 0) {
            EXT_LOG_DEBUG("Tagging node {:d} with label \"{}\"", node->id(), tag.c_str());
            node->setLabel(tag);
            _labels.insert(std::make_pair(node->id(), tag));
          }
        }
      }
    } else if (topograph_params_.automatic_map_labels) {
      // Tag the first node of the map
      std::string tag = fmt::format("map{:d}", node->mapId());
      if (getNodeIdByLabel(tag, false) == 0) {
        EXT_LOG_DEBUG("Tagging node {:d} with label \"{}\"", node->id(), tag.c_str());
        node->setLabel(tag);
        _labels.insert(std::make_pair(node->id(), tag));
      }
    }

    graph_.admit(node);
    _stMem.insert(_stMem.end(), node->id());
    if (node->getWeight() == -1) {
      ++_stMemIntermediateNodesCount;
    }
    ++_nodesAdded;

    EXT_LOG_DEBUG("{:d} words ref for the node {:d} (weight={:d})", (int)node->feature_count(), node->id(), node->getWeight());
    if (node->feature_count()) {
      index_retrieval_summary(*node);
      node->setEnabled(true);
    }
  }

  EXT_LOG_DEBUG("time = {:.6f}s", timer.lap_seconds());
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

void Memory::moveNodeToWMFromSTM(int id, int* reducedToOut) {
  EXT_LOG_DEBUG("Inserting node {:d} from STM in WM...", id);
  EXT_ASSERT(_stMem.find(id) != _stMem.end());
  int reducedId = 0;
  if (graph_.isGraphReduced()) {
    Node* s = this->_getNode(id);
    EXT_ASSERT(s != 0);
    if (s->getWeight() == -1) {
      EXT_LOG_ERROR("Graph reduction with intermediate nodes is not supported.");
    } else {
      // Setting keepLinkedInDb=true to make sure we save all visual
      // words that could be referenced in a previously
      // transferred node in LTM (#979)
      reducedId = reduceNode(s->id(), 0, /*keepLinkedInDb*/ true);
      if (reducedToOut) {
        *reducedToOut = reducedId;
      }
    }
  }
  if (reducedId == 0) {
    _workingMem.insert(_workingMem.end(), std::make_pair(id, tools::steady_time_seconds()));
    if (this->_getNode(id)->getWeight() == -1) {
      ++_workingMemIntermediateNodesCount;
      --_stMemIntermediateNodesCount;
    }
    _stMem.erase(id);
  }
  // else already removed from STM/WM in reduceNode()
}

const Node* Memory::getNode(int id) const { return _getNode(id); }

Node* Memory::_getNode(int id) const { return sapphire::backend::tools::value_or(graph_.nodes(), id, (Node*)0); }

const FbowIndex* Memory::fbow_index() const { return _vwd.get(); }

size_t Memory::getWorkingMemSize(bool ignoreIntermediateNodes) const {
  // The virtual place is absent before init() and after close(). Do not
  // underflow size_t when reporting an empty working memory.
  const size_t nodes = _workingMem.size() - _workingMem.count(kIdVirtual);
  return ignoreIntermediateNodes ? nodes - std::min(nodes, size_t(_workingMemIntermediateNodesCount)) : nodes;
}

std::multimap<int, Link> Memory::getNeighborLinks(int nodeId, bool lookInDatabase) const {
  std::multimap<int, Link> links;
  Node* s = sapphire::backend::tools::value_or(graph_.nodes(), nodeId, (Node*)0);
  if (s) {
    const std::multimap<int, Link>& allLinks = s->getLinks();
    for (std::multimap<int, Link>::const_iterator iter = allLinks.begin(); iter != allLinks.end(); ++iter) {
      if (iter->second.type() == Link::kOdometry) {
        links.insert(*iter);
      }
    }
  } else if (lookInDatabase && _dbDriver) {
    _dbDriver->loadLinks(nodeId, links);
    for (std::multimap<int, Link>::iterator iter = links.begin(); iter != links.end();) {
      if (iter->second.type() != Link::kOdometry) {
        links.erase(iter++);
      } else {
        ++iter;
      }
    }
  } else {
    EXT_LOG_WARN("Cannot find node {:d} in memory", nodeId);
  }
  return links;
}

std::multimap<int, Link> Memory::getLoopClosureLinks(int nodeId, bool lookInDatabase) const {
  const Node* s = this->getNode(nodeId);
  std::multimap<int, Link> loopClosures;
  if (s) {
    const std::multimap<int, Link>& allLinks = s->getLinks();
    for (std::multimap<int, Link>::const_iterator iter = allLinks.begin(); iter != allLinks.end(); ++iter) {
      if (iter->second.isLoopClosure() && iter->second.from() != iter->second.to()) {
        loopClosures.insert(*iter);
      }
    }
  } else if (lookInDatabase && _dbDriver) {
    _dbDriver->loadLinks(nodeId, loopClosures, Link::kLoopClosure);
  }
  return loopClosures;
}

std::multimap<int, Link> Memory::getLinks(int nodeId, bool lookInDatabase) const {
  std::multimap<int, Link> links;
  if (nodeId > 0) {
    Node* s = sapphire::backend::tools::value_or(graph_.nodes(), nodeId, (Node*)0);
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

// return map<Id,Margin>, including nodeId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database

// return map<Id,sqrdDistance>, including nodeId

int Memory::getNextId() { return ++_idCount; }

int Memory::incrementMapId(std::map<int, int>* reducedIds) {
  // don't increment if there is no location in the current map
  if (session_has_observations_) {
    // New session! move all nodes from the STM to WM
    while (_stMem.size()) {
      int reducedId = 0;
      int id = *_stMem.begin();
      moveNodeToWMFromSTM(id, &reducedId);
      if (reducedIds && reducedId > 0) {
        reducedIds->insert(std::make_pair(id, reducedId));
      }
    }

    session_has_observations_ = false;
    graph_.begin_odometry({});
    latest_odom_pose_ = {};
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

int Memory::getDatabaseMemoryUsed() const {
  int memoryUsed = 0;
  if (_dbDriver) {
    memoryUsed = _dbDriver->getMemoryUsed() / (1024 * 1024);  // Byte to MB
  }

  return memoryUsed;
}

std::string Memory::getDatabaseVersion() const {
  std::string version = "0.0.0";
  if (_dbDriver) {
    version = _dbDriver->getDatabaseVersion();
  }
  return version;
}

std::string Memory::getDatabaseUrl() const {
  std::string url = "";
  if (_dbDriver) {
    url = _dbDriver->getUrl();
  }
  return url;
}

double Memory::getDbSavingTime() const { return _dbDriver ? _dbDriver->getEmptyTrashesTime() : 0; }

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
  graph_.reset_belief();
  submap_prefetch_.reset();
  EXT_LOG_DEBUG("");

  // empty the STM
  while (_stMem.size()) {
    moveNodeToWMFromSTM(*_stMem.begin());
  }
  if (_stMem.size() != 0) {
    EXT_LOG_ERROR("_stMem must be empty here, size={:d}", (int)_stMem.size());
  }
  _stMem.clear();
  _stMemIntermediateNodesCount = 0;

  if (_dbDriver) {
    // Resolve unsaved rejected nodes before taking the final resident set: their
    // removal can change neighbors. Historical nodes remain in the database.
    std::vector<int> rejected;
    for (const auto& [id, node] : graph_.nodes())
      if (id > 0 && !node->isSaved() && node->isBadNode() && node_params_.ignore_bad_frames && node->getWeight() != -1) rejected.push_back(id);
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
    info.dictionarySize = static_cast<int>(_vwd ? _vwd->getIndexedWordsCount() : 0);
    info.parameters = parameterRecord();
    // Stop already owns these nodes. Commit directly under the database barrier,
    // without copying the entire working set into another background queue.
    _dbDriver->saveWorkingMemory(residents, sensor_records_, info);
  }

  // Get the tree root (parents)
  if (!_dbDriver) {
    // We are not saving to database anyway, just delete now.
    graph_.clear_residents();
    _workingMem.clear();

    sensor_records_.clear();
    recent_feature_nodes_.clear();

  } else {
    std::map<int, Node*> mem = graph_.nodes();
    for (std::map<int, Node*>::iterator i = mem.begin(); i != mem.end(); ++i) {
      if (i->second) {
        // EXT_LOG_DEBUG("deleting from the working and the short-term memory: %d", i->first);
        this->moveToTrash(i->second, true, nullptr, true);
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
  sensor_records_.clear();
  recent_feature_nodes_.clear();

  EXT_LOG_DEBUG("");
  // Wait until the db trash has finished cleaning the memory
  if (_dbDriver) {
    _dbDriver->emptyTrashes();
  }
  EXT_LOG_DEBUG("");
  _lastNode = 0;
  graph_.begin_odometry({});
  latest_odom_pose_ = {};
  session_has_observations_ = false;

  _nodesAdded = 0;
  _idCount = kIdStart;
  _idMapCount = kIdStart;
  _memoryChanged = false;

  _labels.clear();

  _allNodesInWM = true;

  if (_dbDriver) {
    _dbDriver->emptyTrashes();
  }
  if (_vwd) _vwd->clear();
  retrieval_mass_.clear();
  global_references_.clear();
  graph_.reset_storage();
  EXT_LOG_DEBUG("");
}

/**
 * Compute the likelihood of the node with some others in the memory.
 * Important: Assuming that all other ids are under 'node' id.
 * If an error occurs, the result is empty.
 */
void Memory::index_retrieval_summary(const Node& node) {
  if (graph_.members().count(node.id()) || node.id() <= 0 || !node.feature_count() || node.getWeight() < 0) return;
  if (scene_faiss_) {
    retrieval_mass_[node.id()] = node.feature_count();
    return;
  }
  if (!node.retrieval_mass()) return;
  if (node.vocabulary_hash() != _vwd->getFBoWVocabularyHash()) throw std::runtime_error("Global retrieval vocabulary mismatch");
  for (const auto& word : node.retrieval_words())
    if (word.word_id > 0) {
      _vwd->removeAllWordRef(word.word_id, node.id());
      _vwd->add_word_references(word.word_id, node.id(), word.count);
    }
  retrieval_mass_[node.id()] = node.retrieval_mass();
}
std::vector<int> Memory::global_candidates(const Node& query, size_t limit, std::vector<float>* scores, bool cold_only) const {
  if (scores) scores->clear();
  if (!scene_faiss_) return {};
  // Includes weak-texture references without ORB postings. Resident updates
  // replace immutable capture descriptors after copy/fusion; cold rows stay.
  for (const auto& [id, node] : graph_.nodes()) {
    if (id <= 0 || node->getWeight() < 0 || graph_.is_submap_member(id))
      global_references_.erase(id);
    else
      global_references_.set(id, node->featmap().global_descriptors());
  }
  return global_references_.search(
      query.featmap().global_descriptors(), limit,
      [&](int id) {
        return id != query.id() && !_stMem.count(id) && !graph_.is_submap_member(id) && !query.hasLink(id) && (!cold_only || !getNode(id));
      },
      scores);
}
std::vector<float> Memory::global_scores(const Node& query, const std::vector<int>& ids) const {
  global_candidates(query, 0);  // Reconcile canonical references after loading/fusion.
  auto scores = global_references_.scores(query.featmap().global_descriptors(), ids);
  for (size_t i = 0; i < ids.size(); ++i)
    if (ids[i] <= 0 || ids[i] == query.id() || _stMem.count(ids[i]) || graph_.is_submap_member(ids[i]) || query.hasLink(ids[i])) scores[i] = 0;
  return scores;
}
// Weights of the nodes in the working memory <node id, weight>
std::map<int, int> Memory::getWeights() const {
  std::map<int, int> weights;
  for (std::map<int, double>::const_iterator iter = _workingMem.begin(); iter != _workingMem.end(); ++iter) {
    if (iter->first > 0) {
      const Node* s = this->getNode(iter->first);
      if (!s) {
        EXT_LOG_FATAL("Location {:d} must exist in memory", iter->first);
      }
      weights.insert(weights.end(), std::make_pair(iter->first, s->getWeight()));
    } else {
      weights.insert(weights.end(), std::make_pair(iter->first, -1));
    }
  }
  return weights;
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
    // A clean, restored submap has no update for the writer. Do not generalize
    // this to raw feature/sensor records: their persistence rules differ.
    // isSaved() alone is NOT a durability acknowledgement. The drain below
    // still waits for any previously accepted insert/update before release.
    const bool clean_submap = source->isSaved() && source->has_map_features() && !source->isModified() && !source->feature_modified() &&
                              !source->features_erased() && !sensor_records_.count(id);
    if (clean_submap) continue;
    if (!queue_node(*source)) throw std::runtime_error("WM transfer rejected by database worker");
  }
  _dbDriver->emptyTrashes();
  if (_dbDriver->getWorkerMetrics().failed) throw std::runtime_error("WM transfer persistence failed");
  std::list<int> transferred;
  for (int id : ids) {
    moveToTrash(_getNode(id), true, nullptr, true);
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
    if (id > 0) bytes += graph_.nodes().at(id)->getMemoryUsed() + (sensor_records_.count(id) ? sensor_records_.at(id).memoryUsed() : 0);
  return bytes;
}
bool Memory::working_memory_over_budget(unsigned int node_limit) const {
  return (node_limit && getWorkingMemSize(true) > node_limit) ||
         (memory_params_.resident_budget_bytes && working_memory_bytes() > memory_params_.resident_budget_bytes);
}
std::list<int> Memory::trim_working_memory(unsigned int node_limit, const std::set<int>& protected_ids) {
  if (!_dbDriver || !working_memory_over_budget(node_limit)) return {};
  size_t nodes = getWorkingMemSize(true), bytes = working_memory_bytes();
  // A low-water target avoids repeated load/evict oscillation at the limit.
  const size_t target_nodes = node_limit ? std::max(size_t(1), size_t(node_limit) * 9 / 10) : SIZE_MAX;
  const size_t target_bytes = memory_params_.resident_budget_bytes ? size_t(memory_params_.resident_budget_bytes) * 9 / 10 : SIZE_MAX;
  const auto candidates = getRemovableNodes(std::numeric_limits<int>::max(), protected_ids, true);
  std::vector<int> plan;
  for (const auto* node : candidates) {
    if (nodes <= target_nodes && bytes <= target_bytes) break;
    plan.push_back(node->id());
    bytes -= std::min(bytes, size_t(node->getMemoryUsed() + (sensor_records_.count(node->id()) ? sensor_records_.at(node->id()).memoryUsed() : 0)));
    if (node->getWeight() >= 0) --nodes;
  }
  auto result = transfer_to_ltm(plan);
  if ((node_limit && nodes > node_limit) || (memory_params_.resident_budget_bytes && bytes > memory_params_.resident_budget_bytes))
    EXT_LOG_WARN("WM budget constrained by protected nodes: nodes={}, bytes={}, transferred={}", nodes, bytes, result.size());
  return result;
}

int Memory::cleanup() {
  EXT_LOG_DEBUG("");
  int nodeRemoved = 0;

  // bad node
  if (_lastNode && _lastNode->isBadNode() && node_params_.ignore_bad_frames && _lastNode->getWeight() != -1) {
    if (_lastNode->isBadNode()) {
      EXT_LOG_DEBUG("Bad node! {:d}", _lastNode->id());
    }
    nodeRemoved = _lastNode->id();
    moveToTrash(_lastNode);
  }

  return nodeRemoved;
}

void Memory::saveStatistics(const Statistics& statistics, bool saveWmState) {
  if (_dbDriver) {
    save_keyframe(0, {}, {}, &statistics, saveWmState);
  }
}

uint64_t Memory::save_keyframe(int node_id, const std::vector<OdomObservation>& trajectory, const std::map<int, cv::Mat>& localmaps,
                               const Statistics* statistics, bool save_wm) {
  if (!_dbDriver) return 0;
  DBDriver::WriteBatch batch;
  batch.epoch = _dbDriver->write_epoch();
  batch.trajectory = trajectory;
  batch.localmaps = localmaps;
  std::set<int> anchors;
  if (node_id > 0) anchors.insert(node_id);
  for (const auto& row : trajectory) anchors.insert(row.anchor_id);
  for (const auto& archive : localmaps) anchors.insert(archive.first);
  for (int id : anchors) {
    if (const auto* node = _getNode(id); node && !node->isSaved()) {
      batch.nodes.push_back(std::make_shared<Node>(*node));
      if (const auto record = sensor_records_.find(id); record != sensor_records_.end()) batch.records.emplace(id, record->second);
    }
  }
  if (statistics) {
    // Persisted statistics contain scalars and IDs, not the live graph/images.
    batch.statistics.emplace();
    batch.statistics->setRefImageId(statistics->refImageId());
    batch.statistics->setStamp(statistics->stamp());
    for (const auto& value : statistics->data()) batch.statistics->addStatistic(value.first, value.second);
    if (save_wm) batch.statistics->setWmState(statistics->wmState());
    batch.saveWmState = save_wm;
  }
  const auto sequence = _dbDriver->asyncWrite(std::move(batch));
  if (!sequence) throw std::runtime_error("Cannot submit keyframe persistence: " + _dbDriver->getWorkerError());
  for (int id : anchors) {
    if (auto* node = _getNode(id); node && !node->isSaved()) {
      node->setSaved(true);  // Snapshot is now owned and visible through the writer.
      node->mark_features_saved();
      if (auto record = sensor_records_.find(id); record != sensor_records_.end()) record->second.clearCompressedData();
    }
  }
  if (node_id > 0) saveLocationData(node_id);  // Apply the existing feature-cache residency policy.
  return sequence;
}

void Memory::savePreviewImage(const cv::Mat& image) const {
  if (_dbDriver) {
    _dbDriver->savePreviewImage(image);
  }
}
cv::Mat Memory::loadPreviewImage() const {
  if (_dbDriver) {
    return _dbDriver->loadPreviewImage();
  }
  return cv::Mat();
}

const RootFrame& Memory::create_root_frame(int anchor, const StateGroupf& pose) {
  if (!graph_.persistent_root_frames() || !_dbDriver) throw std::logic_error("Persistent map frames are disabled");
  if (graph_.root_frames().count(anchor)) return graph_.root_frames().at(anchor);
  saveLocationData(anchor);
  const auto frame = _dbDriver->create_root_frame(anchor, pose);
  return graph_.restore_frame(frame);
}

void Memory::saveOptimizedPoses(const std::map<int, StateGroupf>& optimizedPoses, const StateGroupf& lastPose) const {
  if (_dbDriver) {
    _dbDriver->saveOptimizedPoses(optimizedPoses, lastPose);
  }
}

std::map<int, StateGroupf> Memory::loadOptimizedPoses(StateGroupf* lastPose) const {
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
  return std::map<int, StateGroupf>();
}

void Memory::save_trajectory(const std::vector<OdomObservation>& observations) const {
  if (!_dbDriver) return;
  for (const auto& row : observations) {
    if (auto* node = _getNode(row.anchor_id); node && !node->isSaved()) {
      if (!queue_node(*node)) throw std::runtime_error("Cannot queue trajectory anchor");
      node->setSaved(true);
    }
  }
  _dbDriver->save_trajectory(observations);
}
std::vector<OdomObservation> Memory::load_recent_trajectory(int limit) const {
  return _dbDriver ? _dbDriver->load_recent_trajectory(limit) : std::vector<OdomObservation>{};
}
int Memory::trajectory_high_water() const { return _dbDriver ? _dbDriver->trajectory_high_water() : 0; }

void Memory::save_localmap(int anchor_id, const cv::Mat& archive) const {
  if (!_dbDriver) return;
  // In-memory databases normally defer nodes until close; an archive needs its anchor now.
  if (_dbDriver->isInMemory()) {
    if (auto* node = _getNode(anchor_id); node && !node->isSaved()) {
      if (!queue_node(*node)) throw std::runtime_error("Cannot queue localmap anchor");
      node->setSaved(true);
    }
  }
  _dbDriver->save_localmap(anchor_id, archive);
}
void Memory::attach_navimap(std::shared_ptr<const LocalMap> map) const {
  if (!map) return;
  if (auto* node = _getNode(map->anchor_id)) node->set_navimap(std::move(map));
}

std::map<int, cv::Mat> Memory::load_localmaps() const { return _dbDriver ? _dbDriver->load_localmaps() : std::map<int, cv::Mat>{}; }

void Memory::save2DMap(const cv::Mat& map, float xMin, float yMin, float cellSize) const {
  if (_dbDriver) {
    _dbDriver->save2DMap(map, xMin, yMin, cellSize);
  }
}

cv::Mat Memory::load2DMap(float& xMin, float& yMin, float& cellSize) const {
  if (_dbDriver) {
    return _dbDriver->load2DMap(xMin, yMin, cellSize);
  }
  return cv::Mat();
}

void Memory::emptyTrash() {
  if (_dbDriver) {
    _dbDriver->emptyTrashes(true);
  }
}

void Memory::checkpoint_nodes() {
  if (!_dbDriver) return;
  std::vector<Node*> dirty;
  for (const auto& [id, node] : graph_.nodes())
    if (id > 0 && (!node->isSaved() || node->isModified() || node->feature_modified())) {
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
      const size_t budget = std::min(visits.size(), size_t(std::max(0, int(memory_params_.protected_recent_ratio * workingMemSize))));
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
            weightAgeIdMap.insert(std::make_pair(
                WeightAgeIdKey(budget_pressure ? 0 : s->getWeight(),
                               !budget_pressure && memory_params_.eviction_order_by_id && !s->has_map_features() ? 0.0 : memIter->second, s->id()),
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

/**
 * If saveToDatabase=false, deleted words are filled in deletedWords.
 */
void Memory::moveToTrash(Node* s, bool keepLinkedToGraph, std::list<int>* deletedWords, bool committed) {
  // EXT_LOG_DEBUG("id=%d", s?s->id():0);
  if (s) {
    if (graph_.root_frames().count(s->id()) && !keepLinkedToGraph) throw std::runtime_error("Cannot delete a component datum");
    // Keep the WM/STM intermediate-node counters in sync now, before the weight
    // may be set to -9 below.
    if (s->getWeight() == -1) {
      if (this->isInWM(s->id())) {
        --_workingMemIntermediateNodesCount;
      } else if (this->isInSTM(s->id())) {
        --_stMemIntermediateNodesCount;
      }
    }

    // it is a bad node (not saved), remove links!
    if (keepLinkedToGraph && !s->isSaved() && s->isBadNode() && node_params_.ignore_bad_frames && s->getWeight() != -1) {
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

    if (!keepLinkedToGraph || !_dbDriver) this->disableWordsRef(s->id());
    if (!keepLinkedToGraph) {
      global_references_.erase(s->id());
      if (deletedWords) {
        for (const auto& word : s->retrieval_words()) {
          const int wordId = word.word_id;
          if (_vwd && wordId > 0 && !_vwd->contains(wordId)) {
            deletedWords->push_back(wordId);
          }
        }
      }
    }

    _workingMem.erase(s->id());
    _stMem.erase(s->id());
    if (feature_params_.uses_xfeat() && keepLinkedToGraph && !graph_.is_submap_member(s->id()))
      global_references_.set(s->id(), s->featmap().global_descriptors());
    else
      global_references_.erase(s->id());
    graph_.release(s->id());
    auto sensor_entry = sensor_records_.extract(s->id());
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

    if ((archive_params_.keep_unlinked_nodes || keepLinkedToGraph || s->isSaved()) && _dbDriver && s->id() > 0) {
      if (keepLinkedToGraph) {
        _allNodesInWM = false;
      }
      if (committed)
        delete s;
      else
        _dbDriver->asyncSave(s, sensor_entry.empty() ? std::nullopt : std::optional<SensorRecord>(std::move(sensor_entry.mapped())));
    } else {
      delete s;
    }
  }
}

int Memory::getLastNodeId() const { return _lastNode ? _lastNode->id() : _idCount; }
StateGroupf Memory::current_odom_pose() const { return latest_odom_pose_; }

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

int Memory::getNodeIdByLabel(const std::string& label, bool lookInDatabase) const {
  EXT_LOG_DEBUG("label={}", label.c_str());
  int id = 0;
  if (label.size()) {
    for (std::map<int, Node*>::const_iterator iter = graph_.nodes().begin(); iter != graph_.nodes().end(); ++iter) {
      EXT_ASSERT(iter->second != 0);
      if (iter->second->getLabel().compare(label) == 0) {
        id = iter->second->id();
        break;
      }
    }
    if (id == 0 && _dbDriver && lookInDatabase) {
      _dbDriver->getNodeIdByLabel(label, id);
      if (graph_.nodes().find(id) != graph_.nodes().end()) {
        // The node is already in WM, but label was not
        // found above. It means the label has been cleared in
        // current session (not yet saved to database), so return
        // not found.
        id = 0;
      }
    }
  }
  return id;
}

void Memory::convertToIntermediate(int locationId) {
  if (graph_.root_frames().count(locationId)) return;
  // Intermediate nodes are skipped by the Bayes neighborhood traversal.
  EXT_LOG_DEBUG("Converting location {:d} to intermediate node", locationId);
  Node* location = _getNode(locationId);
  if (location) {
    // Keep the WM/STM intermediate-node counters in sync if the node is
    // converted while already resident in memory.
    if (location->getWeight() >= 0) {
      if (this->isInWM(locationId)) {
        ++_workingMemIntermediateNodesCount;
      } else if (this->isInSTM(locationId)) {
        ++_stMemIntermediateNodesCount;
      }
    }
    graph_.make_intermediate(*location);
    location->clear_reference_features();
    this->disableWordsRef(locationId);  // won't be used for loop closure detection anymore
    if (!archive_params_.keep_intermediate_data) {
      location->removeAllWords();
      location->set_global_descriptors({});
    }

    if (auto found = sensor_records_.find(locationId); found != sensor_records_.end()) {
      found->second.observation.clearRawData();
      if (!archive_params_.keep_intermediate_data || !this->isBinDataKept()) found->second.clearCompressedData();
    }
  }
}

void Memory::deleteLocation(int locationId, std::list<int>* deletedWords, bool keepLinkedInDb) {
  EXT_LOG_DEBUG("Deleting location {:d} (keepLinkedInDb={})", locationId, keepLinkedInDb ? "true" : "false");
  Node* location = _getNode(locationId);
  if (location) {
    this->moveToTrash(location, keepLinkedInDb, deletedWords);
    _memoryChanged = true;
  } else {
    EXT_LOG_WARN("Location {:d} has not been found in STM/WM, cannot delete it.", locationId);
  }
}

void Memory::saveLocationData(int location_id) {
  auto* node = _getNode(location_id);
  if (!node || !_dbDriver || location_id <= 0) return;
  if (!node->isSaved()) {
    if (!queue_node(*node)) throw std::runtime_error("Cannot queue node persistence");
    node->setSaved(true);
    node->mark_features_saved();
    sensor_records_[location_id].clearCompressedData();
  }
  if (node->loop_features() || node->map_features()) {
    recent_feature_nodes_.erase(std::remove(recent_feature_nodes_.begin(), recent_feature_nodes_.end(), location_id), recent_feature_nodes_.end());
    recent_feature_nodes_.push_back(location_id);
  }
  while (recent_feature_nodes_.size() > static_cast<size_t>(memory_params_.feature_cache_nodes)) {
    const int expired_id = recent_feature_nodes_.front();
    recent_feature_nodes_.pop_front();
    auto* expired = _getNode(expired_id);
    if (expired && expired->isSaved()) {
      expired->clear_reference_features();
      expired->clear_feature_cache();
    }
  }
}

void Memory::removeRawData(int id, bool image, bool userData, bool occupancyGrid) {
  EXT_LOG_DEBUG("id={:d} image={:d} userData={:d}", id, image ? 1 : 0, userData ? 1 : 0);
  Node* s = this->_getNode(id);
  if (s) {
    if (auto found = sensor_records_.find(id); found != sensor_records_.end()) found->second.observation.clearRawData(image, userData, occupancyGrid);
  }
}

// compute transform fromId -> toId
cv::Mat Memory::load_observation_archive(int id) const { return _dbDriver ? _dbDriver->load_submap_bundle(id) : cv::Mat{}; }

std::shared_ptr<const bow::FeatureBlock> Memory::frame_features_for(const Node& snapshot) const {
  auto features = snapshot.loop_features();
  if (!features && _dbDriver) features = _dbDriver->get_loop_features(snapshot.id());
  return features;
}
void Memory::load_calibration(int id, std::vector<camera::Perspective>& cameras) const {
  cameras.clear();
  if (_dbDriver) _dbDriver->getCalibration(id, cameras);
}
std::shared_ptr<const featmap::LandmarkMap> Memory::map_features_for(int id, const Node* snapshot) const {
  tools::Timer timer;
  const auto* node = snapshot ? snapshot : getNode(id);
  auto map = node ? node->map_features() : nullptr;
  const char* source = map ? "resident" : "cold";
  const featmap::PrefetchCache::Key key{id, node ? node->feature_token() : 0};
  if (!map && submap_prefetch_ && key.second) {
    bool prefetched = false;
    map = submap_prefetch_->get(key, &prefetched);
    if (map) source = prefetched ? "predicted" : "reused";
  }
  if (!map && _dbDriver) {
    map = _dbDriver->get_map_features(id);
    if (map && submap_prefetch_ && key.second) submap_prefetch_->remember(key, map);
  }
  EXT_LOG_DEBUG("[feature_read] node={} source={} ms={:.3f}", id, source, timer.elapsed_seconds() * 1000);
  return map;
}

void Memory::prefetch_map_features(const std::vector<int>& ranked) {
  if (!memory_params_.predictive_prefetch_enabled || !_dbDriver || !submap_prefetch_) return;
  std::vector<featmap::PrefetchCache::Key> plan;
  for (int id : ranked) {
    const auto* node = getNode(id);
    if (node && node->has_map_features() && !node->map_features() && node->isSaved()) plan.emplace_back(id, node->feature_token());
  }
  submap_prefetch_->plan(plan);
  const auto stats = submap_prefetch_->stats();
  EXT_LOG_DEBUG("[prefetch] planned={} loaded={} used={} unused={} discarded={} errors={} entries={} bytes={} load_ms={:.3f}", plan.size(),
                stats.loaded, stats.used, stats.unused, stats.discarded, stats.errors, stats.entries, stats.bytes, stats.load_ms);
}

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
void Memory::updateLink(const Link& link, bool updateInDatabase) {
  if (getNode(link.from()) && getNode(link.to())) {
    graph_.updateLink(link);
    return;
  }
  if (!updateInDatabase || !_dbDriver) return;
  auto drafts = graph_.prepare_cold_link(link, true);
  _dbDriver->write_link_pair(link, !getNode(link.from()), !getNode(link.to()), true);
  graph_.commit_cold_link(drafts);
  // The DB fence rejects pre-edit archive snapshots. Refresh the live owners,
  // exactly as for content/contraction commits; cold drafts remain on disk.
  for (const auto& [id, node] : graph_.nodes()) node->write_epoch = _dbDriver->write_epoch();
}

unsigned long Memory::getMemoryUsed() const {
  unsigned long memoryUsage = sizeof(Memory);
  if (submap_prefetch_) memoryUsage += submap_prefetch_->stats().bytes;
  memoryUsage += global_references_.memory_size();
  memoryUsage += graph_.nodes().size() * (sizeof(int) + sizeof(std::map<int, Node*>::iterator)) + sizeof(std::map<int, Node*>);
  for (std::map<int, Node*>::const_iterator iter = graph_.nodes().begin(); iter != graph_.nodes().end(); ++iter) {
    memoryUsage += iter->second->getMemoryUsed();
  }
  for (const auto& entry : sensor_records_) memoryUsage += entry.second.memoryUsed();

  if (_vwd) memoryUsage += _vwd->getMemoryUsed();
  memoryUsage += retrieval_mass_.size() * (sizeof(std::pair<const int, uint32_t>) + 4 * sizeof(void*));
  memoryUsage += _stMem.size() * (sizeof(int) + sizeof(std::set<int>::iterator)) + sizeof(std::set<int>);
  memoryUsage += _workingMem.size() * (sizeof(int) + sizeof(double) + sizeof(std::map<int, double>::iterator)) + sizeof(std::map<int, double>);
  memoryUsage +=
      _labels.size() * (sizeof(int) + sizeof(std::string) + sizeof(std::map<int, std::string>::iterator)) + sizeof(std::map<int, std::string>);
  for (std::map<int, std::string>::const_iterator iter = _labels.begin(); iter != _labels.end(); ++iter) {
    memoryUsage += iter->second.size();
  }
  memoryUsage += parameters_.size() * (sizeof(std::string) * 2 + sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);
  memoryUsage += sizeof(DBDriver);

  return memoryUsage;
}

bool Memory::apply_rehearsal(int oldId, int newId, bool fullMerge, bool intermediateMerge) {
  if (graph_.root_frames().count(node_params_.rehearsal_use_new_id ? oldId : newId)) return false;
  EXT_LOG_DEBUG("old={:d}, new={:d}", oldId, newId);
  Node* oldS = _getNode(oldId);
  Node* newS = _getNode(newId);
  if (oldS && newS) {
    EXT_ASSERT_MSG(oldS->getWeight() >= 0 && newS->getWeight() >= 0, "{:d} {:d}", oldS->getWeight(), newS->getWeight());
    std::map<int, Link>::const_iterator iter = oldS->getLinks().find(newS->id());
    if (iter != oldS->getLinks().end() && iter->second.type() != Link::kOdometry && iter->second.from() != iter->second.to()) {
      // do nothing, already merged
      EXT_LOG_WARN("already merged, old={:d}, new={:d}", oldId, newId);
      return false;
    }
    EXT_ASSERT(!newS->isSaved());

    EXT_LOG_DEBUG("Rehearsal merging {:d} (w={:d}) and {:d} (w={:d})", oldS->id(), oldS->getWeight(), newS->id(), newS->getWeight());

    EXT_LOG_DEBUG("fullMerge={} intermediateMerge={} node_params_.rehearsal_use_new_id={}", fullMerge ? "true" : "false",
                  intermediateMerge ? "true" : "false", node_params_.rehearsal_use_new_id ? "true" : "false");

    if (fullMerge) {
      graph_.rehearse_edges(*oldS, *newS, node_params_.rehearsal_use_new_id);
      if (node_params_.rehearsal_use_new_id) {
        // Set old image to new node
        this->copyData(oldS, newS);

        // update weight
        newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

        graph_.remap_last_loop(oldS->id(), newS->id());
        oldS->setWeight(-9);
      } else {
        // update weight
        oldS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

        if (_lastNode == newS) {
          _lastNode = oldS;
        }
        newS->setWeight(-9);
      }
      EXT_LOG_DEBUG("New weights: {:d}->{:d} {:d}->{:d}", oldS->id(), oldS->getWeight(), newS->id(), oldS->getWeight());

      // remove location
      moveToTrash(node_params_.rehearsal_use_new_id ? oldS : newS, archive_params_.keep_unlinked_nodes);

      return true;
    } else {
      // update only weights
      if (node_params_.rehearsal_use_new_id) {
        // just update weight
        int w = oldS->getWeight() >= 0 ? oldS->getWeight() : 0;
        newS->setWeight(w + newS->getWeight() + 1);
        oldS->setWeight(0);

        graph_.remap_last_loop(oldS->id(), newS->id());
        if (intermediateMerge) {
          static bool warned = false;
          if (!warned) {
            EXT_LOG_WARN(
                "A rehearsal was accepted ({:d}->{:d}) while not moving but "
                "there are intermediate nodes between them in the graph. "
                "Because {}=true, the node {:d} cannot be converted "
                "into an intermediate node so it will be kept in the graph "
                "even if we are not moving. Set {}=false to handle intermediate "
                "nodes with rehearsal enabled so that loop closure hypotheses "
                "are propagated correctly. This message is only "
                "printed once.",
                oldS->id(), newS->id(), "node.rehearsal_use_new_id", oldS->id(), "node.rehearsal_use_new_id");
            warned = true;
          }
        }
      } else  // !node_params_.rehearsal_use_new_id
      {
        int w = newS->getWeight() >= 0 ? newS->getWeight() : 0;
        oldS->setWeight(w + oldS->getWeight() + 1);
        if (intermediateMerge) {
          this->convertToIntermediate(newS->id());
        } else {
          newS->setWeight(0);
        }
      }
    }
  } else {
    if (!newS) {
      EXT_LOG_ERROR("newId={:d}, oldId={:d}, Node {:d} not found in working/st memories", newId, oldId, newId);
    }
    if (!oldS) {
      EXT_LOG_ERROR("newId={:d}, oldId={:d}, Node {:d} not found in working/st memories", newId, oldId, oldId);
    }
  }
  return false;
}

int Memory::getMapId(int nodeId, bool lookInDatabase) const {
  // Scalar graph metadata is already resident. Loading complete node info
  // would unnecessarily fetch cold environmental sensors from the writer DB.
  if (const auto* node = getNode(nodeId)) return node->mapId();
  StateGroupf pose;
  int mapId = -1, weight;
  std::string label;
  double stamp;
  std::vector<float> velocity;
  EnvSensors sensors;
  getNodeInfo(nodeId, pose, mapId, weight, label, stamp, velocity, sensors, lookInDatabase);
  return mapId;
}

StateGroupf Memory::getOdomPose(int nodeId, bool lookInDatabase) const {
  if (const auto* node = getNode(nodeId)) return node->getPose();
  StateGroupf pose;
  int mapId, weight;
  std::string label;
  double stamp;
  std::vector<float> velocity;
  EnvSensors sensors;
  getNodeInfo(nodeId, pose, mapId, weight, label, stamp, velocity, sensors, lookInDatabase);
  return pose;
}

bool Memory::getNodeInfo(int nodeId, StateGroupf& odomPose, int& mapId, int& weight, std::string& label, double& stamp, std::vector<float>& velocity,
                         EnvSensors& sensors, bool lookInDatabase) const {
  const Node* s = this->getNode(nodeId);
  if (s) {
    odomPose = s->getPose();
    mapId = s->mapId();
    weight = s->getWeight();
    label = std::string(s->getLabel());
    stamp = s->getStamp();
    velocity = std::vector<float>(s->getVelocity());
    sensors = cached_sensor_record(nodeId).observation.envSensors();
    if (!sensor_records_.count(nodeId) && _dbDriver) {
      StateGroupf oldPose;
      int oldMap, oldWeight;
      std::string oldLabel;
      double oldStamp;
      std::vector<float> oldVelocity;
      _dbDriver->getNodeInfo(nodeId, oldPose, oldMap, oldWeight, oldLabel, oldStamp, oldVelocity, sensors);
    }
    return true;
  } else if (lookInDatabase && _dbDriver) {
    return _dbDriver->getNodeInfo(nodeId, odomPose, mapId, weight, label, stamp, velocity, sensors);
  }
  return false;
}

cv::Mat Memory::getImageCompressed(int nodeId) const {
  cv::Mat image;
  const Node* s = this->getNode(nodeId);
  if (s) {
    image = cached_sensor_record(nodeId).archive.image;
  }
  if (image.empty() && this->isBinDataKept() && _dbDriver) {
    SensorRecord data;
    _dbDriver->getNodeData(nodeId, data);
    image = data.archive.image;
  }
  return image;
}

bool Memory::queue_node(const Node& node) const {
  const auto record = sensor_records_.find(node.id());
  return _dbDriver->asyncSave(std::make_unique<Node>(node),
                              record == sensor_records_.end() ? std::nullopt : std::optional<SensorRecord>(record->second));
}

const SensorRecord& Memory::cached_sensor_record(int id) const {
  static const SensorRecord empty;
  auto found = sensor_records_.find(id);
  return found == sensor_records_.end() ? empty : found->second;
}

std::shared_ptr<const LocalGrid> Memory::local_grid(int nodeId) const {
  const auto& cached = cached_sensor_record(nodeId);
  if (cached.observation.localGrid()) return cached.observation.localGrid();
  auto archived = getNodeData(nodeId, false, false, true);
  return decodeLocalGrid(archived);
}

SensorRecord Memory::getNodeData(int locationId, bool images, bool userData, bool occupancyGrid) const {
  SensorRecord result;
  const auto* node = getNode(locationId);
  const auto found = sensor_records_.find(locationId);
  if (node && found != sensor_records_.end() &&
      (!node->isSaved() || ((!images || !found->second.archive.image.empty()) && (!userData || !found->second.archive.user.empty()) &&
                            (!occupancyGrid || found->second.gridCellSize() != 0.0f)))) {
    result = found->second;
    selectSensorData(result, images, userData, occupancyGrid);
  } else if (_dbDriver) {
    _dbDriver->getNodeData(locationId, result, images, userData, occupancyGrid);
  }
  return result;
}

void Memory::copyNodeFeatures(int nodeId, Node& output, bool localFeatures, bool globalDescriptors) const {
  const auto copy = [&](const Node& source) {
    output.set_lidar_cloud(source.lidar_cloud());
    if (localFeatures) {
      if (source.has_map_features()) {
        auto map = map_features_for(nodeId, &source);
        if (map) output.set_map_features(std::move(map));
      } else {
        auto features = source.loop_features();
        if (!features && _dbDriver) features = _dbDriver->get_loop_features(nodeId);
        if (features)
          output.set_loop_features(std::move(features));
        else
          output.setWords(source.getWords(), source.getWordsKpts(), source.getWords3(), source.getWordsDescriptors());
      }
    }
    if (globalDescriptors) output.set_global_descriptors(source.featmap().global_descriptors());
  };
  if (const auto* source = getNode(nodeId)) {
    copy(*source);
  } else if (_dbDriver) {
    std::list<Node*> loaded;
    std::set<int> fromTrash;
    _dbDriver->loadNodes({nodeId}, loaded, &fromTrash);
    for (auto* node : loaded) {
      std::unique_ptr<Node> source(node);
      copy(*source);
    }
  }
}

void Memory::getNodeCalibration(int nodeId, std::vector<camera::Perspective>& models) const {
  // EXT_LOG_DEBUG("nodeId=%d", nodeId);
  Node* s = this->_getNode(nodeId);
  if (s) {
    models = s->visualFrame().perspectives();
  } else if (_dbDriver) {
    // load from database
    _dbDriver->getCalibration(nodeId, models);
  }
}

void Memory::copyData(const Node* from, Node* to) {
  tools::Timer timer;
  timer.reset();
  if (from && to) {
    // words 2d
    this->disableWordsRef(to->id());
    to->removeAllWords();
    to->set_reference_frame(from->visualFrame());
    if (from->loop_features()) to->set_loop_features(from->loop_features());
    if (from->has_map_features()) {
      auto map = from->map_features();
      if (!map && _dbDriver) map = _dbDriver->get_map_features(from->id());
      to->set_map_features(std::move(map));
    } else if (!from->loop_features())
      to->setWords(from->getWords(), from->getWordsKpts(), from->getWords3(), from->getWordsDescriptors());
    std::list<int> id;
    id.push_back(to->id());
    this->enableWordsRef(id);

    if (from->isSaved() && _dbDriver) {
      _dbDriver->getNodeData(from->id(), sensor_records_[to->id()]);
      EXT_LOG_DEBUG("Loaded image data from database");
    } else {
      sensor_records_[to->id()] = cached_sensor_record(from->id());
    }
    sensor_records_[to->id()].observation.setId(to->id());

    if (!from->featmap().global_descriptors().empty()) {
      to->set_global_descriptors(from->featmap().global_descriptors());
    }

    to->setPose(from->getPose());
  } else {
    EXT_LOG_ERROR("Can't merge the nodes because there are not same type.");
  }
  EXT_LOG_DEBUG("Merging time = {:.6f}s", timer.lap_seconds());
}

void Memory::disableWordsRef(int nodeId) {
  // EXT_LOG_DEBUG("id=%d", nodeId);

  Node* ss = this->_getNode(nodeId);
  if (ss && ss->isEnabled()) {
    int count = _vwd ? _vwd->getTotalActiveReferences() : 0;
    if (_vwd)
      for (const auto& word : ss->retrieval_words()) _vwd->removeAllWordRef(word.word_id, nodeId);

    count -= _vwd ? _vwd->getTotalActiveReferences() : 0;
    ss->setEnabled(false);
    retrieval_mass_.erase(nodeId);
    // EXT_LOG_DEBUG("%d words total ref removed from node %d... (total active ref = %d)", count, ss->id(), _vwd->getTotalActiveReferences());
  }
}

void Memory::enableWordsRef(const std::list<int>& nodeIds) {
  EXT_LOG_DEBUG("size={:d}", (int)nodeIds.size());
  tools::Timer timer;
  timer.reset();

  std::list<Node*> surfSigns;
  for (std::list<int>::const_iterator i = nodeIds.begin(); i != nodeIds.end(); ++i) {
    Node* ss = dynamic_cast<Node*>(this->_getNode(*i));
    if (ss && !ss->isEnabled()) {
      surfSigns.push_back(ss);
    }
  }

  int count = _vwd ? _vwd->getTotalActiveReferences() : 0;

  // Reactivate references and nodes
  for (std::list<Node*>::iterator j = surfSigns.begin(); j != surfSigns.end(); ++j) {
    if ((*j)->feature_count()) {
      index_retrieval_summary(**j);
      (*j)->setEnabled(true);
    }
  }

  count = (_vwd ? _vwd->getTotalActiveReferences() : 0) - count;
  EXT_LOG_DEBUG("{:d} words total ref added from {:d} nodes, time={:.6f}s...", count, (int)surfSigns.size(), timer.lap_seconds());
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
    _dbDriver->loadNodes(batch, reactivatedSigns, nullptr, true);
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
      if ((*i)->loop_features()) saveLocationData((*i)->id());
    }
    enableWordsRef(idsLoaded);
  }
  timeDbAccess = timer.elapsed_seconds();
  return totalLoaded;
}

// returns all non-null poses and links
// if lookInDatabase is false, intermediate nodes are ignored and new neighbor links between non-intermediate nodes are returned
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void Memory::getMetricConstraints(const std::set<int>& ids, std::map<int, StateGroupf>& poses, std::multimap<int, Link>& links, bool lookInDatabase) {
  EXT_LOG_DEBUG("");
  for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter) {
    StateGroupf pose = getOdomPose(*iter, lookInDatabase);
    if (!pose.isNull()) {
      poses.insert(std::make_pair(*iter, pose));
    }
  }

  graph_.metric_constraints(ids, poses, links, lookInDatabase, keeps_depth_nodes(),
                            [this, lookInDatabase](int id, int&, std::multimap<int, Link>& result) { result = getLinks(id, lookInDatabase); });
}

Memory::FeatureUpdate Memory::prepare_feature_content(int retained, int source_id, std::unique_ptr<Node> replacement, bool consolidate,
                                                      const std::vector<Link>& added, const std::vector<std::pair<int, int>>& removed, int region) {
  auto* target = _getNode(retained);
  auto* source = _getNode(source_id);
  if (!target || !source || target == source) throw std::runtime_error("Missing submap publication owners");
  if (!replacement || replacement->id() != retained || !replacement->map_features() || !replacement->has_observation_history())
    throw std::runtime_error("Incomplete submap publication draft");
  replacement->set_reference_frame({});
  auto members = graph_.redirect_members(target->id(), source->id(), consolidate);
  const auto* region_node = region ? getNode(region) : nullptr;
  if (region && (!region_node || region == retained || region == source_id || graph_.is_submap_member(region)))
    throw std::runtime_error("Expired previous region owner");
  if (region) {
    for (auto& [capture, owner] : members)
      if (owner == region) owner = retained;
    members[region] = retained;
  }
  // Stage postings before persistence. Graph nodes,
  // factors, LocalMaps, trajectory anchors and the active odom origin stay put.
  std::optional<FbowIndex::ReferenceUpdate> references;
  if (_vwd)
    references.emplace(_vwd->prepare_replacement(retained, target->retrieval_words(), source->id(), source->retrieval_words(),
                                                 replacement->retrieval_words(), region,
                                                 region_node ? region_node->retrieval_words() : std::vector<bow::WordCount>{}));
  auto masses = retrieval_mass_;
  masses.erase(source->id());
  if (region) masses.erase(region);
  masses[target->id()] = uses_raw_features() ? replacement->feature_count() : replacement->retrieval_mass();
  FeatureUpdate update;
  update.retained = retained;
  update.source = source_id;
  update.region = region;
  update.topology = graph_.topology_revision();
  update.members = std::move(members);
  update.masses = std::move(masses);
  update.references = std::move(references);
  update.local_poses = graph_.local_poses();
  for (const auto& [capture, pose] : replacement->observation_history().observation_poses)
    if (capture != retained && update.members.count(capture) && update.members.at(capture) == retained) {
      auto old = update.local_poses.find(capture);
      if (old == update.local_poses.end() || old->second.coefficients() != pose.coefficients() || graph_.submap_owner(capture) != retained)
        update.changed_captures.insert(capture);
      update.local_poses.insert_or_assign(capture, pose);
    }
  // Prepare all link allocations against the new content, before the DB transaction.
  update.link_nodes.emplace(retained, *replacement);
  graph_.prepare_links(added, removed, update.link_nodes, update.last_loop);
  *replacement = std::move(update.link_nodes.at(retained));
  update.link_nodes.erase(retained);
  update.replacement = std::move(replacement);
  return update;
}
bool Memory::can_apply(const FeatureUpdate& update) const {
  return update.topology == graph_.topology_revision() && getNode(update.retained) && getNode(update.source) && update.replacement;
}
void Memory::apply_feature_content(FeatureUpdate&& update) noexcept {
  auto* target = _getNode(update.retained);
  auto* source = _getNode(update.source);
  auto& replacement = update.replacement;
  replacement->setSaved(_dbDriver != nullptr);
  if (_dbDriver) replacement->setModified(false);
  if (_lastNode == target) _lastNode = replacement.get();
  graph_.apply_links(update.link_nodes, update.last_loop);
  graph_.apply_feature_content(update.retained, replacement, update.members, &update.local_poses);
  graph_.record_pose_changes(update.changed_captures);
  graph_.commit_contraction(update.retained, update.source);
  if (update.region) {
    graph_.commit_contraction(update.retained, update.region);
    _getNode(update.region)->setEnabled(false);
  }
  source->setEnabled(false);

  retrieval_mass_.swap(update.masses);
  if (update.references) _vwd->apply_replacement(std::move(*update.references));
  for (auto& [id, node] : graph_.nodes()) node->write_epoch = _dbDriver ? _dbDriver->write_epoch() : 0;
}

Memory::NodeUpdate Memory::prepare_node_merge(int retained, int removed, const featmap::MergeDraft& content, std::shared_ptr<const LocalMap> local,
                                              const StateGroupf& transform, const std::multimap<int, Link>& constraints,
                                              const std::vector<Link>& added, const std::vector<std::pair<int, int>>& removed_links) {
  const auto trace_key = detail::SealTrace::enabled() ? std::to_string(removed) + "_" + std::to_string(retained) : std::string();
  detail::SealTrace trace("merge", trace_key);
  auto* target = _getNode(retained);
  auto* source = _getNode(removed);
  if (!target || !source || source != _lastNode || !retrieval_mass_.count(retained) || !retrieval_mass_.count(removed))
    throw std::runtime_error("Expired submap transaction");
  if (!content.features || !local || local->anchor_id != retained) throw std::runtime_error("Expired prepared submap content");
  // Prepare every incident neighbor, including cold neighbors. RAM pointers do
  // not change until persistence succeeds; cold drafts remain cold after commit.
  std::map<int, std::unique_ptr<Node>> nodes;
  nodes.emplace(retained, std::make_unique<Node>(*target));
  auto neighbors = graph_.contraction_neighbors(*source, added, removed_links);
  if (_dbDriver) {
    const auto incoming = _dbDriver->submap_neighbors(removed);
    neighbors.insert(incoming.begin(), incoming.end());
  }
  neighbors.erase(retained);
  neighbors.erase(removed);
  std::list<int> cold;
  for (int id : neighbors) {
    if (auto* node = _getNode(id))
      nodes.emplace(id, std::make_unique<Node>(*node));
    else
      cold.push_back(id);
  }
  // Load summaries and edges in one batch. No cold neighbor contributes visual
  // content to this merge, so its large Feature.block must remain on disk.
  struct LoadedNodes {
    std::list<Node*> values;
    ~LoadedNodes() {
      for (auto* node : values) delete node;
    }
  } loaded;
  if (_dbDriver && !cold.empty()) _dbDriver->loadNodes(cold, loaded.values, nullptr, true);
  while (!loaded.values.empty()) {
    std::unique_ptr<Node> node(loaded.values.front());
    loaded.values.pop_front();
    const int id = node->id();
    nodes.emplace(id, std::move(node));
  }
  for (int id : neighbors)
    if (!nodes.count(id)) throw std::runtime_error("Missing incident submap neighbor");
  trace.mark("neighbors_read");
  auto source_draft = std::make_unique<Node>(*source);
  graph_.prepare_edge_edits(nodes, *source_draft, added, removed_links);
  source = source_draft.get();

  auto& merged = *nodes.at(retained);
  merged.set_map_features(content.features);
  merged.set_navimap(local);
  merged.set_reference_frame({});  // no historical camera observation pretends to be the merged submap
  merged.setEnabled(true);
  merged.setWeight(std::max(merged.getWeight(), source->getWeight()));
  if (merged.getLabel().empty()) merged.setLabel(source->getLabel());
  Topograph::redirect_edges(nodes, *source, retained, transform, constraints);

  // Stage only touched vocabulary postings; no whole-dictionary copy or re-quantization.
  std::optional<FbowIndex::ReferenceUpdate> references;
  if (_vwd)
    references.emplace(_vwd->prepare_replacement(retained, target->retrieval_words(), removed, source->retrieval_words(), merged.retrieval_words()));
  auto labels = _labels;
  labels.erase(removed);
  if (!merged.getLabel().empty()) labels[retained] = merged.getLabel();
  auto members = graph_.redirect_members(retained, removed, true);
  auto working = _workingMem;
  working.erase(removed);
  working[retained] = tools::steady_time_seconds();
  auto recent = recent_feature_nodes_;
  recent.erase(std::remove(recent.begin(), recent.end(), removed), recent.end());
  recent.erase(std::remove(recent.begin(), recent.end(), retained), recent.end());
  recent.push_back(retained);
  return {retained,
          removed,
          graph_.topology_revision(),
          local->odom_from_anchor,
          std::move(nodes),
          std::move(labels),
          std::move(members),
          std::move(working),
          std::move(recent),
          std::move(references)};
}

bool Memory::can_apply(const NodeUpdate& update) const {
  return update.topology == graph_.topology_revision() && _lastNode && _lastNode->id() == update.removed && graph_.nodes().count(update.retained) &&
         update.nodes.count(update.retained) && retrieval_mass_.count(update.retained);
}
void Memory::apply_submap_merge(NodeUpdate&& update) noexcept {
  if (!can_apply(update)) std::terminate();
  const int retained = update.retained, removed = update.removed;
  _lastNode = &graph_.apply_contraction(retained, removed, update.nodes, update.members, update.odom_from_anchor, _dbDriver != nullptr);
  sensor_records_.erase(removed);
  _stMem.erase(removed);
  _stMem.erase(retained);
  _workingMem.swap(update.working);
  _labels.swap(update.labels);
  recent_feature_nodes_.swap(update.recent);
  _memoryChanged = true;
  for (auto& entry : graph_.nodes()) entry.second->write_epoch = _dbDriver ? _dbDriver->write_epoch() : 0;
  if (update.references) _vwd->apply_replacement(std::move(*update.references));
  retrieval_mass_.at(retained) = uses_raw_features() ? _lastNode->feature_count() : _lastNode->retrieval_mass();
  retrieval_mass_.erase(removed);
}

}  // namespace sapphire::backend

namespace sapphire::backend {
std::set<int> Memory::metric_component(int root) const {
  return graph_.metric_component(root, getAllNodeIds(), [this](int id) { return getLinks(id, true); });
}
}  // namespace sapphire::backend

namespace sapphire::backend {
int Memory::reduceNode(int id, float maxDistance, bool keepLinkedInDb, int direction) {
  const int retained = graph_.reduceNode(id, maxDistance, direction);
  if (retained) {
    moveToTrash(_getNode(id), keepLinkedInDb);
    _memoryChanged = true;
  }
  return retained;
}
}  // namespace sapphire::backend

namespace sapphire::backend {
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
}  // namespace sapphire::backend
