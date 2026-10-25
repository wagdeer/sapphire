#pragma once
#include <database/trajectory.hpp>
#include <maplayer/topograph/root_frame.hpp>
#include <parameters.hpp>
#include <maplayer/topograph/node.hpp>
#include <maplayer/topograph.hpp>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "common/common_type.hpp"
#include "common/se3_state.hpp"
#include "maplayer/topograph/link.hpp"

namespace sapphire {

class Node;
class DBDriver;
class Statistics;

// Working-set residency and persistence; Topograph owns graph structure and nodes.
class Memory final {
 public:
  static constexpr int kIdStart = 0;
  static constexpr int kIdVirtual = -1;

  Memory(const MappingParameters &parameters = {}, Topograph *graph = nullptr);
  Topograph &topograph() { return graph_; }
  const Topograph &topograph() const { return graph_; }
  ~Memory();

  const MappingParameters &getParameters() const { return parameters_; }

  void setParameterRecord(const ParametersMap &parameters);
  bool empty() const { return graph_.nodes().empty(); }
  int current_map_id() const { return _idMapCount; }

  Node *admit_node(std::unique_ptr<Node>, const Mat66_t<double> &covariance);
  // Move mature observations into the working set; never changes graph constraints.
  std::vector<int> promote_recent_nodes(bool all = false);

  bool init(const std::string &dbUrl, bool dbOverwritten = false);

  void close(bool databaseSaved = true, const std::string &ouputDatabasePath = "");

  int incrementMapId();

  std::list<int> forget(const std::set<int> &ignoredIds = std::set<int>());

  size_t working_memory_bytes() const;
  bool working_memory_over_budget(unsigned int node_limit) const;
  std::list<int> trim_working_memory(unsigned int node_limit, const std::set<int> &protected_ids = {});

  std::set<int> reactivateNodes(const std::list<int> &ids, unsigned int maxLoaded, double &timeDbAccess);

  uint64_t save_keyframe(int node_id, const std::vector<OdomObservation> &trajectory, const Statistics *statistics = nullptr, bool save_wm = false);

  const RootFrame &create_root_frame(int anchor, const SE3f &pose);

  void saveOptimizedPoses(const std::map<int, SE3f> &optimizedPoses, const SE3f &lastPose) const;

  std::map<int, SE3f> loadOptimizedPoses(SE3f *lastPose) const;

  int trajectory_high_water() const;

  void emptyTrash();

  void joinTrashThread();
  void checkpoint_nodes();

  bool addLink(const Link &link, bool addInDatabase = false);

  std::set<int> metric_component(int root) const;
  std::map<int, int> getNeighborsId(int nodeId, int maxGraphDepth, int maxCheckedInDatabase = -1, bool incrementMarginOnLoop = false,
                                    bool ignoreLoopIds = false, bool ignoreIntermediateNodes = false, const std::set<int> &nodesSet = std::set<int>(),
                                    double *dbAccessTime = 0) const;

  void deleteLocation(int locationId, bool keepLinkedInDb = false);

  const std::map<int, double> &getWorkingMem() const { return _workingMem; }

  size_t getWorkingMemSize(bool ignoreIntermediateNodes = false) const;

  const std::set<int> &getStMem() const { return _stMem; }

  // Committed adjacency only; cold lookup does not activate Node geometry.
  std::multimap<int, Link> getLinks(int nodeId, bool lookInDatabase = false) const;

  std::multimap<int, Link> getAllLinks(bool lookInDatabase, bool ignoreNullLinks = true) const;

  int getLastNodeId() const;

  const std::shared_ptr<DBDriver> &database() const { return _dbDriver; }

  const Node *getLastWorkingNode(bool ignoreIntermediateNodes) const;

  bool getNodeInfo(int nodeId, SE3f &odomPose, int &mapId, int &weight, std::string &label, double &stamp, std::vector<float> &velocity,
                   bool lookInDatabase = false) const;

  std::set<int> getAllNodeIds(bool ignoreChildren = true) const;

  const Node *getNode(int id) const;

  bool isInSTM(int nodeId) const { return _stMem.find(nodeId) != _stMem.end(); }

  bool isIDsGenerated() const { return parameters_.node.generate_node_ids; }

  void getMetricConstraints(const std::set<int> &ids, std::map<int, SE3f> &poses, std::multimap<int, Link> &links,
                            bool lookInDatabase = false);

 private:
  void updateAge(int nodeId);
  void saveLocationData(int locationId);
  SE3f getOdomPose(int nodeId, bool lookInDatabase = false) const;
  bool isInWM(int nodeId) const { return _workingMem.find(nodeId) != _workingMem.end(); }

  void clear();
  void loadDataFromDb();
  void moveToTrash(Node *s, bool keepLinkedToGraph = true, bool committed = false);

  void moveNodeToWMFromSTM(int id);
  void addNodeToWmFromLTM(Node *node);
  Node *_getNode(int id) const;
  std::list<Node *> getRemovableNodes(int count, const std::set<int> &ignoredIds = std::set<int>(), bool budget_pressure = false);
  std::list<int> transfer_to_ltm(const std::vector<int> &ids);

  int reserve_node_id(int requested);
  int getNextId();

  std::unique_ptr<Topograph> owned_graph_;  // Standalone Memory clients own their domain here.
  Topograph &graph_;

  std::shared_ptr<DBDriver> _dbDriver;

  const MappingParameters parameters_;  // Fixed startup configuration; database records never reconfigure it.
  std::optional<ParametersMap> parameter_record_;
  ParametersMap parameterRecord() const;
  bool use_previous_map_ = true;

  int _idCount;
  int _idMapCount;
  Node *_lastNode;
  bool _memoryChanged;  // Tracks admitted or changed map content.
  int _nodesAdded;
  int _workingMemIntermediateNodesCount;  // number of nodes with weight==-1 currently in _workingMem
  bool queue_node(const Node &node) const;
  std::set<int> _stMem;               // id
  std::map<int, double> _workingMem;  // id,age
};

}  // namespace sapphire
