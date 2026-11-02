#pragma once
#include <common/common_type.hpp>
#include <maplayer/topograph/root_frame.hpp>
#include <parameters.hpp>
#include <maplayer/topograph.hpp>
#include <functional>
#include "module/relocalizer.hpp"
#include <list>
#include <opencv2/core/core.hpp>
#include <set>
#include <string_view>

#include "common/se3_state.hpp"
#include "database/statistics.hpp"
#include "maplayer/topograph/link.hpp"

namespace sapphire {
class Relocalizer;
struct Relocalization;

class Memory;
class Node;
namespace solver {
class PoseGraphSolver;
}
class Mapping final {
 public:
  Mapping();
  ~Mapping();
  void checkpoint();
  // Execution policy only; both paths use the same finite geometry query.
  void enable_async_relocalization();
  bool poll_relocalization(bool wait = false);
  bool relocalization_pending() const;

  int getRelocalizationId() const { return topograph_.committed_loop(); }

  void getCompleteGraph(std::map<int, SE3f>& poses, std::multimap<int, Link>& links) const;
  void append_lidar_odometry(double stamp, const SE3f&);
  bool process_node(std::unique_ptr<Node>, const Mat66_t<double>&);
  void init(const MappingParameters& parameters, const std::string& databasePath = "", const std::string& log_directory = "");

  void close(bool databaseSaved = true, const std::string& ouputDatabasePath = "");

  int getLastLocationId() const;

  bool isIDsGenerated() const;

  const std::map<int, SE3f>& getLocalOptimizedPoses() const { return topograph_.poses(); }
  const std::map<int, SE3f>& getWorldOptimizedPoses() const { return topograph_.world_poses(); }
  std::string getRootFrameId() const { return _memory ? topograph_.root_frame_id() : std::string{}; }
  std::map<std::string, SE3f> getRootFrameAliases() const {
    return _memory ? topograph_.root_frame_aliases() : std::map<std::string, SE3f>{};
  }
  bool isMapAligned() const { return topograph_.aligned(); }
  std::set<int> swap_results();

  SE3f getPose(int locationId) const;

  SE3f getMapCorrection() const { return topograph_.correction(); }

  const Memory* getMemory() const { return _memory.get(); }

  int triggerNewMap();
  void setParameterRecord(const ParametersMap& parameters);

  const MappingParameters& getParameters() const { return _parameters; }

  void setOptimizedPoses(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& constraints);

  Node getNodeCopy(int id) const;

  void getGraph(std::map<int, SE3f>& poses, std::multimap<int, Link>& constraints, bool optimized, bool global,
                std::map<int, Node>* nodes = 0) const;
  std::optional<Link> verify_relocalization(int from, int to, const SE3f& guess) const;
  std::map<int, SE3f> solve_graph(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& links) const;
  void commit_relocalizations(const std::vector<Link>& added, std::map<int, SE3f> poses, const std::multimap<int, Link>& links);

  // Explicit optional alignment on a reopened archive without live odometry.
  // Seed is T_reference_query;
  // a rejected metric match changes no graph constraints or geometry.
  std::optional<Link> join_segments(int reference, int query, const SE3f& seed);
  std::size_t relocalize(); // All saved queries; automatic candidates/seeds, same verifier as live mapping.
  bool addLink(const Link& link);

 private:
  bool prepare_relocalization_graph(const Link&, std::map<int,SE3f>&,std::multimap<int,Link>&) const;
  int trajectory_sequence_ = 0;
  struct RelocalizationProposal;
  struct GraphUpdate;

  void optimize_observation(const Node&, RelocalizationProposal&, Relocalization&, bool reduced, GraphUpdate&);
  RelocalizationProposal prepare_relocalization(const Node&, Relocalization&);
  std::vector<Relocalizer::Input> prepare_relocalization_inputs(const Node&, Relocalization&);
  RelocalizationProposal finish_relocalization(int query, std::vector<Relocalizer::Verification>);
  void schedule_relocalization(const Node&);
  struct RelocalizationWork;
  std::unique_ptr<RelocalizationWork> relocalization_work_;
  bool async_relocalization_ = false;


  void optimizeCurrentMap(int id, bool lookInDatabase, std::map<int, SE3f>& optimizedPoses, std::optional<Mat66_t<double>>& covariance,
                          std::multimap<int, Link>* constraints = 0, double* error = 0, int* iterationsDone = 0,
                          const std::vector<Link>& candidates = {}) const;
  std::map<int, SE3f> optimizeGraph(int fromId, const std::set<int>& ids, const std::map<int, SE3f>& guessPoses, bool lookInDatabase,
                                           std::optional<Mat66_t<double>>& covariance, std::multimap<int, Link>* constraints = 0, double* error = 0,
                                           int* iterationsDone = 0, const std::vector<Link>& candidates = {}) const;
  bool solve_submaps(int root, std::map<int, SE3f>&, const std::multimap<int, Link>&, std::optional<Mat66_t<double>>&, double* error = nullptr,
                     int* iterations = nullptr) const;
  std::list<std::pair<int, int>> repairGraph(graph::MaxGraphErrors& maxGraphErrors, std::map<int, SE3f>& poses,
                                             std::multimap<int, Link>& constraints, double& optimizationError, int& optimizationIterations,
                                             std::optional<Mat66_t<double>>& optimizationCovariance);

  std::map<int, int> maintain_residency(bool all = false);
  void selectWorldComponent(int reference);
  std::map<int, SE3f> optimizeOfflineComponents(const std::map<int, SE3f>& poses, const std::multimap<int, Link>& links) const;
  Mat66_t<double> getInformation(const Mat66_t<double>& covariance) const;

 private:
  struct RunState {
    bool nodes_transferred = false;
    float distance_travelled = 0;
    float distance_since_last_loop = 0;
  } run_;

  std::unique_ptr<solver::PoseGraphSolver> _graphOptimizer;
  MappingParameters _parameters;

  Topograph topograph_;

  std::unique_ptr<Memory> _memory;

  std::unique_ptr<Relocalizer> relocalizer_;

  StatisticsLog statistics_log_;

  Statistics statistics_;
  std::ofstream loop_trace_, query_trace_;
};

}  // namespace sapphire
