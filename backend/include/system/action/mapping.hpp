#pragma once
#include <backend/common/common_type.hpp>
#include <backend/common/root_frame.hpp>
#include <backend/registration/utils/rknn_models.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/io/runtime_resources.hpp>
#include <backend/system/action/build_frame.hpp>
#include <backend/system/action/keyframe_gate.hpp>
#include <backend/system/action/update_database.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/params/archive.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/node.hpp>
#include <backend/params/pgo.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/statistics.hpp>
#include <backend/params/topograph.hpp>
#include <functional>
#include <list>
#include <opencv2/core/core.hpp>
#include <set>
#include <string_view>

#include "backend/common/sensor_data.hpp"
#include "backend/common/state_group.hpp"
#include "backend/database/statistics.hpp"
#include "backend/maplayer/topograph/link.hpp"

namespace sapphire::backend {
class FrameBuilder;
class SensorRuntime;
class LocalGridMaker;
class VisualRegistration;
class Relocalizer;
struct Relocalization;

class Memory;
class Node;
namespace solver {
class PoseGraphSolver;
}
class LocalMapCollection;
struct LocalGrid;

// One explicit outcome per observation; diagnostics never determine control flow.
struct ObservationResult {
  enum class Outcome { InvalidProtocol, WaitingAnchor, Integrated, Absorbed, NodeAdmitted, NodeRejected };
  Outcome outcome = Outcome::WaitingAnchor;
  bool admitted() const { return outcome == Outcome::NodeAdmitted; }
  bool visual_update() const { return outcome == Outcome::NodeAdmitted || outcome == Outcome::NodeRejected; }
  bool session_changed = false;
  bool skip_integration = false;
  size_t ground_cells = 0, obstacle_cells = 0, free_cells = 0;
  int geometry_reference = 0;
  double geometry_ms = 0, feature_ms = 0;
  KeyframeGate::Geometry geometry;
  double depth_ms = 0;
  LocalGridTiming depth;
};

// Mapping algorithms and workspaces. BackendRuntime owns lifecycle and serializes calls.
class Mapping final {
 public:
  Mapping();
  ~Mapping();

  bool process(const SensorData& data, StateGroupf odomPose, const Mat66_t<double>& odomCovariance = Mat66_t<double>::Identity(),
               const std::vector<float>& odomVelocity = std::vector<float>(),
               const std::map<std::string, float>& externalStats = std::map<std::string, float>(), LocalMapCollection* localmaps = nullptr,
               std::shared_ptr<const LocalGrid> depth_grid = {});
  // Tracked online input: ordinary/rejected frames still contribute depth;
  // Protocol proposals are consumed after visual admission or geometric absorption;
  // the latter retains the existing map reference and ordinary-frame data path.
  // The caller may evaluate keyframes() first to avoid preparing visual work.
  ObservationResult process(const KeyframeGate::Observation& tracking, const SensorData& data, const StateGroupf& odom,
                            const Mat66_t<double>& covariance, const std::vector<float>& velocity, const std::map<std::string, float>& stats,
                            std::shared_ptr<const LocalGrid> depth_grid = {});
  // Drain trajectory before sealing/changing anchors. Failed storage stays retryable.
  void seal_localmap();
  // Synchronous durability boundary for the pipeline's existing mapping owner.
  void checkpoint();
  void getCompleteGraph(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& links) const;
  void append_lidar_odometry(double stamp, const StateGroupf&);
  bool process_lidar(const SensorData&, const StateGroupf&, const Mat66_t<double>&);
  void restore_localmaps(bool navimap_enabled, bool load_previous);
  void prepare_models(const ParametersMap& parameters, const RuntimeResources& resources = RuntimeResources::defaults());
  void set_sensor_runtime(std::shared_ptr<SensorRuntime> runtime);
  const std::shared_ptr<SensorRuntime>& sensor_runtime() const { return sensor_runtime_; }
  const std::shared_ptr<tools::ParallelExecutor<>>& archive_executor() const { return archive_executor_; }

  // Configuration is fixed until close(). Database history never supplies tuning.
  void init(const ParametersMap& parameters, const std::string& databasePath = "", const RuntimeResources& resources = RuntimeResources::defaults());

  void close(bool databaseSaved = true, const std::string& ouputDatabasePath = "");

  /** @return Id of the loop-closure hypothesis accepted at the last @ref process() iteration, or 0 if none. */
  int getLoopClosureId() const { return topograph_.committed_loop(); }
  /** @return Id of the last non-intermediate node added to the map (0 if none). */
  int getLastLocationId() const;
  /** @return True if node ids are auto-generated (vs. taken from @ref SensorData::id()). */
  bool isIDsGenerated() const;
  /** @return Statistics produced by the last @ref process() iteration. */
  const Statistics& getStatistics() const;
  /** @return Optimized poses of the current local map (last graph optimization result). */
  const std::map<int, StateGroupf>& getLocalOptimizedPoses() const { return topograph_.poses(); }
  // Only this snapshot is expressed in the persistent world frame. The active
  // local graph may instead be an unlocalized odometry component.
  const std::map<int, StateGroupf>& getWorldOptimizedPoses() const { return topograph_.world_poses(); }
  std::string getRootFrameId() const { return _memory ? topograph_.root_frame_id() : std::string{}; }
  std::map<std::string, StateGroupf> getRootFrameAliases() const {
    return _memory ? topograph_.root_frame_aliases() : std::map<std::string, StateGroupf>{};
  }
  bool isMapAligned() const { return topograph_.aligned(); }
  std::set<int> swap_results();
  /** @return Constraints (links) of the current local map. */
  const std::multimap<int, Link>& getLocalConstraints() const { return topograph_.constraints(); }
  /**
   * @return Optimized pose of @p locationId in the current local map (identity if not present).
   */
  StateGroupf getPose(int locationId) const;

  StateGroupf getMapCorrection() const { return topograph_.correction(); }
  /** @return Owned @ref Memory (may be null before @ref init()). */
  const Memory* getMemory() const { return _memory.get(); }
  LocalMapCollection& localmaps() { return localmaps_; }
  const LocalMapCollection& localmaps() const { return localmaps_; }
  const KeyframeGate& keyframes() const { return keyframe_gate_; }
  TrajectoryWriter& trajectory() { return trajectory_writer_; }

  int triggerNewMap();

  // Offline maintenance may preserve the original Info.parameters record.
  // This changes only saved metadata, never the algorithms running this session.
  void setParameterRecord(const ParametersMap& parameters);

  const ParametersMap& getParameters() const { return _parameters; }

  void setOptimizedPoses(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints);

  Node getNodeCopy(int id, bool withWords, bool withGlobalDescriptors) const;

  void getGraph(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& constraints, bool optimized, bool global, std::map<int, Node>* nodes = 0,
                bool withWords = true, bool withGlobalDescriptors = true) const;

  // Offline callers own enumeration and cancellation; these are single algorithm operations.
  std::optional<Link> verify_loop(int from, int to, const StateGroupf& guess) const;
  std::map<int, StateGroupf> solve_graph(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& links) const;
  void commit_loops(const std::vector<Link>& added, std::map<int, StateGroupf> poses, const std::multimap<int, Link>& links);

  bool addLink(const Link& link);

 private:
  struct LoopProposal;
  struct GraphUpdate;
  bool commit_node_fusion(int current_id, const StateGroupf&, LocalMapCollection&, LoopProposal&, GraphUpdate&);
  void optimize_observation(const Node&, LoopProposal&, Relocalization&, bool reduced, GraphUpdate&);
  LoopProposal prepare_loops(const Node&, Relocalization&, LocalMapCollection*);
  bool process_frame(const SensorData&, StateGroupf, const Mat66_t<double>&, const std::vector<float>&, const std::map<std::string, float>&,
                     LocalMapCollection*, std::shared_ptr<const LocalGrid>, FrameBuilder::Result* prepared, bool allow_rehearsal);

  void optimizeCurrentMap(int id, bool lookInDatabase, std::map<int, StateGroupf>& optimizedPoses, std::optional<Mat66_t<double>>& covariance,
                          std::multimap<int, Link>* constraints = 0, double* error = 0, int* iterationsDone = 0,
                          const std::vector<Link>& candidates = {}, const featmap::NodeObservations* fusion = nullptr) const;
  std::map<int, StateGroupf> optimizeGraph(int fromId, const std::set<int>& ids, const std::map<int, StateGroupf>& guessPoses, bool lookInDatabase,
                                           std::optional<Mat66_t<double>>& covariance, std::multimap<int, Link>* constraints = 0, double* error = 0,
                                           int* iterationsDone = 0, const std::vector<Link>& candidates = {},
                                           const featmap::NodeObservations* fusion = nullptr) const;
  bool solve_submaps(int root, std::map<int, StateGroupf>&, const std::multimap<int, Link>&, std::optional<Mat66_t<double>>&, double* error = nullptr,
                     int* iterations = nullptr, const featmap::NodeObservations* fusion = nullptr) const;
  std::list<std::pair<int, int>> repairGraph(graph::MaxGraphErrors& maxGraphErrors, std::map<int, StateGroupf>& poses,
                                             std::multimap<int, Link>& constraints, double& optimizationError, int& optimizationIterations,
                                             std::optional<Mat66_t<double>>& optimizationCovariance);

  void selectWorldComponent(int reference);
  std::map<int, StateGroupf> optimizeOfflineComponents(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& links) const;
  Mat66_t<double> getInformation(const Mat66_t<double>& covariance) const;

 private:
  // Parameter definitions and loading belong to params/, not the algorithm owner.
  std::shared_ptr<feature::RknnModels> prepared_models_;
  NodeParams node_params_;
  ArchiveParams archive_params_;
  MemoryParams memory_params_;
  PGOParams pose_graph_params_;
  RelocalizerParams relocalizer_params_;
  StatisticsParams statistics_params_;
  TopographParams topograph_params_;

  // Algorithm history, separate from configuration and the task lifecycle state machine.
  struct RunState {
    bool nodes_transferred = false;
    float distance_travelled = 0;
    float distance_since_last_loop = 0;
  } run_;

  std::unique_ptr<solver::PoseGraphSolver> _graphOptimizer;
  ParametersMap _parameters;

  Topograph topograph_;

  LocalMapCollection localmaps_;
  KeyframeGate keyframe_gate_;

  uint64_t tracking_session_ = 0;
  double depth_stamp_ = 0;  // Independent capture order; never advances visual admission.
  bool navimap_enabled_ = true;
  TrajectoryWriter trajectory_writer_;
  std::unique_ptr<Memory> _memory;

  std::shared_ptr<SensorRuntime> sensor_runtime_;
  std::shared_ptr<tools::ParallelExecutor<>> archive_executor_;
  std::unique_ptr<FrameBuilder> frame_builder_;
  std::unique_ptr<LocalGridMaker> grid_builder_;
  std::unique_ptr<ArchiveWriter> archive_writer_;
  std::unique_ptr<Relocalizer> relocalizer_;

  StatisticsLog statistics_log_;

  Statistics statistics_;

  int previous_submap_keyframe_ = 0;  ///< Preceding accepted capture, not highest node ID.
  int previous_submap_map_ = -1;      ///< Do not continue adjacency across sessions.
};

}  // namespace sapphire::backend
