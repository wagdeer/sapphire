#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/common.hpp"
#include "common/key_frame.hpp"
#include "parameters.h"
#include "backend/storage/retrieval_index.hpp"
#include "backend/visual/visual_loop.hpp"

namespace sapphire {
class OccupancyGrid;
struct GraphLink;
struct LocalGrid;
enum class CommitOutcome;

struct NavigationGrid {
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int width = 0;
  int height = 0;
  std::vector<int8_t> data;
  size_t revision = 0;
  // Historical reconstruction source; independent of the occupancy update counter.
  std::string map_uuid;
  std::uint64_t source_graph_revision = 0;
};

enum class AttachmentStatus { Attached, EmptyMap, InvalidTarget, MissingGeometry, InvalidSeed, InvalidQuery,
                              BbsRejected, GicpRejected, RegistrationFailure, NoAssociation, AmbiguousAssociation };

// Registration and local factor/Values preparation failures are retryable and
// have no durable result. From entry into the first potentially mutating ISAM
// operation, failures throw MapError with W's CommitOutcome and are sticky.
struct AttachmentResult {
  AttachmentStatus status = AttachmentStatus::RegistrationFailure;
  std::string message;
  int root_node_id = 0;  // SQL ID; zero on rejection.
  std::uint64_t committed_revision = 0;
  std::optional<Eigen::Isometry3d> correction;
};

enum class ContinuationAdmission { Accepted, Full, Stopped, Failed, WrongProducer, WrongSequence, Oversize, Busy, Invalid };
enum class ContinuationHead { Idle, Processing, Retryable, Failed };
struct ContinuationProgress {
  std::string map_uuid;
  std::optional<std::uint64_t> generation;
  std::uint64_t ready_revision = 0, accepted = 0, completed = 0;
  std::optional<std::uint64_t> committed_revision, completed_revision, head_sequence;
  std::optional<std::uint64_t> root_source, root_revision;
  bool committed_outcome_known = true, ready_available = false;
  std::exception_ptr first_failure;
  std::optional<std::uint64_t> last_accepted, last_completed, canceled_first, canceled_last;
  std::size_t waiting = 0, high_water = 0, waiting_bytes = 0, head_bytes = 0;
  std::optional<CommitOutcome> last_commit_outcome;
  ContinuationHead head = ContinuationHead::Idle;
  std::string cancellation_reason;
  // Pipeline-owned automatic stage, separate from accepted B2 work above.
  std::size_t association_pending = 0, association_bytes = 0, association_high_water = 0;
  std::uint64_t association_attempts = 0;
  bool automatically_attached = false;
};

enum class LoopDecisionStatus { Unavailable };

// Diagnostics from initial A2 reconstruction, retained as opening evidence.
// These are neither current committed poses nor runtime snapshots.
struct ReconstructionDiagnostics {
  std::size_t nodes = 0, factors = 0, odometry_factors = 0, loop_factors = 0;
  std::optional<Eigen::Isometry3d> original_prior;
  double max_translation = 0, max_rotation = 0;
  double max_residual_translation = 0, max_residual_rotation = 0;
  double objective_committed = 0, objective_rebuilt = 0, objective_delta = 0;
};

using NavigationGridCallback = std::function<void(std::shared_ptr<const NavigationGrid>)>;

/// ROS-free backend: new-map processing, historical reconstruction, and explicit
/// explicit attachment and bounded same-producer continuation in resume mode.
class PoseGraphBackend {
 public:
  PoseGraphBackend(const PoseGraphParameters &config, const NaviMapParameters &navi_map, const std::string &database_path,
                   NavigationGridCallback navigation_grid_callback, std::function<void()> failure_notification = {}, std::function<void()> ready_notification = {});

  ~PoseGraphBackend();

  PoseGraphBackend(const PoseGraphBackend &) = delete;
  PoseGraphBackend &operator=(const PoseGraphBackend &) = delete;

  // One root only. Q owns its original T_odomNew_Q; grid is already finalized
  // local evidence. target_node_id is SQL (1-based), seed transforms Q into H.
  // No continuation worker or publication callback is started.
  AttachmentResult attachFreshSession(SubmapFrame query, const LocalGrid &grid, int target_node_id,
                                      const Eigen::Isometry3d &T_H_Q_seed, std::uint64_t producer_generation = 0);
  // Pipeline retains the same frozen Q0 on clean rejection (successful attachment may replace its ID).
  AttachmentResult attachFreshSessionRetained(SubmapFrame &query, const LocalGrid &grid, int target_node_id,
                                              const Eigen::Isometry3d &seed, std::uint64_t producer_generation);
  // Explicit restricted automatic association: metric visual retrieval, independent
  // geometry and agreement among accepted top-k candidates. Clean rejection keeps
  // query/source identity and history unchanged; no guessed cross-domain spatial seed.
  AttachmentResult attachFreshSessionAutomatically(SubmapFrame &query, const LocalGrid &grid,
                                                     std::uint64_t producer_generation,
                                                     const SubmapFrame *later_visual_observation = nullptr);
  // Same-generation later images may initialize the root; root LiDAR is still verified.
  static std::optional<std::size_t> boundedInputBytes(const SubmapFrame &query, const LocalGrid &grid);
  // The producer generation must be the one captured with Q0, before ID replacement.
  void beginContinuation(std::uint64_t producer_generation);
  // Ownership transfers only on Accepted. One registered producer, at most one blocked submitter.
  ContinuationAdmission submitContinuation(SubmapFrame &query, LocalGrid &grid, std::uint64_t producer_generation, bool wait = true);
  void retryContinuation(std::uint64_t source_sequence, std::uint64_t producer_generation);
  ContinuationProgress continuationProgress() const;
  // Wakes capacity waits without acquiring lifecycle or joining. abort cancels queued old-domain work.
  void stopAdmission(bool abort = false);

  // End this frontend session, invalidate correction and checked-close storage.
  // Replace/reconstruct this fenced backend before another explicit attachment.
  void resetFreshSession();

  // Accepted W storage primitives; no resumed processing loop.
  void promoteStorageToWritable();
  void finish();
  void drain();  // Checked accepted-work join; healthy current reads remain available.
  void close();  // Checked read fence and complete storage close.
  bool captureReady(bool navigation, bool try_only, std::string &uuid, std::uint64_t &revision,
                    std::uint64_t &generation, std::uint64_t &source, double &timestamp,
                    Eigen::Isometry3d &correction, Eigen::Isometry3d &pose,
                    std::shared_ptr<const NavigationGrid> &grid) const;
  bool failed() const;
  std::uint64_t commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                      const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                      const std::optional<GraphLink> &loop, const std::string &uuid, std::uint64_t revision, int next_id,
                                      int chain_root);

  bool enabled() const;

  void addFrame(SubmapFrame frame);

  Eigen::Isometry3d T_map_odom() const;

  std::shared_ptr<const NavigationGrid> latestOccupancyGrid() const;

  bool historical() const;
  bool hasActiveCorrection() const;
  LoopDecisionStatus loopDecisionStatus() const;
  std::string mapUuid() const;
  std::uint64_t graphRevision() const;
  ReconstructionDiagnostics reconstructionDiagnostics() const;
  std::optional<Eigen::Isometry3f> committedPose(std::uint64_t id) const;
  std::vector<SpatialMatch> historicalSpatialCandidates(std::uint64_t query_id) const;
  std::vector<VisualSubmapMatch> transientVisualCandidates(const SubmapFrame &query);
  std::vector<VisualSubmapMatch> historicalVisualCandidates(std::uint64_t query_id);
  mapping::DescriptorArchive::IndexNodeMetadata historicalVisualMetadata() const;
  // Diagnostic copy made under the lifecycle lock, safe across attachment/reset.
  // Use latestOccupancyGrid for the normal shared output without copying evidence.
  OccupancyGrid historicalOccupancy() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
