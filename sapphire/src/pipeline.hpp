#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common/common.hpp"
#include "frontend/common/synchronizer.hpp"
#include "frontend/eskf/eskf.hpp"
#include "frontend/ricp/ricp.hpp"
#include "frontend/common/imu_estimator.hpp"
#include "frontend/common/imu_factor.hpp"
#include "common/key_frame.hpp"
#include "tools/parallel_executor.hpp"
#include "parameters.h"
#include "backend/graph/pose_graph.hpp"
#include "backend/grid/local_gridmap.hpp"
#include "frontend/eskf/voxel_map.hpp"

namespace sapphire {


struct OutputSink {
  std::function<void()> ready_changed;
  std::function<void(bool)> owner_reset; // Normal thread acknowledgement; never under backend/owner locks.
  std::function<void(bool)> optional_drop; // true = distinct local-map increment lost.
  std::function<void(const StateGroup &)> odom_state;
  // Synchronous odometry-thread diagnostic, after LiDAR correction and before BA.
  std::function<void(const StateGroup &, bool, double)> lidar_update;
  // Completion of one synchronized scan, including initialization/rejection/BA.
  std::function<void(double, double)> scan_processed;
  // World points, matching sensor origin, source timestamp and live odometry domain.
  std::function<void(std::shared_ptr<const vvec<double, 3>>, const Eigen::Vector3d&, double, std::uint64_t)> local_scan;
  std::function<void(std::shared_ptr<const std::vector<TrajectoryPoint>>)> trajectory;
  std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_map;
  std::function<void(const Eigen::Isometry3d &)> map_odom;
  std::function<void(const Eigen::Isometry3d &, double)> map_pose;
  std::function<void(std::shared_ptr<const NavigationGrid>)> navigation_grid;
};

class SlamPipeline {
 public:
  // Resume uses explicit B1 target/seed or opt-in automatic attachment. Frozen Q0
  // comes from this pipeline's live producer; no externally supplied correction unlocks it.
  explicit SlamPipeline(SapphireParameters parameters, OutputSink output = {},
                        std::optional<std::pair<int, Eigen::Isometry3d>> attachment = std::nullopt);
  void setAttachmentTarget(int target, const Eigen::Isometry3d &seed);
  void invalidateOdometryDomain();
  void notifyInputLoss();
  std::exception_ptr producerFailure() const;
  ContinuationProgress continuationProgress() const;
  void retryContinuation(std::uint64_t source_sequence);
  ~SlamPipeline();

  SlamPipeline(const SlamPipeline &) = delete;
  SlamPipeline &operator=(const SlamPipeline &) = delete;

  bool push_imu(ImuMeas imu);
  bool push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp = std::nullopt);
  bool push_lidar(double timestamp, const std::vector<LidarPoint> &cloud, std::optional<double> end_timestamp = std::nullopt) {
    if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) { notifyInputLoss(); return false; }
    auto owned = cloud; return push_lidar(timestamp, std::move(owned), end_timestamp);
  }
  bool push_image(ImageMeas image);
  void shutdown();
  ContinuationProgress drain();
  void close();
  bool captureReady(bool navigation, bool try_only, std::string &uuid, std::uint64_t &revision,
                    std::uint64_t &generation, std::uint64_t &source, double &timestamp,
                    Eigen::Isometry3d &correction, Eigen::Isometry3d &pose,
                    std::shared_ptr<const NavigationGrid> &grid) const;
  bool failed() const { return failed_.load(std::memory_order_acquire); }
  std::uint64_t unacceptedTailDrops() const { return unaccepted_tail_drops_.load(); }

 private:
  std::string backend_database_path() const;
  void create_pose_graph();
  void emit_local_trajectory(const vvec<double, 3> &points, double journey);
  void emit_local_map(int marginal_count);
  void clear_imu_factors();
  bool update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus,
                           double journey, LidarFactor &voxel_hessian);
  int initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxel_hessian, vvec<double, 3> &points);
  void system_reset(const std::deque<ImuMeas> &imus);
  void thd_odometry();
  void thd_mapping();
  void thd_association();
  void submitAutomaticSubmap(SubmapFrame submap, LocalGrid grid);
  void wakeStoppedProducers();

  const SapphireParameters parameters_;
  std::mutex attachment_mutex_;
  std::condition_variable attachment_cv_;
  std::optional<std::pair<int, Eigen::Isometry3d>> attachment_;
  std::uint64_t attachment_attempt_ = 0;
  std::uint64_t image_evictions_ = 0, image_samples_skipped_ = 0;
  std::atomic_uint64_t imu_refusals_{0}, lidar_refusals_{0}, unaccepted_tail_drops_{0};
  std::exception_ptr producer_failure_;
  OutputSink output_;
  Synchronizer synchronizer_;
  ParallelExecutor parallel_executor_;
  std::vector<TrajectoryPoint> trajectory_;
  StateGroup current_state_, extrinsic_;
  ImuEstimator imu_estimator_;
  ESKF eskf_;
  std::unique_ptr<Ricp> ricp_;
  VoxelMap voxel_map_;
  double down_size_inv_ = 1.0;
  int window_size_ = 0;
  std::vector<StateGroup> state_buffer_;
  std::vector<PointCloudPtr> point_buffer_;
  std::vector<double> time_buffer_;
  std::vector<double> journey_buffer_;
  std::vector<MeasGroup> init_buffer_;
  std::deque<ImuFactor *> imu_factor_buffer_;
  std::unique_ptr<ImuFactor> pending_imu_factor_; // Since the last accepted window frame; odometry-thread owned.
  int window_count_ = 0;
  int window_base_ = 0;
  int degrade_bound_ = 10;

  std::mutex marginal_mutex_;
  std::condition_variable marginal_cv_;
  std::deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>> marginal_frames_, reset_tail_;
  int reset_flag_ = 0;

  friend struct SlamPipelineTestAccess;
  std::atomic_bool failed_{false};
  std::atomic_bool accepting_{true}, output_domain_current_{true};
  std::atomic_bool stopping_{false};
  std::atomic_bool odometry_done_{false};

  std::atomic_int session_id_{0};
  std::uint64_t owner_generation_ = 0; // Installed/read with pose_graph_ under pose_graph_mutex_.
  std::atomic_uint64_t input_revision_{0};

  std::mutex input_mutex_;
  std::condition_variable input_cv_;

  std::mutex image_mutex_;
  std::deque<ImageMeas> image_buffer_;
  std::size_t image_buffer_bytes_ = 0;
  std::array<double, 2> last_image_time_{{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}};

  struct PendingAssociation {
    SubmapFrame submap;
    LocalGrid grid;
    std::size_t bytes;
  };
  mutable std::mutex association_mutex_;
  std::condition_variable association_cv_;
  std::deque<PendingAssociation> association_queue_;
  std::size_t association_bytes_ = 0, association_high_water_ = 0;
  std::uint64_t association_received_ = 0, association_attempts_ = 0;
  bool automatically_attached_ = false, association_producer_done_ = false;
  std::thread association_thread_;
  std::thread odometry_thread_;
  std::thread mapping_thread_;

  std::string filename_;
  std::string save_path_;
  int save_map_ = 0;
  mutable std::mutex pose_graph_mutex_;
  std::unique_ptr<PoseGraphBackend> pose_graph_;
};

using SapphirePipeline = SlamPipeline;

}  // namespace sapphire
