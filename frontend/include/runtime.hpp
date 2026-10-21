#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include "frontend/common/lidar_frame.hpp"
#include "app/output.hpp"
#include "frontend/common/imu_estimator.hpp"
#include "frontend/common/imu_factor.hpp"
#include "frontend/common/synchronizer.hpp"
#include "frontend/eskf/eskf.hpp"
#include "frontend/eskf/voxel_map.hpp"
#include "frontend/ricp/ricp.hpp"
#include "app/parameters.h"
#include "frontend/tools/parallel_executor.hpp"
namespace sapphire {
// Estimator owner. run() belongs to one caller-owned worker; stop/input are concurrent.
// No backend implementation or mutable-map access is exposed by this interface.
class FrontendRuntime {
 public:
  struct Handoff {
    // The producer waits before materializing frozen evidence. One producer only.
    std::function<bool()> wait_for_capacity;
    // Exclusive ownership transfers on entry; callbacks must not copy the cloud.
    std::function<void(LidarFrame &&)> lidar;
    std::function<void()> reset, invalidate_domain, suspend_output;
  };
  FrontendRuntime(const SapphireParameters &, OutputSink, Handoff);
  ~FrontendRuntime();  // caller must stop/join run() first
  FrontendRuntime(const FrontendRuntime &) = delete;
  FrontendRuntime &operator=(const FrontendRuntime &) = delete;
  bool push_imu(ImuMeas);
  bool push_lidar(double, std::vector<LidarPoint> &&, std::optional<double> = std::nullopt);
  bool push_image(ImageMeas);
  void stop();
  void run();
  std::deque<ImageMeas> take_images() { return synchronizer_.take_images(); }
  void clear_images() { synchronizer_.clear_images(); }
  Synchronizer::ImageStats image_stats() const { return synchronizer_.image_stats(); }

 private:
  friend struct SlamPipelineTestAccess;
  void notifyInputLoss();
  void invalidateOdometryDomain();
  void emit_local_scan(const vvec<double, 3> &points);
  void emit_local_map(int marginal_count);
  void clear_imu_factors();
  bool update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus, double journey, LidarFactor &voxel_hessian);
  int initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxel_hessian, vvec<double, 3> &points);
  void system_reset(const std::deque<ImuMeas> &imus);

  const SapphireParameters parameters_;
  OutputSink output_;
  Handoff handoff_;
  std::atomic_bool accepting_{true}, stopping_{false};
  std::atomic_uint64_t imu_refusals_{0}, lidar_refusals_{0}, input_revision_{0};
  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  Synchronizer synchronizer_;
  ParallelExecutor parallel_executor_;
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
  std::unique_ptr<ImuFactor> pending_imu_factor_;  // Since the last accepted window frame; odometry-thread owned.
  int window_count_ = 0;
  int window_base_ = 0;
  int degrade_bound_ = 10;
};
}  // namespace sapphire
