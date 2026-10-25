#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include "slam.hpp"
#include "parameters.hpp"
#include "module/imu_estimator.hpp"
#include "solver/factor/imu_factor.hpp"
#include "common/lidar_frame.hpp"
#include "module/synchronizer.hpp"
#include "module/eskf.hpp"
#include "maplayer/local_map.hpp"
#include "module/ricp.hpp"
#include "tools/parallel_executor.hpp"
namespace sapphire {
// Estimator owner. run() belongs to one caller-owned worker; stop/input are concurrent.
// Pipeline owns parameters/output and stops/joins run() before destroying this owner.
class FrontendRuntime {
 public:
  struct Handoff {
    // The producer waits before materializing frozen evidence. One producer only.
    std::function<bool()> wait_for_capacity;
    // Exclusive ownership transfers on entry; callbacks must not copy the cloud.
    std::function<void(LidarFrame &&)> lidar;
    std::function<void()> reset, invalidate_domain, suspend_output;
  };
  FrontendRuntime(const SapphireParameters &, const SlamPipeline::Callbacks &, Handoff);
  FrontendRuntime(const FrontendRuntime &) = delete;
  FrontendRuntime &operator=(const FrontendRuntime &) = delete;
  bool push_imu(ImuMeas);
  bool push_lidar(double, std::vector<LidarPoint> &&, std::optional<double> = std::nullopt);
  void stop();
  void run();

 private:
  friend struct SlamPipelineTestAccess;
  struct ScanCompletion;
  void notifyInputLoss();
  void invalidateOdometryDomain();
  void emit_local_scan(const vvec<double, 3> &points);
  void emit_local_map(int marginal_count);
  bool update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus, double journey);
  int initialization(MeasGroup &measures, vvec<double, 3> &points);
  void system_reset(const std::deque<ImuMeas> &imus);

  const SapphireParameters &parameters_;
  const SlamPipeline::Callbacks &output_;
  Handoff handoff_;
  std::atomic_bool accepting_{true}, stopping_{false};
  std::atomic_uint64_t imu_refusals_{0}, lidar_refusals_{0}, input_revision_{0};
  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  Synchronizer synchronizer_;
  tools::ParallelExecutor<> parallel_executor_;
  StateGroup current_state_, extrinsic_;
  ImuEstimator imu_estimator_;
  ESKF eskf_;
  std::unique_ptr<Ricp> ricp_;
  LocalMap local_map_;
};
}  // namespace sapphire
