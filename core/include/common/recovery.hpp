#pragma once
#include <cstdint>

namespace sapphire {
enum class RecoveryReason {
  None, InvalidImu, InvalidLidar, InputUnavailable, ClockDiscontinuity,
  CapacityExceeded, ImuCoverage, ImuGap, LidarGap, ImuSilence, LidarSilence,
  ObservationRejected, InitializationRejected, EstimatorFailure, Manual
};
enum class TrackingPhase { WaitingForInput, Initializing, Tracking, Resetting, Stopped, Failed };
enum class ScanDisposition { Initializing, Tracked, ObservationRejected, Rejected, Cancelled, Failed };

struct ScanAdmission {
  std::uint64_t sequence = 0, generation = 0;
  bool accepted = false;
  RecoveryReason reason = RecoveryReason::None;
  explicit operator bool() const { return accepted; }
};
// Terminal processing acknowledgement, not a pose or a durability acknowledgement.
// Sequence is monotonic across clock resets; generation identifies the odom frame.
struct ScanResult {
  std::uint64_t sequence = 0, generation = 0;
  double stamp = 0;
  ScanDisposition disposition = ScanDisposition::Cancelled;
  RecoveryReason reason = RecoveryReason::None;
};
struct RecoveryProgress {
  TrackingPhase phase = TrackingPhase::WaitingForInput;
  RecoveryReason reason = RecoveryReason::None;
  std::uint64_t generation = 0, resets = 0, input_epoch = 0, initialization_attempts = 0;
  bool pending = false;
};
inline const char* recoveryReasonName(RecoveryReason reason) {
  switch (reason) {
    case RecoveryReason::None: return "none";
    case RecoveryReason::InvalidImu: return "invalid_imu";
    case RecoveryReason::InvalidLidar: return "invalid_lidar";
    case RecoveryReason::InputUnavailable: return "input_unavailable";
    case RecoveryReason::ClockDiscontinuity: return "clock_discontinuity";
    case RecoveryReason::CapacityExceeded: return "capacity_exceeded";
    case RecoveryReason::ImuCoverage: return "imu_coverage";
    case RecoveryReason::ImuGap: return "imu_gap";
    case RecoveryReason::LidarGap: return "lidar_gap";
    case RecoveryReason::ImuSilence: return "imu_silence";
    case RecoveryReason::LidarSilence: return "lidar_silence";
    case RecoveryReason::ObservationRejected: return "observation_rejected";
    case RecoveryReason::InitializationRejected: return "initialization_rejected";
    case RecoveryReason::EstimatorFailure: return "estimator_failure";
    case RecoveryReason::Manual: return "manual";
  }
  return "unknown";
}
}  // namespace sapphire
