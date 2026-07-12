#pragma once

#include <sapphire/types.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <memory>

namespace sapphire {

/// GICP registration wrapper for scan-to-scan and scan-to-submap alignment.
///
/// Uses small_gicp::RegistrationPCL with sapphire::Point type.
/// Covariances are computed automatically by small_gicp via setInputSource/Target.
///
/// Usage:
///   Registration reg(config);
///   reg.setSource(source_cloud);
///   reg.setTarget(target_cloud);
///   auto result = reg.align(T_prior);
class Registration {
public:
    using GicpType = small_gicp::RegistrationPCL<Point, Point>;

    explicit Registration(const RegistrationConfig& config = RegistrationConfig{});

    /// Set the source point cloud (current scan, deskewed).
    void setSource(const PointCloudConstPtr& cloud);

    /// Set the target point cloud (previous scan or submap).
    void setTarget(const PointCloudConstPtr& cloud);

    /// Run GICP alignment.
    /// Both source and target clouds are already expressed in the world frame.
    /// @param T_prior IMU-predicted world pose at the scan reference time.
    /// @return global correction and corrected world-frame pose
    RegistrationResult align(const Isometry3d& T_prior);

private:
    RegistrationConfig cfg_;
    GicpType gicp_;
    bool target_set_ = false;
    size_t align_log_count_ = 0;
};

}  // namespace sapphire
