#include <sapphire/odometry/registration.hpp>
#include <spdlog/spdlog.h>

#include <SO3.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace sapphire {

Registration::Registration(
    const RegistrationConfig& config,
    const std::string& type,
    double vgicp_voxel_resolution)
    : cfg_(config)
    , type_(type)
{
    gicp_.setRegistrationType(type_);
    if (type_ == "VGICP") {
        gicp_.setVoxelResolution(vgicp_voxel_resolution);
    }
    gicp_.setCorrespondenceRandomness(cfg_.k_correspondences);
    gicp_.setMaxCorrespondenceDistance(cfg_.max_correspondence_dist);
    gicp_.setMaximumIterations(cfg_.max_iterations);
    gicp_.setTransformationEpsilon(cfg_.transformation_epsilon);
    gicp_.setRotationEpsilon(cfg_.rotation_epsilon);

    spdlog::info(
        "[registration] {} ready: max_iter={} max_corr_dist={:.2f}m "
        "k_corr={} voxel={:.2f}m",
        type_,
        cfg_.max_iterations,
        cfg_.max_correspondence_dist,
        cfg_.k_correspondences,
        vgicp_voxel_resolution);
}

void Registration::setSource(const PointCloudConstPtr& cloud) {
    gicp_.setInputSource(cloud);
}

void Registration::setTarget(const PointCloudConstPtr& cloud) {
    gicp_.setInputTarget(cloud);
    target_set_ = true;
}

RegistrationResult Registration::align(const Isometry3d& T_prior) {
    RegistrationResult result;
    result.T_world_lidar = T_prior;

    if (!target_set_) {
        spdlog::warn("[registration] align() called before setTarget()");
        return result;
    }

    // DLIO pattern: both clouds already contain the IMU world-frame prior.
    // GICP therefore solves a near-identity global correction without an
    // additional relative-pose initial guess.
    PointCloudPtr aligned = std::make_shared<PointCloud>();

    gicp_.align(*aligned);

    bool converged = gicp_.hasConverged();
    const auto& reg_result = gicp_.getRegistrationResult();
    Eigen::Matrix4f T_corr_f = gicp_.getFinalTransformation();

    const Eigen::Isometry3d T_correction(
        T_corr_f.cast<double>());

    // Rejection check (DLIO-compatible)
    const bool finite = T_correction.matrix().allFinite()
        && std::isfinite(reg_result.error);
    double corr_trans_m = std::numeric_limits<double>::infinity();
    double corr_rot_deg = std::numeric_limits<double>::infinity();
    if (finite) {
        corr_trans_m = T_correction.translation().norm();
        corr_rot_deg =
            lie::SO3d::log(lie::SO3d(T_correction.rotation())).norm()
            * (180.0 / M_PI);
    }

    bool accepted = converged && finite
        && reg_result.num_inliers
            >= static_cast<size_t>(cfg_.min_num_points);
    if (corr_trans_m > cfg_.max_correction_trans) {
        accepted = false;
    }
    if (corr_rot_deg > cfg_.max_correction_rot_deg) {
        accepted = false;
    }
    if (!accepted) {
        spdlog::warn(
            "[registration] {} rejected: converged={} finite={} "
            "inliers={}/{} trans={:.3f}m rot={:.1f}deg",
            type_,
            converged,
            finite,
            reg_result.num_inliers,
            cfg_.min_num_points,
            corr_trans_m,
            corr_rot_deg);
    }

    if (accepted) {
        result.T_correction  = T_correction;
        result.T_world_lidar = T_correction * T_prior;
    } else {
        result.T_correction  = Isometry3d::Identity();
        result.T_world_lidar = T_prior;
    }

    result.converged     = converged;
    result.num_inliers   = reg_result.num_inliers;
    result.accepted      = accepted;
    result.hessian       = gicp_.getFinalHessian();
    result.hessian_valid = result.hessian.allFinite()
        && result.hessian.norm() > 0.0;

    return result;
}

}  // namespace sapphire
