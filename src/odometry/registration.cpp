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

    // C is a world-frame rigid motion. ||C.translation()|| includes the lever
    // arm of a rotation about the sensor once the pose leaves the origin.
    // Gate the motion of the sensor pose, T_measured = C * T_prior.
    const bool finite = T_correction.matrix().allFinite()
        && std::isfinite(reg_result.error);
    double lever_trans_m = std::numeric_limits<double>::infinity();
    double sensor_trans_m = std::numeric_limits<double>::infinity();
    double corr_rot_deg = std::numeric_limits<double>::infinity();
    if (finite) {
        lever_trans_m = T_correction.translation().norm();
        const Eigen::Vector3d p_prior = T_prior.translation();
        const Eigen::Vector3d p_measured =
            T_correction.rotation() * p_prior + T_correction.translation();
        sensor_trans_m = (p_measured - p_prior).norm();
        corr_rot_deg =
            lie::SO3d::log(lie::SO3d(T_correction.rotation())).norm()
            * (180.0 / M_PI);
    }

    result.attempted_trans_m = lever_trans_m;
    result.attempted_sensor_trans_m = sensor_trans_m;
    result.attempted_rot_deg = corr_rot_deg;

    bool accepted = converged && finite
        && reg_result.num_inliers
            >= static_cast<size_t>(cfg_.min_num_points);
    if (sensor_trans_m > cfg_.max_correction_trans) {
        accepted = false;
    }
    if (corr_rot_deg > cfg_.max_correction_rot_deg) {
        accepted = false;
    }
    if (!accepted) {
        spdlog::warn(
            "[registration] {} rejected: converged={} finite={} "
            "inliers={}/{} lever={:.3f}m sensor={:.3f}m rot={:.1f}deg",
            type_,
            converged,
            finite,
            reg_result.num_inliers,
            cfg_.min_num_points,
            lever_trans_m,
            sensor_trans_m,
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
