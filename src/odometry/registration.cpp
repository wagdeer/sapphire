#include <sapphire/odometry/registration.hpp>
#include <spdlog/spdlog.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <limits>

namespace sapphire {

Registration::Registration(const RegistrationConfig& config)
    : cfg_(config)
{
    gicp_.setCorrespondenceRandomness(cfg_.k_correspondences);
    gicp_.setMaxCorrespondenceDistance(cfg_.max_correspondence_dist);
    gicp_.setMaximumIterations(cfg_.max_iterations);
    gicp_.setTransformationEpsilon(cfg_.transformation_epsilon);
    gicp_.setRotationEpsilon(cfg_.rotation_epsilon);

    spdlog::info("[registration] GICP ready: max_iter={} max_corr_dist={:.2f}m "
                 "k_corr={}", cfg_.max_iterations, cfg_.max_correspondence_dist,
                 cfg_.k_correspondences);
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

    auto t0 = std::chrono::steady_clock::now();
    gicp_.align(*aligned);
    auto t1 = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    bool converged = gicp_.hasConverged();
    const auto& reg_result = gicp_.getRegistrationResult();
    Eigen::Matrix4f T_corr_f = gicp_.getFinalTransformation();

    spdlog::info("[registration] GICP  {:.0f}ms  converged={}  inliers={}  "
                 "iter={}  error={:.4f}",
                 elapsed_ms, converged,
                 reg_result.num_inliers, reg_result.iterations, reg_result.error);

    const Eigen::Isometry3d T_correction(
        T_corr_f.cast<double>());

    // Rejection check (DLIO-compatible)
    const bool finite = T_correction.matrix().allFinite()
        && std::isfinite(reg_result.error);
    double corr_trans_m = std::numeric_limits<double>::infinity();
    double corr_rot_deg = std::numeric_limits<double>::infinity();
    if (finite) {
        corr_trans_m = T_correction.translation().norm();
        Eigen::Quaterniond q_corr(T_correction.rotation());
        q_corr.normalize();
        corr_rot_deg =
            2.0 * std::atan2(q_corr.vec().norm(), std::abs(q_corr.w()))
            * (180.0 / M_PI);
    }

    bool accepted = converged && finite
        && reg_result.num_inliers
            >= static_cast<size_t>(cfg_.min_num_points);
    if (!converged) {
        spdlog::warn("[registration] rejected: GICP did not converge");
    }
    if (!finite) {
        spdlog::warn("[registration] rejected: non-finite result");
    }
    if (reg_result.num_inliers
        < static_cast<size_t>(cfg_.min_num_points)) {
        spdlog::warn(
            "[registration] rejected: inliers={} < minimum={}",
            reg_result.num_inliers,
            cfg_.min_num_points);
    }
    if (corr_trans_m > cfg_.max_correction_trans) {
        spdlog::warn("[registration] rejected: trans={:.3f}m > {:.2f}m",
                     corr_trans_m, cfg_.max_correction_trans);
        accepted = false;
    }
    if (corr_rot_deg > cfg_.max_correction_rot_deg) {
        spdlog::warn("[registration] rejected: rot={:.1f}deg > {:.1f}deg",
                     corr_rot_deg, cfg_.max_correction_rot_deg);
        accepted = false;
    }

    if (accepted) {
        result.T_correction  = T_correction;
        result.T_world_lidar = T_correction * T_prior;
    } else {
        result.T_correction  = Isometry3d::Identity();
        result.T_world_lidar = T_prior;
    }

    result.converged     = converged;
    result.fitness_score = reg_result.error;
    result.elapsed_ms    = elapsed_ms;
    result.num_inliers   = reg_result.num_inliers;
    result.iterations    = reg_result.iterations;
    result.accepted      = accepted;

    return result;
}

}  // namespace sapphire
