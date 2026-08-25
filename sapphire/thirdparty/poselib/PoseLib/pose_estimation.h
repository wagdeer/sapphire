#ifndef POSELIB_POSE_ESTIMATION_H_
#define POSELIB_POSE_ESTIMATION_H_

#include "PoseLib/camera_pose.h"
#include "PoseLib/types.h"

#include <cstddef>
#include <vector>

namespace poselibplus {

using CameraPose = poselib::CameraPose;
using Point2D = poselib::Point2D;
using Point3D = poselib::Point3D;
using RansacStats = poselib::RansacStats;

/**
 * Robust estimation options for normalized image coordinates.
 *
 * max_error is expressed on the normalized image plane. Pixel thresholds
 * should be divided by the focal length before entering PoseLibPlus.
 */
struct PoseEstimationOptions {
    poselib::RansacOptions ransac;
    poselib::BundleOptions bundle;
    double max_error = 0.01;
    std::size_t lo_iterations = 5;

    PoseEstimationOptions()
    {
        ransac.max_iterations = 100;
        ransac.min_iterations = 20;
        ransac.success_prob = 0.99;
        bundle.max_iterations = 10;
        bundle.loss_type = poselib::BundleOptions::CAUCHY;
    }
};

/**
 * Generalized-camera options. max_errors must be empty or contain one positive
 * normalized threshold per camera. Stratified samples use distinct cameras
 * where possible and never repeat a correspondence.
 */
struct GeneralizedPoseEstimationOptions : PoseEstimationOptions {
    std::vector<double> max_errors;
    std::size_t min_inliers_per_camera = 4;
    std::size_t min_valid_cameras = 2;
    bool stratified_sampling = true;
};

RansacStats estimateAbsolutePose(
        const std::vector<Point2D> & points2D,
        const std::vector<Point3D> & points3D,
        const PoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<char> * inliers);

RansacStats estimateRelativePose(
        const std::vector<Point2D> & points2D1,
        const std::vector<Point2D> & points2D2,
        const PoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<char> * inliers);

/**
 * Estimates world-to-rig pose from per-camera normalized observations.
 *
 * cameraExtrinsics transform rig coordinates into each camera coordinate
 * system. The output pose transforms world coordinates into rig coordinates.
 * All image points are normalized coordinates; no camera model is applied.
 */
RansacStats estimateGeneralizedAbsolutePose(
        const std::vector<std::vector<Point2D> > & points2D,
        const std::vector<std::vector<Point3D> > & points3D,
        const std::vector<CameraPose> & cameraExtrinsics,
        const GeneralizedPoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<std::vector<char> > * inliers);

} // namespace poselibplus

#endif // POSELIBPLUS_POSE_ESTIMATION_H_
