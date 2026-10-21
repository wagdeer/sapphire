#include <backend/common/common_type.hpp>
#include <backend/parameters.hpp>
#include "backend/solver/ba_solver.hpp"
#include "backend/solver/ba_solver_diagnostics.hpp"

#include <PoseLib/pose_estimation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <backend/common/state_group.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <iterator>
#include <limits>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>

#include "backend/solver/ba_solver_factors.hpp"
#include "backend/sensor/camera/perspective.hpp"

namespace sapphire::backend::solver {
// GTSAM Pose3 orders tangent components [rotation, translation].
constexpr std::array<int, 6> kPose3TangentOrder{3, 4, 5, 0, 1, 2};
using Matrix6 = Mat66_t<double>;

static bool finitePoint(const cv::Point3f& point) { return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z); }

static StateGroupf stateFromPose3(const gtsam::Pose3& pose) {
  return sapphire::backend::StateGroupf::fromRotationTranslation(pose.rotation().matrix().cast<float>(), pose.translation().cast<float>());
}

// Reserve ten variable keys per frame so multi-camera rigs stay disjoint.
constexpr int cameraKeyStride = 10;

// Build a gtsam::Symbol for a 3D point. Word ids can be negative,
// but gtsam symbol cannot.
static inline gtsam::Symbol point3dSymbol(int id) {
  return id < 0 ? gtsam::Symbol('L', static_cast<std::uint64_t>(-id)) : gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

std::map<int, StateGroupf> BASolver::optimize_bundle(const BundleOptions& options, int rootId, const std::map<int, StateGroupf>& poses,
                                                     const std::multimap<int, Link>& links, const std::map<int, std::vector<camera::Perspective>>& models,
                                                     std::map<int, cv::Point3f>& points3DMap, const FeatureObservations& wordReferences,
                                                     RejectedObservations* outliers) {
  const int iterations = options.max_iterations;
  const double epsilon = options.min_error_improvement;
  const double pixelVariance_ = options.pixel_variance_px2;
  const double robustKernelDelta_ = options.robust_kernel_delta;
  const bool ignore_covariance = options.ignore_constraint_variance;
  const float gravity_sigma = options.gravity_sigma;
  if (iterations < 0 || !std::isfinite(epsilon) || epsilon < 0 ||
      !std::isfinite(pixelVariance_) || pixelVariance_ <= 0 ||
      !std::isfinite(robustKernelDelta_) || robustKernelDelta_ < 0 || !std::isfinite(gravity_sigma))
    throw std::invalid_argument("Invalid full-bundle solver options");
  std::map<int, StateGroupf> optimizedPoses;
  if (outliers) {
    outliers->clear();
  }
  EXT_LOG_DEBUG("Optimizing BA graph...");

  if (!(poses.size() >= 2 && iterations > 0 && (models.size() == poses.size() || poses.begin()->first < 0))) {
    EXT_LOG_WARN("GTSAM BA: nothing to optimize (poses={} models={} iterations={})", (int)poses.size(), (int)models.size(), iterations);
    return optimizedPoses;
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;

  bool hasGravityConstraints = false;
  if (gravity_sigma > 0.0f) {
    for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
      if (iter->second.from() == iter->second.to() && iter->second.type() == Link::kGravity && poses.find(iter->second.from()) != poses.end()) {
        hasGravityConstraints = true;
        break;
      }
    }
  }

  // 1) Add pose variables (in CAMERA frame: pose composed with local state).
  EXT_LOG_DEBUG("GTSAM BA: adding {} poses... (rootId={})", (int)poses.size(), rootId);
  int freeCamVertices = 0;
  for (std::map<int, StateGroupf>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter) {
    if (iter->first <= 0) {
      continue;
    }
    std::map<int, std::vector<camera::Perspective>>::const_iterator iterModel = models.find(iter->first);
    if (iterModel == models.end() || iterModel->second.empty()) {
      EXT_LOG_ERROR("GTSAM BA: missing camera model for pose {}", iter->first);
      return optimizedPoses;
    }
    for (size_t i = 0; i < iterModel->second.size(); ++i) {
      const camera::Perspective& m = iterModel->second[i];
      if (!m.isValidForProjection()) {
        EXT_LOG_ERROR("GTSAM BA: model {}.{} is invalid for projection", iter->first, (int)i);
        return optimizedPoses;
      }
      const StateGroupf camPose = iter->second.compose(m.cameraToBase());
      if (camPose.isNull()) {
        EXT_LOG_ERROR("GTSAM BA: null camera pose for {}.{}", iter->first, (int)i);
        return optimizedPoses;
      }
      const gtsam::Symbol xkey('x', iter->first * cameraKeyStride + (int)i);
      initialEstimate.insert(xkey, gtsam::Pose3(camPose.matrix().cast<double>()));

      // Fix the root pose (or fix everyone else if rootId<0). GTSAM has
      // no equivalent of g2o's setFixed(); the standard idiom is a
      // near-zero-sigma prior on each axis. We add this only to the
      // primary camera (i==0) of a multi-cam rig -- the others are
      // rigidly linked via the multi-cam BetweenFactors below.
      const bool fixNode = (rootId >= 0 && iter->first == rootId) || (rootId < 0 && iter->first != -rootId);
      if (fixNode && i == 0) {
        const double freeGravityAxisSigma = rootId >= 0 && hasGravityConstraints ? 1.0 : 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << freeGravityAxisSigma, freeGravityAxisSigma, 1e-9, 1e-9, 1e-9, 1e-9).finished());
        const gtsam::Symbol bodyKey('b', iter->first);
        initialEstimate.insert(bodyKey, gtsam::Pose3(iter->second.matrix().cast<double>()));
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(bodyKey, gtsam::Pose3(iter->second.matrix().cast<double>()), priorNoise));

        const gtsam::SharedNoiseModel rigidNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-9);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(xkey, bodyKey, gtsam::Pose3(m.cameraToBase().inverse().matrix().cast<double>()), rigidNoise));
      }
      if (!fixNode) {
        ++freeCamVertices;
      }
    }
  }
  if (freeCamVertices == 0) {
    EXT_LOG_ERROR("GTSAM BA has no free camera pose (rootId={}, poses={}). Not optimizing.", rootId, (int)poses.size());
    return optimizedPoses;
  }

  // 2) Pose-graph BetweenFactors (same role as the g2o EdgeSBACam edges).
  //    Expressed in camera frame using inverse/compose operations.
  //    cam = body composed with the local camera state.
  EXT_LOG_DEBUG("GTSAM BA: adding {} links...", (int)links.size());
  for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    const Link& link = iter->second;
    if (link.from() <= 0 || link.to() <= 0) {
      continue;
    }
    if (link.from() == link.to()) {
      if (gravity_sigma > 0.0f && link.type() == Link::kGravity && poses.find(link.from()) != poses.end() && models.find(link.from()) != models.end() &&
          !models.at(link.from()).empty()) {
        const StateGroupf measuredCameraPose = link.transform().compose(models.at(link.from())[0].cameraToBase());
        const gtsam::Rot3 nRcMeas = gtsam::Pose3(measuredCameraPose.matrix().cast<double>()).rotation();
        const gtsam::Unit3 nZ(0.0, 0.0, 1.0);
        const gtsam::Unit3 cGMeas = nRcMeas.unrotate(nZ);
        const gtsam::SharedNoiseModel gravityNoise = gtsam::noiseModel::Isotropic::Sigma(2, gravity_sigma);
        graph.add(gtsam::AttitudeFactor<gtsam::Pose3>(gtsam::Symbol('x', link.from() * cameraKeyStride), nZ, gravityNoise, cGMeas));
      }
      continue;
    }
    if (poses.find(link.from()) == poses.end() || poses.find(link.to()) == poses.end()) {
      continue;
    }
    EXT_ASSERT(!link.transform().isNull());

    const StateGroupf camLink = models.at(link.from())[0].cameraToBase().between(link.transform()).compose(models.at(link.to())[0].cameraToBase());

    Mat66_t<double> information = Mat66_t<double>::Identity();
    if (!ignore_covariance) {
      information = link.infMatrix();
    }
    const Mat66_t<double> mgtsam = information(kPose3TangentOrder, kPose3TangentOrder).eval();
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(mgtsam);

    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', link.from() * cameraKeyStride), gtsam::Symbol('x', link.to() * cameraKeyStride),
                                                 gtsam::Pose3(camLink.matrix().cast<double>()), noise));
  }

  // 3) Hard rigid edges between camera 0 and the other cameras of a
  //    multi-cam rig (g2o uses Identity*1e7; we mirror that here).
  for (std::map<int, std::vector<camera::Perspective>>::const_iterator iter = models.begin(); iter != models.end(); ++iter) {
    if (poses.find(iter->first) == poses.end()) {
      continue;
    }
    for (size_t i = 1; i < iter->second.size(); ++i) {
      const StateGroupf camLink = iter->second[0].cameraToBase().between(iter->second[i].cameraToBase());
      Mat66_t<double> information = Mat66_t<double>::Identity() * 9999999.0;
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(information);
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', iter->first * cameraKeyStride),
                                                   gtsam::Symbol('x', iter->first * cameraKeyStride + (int)i), gtsam::Pose3(camLink.matrix().cast<double>()),
                                                   noise));
    }
  }

  // 4) 3D points + reprojection observations.
  //
  // Every landmark gets an explicit Point3 variable and one factor per
  // observation, mono and stereo alike, matching g2o and Ceres. The GTSAM-native
  // choice for mono would be a SmartProjectionPoseFactor, but it marginalizes
  // the point out of the graph: no per-observation residual to threshold, and
  // its readback triangulation is a plain DLT one gross outlier drags off by
  // metres.
  EXT_LOG_DEBUG("GTSAM BA: adding {} 3D points and observations...", (int)points3DMap.size());

  // Maps each observation factor back to its <word, pose> for the sweep below.
  struct ObsFactor {
    size_t factorIndex;
    int wordId;
    int poseId;
  };
  std::vector<ObsFactor> obsFactors;
  std::set<gtsam::Key> insertedPoints;

  // Pixel measurements are converted once to camera-frame Unit3 bearings.
  // The tangent-space sigma is the small-angle equivalent of pixel noise.
  const double sigmaPixel = std::sqrt(pixelVariance_);
  gtsam::noiseModel::mEstimator::Base::shared_ptr huber;
  if (robustKernelDelta_ > 0.0) {
    // Huber reads delta in |r| units but bundle.robust_kernel_delta is a chi^2
    // threshold, so the knee deliberately sits above the rejection threshold: pass 1
    // then only caps gross outliers, which keeps its estimate a good basis for
    // deciding what to reject. Matching them throttles legitimate noise and costs
    // accuracy on weakly constrained far points.
    huber = gtsam::noiseModel::mEstimator::Huber::Create(robustKernelDelta_);
  }

  // Pose variables, priors and links are identical in both passes; only the
  // landmark part is rebuilt once observations are rejected.
  const gtsam::NonlinearFactorGraph poseGraph = graph;
  const gtsam::Values poseValues = initialEstimate;

  // Adds every landmark and observation except those in `excluded`. A landmark
  // with none left is dropped rather than added unconstrained, so readback keeps
  // the caller's input estimate -- what a fully rejected point has to keep.
  auto addLandmarks = [&](const RejectedObservations& excluded) {
    obsFactors.clear();
    insertedPoints.clear();
    for (FeatureObservations::const_iterator iter = wordReferences.begin(); iter != wordReferences.end(); ++iter) {
      const int wordId = iter->first;
      if (points3DMap.find(wordId) == points3DMap.end()) {
        continue;
      }
      const cv::Point3f pt3d = points3DMap.at(wordId);
      if (!finitePoint(pt3d)) {
        EXT_LOG_WARN("Ignoring 3D point {} because it has nan value(s)!", wordId);
        continue;
      }

      RejectedObservations::const_iterator excludedIter = excluded.find(wordId);

      // Collect usable observations first: none left means no variable at all.
      std::vector<std::pair<std::map<int, FrameObservation>::const_iterator, cv::Point3d>> kept;
      for (std::map<int, FrameObservation>::const_iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter) {
        const int poseId = jter->first;
        const gtsam::Symbol xkey('x', poseId * cameraKeyStride + jter->second.camera_index);
        const std::map<int, std::vector<camera::Perspective>>::const_iterator modelIter = models.find(poseId);
        if (poses.find(poseId) == poses.end() || modelIter == models.end() || jter->second.camera_index < 0 ||
            jter->second.camera_index >= static_cast<int>(modelIter->second.size()) || !poseValues.exists(xkey) ||
            (excludedIter != excluded.end() && excludedIter->second.count(poseId))) {
          continue;
        }
        const std::optional<cv::Point3d> bearing = modelIter->second[jter->second.camera_index].bearing(jter->second.pixel);
        if (bearing) kept.emplace_back(jter, *bearing);
      }
      if (kept.empty()) {
        continue;
      }

      const gtsam::Symbol pkey = point3dSymbol(wordId);
      initialEstimate.insert(pkey, gtsam::Point3(pt3d.x, pt3d.y, pt3d.z));
      insertedPoints.insert(pkey);

      for (size_t k = 0; k < kept.size(); ++k) {
        const int poseId = kept[k].first->first;
        const FrameObservation& f = kept[k].first->second;
        const gtsam::Symbol xkey('x', poseId * cameraKeyStride + f.camera_index);
        const camera::Perspective& camera = models.at(poseId).at(f.camera_index);
        const cv::Point3d& bearing = kept[k].second;
        gtsam::SharedNoiseModel bearingNoise = gtsam::noiseModel::Isotropic::Sigma(2, sigmaPixel / (0.5 * (camera.fx() + camera.fy())));
        if (huber) bearingNoise = gtsam::noiseModel::Robust::Create(huber, bearingNoise);
        const size_t factorIdx = graph.size();
        graph.add(std::make_shared<Unit3ProjectionFactor>(xkey, pkey, gtsam::Unit3(bearing.x, bearing.y, bearing.z), bearingNoise));
        obsFactors.push_back(ObsFactor{factorIdx, wordId, poseId});
      }
    }
  };
  addLandmarks(RejectedObservations());

  // 5) Optimize. Wrapped so the rejection pass can re-solve; false = gave up.
  tools::Timer timer;
  gtsam::Values result;
  double finalError = std::numeric_limits<double>::quiet_NaN();
  auto solveGraph = [&](int maxIterations) -> bool {
    try {
      // Always use Levenberg-Marquardt for BA, ignoring pose_graph.optimizer.
      // Same rationale as the g2o BA path: BA's Hessian is often
      // near-singular (points near infinity, near-parallel rays), so
      // Gauss-Newton's unbounded step can blow up. Dogleg works but
      // offers no advantage over LM on BA. LM is what every major BA
      // library (Ceres, g2o, COLMAP) defaults to.
      gtsam::LevenbergMarquardtParams params;
      if (epsilon > 0.0) {
        params.relativeErrorTol = epsilon;
        params.absoluteErrorTol = epsilon;
      }
      params.maxIterations = maxIterations;
      // Use PCG + Block-Jacobi instead of GTSAM's default multifrontal
      // Cholesky, which is faster here. The inner tolerances must be tight
      // enough that the iterative solver doesn't bottom out before LM
      // converges; 1e-10 comes from GTSAM's SFMExample_SmartFactorPCG.
      params.linearSolverType = gtsam::NonlinearOptimizerParams::Iterative;
      gtsam::PCGSolverParameters::shared_ptr pcg(new gtsam::PCGSolverParameters());
      gtsam::PreconditionerParameters::shared_ptr preconditioner(new gtsam::BlockJacobiPreconditionerParameters());
      pcg->preconditioner = preconditioner;
      pcg->epsilon_abs = 1e-10;
      pcg->epsilon_rel = 1e-10;
      params.iterativeParams = pcg;
      gtsam::NonlinearOptimizer* optimizer = new gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, params);
      EXT_LOG_DEBUG("GTSAM BA optimizing (max iterations={}, robustKernel={:.6f})...", maxIterations, robustKernelDelta_);
      result = optimizer->optimize();
      finalError = optimizer->error();
      EXT_LOG_DEBUG("GTSAM BA done (initialError={:.6f} finalError={:.6f} time={:.6f}s)", graph.error(initialEstimate), finalError, timer.lap_seconds());
      delete optimizer;
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      EXT_LOG_ERROR("GTSAM BA: indeterminant linear system: {}", e.what());
      return false;
    } catch (const std::exception& e) {
      EXT_LOG_ERROR("GTSAM BA failed: {}", e.what());
      return false;
    }

    if (std::isnan(finalError)) {
      EXT_LOG_ERROR("GTSAM BA produced a NaN error.");
      return false;
    }
    return true;
  };

  // Pass 1 only needs to get close enough for bad residuals to stand out; pass 2
  // re-solves with the full budget. 5 matches the g2o backend.
  const bool rejectOutliers = robustKernelDelta_ > 0.0;
  if (!solveGraph(rejectOutliers ? std::min(5, iterations) : iterations)) {
    EXT_LOG_WARN("GTSAM BA: solve failed, aborting optimization!");
    return optimizedPoses;
  }

  // 5b) Hard rejection, like g2o. The Huber kernel only down-weights, and that
  //     residual pull keeps biasing the landmark however many iterations run.
  //     Runs even when the caller wants no report -- rejection is what fixes
  //     the estimate -- and pass 2 runs even with nothing rejected, since pass 1
  //     was truncated.
  RejectedObservations rejected;
  if (rejectOutliers) {
    // chi^2 > delta, the documented meaning of bundle.robust_kernel_delta,
    // matching the historical behavior. error() returns 0.5*rho(|r|), and rho == chi^2
    // below the kernel knee -- which the whole rejection band sits under -- so
    // 2*error is the raw chi^2 here. Past the knee rho still exceeds delta.
    int rejectedCount = 0;
    for (std::vector<ObsFactor>::const_iterator iter = obsFactors.begin(); iter != obsFactors.end(); ++iter) {
      if (iter->factorIndex >= graph.size()) continue;
      if (2.0 * graph.at(iter->factorIndex)->error(result) > robustKernelDelta_) {
        rejected[iter->wordId].insert(iter->poseId);
        ++rejectedCount;
      }
    }
    EXT_LOG_DEBUG("GTSAM BA: re-solving without {} rejected observation(s) over {} word(s)...", rejectedCount, (int)rejected.size());
    const gtsam::Values firstPass = result;
    graph = poseGraph;
    initialEstimate = poseValues;
    addLandmarks(rejected);
    // Warm-start from pass 1 where the variable survived, like g2o.
    const auto warmStartKeys = initialEstimate.keys();
    for (const gtsam::Key& key : warmStartKeys) {
      if (firstPass.exists(key)) {
        initialEstimate.update(key, firstPass.at(key));
      }
    }
    if (!solveGraph(iterations)) {
      // Rejection left something unsolvable (a landmark down to one ray).
      // The first-pass solution still carries the outliers' pull, so it is
      // not worth handing back -- fail like the other solver paths do.
      EXT_LOG_WARN(
          "GTSAM BA: re-solve without the {} rejected observation(s) failed, "
          "aborting optimization!",
          rejectedCount);
      return optimizedPoses;
    }
  }
  if (outliers) {
    *outliers = rejected;
  }

  // 6) Read back poses (camera frame -> body frame via localTransform^-1).
  for (std::map<int, StateGroupf>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter) {
    if (iter->first <= 0) {
      continue;
    }
    const gtsam::Symbol xkey('x', iter->first * cameraKeyStride);
    if (!result.exists(xkey)) {
      continue;
    }
    StateGroupf t = stateFromPose3(result.at<gtsam::Pose3>(xkey));
    t = t.compose(models.at(iter->first)[0].cameraToBase().inverse());
    if (t.isNull()) {
      EXT_LOG_ERROR("GTSAM BA: optimized pose {} is null", iter->first);
      optimizedPoses.clear();
      return optimizedPoses;
    }
    optimizedPoses.insert(std::make_pair(iter->first, t));
  }

  // 7) Read back 3D points. Fully rejected landmarks were never added as
  //    variables, so their input estimate stands.
  for (std::map<int, cv::Point3f>::iterator iter = points3DMap.begin(); iter != points3DMap.end(); ++iter) {
    const gtsam::Symbol pkey = point3dSymbol(iter->first);
    if (insertedPoints.count(pkey) && result.exists(pkey)) {
      const gtsam::Point3 p = result.at<gtsam::Point3>(pkey);
      iter->second = cv::Point3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
  }

  return optimizedPoses;
}

BASolver::PnPResult BASolver::estimate_motion_pnp(const std::vector<PnPMatch>& observations, const camera::Perspective& perspective, const StateGroupf& guess,
                                                  const PnPOptions& request) {
  PnPResult result;
  if (!perspective.isValidForProjection() || !guess.isValid() || request.min_inliers < 4 || request.iterations <= 0 || !(request.reprojection_error > 0) ||
      !std::isfinite(request.reprojection_error) || request.refine_iterations < 0) {
    result.message = "Invalid PnP request";
    return result;
  }
  try {
    const int minInliers = request.min_inliers, iterations = request.iterations;
    const double reprojError = request.reprojection_error;
    StateGroupf transform;
    std::vector<int> matches, inliers;

    // find correspondences
    std::vector<cv::Point3f> objectPoints(observations.size());
    std::vector<cv::Point2f> imagePoints(observations.size());
    int oi = 0;
    matches.resize(observations.size());
    for (unsigned int i = 0; i < observations.size(); ++i) {
      if (finitePoint(observations[i].source_point)) {
        objectPoints[oi] = observations[i].source_point;
        if (observations[i].target.camera_index != 0) throw std::invalid_argument("Invalid PnP camera index");
        if (!std::isfinite(observations[i].target.pixel.x) || !std::isfinite(observations[i].target.pixel.y))
          throw std::invalid_argument("Nonfinite PnP pixel");
        imagePoints[oi] = observations[i].target.pixel;
        matches[oi++] = i;
      }
    }

    objectPoints.resize(oi);
    imagePoints.resize(oi);
    matches.resize(oi);

    EXT_LOG_DEBUG("PnP observations={} matches={} guess={} reprojError={:.6f} iterations={}", observations.size(), matches.size(), guess.debugString(),
                  reprojError, iterations);

    if ((int)matches.size() >= minInliers) {
      // PnPRansac
      StateGroupf guessCameraFrame = guess.compose(perspective.cameraToBase()).inverse();
      const StateGroupf::Matrix3 guessRotation = guessCameraFrame.rotationMatrix();
      const StateGroupf::Vector3 guessTranslation = guessCameraFrame.translationVector();
      cv::Mat R(3, 3, CV_64FC1);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          R.at<double>(row, column) = guessRotation(row, column);
        }
      }

      cv::Mat rvec(3, 1, CV_64FC1);
      cv::Rodrigues(R, rvec);
      cv::Mat tvec = (cv::Mat_<double>(3, 1) << static_cast<double>(guessTranslation.x()), static_cast<double>(guessTranslation.y()),
                      static_cast<double>(guessTranslation.z()));

      solvePnPRansac(objectPoints, imagePoints, perspective, rvec, tvec, inliers, request);

      if ((int)inliers.size() >= minInliers) {
        cv::Rodrigues(rvec, R);
        StateGroupf::Matrix3 pnpRotation;
        StateGroupf::Vector3 pnpTranslation;
        for (int row = 0; row < 3; ++row) {
          pnpTranslation[row] = static_cast<float>(tvec.at<double>(row));
          for (int column = 0; column < 3; ++column) {
            pnpRotation(row, column) = static_cast<float>(R.at<double>(row, column));
          }
        }
        const StateGroupf pnp = StateGroupf::fromRotationTranslation(pnpRotation, pnpTranslation);

        transform = perspective.cameraToBase().compose(pnp).inverse();
      }
    }

    result.matches = std::move(matches);
    result.inliers.reserve(inliers.size());
    for (int row : inliers) result.inliers.push_back(result.matches[row]);
    result.inliers_per_camera = {int(result.inliers.size())};
    result.pose = transform;
    result.status = transform.isValid() ? Status::Success : Status::InsufficientSupport;
    if (result.status != Status::Success) result.message = "Insufficient PnP inliers";
    return result;
  } catch (const std::invalid_argument& e) {
    return {Status::InvalidProblem, e.what()};
  } catch (const std::exception& e) {
    return {Status::NumericalFailure, e.what()};
  }
}

BASolver::PnPResult BASolver::estimate_motion_pnp(const std::vector<PnPMatch>& observations, const std::vector<camera::Perspective>& perspectives,
                                                  const StateGroupf& guess, const PnPOptions& request) {
  PnPResult result;
  if (perspectives.empty() || !guess.isValid() || request.min_inliers < 4 || request.iterations <= 0 || !(request.reprojection_error > 0) ||
      !std::isfinite(request.reprojection_error) || request.refine_iterations < 0 ||
      std::any_of(perspectives.begin(), perspectives.end(), [](const auto& camera) { return !camera.isValidForProjection(); })) {
    result.message = "Invalid PnP request";
    return result;
  }
  if (perspectives.size() == 1) return estimate_motion_pnp(observations, perspectives.front(), guess, request);
  try {
    const int minInliers = request.min_inliers, iterations = request.iterations, refineIterations = request.refine_iterations;
    const double reprojError = request.reprojection_error;
    StateGroupf transform;
    std::vector<int> matches, inliers;

    // find correspondences
    std::vector<cv::Point3f> objectPoints(observations.size());
    std::vector<cv::Point2f> imagePoints(observations.size());
    int oi = 0;
    matches.resize(observations.size());
    std::vector<int> cameraIndexes(observations.size());
    for (unsigned int i = 0; i < observations.size(); ++i) {
      if (finitePoint(observations[i].source_point)) {
        const cv::Point2f& kpt = observations[i].target.pixel;
        if (!std::isfinite(kpt.x) || !std::isfinite(kpt.y)) throw std::invalid_argument("Nonfinite PnP pixel");
        const int cameraIndex = observations[i].target.camera_index;
        if (cameraIndex < 0 || cameraIndex >= int(perspectives.size())) throw std::invalid_argument("Invalid PnP camera index");

        const cv::Point3f& pt = observations[i].source_point;
        objectPoints[oi] = pt;
        imagePoints[oi] = kpt;
        cameraIndexes[oi] = cameraIndex;
        matches[oi++] = i;
      }
    }

    objectPoints.resize(oi);
    imagePoints.resize(oi);
    cameraIndexes.resize(oi);
    matches.resize(oi);

    EXT_LOG_DEBUG("PnP observations={} matches={} guess={} reprojError={:.6f} iterations={}", observations.size(), matches.size(), guess.debugString(),
                  reprojError, iterations);

    std::vector<int> matchCounts(perspectives.size(), 0);
    for (size_t i = 0; i < cameraIndexes.size(); ++i) {
      ++matchCounts[cameraIndexes[i]];
    }

    std::vector<int> activeCameraIndexes;
    std::vector<int> cameraToActiveIndex(perspectives.size(), -1);
    for (size_t i = 0; i < perspectives.size(); ++i) {
      EXT_LOG_DEBUG("Matches in Camera {}: {}", (int)i, matchCounts[i]);
      if (matchCounts[i] >= minInliers) {
        cameraToActiveIndex[i] = activeCameraIndexes.size();
        activeCameraIndexes.push_back(i);
      }
    }

    if (activeCameraIndexes.size() >= 2) {
      std::vector<std::vector<poselibplus::Point2D>> multiPoints2D(activeCameraIndexes.size());
      std::vector<std::vector<poselibplus::Point3D>> multiPoints3D(activeCameraIndexes.size());
      std::vector<poselibplus::CameraPose> cameraExtrinsics;
      cameraExtrinsics.reserve(activeCameraIndexes.size());
      std::vector<std::vector<int>> localIndexToGlobalIndex(activeCameraIndexes.size());
      for (size_t i = 0; i < activeCameraIndexes.size(); ++i) {
        const camera::Perspective& model = perspectives[activeCameraIndexes[i]];
        const StateGroupd rigToCamera(model.cameraToBase().inverse());
        cameraExtrinsics.push_back(poselibplus::CameraPose(rigToCamera.rotationMatrix(), rigToCamera.translationVector()));
      }

      for (size_t i = 0; i < objectPoints.size(); ++i) {
        int cameraIndex = cameraIndexes[i];
        int activeCameraIndex = cameraToActiveIndex[cameraIndex];
        if (activeCameraIndex < 0) {
          continue;
        }
        const std::optional<cv::Point3d> bearing = perspectives[cameraIndex].bearing(imagePoints[i]);
        if (!bearing || std::abs(bearing->z) <= std::numeric_limits<double>::epsilon()) {
          continue;
        }
        multiPoints2D[activeCameraIndex].push_back(poselibplus::Point2D(bearing->x / bearing->z, bearing->y / bearing->z));
        multiPoints3D[activeCameraIndex].push_back(poselibplus::Point3D(objectPoints[i].x, objectPoints[i].y, objectPoints[i].z));
        localIndexToGlobalIndex[activeCameraIndex].push_back(i);
      }

      const StateGroupd worldToRig(guess.inverse());
      poselibplus::CameraPose pose(worldToRig.rotationMatrix(), worldToRig.translationVector());

      poselibplus::GeneralizedPoseEstimationOptions options;
      options.max_errors.reserve(activeCameraIndexes.size());
      for (size_t i = 0; i < activeCameraIndexes.size(); ++i) {
        options.max_errors.push_back(reprojError / perspectives[activeCameraIndexes[i]].fx());
      }
      options.max_error = options.max_errors[0];
      options.min_inliers_per_camera = minInliers;
      options.min_valid_cameras = 2;
      options.ransac.max_iterations = std::max(1, iterations);
      options.ransac.min_iterations = std::min(options.ransac.max_iterations, size_t(20));
      options.ransac.score_initial_model = true;
      options.ransac.seed = request.seed ? *request.seed : static_cast<unsigned long>(std::chrono::steady_clock::now().time_since_epoch().count());
      options.lo_iterations = std::max(0, refineIterations);
      options.bundle.max_iterations = std::max(0, refineIterations);
      options.bundle.loss_scale = options.max_error;

      std::vector<std::vector<char>> inlierMasks;
      poselibplus::RansacStats stats =
          poselibplus::estimateGeneralizedAbsolutePose(multiPoints2D, multiPoints3D, cameraExtrinsics, options, &pose, &inlierMasks);

      const poselibplus::CameraPose rigToWorld = pose.inverse();
      const StateGroupf pnp = StateGroupf::fromRotationTranslation(rigToWorld.R().cast<float>(), rigToWorld.t.cast<float>());

      EXT_LOG_DEBUG("Ransac result: {}", pnp.debugString());
      EXT_LOG_DEBUG("Ransac iterations done: {}", (int)stats.iterations);
      std::vector<int> inlierCounts(perspectives.size(), 0);
      for (size_t i = 0; i < inlierMasks.size(); ++i) {
        const int cameraIndex = activeCameraIndexes[i];
        for (size_t j = 0; j < inlierMasks[i].size(); ++j) {
          if (inlierMasks[i][j]) {
            inliers.push_back(localIndexToGlobalIndex[i][j]);
            ++inlierCounts[cameraIndex];
          }
        }
      }

      int validatedCameras = 0;
      for (size_t i = 0; i < inlierCounts.size(); ++i) {
        EXT_LOG_DEBUG("Inliers in Camera {}: {}/{}", (int)i, inlierCounts[i], matchCounts[i]);
        if (inlierCounts[i] >= minInliers) {
          ++validatedCameras;
        }
      }

      EXT_LOG_DEBUG("Ransac inliers: {}, validated cameras: {}", inliers.size(), validatedCameras);

      if (validatedCameras >= 2 && !pnp.isNull()) {
        transform = pnp;
      } else {
        EXT_LOG_WARN("Rejected multi-camera PnP transform: only {} camera(s) reached the minimum of {} inliers.", validatedCameras, minInliers);
      }
    } else {
      EXT_LOG_WARN("Rejected multi-camera PnP input: only {} camera(s) have at least {} matches.", activeCameraIndexes.size(), minInliers);
    }

    // Retain camera-grouped ordering used by registration, without nested output containers.
    result.inliers_per_camera.resize(perspectives.size(), 0);
    for (size_t camera = 0; camera < perspectives.size(); ++camera) {
      for (size_t row = 0; row < matches.size(); ++row)
        if (cameraIndexes[row] == int(camera)) result.matches.push_back(matches[row]);
      for (int row : inliers)
        if (cameraIndexes[row] == int(camera)) {
          result.inliers.push_back(matches[row]);
          ++result.inliers_per_camera[camera];
        }
    }
    result.pose = transform;
    result.status = transform.isValid() ? Status::Success : Status::InsufficientSupport;
    if (result.status != Status::Success) result.message = "Insufficient multi-camera PnP inliers";
    return result;
  } catch (const std::invalid_argument& e) {
    return {Status::InvalidProblem, e.what()};
  } catch (const std::exception& e) {
    return {Status::NumericalFailure, e.what()};
  }
}

void BASolver::solvePnPRansac(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const camera::Perspective& perspective,
                              cv::Mat& rvec, cv::Mat& tvec, std::vector<int>& inliers, const PnPOptions& request) {
  int minInliersCount = request.min_inliers;
  const int iterationsCount = request.iterations, refineIterations = request.refine_iterations;
  const double reprojectionError = request.reprojection_error;
  const bool useExtrinsicGuess = true;
  if (minInliersCount < 4) {
    minInliersCount = 4;
  }
  inliers.clear();
  if (objectPoints.size() != imagePoints.size() || (int)objectPoints.size() < minInliersCount) {
    return;
  }

  const double focal = (perspective.fx() + perspective.fy()) * 0.5;
  if (!std::isfinite(focal) || focal <= 0.0) {
    return;
  }

  std::vector<poselibplus::Point2D> points2D;
  std::vector<poselibplus::Point3D> points3D;
  points2D.reserve(imagePoints.size());
  points3D.reserve(objectPoints.size());
  for (size_t i = 0; i < objectPoints.size(); ++i) {
    const std::optional<cv::Point3d> bearing = perspective.bearing(imagePoints[i]);
    if (!bearing || std::abs(bearing->z) <= std::numeric_limits<double>::epsilon()) {
      inliers.clear();
      return;
    }
    points2D.push_back(poselibplus::Point2D(bearing->x / bearing->z, bearing->y / bearing->z));
    points3D.push_back(poselibplus::Point3D(objectPoints[i].x, objectPoints[i].y, objectPoints[i].z));
  }

  poselibplus::CameraPose pose;
  if (useExtrinsicGuess && !rvec.empty() && !tvec.empty()) {
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    cv::Mat rotation64;
    cv::Mat translation64;
    rotation.convertTo(rotation64, CV_64FC1);
    tvec.convertTo(translation64, CV_64FC1);
    Mat33_t<double> eigenRotation;
    Vec3_t<double> eigenTranslation;
    for (int row = 0; row < 3; ++row) {
      eigenTranslation[row] = translation64.at<double>(row);
      for (int column = 0; column < 3; ++column) {
        eigenRotation(row, column) = rotation64.at<double>(row, column);
      }
    }
    pose = poselibplus::CameraPose(eigenRotation, eigenTranslation);
  }

  poselibplus::PoseEstimationOptions options;
  options.max_error = reprojectionError / focal;
  options.ransac.max_iterations = std::max(1, iterationsCount);
  options.ransac.min_iterations = std::min(options.ransac.max_iterations, size_t(20));
  options.ransac.score_initial_model = useExtrinsicGuess;
  options.ransac.seed = request.seed ? *request.seed : static_cast<unsigned long>(std::chrono::steady_clock::now().time_since_epoch().count());
  options.lo_iterations = refineIterations > 0 ? std::min(refineIterations, 5) : 0;
  options.bundle.max_iterations = std::max(0, refineIterations);
  options.bundle.loss_scale = options.max_error;

  std::vector<char> inlierMask;
  poselibplus::RansacStats stats = poselibplus::estimateAbsolutePose(points2D, points3D, options, &pose, &inlierMask);

  if ((int)stats.num_inliers < minInliersCount) {
    return;
  }

  for (size_t i = 0; i < inlierMask.size(); ++i) {
    if (inlierMask[i]) {
      inliers.push_back(i);
    }
  }

  const Mat33_t<double> resultRotation = pose.R();
  cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64FC1);
  tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  for (int row = 0; row < 3; ++row) {
    tvec.at<double>(row) = pose.t[row];
    for (int column = 0; column < 3; ++column) {
      rotation.at<double>(row, column) = resultRotation(row, column);
    }
  }
  cv::Rodrigues(rotation, rvec);
}

// Conditional pose covariance from left-image observations. The source points
// define projection geometry; independently measured query depths never enter.
// T = source_from_query, T' = T Exp(delta), delta = [translation, rotation].
std::optional<BASolver::RightPoseCovariance> BASolver::compute_covariance(const std::vector<PnPMatch>& observations, const std::vector<int>& inliers,
                                     const std::vector<camera::Perspective>& cameras, const StateGroupf& source_from_query) {
  if (!source_from_query.isValid() || inliers.size() < 3 || cameras.empty()) return {};
  struct CameraTransform {
    Mat33_t<double> rotation;
    Vec3_t<double> translation;
  };
  std::vector<CameraTransform> camera_from_query;
  camera_from_query.reserve(cameras.size());
  for (const auto& camera : cameras) {
    if (!camera.isValidForProjection() || !camera.cameraToBase().isValid()) return {};
    const auto optical_from_body = camera.cameraToBase().inverse();
    camera_from_query.push_back({optical_from_body.rotationMatrix().cast<double>(), optical_from_body.translationVector().cast<double>()});
  }
  Matrix6 information = Matrix6::Zero();
  double squared_error = 0;
  const auto query_from_source = source_from_query.inverse();
  const Mat33_t<double> query_rotation = query_from_source.rotationMatrix().cast<double>();
  const Vec3_t<double> query_translation = query_from_source.translationVector().cast<double>();
  for (const int index : inliers) {
    if (index < 0 || size_t(index) >= observations.size()) return {};
    const auto& match = observations[index];
    const auto& measurement = match.target;
    const int camera_index = measurement.camera_index;
    if (camera_index < 0 || size_t(camera_index) >= cameras.size() || !std::isfinite(measurement.variance) || measurement.variance <= 0 ||
        !std::isfinite(measurement.pixel.x) || !std::isfinite(measurement.pixel.y))
      return {};
    const auto& camera = cameras[camera_index];
    const auto& extrinsic = camera_from_query[camera_index];
    const Vec3_t<double> source(match.source_point.x, match.source_point.y, match.source_point.z);
    const Vec3_t<double> query = query_rotation * source + query_translation;
    const Vec3_t<double> optical = extrinsic.rotation * query + extrinsic.translation;
    if (!optical.allFinite() || optical.z() <= 0) return {};
    const double iz = 1.0 / optical.z();
    const auto jacobian = camera.poseJacobian(optical, query, extrinsic.rotation);
    const double weight = 1.0 / measurement.variance;
    information.noalias() += weight * jacobian.transpose() * jacobian;
    const double du = camera.fx() * optical.x() * iz + camera.cx() - measurement.pixel.x;
    const double dv = camera.fy() * optical.y() * iz + camera.cy() - measurement.pixel.y;
    squared_error += weight * (du * du + dv * dv);
  }
  // Normalize before factorization: translation and rotation have different
  // units. Do not invent information in unobservable directions by damping H.
  if (!std::isfinite(squared_error) || !information.allFinite() || (information.diagonal().array() <= 0).any()) return {};
  const Vec6_t<double> scale = information.diagonal().array().sqrt().inverse();
  const Matrix6 normalized = scale.asDiagonal() * information * scale.asDiagonal();
  const Eigen::LDLT<Matrix6> factor(normalized);
  if (factor.info() != Eigen::Success || (factor.vectorD().array() <= 1e-10).any()) return {};
  const double noise = std::max(1.0, squared_error / std::max(1.0, 2.0 * inliers.size() - 6.0));
  const Matrix6 estimate = noise * scale.asDiagonal() * factor.solve(Matrix6::Identity()) * scale.asDiagonal();
  if (!estimate.allFinite()) return {};
  return RightPoseCovariance{(.5 * (estimate + estimate.transpose())).eval()};
}

void configure_point_batch_solver(gtsam::LevenbergMarquardtParams& params) {
  params.setLinearSolverType("MULTIFRONTAL_SOLVER");
  // Independent 3D point blocks followed by a bounded pose separator. Keep
  // those small blocks intact and avoid worker creation inside the SLAM thread.
  params.multifrontalParams.leafMergeDimCap = 0;
  params.multifrontalParams.mergeDimCap = 0;
  params.multifrontalParams.numThreads = 1;
  // One worker gains nothing by queuing hundreds of tiny independent cliques.
  params.multifrontalParams.eliminationParallelThreshold = std::numeric_limits<int>::max();
  params.multifrontalParams.solutionParallelThreshold = std::numeric_limits<int>::max();
  params.multifrontalParams.qrMode = gtsam::MultifrontalParameters::QRMode::Off;
  // LM damping/stopping rules are unchanged. This controls only the damped
  // update solve; final uncertainty uses undamped equations and retains QR
  // fallback for weak camera systems.
}

template <int D, int... Dims>
bool PointSchurSolver::add(const gtsam::GaussianFactor& f, const Plan& plan) {
  const auto* batch = dynamic_cast<const gtsam::BatchJacobianFactor<D, Dims...>*>(&f);
  if (!batch) return false;
  if (batch->get_model() && !batch->get_model()->isUnit()) throw std::runtime_error("requires whitened rows");
  constexpr int N = sizeof...(Dims);
  auto& point = points_[plan.point];
  const auto& J = batch->rowBlocks();
  const auto& rhs = batch->rowRhs();
  for (size_t row = 0; row < rhs.size(); ++row) {
    const auto& jp = std::get<N - 1>(J)[row];
    point.A.noalias() += jp.transpose() * jp;
    point.b.noalias() += jp.transpose() * rhs[row];
    if constexpr (N == 2) {
      const int local = plan.camera_slots[batch->rowSlots()[row][0]];
      auto& camera = cameras_[point.cameras[local]];
      const auto& jc = std::get<0>(J)[row];
      camera.A.noalias() += jc.transpose() * jc;
      camera.b.noalias() += jc.transpose() * rhs[row];
      point.B[local].noalias() += jp.transpose() * jc;
    }
  }
  return true;
}

void PointSchurSolver::initialize(const gtsam::GaussianFactorGraph& graph) {
  std::map<gtsam::Key, std::set<gtsam::Key>> topology;
  std::set<gtsam::Key> cameras;
  for (const auto& f : graph) {
    gtsam::Key point = 0;
    bool found = false;
    for (auto k : f->keys())
      if (f->getDim(f->find(k)) == 3) {
        if (found) throw std::runtime_error("multiple points");
        point = k;
        found = true;
      }
    if (!found) throw std::runtime_error("missing point");
    auto& linked = topology[point];
    for (auto k : f->keys())
      if (k != point) {
        if (f->getDim(f->find(k)) != 6) throw std::runtime_error("not camera");
        linked.insert(k);
        cameras.insert(k);
      }
  }
  std::map<gtsam::Key, int> ci, pi;
  for (auto k : cameras) {
    ci[k] = cameras_.size();
    Camera c;
    c.key = k;
    cameras_.push_back(c);
  }
  for (const auto& [k, linked] : topology) {
    Point p;
    p.key = k;
    for (auto c : linked) p.cameras.push_back(ci.at(c));
    p.B.resize(linked.size());
    p.solved_B.resize(linked.size());
    pi[k] = points_.size();
    points_.push_back(std::move(p));
  }
  for (const auto& f : graph) {
    gtsam::Key point = 0;
    for (auto k : f->keys())
      if (pi.count(k)) point = k;
    Plan plan;
    plan.point = pi.at(point);
    const auto& p = points_[plan.point];
    for (auto k : f->keys())
      plan.camera_slots.push_back(k == point ? -1 : int(std::lower_bound(p.cameras.begin(), p.cameras.end(), ci.at(k)) - p.cameras.begin()));
    plans_.push_back(std::move(plan));
  }
  S_.resize(cameras_.size() * 6, cameras_.size() * 6);
  b_.resize(S_.rows());
  dx_.resize(S_.rows());
}

void PointSchurSolver::assemble(const gtsam::GaussianFactorGraph& graph) {
  if (plans_.empty()) initialize(graph);
  if (graph.size() != plans_.size()) throw std::runtime_error("structure changed");
  for (auto& c : cameras_) {
    c.A.setZero();
    c.b.setZero();
  }
  for (auto& p : points_) {
    p.A.setZero();
    p.b.setZero();
    for (auto& B : p.B) B.setZero();
  }
  for (size_t i = 0; i < graph.size(); ++i)
    if (!add<2, 6, 3>(*graph[i], plans_[i]) && !add<3, 6, 3>(*graph[i], plans_[i]) && !add<2, 3>(*graph[i], plans_[i]) && !add<3, 3>(*graph[i], plans_[i]))
      throw std::runtime_error("unsupported compact factor");
}

gtsam::VectorValues PointSchurSolver::solve(double lambda) {
  solveSchurComplement(lambda);
  return solveBackSubstitution();
}

void PointSchurSolver::solveSchurComplement(double lambda) {
  S_.setZero();
  for (size_t i = 0; i < cameras_.size(); ++i) {
    S_.block<6, 6>(6 * i, 6 * i) = cameras_[i].A;
    b_.segment<6>(6 * i) = cameras_[i].b;
  }
  S_.diagonal().array() += lambda;
  for (auto& p : points_) {
    Mat33_t<double> A = p.A;
    A.diagonal().array() += lambda;
    Eigen::LLT<Mat33_t<double>> llt(A);
    if (llt.info() != Eigen::Success) throw gtsam::IndeterminantLinearSystemException(p.key);
    p.solved_b = llt.solve(p.b);
    for (size_t i = 0; i < p.B.size(); ++i) p.solved_B[i] = llt.solve(p.B[i]);
    for (size_t i = 0; i < p.B.size(); ++i) {
      const int ci = 6 * p.cameras[i];
      b_.segment<6>(ci).noalias() -= p.B[i].transpose() * p.solved_b;
      for (size_t j = i; j < p.B.size(); ++j) S_.block<6, 6>(ci, 6 * p.cameras[j]).noalias() -= p.B[i].transpose().lazyProduct(p.solved_B[j]);
    }
  }
  if (S_.rows()) {
    Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>, Eigen::Upper> llt(S_);
    if (llt.info() != Eigen::Success) throw gtsam::IndeterminantLinearSystemException(cameras_.front().key);
    dx_ = llt.solve(b_);
  }
}

gtsam::VectorValues PointSchurSolver::solveBackSubstitution() const {
  gtsam::VectorValues delta;
  for (size_t i = 0; i < cameras_.size(); ++i) delta.insert(cameras_[i].key, dx_.segment<6>(6 * i));
  for (const auto& p : points_) {
    Vec3_t<double> dp = p.solved_b;
    for (size_t i = 0; i < p.B.size(); ++i) dp.noalias() -= p.solved_B[i] * dx_.segment<6>(6 * p.cameras[i]);
    delta.insert(p.key, dp);
  }
  return delta;
}
std::map<uint64_t, Eigen::MatrixXd> bundle_covariances(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, CovarianceProfile* profile,
                                                       const gtsam::GaussianFactorGraph* final_linear) {
  auto profile_start = std::chrono::steady_clock::time_point{};
  if (profile) {
    *profile = {};
    profile_start = std::chrono::steady_clock::now();
  }
  const auto profile_phase = [&] {
    const auto now = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(now - profile_start).count();
    profile_start = now;
    return ms;
  };
  gtsam::Ordering points, poses;
  std::map<gtsam::Key, int> offsets;
  int dimension = 0;
  for (const auto key : graph.keys()) {
    if (gtsam::Symbol(key).chr() == 'l')
      points.push_back(key);
    else {
      poses.push_back(key);
      offsets.emplace(key, dimension);
      dimension += int(values.at(key).dim());
    }
  }
  for (const auto& factor : graph)
    if (factor) {
      size_t point_count = 0;
      for (const auto key : factor->keys()) point_count += gtsam::Symbol(key).chr() == 'l';
      if (point_count > 1) throw std::invalid_argument("Local covariance requires independent point blocks");
    }
  const auto owned_linear = final_linear ? nullptr : graph.linearize(values);
  const auto* linear = final_linear ? final_linear : owned_linear.get();
  if (profile) profile->linearize_ms = profile_phase();
  // QR leaves each point as R*l + S*p = d + epsilon. Only the remaining
  // pose system is dense (at most the active local-BA pose budget).
  // Point blocks are independent: partition once instead of rebuilding a
  // general variable index/elimination tree for every covariance query.
  std::map<gtsam::Key, gtsam::GaussianFactorGraph> point_factors;
  gtsam::GaussianFactorGraph pose_factors;
  for (const auto& factor : *linear)
    if (factor) {
      std::optional<gtsam::Key> point;
      for (const auto key : factor->keys())
        if (gtsam::Symbol(key).chr() == 'l') point = key;
      if (point)
        point_factors[*point].push_back(factor);
      else
        pose_factors.push_back(factor);
    }
  Eigen::MatrixXd pose_covariance = Eigen::MatrixXd::Zero(dimension, dimension);
  const auto inverse_root = [](const auto& root, gtsam::Key key) -> typename std::decay_t<decltype(root)>::PlainObject {
    using Matrix = typename std::decay_t<decltype(root)>::PlainObject;
    if (root.rows() != root.cols()) throw std::logic_error("Invalid covariance square root shape");
    if (!root.allFinite()) throw gtsam::IndeterminantLinearSystemException(key);
    if (root.rows() == 0) return root;
    const double tolerance = 64 * std::numeric_limits<double>::epsilon() * root.cwiseAbs().maxCoeff();
    if ((root.diagonal().cwiseAbs().array() <= tolerance).any()) throw gtsam::IndeterminantLinearSystemException(key);
    Matrix inverse = root.template triangularView<Eigen::Upper>().solve(Matrix::Identity(root.rows(), root.cols()));
    if (!inverse.allFinite()) throw gtsam::IndeterminantLinearSystemException(key);
    return inverse;
  };
  struct PointConditional {
    gtsam::Key key;
    Mat33_t<double> covariance;
    Eigen::Matrix<double, 3, Eigen::Dynamic> coupling;
  };
  std::vector<PointConditional> point_conditionals;
  point_conditionals.reserve(points.size());
  const auto point_joint = [&](gtsam::Key point) {
    const auto& factors = point_factors.at(point);
    size_t rows = 0;
    for (const auto& f : factors) {
      const auto* batch = dynamic_cast<const gtsam::BatchJacobianFactorBase*>(f.get());
      if (!batch || (batch->get_model() && !batch->get_model()->isUnit())) return gtsam::JacobianFactor(factors, gtsam::Ordering{point});
      rows += batch->rows();
    }
    // Scatter compact whitened rows once. The generic combine constructor
    // first expands every batch, then copies those dense matrices again.
    const auto all_keys = factors.keys();
    gtsam::KeyVector keys{point};
    for (const auto key : all_keys)
      if (key != point) keys.push_back(key);
    std::vector<size_t> dims;
    for (const auto key : keys) dims.push_back(values.at(key).dim());
    gtsam::VerticalBlockMatrix matrix(dims, rows, true);
    matrix.matrix().setZero();
    size_t row = 0;
    for (const auto& f : factors) {
      const auto& batch = static_cast<const gtsam::BatchJacobianFactorBase&>(*f);
      std::vector<gtsam::DenseIndex> slots;
      for (const auto key : batch.keys()) slots.push_back(key == point ? 0 : std::lower_bound(keys.begin() + 1, keys.end(), key) - keys.begin());
      row += batch.scatterInto(matrix, row, slots);
    }
    return gtsam::JacobianFactor(keys, std::move(matrix), nullptr);
  };
  for (const auto key : points) {
    // GTSAM supplies the whitened Jacobians (including each robust weight).
    // Only eliminate the three point columns here. EliminateQR also performs
    // QR on this point's camera columns, which will be eliminated again below.
    gtsam::JacobianFactor joint = point_joint(key);
    if (joint.get_model() && joint.get_model()->isConstrained()) throw std::invalid_argument("Constrained local covariance requires general QR");
    if (joint.get_model() && !joint.get_model()->isUnit()) joint = joint.whiten();
    // Borrow the whitened Jacobian. Covariance does not use the RHS, and
    // augmentedJacobian() needlessly copies both the Jacobian and RHS here.
    const auto ab = joint.getA();
    if (joint.getDim(joint.begin()) != 3) throw std::logic_error("Invalid local point dimension");
    if (ab.rows() < 3) throw gtsam::IndeterminantLinearSystemException(key);
    Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, 3>> qr(ab.leftCols(3));
    const Mat33_t<double> root = qr.matrixQR().topRows<3>().triangularView<Eigen::Upper>();
    const Mat33_t<double> inverse = inverse_root(root, key);
    const int camera_dimension = int(ab.cols()) - 3;
    const Eigen::MatrixXd transformed = qr.householderQ().adjoint() * ab.middleCols(3, camera_dimension);
    PointConditional conditional{key, inverse * inverse.transpose(), Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, dimension)};
    gtsam::KeyVector remaining;
    std::vector<size_t> remaining_dimensions;
    int local_offset = 0;
    for (auto it = std::next(joint.begin()); it != joint.end(); ++it) {
      const int d = int(joint.getDim(it));
      conditional.coupling.middleCols(offsets.at(*it), d).noalias() = inverse * transformed.block(0, local_offset, 3, d);
      remaining.push_back(*it);
      remaining_dimensions.push_back(d);
      local_offset += d;
    }
    // Marginals depend on J, not the residual RHS. Keep the reduced square
    // root instead of forming/subtracting normal equations at weak depths.
    if (!remaining.empty() && ab.rows() > 3) {
      gtsam::VerticalBlockMatrix rows(remaining_dimensions, ab.rows() - 3, true);
      rows.matrix().leftCols(camera_dimension) = transformed.bottomRows(ab.rows() - 3);
      rows.matrix().col(camera_dimension).setZero();
      pose_factors.emplace_shared<gtsam::JacobianFactor>(remaining, std::move(rows), nullptr);
    }
    point_conditionals.push_back(std::move(conditional));
  }
  if (profile) profile->point_elimination_ms = profile_phase();
  if (dimension) {
    // Accumulate the already reduced square-root rows into the small camera
    // system. Unlike Hpp - Hpl*Hll^-1*Hlp, this does not subtract nearly equal
    // normal-equation blocks when a point has weak depth. GTSAM performs the
    // key/block scattering; no tall camera QR or general elimination tree is
    // needed for a well-conditioned local system.
    const gtsam::HessianFactor camera_system(pose_factors, gtsam::Scatter(pose_factors, poses));
    const Eigen::MatrixXd information = camera_system.information();
    if (information.rows() != dimension) throw std::logic_error("Invalid local camera dimension");
    if (!information.allFinite()) throw gtsam::IndeterminantLinearSystemException(poses.front());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(information, Eigen::EigenvaluesOnly);
    if (profile && spectrum.info() == Eigen::Success && spectrum.eigenvalues().maxCoeff() > 0)
      profile->camera_information_ratio = spectrum.eigenvalues().minCoeff() / spectrum.eigenvalues().maxCoeff();
    bool solved = false;
    // Forming J^T J squares the condition number. Retain the square-root
    // route for weak systems instead of regularizing away their uncertainty.
    if (spectrum.info() == Eigen::Success && spectrum.eigenvalues().allFinite() &&
        spectrum.eigenvalues().minCoeff() > 1e-8 * spectrum.eigenvalues().maxCoeff()) {
      const Eigen::LLT<Eigen::MatrixXd> factor(information);
      if (factor.info() == Eigen::Success) {
        pose_covariance = factor.solve(Eigen::MatrixXd::Identity(dimension, dimension));
        solved = pose_covariance.allFinite();
      }
    }
    if (!solved) {
      if (profile) profile->camera_qr = true;
      const auto net = pose_factors.eliminateSequential(poses, gtsam::EliminateQR);
      const Eigen::MatrixXd root = net->matrix(poses).first;
      const Eigen::MatrixXd inverse = inverse_root(root, poses.front());
      pose_covariance.noalias() = inverse * inverse.transpose();
    }
  }
  if (profile) profile->camera_ms = profile_phase();
  std::map<uint64_t, Eigen::MatrixXd> result;
  for (const auto key : poses) {
    const int d = int(values.at(key).dim()), offset = offsets.at(key);
    result.emplace(key, pose_covariance.block(offset, offset, d, d));
  }
  for (const auto& conditional : point_conditionals) {
    Mat33_t<double> covariance = conditional.covariance;
    // Law of total covariance: conditional point uncertainty PLUS uncertain
    // camera contribution, including cross-correlation between camera poses.
    covariance.noalias() += conditional.coupling * pose_covariance * conditional.coupling.transpose();
    result.emplace(conditional.key, .5 * (covariance + covariance.transpose()).eval());
  }
  if (profile) profile->recovery_ms = profile_phase();
  return result;
}

static double elapsed(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}
BASolver::BASolver()
    : values_(std::make_unique<gtsam::Values>()),
      seed_(std::make_unique<gtsam::Values>()),
      graph_(std::make_unique<gtsam::NonlinearFactorGraph>()),
      params_(std::make_unique<gtsam::LevenbergMarquardtParams>()) {}
BASolver::~BASolver() = default;
BASolver::Result BASolver::solve(const Problem& problem, const std::vector<Observation>& observations, const Options& options, BADiagnostics* diagnostics) {
  // Release borrowed factors on every exit, including a system exception.
  struct CallGraph {
    gtsam::NonlinearFactorGraph& graph;
    std::shared_ptr<gtsam::GaussianFactorGraph>& linear;
    ~CallGraph() { linear.reset(); graph.resize(0); }
  } call_graph{*graph_, linear_};
  Result result;
  const bool reference_solver = diagnostics && diagnostics->reference_solver;
  const bool reference_marginals = diagnostics && diagnostics->reference_marginals;
  using Clock = std::chrono::steady_clock;
  const auto start = diagnostics ? Clock::now() : Clock::time_point{};
  WorkspaceProfile workspace_profile;
  if (diagnostics) {
    diagnostics->setup_ms = diagnostics->factors_ms = diagnostics->optimize_ms = 0;
    diagnostics->uncertainty_ms = diagnostics->check_ms = diagnostics->elapsed_ms = 0;
  }
  active_.clear();
  active_poses_.clear();
  values_->clear();
  seed_->clear();
  graph_->resize(0);
  linear_.reset();
  result.poses.reserve(problem.poses.size());
  for (const auto& p : problem.poses) result.poses.push_back(p.initial);
  result.points.reserve(problem.points.size());
  for (const auto& p : problem.points) result.points.push_back(p.initial);
  result.pose_covariances.resize(problem.poses.size());
  result.point_covariances.resize(problem.points.size(), Mat33_t<double>::Zero());
  result.active_poses.resize(problem.poses.size(), false);
  result.active_points.resize(problem.points.size(), true);
  result.active_observations.resize(observations.size(), true);
  Status failure_status = Status::InvalidProblem;
  try {
    if (problem.points.empty() || options.max_iterations <= 0) throw std::invalid_argument("Empty or invalid BA problem");
    for (size_t i = 0; i < problem.poses.size(); ++i) {
      const auto& p = problem.poses[i];
      if (!p.initial.isValid() || !p.initial.matrix().allFinite()) throw std::invalid_argument("Invalid BA pose");
      if (!p.fixed) values_->insert(gtsam::Symbol('v', i), to_pose3(p.initial));
    }
    size_t next = 0;
    for (size_t i = 0; i < problem.points.size(); ++i) {
      const auto& p = problem.points[i];
      if (p.history) p.history->validate();
      if (p.source) {
        p.source->history.validate();
        if (!p.history || p.source->pose >= problem.poses.size() || problem.poses[p.source->pose].fixed)
          throw std::invalid_argument("Invalid source summary pose or missing target summary");
      }
      if (p.begin != next || p.end > observations.size() || p.end < p.begin || !p.minimum || p.minimum > p.end - p.begin + bool(p.source) ||
          (p.required && (*p.required < p.begin || *p.required >= p.end)))
        throw std::invalid_argument("Invalid BA observation range");
      next = p.end;
      const gtsam::Point3& xyz = p.initial;
      if (!xyz.allFinite()) throw std::invalid_argument("Invalid BA point");
      values_->insert(gtsam::Symbol('l', i), xyz);
      active_.push_back(i);
    }
    if (next != observations.size()) throw std::invalid_argument("Unassigned BA observations");
    for (const auto& o : observations) {
      const auto& m = o.measurement.get();
      if (o.pose >= problem.poses.size() || !m.valid() || !std::isfinite(m.sigma()) || !m.anchor_from_camera().isValid() ||
          !m.anchor_from_camera().matrix().allFinite() || !std::isfinite(m.pixel().x) || !std::isfinite(m.pixel().y) || !std::isfinite(m.pixel().z) ||
          !std::all_of(m.calibration().begin(), m.calibration().end(), [](float x) { return std::isfinite(x); }) ||
          m.calibration()[0] <= 0 || m.calibration()[1] <= 0 || m.calibration()[4] < 0)
        throw std::invalid_argument("Invalid BA measurement");
    }
    if (diagnostics) diagnostics->setup_ms = elapsed(start);
    failure_status = Status::NumericalFailure;
    std::unique_ptr<WorkspaceOptimizer<BundleBoundaryFactor>> previous;
    bool converged = false;
    for (int pass = 0; pass < 3; ++pass) {
      const auto factor_start = diagnostics ? Clock::now() : Clock::time_point{};
      const auto solve_graph = updateFactors(problem, observations, options, result, pass == 0, reference_solver);
      if (diagnostics) diagnostics->factors_ms += elapsed(factor_start);
      const auto optimize_start = diagnostics ? Clock::now() : Clock::time_point{};
      auto optimizer = std::make_unique<WorkspaceOptimizer<BundleBoundaryFactor>>(solve_graph, *seed_, *params_, !reference_solver,
                                                                                diagnostics ? &workspace_profile : nullptr);
      if (previous) optimizer->reuseStructure(*previous);
      previous.reset();
      *values_ = optimizer->optimize();
      const double pass_ms = diagnostics ? elapsed(optimize_start) : 0;
      if (diagnostics) diagnostics->optimize_ms += pass_ms;
      if (diagnostics && diagnostics->inspect_problem) inspectProblem(problem, observations, result, *diagnostics, pass, optimizer->iterations(), pass_ms);
      const auto check_start = diagnostics ? Clock::now() : Clock::time_point{};
      const bool stable = updateStates(problem, observations, options, result, optimizer->iterations());
      if (diagnostics) diagnostics->check_ms += elapsed(check_start);
      if (pass > 0 && stable) {
        if (!reference_marginals) linear_ = optimizer->linearize();
        converged = true;
        break;
      }
      previous = std::move(optimizer);
    }
    if (!converged) {
      throw Status::NotConverged;
    }
    const auto covariance_start = diagnostics ? Clock::now() : Clock::time_point{};
    failure_status = Status::Unobservable;
    computeCovariances(problem, result, reference_marginals);
    result.status = Status::Success;
    if (diagnostics) diagnostics->uncertainty_ms += elapsed(covariance_start);
  } catch (Status status) {
    result = {};
    result.status = status;
    result.message = status == Status::InsufficientSupport ? "Insufficient consistent BA tracks" : "BA did not converge within budget";
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    result = {};
    result.status = failure_status;
    result.message = e.what();
  } catch (const std::invalid_argument& e) {
    // Only validation errors at entry are API-level InvalidProblem results.
    // A malformed internal factor or a diagnostic exception is a system fault.
    if (failure_status != Status::InvalidProblem) throw;
    result = {};
    result.status = Status::InvalidProblem;
    result.message = e.what();
  }
  if (diagnostics) {
    diagnostics->elapsed_ms = elapsed(start);
    diagnostics->linearize_ms = workspace_profile.linearize_ms;
    diagnostics->error_ms = workspace_profile.error_ms;
    diagnostics->assemble_ms = workspace_profile.assemble_ms;
    diagnostics->solve_ms = workspace_profile.solve_ms;
  }
  return result;
}
gtsam::NonlinearFactorGraph BASolver::updateFactors(const Problem& problem, const std::vector<Observation>& observations, const Options& options, Result& result, bool robust, bool reference_solver) {
  graph_->resize(0);
  seed_->clear();
  active_poses_.clear();
  std::fill(result.active_poses.begin(), result.active_poses.end(), false);
  PointBatchBuilder<BundleBoundaryFactor> builder;
  BundleFactors data(robust);
  gtsam::Ordering order;
  size_t boundary_tracks = 0;
  for (size_t i : active_) {
    const auto& p = problem.points[i];
    const auto key = gtsam::Symbol('l', i);
    seed_->insert(key, values_->at<gtsam::Point3>(key));
    order.push_back(key);
    bool anchored = p.history.has_value();
    if (p.history) {
      if (reference_solver)
        graph_->emplace_shared<BundleBoundaryFactor>(*p.history, key);
      else
        builder.boundary(key, p.end - p.begin + 1, false, *p.history, key);
    }
    if (p.source) {
      const auto camera_key = gtsam::Symbol('v', p.source->pose);
      result.active_poses[p.source->pose] = true;
      if (active_poses_.insert(p.source->pose).second) seed_->insert(camera_key, values_->at<gtsam::Pose3>(camera_key));
      if (reference_solver)
        graph_->emplace_shared<BundleSummaryFactor>(p.source->history, camera_key, key, robust);
      else
        builder.summary(key, p.source->history, camera_key, key, robust);
    }
    for (size_t row = p.begin; row < p.end; ++row) {
      const auto& o = observations[row];
      if (!result.active_observations[row]) continue;
      auto& pose = problem.poses[o.pose];
      result.active_poses[o.pose] = true;
      if (pose.fixed) {
        data.addBoundary(*graph_, builder, reference_solver, o.measurement.get(), key, p.end - p.begin, pose.initial);
        anchored = true;
      } else {
        const auto camera_key = gtsam::Symbol('v', o.pose);
        if (active_poses_.insert(o.pose).second) seed_->insert(camera_key, values_->at<gtsam::Pose3>(camera_key));
        data.addPixel(*graph_, builder, reference_solver, o.measurement.get(), key, p.end - p.begin, camera_key);
      }
    }
    boundary_tracks += anchored;
  }
  if (boundary_tracks < options.minimum_fixed_tracks) throw Status::InsufficientSupport;
  for (size_t i : active_poses_) order.push_back(gtsam::Symbol('v', i));
  *params_ = gtsam::LevenbergMarquardtParams{};
  params_->setOrdering(order);
  params_->setMaxIterations(options.max_iterations);
  params_->setRelativeErrorTol(1e-6);
  params_->setAbsoluteErrorTol(1e-6);
  if (reference_solver) return *graph_;
  auto graphs = std::move(builder).build();
  *graph_ = std::move(graphs.original);
  configure_point_batch_solver(*params_);
  return std::move(graphs.batched);
}
bool BASolver::updateStates(const Problem& problem, const std::vector<Observation>& observations, const Options& options, Result& result, size_t iterations) {
  const double error = graph_->error(*values_);
  if (iterations >= params_->maxIterations || !std::isfinite(error) || error > graph_->error(*seed_) + 1e-8) throw Status::NotConverged;
  for (size_t i : active_poses_) {
    const auto& p = values_->at<gtsam::Pose3>(gtsam::Symbol('v', i));
    result.poses[i] = stateFromPose3(p);
  }
  BundleProjection checks;
  bool stable = true;
  size_t required = 0, retained = 0;
  for (size_t i : active_) {
    auto& p = problem.points[i];
    const auto& xyz = values_->at<gtsam::Point3>(gtsam::Symbol('l', i));
    size_t count = 0;
    double historical_error = 0;
    if (p.history && !p.source) {
      try { historical_error = p.history->evaluate(xyz).squaredNorm(); }
      catch (const std::domain_error&) { historical_error = std::numeric_limits<double>::infinity(); }
    }
    for (size_t row = p.begin; row < p.end; ++row) {
      auto& o = observations[row];
      if (!result.active_observations[row]) continue;
      if (!checks.inlier(o.measurement.get(), result.poses[o.pose], xyz, historical_error)) {
        result.active_observations[row] = false;
        result.rejected_observations.push_back(row);
        stable = false;
      } else
        ++count;
    }
    bool source_inlier = true;
    if (p.source) {
      const auto local = values_->at<gtsam::Pose3>(gtsam::Symbol('v', p.source->pose)).transformTo(xyz);
      try {
        // Test the NEW association, not either already committed history in
        // isolation. A point can fit the source by moving far from the target;
        // both summaries must contribute to this joint compatibility residual.
        const double source_error = p.source->history.evaluate(local).squaredNorm() + p.history->evaluate(xyz).squaredNorm();
        source_inlier = std::isfinite(source_error) && source_error <= 7.81473;
      } catch (const std::domain_error&) { source_inlier = false; }
      count += source_inlier;
    }
    result.active_points[i] = count >= p.minimum && source_inlier && (!p.required || result.active_observations[*p.required]);
    if (result.active_points[i]) {
      active_[retained++] = i;
      required += p.required.has_value() || p.source.has_value();
    } else
      stable = false;
  }
  active_.resize(retained);
  if (active_.empty() || required < options.minimum_required_points) throw Status::InsufficientSupport;
  return stable;
}
void BASolver::computeCovariances(const Problem& problem, Result& result, bool reference_marginals) {
  std::map<uint64_t, Eigen::MatrixXd> covariances;
  std::unique_ptr<gtsam::Marginals> reference;
  if (reference_marginals)
    reference = std::make_unique<gtsam::Marginals>(*graph_, *values_, gtsam::Marginals::QR);
  else
    covariances = bundle_covariances(*graph_, *values_, nullptr, linear_.get());
  const auto marginal = [&](gtsam::Key key) -> Eigen::MatrixXd { return reference ? reference->marginalCovariance(key) : covariances.at(key); };
  for (size_t i : active_poses_) {
    auto& p = problem.poses[i];
    if (!p.marginal) continue;
    const auto cov = marginal(gtsam::Symbol('v', i));
    if (!cov.allFinite() || Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(cov).eigenvalues().minCoeff() <= 0) throw gtsam::IndeterminantLinearSystemException(gtsam::Symbol('v', i));
    result.pose_covariances[i].emplace();
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) result.pose_covariances[i]->matrix(r, c) = cov((r + 3) % 6, (c + 3) % 6);
  }
  for (size_t i : active_) {
    const auto key = gtsam::Symbol('l', i);
    const auto& xyz = values_->at<gtsam::Point3>(key);
    result.points[i] = xyz;
    result.point_covariances[i] = marginal(key);
  }
}
void BASolver::inspectProblem(const Problem& problem, const std::vector<Observation>& observations, const Result& result, BADiagnostics& diagnostics, int pass, size_t iterations, double optimize_ms) const {
  BASnapshot snapshot;
  snapshot.pass = pass;
  snapshot.robust = pass == 0;
  snapshot.iterations = iterations;
  snapshot.optimize_ms = optimize_ms;
  snapshot.initial_error = graph_->error(*seed_);
  snapshot.solved_error = graph_->error(*values_);
  for (size_t i : active_) {
    const auto& p = problem.points[i];
    const auto key = gtsam::Symbol('l', i);
    snapshot.points.emplace(p.id, BASnapshot::Point{seed_->at<gtsam::Point3>(key), values_->at<gtsam::Point3>(key)});
    if (p.history) snapshot.summaries.push_back({p.id, {}, *p.history});
    if (p.source) {
      const auto& pose = problem.poses[p.source->pose];
      const auto pk = gtsam::Symbol('v', p.source->pose);
      snapshot.poses.emplace(pose.id, BASnapshot::Pose{seed_->at<gtsam::Pose3>(pk).matrix(), values_->at<gtsam::Pose3>(pk).matrix(), false});
      snapshot.summaries.push_back({p.id, pose.id, p.source->history});
    }
    for (size_t row = p.begin; row < p.end; ++row) {
      const auto& o = observations[row];
      if (!result.active_observations[row]) continue;
      const auto& pose = problem.poses[o.pose];
      const auto pk = gtsam::Symbol('v', o.pose);
      if (!snapshot.poses.count(pose.id)) {
        const auto a = pose.fixed ? to_pose3(pose.initial) : seed_->at<gtsam::Pose3>(pk);
        const auto b = pose.fixed ? a : values_->at<gtsam::Pose3>(pk);
        snapshot.poses.emplace(pose.id, BASnapshot::Pose{a.matrix(), b.matrix(), pose.fixed});
      }
      snapshot.observations.push_back({p.id, pose.id, o.measurement.get()});
    }
  }
  diagnostics.inspect_problem(snapshot);
}

}  // namespace sapphire::backend::solver
