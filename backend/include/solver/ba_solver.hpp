#pragma once
#include <backend/common/common_type.hpp>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "backend/common/state_group.hpp"
#include "backend/common/visual_frame.hpp"
#include "backend/maplayer/topograph/link.hpp"
#include "backend/common/stereo_frame.hpp"
#include "backend/maplayer/featmap/point_summary.hpp"
#include "backend/sensor/camera/perspective.hpp"

namespace gtsam {
class Values;
class NonlinearFactorGraph;
class GaussianFactorGraph;
class LevenbergMarquardtParams;
}  // namespace gtsam

namespace sapphire::backend::solver {
struct PoseResult {
  StateGroupf pose;
  std::optional<Mat66_t<double>> covariance;        // right perturbation, translation then rotation
  std::vector<int> inliers;  // indices into input matches
  size_t available = 0;
  double elapsed_ms = 0;
  std::string failure;
};

struct PnPMatch {
  cv::Point3f source_point;
  PixelObservation target;
};

struct BADiagnostics;  // Opaque optional instrumentation, defined only in a private header.
class BASolver {
 public:
  using Index = size_t;
  // Labels belong to the caller; array indices are always Index.
  using PoseId = int;
  using PointId = uint64_t;
  // T' = T Exp(delta). This type always uses [translation, rotation].
  struct RightPoseCovariance {
    enum Offset { Translation = 0, Rotation = 3 };
    Mat66_t<double> matrix = Mat66_t<double>::Zero();
  };
  enum class Status { Success, InvalidProblem, InsufficientSupport, NotConverged, Unobservable, NumericalFailure };
  struct Pose {
    PoseId id = 0;
    StateGroupf initial;
    bool fixed = false;
    bool marginal = false;
  };
  struct Point {
    struct Source {
      Index pose = 0;
      featmap::PointSummary history;
    };
    PointId id = 0;
    Vec3_t<double> initial = Vec3_t<double>::Zero();
    Index begin = 0, end = 0;
    std::optional<Index> required;
    Index minimum = 1;
    // Accepted pixel history at fixed capture poses. Never reclassified as
    // individual outliers; only the new observations below are checked.
    std::optional<featmap::PointSummary> history;
    // A new association with an independently owned Node. Its internal
    // geometry is fixed; this Node's single relative pose remains variable.
    std::optional<Source> source;
  };
  struct Observation {
    Index pose;
    std::reference_wrapper<const camera::StereoObservation> measurement;
  };
  // Initial values and topology only. No borrowed measurement survives in Problem.
  // Point ranges partition the observation view supplied to solve().
  struct Problem {
    std::vector<Pose> poses;
    std::vector<Point> points;
  };
  struct Options {
    Index minimum_required_points = 0, minimum_fixed_tracks = 0;
    int max_iterations = 20;
  };
  struct Result {
    Status status = Status::InvalidProblem;
    std::string message;  // Diagnostic only; branch on status.
    // Arrays use the same indices as Problem. All are empty on failure.
    std::vector<StateGroupf> poses;
    std::vector<Vec3_t<double>> points;
    std::vector<std::optional<RightPoseCovariance>> pose_covariances;
    std::vector<Mat33_t<double>> point_covariances;
    std::vector<uint8_t> active_poses, active_points, active_observations;
    std::vector<Index> rejected_observations;
  };

  BASolver();
  ~BASolver();
  BASolver(const BASolver&) = delete;
  BASolver& operator=(const BASolver&) = delete;
  // Synchronous borrow: observation sources must remain alive and unmodified for this call.
  // reference_wrapper is non-owning: it does not keep its source alive or survive source-vector reallocation.
  // Build references only after source storage is fixed, and do not grow it during the call.
  // No input references or factors are retained after return.
  // Result owns its data and remains valid after the inputs are destroyed.
  // Each call starts from Problem::initial, including after a failed solve. Reuse is serial.
  // Expected geometric degeneracy returns Status. Allocation, internal-invariant and
  // instrumentation errors propagate to the task failure boundary.
  // No build/reset phase is needed: scratch state is reset at entry on every call.
  [[nodiscard]] Result solve(const Problem&, const std::vector<Observation>&, const Options&, BADiagnostics* diagnostics = nullptr);

  // Standalone full-bundle solver options; independent of online local BA and PGO tuning.
  struct BundleOptions {
    int max_iterations = 20;
    double min_error_improvement = 0;
    double pixel_variance_px2 = 1;
    double robust_kernel_delta = 8;
    bool ignore_constraint_variance = false;
    float gravity_sigma = 0.3f;
  };
  static std::map<int, StateGroupf> optimize_bundle(const BundleOptions&, int rootId, const std::map<int, StateGroupf>& poses,
                                                    const std::multimap<int, Link>& links, const std::map<int, std::vector<camera::Perspective>>& models,
                                                    std::map<int, cv::Point3f>& points, const FeatureObservations& observations,
                                                    RejectedObservations* outliers = nullptr);

  struct PnPOptions {
    int min_inliers = 6, iterations = 100, refine_iterations = 5;
    double reprojection_error = 2;
    // Empty retains the production clock-seeded policy. An explicit seed is repeatable per call.
    std::optional<uint64_t> seed;
  };
  struct PnPResult {
    Status status = Status::InvalidProblem;
    std::string message;
    StateGroupf pose;
    std::vector<int> matches, inliers;  // Indices into input matches.
    std::vector<int> inliers_per_camera;
  };
  [[nodiscard]] static PnPResult estimate_motion_pnp(const std::vector<PnPMatch>&, const camera::Perspective&, const StateGroupf& initial, const PnPOptions&);
  [[nodiscard]] static PnPResult estimate_motion_pnp(const std::vector<PnPMatch>&, const std::vector<camera::Perspective>&, const StateGroupf& initial,
                                                     const PnPOptions&);

  static std::optional<RightPoseCovariance> compute_covariance(const std::vector<PnPMatch>& observations, const std::vector<int>& inliers,
                                                               const std::vector<camera::Perspective>& cameras, const StateGroupf& source_from_query);


 private:
  static void solvePnPRansac(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const camera::Perspective& perspective,
                             cv::Mat& rvec, cv::Mat& tvec, std::vector<int>& inliers, const PnPOptions&);

  gtsam::NonlinearFactorGraph updateFactors(const Problem&, const std::vector<Observation>&, const Options&, Result&, bool robust, bool reference_solver);
  bool updateStates(const Problem&, const std::vector<Observation>&, const Options&, Result&, size_t iterations);
  void computeCovariances(const Problem&, Result&, bool reference_marginals);
  void inspectProblem(const Problem&, const std::vector<Observation>&, const Result&, BADiagnostics&, int pass, size_t iterations, double optimize_ms) const;
  std::vector<size_t> active_;
  std::set<size_t> active_poses_;
  std::unique_ptr<gtsam::Values> values_, seed_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph_;
  std::shared_ptr<gtsam::GaussianFactorGraph> linear_;
  std::unique_ptr<gtsam::LevenbergMarquardtParams> params_;
};

}  // namespace sapphire::backend::solver
