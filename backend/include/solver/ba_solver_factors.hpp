#pragma once
#include <backend/common/common_type.hpp>
#include <backend/maplayer/featmap/point_summary.hpp>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <backend/common/stereo_frame.hpp>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "backend/sensor/camera/kb4.hpp"
#include "backend/sensor/camera/perspective.hpp"
#include "backend/solver/ba_solver.hpp"

namespace sapphire::backend::solver {
class Unit3ProjectionFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

  Unit3ProjectionFactor(gtsam::Key poseKey, gtsam::Key pointKey, const gtsam::Unit3& measured, const gtsam::SharedNoiseModel& noise)
      : Base(noise, poseKey, pointKey), measured_(measured) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<Unit3ProjectionFactor>(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& point, gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override {
    gtsam::Matrix Hpose;
    gtsam::Matrix Hpoint;
    const gtsam::Unit3 predicted = pose.bearing(point, Hpose, Hpoint);
    gtsam::Matrix Herror;
    const gtsam::Vector2 error = measured_.errorVector(predicted, {}, Herror);
    if (H1) *H1 = Herror * Hpose;
    if (H2) *H2 = Herror * Hpoint;
    return error;
  }

 private:
  gtsam::Unit3 measured_;
};

// Build native contiguous factors directly. The observation-order graph holds
// aliasing shared_ptrs into batch-owned buffers: no individual factor allocation
// or second factor copy, while preserving the original error/covariance order.
class BundleSummaryFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
  const featmap::PointSummary* summary_;  // Synchronous borrow from Problem.

 public:
  BundleSummaryFactor(const featmap::PointSummary& summary, gtsam::Key pose, gtsam::Key point, bool robust)
      : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>(
            robust ? gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
                         gtsam::noiseModel::mEstimator::Huber::Create(std::sqrt(7.81473)), gtsam::noiseModel::Unit::Create(3)))
                   : gtsam::SharedNoiseModel(gtsam::noiseModel::Unit::Create(3)), pose, point), summary_(&summary) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& point,
                              gtsam::OptionalMatrixType pose_j, gtsam::OptionalMatrixType point_j) const override {
    gtsam::Matrix36 dp;
    gtsam::Matrix33 dx;
    const auto local = pose.transformTo(point, pose_j ? &dp : nullptr, point_j ? &dx : nullptr);
    Mat33_t<double> j;
    try {
      const auto r = summary_->evaluate(local, (pose_j || point_j) ? &j : nullptr);
      if (pose_j) *pose_j = j * dp;
      if (point_j) *point_j = j * dx;
      return r;
    } catch (const std::domain_error&) {
      if (pose_j) *pose_j = gtsam::Matrix::Zero(3, 6);
      if (point_j) *point_j = gtsam::Matrix::Zero(3, 3);
      return gtsam::Vector3::Constant(std::numeric_limits<double>::infinity());
    }
  }
};

template <class BoundaryFactor>
class PointBatchBuilder {
 public:
  using Mono = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;
  using Stereo = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
  struct Graphs {
    gtsam::NonlinearFactorGraph original, batched;
  };

 private:
  template <class Factor, int Dimension>
  struct Group {
    typename gtsam::BatchFactor<Factor, Dimension>::FactorVector factors;
    std::vector<size_t> slots;
  };
  std::map<gtsam::Key, Group<Mono, 2>> mono_;
  std::map<gtsam::Key, Group<Stereo, 3>> stereo_;
  std::map<gtsam::Key, Group<BoundaryFactor, 2>> boundary_mono_;
  std::map<gtsam::Key, Group<BoundaryFactor, 3>> boundary_stereo_;
  std::map<gtsam::Key, Group<BundleSummaryFactor, 3>> summaries_;
  size_t count_ = 0;

  template <class Factor, int Dimension, class... Args>
  void add(std::map<gtsam::Key, Group<Factor, Dimension>>& groups, gtsam::Key point, size_t capacity, Args&&... args) {
    auto& group = groups[point];
    if (group.factors.empty()) {
      group.factors.reserve(capacity);
      group.slots.reserve(capacity);
    }
    group.factors.emplace_back(std::forward<Args>(args)...);
    group.slots.push_back(count_++);
  }
  template <class Factor, int Dimension>
  static void finish(std::map<gtsam::Key, Group<Factor, Dimension>>& groups, Graphs& out) {
    for (auto& [key, group] : groups) {
      // Moving this exact aligned vector preserves all element addresses.
      // Aliases retain the batch owner, including after batched is destroyed.
      auto* data = group.factors.data();
      auto batch = std::make_shared<gtsam::BatchFactor<Factor, Dimension>>(std::move(group.factors));
      for (size_t i = 0; i < group.slots.size(); ++i) out.original.at(group.slots[i]) = std::shared_ptr<gtsam::NonlinearFactor>(batch, data + i);
      out.batched.push_back(std::move(batch));
    }
  }

 public:
  template <class... Args>
  void summary(gtsam::Key point, Args&&... args) {
    add(summaries_, point, 1, std::forward<Args>(args)...);
  }
  template <class... Args>
  void mono(gtsam::Key point, size_t capacity, Args&&... args) {
    add(mono_, point, capacity, std::forward<Args>(args)...);
  }
  template <class... Args>
  void stereo(gtsam::Key point, size_t capacity, Args&&... args) {
    add(stereo_, point, capacity, std::forward<Args>(args)...);
  }
  template <class... Args>
  void boundary(gtsam::Key point, size_t capacity, bool mono, Args&&... args) {
    if (mono)
      add(boundary_mono_, point, capacity, std::forward<Args>(args)...);
    else
      add(boundary_stereo_, point, capacity, std::forward<Args>(args)...);
  }
  Graphs build() && {
    Graphs out;
    out.original.resize(count_);
    out.batched.reserve(mono_.size() + stereo_.size() + boundary_mono_.size() + boundary_stereo_.size());
    finish(mono_, out);
    finish(stereo_, out);
    finish(boundary_mono_, out);
    finish(boundary_stereo_, out);
    finish(summaries_, out);
    return out;
  }
};

template <class Factor, int Dimension>
void append_point_batches(std::map<gtsam::Key, typename gtsam::BatchFactor<Factor, Dimension>::FactorVector>& groups, gtsam::NonlinearFactorGraph& graph) {
  // The native aligned-vector overload transfers the buffer itself. A standard
  // vector rvalue overload still visits/moves each factor into another buffer.
  for (auto& [key, factors] : groups) graph.push_back(std::make_shared<gtsam::BatchFactor<Factor, Dimension>>(std::move(factors)));
}

// A batch contains observations of ONE landmark. Batching across landmarks
// would introduce artificial symbolic connectivity between independent points.
// Each child retains its own noise/Huber model, calibration and extrinsic.
// Keep the original graph for outlier decisions and undamped QR covariance.
template <class BoundaryFactor>
gtsam::NonlinearFactorGraph point_batch_graph(const gtsam::NonlinearFactorGraph& source) {
  using Mono = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;
  using Stereo = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
  std::map<gtsam::Key, typename gtsam::BatchFactor<Mono, 2>::FactorVector> mono;
  std::map<gtsam::Key, typename gtsam::BatchFactor<Stereo, 3>::FactorVector> stereo;
  std::map<gtsam::Key, typename gtsam::BatchFactor<BoundaryFactor, 2>::FactorVector> boundary_mono, boundary_stereo;
  for (const auto& factor : source) {
    if (const auto* f = dynamic_cast<const Mono*>(factor.get()))
      mono[f->keys().back()].push_back(*f);
    else if (const auto* f = dynamic_cast<const Stereo*>(factor.get()))
      stereo[f->keys().back()].push_back(*f);
    else if (const auto* f = dynamic_cast<const BoundaryFactor*>(factor.get())) {
      if (f->dim() != 2 && f->dim() != 3) throw std::invalid_argument("Invalid boundary pixel dimension");
      (f->dim() == 2 ? boundary_mono : boundary_stereo)[f->keys().front()].push_back(*f);
    } else {
      throw std::invalid_argument("Unsupported local BA factor for point batching");
    }
  }
  gtsam::NonlinearFactorGraph graph;
  graph.reserve(mono.size() + stereo.size() + boundary_mono.size() + boundary_stereo.size());
  append_point_batches<Mono, 2>(mono, graph);
  append_point_batches<Stereo, 3>(stereo, graph);
  append_point_batches<BoundaryFactor, 2>(boundary_mono, graph);
  append_point_batches<BoundaryFactor, 3>(boundary_stereo, graph);
  return graph;
}

// ICE-BA's unitary-camera / independent-point / cross blocks, adapted from
// scalar inverse depth to the current XYZ Jacobians. Camera Schur solve remains
// the existing small dense Cholesky, not ICE-BA's large-system PCG.
class PointSchurSolver {
  using M36 = Eigen::Matrix<double, 3, 6>;
  struct Point {
    gtsam::Key key;
    std::vector<int> cameras;
    Mat33_t<double> A = Mat33_t<double>::Zero();
    Vec3_t<double> b = Vec3_t<double>::Zero(), solved_b = Vec3_t<double>::Zero();
    std::vector<M36> B, solved_B;
  };
  struct Camera {
    gtsam::Key key;
    Mat66_t<double> A = Mat66_t<double>::Zero();
    Vec6_t<double> b = Vec6_t<double>::Zero();
  };
  struct Plan {
    size_t point;
    std::vector<int> camera_slots;
  };
  std::vector<Point> points_;
  std::vector<Camera> cameras_;
  std::vector<Plan> plans_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd b_, dx_;
  template <int D, int... Dims>
  bool add(const gtsam::GaussianFactor&, const Plan&);
  void initialize(const gtsam::GaussianFactorGraph&);
  void solveSchurComplement(double lambda);
  gtsam::VectorValues solveBackSubstitution() const;

 public:
  void assemble(const gtsam::GaussianFactorGraph& graph);
  gtsam::VectorValues solve(double lambda);
};

void configure_point_batch_solver(gtsam::LevenbergMarquardtParams& params);

// One workspace per optimizer, never attached to shared factors or Values.
// Structure is immutable within one BA pass. Every evaluation refreshes all
// state references and cameras, including rejected LM trial states.
struct WorkspaceProfile {
  double linearize_ms = 0, error_ms = 0, assemble_ms = 0, solve_ms = 0;
};
struct WorkspacePhase {
  double* result;
  std::chrono::steady_clock::time_point start;
  explicit WorkspacePhase(double* result) : result(result) {
    if (result) start = std::chrono::steady_clock::now();
  }
  ~WorkspacePhase() {
    if (result) *result += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }
};
template <class Boundary>
class WorkspaceOptimizer : public gtsam::LevenbergMarquardtOptimizer {
  using Mono = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;
  using Stereo = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
  struct Camera {
    gtsam::Key key;
    std::optional<gtsam::Pose3> extrinsic;
    gtsam::Matrix66 body_jacobian;
    std::shared_ptr<gtsam::Cal3_S2> mono_k;
    std::shared_ptr<gtsam::Cal3_S2Stereo> stereo_k;
    std::optional<gtsam::PinholeCamera<gtsam::Cal3_S2>> mono;
    std::optional<gtsam::StereoCamera> stereo;
  };
  template <class Factor, int D>
  struct Batch {
    using Source = gtsam::BatchFactor<Factor, D>;
    using Linear = std::conditional_t<Factor::N == 2, gtsam::BatchJacobianFactor<D, 6, 3>, gtsam::BatchJacobianFactor<D, 3>>;
    const Source* source;
    size_t point;
    std::vector<size_t> cameras;
    std::vector<typename Linear::SlotIndices> slots;
    std::vector<size_t> dimensions;
    std::vector<gtsam::Matrix> H;
    gtsam::Vector residual;
  };
  using Work = std::variant<Batch<Mono, 2>, Batch<Stereo, 3>, Batch<Boundary, 2>, Batch<Boundary, 3>, Batch<BundleSummaryFactor, 3>>;
  mutable const gtsam::Values* prepared_ = nullptr;
  mutable std::vector<Work> work_;
  mutable std::vector<Camera> cameras_;
  std::vector<gtsam::Key> point_keys_;
  mutable std::vector<const gtsam::Point3*> points_;
  mutable gtsam::GaussianFactorGraph::shared_ptr linear_;
  bool supported_ = true;
  bool point_schur_ = false;
  mutable PointSchurSolver schur_;
  mutable gtsam::GaussianFactorGraph::shared_ptr pending_linear_;
  mutable bool schur_assembled_ = false;
  WorkspaceProfile* profile_ = nullptr;
  static gtsam::LevenbergMarquardtParams solverParams(gtsam::LevenbergMarquardtParams p, bool point_schur) {
    if (point_schur && !p.dampingParams.diagonalDamping) p.setLinearSolverType("SEQUENTIAL_CHOLESKY");
    return p;
  }

  template <class Factor>
  size_t camera(const Factor& factor) {
    for (size_t i = 0; i < cameras_.size(); ++i) {
      const auto& c = cameras_[i];
      if (c.key != factor.keys().front() || bool(c.extrinsic) != bool(factor.body_P_sensor())) continue;
      if (c.extrinsic && !(c.extrinsic->matrix().array() == factor.body_P_sensor()->matrix().array()).all()) continue;
      if constexpr (std::is_same_v<Factor, Mono>) {
        if (c.mono_k && (c.mono_k->vector().array() == factor.calibration()->vector().array()).all()) return i;
      } else {
        if (c.stereo_k && (c.stereo_k->vector().array() == factor.calibration()->vector().array()).all()) return i;
      }
    }
    Camera c;
    c.key = factor.keys().front();
    c.extrinsic = factor.body_P_sensor();
    c.body_jacobian.setIdentity();
    if (c.extrinsic) gtsam::Pose3().compose(*c.extrinsic, c.body_jacobian);
    if constexpr (std::is_same_v<Factor, Mono>)
      c.mono_k = factor.calibration();
    else
      c.stereo_k = factor.calibration();
    cameras_.push_back(std::move(c));
    return cameras_.size() - 1;
  }
  template <class Factor, int D>
  bool add(const gtsam::NonlinearFactor::shared_ptr& factor, std::map<gtsam::Key, size_t>& indices) {
    const auto* batch = dynamic_cast<const gtsam::BatchFactor<Factor, D>*>(factor.get());
    if (!batch) return false;
    if (batch->factors().empty()) {
      supported_ = false;
      return true;
    }
    const auto point = batch->factors().front().keys().back();
    auto found = indices.emplace(point, point_keys_.size());
    if (found.second) point_keys_.push_back(point);
    Batch<Factor, D> w;
    w.source = batch;
    w.point = found.first->second;
    w.H.resize(Factor::N);
    w.residual.resize(D);
    w.dimensions.reserve(batch->keys().size());
    for (const auto key : batch->keys()) w.dimensions.push_back(key == point ? 3 : 6);
    for (const auto& f : batch->factors()) {
      if (f.keys().back() != point || (f.noiseModel() && f.noiseModel()->isConstrained())) supported_ = false;
      typename Batch<Factor, D>::Linear::SlotIndices slots;
      for (size_t j = 0; j < Factor::N; ++j) slots[j] = std::lower_bound(batch->keys().begin(), batch->keys().end(), f.keys()[j]) - batch->keys().begin();
      w.slots.push_back(slots);
      if constexpr (Factor::N == 2 && !std::is_same_v<Factor, BundleSummaryFactor>) {
        if (f.throwCheirality() || f.verboseCheirality()) supported_ = false;
        w.cameras.push_back(camera(f));
      }
    }
    work_.emplace_back(std::move(w));
    return true;
  }
  void prepare(const gtsam::Values& values) const {
    prepared_ = &values;
    for (size_t i = 0; i < point_keys_.size(); ++i) points_[i] = &values.atRef<gtsam::Point3>(point_keys_[i]);
    for (auto& c : cameras_) {
      const auto& body = values.atRef<gtsam::Pose3>(c.key);
      const auto sensor = c.extrinsic ? body.compose(*c.extrinsic) : body;
      if (c.mono_k)
        c.mono.emplace(sensor, *c.mono_k);
      else
        c.stereo.emplace(sensor, c.stereo_k);
    }
  }
  template <int D, class Factor>
  static auto boundary_error(const Factor& f, const gtsam::Point3& p, gtsam::OptionalMatrixType h, int) -> decltype(f.template evaluateFixed<D>(p, h)) {
    return f.template evaluateFixed<D>(p, h);
  }
  template <int D, class Factor>
  static gtsam::Vector boundary_error(const Factor& f, const gtsam::Point3& p, gtsam::OptionalMatrixType h, long) {
    return f.evaluateError(p, h);
  }

  template <class Factor, int D>
  void residual(Batch<Factor, D>& w, size_t i, bool jacobians) const {
    const auto& f = w.source->factors()[i];
    const auto& p = *points_[w.point];
    if constexpr (Factor::N == 1) {
      w.residual = boundary_error<D>(f, p, jacobians ? &w.H[0] : nullptr, 0);
    } else if constexpr (std::is_same_v<Factor, BundleSummaryFactor>) {
      w.residual = f.evaluateError(prepared_->atRef<gtsam::Pose3>(f.keys().front()), p,
                                  jacobians ? &w.H[0] : nullptr, jacobians ? &w.H[1] : nullptr);
    } else {
      const auto& c = cameras_[w.cameras[i]];
      try {
        if constexpr (std::is_same_v<Factor, Mono>)
          w.residual = c.mono->project(p, jacobians ? &w.H[0] : nullptr, jacobians ? &w.H[1] : nullptr, {}) - f.measured();
        else
          w.residual = (c.stereo->project(p, jacobians ? &w.H[0] : nullptr, jacobians ? &w.H[1] : nullptr) - f.measured()).vector();
        if (jacobians && c.extrinsic) {
          const Eigen::Matrix<double, D, 6> body = w.H[0] * c.body_jacobian;
          w.H[0] = body;
        }
      } catch (const gtsam::CheiralityException&) {
        if (f.throwCheirality()) throw;
        w.residual.setConstant(2 * f.calibration()->fx());
        if (jacobians) {
          w.H[0].setZero(D, 6);
          w.H[1].setZero(D, 3);
        }
      } catch (const gtsam::StereoCheiralityException&) {
        if (f.throwCheirality()) throw;
        w.residual.setConstant(2 * f.calibration()->fx());
        if (jacobians) {
          w.H[0].setZero(D, 6);
          w.H[1].setZero(D, 3);
        }
      }
    }
  }

 public:
  WorkspaceOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial, const gtsam::LevenbergMarquardtParams& params,
                     bool point_schur = false, WorkspaceProfile* profile = nullptr)
      : gtsam::LevenbergMarquardtOptimizer(graph, initial, solverParams(params, point_schur)), profile_(profile) {
    std::map<gtsam::Key, size_t> indices;
    work_.reserve(graph.size());
    for (const auto& f : graph)
      if (!add<Mono, 2>(f, indices) && !add<Stereo, 3>(f, indices) && !add<Boundary, 2>(f, indices) && !add<Boundary, 3>(f, indices) &&
          !add<BundleSummaryFactor, 3>(f, indices)) supported_ = false;
    points_.resize(point_keys_.size());
    point_schur_ = point_schur && supported_ && !params.dampingParams.diagonalDamping;
  }
  // Reuse only the symbolic structure and allocated solver buffers. All
  // numerical rows are reloaded by LM before its first solve in the new pass.
  bool reuseStructure(WorkspaceOptimizer& previous) {
    const bool reuse_points = point_schur_ && previous.point_schur_;
    if (!supported_ || !previous.supported_ || work_.size() != previous.work_.size() ||
        (!reuse_points && (!previous.nonlinearMultifrontalSolver_ || nonlinearMultifrontalSolver_)))
      return false;
    const auto settings = [](const gtsam::LevenbergMarquardtParams& p) {
      const auto& m = p.multifrontalParams;
      const auto& d = p.dampingParams;
      return std::make_tuple(p.linearSolverType, p.ordering, m.leafMergeDimCap, m.mergeDimCap, m.qrMode, m.qrAspectRatio, m.reportStream,
                             m.eliminationParallelThreshold, m.solutionParallelThreshold, m.numThreads, d.diagonalDamping, d.exactHessianDiagonal,
                             d.minDiagonal, d.maxDiagonal);
    };
    if (settings(params()) != settings(previous.params())) return false;
    for (size_t i = 0; i < work_.size(); ++i) {
      const bool same = std::visit(
          [&](const auto& w) {
            const auto* old = std::get_if<std::decay_t<decltype(w)>>(&previous.work_[i]);
            return old && w.source->keys() == old->source->keys() && w.dimensions == old->dimensions && w.slots == old->slots;
          },
          work_[i]);
      if (!same) return false;
    }
    if (reuse_points)
      schur_ = std::move(previous.schur_);
    else
      nonlinearMultifrontalSolver_ = std::move(previous.nonlinearMultifrontalSolver_);
    return true;
  }
  gtsam::GaussianFactorGraph::shared_ptr linearize() const override {
    WorkspacePhase phase(profile_ ? &profile_->linearize_ms : nullptr);
    pending_linear_.reset();
    if (!supported_) return gtsam::LevenbergMarquardtOptimizer::linearize();
    prepare(values());
    // iterate() may return a graph retained by a caller. Never mutate such a
    // snapshot: reuse only when this workspace is its sole owner.
    const bool reuse =
        linear_ && linear_.use_count() == 1 && std::all_of(linear_->begin(), linear_->end(), [](const auto& factor) { return factor.use_count() == 1; });
    if (!reuse) {
      linear_ = std::make_shared<gtsam::GaussianFactorGraph>();
      linear_->reserve(work_.size());
    }
    for (size_t index = 0; index < work_.size(); ++index)
      std::visit(
          [&](auto& w) {
            using Linear = typename std::decay_t<decltype(w)>::Linear;
            Linear* linear;
            if (reuse) {
              linear = static_cast<Linear*>((*linear_)[index].get());
              linear->clearRows();
            } else {
              auto owned = std::make_shared<Linear>(w.source->keys(), w.dimensions);
              linear = owned.get();
              linear->reserve(w.slots.size());
              linear_->push_back(std::move(owned));
            }
            for (size_t i = 0; i < w.slots.size(); ++i) {
              residual(w, i, true);
              const auto& noise = w.source->factors()[i].noiseModel();
              if (noise && !noise->isUnit()) noise->WhitenSystem(w.H, w.residual);
              w.residual = -w.residual;
              linear->addRow(w.slots[i], w.H, w.residual);
            }
          },
          work_[index]);
    if (point_schur_) {
      pending_linear_ = linear_;
      schur_assembled_ = false;
    }
    return linear_;
  }
  // The native LM state machine still chooses lambda, retracts, evaluates
  // robust loss and accepts/rejects trials. Only its linear solve is specialized.
  gtsam::VectorValues solveDamped(const gtsam::GaussianFactorGraph& graph, const gtsam::VectorValues& diagonal) const override {
    if (!point_schur_) return gtsam::LevenbergMarquardtOptimizer::solveDamped(graph, diagonal);
    if (!schur_assembled_) {
      WorkspacePhase phase(profile_ ? &profile_->assemble_ms : nullptr);
      schur_.assemble(*pending_linear_);
      schur_assembled_ = true;
    }
    WorkspacePhase phase(profile_ ? &profile_->solve_ms : nullptr);
    return schur_.solve(lambda());
  }
  double evaluateError(const gtsam::Values& values) const override {
    WorkspacePhase phase(profile_ ? &profile_->error_ms : nullptr);
    if (!supported_) return gtsam::LevenbergMarquardtOptimizer::evaluateError(values);
    prepare(values);
    double total = 0;
    for (auto& work : work_)
      std::visit(
          [&](auto& w) {
            double batch_error = 0;
            for (size_t i = 0; i < w.slots.size(); ++i) {
              residual(w, i, false);
              const auto& noise = w.source->factors()[i].noiseModel();
              batch_error += noise ? noise->loss(noise->squaredMahalanobisDistance(w.residual)) : .5 * w.residual.squaredNorm();
            }
            total += batch_error;
          },
          work);
    return total;
  }
};

struct CovarianceProfile {
  double linearize_ms = 0, point_elimination_ms = 0, camera_ms = 0, recovery_ms = 0;
  double camera_information_ratio = 0;
  bool camera_qr = false;
};
// Three-column square-root point elimination and a small camera solve with a
// condition-checked Cholesky path / QR fallback, followed by marginal recovery.
// Keeps all pose-point correlations; specialized to independent XYZ landmarks
// and the small set of variable poses of one local BA, not an arbitrary graph.
std::map<uint64_t, Eigen::MatrixXd> bundle_covariances(const gtsam::NonlinearFactorGraph&, const gtsam::Values&, CovarianceProfile* = nullptr,
                                                       const gtsam::GaussianFactorGraph* final_linear = nullptr);

// Projection-only checks do not need a factor, variable keys or a noise model.
// Use the same native camera projection and arithmetic as GTSAM pixel factors.
inline double squared_pixel_error(const gtsam::Pose3& camera_pose, const gtsam::Point3& point, const std::array<float, 5>& k, const gtsam::Vector3& measured) {
  if (!point.allFinite()) return std::numeric_limits<double>::infinity();
#ifndef GTSAM_THROW_CHEIRALITY_EXCEPTION
  if (camera_pose.transformTo(point).z() <= 0) return std::numeric_limits<double>::infinity();
#endif
  if (measured.y() < 0) {
    try {
      const gtsam::PinholeCamera<gtsam::Cal3_S2> camera(camera_pose, gtsam::Cal3_S2(k[0], k[1], 0, k[2], k[3]));
      return (camera.project(point) - gtsam::Point2(measured.x(), measured.z())).squaredNorm();
    } catch (const gtsam::CheiralityException&) {
      return std::numeric_limits<double>::infinity();
    }
  }
  try {
    gtsam::Cal3_S2Stereo calibration(k[0], k[1], 0, k[2], k[3], k[4] / k[0]);
    // StereoCamera accepts shared_ptr only. An empty-owner alias borrows this
    // stack calibration without a control-block allocation; neither escapes.
    const std::shared_ptr<gtsam::Cal3_S2Stereo> borrowed(std::shared_ptr<gtsam::Cal3_S2Stereo>{}, &calibration);
    const gtsam::StereoCamera camera(camera_pose, borrowed);
    return (camera.project(point) - gtsam::StereoPoint2(measured.x(), measured.y(), measured.z())).vector().squaredNorm();
  } catch (const gtsam::StereoCheiralityException&) {
    return std::numeric_limits<double>::infinity();
  }
}

inline gtsam::Pose3 to_pose3(const StateGroupf& p) {
  return {gtsam::Rot3(Eigen::Quaterniond(p.rotationMatrix().cast<double>()).normalized().toRotationMatrix()), p.translationVector().cast<double>()};
}
// Immutable camera data is shared by all observations of the same fixed view.
struct BundleBoundaryCamera {
  gtsam::PinholeCamera<gtsam::Cal3_S2> mono;
  gtsam::StereoCamera stereo;
  BundleBoundaryCamera(const gtsam::Pose3& p, const std::shared_ptr<gtsam::Cal3_S2>& m, const std::shared_ptr<gtsam::Cal3_S2Stereo>& s)
      : mono(p, *m), stereo(p, s) {}
  BundleBoundaryCamera(const camera::StereoObservation& o, const StateGroupf& boundary)
      : BundleBoundaryCamera(to_pose3(boundary).compose(to_pose3(o.anchor_from_camera())),
                             std::make_shared<gtsam::Cal3_S2>(o.calibration()[0], o.calibration()[1], 0, o.calibration()[2], o.calibration()[3]),
                             std::make_shared<gtsam::Cal3_S2Stereo>(o.calibration()[0], o.calibration()[1], 0, o.calibration()[2], o.calibration()[3],
                                                                    o.calibration()[4] / o.calibration()[0])) {}
};
// Reuse camera geometry within one validation pass; no factors or noise models.
struct BundleProjection {
  struct Sensor {
    const camera::StereoObservation* source;
    gtsam::Pose3 extrinsic;
    std::shared_ptr<gtsam::Cal3_S2> mono;
    std::shared_ptr<gtsam::Cal3_S2Stereo> stereo;
    std::map<const StateGroupf*, std::shared_ptr<const BundleBoundaryCamera>> fixed;
  };
  std::vector<Sensor> sensors;
  Sensor& sensor(const camera::StereoObservation& o) {
    for (auto& s : sensors)
      if (s.source->calibration() == o.calibration() &&
          (s.source->anchor_from_camera().rotationMatrix().array() == o.anchor_from_camera().rotationMatrix().array()).all() &&
          (s.source->anchor_from_camera().translationVector().array() == o.anchor_from_camera().translationVector().array()).all())
        return s;
    const auto& k = o.calibration();
    sensors.push_back({&o,
                       to_pose3(o.anchor_from_camera()),
                       std::make_shared<gtsam::Cal3_S2>(k[0], k[1], 0, k[2], k[3]),
                       std::make_shared<gtsam::Cal3_S2Stereo>(k[0], k[1], 0, k[2], k[3], k[4] / k[0]),
                       {}});
    return sensors.back();
  }
  std::shared_ptr<const BundleBoundaryCamera> boundary(const camera::StereoObservation& o, const StateGroupf& p) {
    auto& s = sensor(o);
    auto [it, inserted] = s.fixed.try_emplace(&p);
    if (inserted) it->second = std::make_shared<BundleBoundaryCamera>(to_pose3(p).compose(s.extrinsic), s.mono, s.stereo);
    return it->second;
  }
  bool inlier(const camera::StereoObservation& o, const StateGroupf& p, const gtsam::Point3& point, double historical_error = 0) {
    if (!point.allFinite()) return false;
    const auto camera = boundary(o, p);
#ifndef GTSAM_THROW_CHEIRALITY_EXCEPTION
    if (camera->mono.pose().transformTo(point).z() <= 0) return false;
#endif
    double error;
    try {
      error = o.pixel().y < 0 ? (camera->mono.project(point) - gtsam::Point2(o.pixel().x, o.pixel().z)).squaredNorm()
                            : (camera->stereo.project(point) - gtsam::StereoPoint2(o.pixel().x, o.pixel().y, o.pixel().z)).vector().squaredNorm();
    } catch (const gtsam::CheiralityException&) {
      return false;
    } catch (const gtsam::StereoCheiralityException&) {
      return false;
    }
    return std::isfinite(error) && std::isfinite(historical_error) &&
           error / (o.sigma() * o.sigma()) + historical_error <= (o.pixel().y < 0 ? 5.99146 : 7.81473);
  }
};
using BundlePixelFactor = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;
inline gtsam::SharedNoiseModel bundleNoise(float sigma, bool robust, int dimension = 3) {
  auto n = gtsam::noiseModel::Isotropic::Sigma(dimension, sigma);
  return robust ? gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(std::sqrt(dimension == 3 ? 7.81473 : 5.99146)), n)
                : gtsam::SharedNoiseModel(n);
}
inline std::shared_ptr<BundlePixelFactor> bundleFactor(const camera::StereoObservation& o, gtsam::Key key, bool robust, gtsam::Key camera_key) {
  const auto& k = o.calibration();
  if (o.pixel().y < 0)
    return std::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
        gtsam::Point2(o.pixel().x, o.pixel().z), bundleNoise(o.sigma(), robust, 2), camera_key, key, std::make_shared<gtsam::Cal3_S2>(k[0], k[1], 0, k[2], k[3]),
        false, false, to_pose3(o.anchor_from_camera()));
  return std::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
      gtsam::StereoPoint2(o.pixel().x, o.pixel().y, o.pixel().z), bundleNoise(o.sigma(), robust), camera_key, key,
      std::make_shared<gtsam::Cal3_S2Stereo>(k[0], k[1], 0, k[2], k[3], k[4] / k[0]), false, false, to_pose3(o.anchor_from_camera()));
}
class BundleBoundaryFactor final : public gtsam::NoiseModelFactorN<gtsam::Point3> {
  std::shared_ptr<const BundleBoundaryCamera> camera_;
  cv::Point3f measured_;
  const featmap::PointSummary* summary_ = nullptr;  // Call-scoped borrow from Problem.

 public:
  BundleBoundaryFactor(const featmap::PointSummary& summary, gtsam::Key key)
      : gtsam::NoiseModelFactorN<gtsam::Point3>(gtsam::noiseModel::Unit::Create(3), key), summary_(&summary) {}
  BundleBoundaryFactor(const camera::StereoObservation& o, gtsam::Key key, std::shared_ptr<const BundleBoundaryCamera> camera,
                       const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactorN<gtsam::Point3>(model, key), camera_(std::move(camera)), measured_(o.pixel()) {}
  BundleBoundaryFactor(const camera::StereoObservation& o, gtsam::Key key, const StateGroupf& boundary, bool robust)
      : BundleBoundaryFactor(o, key, std::make_shared<BundleBoundaryCamera>(o, boundary), bundleNoise(o.sigma(), robust, o.pixel().y < 0 ? 2 : 3)) {}
  template <int D>
  Eigen::Matrix<double, D, 1> evaluateFixed(const gtsam::Point3& p, gtsam::OptionalMatrixType h) const {
    if constexpr (D == 2) {
      try {
        return camera_->mono.project(p, {}, h, {}) - gtsam::Point2(measured_.x, measured_.z);
      } catch (const gtsam::CheiralityException&) {
        if (h) *h = gtsam::Matrix::Zero(2, 3);
        return gtsam::Vector2::Constant(2 * camera_->mono.calibration().fx());
      }
    } else {
      if (summary_) {
        Mat33_t<double> jacobian;
        try {
          const auto residual = summary_->evaluate(p, h ? &jacobian : nullptr);
          if (h) *h = jacobian;
          return residual;
        } catch (const std::domain_error&) {
          // Invalid trial depth rejects the LM step; it is not evidence that
          // a previously accepted historical pixel has become an outlier.
          if (h) *h = gtsam::Matrix::Zero(3, 3);
          return gtsam::Vector3::Constant(std::numeric_limits<double>::infinity());
        }
      }
      try {
        return (camera_->stereo.project(p, {}, h) - gtsam::StereoPoint2(measured_.x, measured_.y, measured_.z)).vector();
      } catch (const gtsam::StereoCheiralityException&) {
        if (h) *h = gtsam::Matrix::Zero(3, 3);
        return gtsam::Vector3::Constant(2 * camera_->mono.calibration().fx());
      }
    }
  }
  gtsam::Vector evaluateError(const gtsam::Point3& p, gtsam::OptionalMatrixType h) const override {
    if (summary_) return evaluateFixed<3>(p, h);
    if (measured_.y < 0) return evaluateFixed<2>(p, h);
    return evaluateFixed<3>(p, h);
  }
};
struct BundleFactors : BundleProjection {
  std::map<std::pair<float, int>, gtsam::SharedNoiseModel> models;
  bool robust;
  explicit BundleFactors(bool robust) : robust(robust) {}
  const gtsam::SharedNoiseModel& model(const camera::StereoObservation& o) {
    const int d = o.pixel().y < 0 ? 2 : 3;
    auto [it, inserted] = models.try_emplace({o.sigma(), d});
    if (inserted) it->second = bundleNoise(o.sigma(), robust, d);
    return it->second;
  }
  void addPixel(gtsam::NonlinearFactorGraph& graph, PointBatchBuilder<BundleBoundaryFactor>& builder, bool reference, const camera::StereoObservation& o,
                gtsam::Key key, size_t capacity, gtsam::Key camera_key) {
    if (reference) {
      graph.push_back(bundleFactor(o, key, robust, camera_key));
      return;
    }
    const auto& s = sensor(o);
    if (o.pixel().y < 0)
      builder.mono(key, capacity, gtsam::Point2(o.pixel().x, o.pixel().z), model(o), camera_key, key, s.mono, false, false, s.extrinsic);
    else
      builder.stereo(key, capacity, gtsam::StereoPoint2(o.pixel().x, o.pixel().y, o.pixel().z), model(o), camera_key, key, s.stereo, false, false, s.extrinsic);
  }
  void addBoundary(gtsam::NonlinearFactorGraph& graph, PointBatchBuilder<BundleBoundaryFactor>& builder, bool reference, const camera::StereoObservation& o,
                   gtsam::Key key, size_t capacity, const StateGroupf& pose) {
    if (reference)
      graph.push_back(std::make_shared<BundleBoundaryFactor>(o, key, pose, robust));
    else
      builder.boundary(key, capacity, o.pixel().y < 0, o, key, boundary(o, pose), model(o));
  }
};

}  // namespace sapphire::backend::solver
