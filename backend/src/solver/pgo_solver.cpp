#include "backend/solver/pgo_solver.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <array>
#include <backend/common/common_type.hpp>
#include <backend/common/state_group.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/params/pgo.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <tuple>

#include "backend/solver/pgo_solver_factors.hpp"

namespace sapphire::backend::solver {
// GTSAM Pose3 orders tangent components [rotation, translation].
constexpr std::array<int, 6> kPose3TangentOrder{3, 4, 5, 0, 1, 2};
static inline gtsam::Pose3 to_pose3(const sapphire::backend::StateGroupf& state) {
  return gtsam::Pose3(gtsam::Rot3(state.rotationMatrix().cast<double>()), gtsam::Point3(state.x(), state.y(), state.z()));
}

static sapphire::backend::StateGroupf stateFromPose3(const gtsam::Pose3& pose) {
  return sapphire::backend::StateGroupf::fromRotationTranslation(pose.rotation().matrix().cast<float>(), pose.translation().cast<float>());
}

constexpr unsigned char kPoseSymbol = 'x';

static gtsam::Key poseKey(int id) {
  if (id <= 0) throw std::invalid_argument("Graph poses require positive capture IDs");
  return gtsam::Symbol(kPoseSymbol, static_cast<std::uint64_t>(id));
}

struct LinkIdentity {
  int from = 0;
  int to = 0;
  int type = 0;
  std::array<float, sapphire::backend::StateGroupf::kCoefficientCount> transform{};
  std::array<double, 36> information{};

  explicit LinkIdentity(const sapphire::backend::Link& link)
      : from(link.from()), to(link.to()), type(static_cast<int>(link.type())), transform(link.transform().coefficients()) {
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        information[static_cast<std::size_t>(row * 6 + column)] = link.infMatrix()(row, column);
      }
    }
  }

  bool operator<(const LinkIdentity& other) const {
    return std::tie(from, to, type, transform, information) < std::tie(other.from, other.to, other.type, other.transform, other.information);
  }
};

struct PoseGraphSolver::IncrementalState {
  struct FactorRecord {
    gtsam::FactorIndices indices;
  };

  explicit IncrementalState(const gtsam::ISAM2Params& paramsIn) : params(paramsIn) { reset(); }
  const gtsam::ISAM2& solver() const { return *isam; }
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values, const gtsam::ISAM2UpdateParams& options) {
    return isam->update(factors, values, options);
  }
  IncrementalState(const IncrementalState& other)
      : params(other.params),
        isam(other.isam ? new gtsam::ISAM2(*other.isam) : nullptr),
        factors(other.factors),
        poseCache(other.poseCache),
        rootId(other.rootId),
        rootHasGravity(other.rootHasGravity),
        rootFixed(other.rootFixed),
        rootPose(other.rootPose),
        rootFactorIndex(other.rootFactorIndex) {}

  void reset() {
    isam.reset(new gtsam::ISAM2(params));
    factors.clear();
    poseCache.clear();
    rootId = 0;
    rootHasGravity = false;
    rootFixed = false;
    rootPose = StateGroupf{};
    rootFactorIndex = std::numeric_limits<gtsam::FactorIndex>::max();
  }

  gtsam::ISAM2Params params;
  std::unique_ptr<gtsam::ISAM2> isam;
  std::map<LinkIdentity, std::vector<FactorRecord>> factors;
  std::map<int, StateGroupf> poseCache;
  int rootId = 0;
  bool rootHasGravity = false;
  bool rootFixed = false;
  StateGroupf rootPose;
  gtsam::FactorIndex rootFactorIndex = std::numeric_limits<gtsam::FactorIndex>::max();
};

PoseGraphSolver::PoseGraphSolver(int iterations, bool covarianceIgnored, double epsilon, bool robust, bool priorsIgnored, float gravitySigma)
    : options{iterations, covarianceIgnored, epsilon, robust, priorsIgnored, gravitySigma},
      internalOptimizerType_(sapphire::backend::PGOParams{}.optimizer) {}

PoseGraphSolver::PoseGraphSolver(const ParametersMap& parameters) : PoseGraphSolver(PGOParams{}.withOverrides(parameters)) {}

std::map<int, StateGroupf> PoseGraphSolver::optimize(int rootId, const std::map<int, StateGroupf>& poses,
                                                     const std::multimap<int, Link>& edgeConstraints,
                                                     std::list<std::map<int, StateGroupf>>* intermediateGraphs, double* finalError,
                                                     int* iterationsDone) {
  std::optional<Mat66_t<double>> covariance;
  return optimize(rootId, poses, edgeConstraints, covariance, intermediateGraphs, finalError, iterationsDone);
}

PoseGraphSolver::~PoseGraphSolver() = default;

PoseGraphSolver::Trial::Trial(PoseGraphSolver& optimizer) : optimizer_(&optimizer) {
  if (optimizer.trialActive_) throw std::logic_error("Nested optimizer trial");
  tools::Timer timer;
  if (optimizer.incrementalState_ && !optimizer.options.robust) optimizer.trialState_.reset(new IncrementalState(*optimizer.incrementalState_));
  optimizer.trial_pose_changes_.clear();
  optimizer.trialActive_ = true;
  EXT_LOG_INFO("[optimization_trial] Created elapsed_ms={:.3f}", timer.elapsed_seconds() * 1000);
}

PoseGraphSolver::Trial::~Trial() {
  if (!optimizer_) return;
  optimizer_->trialState_.reset();
  optimizer_->trial_pose_changes_.clear();
  optimizer_->trialActive_ = false;
  EXT_LOG_INFO("[optimization_trial] Discarded");
}

void PoseGraphSolver::Trial::commit() noexcept {
  if (!optimizer_) return;
  // Publish using pointer/node transfers only, after the database commit.
  optimizer_->incrementalState_.swap(optimizer_->trialState_);
  optimizer_->changed_pose_ids_.merge(optimizer_->trial_pose_changes_);
  optimizer_->trial_pose_changes_.clear();
  optimizer_->trialState_.reset();
  optimizer_->trialActive_ = false;
  optimizer_ = nullptr;
}

PoseGraphSolver::PoseGraphSolver(const PGOParams& configuration)
    : PoseGraphSolver(configuration.max_iterations, configuration.ignore_constraint_variance, configuration.min_error_improvement,
                      configuration.gnc_enabled, configuration.ignore_priors, configuration.gravity_sigma) {
  configuration.validate();
  internalOptimizerType_ = configuration.optimizer;
  const bool incremental = configuration.incremental_enabled;
  const double rotationThreshold = configuration.isam2_relinearize_rotation_rad;
  const double translationThreshold = configuration.isam2_relinearize_translation_m;
  const double wildfireThreshold = configuration.isam2_wildfire_threshold;
  const int skip = configuration.isam2_relinearize_skip;
  const bool adaptiveReorder = configuration.isam2_adaptive_reorder_enabled;
  const double adaptiveReorderThreshold = configuration.isam2_reorder_growth_ratio;
  EXT_LOG_DEBUG("GTSAM {}={}", "pose_graph.optimizer", internalOptimizerType_);
  EXT_LOG_DEBUG("GTSAM {}={}", "pose_graph.incremental_enabled", incremental ? 1 : 0);
  EXT_LOG_DEBUG("GTSAM typed thresholds rotation={:.6f}rad translation={:.6f}m", rotationThreshold, translationThreshold);
  EXT_LOG_DEBUG("GTSAM {}={}", "pose_graph.isam2.relinearize_skip", skip);
  EXT_LOG_DEBUG("GTSAM wildfire={:.6f} adaptiveReorder={} ratio={:.3f}", wildfireThreshold, adaptiveReorder ? 1 : 0, adaptiveReorderThreshold);
  if (incremental) {
    gtsam::ISAM2Params::OptimizationParams optParams;
    if (internalOptimizerType_ == 2) {
      optParams = gtsam::ISAM2DoglegParams(1.0, wildfireThreshold);
    } else {
      optParams = gtsam::ISAM2GaussNewtonParams(wildfireThreshold);
    }
    gtsam::ISAM2Params params(optParams);
    gtsam::ISAM2ThresholdMap thresholds;
    thresholds[kPoseSymbol] = (gtsam::Vector(6) << rotationThreshold, rotationThreshold, rotationThreshold, translationThreshold,
                               translationThreshold, translationThreshold)
                                  .finished();
    params.relinearizeThreshold = thresholds;
    params.relinearizeSkip = skip;
    params.evaluateNonlinearError = false;
    params.findUnusedFactorSlots = true;
    params.enableAdaptiveReorder = adaptiveReorder;
    params.adaptiveReorderThreshold = adaptiveReorderThreshold;
    incrementalState_.reset(new IncrementalState(params));
  } else {
    incrementalState_.reset();
  }
}

bool PoseGraphSolver::optimizeInPlace(int rootId, std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                                      std::optional<Mat66_t<double>>& outputCovariance, double* finalError, int* iterationsDone) {
  robust_decisions.clear();
  if (incrementalState_ && !options.robust) {
    return optimizeISAM2(rootId, poses, constraints, outputCovariance, finalError, iterationsDone);
  }

  const std::map<int, StateGroupf> optimized = optimize(rootId, poses, constraints, outputCovariance, nullptr, finalError, iterationsDone);
  if (optimized.empty()) {
    return false;
  }
  for (const auto& pose : optimized) {
    const auto old = poses.find(pose.first);
    if (old == poses.end() || old->second != pose.second) pending_pose_changes().insert(pose.first);
  }
  poses = optimized;
  return true;
}

bool PoseGraphSolver::optimizeISAM2(int rootId, std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                                    std::optional<Mat66_t<double>>& outputCovariance, double* finalError, int* iterationsDone) {
  outputCovariance.reset();
  if (iterationsDone) *iterationsDone = 0;
  if (finalError) *finalError = 0.0;
  if (options.max_iterations <= 0) return true;
  // A fused component may shrink to its fixed datum. Reconcile even that
  // root-only graph: otherwise old member variables/factors survive the commit.
  if (poses.empty() || (poses.size() > 1 && constraints.empty()) || poses.rbegin()->first <= 0) {
    EXT_LOG_WARN("iSAM2 requires a pose datum and constraints between distinct variables.");
    return false;
  }

  IncrementalState& state = *(trialActive_ ? trialState_ : incrementalState_);
  tools::Timer stageTimer;

  bool intersectsPreviousSession = state.poseCache.empty();
  if (!intersectsPreviousSession) {
    for (const auto& pose : poses) {
      if (state.poseCache.count(pose.first)) {
        intersectsPreviousSession = true;
        break;
      }
    }
  }
  if (!intersectsPreviousSession) {
    EXT_LOG_DEBUG("iSAM2: rebuilding for a disjoint mapping session.");
    state.reset();
  }

  bool hasGravityConstraints = false;
  bool hasPosePrior = false;
  for (const auto& constraint : constraints) {
    const Link& link = constraint.second;
    if (link.from() == link.to()) {
      hasPosePrior = hasPosePrior || (!options.ignore_priors && link.type() == Link::kPosePrior);
      hasGravityConstraints = hasGravityConstraints || (options.gravity_sigma > 0.0f && link.type() == Link::kGravity);
    }
  }
  for (const auto& pose : poses)
    if (pose.first <= 0) throw std::invalid_argument("Graph poses require positive capture IDs");

  const int effectiveRootId = hasPosePrior && !fixed_root ? 0 : rootId;
  if (effectiveRootId != 0 && poses.find(effectiveRootId) == poses.end()) {
    EXT_LOG_ERROR("iSAM2: root pose {} is absent.", effectiveRootId);
    return false;
  }

  auto isActiveConstraint = [&](const Link& link) {
    if (link.from() == link.to()) {
      if (link.type() == Link::kPosePrior) return !options.ignore_priors;
      if (link.type() == Link::kGravity) return options.gravity_sigma > 0.0f && link.from() > 0;
      return false;
    }
    if (link.from() < 0 || link.to() < 0) return false;
    return true;
  };

  std::map<LinkIdentity, std::vector<const Link*>> desiredFactors;
  for (const auto& constraint : constraints) {
    const Link& link = constraint.second;
    if (poses.find(link.from()) == poses.end() || poses.find(link.to()) == poses.end()) {
      EXT_LOG_ERROR("iSAM2: constraint {}->{} references a missing pose.", link.from(), link.to());
      return false;
    }
    if (link.transform().isNull() || !isActiveConstraint(link)) continue;
    desiredFactors[LinkIdentity(link)].push_back(&link);
  }
  const StateGroupf requestedRootPose = effectiveRootId ? poses.at(effectiveRootId) : StateGroupf{};
  const double reconciliationSeconds = stageTimer.lap_seconds();

  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newTheta;
  gtsam::ISAM2UpdateParams updateParams;
  struct PendingGroup {
    LinkIdentity identity;
    std::size_t first;
    std::size_t count;
  };
  std::vector<PendingGroup> pendingGroups;
  bool loopClosureAdded = false;

  for (const auto& previous : state.factors) {
    const auto desired = desiredFactors.find(previous.first);
    const std::size_t keep = desired == desiredFactors.end() ? 0 : std::min(previous.second.size(), desired->second.size());
    for (std::size_t index = keep; index < previous.second.size(); ++index) {
      updateParams.removeFactorIndices.insert(updateParams.removeFactorIndices.end(), previous.second[index].indices.begin(),
                                              previous.second[index].indices.end());
    }
  }

  // Rigid submap refinement can change the fixed capture-to-owner placement
  // while keeping the owner ID. Refresh its prior rather than pinning the owner
  // at an obsolete pose and moving the persistent capture datum on expansion.
  const bool fixedPoseChanged = fixed_root && effectiveRootId != 0 &&
                                (state.rootPose.isNull() || (poses.at(effectiveRootId).matrix() - state.rootPose.matrix()).norm() > 1e-6f);
  const bool replaceRoot =
      effectiveRootId != state.rootId ||
      (effectiveRootId != 0 && (hasGravityConstraints != state.rootHasGravity || fixed_root != state.rootFixed || fixedPoseChanged));
  if (replaceRoot && state.rootId != 0 && state.rootFactorIndex != std::numeric_limits<gtsam::FactorIndex>::max()) {
    updateParams.removeFactorIndices.push_back(state.rootFactorIndex);
  }
  std::size_t pendingRootFactor = std::numeric_limits<std::size_t>::max();
  if (replaceRoot && effectiveRootId != 0) {
    const StateGroupf& initialPose = poses.at(effectiveRootId);
    const gtsam::Vector6 variances =
        fixed_root
            ? gtsam::Vector6::Constant(1e-12)
            : (gtsam::Vector(6) << (hasGravityConstraints ? 2.0 : 1e-9), (hasGravityConstraints ? 2.0 : 1e-9), 1e-9, 1e-9, 1e-9, 1e-9).finished();
    pendingRootFactor = newFactors.size();
    newFactors.add(
        gtsam::PriorFactor<gtsam::Pose3>(poseKey(effectiveRootId), to_pose3(initialPose), gtsam::noiseModel::Diagonal::Variances(variances)));
  }

  auto addConstraintFactors = [&](const Link& original) {
    int id1 = original.from();
    int id2 = original.to();
    StateGroupf measurement = original.transform();
    const std::size_t first = newFactors.size();

    if (id1 == id2) {
      const gtsam::Point3 translation(measurement.x(), measurement.y(), measurement.z());
      if (original.type() == Link::kPosePrior) {
        if (1.0 / original.infMatrix()(3, 3) >= 9999.0 || 1.0 / original.infMatrix()(4, 4) >= 9999.0 || 1.0 / original.infMatrix()(5, 5) >= 9999.0) {
          const gtsam::Vector3 precision(original.infMatrix()(0, 0), original.infMatrix()(1, 1), original.infMatrix()(2, 2));
          const gtsam::Key key = poseKey(id1);
          newFactors.add(PositionFactor(key, translation, gtsam::noiseModel::Diagonal::Precisions(precision)));
        } else {
          const gtsam::Key key = poseKey(id1);
          const Eigen::Matrix<double, 6, 6> information = options.ignore_covariance
                                                              ? Eigen::Matrix<double, 6, 6>::Identity()
                                                              : original.infMatrix()(kPose3TangentOrder, kPose3TangentOrder).eval();
          newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(key, to_pose3(measurement), gtsam::noiseModel::Gaussian::Information(information)));
        }
      } else if (original.type() == Link::kGravity) {
        const gtsam::Rot3 measuredRotation = to_pose3(measurement).rotation();
        const gtsam::Unit3 worldUp(0.0, 0.0, 1.0);
        newFactors.add(gtsam::AttitudeFactor<gtsam::Pose3>(poseKey(id1), worldUp, gtsam::noiseModel::Isotropic::Sigma(2, options.gravity_sigma),
                                                           measuredRotation.unrotate(worldUp)));
      }
    } else {
      const Eigen::Matrix<double, 6, 6> information =
          options.ignore_covariance ? Eigen::Matrix<double, 6, 6>::Identity() : original.infMatrix()(kPose3TangentOrder, kPose3TangentOrder).eval();
      const gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(information);
      newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(poseKey(id1), poseKey(id2), to_pose3(measurement), model));
    }
    return std::make_pair(first, newFactors.size() - first);
  };

  for (const auto& desired : desiredFactors) {
    const auto previous = state.factors.find(desired.first);
    const std::size_t alreadyRegistered = previous == state.factors.end() ? 0 : std::min(previous->second.size(), desired.second.size());
    for (std::size_t occurrence = alreadyRegistered; occurrence < desired.second.size(); ++occurrence) {
      const auto range = addConstraintFactors(*desired.second[occurrence]);
      if (range.second == 0) continue;
      pendingGroups.push_back(PendingGroup{desired.first, range.first, range.second});
      const Link& link = *desired.second[occurrence];
      loopClosureAdded = loopClosureAdded || link.isLoopClosure();
    }
  }

  const gtsam::KeySet requiredKeys = newFactors.keys();
  for (const auto& pose : poses) {
    const gtsam::Key key = poseKey(pose.first);
    if (!state.solver().valueExists(key) && requiredKeys.count(key)) {
      newTheta.insert(key, to_pose3(pose.second));
    }
  }

  updateParams.force_relinearize = loopClosureAdded;
  const double materializationSeconds = stageTimer.lap_seconds();

  gtsam::ISAM2Result aggregate;
  gtsam::ISAM2Result result;
  gtsam::FactorIndices newFactorIndices;
  int updates = 0;
  double primaryUpdateSeconds = 0.0;
  double followupUpdateSeconds = 0.0;
  try {
    result = state.update(newFactors, newTheta, updateParams);
    newFactorIndices = result.newFactorsIndices;
    ++updates;
    aggregate = result;
    primaryUpdateSeconds = stageTimer.lap_seconds();
    while (updates < std::max(1, options.max_iterations)) {
      const gtsam::ISAM2UpdateParams nextParams;
      const std::pair<gtsam::KeySet, bool> prediction = state.solver().predictUpdateInfo(gtsam::NonlinearFactorGraph(), gtsam::Values(), nextParams);
      if (prediction.first.empty()) break;
      result = state.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), nextParams);
      ++updates;
      aggregate.variablesRelinearized += result.variablesRelinearized;
      aggregate.variablesReeliminated += result.variablesReeliminated;
      aggregate.factorsRecalculated += result.factorsRecalculated;
      aggregate.cliques = result.cliques;
      aggregate.treeNnz = result.treeNnz;
      aggregate.batchReorderTriggered = aggregate.batchReorderTriggered || result.batchReorderTriggered;
    }
    followupUpdateSeconds = stageTimer.lap_seconds();
  } catch (const std::exception& exception) {
    EXT_LOG_ERROR("iSAM2 update failed: {}. Incremental state will be rebuilt on the next call.", exception.what());
    state.reset();
    return false;
  }

  if (newFactorIndices.size() != newFactors.size()) {
    EXT_LOG_ERROR("iSAM2 returned {} factor indices for {} new factors.", newFactorIndices.size(), newFactors.size());
    state.reset();
    return false;
  }

  std::map<LinkIdentity, std::vector<IncrementalState::FactorRecord>> pendingRecords;
  for (const PendingGroup& pending : pendingGroups) {
    IncrementalState::FactorRecord record;
    record.indices.insert(record.indices.end(), newFactorIndices.begin() + pending.first, newFactorIndices.begin() + pending.first + pending.count);
    pendingRecords[pending.identity].push_back(std::move(record));
  }
  std::map<LinkIdentity, std::vector<IncrementalState::FactorRecord>> nextRegistry;
  for (const auto& desired : desiredFactors) {
    std::vector<IncrementalState::FactorRecord>& records = nextRegistry[desired.first];
    const auto previous = state.factors.find(desired.first);
    if (previous != state.factors.end()) {
      const std::size_t keep = std::min(previous->second.size(), desired.second.size());
      records.insert(records.end(), previous->second.begin(), previous->second.begin() + keep);
    }
    const auto pending = pendingRecords.find(desired.first);
    if (pending != pendingRecords.end()) records.insert(records.end(), pending->second.begin(), pending->second.end());
  }

  std::pair<gtsam::Values, gtsam::KeySet> estimate = state.solver().calculateEstimateWithAffectedKeys();
  std::vector<std::pair<int, StateGroupf>> poseUpdates;
  poseUpdates.reserve(estimate.second.size());
  for (const auto& pose : poses) {
    const gtsam::Key key = poseKey(pose.first);
    if (!estimate.first.exists(key) || (!estimate.second.count(key) && state.poseCache.count(pose.first))) continue;
    poseUpdates.emplace_back(pose.first, stateFromPose3(estimate.first.at<gtsam::Pose3>(key)));
  }
  const double readbackSeconds = stageTimer.lap_seconds();

  const double error = state.solver().getFactorsUnsafe().error(estimate.first);
  if (!std::isfinite(error) || (options.epsilon > 0.0 && std::abs(error) > 1e12)) {
    EXT_LOG_ERROR("iSAM2 produced invalid/diverging graph error {:.6e}; resetting incremental state.", error);
    state.reset();
    return false;
  }
  const double errorSeconds = stageTimer.lap_seconds();

  try {
    const int covarianceId = poses.count(covariance_target) ? covariance_target : poses.rbegin()->first;
    const gtsam::Matrix covariance = state.solver().marginalCovariance(poseKey(covarianceId));
    if (covariance.rows() != 6 || covariance.cols() != 6) throw std::runtime_error("unexpected marginal covariance dimension");
    outputCovariance = covariance(kPose3TangentOrder, kPose3TangentOrder).eval();
  } catch (const std::exception& exception) {
    EXT_LOG_ERROR("iSAM2 marginal covariance failed: {}. Incremental state will be rebuilt.", exception.what());
    state.reset();
    return false;
  }
  const double marginalSeconds = stageTimer.lap_seconds();

  state.factors.swap(nextRegistry);
  state.rootId = effectiveRootId;
  state.rootFixed = fixed_root;
  state.rootHasGravity = hasGravityConstraints;
  if (replaceRoot) {
    state.rootPose = requestedRootPose;
    state.rootFactorIndex = effectiveRootId == 0 ? std::numeric_limits<gtsam::FactorIndex>::max() : newFactorIndices.at(pendingRootFactor);
  }
  for (auto cache = state.poseCache.begin(); cache != state.poseCache.end();) {
    if (!poses.count(cache->first))
      cache = state.poseCache.erase(cache);
    else
      ++cache;
  }
  for (const auto& update : poseUpdates) {
    if (poses.at(update.first) != update.second) pending_pose_changes().insert(update.first);
    poses.at(update.first) = update.second;
    state.poseCache[update.first] = update.second;
  }
  for (auto& pose : poses) {
    const auto cached = state.poseCache.find(pose.first);
    if (cached == state.poseCache.end())
      state.poseCache.emplace(pose.first, pose.second);
    else {
      if (pose.second != cached->second) pending_pose_changes().insert(pose.first);
      pose.second = cached->second;
    }
  }

  if (finalError) *finalError = error;
  if (iterationsDone) *iterationsDone = updates;
  EXT_LOG_INFO(
      "{}: updates={} newFactors={} removedFactors={} affected={} marked={} reelim={} recalculated={} cliques={} treeNnz={} reordered={} "
      "time[reconcile={:.3f}ms materialize={:.3f}ms update={:.3f}ms followup={:.3f}ms readback={:.3f}ms error={:.3f}ms marginal={:.3f}ms] "
      "error={:.6f}",
      "iSAM2", updates, newFactors.size(), updateParams.removeFactorIndices.size(), estimate.second.size(), aggregate.variablesRelinearized,
      aggregate.variablesReeliminated, aggregate.factorsRecalculated, aggregate.cliques, aggregate.treeNnz, aggregate.batchReorderTriggered ? 1 : 0,
      reconciliationSeconds * 1000.0, materializationSeconds * 1000.0, primaryUpdateSeconds * 1000.0, followupUpdateSeconds * 1000.0,
      readbackSeconds * 1000.0, errorSeconds * 1000.0, marginalSeconds * 1000.0, error);
  return true;
}

std::map<int, StateGroupf> PoseGraphSolver::optimize(int rootId, const std::map<int, StateGroupf>& poses,
                                                     const std::multimap<int, Link>& edgeConstraints,
                                                     std::optional<Mat66_t<double>>& outputCovariance,
                                                     std::list<std::map<int, StateGroupf>>* intermediateGraphs, double* finalError,
                                                     int* iterationsDone) {
  robust_decisions.clear();
  if (iterationsDone) *iterationsDone = 0;
  outputCovariance.reset();
  if (options.robust && options.max_iterations <= 0) {
    EXT_LOG_WARN("[GNC] Cannot validate candidate graph: solver iterations disabled");
    return {};
  }
  std::map<int, StateGroupf> optimizedPoses;
  EXT_LOG_DEBUG("Optimizing graph...");
  if (!edgeConstraints.empty() && poses.size() >= 2 && poses.rbegin()->first > 0 && options.max_iterations > 0) {
    gtsam::NonlinearFactorGraph graph;
    std::map<size_t, Link> uncertainFactors;

    // detect if there is a global pose prior set, if so remove rootId
    bool hasGravityConstraints = false;
    if (!options.ignore_priors || options.gravity_sigma > 0) {
      for (std::multimap<int, Link>::const_iterator iter = edgeConstraints.begin(); iter != edgeConstraints.end(); ++iter) {
        if (iter->second.from() == iter->second.to()) {
          if (!fixed_root && !options.ignore_priors && iter->second.type() == Link::kPosePrior) {
            if (rootId != 0) {
              EXT_LOG_DEBUG("Removed rootId={} because there are pose priors.", rootId);
            }
            rootId = 0;
            break;
          }
          if (iter->second.type() == Link::kGravity) {
            hasGravityConstraints = true;
            if (options.ignore_priors) {
              break;
            }
          }
        }
      }
    }

    // prior first pose
    if (rootId != 0) {
      EXT_LOG_DEBUG("Setting prior for rootId={}", rootId);
      EXT_ASSERT(poses.find(rootId) != poses.end());
      const StateGroupf& initialPose = poses.at(rootId);
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
          fixed_root
              ? gtsam::Vector6::Constant(1e-12)
              : (gtsam::Vector(6) << (hasGravityConstraints ? 2 : 1e-9), (hasGravityConstraints ? 2 : 1e-9), 1e-9, 1e-9, 1e-9, 1e-9).finished());
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(rootId, to_pose3(initialPose), priorNoise));
    }

    const std::map<int, StateGroupf>& newPoses = poses;
    const std::multimap<int, Link>& newEdgeConstraints = edgeConstraints;

    EXT_LOG_DEBUG("fill poses to gtsam... rootId={}", rootId);
    gtsam::Values initialEstimate;
    for (std::map<int, StateGroupf>::const_iterator iter = newPoses.begin(); iter != newPoses.end(); ++iter) {
      EXT_ASSERT(!iter->second.isNull());
      if (iter->first > 0) {
        initialEstimate.insert(iter->first, to_pose3(iter->second));
      }
    }

    EXT_LOG_DEBUG("fill edges to gtsam...");
    for (std::multimap<int, Link>::const_iterator iter = newEdgeConstraints.begin(); iter != newEdgeConstraints.end(); ++iter) {
      const size_t firstFactor = graph.size();
      int id1 = iter->second.from();
      int id2 = iter->second.to();

      EXT_ASSERT_MSG(poses.find(id1) != poses.end(), "{}", fmt::format("id1={} for constraint {}->{} (type={})", id1, id1, id2, iter->second.type()));
      EXT_ASSERT_MSG(poses.find(id2) != poses.end(), "{}", fmt::format("id2={} for constraint {}->{} (type={})", id2, id1, id2, iter->second.type()));

      EXT_ASSERT(!iter->second.transform().isNull());
      if (id1 == id2) {
        if (iter->second.type() == Link::kPosePrior && !options.ignore_priors) {
          if (1 / static_cast<double>(iter->second.infMatrix()(3, 3)) >= 9999.0 ||
              1 / static_cast<double>(iter->second.infMatrix()(4, 4)) >= 9999.0 ||
              1 / static_cast<double>(iter->second.infMatrix()(5, 5)) >= 9999.0) {
            gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Precisions(
                gtsam::Vector3(iter->second.infMatrix()(0, 0), iter->second.infMatrix()(1, 1), iter->second.infMatrix()(2, 2)));
            graph.add(PositionFactor(id1, gtsam::Point3(iter->second.transform().translationVector().cast<double>()), model));
          } else {
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
            if (!options.ignore_covariance) {
              information = iter->second.infMatrix();
            }

            const Mat66_t<double> mgtsam = information(kPose3TangentOrder, kPose3TangentOrder).eval();
            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);

            graph.add(gtsam::PriorFactor<gtsam::Pose3>(id1, to_pose3(iter->second.transform()), model));
          }
        } else if (options.gravity_sigma > 0 && iter->second.type() == Link::kGravity && newPoses.find(iter->first) != newPoses.end()) {
          gtsam::Rot3 nRbMeas = to_pose3(iter->second.transform()).rotation();
          gtsam::Unit3 nZ(0, 0, 1);
          gtsam::Unit3 bGMeas = nRbMeas.unrotate(nZ);
          gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(2, options.gravity_sigma);
          graph.add(gtsam::AttitudeFactor<gtsam::Pose3>(iter->first, nZ, model, bGMeas));
        }
      } else  // id1 != id2
      {
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        if (!options.ignore_covariance) {
          information = iter->second.infMatrix();
        }

        const Mat66_t<double> mgtsam = information(kPose3TangentOrder, kPose3TangentOrder).eval();
        gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);

        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id1, id2, to_pose3(iter->second.transform()), model));
      }
      if (iter->second.isLoopClosure()) {
        for (size_t factor = firstFactor; factor < graph.size(); ++factor) uncertainFactors.emplace(factor, iter->second);
      }
    }

    gtsam::Values values;
    if (options.robust) {
      tools::Timer gncTimer;
      gtsam::GncParams<gtsam::LevenbergMarquardtParams> parameters;
      parameters.lossType = gtsam::TLS;
      parameters.baseOptimizerParams.maxIterations = options.max_iterations;
      parameters.baseOptimizerParams.relativeErrorTol = options.epsilon;
      // Datum, odometry and absolute priors are trusted; verify candidate loops.
      for (size_t factor = 0; factor < graph.size(); ++factor)
        if (!uncertainFactors.count(factor)) parameters.knownInliers.push_back(factor);
      try {
        gtsam::GncOptimizer<decltype(parameters)> gnc(graph, initialEstimate, parameters);
        values = gnc.optimize();
        const auto& weights = gnc.getWeights();
        bool accepted = true;
        for (const auto& item : uncertainFactors) {
          const double weight = weights(Eigen::Index(item.first));
          const bool supported = std::isfinite(weight) && std::abs(1.0 - weight) <= parameters.weightsTol;
          robust_decisions.push_back({item.second, weight, supported});
          accepted = accepted && supported;
          if (!supported)
            EXT_LOG_WARN("[GNC] Constraint {}->{} type={} weight={:.6f}; rejecting candidate graph", item.second.from(), item.second.to(),
                         int(item.second.type()), weight);
        }
        if (iterationsDone) *iterationsDone = int(gnc.getTiming().iterations.size()) + 1;
        if (finalError) *finalError = graph.error(values);
        EXT_LOG_INFO("[GNC] factors={} uncertain={} accepted={} ms={:.3f}", graph.size(), uncertainFactors.size(), accepted,
                     gncTimer.elapsed_seconds() * 1000);
        if (!accepted || !std::isfinite(graph.error(values))) return {};
      } catch (const std::exception& error) {
        EXT_LOG_WARN("[GNC] Candidate solve failed: {}", error.what());
        return {};
      }
    } else {
      EXT_LOG_DEBUG("create optimizer");
      std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

      EXT_LOG_DEBUG("Batch optimization...");
      if (internalOptimizerType_ == 2) {
        gtsam::DoglegParams parameters;
        parameters.relativeErrorTol = options.epsilon;
        parameters.maxIterations = options.max_iterations;
        optimizer.reset(new gtsam::DoglegOptimizer(graph, initialEstimate, parameters));
      } else if (internalOptimizerType_ == 1) {
        gtsam::GaussNewtonParams parameters;
        parameters.relativeErrorTol = options.epsilon;
        parameters.maxIterations = options.max_iterations;
        optimizer.reset(new gtsam::GaussNewtonOptimizer(graph, initialEstimate, parameters));
      } else {
        gtsam::LevenbergMarquardtParams parameters;
        parameters.relativeErrorTol = options.epsilon;
        parameters.maxIterations = options.max_iterations;
        optimizer.reset(new gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, parameters));
      }

      EXT_LOG_DEBUG("GTSAM optimizing begin (max iterations={}, robust={})", options.max_iterations, options.robust ? 1 : 0);
      tools::Timer timer;
      int it = 0;
      double initialError = optimizer ? graph.error(initialEstimate) : 0;
      double lastError = optimizer ? optimizer->error() : 0;
      for (int i = 0; i < options.max_iterations; ++i) {
        if (intermediateGraphs && i > 0) {
          std::map<int, StateGroupf> tmpPoses;
          const gtsam::Values values = optimizer->values();
          for (gtsam::Values::deref_iterator iter = values.begin(); iter != values.end(); ++iter) {
            int key = (int)iter->key;
            if (iter->value.dim() > 1 && newPoses.find(key) != newPoses.end()) {
              if (key > 0) {
                gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
                tmpPoses.insert(std::make_pair(key, stateFromPose3(p)));
              }
            }
          }
          intermediateGraphs->push_back(tmpPoses);
        }

        double error = 0;
        try {
          optimizer->iterate();
          error = optimizer->error();
          ++it;
        } catch (gtsam::IndeterminantLinearSystemException& e) {
          EXT_LOG_WARN("GTSAM exception caught: {}\n Graph has {} edges and {} vertices", e.what(), (int)newEdgeConstraints.size(),
                       (int)newPoses.size());
          return optimizedPoses;
        }

        // early stop condition
        EXT_LOG_DEBUG("iteration {} error ={:.6f}", i + 1, error);
        double errorDelta = lastError - error;
        if (options.epsilon > 0.0 && fabs(error) > 1000000000000.0) {
          EXT_LOG_WARN(
              "Error computed ({:.6e}) is very huge and/or diverging! Aborting! "
              "Set {} to 0 to ignore that check and keep iterating up to {} ({}).",
              error, "pose_graph.min_error_improvement", "pose_graph.max_iterations", options.max_iterations);
          return optimizedPoses;
        } else {
          if (i > 0 && errorDelta < options.epsilon) {
            if (errorDelta < 0) {
              EXT_LOG_DEBUG("Negative improvement?! Ignore and continue optimizing... ({:.6f} < {:.6f})", errorDelta, options.epsilon);
            } else {
              EXT_LOG_DEBUG("Stop optimizing, not enough improvement ({:.6f} < {:.6f})", errorDelta, options.epsilon);
              break;
            }
          } else if (i == 0) {
            if (error < 0) {
              EXT_LOG_DEBUG("Negative error?! Ignore and continue optimizing... ({:.6f})", error);
            } else if (error < options.epsilon) {
              EXT_LOG_INFO("Stop optimizing, error is already under epsilon ({:.6f} < {:.6f})", error, options.epsilon);
              break;
            }
          }
        }
        lastError = error;
      }
      if (finalError) {
        *finalError = lastError;
      }
      if (iterationsDone) {
        *iterationsDone = it;
      }
      EXT_LOG_DEBUG("GTSAM optimizing end ({} iterations done (error initial={:.6f} final={:.6f}), time={:.6f} s)", it, initialError, lastError,
                    timer.lap_seconds());

      values = optimizer->values();
    }
    if (!std::isfinite(graph.error(values))) return {};
    if (finalError) *finalError = graph.error(values);
    for (gtsam::Values::deref_iterator iter = values.begin(); iter != values.end(); ++iter) {
      int key = (int)iter->key;
      if (iter->value.dim() > 1 && poses.find(key) != poses.end()) {
        if (key > 0) {
          gtsam::Pose3 p = iter->value.cast<gtsam::Pose3>();
          optimizedPoses.insert(std::make_pair(key, stateFromPose3(p)));
        }
      }
    }

    // compute marginals
    try {
      EXT_LOG_DEBUG("Computing marginals for node {}...", poses.rbegin()->first);
      tools::Timer t;
      const gtsam::Marginals marginals(graph, values);
      const gtsam::Matrix info = marginals.marginalCovariance(poses.count(covariance_target) ? covariance_target : poses.rbegin()->first);
      EXT_LOG_DEBUG("Computed marginals = {:.6f}s (key={})", t.lap_seconds(), poses.rbegin()->first);
      if (info.cols() == 6 && info.rows() == 6) {
        const Mat66_t<double> mgtsam = info(kPose3TangentOrder, kPose3TangentOrder).eval();
        outputCovariance = mgtsam;
      } else {
        EXT_LOG_ERROR("GTSAM: Could not compute marginal covariance!");
        optimizedPoses.clear();  // Failed optimization
      }
    } catch (gtsam::IndeterminantLinearSystemException& e) {
      EXT_LOG_ERROR("GTSAM exception caught: {}", e.what());
      optimizedPoses.clear();  // Failed optimization
    } catch (std::exception& e) {
      EXT_LOG_ERROR("GTSAM exception caught: {}", e.what());
      optimizedPoses.clear();  // Failed optimization
    }

  } else if (poses.size() == 1 || options.max_iterations <= 0) {
    optimizedPoses = poses;
  } else {
    EXT_LOG_WARN("This method should be called at least with 1 pose!");
  }
  EXT_LOG_DEBUG("Optimizing graph...end!");
  return optimizedPoses;
}

}  // namespace sapphire::backend::solver
