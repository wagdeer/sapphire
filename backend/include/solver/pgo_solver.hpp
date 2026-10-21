#pragma once
#include <backend/common/common_type.hpp>
#include <backend/maplayer/topograph/link.hpp>
#include <backend/params/pgo.hpp>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace sapphire::backend::solver {

class PoseGraphSolver final {
 public:
  struct Options {
    int max_iterations = sapphire::backend::PGOParams{}.max_iterations;
    bool ignore_covariance = sapphire::backend::PGOParams{}.ignore_constraint_variance;
    double epsilon = sapphire::backend::PGOParams{}.min_error_improvement;
    bool robust = sapphire::backend::PGOParams{}.gnc_enabled;
    bool ignore_priors = sapphire::backend::PGOParams{}.ignore_priors;
    float gravity_sigma = sapphire::backend::PGOParams{}.gravity_sigma;
  };
  Options options;
  int covariance_target = 0;
  bool fixed_root = false;

  PoseGraphSolver(int iterations = sapphire::backend::PGOParams{}.max_iterations,
                  bool covarianceIgnored = sapphire::backend::PGOParams{}.ignore_constraint_variance,
                  double epsilon = sapphire::backend::PGOParams{}.min_error_improvement, bool robust = sapphire::backend::PGOParams{}.gnc_enabled,
                  bool priorsIgnored = sapphire::backend::PGOParams{}.ignore_priors,

                  float gravitySigma = sapphire::backend::PGOParams{}.gravity_sigma);
  PoseGraphSolver(const ParametersMap& parameters);
  explicit PoseGraphSolver(const PGOParams& configuration);

  ~PoseGraphSolver();
  std::set<int> swap_results() {
    std::set<int> result;
    result.swap(changed_pose_ids_);
    return result;
  }
  PoseGraphSolver(const PoseGraphSolver&) = delete;
  PoseGraphSolver& operator=(const PoseGraphSolver&) = delete;

  class Trial {
   public:
    explicit Trial(PoseGraphSolver& optimizer);
    ~Trial();
    Trial(const Trial&) = delete;
    Trial& operator=(const Trial&) = delete;
    void commit() noexcept;

   private:
    PoseGraphSolver* optimizer_;
  };

  struct RobustConstraintDecision {
    Link link;
    double weight;
    bool accepted;
  };
  std::vector<RobustConstraintDecision> robust_decisions;

  std::map<int, StateGroupf> optimize(int rootId, const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                                      std::list<std::map<int, StateGroupf>>* intermediateGraphs = nullptr, double* finalError = nullptr,
                                      int* iterationsDone = nullptr);

  std::map<int, StateGroupf> optimize(int rootId, const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                                      std::optional<Mat66_t<double>>& outputCovariance,
                                      std::list<std::map<int, StateGroupf>>* intermediateGraphs = nullptr, double* finalError = nullptr,
                                      int* iterationsDone = nullptr);

  bool optimizeInPlace(int rootId, std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                       std::optional<Mat66_t<double>>& outputCovariance, double* finalError = nullptr, int* iterationsDone = nullptr);

 private:
  int internalOptimizerType_;

  std::set<int> changed_pose_ids_;
  struct IncrementalState;
  std::unique_ptr<IncrementalState> incrementalState_;
  std::unique_ptr<IncrementalState> trialState_;
  bool trialActive_ = false;
  std::set<int> trial_pose_changes_;
  std::set<int>& pending_pose_changes() { return trialActive_ ? trial_pose_changes_ : changed_pose_ids_; }

  bool optimizeISAM2(int rootId, std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints,
                     std::optional<Mat66_t<double>>& outputCovariance, double* finalError, int* iterationsDone);
};

}  // namespace sapphire::backend::solver
