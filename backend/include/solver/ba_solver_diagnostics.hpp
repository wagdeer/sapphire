#pragma once
// Private instrumentation for numerical equivalence tests and offline benchmarks.
// This header is deliberately not installed.
#include <functional>

#include "backend/solver/ba_solver.hpp"
namespace sapphire::backend::solver {
// Optional diagnostic snapshot. IDs are caller labels, never optimizer keys.
struct BASnapshot {
  struct Pose {
    Eigen::Matrix4d initial, solved;
    bool fixed = false;
  };
  struct Point {
    Eigen::Vector3d initial, solved;
  };
  struct Observation {
    uint64_t landmark;
    int observer;
    camera::StereoObservation measurement;
  };
  struct Summary {
    uint64_t landmark;
    std::optional<int> observer;  // Empty: fixed Node-local history.
    featmap::PointSummary measurement;
  };
  int pass = 0;
  bool robust = false;
  double initial_error = 0, solved_error = 0, optimize_ms = 0;
  size_t iterations = 0;
  std::map<int, Pose> poses;
  std::map<uint64_t, Point> points;
  std::vector<Observation> observations;
  std::vector<Summary> summaries;
};
struct BADiagnostics {
  bool reference_solver = false, reference_marginals = false;
  std::function<void(const BASnapshot&)> inspect_problem;
  double setup_ms = 0, factors_ms = 0, optimize_ms = 0, uncertainty_ms = 0, check_ms = 0, elapsed_ms = 0;
  double linearize_ms = 0, error_ms = 0, assemble_ms = 0, solve_ms = 0;
};
}  // namespace sapphire::backend::solver
