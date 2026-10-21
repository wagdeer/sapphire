#pragma once
#include <backend/common/visual_frame.hpp>
#include <backend/registration/matcher/match.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sapphire::backend {

class KeyframeGate {
 public:
  // Geometry is evaluated without changing protocol state or estimating a pose.
  struct Geometry {
    bool checked = false, redundant = false;
    size_t image_features = 0, features = 0, matches = 0, supported_cells = 0, new_cells = 0, inconsistent_depth = 0;
    uint64_t comparisons = 0, spatial_visits = 0;
  };
  static Geometry assess(const VisualFeatures& reference, const camera::Perspective& reference_camera,
                         const VisualFeatures& query, const camera::Perspective& query_camera,
                         const StateGroupf& query_from_reference, const std::vector<double>& variances,
                         const MatchOptions&);

  enum class VisionState : uint8_t { normal = 0, degraded = 1, init = 2, invalid = 3 };
  // Frontend stereo health is diagnostic only. Admission uses valid poses,
  // protocol and measured geometry; auxiliary-camera takeover is frontend-owned.
  struct Observation {
    uint64_t session = 0;
    double stamp = 0;
    uint64_t keyframe_sequence = 0;
    VisionState state = VisionState::init;
    bool is_keyframe = false;
  };
  struct Decision { bool insert = false; bool healthy = false; };
  Decision evaluate(const Observation& observation) const {
    if (!observation.session || !std::isfinite(observation.stamp) || observation.stamp <= 0) return {};
    const bool same_session = session_ == observation.session;
    if (same_session && (observation.stamp <= stamp_ || observation.keyframe_sequence < sequence_)) return {};
    return {observation.is_keyframe && observation.keyframe_sequence != 0 &&
            (!same_session || observation.keyframe_sequence > sequence_), true};
  }
  void accept(const Observation& observation) {
    if (!evaluate(observation).insert) throw std::invalid_argument("Invalid frontend keyframe reference");
    session_ = observation.session;
    sequence_ = observation.keyframe_sequence;
    stamp_ = observation.stamp;
  }
  void reset() { session_ = sequence_ = 0; stamp_ = 0; }
  // Last consumed frontend proposal, including an absorbed proposal. The map
  // reference is resolved independently by Mapping from its committed owners.
  double reference_stamp() const { return stamp_; }
 private:
  uint64_t session_ = 0, sequence_ = 0;
  double stamp_ = 0;
};
} // namespace sapphire::backend
