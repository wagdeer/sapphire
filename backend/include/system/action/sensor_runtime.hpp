#pragma once

#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/registration/fast_orb_stereo.hpp>
#include <map>
#include <memory>
#include <optional>

namespace sapphire::backend {

class SensorRuntime {
 public:
  using Id = int64_t;
  using Slot = size_t;

  // Physical-camera IDs and pair IDs are external identities, not storage slots.
  struct Camera {
    Id id;
    camera::Perspective model;
    std::optional<std::array<double, 4>> kb4;
  };

  struct Pair {
    Id id, left, right;
    camera::Perspective rectified_left;
  };

  struct Channel {
    Id camera;
    std::optional<Id> pair;
    int feature_budget;
  };

  SensorRuntime(std::vector<Camera> cameras, std::vector<Pair> pairs, std::vector<Channel> channels, const ParametersMap& parameters);
  Slot camera_slot(Id id) const { return camera_slots_.at(id); }
  Slot pair_slot(Id id) const { return pair_slots_.at(id); }
  Slot channel_slot(Id id) const { return channel_slots_.at(id); }
  const Camera& camera(Slot slot) const { return cameras_.at(slot); }
  const Pair& pair(Slot slot) const { return pairs_.at(slot); }
  const StateGroupf& right_from_left(Slot slot) const { return relative_.at(slot); }
  const std::vector<Camera>& cameras() const { return cameras_; }
  FastOrbStereo& extractor(Slot slot) { return *extractors_.at(slot); }
  const camera::Perspective& model(Slot slot) const { return models_.at(slot); }
  bool stereo(Slot slot) const { return channels_.at(slot).pair.has_value(); }
  void validate_parameters(const ParametersMap& parameters) const;
  void validate_parameters(const FeatureParams& feature, unsigned int decimation, cv::Size stereo_size = {}) const;

 private:
  const std::vector<Camera> cameras_;
  const std::vector<Pair> pairs_;
  std::vector<Channel> channels_;
  std::map<Id, Slot> camera_slots_, pair_slots_, channel_slots_;
  std::vector<StateGroupf> relative_;
  std::vector<camera::Perspective> models_;
  std::vector<std::unique_ptr<FastOrbStereo>> extractors_;
  ParametersMap parameters_;
};

}  // namespace sapphire::backend
