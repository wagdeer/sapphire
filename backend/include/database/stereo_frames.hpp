#pragma once

#include <cstring>
#include <map>
#include <utility>
#include "backend/common/stereo_frame.hpp"

namespace sapphire::backend::camera {
// Temporary decoder staging. Legacy rows repeat camera parameters; restore one
// immutable frame per capture/model without changing any measured float bits.
class StereoArchiveFrames {
 public:
  void add(int capture, StateGroupf pose, std::array<float, 5> calibration, StereoPixel pixel) {
    std::array<uint32_t, 17> bits{};
    std::memcpy(bits.data(), pose.coefficients().data(), 12 * sizeof(float));
    std::memcpy(bits.data() + 12, calibration.data(), 5 * sizeof(float));
    const auto [it, inserted] = index_.emplace(std::make_pair(capture, bits), frames_.size());
    if (inserted) frames_.push_back({pose, calibration, {}});
    auto& frame = frames_[it->second];
    rows_.push_back({it->second, frame.pixels.size()});
    frame.pixels.push_back(pixel);
  }
  std::vector<StereoObservation> finish() && {
    std::vector<std::shared_ptr<const StereoFrame>> frames;
    frames.reserve(frames_.size());
    for (auto& draft : frames_) frames.push_back(std::make_shared<const StereoFrame>(draft.pose, draft.calibration, std::move(draft.pixels)));
    std::vector<StereoObservation> out;
    out.reserve(rows_.size());
    for (const auto& [frame, row] : rows_) out.emplace_back(frames[frame], row);
    return out;
  }
 private:
  struct Draft { StateGroupf pose; std::array<float, 5> calibration; std::vector<StereoPixel> pixels; };
  std::map<std::pair<int, std::array<uint32_t, 17>>, size_t> index_;
  std::vector<Draft> frames_;
  std::vector<std::pair<size_t, size_t>> rows_;
};
}  // namespace sapphire::backend::camera
