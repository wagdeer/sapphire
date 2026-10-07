#pragma once

#include "common/visual_frame.hpp"

namespace sapphire::database_detail {
// Stateless ImageRecord.points codec. Legacy rows remain readable; extended
// VOM family v1 preserves original camera poses and visual-only geometry;
// v2 additionally freezes the rectified-pixel projection. Neither uses current calibration.
std::vector<std::uint8_t> packVisualObservation(const VisualFrame &frame);
VisualFrame readVisualObservation(double timestamp, std::size_t camera, std::size_t count,
    const void *data, std::size_t bytes, cv::Mat descriptors);
}  // namespace sapphire::database_detail
