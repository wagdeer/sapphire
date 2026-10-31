#pragma once

#include <sapphire/backend/pose_graph.hpp>

#include <cstddef>
#include <filesystem>

namespace sapphire {

struct TumWriteResult {
    size_t written = 0;
    size_t skipped = 0;
};

/// Write a TUM RGB-D trajectory:
/// `timestamp tx ty tz qx qy qz qw`.
///
/// `qx qy qz qw` is the stored pose rotation, in that order. The components
/// are not renormalized and not reordered to `wxyz`. A non-finite pose, or a
/// timestamp that does not strictly increase, is skipped so the file stays
/// readable by evo. An empty result does not create `path`.
/// Throws std::runtime_error when the file cannot be written.
TumWriteResult writeTumTrajectory(
    const std::filesystem::path& path,
    const MapTrajectory& trajectory);

}  // namespace sapphire
