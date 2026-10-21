#pragma once
#include "backend/maplayer/featmap/observations.hpp"

namespace sapphire::backend::featmap::detail {
void validate_local_poses(const NodeObservations&);
void validate_bundle_values(const NodeObservations&);
std::map<uint64_t, uint32_t> validate_bundle_structure(const NodeObservations&);
}  // namespace sapphire::backend::featmap::detail
