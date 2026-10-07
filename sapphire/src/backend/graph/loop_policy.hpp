#pragma once
#include <algorithm>
#include <cstdint>

namespace sapphire::loop_policy {
// Native Policy C / schema 1 semantics. Not a workload/configuration parameter.
inline constexpr std::uint64_t temporal_exclusion = 3;
// All identities are zero-based keys; roots must come from validated type-0 topology.
inline constexpr bool eligible(std::uint64_t query, std::uint64_t target, std::uint64_t query_root,
                               std::uint64_t target_root, std::uint64_t committed_count) noexcept {
  return target < committed_count && target < query &&
         (query_root != target_root || query - target > temporal_exclusion);
}
inline constexpr std::uint64_t prefix(std::uint64_t query, std::uint64_t root) noexcept {
  return std::max(root, query > temporal_exclusion ? query - temporal_exclusion : 0);
}
}  // namespace sapphire::loop_policy
