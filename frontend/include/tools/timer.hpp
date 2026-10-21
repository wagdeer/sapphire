#pragma once

#include <chrono>
#include <cstdlib>
#include <cstring>

namespace sapphire::timer {
using Clock = std::chrono::steady_clock;
inline bool enabled() {
  static const bool value = [] {
    // The generic switch takes precedence; retain existing profiling scripts.
    const char* setting = std::getenv("SAPPHIRE_PROFILE");
    if (!setting) setting = std::getenv("SAPPHIRE_PROFILE_BACKEND");
    return setting && std::strcmp(setting, "1") == 0;
  }();
  return value;
}
inline double ms(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
}  // namespace sapphire::timer
