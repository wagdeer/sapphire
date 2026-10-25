#pragma once

#include <chrono>

namespace sapphire::tools {

using SteadyClock = std::chrono::steady_clock;
using WallClock = std::chrono::system_clock;
using SteadyDuration = SteadyClock::duration;
using SteadyTimePoint = SteadyClock::time_point;
using WallTimePoint = WallClock::time_point;

// In-process access ordering; never persist this value as a capture timestamp.
inline double steady_time_seconds() noexcept { return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count(); }
inline WallTimePoint wall_time() noexcept { return WallClock::now(); }
inline double wall_time_seconds(WallTimePoint time) noexcept { return std::chrono::duration<double>(time.time_since_epoch()).count(); }
inline double wall_time_seconds() noexcept { return wall_time_seconds(wall_time()); }

class Timer {
 public:
  Timer() noexcept = default;

  void reset() noexcept { started_at_ = SteadyClock::now(); }

  SteadyDuration elapsed() const noexcept { return SteadyClock::now() - started_at_; }

  double elapsed_seconds() const noexcept { return std::chrono::duration<double>(elapsed()).count(); }

  double elapsed_milliseconds() const noexcept { return std::chrono::duration<double, std::milli>(elapsed()).count(); }

  SteadyDuration lap() noexcept {
    const auto now = SteadyClock::now();
    const auto duration = now - started_at_;
    started_at_ = now;
    return duration;
  }

  double lap_seconds() noexcept { return std::chrono::duration<double>(lap()).count(); }

 private:
  SteadyTimePoint started_at_ = SteadyClock::now();
};

}  // namespace sapphire::tools
