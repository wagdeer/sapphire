#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <optional>

namespace sapphire::backend::tools {

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

// Bounded, single-thread collection. Observers and rendering belong to callers.
// A trace must outlive its scopes and steps; labels must outlive the trace.
class EventTrace {
 public:
  using Clock = SteadyClock;

  enum class Status { Pending, Completed, Failed };

  struct Step {
    const char* stage = "";
    const char* label = ""; // Static strings owned by the call site.
    Status status = Status::Pending;
    double elapsed_ms = 0;
  };
  using Observer = void (*)(void*, const Step&, bool timed) noexcept;

  explicit EventTrace(bool timed = true, Observer observer = nullptr, void* context = nullptr) noexcept
      : timed_(timed), observer_(observer), context_(context) {
    if (timed_) timer_.emplace();
  }

  EventTrace(const EventTrace&) = delete;
  EventTrace& operator=(const EventTrace&) = delete;

  void finish() noexcept {
    if (finished_) return;
    if (timed_) elapsed_ms_ = timer_->elapsed_milliseconds();
    finished_ = true;
  }

  bool timed() const noexcept { return timed_; }
  double elapsed_ms() const noexcept { return elapsed_ms_; }
  size_t size() const noexcept { return size_; }
  size_t omitted() const noexcept { return omitted_; }
  const Step& operator[](size_t i) const noexcept { return steps_[i]; }

 private:
  friend class TimedStep;

  std::array<Step, 48> steps_{};
  size_t size_ = 0, omitted_ = 0;
  bool timed_, finished_ = false;
  Observer observer_;
  void* context_;
  // No Timer construction or clock read when timing is disabled.
  std::optional<Timer> timer_;
  double elapsed_ms_ = 0;
};

// Synchronous descendants inherit the current trace; unrelated threads do not.
inline thread_local EventTrace* currentTrace = nullptr;
class TraceScope {
 public:
  explicit TraceScope(EventTrace& trace) noexcept : previous_(currentTrace) { currentTrace = &trace; }

  ~TraceScope() { currentTrace = previous_; }

  TraceScope(const TraceScope&) = delete;
  TraceScope& operator=(const TraceScope&) = delete;

 private:
  EventTrace* previous_;
};

class TimedStep {
 public:
  TimedStep(const char* stage, const char* label, EventTrace* trace = currentTrace) noexcept : trace_(trace) {
    if (!trace_ || trace_->finished_) { trace_ = nullptr; return; }
    if (trace_->size_ == trace_->steps_.size()) { ++trace_->omitted_; trace_ = nullptr; return; }
    index_ = trace_->size_++;
    trace_->steps_[index_] = {stage, label, EventTrace::Status::Pending, 0};
    notify();
    // Measure work, excluding observer notification time.
    if (trace_->timed_) timer_.emplace();
  }

  ~TimedStep() { finish(EventTrace::Status::Failed); }

  void complete() noexcept { finish(EventTrace::Status::Completed); }

  TimedStep(const TimedStep&) = delete;
  TimedStep& operator=(const TimedStep&) = delete;

 private:
  void notify() noexcept {
    if (trace_->observer_) trace_->observer_(trace_->context_, trace_->steps_[index_], trace_->timed_);
  }

  void finish(EventTrace::Status status) noexcept {
    if (!trace_) return;
    auto& step = trace_->steps_[index_];
    if (trace_->timed_) step.elapsed_ms = timer_->elapsed_milliseconds();
    step.status = status;
    notify();
    trace_ = nullptr;
  }

  EventTrace* trace_;
  size_t index_ = 0;
  std::optional<Timer> timer_;
};

}  // namespace sapphire::backend::tools
