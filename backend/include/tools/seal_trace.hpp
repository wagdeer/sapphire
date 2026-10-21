#pragma once

// Opt-in diagnostic checkpoints. No public ABI changes or per-stage file I/O.
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

namespace sapphire::backend::detail {
class SealTrace {
 public:
  SealTrace(const char* operation, const std::string& key) : operation_(operation), key_(key) {
    if (enabled()) mark("begin");
  }
  static bool enabled() { return !path().empty(); }
  void mark(const char* stage) {
    if (!enabled() || count_ == samples_.size()) return;
    auto& s = samples_[count_++];
    s.stage = stage;
    s.wall = now(CLOCK_MONOTONIC);
    s.realtime = now(CLOCK_REALTIME);
    s.thread = now(CLOCK_THREAD_CPUTIME_ID);
    s.process = now(CLOCK_PROCESS_CPUTIME_ID);
    s.tid = ::syscall(SYS_gettid);
    ::getrusage(RUSAGE_THREAD, &s.usage);
  }
  ~SealTrace() noexcept {
    if (count_ < 2) return;
    try {
      static std::mutex mutex;
      const std::lock_guard<std::mutex> lock(mutex);
      std::ofstream out(path(), std::ios::app);
      out << std::fixed << std::setprecision(6);
      for (size_t i = 1; i < count_; ++i) {
        const auto& a = samples_[i - 1];
        const auto& b = samples_[i];
        const bool same = a.tid == b.tid;
        // operation,key,stage,end_epoch_s,end_monotonic_s,wall_ms,thread_cpu_ms,process_cpu_ms,
        // start_tid,end_tid,voluntary_switches,involuntary_switches,major_faults
        out << operation_ << ',' << std::quoted(key_) << ',' << b.stage << ',' << b.realtime << ',' << b.wall << ','
            << (b.wall - a.wall) * 1000 << ',' << (same ? (b.thread - a.thread) * 1000 : -1) << ','
            << (b.process - a.process) * 1000 << ',' << a.tid << ',' << b.tid << ','
            << (same ? b.usage.ru_nvcsw - a.usage.ru_nvcsw : -1) << ','
            << (same ? b.usage.ru_nivcsw - a.usage.ru_nivcsw : -1) << ','
            << (same ? b.usage.ru_majflt - a.usage.ru_majflt : -1) << '\n';
      }
    } catch (...) {
      // Diagnostic output must never change archive success/failure semantics.
    }
  }

 private:
  static const std::string& path() {
    static const std::string value = [] {
      const char* p = std::getenv("PDR_VDB_TRACE");
      return p ? std::string(p) : std::string();
    }();
    return value;
  }
  static double now(clockid_t clock) {
    timespec t{};
    ::clock_gettime(clock, &t);
    return double(t.tv_sec) + double(t.tv_nsec) * 1e-9;
  }
  struct Sample {
    const char* stage = nullptr;
    double wall = 0, realtime = 0, thread = 0, process = 0;
    long tid = 0;
    rusage usage{};
  };
  const char* operation_;
  const std::string& key_;
  std::array<Sample, 16> samples_;
  size_t count_ = 0;
};
}  // namespace sapphire::backend::detail
