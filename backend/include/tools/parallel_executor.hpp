#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "backend/tools/simd.hpp"

namespace sapphire::backend::tools {

namespace detail {

class ParallelExecutorCore {
 public:
  explicit ParallelExecutorCore(size_t thread_count);
  ~ParallelExecutorCore() noexcept;

  ParallelExecutorCore(const ParallelExecutorCore &) = delete;
  ParallelExecutorCore &operator=(const ParallelExecutorCore &) = delete;
  ParallelExecutorCore(ParallelExecutorCore &&) = delete;
  ParallelExecutorCore &operator=(ParallelExecutorCore &&) = delete;

  size_t thread_count() const noexcept { return thread_count_; }
  size_t background_dispatch_count() const noexcept;

  // Dispatch descriptors and callable tuples live in the submitting thread's
  // stack frame. Executor dispatch itself performs no heap allocation after
  // construction; allocations made by user callables are outside this count.
  static constexpr size_t dispatch_heap_allocation_count() noexcept { return 0; }

  // Independent external submissions are serialized. A submission made from
  // any task already running on this executor uses caller-runs execution with
  // worker index zero and returns one active worker, avoiding nested deadlock.
  template <typename Function>
  size_t parallel_for(size_t item_count, size_t minimum_grain, Function &&function) {
    using FunctionType = std::remove_reference_t<Function>;
    static_assert(std::is_invocable_v<FunctionType &, size_t, size_t, size_t>, "parallel_for callable must accept (worker_index, begin, end)");

    const size_t grain = minimum_grain == 0 ? 1 : minimum_grain;
    const size_t useful_workers = item_count == 0 ? 0 : (item_count / grain == 0 ? 1 : item_count / grain);
    size_t active_workers = useful_workers < thread_count_ ? useful_workers : thread_count_;
    if (active_workers > 1 && is_worker_context()) {
      active_workers = 1;
    }
    RangeJob<FunctionType> job(function, item_count, active_workers);
    return dispatch(job, active_workers);
  }

  template <typename... Callables, std::enable_if_t<(std::is_invocable_v<std::remove_reference_t<Callables> &> && ...), int> = 0>
  size_t parallel_invoke(Callables &&...callables) {
    using Tuple = std::tuple<Callables...>;
    Tuple tasks(std::forward<Callables>(callables)...);
    constexpr size_t task_count = sizeof...(Callables);
    size_t active_workers = task_count < thread_count_ ? task_count : thread_count_;
    if (active_workers > 1 && is_worker_context()) {
      active_workers = 1;
    }
    InvokeJob<Tuple, task_count> job(tasks);
    return dispatch(job, active_workers);
  }

  // Runtime-sized heterogeneous batches can map each task index to a distinct
  // operation in the callable. Only task_count entries are scheduled, so empty
  // and partially populated batches do not consume workers for no-op tasks.
  template <typename Function>
  size_t parallel_invoke(size_t task_count, Function &&function) {
    using FunctionType = std::remove_reference_t<Function>;
    static_assert(std::is_invocable_v<FunctionType &, size_t>, "indexed parallel_invoke callable must accept task_index");

    size_t active_workers = task_count < thread_count_ ? task_count : thread_count_;
    if (active_workers > 1 && is_worker_context()) {
      active_workers = 1;
    }
    IndexedInvokeJob<FunctionType> job(function, task_count);
    return dispatch(job, active_workers);
  }

  // Idempotently rejects future submissions, waits for an accepted dispatch,
  // and joins background workers. Calling this from this executor's task
  // throws std::logic_error instead of deadlocking.
  void shutdown();

 private:
  class Job {
   public:
    virtual ~Job() = default;
    virtual void execute(size_t worker_index) noexcept = 0;

    void capture_exception() noexcept;
    void rethrow_if_exception();

   private:
    std::mutex exception_mutex_;
    std::exception_ptr exception_;
  };

  template <typename Function>
  class RangeJob final : public Job {
   public:
    RangeJob(Function &function, size_t item_count, size_t active_workers) : function_(function), item_count_(item_count), active_workers_(active_workers) {}

    void execute(size_t worker_index) noexcept override {
      if (active_workers_ == 0) {
        return;
      }
      const size_t base_size = item_count_ / active_workers_;
      const size_t remainder = item_count_ % active_workers_;
      const size_t begin = worker_index * base_size + (worker_index < remainder ? worker_index : remainder);
      const size_t end = begin + base_size + (worker_index < remainder ? 1 : 0);
      try {
        std::invoke(function_, worker_index, begin, end);
      } catch (...) {
        this->capture_exception();
      }
    }

   private:
    Function &function_;
    const size_t item_count_;
    const size_t active_workers_;
  };

  template <typename Tuple, size_t TaskCount>
  class InvokeJob final : public Job {
   public:
    explicit InvokeJob(Tuple &tasks) : tasks_(tasks) {}

    void execute(size_t) noexcept override {
      while (true) {
        const size_t task_index = next_task_.fetch_add(1, std::memory_order_relaxed);
        if (task_index >= TaskCount) {
          return;
        }
        invoke_at(task_index);
      }
    }

   private:
    template <size_t Index = 0>
    void invoke_at(size_t task_index) noexcept {
      if constexpr (Index < TaskCount) {
        if (task_index == Index) {
          try {
            std::invoke(std::get<Index>(tasks_));
          } catch (...) {
            this->capture_exception();
          }
          return;
        }
        invoke_at<Index + 1>(task_index);
      }
    }

    Tuple &tasks_;
    std::atomic<size_t> next_task_{0};
  };

  template <typename Function>
  class IndexedInvokeJob final : public Job {
   public:
    IndexedInvokeJob(Function &function, size_t task_count) : function_(function), task_count_(task_count) {}

    void execute(size_t) noexcept override {
      while (true) {
        const size_t task_index = next_task_.fetch_add(1, std::memory_order_relaxed);
        if (task_index >= task_count_) {
          return;
        }
        try {
          std::invoke(function_, task_index);
        } catch (...) {
          this->capture_exception();
        }
      }
    }

   private:
    Function &function_;
    const size_t task_count_;
    std::atomic<size_t> next_task_{0};
  };

  size_t dispatch(Job &job, size_t active_workers);
  void execute_job(Job &job, size_t worker_index) noexcept;
  void worker_loop(size_t worker_index);
  bool is_worker_context() const noexcept;

  const size_t thread_count_;
  std::vector<std::thread> workers_;
  std::mutex dispatch_mutex_;
  mutable std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::condition_variable done_cv_;
  Job *job_ = nullptr;
  size_t next_worker_index_ = 1;
  size_t published_workers_ = 0;
  size_t pending_workers_ = 0;
  size_t started_workers_ = 0;
  uint64_t generation_ = 0;
  bool stopping_ = false;
};

}  // namespace detail

// Options are immutable after construction. Per-call grain overrides the
// default; nested work still executes on its submitting thread.
struct ParallelOptions {
  size_t thread_count = 1;
  size_t minimum_grain = 1;
};

// ISA selection is explicit and compile-time. Native means this build's
// architecture baseline (NEON or SSE2), not runtime CPU detection. All ISA
// instantiations share the same non-template thread-pool implementation.
template <class ISA_TYPE = simd::Native>
class ParallelExecutor final : private detail::ParallelExecutorCore {
  static_assert(simd::isa_available<ISA_TYPE>, "SIMD ISA is unavailable on this target");
  using Core = detail::ParallelExecutorCore;

 public:
  using isa_type = ISA_TYPE;
  using kernel_type = simd::Kernel<ISA_TYPE>;
  explicit ParallelExecutor(size_t threads) : ParallelExecutor(ParallelOptions{threads, 1}) {}
  explicit ParallelExecutor(ParallelOptions options)
      : Core(options.thread_count), options_{thread_count(), options.minimum_grain ? options.minimum_grain : 1} {}

  using Core::background_dispatch_count;
  using Core::dispatch_heap_allocation_count;
  using Core::shutdown;
  using Core::thread_count;
  const ParallelOptions &options() const noexcept { return options_; }
  static constexpr kernel_type kernel() noexcept { return {}; }

  // Accept either (kernel, worker, begin, end) or the existing three-argument
  // callable. Captures are borrowed synchronously; adaptation does not allocate.
  template <class Function>
  size_t parallel_for(size_t count, size_t grain, Function &&function) {
    if constexpr (std::is_invocable_v<Function &, kernel_type, size_t, size_t, size_t>) {
      auto task = [&](size_t worker, size_t begin, size_t end) { std::invoke(function, kernel(), worker, begin, end); };
      return Core::parallel_for(count, grain, task);
    } else {
      return Core::parallel_for(count, grain, std::forward<Function>(function));
    }
  }
  template <class Function>
  size_t parallel_for(size_t count, Function &&function) {
    return parallel_for(count, options_.minimum_grain, std::forward<Function>(function));
  }
  template <class Function>
  size_t parallel_invoke(size_t count, Function &&function) {
    if constexpr (std::is_invocable_v<Function &, kernel_type, size_t>) {
      auto task = [&](size_t index) { std::invoke(function, kernel(), index); };
      return Core::parallel_invoke(count, task);
    } else {
      return Core::parallel_invoke(count, std::forward<Function>(function));
    }
  }
  template <class... Functions, std::enable_if_t<((std::is_invocable_v<Functions &> || std::is_invocable_v<Functions &, kernel_type>)&&...), int> = 0>
  size_t parallel_invoke(Functions &&...functions) {
    return Core::parallel_invoke(adapt(functions)...);
  }

 private:
  template <class Function>
  static auto adapt(Function &function) {
    return [&function] {
      if constexpr (std::is_invocable_v<Function &, kernel_type>)
        std::invoke(function, kernel());
      else
        std::invoke(function);
    };
  }
  const ParallelOptions options_;
};

ParallelExecutor(size_t)->ParallelExecutor<>;
ParallelExecutor(ParallelOptions)->ParallelExecutor<>;

}  // namespace sapphire::backend::tools
