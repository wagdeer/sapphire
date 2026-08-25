#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sapphire {

class ParallelExecutor final {
 public:
  using RangeTask = std::function<void(size_t worker_index, size_t begin, size_t end)>;
  using ForegroundTask = std::function<void()>;

  explicit ParallelExecutor(int thread_count);
  ~ParallelExecutor();

  ParallelExecutor(const ParallelExecutor &) = delete;
  ParallelExecutor &operator=(const ParallelExecutor &) = delete;

  size_t thread_count() const noexcept { return thread_count_; }
  size_t parallel_for(size_t item_count, size_t minimum_items_per_worker, const RangeTask &range_task, const ForegroundTask &foreground_task = {});

 private:
  void worker_loop(size_t worker_index);
  void execute_range(size_t worker_index, size_t item_count, size_t active_workers, const RangeTask &task) noexcept;
  void capture_exception() noexcept;

  const size_t thread_count_;
  std::vector<std::thread> workers_;
  std::mutex dispatch_mutex_;
  std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::condition_variable done_cv_;
  const RangeTask *task_ = nullptr;
  size_t item_count_ = 0;
  size_t active_workers_ = 0;
  size_t pending_workers_ = 0;
  uint64_t generation_ = 0;
  bool stopping_ = false;
  std::exception_ptr exception_;
};

}  // namespace sapphire
