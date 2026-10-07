#include "tools/parallel_executor.hpp"

#include <algorithm>
#include <stdexcept>

namespace sapphire {

ParallelExecutor::ParallelExecutor(int thread_count) : thread_count_(static_cast<size_t>(std::max(1, thread_count))) {
  workers_.reserve(thread_count_ - 1);
  try {
    for (size_t worker_index = 1; worker_index < thread_count_; ++worker_index) {
      workers_.emplace_back(&ParallelExecutor::worker_loop, this, worker_index);
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stopping_ = true;
    }
    work_cv_.notify_all();
    for (std::thread &worker : workers_) {
      worker.join();
    }
    throw;
  }
}

ParallelExecutor::~ParallelExecutor() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stopping_ = true;
  }
  work_cv_.notify_all();
  for (std::thread &worker : workers_) {
    worker.join();
  }
}

size_t ParallelExecutor::parallel_for(size_t item_count, size_t minimum_items_per_worker, const RangeTask &range_task,
                                      const ForegroundTask &foreground_task) {
  if (!range_task) {
    throw std::invalid_argument("ParallelExecutor requires a range task");
  }

  std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
  minimum_items_per_worker = std::max<size_t>(1, minimum_items_per_worker);
  const size_t useful_workers = item_count == 0 ? 0 : 1 + (item_count - 1) / minimum_items_per_worker;
  const size_t active_workers = std::min(thread_count_, useful_workers);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    task_ = &range_task;
    item_count_ = item_count;
    active_workers_ = active_workers;
    pending_workers_ = active_workers > 0 ? active_workers - 1 : 0;
    exception_ = nullptr;
    ++generation_;
  }
  work_cv_.notify_all();

  try {
    if (foreground_task) {
      foreground_task();
    }
  } catch (...) {
    capture_exception();
  }

  if (active_workers > 0) {
    execute_range(0, item_count, active_workers, range_task);
  }

  std::exception_ptr exception;
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    done_cv_.wait(lock, [this] { return pending_workers_ == 0; });
    task_ = nullptr;
    exception = exception_;
  }
  if (exception) {
    std::rethrow_exception(exception);
  }
  return active_workers;
}

void ParallelExecutor::worker_loop(size_t worker_index) {
  uint64_t observed_generation = 0;
  while (true) {
    const RangeTask *task = nullptr;
    size_t item_count = 0;
    size_t active_workers = 0;
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      work_cv_.wait(lock, [this, observed_generation] { return stopping_ || generation_ != observed_generation; });
      if (stopping_) {
        return;
      }
      observed_generation = generation_;
      if (worker_index >= active_workers_) {
        continue;
      }
      task = task_;
      item_count = item_count_;
      active_workers = active_workers_;
    }

    execute_range(worker_index, item_count, active_workers, *task);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (--pending_workers_ == 0) {
        done_cv_.notify_one();
      }
    }
  }
}

void ParallelExecutor::execute_range(size_t worker_index, size_t item_count, size_t active_workers, const RangeTask &task) noexcept {
  const size_t base_size = item_count / active_workers;
  const size_t remainder = item_count % active_workers;
  const size_t begin = worker_index * base_size + std::min(worker_index, remainder);
  const size_t end = begin + base_size + (worker_index < remainder ? 1 : 0);
  try {
    task(worker_index, begin, end);
  } catch (...) {
    capture_exception();
  }
}

void ParallelExecutor::capture_exception() noexcept {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!exception_) {
    exception_ = std::current_exception();
  }
}

}  // namespace sapphire
