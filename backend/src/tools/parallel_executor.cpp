#include <algorithm>
#include <exception>
#include <backend/tools/parallel_executor.hpp>
#include <stdexcept>

namespace sapphire::backend::tools::detail {
namespace {

struct ExecutionContext {
  const ParallelExecutorCore *executor;
  ExecutionContext *previous;
};

thread_local ExecutionContext *current_execution = nullptr;

class ExecutionScope final {
 public:
  explicit ExecutionScope(const ParallelExecutorCore *executor) : context_{executor, current_execution} { current_execution = &context_; }

  ~ExecutionScope() { current_execution = context_.previous; }

  ExecutionScope(const ExecutionScope &) = delete;
  ExecutionScope &operator=(const ExecutionScope &) = delete;

 private:
  ExecutionContext context_;
};

}  // namespace

ParallelExecutorCore::ParallelExecutorCore(size_t thread_count) : thread_count_(std::max<size_t>(1, thread_count)) {
  workers_.reserve(thread_count_ - 1);
  try {
    for (size_t worker_index = 1; worker_index < thread_count_; ++worker_index) {
      workers_.emplace_back(&ParallelExecutorCore::worker_loop, this, worker_index);
    }
    std::unique_lock<std::mutex> lock(state_mutex_);
    done_cv_.wait(lock, [this] { return started_workers_ == thread_count_ - 1; });
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

ParallelExecutorCore::~ParallelExecutorCore() noexcept {
  if (is_worker_context()) {
    std::terminate();
  }
  try {
    shutdown();
  } catch (...) {
    std::terminate();
  }
}

void ParallelExecutorCore::Job::capture_exception() noexcept {
  std::lock_guard<std::mutex> lock(exception_mutex_);
  if (!exception_) {
    exception_ = std::current_exception();
  }
}

void ParallelExecutorCore::Job::rethrow_if_exception() {
  std::exception_ptr exception;
  {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    exception = exception_;
  }
  if (exception) {
    std::rethrow_exception(exception);
  }
}

size_t ParallelExecutorCore::background_dispatch_count() const noexcept {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return static_cast<size_t>(generation_);
}

size_t ParallelExecutorCore::dispatch(Job &job, size_t active_workers) {
  if (is_worker_context()) {
    if (active_workers == 0) {
      return 0;
    }
    execute_job(job, 0);
    job.rethrow_if_exception();
    return 1;
  }

  std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) {
      throw std::runtime_error("ParallelExecutorCore is shut down");
    }
  }

  if (active_workers == 0) {
    return 0;
  }
  if (active_workers == 1) {
    execute_job(job, 0);
    job.rethrow_if_exception();
    return 1;
  }

  const size_t background_workers = active_workers - 1;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    job_ = &job;
    next_worker_index_ = 1;
    published_workers_ = background_workers;
    pending_workers_ = background_workers;
    ++generation_;
    // Notify while holding state_mutex_: a very short task cannot finish,
    // wait again, and consume another notification for this generation.
    for (size_t worker = 0; worker < background_workers; ++worker) {
      work_cv_.notify_one();
    }
  }

  execute_job(job, 0);

  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    done_cv_.wait(lock, [this] { return pending_workers_ == 0; });
    job_ = nullptr;
    published_workers_ = 0;
  }
  job.rethrow_if_exception();
  return active_workers;
}

void ParallelExecutorCore::execute_job(Job &job, size_t worker_index) noexcept {
  ExecutionScope execution_scope(this);
  try {
    job.execute(worker_index);
  } catch (...) {
    job.capture_exception();
  }
}

void ParallelExecutorCore::shutdown() {
  if (is_worker_context()) {
    throw std::logic_error("ParallelExecutorCore cannot be shut down from its own task");
  }

  std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
  }
  work_cv_.notify_all();

  for (std::thread &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

bool ParallelExecutorCore::is_worker_context() const noexcept {
  for (ExecutionContext *context = current_execution; context != nullptr; context = context->previous) {
    if (context->executor == this) {
      return true;
    }
  }
  return false;
}

void ParallelExecutorCore::worker_loop(size_t) {
  uint64_t observed_generation = 0;
  std::unique_lock<std::mutex> lock(state_mutex_);
  ++started_workers_;
  done_cv_.notify_one();

  while (true) {
    work_cv_.wait(lock, [this, observed_generation] { return stopping_ || (generation_ != observed_generation && next_worker_index_ <= published_workers_); });
    if (stopping_) {
      return;
    }
    observed_generation = generation_;
    const size_t worker_index = next_worker_index_++;
    Job *const job = job_;

    lock.unlock();
    execute_job(*job, worker_index);
    lock.lock();

    if (--pending_workers_ == 0) {
      done_cv_.notify_one();
    }
  }
}

}  // namespace sapphire::backend::tools
