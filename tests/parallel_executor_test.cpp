#include <atomic>
#include <chrono>
#include <cmath>
#include <tools/parallel_executor.hpp>
#include <future>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

template <class ISA_TYPE, class T>
void check_arithmetic() {
  sapphire::tools::ParallelExecutor<ISA_TYPE> executor(sapphire::tools::ParallelOptions{3, 8});
  if (executor.options().thread_count != 3 || executor.options().minimum_grain != 8) throw std::runtime_error("Executor options were not retained");
  for (size_t n : {0, 1, 2, 3, 7, 8, 9, 15, 17, 1025}) {
    std::vector<T> input(n + 2, T(-99)), output(n + 2, T(-99));
    for (size_t i = 0; i < n; ++i) input[i + 1] = T(int(i % 17) - 8) / T(8);
    const auto active = executor.parallel_for(n, [&](auto kernel, size_t, size_t begin, size_t end) {
      static_assert(std::is_same_v<typename decltype(kernel)::isa_type, ISA_TYPE>);
      kernel.set(end - begin, T(.25), output.data() + 1 + begin);
      kernel.multiply_add(end - begin, T(.5), input.data() + 1 + begin, output.data() + 1 + begin);
    });
    if (n == 0 ? active != 0 : (n < 16 && active != 1)) throw std::runtime_error("Default grain policy was ignored");
    for (size_t i = 0; i < n; ++i)
      if (output[i + 1] != T(.25) + T(.5) * input[i + 1]) throw std::runtime_error("SIMD worker/tail arithmetic mismatch");
    if (output.front() != T(-99) || output.back() != T(-99)) throw std::runtime_error("SIMD worker wrote outside its range");
    long double reference = 0;
    for (size_t i = 0; i < n; ++i) reference += static_cast<long double>(input[i + 1]) * output[i + 1];
    const T actual = executor.kernel().dot(n, input.data() + 1, output.data() + 1);
    if (std::abs(actual - reference) > 32 * std::numeric_limits<T>::epsilon() * std::max(1.L, std::abs(reference)))
      throw std::runtime_error("SIMD dot differs from independent reference");
    executor.kernel().multiply_add(n, T(1), output.data() + 1, output.data() + 1);
    for (size_t i = 0; i < n; ++i)
      if (output[i + 1] != T(.5) + input[i + 1]) throw std::runtime_error("In-place SIMD arithmetic mismatch");
  }
  std::atomic<int> visited{0};
  executor.parallel_invoke(
      [&](auto kernel) {
        const auto active = executor.parallel_for(17, 1, [&](auto nested, size_t worker, size_t begin, size_t end) {
          static_assert(std::is_same_v<decltype(kernel), decltype(nested)>);
          if (worker != 0 || begin != 0 || end != 17) throw std::runtime_error("Nested SIMD task did not use caller-runs");
          visited += int(end - begin);
        });
        if (active != 1) throw std::runtime_error("Nested SIMD task dispatched workers");
      },
      [&] { ++visited; });
  executor.parallel_invoke(3, [&](auto kernel, size_t index) {
    static_assert(std::is_same_v<typename decltype(kernel)::isa_type, ISA_TYPE>);
    visited += int(index);
  });
  if (visited != 21) throw std::runtime_error("Typed invoke lost work");
  bool caught = false;
  try {
    executor.parallel_invoke(
        [&](auto) {
          ++visited;
          throw std::runtime_error("expected");
        },
        [&](auto) { ++visited; });
  } catch (const std::runtime_error &) {
    caught = true;
  }
  if (!caught || visited != 23) throw std::runtime_error("Typed tasks did not settle before exception propagation");
}

struct MoveOnlyCallable {
  explicit MoveOnlyCallable(std::atomic<int> &calls) : calls_(&calls) {}
  MoveOnlyCallable(MoveOnlyCallable &&) = default;
  MoveOnlyCallable(const MoveOnlyCallable &) = delete;

  void operator()() { ++*calls_; }

  std::atomic<int> *calls_;
};

}  // namespace

int main() {
  check_arithmetic<simd::Native, float>();
  check_arithmetic<simd::Native, double>();
  check_arithmetic<simd::Scalar, float>();
  check_arithmetic<simd::Scalar, double>();
  static_assert(sapphire::tools::ParallelExecutor<>::dispatch_heap_allocation_count() == 0, "steady-state dispatch must not allocate");

  sapphire::tools::ParallelExecutor executor(4);
  const std::thread::id caller_id = std::this_thread::get_id();
  const size_t initial_dispatches = executor.background_dispatch_count();

  std::atomic<int> empty_calls{0};
  if (executor.parallel_for(0, 1, [&empty_calls](size_t, size_t, size_t) { ++empty_calls; }) != 0 || empty_calls != 0 ||
      executor.background_dispatch_count() != initial_dispatches) {
    return 1;
  }

  if (executor.parallel_invoke(0, [&empty_calls](size_t) { ++empty_calls; }) != 0 || empty_calls != 0 ||
      executor.background_dispatch_count() != initial_dispatches) {
    return 13;
  }

  std::atomic<int> indexed_calls{0};
  if (executor.parallel_invoke(3, [&indexed_calls](size_t task_index) { indexed_calls += static_cast<int>(task_index + 1); }) != 3 || indexed_calls != 6 ||
      executor.background_dispatch_count() != initial_dispatches + 1) {
    return 14;
  }

  bool worker_zero_is_caller = false;
  if (executor.parallel_for(1, 1,
                            [&worker_zero_is_caller, caller_id](size_t worker, size_t begin, size_t end) {
                              worker_zero_is_caller = worker == 0 && begin == 0 && end == 1 && std::this_thread::get_id() == caller_id;
                            }) != 1 ||
      !worker_zero_is_caller || executor.background_dispatch_count() != initial_dispatches + 1) {
    return 2;
  }

  std::vector<std::atomic<int>> visits(17);
  for (auto &visit : visits) {
    visit.store(0);
  }
  std::atomic<bool> grain_valid{true};
  const size_t range_workers = executor.parallel_for(visits.size(), 4, [&visits, &grain_valid](size_t, size_t begin, size_t end) {
    if (end - begin < 4) {
      grain_valid = false;
    }
    for (size_t index = begin; index < end; ++index) {
      ++visits[index];
    }
  });
  if (range_workers != 4 || !grain_valid) {
    return 3;
  }
  for (const auto &visit : visits) {
    if (visit != 1) {
      return 4;
    }
  }
  if (executor.background_dispatch_count() != initial_dispatches + 2) {
    return 5;
  }

  std::atomic<int> heterogeneous_calls{0};
  MoveOnlyCallable move_only(heterogeneous_calls);
  const size_t invoke_workers = executor.parallel_invoke([&heterogeneous_calls] { ++heterogeneous_calls; }, move_only,
                                                         [&heterogeneous_calls] {
                                                           ++heterogeneous_calls;
                                                           return 7;
                                                         });
  if (invoke_workers != 3 || heterogeneous_calls != 3) {
    return 6;
  }

  std::atomic<int> settled_tasks{0};
  bool saw_exception = false;
  try {
    executor.parallel_invoke([&settled_tasks] { ++settled_tasks; },
                             [&settled_tasks] {
                               ++settled_tasks;
                               throw std::runtime_error("expected");
                             },
                             [&settled_tasks] { ++settled_tasks; }, [&settled_tasks] { ++settled_tasks; });
  } catch (const std::runtime_error &) {
    saw_exception = true;
  }
  if (!saw_exception || settled_tasks != 4) {
    return 7;
  }

  std::atomic<int> nested_items{0};
  std::atomic<size_t> nested_workers{0};
  executor.parallel_invoke(
      [&] {
        nested_workers = executor.parallel_for(9, 1, [&nested_items](size_t worker, size_t begin, size_t end) {
          if (worker != 0 || begin != 0 || end != 9) {
            return;
          }
          nested_items += static_cast<int>(end - begin);
        });
      },
      [] {});
  if (nested_workers != 1 || nested_items != 9) {
    return 8;
  }

  std::atomic<int> concurrent_items{0};
  auto submit = [&] {
    executor.parallel_for(64, 2, [&concurrent_items](size_t, size_t begin, size_t end) { concurrent_items += static_cast<int>(end - begin); });
  };
  std::thread first_submitter(submit);
  std::thread second_submitter(submit);
  first_submitter.join();
  second_submitter.join();
  if (concurrent_items != 128) {
    return 9;
  }

  bool worker_shutdown_rejected = false;
  try {
    executor.parallel_invoke([&executor] { executor.shutdown(); }, [] {});
  } catch (const std::logic_error &) {
    worker_shutdown_rejected = true;
  }
  if (!worker_shutdown_rejected) {
    return 10;
  }

  std::promise<void> accepted;
  std::promise<void> release_promise;
  std::shared_future<void> release = release_promise.get_future().share();
  std::thread accepted_submitter([&] {
    executor.parallel_invoke([&] {
      accepted.set_value();
      release.wait();
    });
  });
  accepted.get_future().wait();

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&] {
    executor.shutdown();
    shutdown_finished = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (shutdown_finished) {
    release_promise.set_value();
    accepted_submitter.join();
    shutdown_thread.join();
    return 11;
  }
  release_promise.set_value();
  accepted_submitter.join();
  shutdown_thread.join();

  bool rejected_after_shutdown = false;
  try {
    executor.parallel_invoke([] {});
  } catch (const std::runtime_error &) {
    rejected_after_shutdown = true;
  }
  return rejected_after_shutdown ? 0 : 12;
}
