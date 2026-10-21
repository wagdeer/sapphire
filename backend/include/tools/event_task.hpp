#pragma once

#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <stdexcept>
#include <utility>

namespace sapphire::backend::tools {

class EventTask {
 public:
  using Clock = std::chrono::steady_clock;
  using Deadline = std::optional<Clock::time_point>;
  using Work = std::function<Deadline()>;
  using Error = std::function<void(const std::exception&)>;

  EventTask() = default;
  EventTask(const EventTask&) = delete;
  EventTask& operator=(const EventTask&) = delete;
  ~EventTask() { stop(); }

  void start(Work work, Error error) {
    if (started_ || stopped_) throw std::logic_error("EventTask can only be started once");
    if (!work || !error) throw std::invalid_argument("EventTask requires work and an error handler");
    started_ = true;
    worker_ = std::thread([this, work = std::move(work), error = std::move(error)] {
      Deadline deadline;
      std::unique_lock<std::mutex> lock(mutex_);
      while (!stopped_) {
        const auto ready = [this] { return stopped_ || requested_; };
        if (deadline) wakeup_.wait_until(lock, *deadline, ready);
        else wakeup_.wait(lock, ready);
        if (stopped_) break;
        requested_ = false;
        lock.unlock();
        try { deadline = work(); }
        catch (const std::exception& failure) {
          deadline.reset();
          error(failure);
        }
        lock.lock();
      }
    });
  }

  void notify() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      requested_ = true;
    }
    wakeup_.notify_one();
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    wakeup_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

 private:
  std::mutex mutex_;
  std::condition_variable wakeup_;
  bool requested_ = false, stopped_ = false;
  bool started_ = false; // Owner-thread lifecycle only.

  std::thread worker_;
};
} // namespace sapphire::backend::tools
