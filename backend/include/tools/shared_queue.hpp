#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace sapphire::backend::tools {

// Thread-safe blocking queue with optional item-count capacity. The owner must
// close the queue, join all producers/consumers, and only then destroy it.
// Destruction is intentionally not claimed safe while another thread is
// executing a member function or waiting on this queue.
template <typename T>
class SharedQueue final {
 public:
  // A capacity of zero selects an unbounded queue.
  explicit SharedQueue(size_t capacity = 0) : capacity_(capacity) {}
  ~SharedQueue() = default;

  SharedQueue(const SharedQueue &) = delete;
  SharedQueue &operator=(const SharedQueue &) = delete;
  SharedQueue(SharedQueue &&) = delete;
  SharedQueue &operator=(SharedQueue &&) = delete;

  bool push(const T &item) { return emplace(item); }
  bool push(T &&item) { return emplace(std::move(item)); }

  template <typename... Arguments>
  bool emplace(Arguments &&...arguments) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      not_full_.wait(lock, [this] { return closed_ || capacity_ == 0 || queue_.size() < capacity_; });
      if (closed_) {
        return false;
      }
      queue_.emplace_back(std::forward<Arguments>(arguments)...);
    }
    not_empty_.notify_one();
    return true;
  }

  bool try_pop(T &item) {
    std::optional<T> popped_item;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.empty()) {
        return false;
      }
      popped_item.emplace(std::move(queue_.front()));
      queue_.pop_front();
    }
    notify_one_producer();
    item = std::move(*popped_item);
    return true;
  }

  bool wait_pop(T &item) {
    std::optional<T> popped_item;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      not_empty_.wait(lock, [this] { return closed_ || !queue_.empty(); });
      if (queue_.empty()) {
        return false;
      }
      popped_item.emplace(std::move(queue_.front()));
      queue_.pop_front();
    }
    notify_one_producer();
    item = std::move(*popped_item);
    return true;
  }

  bool wait_pop_latest(T &item) {
    std::deque<T> drained;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      not_empty_.wait(lock, [this] { return closed_ || !queue_.empty(); });
      if (queue_.empty()) {
        return false;
      }
      drained.swap(queue_);
    }
    notify_all_producers();
    item = std::move(drained.back());
    return true;
  }

  bool close() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        return false;
      }
      closed_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
    return true;
  }

  bool closed() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  bool empty() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  size_t size() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  size_t capacity() const noexcept { return capacity_; }

 private:
  void notify_one_producer() noexcept {
    if (capacity_ != 0) {
      not_full_.notify_one();
    }
  }

  void notify_all_producers() noexcept {
    if (capacity_ != 0) {
      not_full_.notify_all();
    }
  }

  const size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<T> queue_;
  bool closed_ = false;
};

}  // namespace sapphire::backend::tools
