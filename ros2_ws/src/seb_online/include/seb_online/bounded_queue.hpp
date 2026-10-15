#pragma once

// Bounded multi-producer / multi-consumer FIFO with blocking pop.
// Capacity is fixed at construction; memory is allocated once (ring buffer).
// No busy waiting: consumers block on a condition variable.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <vector>

namespace seb {

enum class OverflowPolicy {
  kRejectNewest,  // the new item is not inserted
  kDropOldest,    // the oldest queued item is removed to make room
};

template <typename T>
class BoundedQueue {
 public:
  explicit BoundedQueue(std::size_t capacity) : buffer_(capacity > 0 ? capacity : 1) {}

  BoundedQueue(const BoundedQueue&) = delete;
  BoundedQueue& operator=(const BoundedQueue&) = delete;

  std::size_t capacity() const { return buffer_.size(); }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }

  // Returns false (and leaves the queue unchanged) if full or closed.
  bool tryPush(T item) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_ || size_ == buffer_.size()) return false;
      pushLocked(std::move(item));
    }
    not_empty_.notify_one();
    return true;
  }

  // Push applying an overflow policy. If an item is discarded it is moved into
  // *discarded (the oldest one for kDropOldest, `item` itself for kRejectNewest).
  // Returns true if `item` was inserted.
  bool pushWithPolicy(T item, OverflowPolicy policy, std::optional<T>* discarded) {
    bool inserted = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        if (discarded) *discarded = std::move(item);
        return false;
      }
      if (size_ == buffer_.size()) {
        if (policy == OverflowPolicy::kRejectNewest) {
          if (discarded) *discarded = std::move(item);
          return false;
        }
        T oldest = popLocked();
        if (discarded) *discarded = std::move(oldest);
      }
      pushLocked(std::move(item));
      inserted = true;
    }
    not_empty_.notify_one();
    return inserted;
  }

  // Blocks until an item is available or the queue is closed and empty.
  std::optional<T> pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return size_ > 0 || closed_; });
    if (size_ == 0) return std::nullopt;
    return popLocked();
  }

  // Blocks until an item is available, the queue is closed, or the deadline passes.
  template <typename Clock, typename Duration>
  std::optional<T> popUntil(const std::chrono::time_point<Clock, Duration>& deadline) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait_until(lock, deadline, [this] { return size_ > 0 || closed_; });
    if (size_ == 0) return std::nullopt;
    return popLocked();
  }

  std::optional<T> tryPop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) return std::nullopt;
    return popLocked();
  }

  // Wakes all consumers; further pushes fail. Remaining items can still be popped.
  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
  }

  void reopen() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = false;
  }

  bool closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (size_ > 0) popLocked();
  }

 private:
  void pushLocked(T&& item) {
    buffer_[(head_ + size_) % buffer_.size()] = std::move(item);
    ++size_;
  }

  T popLocked() {
    T item = std::move(buffer_[head_]);
    buffer_[head_] = T();
    head_ = (head_ + 1) % buffer_.size();
    --size_;
    return item;
  }

  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::vector<T> buffer_;
  std::size_t head_ = 0;
  std::size_t size_ = 0;
  bool closed_ = false;
};

}  // namespace seb
