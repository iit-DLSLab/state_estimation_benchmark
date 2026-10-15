#pragma once

// A worker thread processing SensorSamples in FIFO order.
// The processing function and its state are confined to the worker thread, so
// no lock is held during the (possibly expensive) computation. Input is a
// bounded queue; the thread blocks on a condition variable when idle.

#include "seb_online/bounded_queue.hpp"
#include "seb_online/thread_utils.hpp"
#include "seb_online/types.hpp"

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace seb {

template <typename Result>
class SampleWorker {
 public:
  // Called in the worker thread. Must fill `out` (sample_id, stamp, data).
  // `reset_state` is true for the first sample after requestReset().
  using Process = std::function<void(const SensorSample& sample, Result& out, bool reset_state)>;
  // Receives the finished result (timing fields set). Called in the worker thread.
  using Sink = std::function<void(Result&& result)>;
  using StatusSink = std::function<void(const std::string&)>;

  SampleWorker(std::string name, std::size_t capacity, Process process, Sink sink,
               ThreadSettings thread_settings = ThreadSettings(), StatusSink status = nullptr)
      : name_(std::move(name)),
        queue_(capacity),
        process_(std::move(process)),
        sink_(std::move(sink)),
        thread_settings_(thread_settings),
        status_(std::move(status)) {}

  ~SampleWorker() { stop(); }

  SampleWorker(const SampleWorker&) = delete;
  SampleWorker& operator=(const SampleWorker&) = delete;

  void start() {
    if (thread_.joinable()) return;
    queue_.reopen();
    thread_ = std::thread([this] { run(); });
  }

  // Stops after the queued samples have been processed.
  void stop() {
    queue_.close();
    if (thread_.joinable()) thread_.join();
  }

  bool tryPush(const SensorSamplePtr& sample) { return queue_.tryPush(sample); }
  std::size_t queued() const { return queue_.size(); }
  std::size_t capacity() const { return queue_.capacity(); }

  // The processing state is reset before the next processed sample.
  void requestReset() { reset_requested_.store(true); }

  // Discards queued samples (used on pipeline reset).
  void clear() { queue_.clear(); }

 private:
  void run() {
    const std::string err = applyThreadSettings(name_, thread_settings_);
    if (!err.empty() && status_) status_(name_ + ": " + err);
    while (auto sample = queue_.pop()) {
      Result out;
      const bool reset = reset_requested_.exchange(false);
      out.start_steady_ns = steadyNowNs();
      process_(**sample, out, reset);
      out.end_steady_ns = steadyNowNs();
      sink_(std::move(out));
    }
  }

  std::string name_;
  BoundedQueue<SensorSamplePtr> queue_;
  Process process_;
  Sink sink_;
  ThreadSettings thread_settings_;
  StatusSink status_;
  std::atomic<bool> reset_requested_{false};
  std::thread thread_;
};

}  // namespace seb
