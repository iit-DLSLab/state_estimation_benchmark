#pragma once

// Exact sample-identity synchronisation barrier.
//
// Contract
//  * The acquisition stage registers every dispatched SensorSample with a
//    sample_id equal to the previous one + 1 (registerSample rejects anything else).
//  * The kinematics and contact workers submit results carrying the sample_id of
//    the sample they were computed from, in any order and from any thread.
//  * A SynchronizedMeasurement is built ONLY from {sample k, kinematics k,
//    contact k}. Results are never paired across different sample ids, never
//    reused, and never substituted.
//  * Bundles are released strictly in increasing sample_id order (chronological
//    order of acquisition). A complete bundle k+1 waits while k is incomplete.
//  * Each registered sample has a deadline (dispatch + worker_timeout). When the
//    oldest pending sample passes its deadline without both results it is expired:
//    recorded as kMissingKinematics / kMissingContact / kMissingBoth and never
//    released. Results that arrive afterwards are recorded as kLateResult and
//    discarded.
//  * Memory is bounded: at most `capacity` samples pending (fixed ring buffer),
//    output queue of `output_capacity` bundles with an explicit overflow policy;
//    a bundle discarded by the policy is recorded as kEstimatorBacklog.
//  * Deadlines are enforced by a supervisor thread that sleeps on a condition
//    variable until the next deadline (no busy waiting).

#include "seb_online/bounded_queue.hpp"
#include "seb_online/types.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace seb {

struct SynchronizerConfig {
  std::size_t capacity = 64;                   // max samples dispatched but not yet released/expired
  std::int64_t worker_timeout_ns = 20000000;   // 20 ms from dispatch
  std::size_t output_capacity = 50;            // estimator input queue
  OverflowPolicy output_overflow = OverflowPolicy::kDropOldest;
};

struct SynchronizerStats {
  std::uint64_t registered = 0;
  std::uint64_t released = 0;
  std::uint64_t expired_missing_kinematics = 0;
  std::uint64_t expired_missing_contact = 0;
  std::uint64_t expired_missing_both = 0;
  std::uint64_t late_results = 0;
  std::uint64_t unknown_results = 0;     // results for ids never registered (must stay 0)
  std::uint64_t duplicate_results = 0;   // second result of the same kind for an id (must stay 0)
  std::uint64_t identity_mismatches = 0; // internal consistency check (must stay 0)
  std::uint64_t backlog_drops = 0;
  std::uint64_t discarded_on_reset = 0;
  std::size_t pending = 0;
  std::size_t max_pending = 0;
  std::size_t output_depth = 0;
  std::size_t max_output_depth = 0;
};

class ExactSampleSynchronizer {
 public:
  using EventSink = std::function<void(const PipelineEvent&)>;
  using Clock = std::function<std::int64_t()>;  // steady time [ns]

  ExactSampleSynchronizer(const SynchronizerConfig& config, EventSink event_sink, Clock clock);
  ~ExactSampleSynchronizer();

  ExactSampleSynchronizer(const ExactSampleSynchronizer&) = delete;
  ExactSampleSynchronizer& operator=(const ExactSampleSynchronizer&) = delete;

  // Starts/stops the deadline supervisor thread. Without it, deadlines are only
  // enforced by explicit expireOverdue() calls (used by deterministic tests).
  void startSupervisor();
  void stopSupervisor();

  // Id expected by the next registerSample call.
  std::uint64_t nextSampleId() const;
  bool hasCapacity() const;

  // Registers a sample before it is handed to the workers. Returns false if the
  // id is not the expected one or the barrier is full (nothing is modified).
  bool registerSample(const SensorSamplePtr& sample, std::int64_t dispatch_steady_ns);

  void submitKinematics(KinematicsResult result);
  void submitContact(ContactResult result);

  // Expires pending samples whose deadline is <= now and releases what follows.
  void expireOverdue(std::int64_t now_steady_ns);

  // Discards every pending sample and queued bundle; ids keep increasing.
  void reset();

  BoundedQueue<SynchronizedMeasurement>& output() { return output_; }

  SynchronizerStats stats() const;

 private:
  struct Slot {
    bool used = false;
    std::uint64_t id = 0;
    SensorSamplePtr sensor;
    std::optional<KinematicsResult> kinematics;
    std::optional<ContactResult> contacts;
    std::int64_t dispatch_steady_ns = 0;
    std::int64_t deadline_steady_ns = 0;
    std::int64_t complete_steady_ns = 0;
  };

  Slot& slotFor(std::uint64_t id) { return slots_[id % slots_.size()]; }
  // Returns the slot for a result id, or nullptr after recording why it is rejected.
  Slot* acceptResultLocked(std::uint64_t id, std::int64_t stamp_ns, std::vector<PipelineEvent>& events);
  void processHeadLocked(std::int64_t now, std::vector<PipelineEvent>& events);
  void releaseLocked(Slot& slot, std::int64_t now, std::vector<PipelineEvent>& events);
  void expireLocked(Slot& slot, std::int64_t now, std::vector<PipelineEvent>& events);
  void emit(const std::vector<PipelineEvent>& events);
  void supervisorLoop();

  SynchronizerConfig config_;
  EventSink event_sink_;
  Clock clock_;

  mutable std::mutex mutex_;
  std::condition_variable supervisor_cv_;
  std::vector<Slot> slots_;
  std::uint64_t head_id_ = 1;  // oldest id not yet released/expired
  std::uint64_t next_id_ = 1;  // next id to register
  SynchronizerStats stats_;

  BoundedQueue<SynchronizedMeasurement> output_;

  std::thread supervisor_;
  bool stop_supervisor_ = false;
};

}  // namespace seb
