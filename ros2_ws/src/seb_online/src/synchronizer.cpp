#include "seb_online/synchronizer.hpp"

#include <algorithm>
#include <chrono>

namespace seb {

ExactSampleSynchronizer::ExactSampleSynchronizer(const SynchronizerConfig& config, EventSink event_sink, Clock clock)
    : config_(config),
      event_sink_(std::move(event_sink)),
      clock_(std::move(clock)),
      slots_(std::max<std::size_t>(config.capacity, 1)),
      output_(std::max<std::size_t>(config.output_capacity, 1)) {}

ExactSampleSynchronizer::~ExactSampleSynchronizer() { stopSupervisor(); }

void ExactSampleSynchronizer::startSupervisor() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (supervisor_.joinable()) return;
  stop_supervisor_ = false;
  supervisor_ = std::thread([this] { supervisorLoop(); });
}

void ExactSampleSynchronizer::stopSupervisor() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_supervisor_ = true;
  }
  supervisor_cv_.notify_all();
  if (supervisor_.joinable()) supervisor_.join();
}

std::uint64_t ExactSampleSynchronizer::nextSampleId() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return next_id_;
}

bool ExactSampleSynchronizer::hasCapacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return next_id_ - head_id_ < slots_.size();
}

bool ExactSampleSynchronizer::registerSample(const SensorSamplePtr& sample, std::int64_t dispatch_steady_ns) {
  if (!sample) return false;
  bool was_empty = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sample->sample_id != next_id_) return false;
    if (next_id_ - head_id_ >= slots_.size()) return false;
    was_empty = (next_id_ == head_id_);
    Slot& slot = slotFor(next_id_);
    slot = Slot();
    slot.used = true;
    slot.id = next_id_;
    slot.sensor = sample;
    slot.dispatch_steady_ns = dispatch_steady_ns;
    slot.deadline_steady_ns = dispatch_steady_ns + config_.worker_timeout_ns;
    ++next_id_;
    ++stats_.registered;
    stats_.pending = next_id_ - head_id_;
    stats_.max_pending = std::max(stats_.max_pending, stats_.pending);
  }
  // The supervisor sleeps until the head deadline; it only needs waking when
  // this sample became the head (barrier previously empty).
  if (was_empty) supervisor_cv_.notify_all();
  return true;
}

ExactSampleSynchronizer::Slot* ExactSampleSynchronizer::acceptResultLocked(std::uint64_t id, std::int64_t stamp_ns,
                                                                           std::vector<PipelineEvent>& events) {
  if (id < head_id_) {
    // Sample already released or expired: never pair this result with anything.
    ++stats_.late_results;
    PipelineEvent e;
    e.type = EventType::kLateResult;
    e.sample_id = id;
    e.has_sample_id = true;
    e.stamp_ns = stamp_ns;
    e.steady_ns = clock_();
    events.push_back(e);
    return nullptr;
  }
  if (id >= next_id_) {
    ++stats_.unknown_results;
    return nullptr;
  }
  Slot& slot = slotFor(id);
  if (!slot.used || slot.id != id) {
    ++stats_.identity_mismatches;
    return nullptr;
  }
  return &slot;
}

void ExactSampleSynchronizer::submitKinematics(KinematicsResult result) {
  std::vector<PipelineEvent> events;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot* slot = acceptResultLocked(result.sample_id, result.stamp_ns, events);
    if (slot != nullptr) {
      if (slot->kinematics) {
        ++stats_.duplicate_results;
      } else {
        slot->kinematics = std::move(result);
        if (slot->contacts) slot->complete_steady_ns = clock_();
        processHeadLocked(clock_(), events);
      }
    }
  }
  emit(events);
}

void ExactSampleSynchronizer::submitContact(ContactResult result) {
  std::vector<PipelineEvent> events;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot* slot = acceptResultLocked(result.sample_id, result.stamp_ns, events);
    if (slot != nullptr) {
      if (slot->contacts) {
        ++stats_.duplicate_results;
      } else {
        slot->contacts = std::move(result);
        if (slot->kinematics) slot->complete_steady_ns = clock_();
        processHeadLocked(clock_(), events);
      }
    }
  }
  emit(events);
}

void ExactSampleSynchronizer::expireOverdue(std::int64_t now_steady_ns) {
  std::vector<PipelineEvent> events;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processHeadLocked(now_steady_ns, events);
  }
  emit(events);
}

void ExactSampleSynchronizer::processHeadLocked(std::int64_t now, std::vector<PipelineEvent>& events) {
  while (head_id_ < next_id_) {
    Slot& slot = slotFor(head_id_);
    if (slot.kinematics && slot.contacts) {
      releaseLocked(slot, now, events);
    } else if (slot.deadline_steady_ns <= now) {
      expireLocked(slot, now, events);
    } else {
      break;  // keep chronological order: later samples wait for this one
    }
    slot = Slot();
    ++head_id_;
  }
  stats_.pending = next_id_ - head_id_;
  stats_.output_depth = output_.size();
  stats_.max_output_depth = std::max(stats_.max_output_depth, stats_.output_depth);
}

void ExactSampleSynchronizer::releaseLocked(Slot& slot, std::int64_t now, std::vector<PipelineEvent>& events) {
  SynchronizedMeasurement m;
  m.sensor = slot.sensor;
  m.kinematics = std::move(*slot.kinematics);
  m.contacts = std::move(*slot.contacts);
  m.dispatch_steady_ns = slot.dispatch_steady_ns;
  m.complete_steady_ns = slot.complete_steady_ns;
  m.release_steady_ns = now;

  // Defensive identity check: all three parts must come from the same sample.
  if (m.sensor->sample_id != slot.id || m.kinematics.sample_id != slot.id || m.contacts.sample_id != slot.id ||
      m.kinematics.stamp_ns != m.sensor->stamp_ns || m.contacts.stamp_ns != m.sensor->stamp_ns) {
    ++stats_.identity_mismatches;
    return;
  }

  std::optional<SynchronizedMeasurement> discarded;
  output_.pushWithPolicy(std::move(m), config_.output_overflow, &discarded);
  ++stats_.released;
  if (discarded) {
    ++stats_.backlog_drops;
    PipelineEvent e;
    e.type = EventType::kEstimatorBacklog;
    e.sample_id = discarded->sensor->sample_id;
    e.has_sample_id = true;
    e.msg_seq = discarded->sensor->msg_seq;
    e.stamp_ns = discarded->sensor->stamp_ns;
    e.tick = discarded->sensor->tick;
    e.steady_ns = now;
    e.value = static_cast<double>(output_.capacity());
    events.push_back(e);
  }
}

void ExactSampleSynchronizer::expireLocked(Slot& slot, std::int64_t now, std::vector<PipelineEvent>& events) {
  PipelineEvent e;
  if (!slot.kinematics && !slot.contacts) {
    e.type = EventType::kMissingBoth;
    ++stats_.expired_missing_both;
  } else if (!slot.kinematics) {
    e.type = EventType::kMissingKinematics;
    ++stats_.expired_missing_kinematics;
  } else {
    e.type = EventType::kMissingContact;
    ++stats_.expired_missing_contact;
  }
  e.sample_id = slot.id;
  e.has_sample_id = true;
  e.msg_seq = slot.sensor ? slot.sensor->msg_seq : 0;
  e.stamp_ns = slot.sensor ? slot.sensor->stamp_ns : 0;
  e.tick = slot.sensor ? slot.sensor->tick : 0;
  e.steady_ns = now;
  e.value = static_cast<double>(now - slot.dispatch_steady_ns) * 1e-9;  // waited time [s]
  events.push_back(e);
}

void ExactSampleSynchronizer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.discarded_on_reset += (next_id_ - head_id_) + output_.size();
  while (head_id_ < next_id_) {
    slotFor(head_id_) = Slot();
    ++head_id_;
  }
  output_.clear();
  stats_.pending = 0;
}

SynchronizerStats ExactSampleSynchronizer::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  SynchronizerStats s = stats_;
  s.output_depth = output_.size();
  return s;
}

void ExactSampleSynchronizer::emit(const std::vector<PipelineEvent>& events) {
  if (!event_sink_) return;
  for (const auto& e : events) event_sink_(e);
}

void ExactSampleSynchronizer::supervisorLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_supervisor_) {
    if (head_id_ < next_id_) {
      const std::int64_t deadline = slotFor(head_id_).deadline_steady_ns;
      const std::int64_t now = clock_();
      if (deadline > now) {
        // Sleep until the head deadline (or until notified by register/stop).
        supervisor_cv_.wait_for(lock, std::chrono::nanoseconds(deadline - now));
        continue;
      }
      std::vector<PipelineEvent> events;
      processHeadLocked(now, events);
      if (!events.empty()) {
        lock.unlock();
        emit(events);
        lock.lock();
      }
      // If the head is waiting for a not-yet-expired deadline, loop recomputes it.
      if (head_id_ < next_id_ && slotFor(head_id_).deadline_steady_ns <= clock_()) continue;
    } else {
      supervisor_cv_.wait(lock);
    }
  }
}

}  // namespace seb
