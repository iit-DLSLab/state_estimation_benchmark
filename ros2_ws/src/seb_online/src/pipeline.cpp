#include "seb_online/pipeline.hpp"

#include "seb_online/foot_kinematics.hpp"

#include <chrono>

namespace seb {

std::uint64_t PipelineCounters::droppedBeforeEstimator() const {
  // Samples that were real measurements but did not reach an estimator update.
  // (Decimated samples are excluded: decimation is a configuration choice.)
  const EventType lost[] = {EventType::kInvalidMessage,  EventType::kOutOfOrder,   EventType::kPipelineFull,
                            EventType::kMissingKinematics, EventType::kMissingContact, EventType::kMissingBoth,
                            EventType::kEstimatorBacklog, EventType::kStaleInput,   EventType::kDtTooSmall};
  std::uint64_t n = 0;
  for (EventType t : lost) n += events[static_cast<std::size_t>(t)];
  return n;
}

Pipeline::Pipeline(const PipelineConfig& config, std::unique_ptr<EstimatorAdapter> estimator, Sinks sinks,
                   Hooks hooks)
    : config_(config), sinks_(std::move(sinks)), hooks_(std::move(hooks)), acquisition_(config.acquisition, config.mapping) {
  config_.mapping.validate();

  sync_ = std::make_unique<ExactSampleSynchronizer>(
      config_.sync, [this](const PipelineEvent& e) { recordEvent(e); }, [] { return steadyNowNs(); });

  // ---- kinematics worker ----
  KinematicsFn kin_fn = hooks_.kinematics;
  if (!kin_fn) {
    // The Pinocchio model is built here (errors surface at configuration) and
    // then used exclusively by the worker thread.
    auto model = std::make_shared<EstimationFrameKinematics>(config_.mapping);
    kin_fn = [model](const SensorSample& s, KinematicsResult& out, bool) { model->compute(s, out); };
  }
  auto deliver_kin = hooks_.deliver_kinematics;
  kin_worker_ = std::make_unique<SampleWorker<KinematicsResult>>(
      "seb_kinematics", config_.worker_queue_capacity, kin_fn,
      [this, deliver_kin](KinematicsResult&& r) {
        if (deliver_kin && !deliver_kin(r)) return;
        sync_->submitKinematics(std::move(r));
      },
      config_.kinematics_thread, sinks_.status);

  // ---- contact worker ----
  ContactFn contact_fn = hooks_.contact;
  if (!contact_fn) {
    auto detector = std::make_shared<Go2FootForceContactDetector>(config_.contact);
    contact_fn = [detector](const SensorSample& s, ContactResult& out, bool reset) {
      if (reset) detector->reset();
      detector->detect(s, out);
    };
  }
  auto deliver_contact = hooks_.deliver_contact;
  contact_worker_ = std::make_unique<SampleWorker<ContactResult>>(
      "seb_contact", config_.worker_queue_capacity, contact_fn,
      [this, deliver_contact](ContactResult&& r) {
        if (deliver_contact && !deliver_contact(r)) return;
        sync_->submitContact(std::move(r));
      },
      config_.contact_thread, sinks_.status);

  // ---- estimator worker ----
  estimator_worker_ = std::make_unique<EstimatorWorker>(
      config_.estimator_worker, config_.mapping, std::move(estimator), sync_->output(), sinks_.output, sinks_.record,
      [this](const PipelineEvent& e) { recordEvent(e); }, sinks_.status,
      [this] { return counters().droppedBeforeEstimator(); });
}

Pipeline::~Pipeline() { stop(); }

void Pipeline::start() {
  if (started_) return;
  sync_->output().reopen();
  sync_->startSupervisor();
  kin_worker_->start();
  contact_worker_->start();
  estimator_worker_->start();
  started_ = true;
}

void Pipeline::stop() {
  if (!started_) return;
  // Upstream first, so that queued work drains in order.
  kin_worker_->stop();
  contact_worker_->stop();
  sync_->stopSupervisor();
  estimator_worker_->stop();
  started_ = false;
}

void Pipeline::recordEvent(const PipelineEvent& e) {
  event_counts_[static_cast<std::size_t>(e.type)].fetch_add(1);
  if (sinks_.event) sinks_.event(e);
}

void Pipeline::requestEstimatorReset() { estimator_worker_->requestReset(); }

void Pipeline::resetDownstream() {
  kin_worker_->clear();
  contact_worker_->clear();
  contact_worker_->requestReset();
  sync_->reset();
  estimator_worker_->requestReset();
}

void Pipeline::onLowState(const RawLowState& raw) {
  received_.fetch_add(1);
  LowStateAcquisition::Output acq = acquisition_.process(raw);
  if (acq.event) recordEvent(*acq.event);
  if (acq.reset_required) {
    resetDownstream();
    if (sinks_.status) sinks_.status("time base jumped backwards: pipeline re-initialised");
  }
  if (!acq.sample) return;

  // Explicit backpressure: the sample is dispatched to BOTH workers or to none.
  // This thread is the only producer, so free space cannot shrink in between.
  const std::size_t kin_q = kin_worker_->queued();
  const std::size_t con_q = contact_worker_->queued();
  if (kin_q >= kin_worker_->capacity() || con_q >= contact_worker_->capacity() || !sync_->hasCapacity()) {
    PipelineEvent e;
    e.type = EventType::kPipelineFull;
    e.msg_seq = acq.sample->msg_seq;
    e.stamp_ns = acq.sample->stamp_ns;
    e.tick = acq.sample->tick;
    e.steady_ns = steadyNowNs();
    recordEvent(e);
    if (sinks_.status) sinks_.status("pipeline full: sample dropped before dispatch (workers or synchronizer backlog)");
    return;
  }

  std::shared_ptr<SensorSample> s = std::move(acq.sample);
  s->sample_id = sync_->nextSampleId();
  s->kin_queue_at_dispatch = static_cast<std::uint32_t>(kin_q);
  s->contact_queue_at_dispatch = static_cast<std::uint32_t>(con_q);
  s->sync_pending_at_dispatch = static_cast<std::uint32_t>(sync_->stats().pending);
  const SensorSamplePtr sample = std::move(s);  // immutable from here on

  const std::int64_t dispatch_ns = steadyNowNs();
  if (!sync_->registerSample(sample, dispatch_ns)) {
    // Cannot happen with a single producer; reported rather than ignored.
    if (sinks_.status) sinks_.status("synchronizer refused a sample (unexpected id)");
    return;
  }
  dispatched_.fetch_add(1);
  kin_worker_->tryPush(sample);
  contact_worker_->tryPush(sample);
}

bool Pipeline::waitForInFlightBelow(std::uint64_t max_in_flight, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const SynchronizerStats st = sync_->stats();
    const std::uint64_t expired =
        st.expired_missing_both + st.expired_missing_contact + st.expired_missing_kinematics + st.backlog_drops +
        st.discarded_on_reset + st.identity_mismatches;
    const std::uint64_t done = estimator_worker_->consumed() + expired;
    const std::uint64_t dispatched = dispatched_.load();
    if (dispatched <= done || dispatched - done < max_in_flight) return true;
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;
    // Woken by the estimator after each bundle; the bounded wait also covers
    // samples that leave through expiry instead.
    estimator_worker_->waitConsumed(estimator_worker_->consumed() + 1,
                                    std::min<std::chrono::milliseconds>(
                                        std::chrono::milliseconds(5),
                                        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now) +
                                            std::chrono::milliseconds(1)));
  }
}

bool Pipeline::drain(std::chrono::milliseconds timeout) { return waitForInFlightBelow(1, timeout); }

PipelineCounters Pipeline::counters() const {
  PipelineCounters c;
  c.received = received_.load();
  c.dispatched = dispatched_.load();
  c.processed = estimator_worker_ ? estimator_worker_->processed() : 0;
  for (std::size_t i = 0; i < c.events.size(); ++i) c.events[i] = event_counts_[i].load();
  return c;
}

}  // namespace seb
