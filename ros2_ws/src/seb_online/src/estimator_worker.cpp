#include "seb_online/estimator_worker.hpp"

#include <cmath>
#include <exception>

namespace seb {

const char* toString(EstimateStatus status) {
  switch (status) {
    case EstimateStatus::kInitializing: return "initializing";
    case EstimateStatus::kValid: return "valid";
    case EstimateStatus::kDegraded: return "degraded";
    case EstimateStatus::kInvalid: return "invalid";
  }
  return "unknown";
}

EstimatorWorker::EstimatorWorker(const EstimatorWorkerConfig& config, const RobotMapping& mapping,
                                 std::unique_ptr<EstimatorAdapter> adapter,
                                 BoundedQueue<SynchronizedMeasurement>& input, OutputSink output_sink,
                                 RecordSink record_sink, EventSink event_sink, StatusSink status_sink,
                                 DropCounter drop_counter)
    : config_(config),
      mapping_(mapping),
      adapter_(std::move(adapter)),
      input_(input),
      output_sink_(std::move(output_sink)),
      record_sink_(std::move(record_sink)),
      event_sink_(std::move(event_sink)),
      status_sink_(std::move(status_sink)),
      drop_counter_(std::move(drop_counter)) {}

EstimatorWorker::~EstimatorWorker() { stop(); }

void EstimatorWorker::start() {
  if (thread_.joinable()) return;
  thread_ = std::thread([this] { run(); });
}

void EstimatorWorker::stop() {
  input_.close();
  if (thread_.joinable()) thread_.join();
}

bool EstimatorWorker::waitConsumed(std::uint64_t target, std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(progress_mutex_);
  return progress_cv_.wait_for(lock, timeout, [&] { return consumed_.load() >= target; });
}

void EstimatorWorker::run() {
  const std::string err = applyThreadSettings("seb_estimator", config_.thread);
  if (!err.empty() && status_sink_) status_sink_("estimator thread: " + err);
  while (auto m = input_.pop()) {
    const std::size_t depth = input_.size();
    processOne(*m, depth);
  }
}

void EstimatorWorker::emitEvent(EventType type, const SensorSample& s, double value) {
  if (!event_sink_) return;
  PipelineEvent e;
  e.type = type;
  e.msg_seq = s.msg_seq;
  e.sample_id = s.sample_id;
  e.has_sample_id = true;
  e.stamp_ns = s.stamp_ns;
  e.tick = s.tick;
  e.steady_ns = steadyNowNs();
  e.value = value;
  event_sink_(e);
}

void EstimatorWorker::resetEstimator() {
  adapter_->reset();
  initialized_ = false;
  samples_since_init_ = 0;
}

void EstimatorWorker::processOne(SynchronizedMeasurement& m, std::size_t queue_depth_after_pop) {
  const std::int64_t dequeue_ns = steadyNowNs();
  const SensorSample& s = *m.sensor;

  auto finish = [this] {
    consumed_.fetch_add(1);
    { std::lock_guard<std::mutex> lock(progress_mutex_); }
    progress_cv_.notify_all();
  };

  if (reset_requested_.exchange(false)) {
    resetEstimator();
    emitEvent(EventType::kResetRequested, s, 0.0);
  }

  // Stale input: the estimator is too far behind real time for this sample.
  if (config_.max_input_age_s > 0.0) {
    const double age_s = static_cast<double>(dequeue_ns - s.receive_steady_ns) * 1e-9;
    if (age_s > config_.max_input_age_s) {
      emitEvent(EventType::kStaleInput, s, age_s);
      finish();
      return;
    }
  }

  StepTiming timing;
  EstimateStatus status = EstimateStatus::kValid;
  std::uint64_t missing = 0;

  if (initialized_ && s.stamp_ns < last_stamp_ns_) {
    // Stamps are monotonic within an epoch (acquisition stage), so an earlier
    // stamp means a new epoch (time base reset) raced with this thread: restart
    // the estimator on this sample instead of rejecting every later sample.
    emitEvent(EventType::kResetTimeJump, s, static_cast<double>(s.stamp_ns - last_stamp_ns_) * 1e-9);
    resetEstimator();
  }
  if (initialized_) {
    const double dt = static_cast<double>(s.stamp_ns - last_stamp_ns_) * 1e-9;
    if (dt < config_.min_dt_s) {
      // Includes dt <= 0: never advance an estimator with a non-positive time step.
      emitEvent(EventType::kDtTooSmall, s, dt);
      finish();
      return;
    }
    missing = s.acq_seq > last_acq_seq_ + 1 ? s.acq_seq - last_acq_seq_ - 1 : 0;
    if (dt > config_.max_gap_s) {
      emitEvent(EventType::kResetGap, s, dt);
      resetEstimator();
    } else {
      timing.dt_s = dt;
      if (missing > 0) {
        emitEvent(EventType::kGap, s, static_cast<double>(missing));
        status = EstimateStatus::kDegraded;
      }
    }
  }
  if (!initialized_) {
    timing.first = true;
    timing.dt_s = 0.0;
    first_stamp_ns_ = s.stamp_ns;
    initialized_ = true;
  }
  timing.t_rel_s = static_cast<double>(s.stamp_ns - first_stamp_ns_) * 1e-9;

  StateEstimate out;
  out.sample_id = s.sample_id;
  out.acq_seq = s.acq_seq;
  out.msg_seq = s.msg_seq;
  out.stamp_ns = s.stamp_ns;
  out.stamp_source = s.stamp_source;
  out.tick = s.tick;
  out.estimator = adapter_->name();
  out.contact = m.contacts.stance;
  out.dt_s = timing.dt_s;
  out.missing_before = missing;
  out.receive_steady_ns = s.receive_steady_ns;
  out.receive_ros_ns = s.receive_ros_ns;
  out.dds_source_ns = s.dds_source_ns;
  out.dispatch_steady_ns = m.dispatch_steady_ns;
  out.kin_start_steady_ns = m.kinematics.start_steady_ns;
  out.kin_end_steady_ns = m.kinematics.end_steady_ns;
  out.contact_start_steady_ns = m.contacts.start_steady_ns;
  out.contact_end_steady_ns = m.contacts.end_steady_ns;
  out.complete_steady_ns = m.complete_steady_ns;
  out.release_steady_ns = m.release_steady_ns;
  out.dequeue_steady_ns = dequeue_ns;
  out.estimator_queue_depth = queue_depth_after_pop;
  out.kin_queue_at_dispatch = s.kin_queue_at_dispatch;
  out.contact_queue_at_dispatch = s.contact_queue_at_dispatch;
  out.sync_pending_at_dispatch = s.sync_pending_at_dispatch;

  EstimatorOutput est;
  bool ok = true;
  out.est_start_steady_ns = steadyNowNs();
  try {
    est = adapter_->step(m, timing);
  } catch (const std::exception& ex) {
    ok = false;
    if (status_sink_) status_sink_(std::string("estimator exception: ") + ex.what());
  }
  out.est_end_steady_ns = steadyNowNs();

  last_stamp_ns_ = s.stamp_ns;
  last_acq_seq_ = s.acq_seq;
  ++samples_since_init_;

  if (!ok) {
    emitEvent(EventType::kEstimatorException, s, 0.0);
    resetEstimator();
    finish();
    return;
  }

  // Estimation frame E (base axes, IMU origin) -> base origin.
  const Eigen::Vector3d r_E_base = -mapping_.p_base_imu;  // base origin in E
  out.gyro_bias = est.gyro_bias;
  out.acc_bias = est.acc_bias;
  out.gyro_bias_valid = est.gyro_bias_valid;
  out.acc_bias_valid = est.acc_bias_valid;
  out.omega_bias_corrected = est.gyro_bias_valid;
  out.omega_b = est.gyro_bias_valid ? Eigen::Vector3d(s.gyro - est.gyro_bias) : s.gyro;

  const Eigen::Matrix3d& R = est.R_wE;
  out.p_wb = est.p_wE + R * r_E_base;
  out.position_valid = est.position_valid;
  out.v_wb = est.v_wE + R * out.omega_b.cross(r_E_base);
  out.v_b = R.transpose() * out.v_wb;
  out.q_wb = Eigen::Quaterniond(R);
  out.q_wb.normalize();
  out.iterations = est.iterations;
  out.window_frames = est.window_frames;
  out.window_duration_s = est.window_duration_s;

  const bool finite = R.allFinite() && out.p_wb.allFinite() && out.v_wb.allFinite();
  if (!finite || out.v_wb.norm() > config_.max_speed_mps) {
    status = EstimateStatus::kInvalid;
  } else if (samples_since_init_ <= config_.warmup_samples) {
    status = EstimateStatus::kInitializing;
  } else if (!m.contacts.all_valid || !m.kinematics.valid) {
    status = EstimateStatus::kDegraded;
  }
  out.status = status;
  out.dropped_total = drop_counter_ ? drop_counter_() : 0;

  processed_.fetch_add(1);
  if (output_sink_) output_sink_(out);
  if (record_sink_) record_sink_(out);

  if (status == EstimateStatus::kInvalid) {
    emitEvent(EventType::kResetDiverged, s, out.v_wb.norm());
    resetEstimator();
  }
  finish();
}

}  // namespace seb
