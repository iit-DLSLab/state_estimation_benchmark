#pragma once

// Estimator worker: the only thread that owns and calls the selected estimator.
//
// For each SynchronizedMeasurement (released in sample order):
//   1. reset request?           -> re-initialise (kResetRequested)
//   2. stale?                   -> age since reception > max_input_age_s: dropped (kStaleInput)
//   3. dt = stamp_k - stamp_prev (time base of the samples, never wall/publication time)
//        dt < min_dt_s          -> not processed (kDtTooSmall)
//        dt > max_gap_s         -> estimator re-initialised on this sample (kResetGap)
//        acq_seq jump           -> processed across the gap, flagged (kGap, status DEGRADED)
//   4. estimator update (timed with the monotonic clock)
//   5. no leg in stance (hold_without_stance) -> velocity forced to zero and position held
//      at the previous estimate, in the estimator state and in the output (kNoStanceHold);
//      only for estimators implementing holdStationary (IEKF, smoother), not MUSE;
//      first stance sample after a hold >= hold_restart_s -> estimator restarted at the
//      held position, zero velocity, roll/pitch from the accelerometer (kHoldRestart)
//   6. non-finite / speed > max_speed_mps -> status INVALID, re-initialised (kResetDiverged)
//   7. conversion of the estimate from frame E (base axes, IMU origin) to the base
//      origin, stamped with the SAMPLE stamp, handed to the output sink
//      (publication) and then to the record sink (logging).

#include "seb_online/bounded_queue.hpp"
#include "seb_online/estimator_adapter.hpp"
#include "seb_online/robot_mapping.hpp"
#include "seb_online/thread_utils.hpp"
#include "seb_online/types.hpp"

#include <Eigen/Geometry>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace seb {

enum class EstimateStatus : std::uint8_t { kInitializing = 0, kValid = 1, kDegraded = 2, kInvalid = 3 };

const char* toString(EstimateStatus status);

// Published state + everything needed for the per-sample performance log.
struct StateEstimate {
  // identity / time
  std::uint64_t sample_id = 0;
  std::uint64_t acq_seq = 0;
  std::uint64_t msg_seq = 0;
  std::int64_t stamp_ns = 0;  // time of the state = stamp of the source sample
  StampSource stamp_source = StampSource::kReceiveTime;
  std::uint32_t tick = 0;
  const char* estimator = "";
  EstimateStatus status = EstimateStatus::kInitializing;

  // base state (world frame W, base frame B)
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  bool position_valid = false;
  Eigen::Vector3d v_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();  // gyro of the sample (bias-corrected if available)
  bool omega_bias_corrected = false;
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  bool gyro_bias_valid = false;
  bool acc_bias_valid = false;
  std::array<bool, kNumLegs> contact{{false, false, false, false}};

  // estimator diagnostics
  double dt_s = 0.0;
  int iterations = -1;
  int window_frames = -1;
  double window_duration_s = 0.0;
  std::uint64_t missing_before = 0;  // acquisition samples missing right before this one

  // timing, steady clock [ns]
  std::int64_t receive_steady_ns = 0;
  std::int64_t dispatch_steady_ns = 0;
  std::int64_t kin_start_steady_ns = 0;
  std::int64_t kin_end_steady_ns = 0;
  std::int64_t contact_start_steady_ns = 0;
  std::int64_t contact_end_steady_ns = 0;
  std::int64_t complete_steady_ns = 0;
  std::int64_t release_steady_ns = 0;
  std::int64_t dequeue_steady_ns = 0;
  std::int64_t est_start_steady_ns = 0;
  std::int64_t est_end_steady_ns = 0;
  std::int64_t receive_ros_ns = 0;
  std::int64_t dds_source_ns = 0;  // publisher clock (robot), separate domain; 0 if unavailable

  // queue occupancy: stages at dispatch, estimator queue when the bundle was dequeued
  std::uint32_t kin_queue_at_dispatch = 0;
  std::uint32_t contact_queue_at_dispatch = 0;
  std::uint32_t sync_pending_at_dispatch = 0;
  std::size_t estimator_queue_depth = 0;

  // counters at the time of this sample
  std::uint64_t dropped_total = 0;

  // publication (filled by the output sink)
  std::int64_t publish_steady_ns = 0;
  std::int64_t publish_ros_ns = 0;
};

struct EstimatorWorkerConfig {
  double max_gap_s = 0.05;        // larger dt re-initialises the estimator
  double max_input_age_s = 0.1;   // older bundles (since reception) are dropped; <= 0 disables
  double min_dt_s = 1e-6;         // smaller dt is not processed
  double max_speed_mps = 20.0;    // divergence check on the base speed
  bool hold_without_stance = true;  // no leg in stance: zero velocity, hold position (kNoStanceHold)
  double hold_restart_s = 0.5;      // stance after a hold this long: restart at the held pose (< 0: never)
  int warmup_samples = 1;         // status INITIALIZING for the first samples after (re)initialisation
  ThreadSettings thread;
};

class EstimatorWorker {
 public:
  using OutputSink = std::function<void(StateEstimate&)>;   // publish; may fill publish_* fields
  using RecordSink = std::function<void(const StateEstimate&)>;
  using EventSink = std::function<void(const PipelineEvent&)>;
  using StatusSink = std::function<void(const std::string&)>;
  using DropCounter = std::function<std::uint64_t()>;

  EstimatorWorker(const EstimatorWorkerConfig& config, const RobotMapping& mapping,
                  std::unique_ptr<EstimatorAdapter> adapter, BoundedQueue<SynchronizedMeasurement>& input,
                  OutputSink output_sink, RecordSink record_sink, EventSink event_sink, StatusSink status_sink,
                  DropCounter drop_counter);
  ~EstimatorWorker();

  void start();
  void stop();  // processes nothing more after the input queue is closed and drained

  void requestReset() { reset_requested_.store(true); }

  // Number of bundles taken from the queue (processed or dropped).
  std::uint64_t consumed() const { return consumed_.load(); }
  std::uint64_t processed() const { return processed_.load(); }

  // Waits (no busy loop) until consumed() >= target or the timeout expires.
  bool waitConsumed(std::uint64_t target, std::chrono::milliseconds timeout);

  const EstimatorAdapter& adapter() const { return *adapter_; }

  // Processes one bundle synchronously (the worker loop calls this; tests may
  // call it directly when the thread is not started).
  void processOne(SynchronizedMeasurement& m, std::size_t queue_depth_after_pop);

 private:
  void run();
  void emitEvent(EventType type, const SensorSample& s, double value);
  void resetEstimator();

  EstimatorWorkerConfig config_;
  RobotMapping mapping_;
  std::unique_ptr<EstimatorAdapter> adapter_;
  BoundedQueue<SynchronizedMeasurement>& input_;
  OutputSink output_sink_;
  RecordSink record_sink_;
  EventSink event_sink_;
  StatusSink status_sink_;
  DropCounter drop_counter_;

  // Estimator-thread state.
  bool initialized_ = false;
  std::int64_t first_stamp_ns_ = 0;
  std::int64_t last_stamp_ns_ = 0;
  std::uint64_t last_acq_seq_ = 0;
  int samples_since_init_ = 0;
  bool holding_ = false;                                 // previous sample was held
  Eigen::Vector3d last_p_wE_ = Eigen::Vector3d::Zero();  // position of the previous estimate (frame E)
  Eigen::Matrix3d last_R_wE_ = Eigen::Matrix3d::Identity();  // attitude of the previous estimate
  std::int64_t hold_start_ns_ = 0;                       // stamp of the first held sample

  std::atomic<bool> reset_requested_{false};
  std::atomic<std::uint64_t> consumed_{0};
  std::atomic<std::uint64_t> processed_{0};
  std::mutex progress_mutex_;
  std::condition_variable progress_cv_;
  std::thread thread_;
};

}  // namespace seb
