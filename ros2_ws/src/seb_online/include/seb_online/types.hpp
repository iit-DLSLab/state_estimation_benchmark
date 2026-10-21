#pragma once

// Core data types of the online pipeline.
//
// Leg order: every per-leg array uses the benchmark order LF, RF, LH, RH
// (= Unitree FL, FR, RL, RR). Joint order inside a leg: HAA (hip ab/adduction,
// Unitree "hip"), HFE (hip flexion, "thigh"), KFE (knee, "calf").
//
// Time: all timestamps are int64 nanoseconds. Each field states its clock domain.
//   steady  : std::chrono::steady_clock of this process (monotonic). Used for every
//             duration (execution time, waiting time, latency).
//   ros     : ROS clock of the node (system time, or /clock in simulation).
//   stamp   : the estimator time base of a sample, see StampSource.
// Values from different domains are never subtracted from each other, except
// where a field explicitly documents the relationship.

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace seb {

constexpr int kNumLegs = 4;
constexpr int kNumJoints = 12;

enum Leg : int { LF = 0, RF = 1, LH = 2, RH = 3 };

using Vector12d = Eigen::Matrix<double, 12, 1>;

enum class StampSource : std::uint8_t {
  // ROS clock when the LowState message reached the subscription callback.
  // This is a RECEPTION time: it includes the unknown robot-to-PC transport delay.
  kReceiveTime = 0,
  // LowState.tick (robot MCU counter) * tick_period, unwrapped, plus a constant
  // offset fixed at the first sample so that it is comparable to ROS time.
  kLowStateTick = 1,
};

// One LowState sample, converted to the canonical representation.
// Created once by the acquisition stage and shared read-only (shared_ptr<const>)
// between the kinematics worker, the contact worker and the estimator.
struct SensorSample {
  std::uint64_t sample_id = 0;   // monotonic, consecutive among dispatched samples
  std::uint64_t acq_seq = 0;     // index among accepted, non-decimated samples (gaps = lost samples)
  std::uint64_t msg_seq = 0;     // index of the received message (counts every message)

  // --- time ---
  std::int64_t stamp_ns = 0;               // estimator time base (domain: stamp_source)
  StampSource stamp_source = StampSource::kReceiveTime;
  std::int64_t receive_ros_ns = 0;         // ros: reception time in the subscription callback
  std::int64_t receive_steady_ns = 0;      // steady: reception time in the subscription callback
  std::int64_t dds_source_ns = 0;          // DDS source timestamp of the publisher (publisher clock, 0 if unavailable)
  std::uint32_t tick = 0;                  // LowState.tick
  std::uint64_t tick_unwrapped = 0;        // tick with uint32 wrap-around removed
  bool tick_repeated = false;              // tick equal to the previous sample (1 ms quantisation)

  // Occupancy of the stages when the sample was dispatched (set before sharing).
  std::uint32_t kin_queue_at_dispatch = 0;
  std::uint32_t contact_queue_at_dispatch = 0;
  std::uint32_t sync_pending_at_dispatch = 0;

  // --- IMU (gyroscope [rad/s], accelerometer specific force [m/s^2]) ---
  Eigen::Vector3d gyro_imu = Eigen::Vector3d::Zero();   // as measured, IMU axes
  Eigen::Vector3d acc_imu = Eigen::Vector3d::Zero();    // as measured, IMU axes
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();       // rotated into base axes
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();        // rotated into base axes
  Eigen::Vector4d imu_quaternion_wxyz = Eigen::Vector4d(1, 0, 0, 0);  // robot onboard attitude (diagnostics only)

  // --- joints, canonical order ---
  Vector12d q = Vector12d::Zero();       // [rad]
  Vector12d dq = Vector12d::Zero();      // [rad/s]
  Vector12d tau_est = Vector12d::Zero(); // [Nm] (diagnostics only)

  // --- foot force, canonical leg order (raw Go2 foot-force units) ---
  std::array<double, kNumLegs> foot_force{{0.0, 0.0, 0.0, 0.0}};
};

using SensorSamplePtr = std::shared_ptr<const SensorSample>;

// Result of the kinematics worker for one sample.
// Frame "E" (estimation frame): axes of the robot base, origin at the IMU.
// All estimators estimate the pose/velocity of frame E; the output stage
// converts to the base origin. This keeps the inertial propagation exact (the
// accelerometer measures the specific force at the IMU origin).
struct KinematicsResult {
  std::uint64_t sample_id = 0;
  std::int64_t stamp_ns = 0;          // copied from the source sample
  bool valid = false;
  std::array<Eigen::Vector3d, kNumLegs> p_foot;    // foot position in E [m]
  std::array<Eigen::Matrix3d, kNumLegs> J_foot;    // d p_foot / d q_leg, E axes
  std::array<Eigen::Vector3d, kNumLegs> v_foot;    // J_foot * dq_leg, E axes [m/s]
  std::int64_t start_steady_ns = 0;
  std::int64_t end_steady_ns = 0;
};

// Result of the contact worker for one sample.
struct ContactResult {
  std::uint64_t sample_id = 0;
  std::int64_t stamp_ns = 0;          // copied from the source sample
  std::array<bool, kNumLegs> stance{{false, false, false, false}};
  std::array<double, kNumLegs> metric{{0.0, 0.0, 0.0, 0.0}};   // |foot force| used for the decision
  std::array<bool, kNumLegs> valid{{false, false, false, false}};  // foot-force reading within the valid range
  bool all_valid = false;
  std::int64_t start_steady_ns = 0;
  std::int64_t end_steady_ns = 0;
};

// A complete measurement: the sensor sample and the two worker results that were
// computed FROM THAT SAME sample (identical sample_id). Built only by the
// ExactSampleSynchronizer.
struct SynchronizedMeasurement {
  SensorSamplePtr sensor;
  KinematicsResult kinematics;
  ContactResult contacts;
  std::int64_t dispatch_steady_ns = 0;   // steady: sample handed to the workers
  std::int64_t complete_steady_ns = 0;   // steady: last of the two results arrived
  std::int64_t release_steady_ns = 0;    // steady: bundle released to the estimator queue (in order)
};

// Reasons for which a received message or a dispatched sample does not reach
// the estimator, or for which the estimator state was affected.
enum class EventType : std::uint8_t {
  kInvalidMessage = 0,      // non-finite values in the message
  kDuplicate = 1,           // identical payload and tick as the previous message
  kOutOfOrder = 2,          // tick or stamp not increasing
  kDecimated = 3,           // skipped by the configured input decimation
  kPipelineFull = 4,        // worker queues or synchronizer full when the sample arrived
  kMissingKinematics = 5,   // worker result not available before the timeout
  kMissingContact = 6,
  kMissingBoth = 7,
  kLateResult = 8,          // worker result arrived after its sample was expired
  kEstimatorBacklog = 9,    // dropped from the estimator input queue (overflow policy)
  kStaleInput = 10,         // older than estimator.max_input_age_s when dequeued
  kGap = 11,                // estimator advanced across missing sample ids (dt spans the gap)
  kResetGap = 12,           // gap longer than estimator.max_gap_s: estimator re-initialised
  kResetDiverged = 13,      // non-finite / diverged estimate: estimator re-initialised
  kResetRequested = 14,     // external reset (service / lifecycle)
  kResetTimeJump = 15,      // time base jumped backwards: pipeline re-initialised
  kDtTooSmall = 16,         // dt below estimator.min_dt_s: sample not processed
  kEstimatorException = 17, // exception thrown by the estimator update
  kNoStanceHold = 18,       // no leg in stance: velocity forced to zero, position held (episode start)
  kHoldRestart = 19,        // stance again after a long hold: estimator restarted at the held pose
};
// Size of the per-type event counters: keep equal to the last EventType + 1.
constexpr std::size_t kNumEventTypes = 20;
static_assert(static_cast<std::size_t>(EventType::kHoldRestart) + 1 == kNumEventTypes,
              "kNumEventTypes must cover every EventType");

const char* toString(EventType type);

struct PipelineEvent {
  EventType type = EventType::kInvalidMessage;
  std::uint64_t msg_seq = 0;
  std::uint64_t sample_id = 0;       // 0 if the message never got a sample id
  bool has_sample_id = false;
  std::int64_t stamp_ns = 0;
  std::int64_t steady_ns = 0;        // when the event was recorded
  std::uint32_t tick = 0;
  double value = 0.0;                // event specific (e.g. gap duration [s], number of missing ids)
};

}  // namespace seb
