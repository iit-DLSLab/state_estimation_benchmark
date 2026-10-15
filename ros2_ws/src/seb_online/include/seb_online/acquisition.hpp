#pragma once

// Acquisition stage: converts a raw Go2 LowState into an immutable SensorSample.
//
// Go2 LowState has no header/stamp. The only time information in the message is
// `tick`, a robot MCU counter (measured on lab bags: +1 per ms, 1 ms resolution,
// published every ~2 ms, so consecutive deltas are 1/2/3 counts; repeated ticks
// and exact duplicate messages also occur). Therefore the estimator time base is
// selectable and always labelled (StampSource):
//   kReceiveTime  : ROS clock in the subscription callback (default). This is a
//                   reception time, not a hardware acquisition time.
//   kLowStateTick : tick * tick_period, unwrapped, offset once to the ROS clock of
//                   the first sample. Quantised to 1 ms.
// The tick, the reception times and the DDS source timestamp are kept in every
// sample for offline analysis of the clock relationship.

#include "seb_online/robot_mapping.hpp"
#include "seb_online/types.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

namespace seb {

// ROS-independent copy of the LowState fields used by the pipeline.
struct RawLowState {
  std::array<float, 4> imu_quaternion{{1.f, 0.f, 0.f, 0.f}};  // w, x, y, z
  std::array<float, 3> gyro{{0.f, 0.f, 0.f}};
  std::array<float, 3> acc{{0.f, 0.f, 0.f}};
  std::array<float, 20> q{};
  std::array<float, 20> dq{};
  std::array<float, 20> tau_est{};
  std::array<std::int16_t, 4> foot_force{};
  std::uint32_t tick = 0;

  std::int64_t receive_ros_ns = 0;
  std::int64_t receive_steady_ns = 0;
  std::int64_t dds_source_ns = 0;
};

// Copies the fields of a unitree_go::msg::LowState (or any type with the same
// field names) into a RawLowState. Header-only so that the core library does not
// depend on the Unitree message package.
template <typename LowStateMsg>
void fillRawLowState(const LowStateMsg& msg, RawLowState& raw) {
  for (int i = 0; i < 4; ++i) raw.imu_quaternion[i] = msg.imu_state.quaternion[i];
  for (int i = 0; i < 3; ++i) {
    raw.gyro[i] = msg.imu_state.gyroscope[i];
    raw.acc[i] = msg.imu_state.accelerometer[i];
  }
  for (int i = 0; i < 20; ++i) {
    raw.q[i] = msg.motor_state[i].q;
    raw.dq[i] = msg.motor_state[i].dq;
    raw.tau_est[i] = msg.motor_state[i].tau_est;
  }
  for (int i = 0; i < 4; ++i) raw.foot_force[i] = msg.foot_force[i];
  raw.tick = msg.tick;
}

struct AcquisitionConfig {
  StampSource stamp_source = StampSource::kReceiveTime;
  std::int64_t tick_period_ns = 1000000;  // 1 ms
  bool drop_duplicates = true;            // same tick and identical payload as the previous message
  int decimation = 1;                     // keep one accepted sample every `decimation`
  // A backward jump of the time base larger than this re-initialises the
  // pipeline (e.g. bag restarted, robot rebooted). Smaller ones are dropped.
  std::int64_t time_jump_reset_ns = 1000000000;
};

class LowStateAcquisition {
 public:
  struct Output {
    // Sample ready to be dispatched (sample_id not yet assigned), or empty.
    std::shared_ptr<SensorSample> sample;
    // Set when the message was rejected or caused a reset.
    std::optional<PipelineEvent> event;
    // True if the time base jumped backwards: downstream state must be reset
    // before dispatching `sample` (which starts the new epoch).
    bool reset_required = false;
  };

  LowStateAcquisition(const AcquisitionConfig& config, const RobotMapping& mapping);

  Output process(const RawLowState& raw);

  void reset();

  std::uint64_t receivedCount() const { return msg_seq_; }

 private:
  bool isDuplicate(const RawLowState& raw) const;
  PipelineEvent makeEvent(EventType type, const RawLowState& raw, double value = 0.0) const;

  AcquisitionConfig config_;
  RobotMapping mapping_;

  std::uint64_t msg_seq_ = 0;
  std::uint64_t accepted_ = 0;  // accepted in the current epoch (decimation phase)
  std::uint64_t kept_ = 0;      // accepted and not decimated, never reset (acq_seq)

  bool have_previous_ = false;
  RawLowState previous_;

  bool have_epoch_ = false;
  std::uint32_t last_tick_ = 0;
  std::uint64_t tick_unwrapped_ = 0;
  std::int64_t tick_offset_ns_ = 0;
  std::int64_t last_stamp_ns_ = 0;
};

}  // namespace seb
