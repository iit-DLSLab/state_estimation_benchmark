#include "seb_online/acquisition.hpp"

#include <cmath>
#include <cstring>

namespace seb {

namespace {

template <std::size_t N>
bool allFinite(const std::array<float, N>& a) {
  for (float v : a) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

}  // namespace

LowStateAcquisition::LowStateAcquisition(const AcquisitionConfig& config, const RobotMapping& mapping)
    : config_(config), mapping_(mapping) {
  if (config_.decimation < 1) config_.decimation = 1;
}

void LowStateAcquisition::reset() {
  have_previous_ = false;
  have_epoch_ = false;
  accepted_ = 0;
  tick_unwrapped_ = 0;
  last_stamp_ns_ = 0;
}

PipelineEvent LowStateAcquisition::makeEvent(EventType type, const RawLowState& raw, double value) const {
  PipelineEvent e;
  e.type = type;
  e.msg_seq = msg_seq_;
  e.tick = raw.tick;
  e.stamp_ns = raw.receive_ros_ns;
  e.steady_ns = raw.receive_steady_ns;
  e.value = value;
  return e;
}

bool LowStateAcquisition::isDuplicate(const RawLowState& raw) const {
  if (!have_previous_ || raw.tick != previous_.tick) return false;
  // Exact comparison on purpose: a duplicate is a re-delivery of the same message.
  return raw.gyro == previous_.gyro && raw.acc == previous_.acc && raw.q == previous_.q &&
         raw.dq == previous_.dq && raw.foot_force == previous_.foot_force &&
         raw.imu_quaternion == previous_.imu_quaternion;
}

LowStateAcquisition::Output LowStateAcquisition::process(const RawLowState& raw) {
  ++msg_seq_;
  Output out;

  // 1) Validity.
  const bool finite = allFinite(raw.gyro) && allFinite(raw.acc) && allFinite(raw.q) && allFinite(raw.dq) &&
                      allFinite(raw.imu_quaternion);
  if (!finite) {
    out.event = makeEvent(EventType::kInvalidMessage, raw);
    return out;
  }

  // 2) Duplicate delivery.
  if (config_.drop_duplicates && isDuplicate(raw)) {
    out.event = makeEvent(EventType::kDuplicate, raw);
    return out;
  }

  // 3) Tick unwrapping and time base.
  bool new_epoch = !have_epoch_;
  std::uint64_t tick_unwrapped = 0;
  bool tick_repeated = false;
  if (have_epoch_) {
    const std::uint32_t delta = raw.tick - last_tick_;  // modulo 2^32
    if (delta >= 0x80000000u) {
      // Tick went backwards.
      const std::int64_t back_ns =
          static_cast<std::int64_t>(0x100000000ull - delta) * config_.tick_period_ns;
      if (back_ns > config_.time_jump_reset_ns) {
        // Large regression (robot restart, bag restart): start a new epoch,
        // otherwise every following message would be rejected.
        new_epoch = true;
      } else {
        out.event = makeEvent(EventType::kOutOfOrder, raw, -static_cast<double>(back_ns) * 1e-9);
        return out;
      }
    } else {
      tick_unwrapped = tick_unwrapped_ + delta;
      tick_repeated = (delta == 0);
    }
  }

  std::int64_t stamp_ns = 0;
  if (!new_epoch) {
    if (config_.stamp_source == StampSource::kReceiveTime) {
      stamp_ns = raw.receive_ros_ns;
    } else {
      stamp_ns = tick_offset_ns_ + static_cast<std::int64_t>(tick_unwrapped) * config_.tick_period_ns;
    }
    if (stamp_ns <= last_stamp_ns_) {
      const std::int64_t back_ns = last_stamp_ns_ - stamp_ns;
      if (config_.stamp_source == StampSource::kReceiveTime && back_ns > config_.time_jump_reset_ns) {
        new_epoch = true;
      } else {
        // Non-increasing time base: never hand a zero/negative dt to an estimator.
        out.event = makeEvent(EventType::kOutOfOrder, raw, -static_cast<double>(back_ns) * 1e-9);
        return out;
      }
    }
  }

  if (new_epoch) {
    if (have_epoch_) {
      out.reset_required = true;
      out.event = makeEvent(EventType::kResetTimeJump, raw);
      accepted_ = 0;
    }
    have_epoch_ = true;
    tick_unwrapped = 0;
    tick_repeated = false;
    stamp_ns = raw.receive_ros_ns;
    tick_offset_ns_ = raw.receive_ros_ns;  // tick_unwrapped = 0 at the epoch start
  }

  last_tick_ = raw.tick;
  tick_unwrapped_ = tick_unwrapped;
  last_stamp_ns_ = stamp_ns;
  previous_ = raw;
  have_previous_ = true;

  // 4) Decimation (explicit and logged, never silent).
  const std::uint64_t index = accepted_++;
  if (index % static_cast<std::uint64_t>(config_.decimation) != 0) {
    out.event = makeEvent(EventType::kDecimated, raw);
    return out;
  }

  // 5) Canonical sample.
  auto s = std::make_shared<SensorSample>();
  s->acq_seq = ++kept_;
  s->msg_seq = msg_seq_;
  s->stamp_ns = stamp_ns;
  s->stamp_source = config_.stamp_source;
  s->receive_ros_ns = raw.receive_ros_ns;
  s->receive_steady_ns = raw.receive_steady_ns;
  s->dds_source_ns = raw.dds_source_ns;
  s->tick = raw.tick;
  s->tick_unwrapped = tick_unwrapped;
  s->tick_repeated = tick_repeated;

  s->gyro_imu = Eigen::Vector3d(raw.gyro[0], raw.gyro[1], raw.gyro[2]);
  s->acc_imu = Eigen::Vector3d(raw.acc[0], raw.acc[1], raw.acc[2]);
  s->gyro = mapping_.R_base_imu * s->gyro_imu;
  s->acc = mapping_.R_base_imu * s->acc_imu;
  s->imu_quaternion_wxyz = Eigen::Vector4d(raw.imu_quaternion[0], raw.imu_quaternion[1], raw.imu_quaternion[2],
                                           raw.imu_quaternion[3]);
  for (int j = 0; j < kNumJoints; ++j) {
    const int m = mapping_.lowstate_motor_index[j];
    s->q[j] = raw.q[m];
    s->dq[j] = raw.dq[m];
    s->tau_est[j] = raw.tau_est[m];
  }
  for (int leg = 0; leg < kNumLegs; ++leg) {
    s->foot_force[leg] = static_cast<double>(raw.foot_force[mapping_.lowstate_foot_force_index[leg]]);
  }
  out.sample = std::move(s);
  return out;
}

}  // namespace seb
