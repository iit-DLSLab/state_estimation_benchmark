#pragma once

#include "seb_online/acquisition.hpp"
#include "seb_online/estimator_adapter.hpp"
#include "seb_online/types.hpp"

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace seb::test {

// Sample whose payload encodes its identity: q[j] = id + 0.01 j, foot_force[l] = 1000 + id + l.
inline SensorSamplePtr makeSample(std::uint64_t id, std::int64_t stamp_ns) {
  auto s = std::make_shared<SensorSample>();
  s->sample_id = id;
  s->acq_seq = id;
  s->msg_seq = id;
  s->stamp_ns = stamp_ns;
  s->receive_ros_ns = stamp_ns;
  s->receive_steady_ns = stamp_ns;
  for (int j = 0; j < kNumJoints; ++j) s->q[j] = static_cast<double>(id) + 0.01 * j;
  for (int l = 0; l < kNumLegs; ++l) s->foot_force[l] = 1000.0 + static_cast<double>(id) + l;
  return s;
}

// Worker results that encode the sample they were computed from.
inline KinematicsResult makeKinematics(const SensorSample& s) {
  KinematicsResult k;
  k.sample_id = s.sample_id;
  k.stamp_ns = s.stamp_ns;
  k.valid = true;
  for (int l = 0; l < kNumLegs; ++l) {
    k.p_foot[l] = Eigen::Vector3d(s.q[3 * l], 0.0, 0.0);
    k.J_foot[l] = Eigen::Matrix3d::Identity();
    k.v_foot[l] = Eigen::Vector3d::Zero();
  }
  return k;
}

inline ContactResult makeContact(const SensorSample& s) {
  ContactResult c;
  c.sample_id = s.sample_id;
  c.stamp_ns = s.stamp_ns;
  c.all_valid = true;
  for (int l = 0; l < kNumLegs; ++l) {
    c.metric[l] = s.foot_force[l];
    c.valid[l] = true;
    c.stance[l] = true;
  }
  return c;
}

// True if the three parts of the bundle were computed from the same sample.
inline bool bundleIsCoherent(const SynchronizedMeasurement& m) {
  const SensorSample& s = *m.sensor;
  if (m.kinematics.sample_id != s.sample_id || m.contacts.sample_id != s.sample_id) return false;
  if (m.kinematics.stamp_ns != s.stamp_ns || m.contacts.stamp_ns != s.stamp_ns) return false;
  for (int l = 0; l < kNumLegs; ++l) {
    if (m.kinematics.p_foot[l].x() != s.q[3 * l]) return false;
    if (m.contacts.metric[l] != s.foot_force[l]) return false;
  }
  return true;
}

// Estimator stand-in that records what it receives and checks coherence.
class RecordingEstimator : public EstimatorAdapter {
 public:
  struct Call {
    std::uint64_t sample_id;
    std::uint64_t acq_seq;
    std::int64_t stamp_ns;
    bool first;
    double dt_s;
    bool coherent;
  };

  explicit RecordingEstimator(std::function<void()> work = nullptr) : work_(std::move(work)) {}

  const char* name() const override { return "recording"; }
  void reset() override { ++resets; }
  std::string describe() const override { return "recording"; }

  EstimatorOutput step(const SynchronizedMeasurement& m, const StepTiming& t) override {
    if (work_) work_();
    // Mutable estimator state: concurrent calls would be a bug.
    const int prev = in_step_.fetch_add(1);
    if (prev != 0) concurrent_calls.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calls.push_back({m.sensor->sample_id, m.sensor->acq_seq, m.sensor->stamp_ns, t.first, t.dt_s,
                       bundleIsCoherent(m)});
    }
    in_step_.fetch_sub(1);
    return EstimatorOutput();
  }

  std::vector<Call> snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls;
  }

  std::atomic<int> concurrent_calls{0};
  int resets = 0;

 private:
  std::function<void()> work_;
  std::mutex mutex_;
  std::vector<Call> calls;
  std::atomic<int> in_step_{0};
};

// Synthetic Go2 LowState: standing robot, values encode the message index.
inline RawLowState makeRaw(std::uint32_t index, std::int64_t receive_ns, std::uint32_t tick) {
  RawLowState r;
  r.receive_ros_ns = receive_ns;
  r.receive_steady_ns = receive_ns;
  r.tick = tick;
  r.acc = {0.0f, 0.0f, 9.81f};
  r.gyro = {0.0f, 0.0f, 0.0f};
  const float stand[3] = {0.0f, 0.8f, -1.5f};
  for (int m = 0; m < 12; ++m) {
    r.q[m] = stand[m % 3] + 1e-4f * static_cast<float>(index % 100);
    r.dq[m] = 0.0f;
  }
  for (int f = 0; f < 4; ++f) r.foot_force[f] = static_cast<std::int16_t>(50 + (index % 50));
  return r;
}

}  // namespace seb::test
