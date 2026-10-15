#pragma once

// Go2 contact detection from LowState.foot_force.
//
// Adapted from the MUSE state estimator, branch unitree_sdk:
//   muse_ws/src/state_estimator/include/state_estimator/Models/contact_estimator.hpp
//   muse_ws/src/state_estimator/src/plugins/contact_detection_plugin.cpp
//   Copyright (c) 2025, DLS Lab at Istituto Italiano di Tecnologia, BSD-3-Clause.
// Same strategy as the reference raw-foot-force path used for robot type "go2":
//   metric = |foot_force|
//   without hysteresis : stance = metric > threshold
//   with hysteresis    : stance = previous_stance ? metric > threshold_low
//                                                 : metric > threshold_high
// The GRF / inverse-dynamics path of the reference (other robots) is not used.
//
// Differences with the reference plugin, required for this benchmark:
//   * results keep the sample_id and stamp of the LowState sample they were
//     computed from (the plugin stamps its message with node->now());
//   * the foot-force -> leg assignment uses the explicit RobotMapping (the plugin
//     assigns foot_force[0..3] to LF, RF, LH, RH directly);
//   * a per-foot validity flag based on a configurable plausible range.
// The detector is stateful (hysteresis): feed samples in acquisition order from a
// single thread.

#include "seb_online/types.hpp"

#include <array>

namespace seb {

struct ContactDetectionConfig {
  double threshold = 20.0;       // reference default (raw Go2 foot-force units)
  bool use_hysteresis = false;
  double threshold_low = 0.0;    // stance -> swing below this value
  double threshold_high = 20.0;  // swing -> stance above this value
  // Readings outside [valid_min, valid_max] are flagged invalid and treated as no contact.
  double valid_min = -32768.0;
  double valid_max = 32767.0;
};

class Go2FootForceContactDetector {
 public:
  explicit Go2FootForceContactDetector(const ContactDetectionConfig& config = ContactDetectionConfig())
      : config_(config) {}

  void configure(const ContactDetectionConfig& config) {
    config_ = config;
    reset();
  }

  void reset() { stance_state_.fill(false); }

  // Fills sample-related fields and decisions of `out` (timing left to the caller).
  void detect(const SensorSample& sample, ContactResult& out) {
    out.sample_id = sample.sample_id;
    out.stamp_ns = sample.stamp_ns;
    out.all_valid = true;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const double force = sample.foot_force[leg];
      const bool valid = force >= config_.valid_min && force <= config_.valid_max;
      out.valid[leg] = valid;
      out.metric[leg] = valid ? std::abs(force) : 0.0;
      out.stance[leg] = valid ? contactFromMetric(leg, out.metric[leg]) : false;
      stance_state_[leg] = out.stance[leg];
      out.all_valid = out.all_valid && valid;
    }
  }

  const ContactDetectionConfig& config() const { return config_; }

 private:
  bool contactFromMetric(int foot, double metric) const {
    if (config_.use_hysteresis) {
      return stance_state_[foot] ? metric > config_.threshold_low : metric > config_.threshold_high;
    }
    return metric > config_.threshold;
  }

  ContactDetectionConfig config_;
  std::array<bool, kNumLegs> stance_state_{{false, false, false, false}};
};

}  // namespace seb
