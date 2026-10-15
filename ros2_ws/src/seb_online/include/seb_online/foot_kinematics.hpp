#pragma once

// Foot kinematics in the estimation frame E (base axes, origin at the IMU),
// built on the shared Pinocchio implementation of the offline benchmark
// (common/include/benchmark/FootKinematics.hpp).

#include "seb_online/robot_mapping.hpp"
#include "seb_online/types.hpp"

#include "benchmark/FootKinematics.hpp"

namespace seb {

class EstimationFrameKinematics {
 public:
  explicit EstimationFrameKinematics(const RobotMapping& mapping);

  // Fills the kinematic fields of `out` for the joint state of `sample`
  // (timing fields are left to the caller). Not thread-safe: one instance per thread.
  void compute(const SensorSample& sample, KinematicsResult& out);

  // Foot positions in the BASE frame (origin at the base) - used by tests.
  void footPositionsBase(const Vector12d& q, std::array<Eigen::Vector3d, kNumLegs>& p_base);

 private:
  RobotMapping mapping_;
  benchmark::FootKinematicsModel model_;
  benchmark::FootKinematicsSample fk_;
};

}  // namespace seb
