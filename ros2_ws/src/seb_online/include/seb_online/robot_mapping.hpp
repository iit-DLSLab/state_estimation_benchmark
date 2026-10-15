#pragma once

// Explicit robot mapping layer shared by every stage and every estimator.
//
// Canonical order (used everywhere inside seb_online):
//   legs   LF, RF, LH, RH                         (Unitree: FL, FR, RL, RR)
//   joints per leg HAA, HFE, KFE                  (Unitree: hip, thigh, calf)
//   canonical joint index = 3 * leg + joint
//
// Unitree Go2 LowState order (unitree_sdk2 documentation, verified on the Go2
// bags of this lab, see docs): motor_state[0..11] = FR(hip,thigh,calf),
// FL(...), RR(...), RL(...), and foot_force[0..3] = FR, FL, RR, RL.
// Hence the default index maps below.
//
// NOTE: the reference MUSE Go2 contact plugin (unitree_sdk branch) assigns
// foot_force[0..3] directly to LF, RF, LH, RH. With the order above that would
// label the FR foot as LF; seb_online uses the explicit map instead. The map must
// be validated on the real robot (lift one foot at a time, see docs).

#include "seb_online/types.hpp"

#include <Eigen/Core>

#include <array>
#include <string>

namespace seb {

// Leg order used internally by the KAIST invariant estimators (IEKF and
// invariant smoother), as set up by the offline benchmark mains:
// index 0..3 = RH(RR), LH(RL), LF(FL), RF(FR).
constexpr std::array<int, kNumLegs> kInvariantLegOrder{{RH, LH, LF, RF}};

const char* legName(int leg);  // "LF", "RF", "LH", "RH"

struct RobotMapping {
  std::string robot_type = "go2";
  std::string urdf_path;

  // canonical joint index -> index into LowState.motor_state
  std::array<int, kNumJoints> lowstate_motor_index{};
  // canonical leg index -> index into LowState.foot_force
  std::array<int, kNumLegs> lowstate_foot_force_index{};

  // URDF names in canonical order.
  std::array<std::string, kNumJoints> urdf_joint_names;
  std::array<std::string, kNumLegs> urdf_foot_frames;
  std::string base_frame = "base";

  // IMU placement in the base frame: x_base = R_base_imu * x_imu + p_base_imu.
  // If imu_frame is not empty it is read from the URDF (resolveImuFromUrdf).
  std::string imu_frame = "imu";
  Eigen::Matrix3d R_base_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_base_imu = Eigen::Vector3d::Zero();

  // Default mapping for the Unitree Go2 (URDF: models/go2/go2.urdf).
  static RobotMapping go2Defaults();

  // Throws std::invalid_argument if the index maps are not permutations,
  // names are empty, or R_base_imu is not a rotation.
  void validate() const;
};

// Reads R_base_imu and p_base_imu of mapping.imu_frame from the URDF (fixed
// joint placement relative to mapping.base_frame). No-op if imu_frame is empty.
void resolveImuFromUrdf(RobotMapping& mapping);

}  // namespace seb
