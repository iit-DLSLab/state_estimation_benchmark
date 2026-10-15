#include "seb_online/robot_mapping.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>
#include <string>

namespace seb {

const char* legName(int leg) {
  switch (leg) {
    case LF: return "LF";
    case RF: return "RF";
    case LH: return "LH";
    case RH: return "RH";
    default: return "??";
  }
}

RobotMapping RobotMapping::go2Defaults() {
  RobotMapping m;
  m.robot_type = "go2";
  // LowState.motor_state blocks: FR=0..2, FL=3..5, RR=6..8, RL=9..11.
  // canonical: LF(=FL), RF(=FR), LH(=RL), RH(=RR)
  m.lowstate_motor_index = {3, 4, 5,    // LF <- FL
                            0, 1, 2,    // RF <- FR
                            9, 10, 11,  // LH <- RL
                            6, 7, 8};   // RH <- RR
  // LowState.foot_force: FR=0, FL=1, RR=2, RL=3.
  m.lowstate_foot_force_index = {1, 0, 3, 2};
  m.urdf_joint_names = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  m.urdf_foot_frames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  m.base_frame = "base";
  m.imu_frame = "imu";
  return m;
}

namespace {

template <std::size_t N>
void checkPermutation(const std::array<int, N>& idx, int range, const std::string& what) {
  std::array<bool, 64> seen{};
  if (range > 64) throw std::invalid_argument(what + ": range too large");
  for (std::size_t i = 0; i < N; ++i) {
    if (idx[i] < 0 || idx[i] >= range) {
      throw std::invalid_argument(what + ": index " + std::to_string(idx[i]) + " out of range [0," +
                                  std::to_string(range) + ")");
    }
    if (seen[static_cast<std::size_t>(idx[i])]) {
      throw std::invalid_argument(what + ": index " + std::to_string(idx[i]) + " used twice");
    }
    seen[static_cast<std::size_t>(idx[i])] = true;
  }
}

}  // namespace

void RobotMapping::validate() const {
  // Go2 LowState carries 20 motor slots (12 used) and 4 foot forces.
  checkPermutation(lowstate_motor_index, 20, "lowstate_motor_index");
  checkPermutation(lowstate_foot_force_index, 4, "lowstate_foot_force_index");
  for (const auto& n : urdf_joint_names) {
    if (n.empty()) throw std::invalid_argument("urdf_joint_names contains an empty name");
  }
  for (const auto& n : urdf_foot_frames) {
    if (n.empty()) throw std::invalid_argument("urdf_foot_frames contains an empty name");
  }
  if (base_frame.empty()) throw std::invalid_argument("base_frame is empty");
  const double orth = (R_base_imu * R_base_imu.transpose() - Eigen::Matrix3d::Identity()).norm();
  if (!R_base_imu.allFinite() || orth > 1e-6 || R_base_imu.determinant() < 0.999) {
    throw std::invalid_argument("R_base_imu is not a rotation matrix");
  }
  if (!p_base_imu.allFinite()) throw std::invalid_argument("p_base_imu is not finite");
}

void resolveImuFromUrdf(RobotMapping& mapping) {
  if (mapping.imu_frame.empty()) return;
  pinocchio::Model model;
  pinocchio::urdf::buildModel(mapping.urdf_path, model);
  if (!model.existFrame(mapping.imu_frame)) {
    throw std::invalid_argument("IMU frame '" + mapping.imu_frame + "' not found in " + mapping.urdf_path);
  }
  if (!model.existFrame(mapping.base_frame)) {
    throw std::invalid_argument("Base frame '" + mapping.base_frame + "' not found in " + mapping.urdf_path);
  }
  pinocchio::Data data(model);
  const Eigen::VectorXd q0 = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model, data, q0);
  pinocchio::updateFramePlacements(model, data);
  const pinocchio::SE3& oMb = data.oMf[model.getFrameId(mapping.base_frame)];
  const pinocchio::SE3& oMi = data.oMf[model.getFrameId(mapping.imu_frame)];
  const pinocchio::SE3 bMi = oMb.actInv(oMi);
  mapping.R_base_imu = bMi.rotation();
  mapping.p_base_imu = bMi.translation();
}

}  // namespace seb
