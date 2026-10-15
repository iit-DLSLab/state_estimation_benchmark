#include "seb_online/foot_kinematics.hpp"

#include <cmath>

namespace seb {

namespace {

benchmark::FootKinematicsConfig makeConfig(const RobotMapping& mapping) {
  benchmark::FootKinematicsConfig c;
  c.urdf_path = mapping.urdf_path;
  c.foot_frames = mapping.urdf_foot_frames;
  c.joint_names = mapping.urdf_joint_names;
  c.base_frame = mapping.base_frame;
  return c;
}

}  // namespace

EstimationFrameKinematics::EstimationFrameKinematics(const RobotMapping& mapping)
    : mapping_(mapping), model_(makeConfig(mapping)) {}

void EstimationFrameKinematics::compute(const SensorSample& sample, KinematicsResult& out) {
  out.sample_id = sample.sample_id;
  out.stamp_ns = sample.stamp_ns;
  model_.compute(sample.q, sample.dq, fk_);
  bool finite = true;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    // Base frame -> estimation frame E (base axes, origin at the IMU).
    out.p_foot[leg] = fk_.position[leg] - mapping_.p_base_imu;
    out.J_foot[leg] = fk_.jacobian[leg];
    out.v_foot[leg] = fk_.velocity[leg];
    finite = finite && out.p_foot[leg].allFinite() && out.J_foot[leg].allFinite() && out.v_foot[leg].allFinite();
  }
  out.valid = finite;
}

void EstimationFrameKinematics::footPositionsBase(const Vector12d& q, std::array<Eigen::Vector3d, kNumLegs>& p_base) {
  model_.compute(q, Vector12d::Zero(), fk_);
  for (int leg = 0; leg < kNumLegs; ++leg) p_base[leg] = fk_.position[leg];
}

}  // namespace seb
