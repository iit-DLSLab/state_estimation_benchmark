// Go2 model, mapping and frame conventions against the URDF (models/go2/go2.urdf).

#include "seb_online/foot_kinematics.hpp"
#include "seb_online/robot_mapping.hpp"

#include <gtest/gtest.h>

using namespace seb;

namespace {

RobotMapping go2Mapping() {
  RobotMapping m = RobotMapping::go2Defaults();
  m.urdf_path = SEB_GO2_URDF;
  resolveImuFromUrdf(m);
  return m;
}

Vector12d standPose() {
  Vector12d q;
  for (int leg = 0; leg < 4; ++leg) q.segment<3>(3 * leg) << 0.0, 0.8, -1.5;
  return q;
}

}  // namespace

TEST(Go2Mapping, DefaultsAreConsistentPermutations) {
  RobotMapping m = go2Mapping();
  EXPECT_NO_THROW(m.validate());
  // Canonical leg -> Unitree name prefix.
  const char* prefix[4] = {"FL_", "FR_", "RL_", "RR_"};
  const char* joint[3] = {"hip", "thigh", "calf"};
  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_EQ(m.urdf_foot_frames[leg], std::string(prefix[leg]) + "foot");
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(m.urdf_joint_names[3 * leg + j], std::string(prefix[leg]) + joint[j] + "_joint");
    }
  }
  // LowState blocks FR=0, FL=1, RR=2, RL=3 (motor index / 3 and foot_force index).
  const int lowstate_leg_of_canonical[4] = {1, 0, 3, 2};
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(m.lowstate_motor_index[3 * leg + j], 3 * lowstate_leg_of_canonical[leg] + j);
    }
    EXPECT_EQ(m.lowstate_foot_force_index[leg], lowstate_leg_of_canonical[leg]);
  }
}

TEST(Go2Mapping, InvalidMappingsRejected) {
  RobotMapping m = go2Mapping();
  m.lowstate_motor_index[0] = m.lowstate_motor_index[1];
  EXPECT_THROW(m.validate(), std::invalid_argument);
  m = go2Mapping();
  m.lowstate_foot_force_index[2] = 7;
  EXPECT_THROW(m.validate(), std::invalid_argument);
  m = go2Mapping();
  m.R_base_imu = 2.0 * Eigen::Matrix3d::Identity();
  EXPECT_THROW(m.validate(), std::invalid_argument);
}

TEST(Go2Mapping, InvariantEstimatorLegOrder) {
  // KAIST IEKF / smoother internal order RR, RL, FL, FR == RH, LH, LF, RF.
  EXPECT_EQ(kInvariantLegOrder[0], RH);
  EXPECT_EQ(kInvariantLegOrder[1], LH);
  EXPECT_EQ(kInvariantLegOrder[2], LF);
  EXPECT_EQ(kInvariantLegOrder[3], RF);
}

TEST(Go2Model, ImuPlacementFromUrdf) {
  const RobotMapping m = go2Mapping();
  // go2.urdf: imu_joint origin xyz="-0.02557 0 0.04232" rpy="0 0 0"
  EXPECT_NEAR(m.p_base_imu.x(), -0.02557, 1e-9);
  EXPECT_NEAR(m.p_base_imu.y(), 0.0, 1e-9);
  EXPECT_NEAR(m.p_base_imu.z(), 0.04232, 1e-9);
  EXPECT_TRUE(m.R_base_imu.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(Go2Model, FootPositionsFrontRearLeftRight) {
  const RobotMapping m = go2Mapping();
  EstimationFrameKinematics kin(m);
  std::array<Eigen::Vector3d, 4> p;
  kin.footPositionsBase(standPose(), p);
  // x forward, y left, z up (base frame).
  EXPECT_GT(p[LF].x(), 0.1);
  EXPECT_GT(p[LF].y(), 0.1);
  EXPECT_GT(p[RF].x(), 0.1);
  EXPECT_LT(p[RF].y(), -0.1);
  EXPECT_LT(p[LH].x(), -0.1);
  EXPECT_GT(p[LH].y(), 0.1);
  EXPECT_LT(p[RH].x(), -0.1);
  EXPECT_LT(p[RH].y(), -0.1);
  for (int leg = 0; leg < 4; ++leg) EXPECT_LT(p[leg].z(), -0.2) << legName(leg);
  // Hip offset 0.1934 (x), 0.0465 + 0.0955 (y) in go2.urdf.
  EXPECT_NEAR(std::abs(p[LF].y()), 0.0465 + 0.0955, 1e-6);
  // Left/right symmetry.
  EXPECT_NEAR(p[LF].x(), p[RF].x(), 1e-9);
  EXPECT_NEAR(p[LF].y(), -p[RF].y(), 1e-9);
}

TEST(Go2Model, HipAbductionAxis) {
  // Positive HAA (axis +x) moves a foot below the hip towards +y on both sides.
  const RobotMapping m = go2Mapping();
  EstimationFrameKinematics kin(m);
  std::array<Eigen::Vector3d, 4> p0, p1;
  Vector12d q = standPose();
  kin.footPositionsBase(q, p0);
  for (int leg = 0; leg < 4; ++leg) q[3 * leg] = 0.1;
  kin.footPositionsBase(q, p1);
  for (int leg = 0; leg < 4; ++leg) EXPECT_GT(p1[leg].y(), p0[leg].y()) << legName(leg);
}

TEST(Go2Model, EstimationFrameAndJacobian) {
  const RobotMapping m = go2Mapping();
  EstimationFrameKinematics kin(m);
  SensorSample s;
  s.sample_id = 5;
  s.stamp_ns = 123;
  s.q = standPose();
  s.q[1] += 0.1;
  s.q[5] -= 0.2;
  for (int j = 0; j < 12; ++j) s.dq[j] = 0.1 * (j + 1);
  KinematicsResult r;
  kin.compute(s, r);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.sample_id, 5u);
  EXPECT_EQ(r.stamp_ns, 123);

  std::array<Eigen::Vector3d, 4> p_base;
  kin.footPositionsBase(s.q, p_base);
  const double h = 1e-6;
  for (int leg = 0; leg < 4; ++leg) {
    // Origin of E is the IMU: p_E = p_base - p_base_imu (axes aligned for Go2).
    EXPECT_TRUE(r.p_foot[leg].isApprox(p_base[leg] - m.p_base_imu, 1e-12)) << legName(leg);
    // Jacobian against central finite differences.
    for (int j = 0; j < 3; ++j) {
      Vector12d qp = s.q, qm = s.q;
      qp[3 * leg + j] += h;
      qm[3 * leg + j] -= h;
      std::array<Eigen::Vector3d, 4> pp, pm;
      kin.footPositionsBase(qp, pp);
      kin.footPositionsBase(qm, pm);
      const Eigen::Vector3d fd = (pp[leg] - pm[leg]) / (2 * h);
      EXPECT_TRUE(r.J_foot[leg].col(j).isApprox(fd, 1e-6)) << legName(leg) << " joint " << j;
    }
    // Foot velocity relative to the base = J * dq_leg.
    EXPECT_TRUE(r.v_foot[leg].isApprox(r.J_foot[leg] * s.dq.segment<3>(3 * leg), 1e-9)) << legName(leg);
  }
}

TEST(Go2Model, AnymalOfflineKinematicsUnchangedByBaseFrameOption) {
  // The shared implementation must reproduce the offline (root-frame) numbers when
  // base_frame is the URDF root link.
  benchmark::FootKinematicsConfig c;
  c.urdf_path = SEB_ANYMAL_URDF;
  c.foot_frames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  c.joint_names = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                   "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  benchmark::FootKinematicsModel root(c);
  c.base_frame = "base";
  benchmark::FootKinematicsModel base(c);
  Vector12d q, dq;
  for (int j = 0; j < 12; ++j) {
    q[j] = 0.1 * j - 0.5;
    dq[j] = 0.05 * j;
  }
  benchmark::FootKinematicsSample a, b;
  root.compute(q, dq, a);
  base.compute(q, dq, b);
  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_TRUE(a.position[leg].isApprox(b.position[leg], 1e-12));
    EXPECT_TRUE(a.jacobian[leg].isApprox(b.jacobian[leg], 1e-12));
    EXPECT_TRUE(a.velocity[leg].isApprox(b.velocity[leg], 1e-12));
  }
}
