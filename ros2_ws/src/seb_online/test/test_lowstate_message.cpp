// Conversion of a real unitree_go::msg::LowState into the canonical sample.

#include "seb_online/acquisition.hpp"

#include <gtest/gtest.h>
#include <unitree_go/msg/low_state.hpp>

using namespace seb;

TEST(LowStateMessage, FieldsAndOrderAreMappedExplicitly) {
  unitree_go::msg::LowState msg;
  msg.imu_state.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
  msg.imu_state.gyroscope = {0.1f, -0.2f, 0.3f};
  msg.imu_state.accelerometer = {0.5f, -0.4f, 9.7f};
  for (int m = 0; m < 20; ++m) {
    msg.motor_state[m].q = 0.01f * static_cast<float>(m);
    msg.motor_state[m].dq = -0.02f * static_cast<float>(m);
    msg.motor_state[m].tau_est = 0.5f * static_cast<float>(m);
  }
  msg.foot_force = {11, 22, 33, 44};  // FR, FL, RR, RL
  msg.tick = 123456u;

  RawLowState raw;
  raw.receive_ros_ns = 42;
  raw.receive_steady_ns = 7;
  fillRawLowState(msg, raw);
  EXPECT_EQ(raw.tick, 123456u);

  LowStateAcquisition acq(AcquisitionConfig(), RobotMapping::go2Defaults());
  const auto out = acq.process(raw);
  ASSERT_TRUE(out.sample);
  const SensorSample& s = *out.sample;
  EXPECT_EQ(s.tick, 123456u);
  EXPECT_EQ(s.receive_ros_ns, 42);
  EXPECT_EQ(s.stamp_ns, 42) << "default time base is the reception time";
  EXPECT_FLOAT_EQ(static_cast<float>(s.gyro.y()), -0.2f);
  EXPECT_FLOAT_EQ(static_cast<float>(s.acc.z()), 9.7f);
  // canonical LF HAA <- motor 3 (FL hip), RF HAA <- motor 0 (FR hip)
  EXPECT_FLOAT_EQ(static_cast<float>(s.q[0]), 0.03f);
  EXPECT_FLOAT_EQ(static_cast<float>(s.q[3]), 0.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(s.dq[11]), -0.16f);  // RH KFE <- motor 8 (RR calf)
  EXPECT_EQ(s.foot_force[LF], 22.0);
  EXPECT_EQ(s.foot_force[RF], 11.0);
  EXPECT_EQ(s.foot_force[LH], 44.0);
  EXPECT_EQ(s.foot_force[RH], 33.0);
}
