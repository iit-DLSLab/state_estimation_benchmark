// LowState acquisition: canonical mapping, duplicates, ordering, time base.

#include "seb_online/acquisition.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace seb;
using namespace seb::test;

namespace {

constexpr std::int64_t kMs = 1000000;

RobotMapping go2() { return RobotMapping::go2Defaults(); }

}  // namespace

TEST(Acquisition, CanonicalJointAndFootOrderFromGo2LowState) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  RawLowState raw = makeRaw(0, 10 * kMs, 100);
  for (int m = 0; m < 20; ++m) {
    raw.q[m] = static_cast<float>(m);
    raw.dq[m] = static_cast<float>(100 + m);
  }
  for (int f = 0; f < 4; ++f) raw.foot_force[f] = static_cast<std::int16_t>(10 * (f + 1));  // FR,FL,RR,RL
  const auto out = acq.process(raw);
  ASSERT_TRUE(out.sample);
  const SensorSample& s = *out.sample;
  // LowState motor order FR(0-2) FL(3-5) RR(6-8) RL(9-11) -> canonical LF RF LH RH
  const int expected_motor[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
  for (int j = 0; j < 12; ++j) {
    EXPECT_EQ(s.q[j], expected_motor[j]) << "joint " << j;
    EXPECT_EQ(s.dq[j], 100 + expected_motor[j]) << "joint " << j;
  }
  EXPECT_EQ(s.foot_force[LF], 20.0);  // FL = foot_force[1]
  EXPECT_EQ(s.foot_force[RF], 10.0);  // FR = foot_force[0]
  EXPECT_EQ(s.foot_force[LH], 40.0);  // RL = foot_force[3]
  EXPECT_EQ(s.foot_force[RH], 30.0);  // RR = foot_force[2]
}

TEST(Acquisition, ImuRotatedIntoBaseAxes) {
  RobotMapping m = go2();
  // ANYmal-like mounting: 180 deg about y (the offline benchmark flips x and z).
  m.R_base_imu << -1, 0, 0, 0, 1, 0, 0, 0, -1;
  LowStateAcquisition acq(AcquisitionConfig(), m);
  RawLowState raw = makeRaw(0, kMs, 1);
  raw.gyro = {1.f, 2.f, 3.f};
  raw.acc = {4.f, 5.f, 6.f};
  const auto out = acq.process(raw);
  ASSERT_TRUE(out.sample);
  EXPECT_EQ(out.sample->gyro_imu, Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(out.sample->gyro, Eigen::Vector3d(-1, 2, -3));
  EXPECT_EQ(out.sample->acc, Eigen::Vector3d(-4, 5, -6));
}

TEST(Acquisition, ExactDuplicateIsDropped) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  RawLowState a = makeRaw(1, 2 * kMs, 100);
  ASSERT_TRUE(acq.process(a).sample);
  RawLowState dup = a;
  dup.receive_ros_ns = 3 * kMs;  // re-delivered later
  const auto out = acq.process(dup);
  EXPECT_FALSE(out.sample);
  ASSERT_TRUE(out.event);
  EXPECT_EQ(out.event->type, EventType::kDuplicate);
}

TEST(Acquisition, RepeatedTickWithNewPayload) {
  // receive_time base: accepted, flagged as tick_repeated.
  {
    LowStateAcquisition acq(AcquisitionConfig(), go2());
    ASSERT_TRUE(acq.process(makeRaw(1, 2 * kMs, 100)).sample);
    const auto out = acq.process(makeRaw(2, 4 * kMs, 100));
    ASSERT_TRUE(out.sample);
    EXPECT_TRUE(out.sample->tick_repeated);
    EXPECT_EQ(out.sample->stamp_ns, 4 * kMs);
  }
  // lowstate_tick base: same stamp -> never handed on with dt = 0.
  {
    AcquisitionConfig cfg;
    cfg.stamp_source = StampSource::kLowStateTick;
    LowStateAcquisition acq(cfg, go2());
    ASSERT_TRUE(acq.process(makeRaw(1, 2 * kMs, 100)).sample);
    const auto out = acq.process(makeRaw(2, 4 * kMs, 100));
    EXPECT_FALSE(out.sample);
    ASSERT_TRUE(out.event);
    EXPECT_EQ(out.event->type, EventType::kOutOfOrder);
  }
}

TEST(Acquisition, TickTimeBaseUnwrapsAndIsLabelled) {
  AcquisitionConfig cfg;
  cfg.stamp_source = StampSource::kLowStateTick;
  LowStateAcquisition acq(cfg, go2());
  const auto a = acq.process(makeRaw(1, 1000 * kMs, 0xFFFFFFFEu));
  const auto b = acq.process(makeRaw(2, 1003 * kMs, 0x00000001u));  // wrapped: +3 ticks
  ASSERT_TRUE(a.sample && b.sample);
  EXPECT_EQ(a.sample->stamp_source, StampSource::kLowStateTick);
  EXPECT_EQ(a.sample->stamp_ns, 1000 * kMs);  // epoch offset = first reception time
  EXPECT_EQ(b.sample->stamp_ns - a.sample->stamp_ns, 3 * kMs);
  EXPECT_EQ(b.sample->tick_unwrapped, 3u);
  // Reception time is kept separately and never substituted.
  EXPECT_EQ(b.sample->receive_ros_ns, 1003 * kMs);
}

TEST(Acquisition, NonIncreasingReceiveTimeIsRejected) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  ASSERT_TRUE(acq.process(makeRaw(1, 10 * kMs, 100)).sample);
  const auto out = acq.process(makeRaw(2, 10 * kMs, 102));
  EXPECT_FALSE(out.sample);
  ASSERT_TRUE(out.event);
  EXPECT_EQ(out.event->type, EventType::kOutOfOrder);
  const auto back = acq.process(makeRaw(3, 9 * kMs, 104));
  EXPECT_FALSE(back.sample);
}

TEST(Acquisition, TickGoingBackwardsIsRejected) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  ASSERT_TRUE(acq.process(makeRaw(1, 10 * kMs, 100)).sample);
  const auto out = acq.process(makeRaw(2, 12 * kMs, 99));
  EXPECT_FALSE(out.sample);
  EXPECT_EQ(out.event->type, EventType::kOutOfOrder);
}

TEST(Acquisition, LargeBackwardJumpStartsNewEpoch) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  ASSERT_TRUE(acq.process(makeRaw(1, 5000 * kMs, 5000)).sample);
  const auto out = acq.process(makeRaw(2, 100 * kMs, 10));  // bag restarted
  ASSERT_TRUE(out.sample);
  EXPECT_TRUE(out.reset_required);
  ASSERT_TRUE(out.event);
  EXPECT_EQ(out.event->type, EventType::kResetTimeJump);
}

TEST(Acquisition, DecimationIsExplicit) {
  AcquisitionConfig cfg;
  cfg.decimation = 2;
  LowStateAcquisition acq(cfg, go2());
  int kept = 0, decimated = 0;
  std::uint64_t last_seq = 0;
  for (int i = 0; i < 10; ++i) {
    const auto out = acq.process(makeRaw(i, (i + 1) * 2 * kMs, 100 + 2 * i));
    if (out.sample) {
      ++kept;
      EXPECT_EQ(out.sample->acq_seq, last_seq + 1) << "acq_seq counts processed samples only";
      last_seq = out.sample->acq_seq;
    } else if (out.event && out.event->type == EventType::kDecimated) {
      ++decimated;
    }
  }
  EXPECT_EQ(kept, 5);
  EXPECT_EQ(decimated, 5);
}

TEST(Acquisition, NonFiniteMessageRejected) {
  LowStateAcquisition acq(AcquisitionConfig(), go2());
  RawLowState raw = makeRaw(1, kMs, 1);
  raw.gyro[1] = std::numeric_limits<float>::quiet_NaN();
  const auto out = acq.process(raw);
  EXPECT_FALSE(out.sample);
  EXPECT_EQ(out.event->type, EventType::kInvalidMessage);
}
