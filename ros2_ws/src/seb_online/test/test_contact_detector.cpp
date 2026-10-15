// Go2 foot-force contact detection (MUSE reference strategy).

#include "seb_online/contact_detector.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

using namespace seb;

namespace {

SensorSample sampleWithForces(std::uint64_t id, double lf, double rf, double lh, double rh) {
  SensorSample s;
  s.sample_id = id;
  s.stamp_ns = static_cast<std::int64_t>(id) * 2000000;
  s.foot_force = {lf, rf, lh, rh};
  return s;
}

}  // namespace

TEST(ContactDetector, ThresholdWithoutHysteresisMatchesReference) {
  ContactDetectionConfig cfg;  // reference defaults: threshold 20, no hysteresis
  Go2FootForceContactDetector det(cfg);
  ContactResult out;
  det.detect(sampleWithForces(7, 20.0, 20.5, -25.0, 0.0), out);
  EXPECT_FALSE(out.stance[LF]);  // strictly greater than the threshold
  EXPECT_TRUE(out.stance[RF]);
  EXPECT_TRUE(out.stance[LH]);   // |force| as in the reference
  EXPECT_FALSE(out.stance[RH]);
  EXPECT_DOUBLE_EQ(out.metric[LH], 25.0);
  EXPECT_TRUE(out.all_valid);
  // Identity and time of the source sample are preserved.
  EXPECT_EQ(out.sample_id, 7u);
  EXPECT_EQ(out.stamp_ns, 14000000);
}

TEST(ContactDetector, Hysteresis) {
  ContactDetectionConfig cfg;
  cfg.use_hysteresis = true;
  cfg.threshold_low = 10.0;
  cfg.threshold_high = 30.0;
  Go2FootForceContactDetector det(cfg);
  ContactResult out;
  const double seq[] = {20, 31, 20, 11, 9, 20, 29, 31};
  const bool expected[] = {false, true, true, true, false, false, false, true};
  for (int i = 0; i < 8; ++i) {
    det.detect(sampleWithForces(i, seq[i], 0, 0, 0), out);
    EXPECT_EQ(out.stance[LF], expected[i]) << "step " << i << " force " << seq[i];
  }
  det.reset();
  det.detect(sampleWithForces(9, 20, 0, 0, 0), out);
  EXPECT_FALSE(out.stance[LF]) << "reset clears the hysteresis state";
}

TEST(ContactDetector, ImplausibleReadingIsInvalidAndNoContact) {
  ContactDetectionConfig cfg;
  cfg.valid_min = -50.0;
  cfg.valid_max = 500.0;
  Go2FootForceContactDetector det(cfg);
  ContactResult out;
  det.detect(sampleWithForces(1, 600.0, 40.0, -60.0, 40.0), out);
  EXPECT_FALSE(out.valid[LF]);
  EXPECT_FALSE(out.stance[LF]);
  EXPECT_FALSE(out.valid[LH]);
  EXPECT_TRUE(out.valid[RF]);
  EXPECT_TRUE(out.stance[RF]);
  EXPECT_FALSE(out.all_valid);
}
