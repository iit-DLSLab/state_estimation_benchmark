// Estimator adapters: smoke tests on the Go2 model, time-step semantics,
// smoother window sizes, and regression against the offline benchmark outputs.
//
// Regression (optional, skipped unless the environment provides the data):
//   SEB_REGRESSION_DATASET    directory with sensor_data.csv and feet_kinematics.csv (ANYmal GrandTour)
//   SEB_REGRESSION_REFERENCE  directory with muse/, iekf/, invariant_smoother/ fused_state.csv
//                             produced by the offline executables on the same dataset
//   SEB_REGRESSION_MAX_SAMPLES number of samples to compare (default 20000, 0 = all)

#include "seb_online/estimator_adapter.hpp"
#include "seb_online/foot_kinematics.hpp"
#include "seb_online/robot_mapping.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

using namespace seb;

namespace {

constexpr std::int64_t kMs = 1000000;

RobotMapping go2Mapping() {
  RobotMapping m = RobotMapping::go2Defaults();
  m.urdf_path = SEB_GO2_URDF;
  resolveImuFromUrdf(m);
  return m;
}

struct StandingSequence {
  explicit StandingSequence(const RobotMapping& m) : kin(m) {}

  // Robot standing still on four feet; stamps follow the given dt pattern [ms].
  SynchronizedMeasurement make(std::uint64_t id, std::int64_t stamp_ns, bool contact,
                               const Eigen::Vector3d& acc = Eigen::Vector3d(0, 0, 9.81)) {
    auto s = std::make_shared<SensorSample>();
    s->sample_id = id;
    s->acq_seq = id;
    s->stamp_ns = stamp_ns;
    for (int leg = 0; leg < 4; ++leg) s->q.segment<3>(3 * leg) << 0.0, 0.8, -1.5;
    s->acc = acc;
    s->gyro.setZero();
    SynchronizedMeasurement m;
    m.sensor = s;
    kin.compute(*s, m.kinematics);
    m.contacts.sample_id = id;
    m.contacts.stamp_ns = stamp_ns;
    m.contacts.stance = {contact, contact, contact, contact};
    m.contacts.all_valid = true;
    return m;
  }

  EstimationFrameKinematics kin;
};

double rotationAngle(const Eigen::Matrix3d& R) { return Eigen::AngleAxisd(R).angle(); }

}  // namespace

class EstimatorSmoke : public ::testing::TestWithParam<std::string> {};

TEST_P(EstimatorSmoke, StandingStillWithJitteredSampling) {
  const RobotMapping m = go2Mapping();
  EstimatorSettings settings;
  settings.type = GetParam();
  auto est = createEstimator(settings);
  StandingSequence seq(m);
  const int jitter_us[] = {2000, 1700, 2300, 2100, 1900, 40, 3960};  // includes a reception burst
  std::int64_t t = 1000 * kMs;
  std::int64_t t0 = t;
  for (int k = 0; k < 1500; ++k) {
    const double dt = k == 0 ? 0.0 : jitter_us[k % 7] * 1e-6;
    t += k == 0 ? 0 : jitter_us[k % 7] * 1000;
    StepTiming timing;
    timing.first = (k == 0);
    timing.dt_s = dt;
    timing.t_rel_s = static_cast<double>(t - t0) * 1e-9;
    const EstimatorOutput out = est->step(seq.make(k + 1, t, true), timing);
    ASSERT_TRUE(out.R_wE.allFinite() && out.v_wE.allFinite() && out.p_wE.allFinite()) << "step " << k;
    if (k > 100) {
      EXPECT_LT(out.v_wE.norm(), 0.05) << settings.type << " step " << k;
      EXPECT_LT(rotationAngle(out.R_wE), 0.02) << settings.type << " step " << k;
    }
  }
  // Reset brings the estimator back to its initial state.
  est->reset();
  StepTiming timing;
  timing.first = true;
  const EstimatorOutput out = est->step(seq.make(9999, t + 2 * kMs, true), timing);
  EXPECT_TRUE(out.v_wE.allFinite());
}

INSTANTIATE_TEST_SUITE_P(AllEstimators, EstimatorSmoke,
                         ::testing::Values("muse", "iekf", "invariant_smoother"));

TEST(EstimatorFactory, UnknownTypeRejected) {
  EstimatorSettings s;
  s.type = "ekf";
  EXPECT_THROW(createEstimator(s), std::invalid_argument);
}

TEST(EstimatorInit, GravityAlignedAttitude) {
  // Robot pitched by 0.2 rad: f_b = R^T (0,0,g).
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d f_b = R.transpose() * Eigen::Vector3d(0, 0, 9.81);
  const Eigen::Matrix3d R0 = gravityAlignedAttitude(f_b);
  EXPECT_TRUE((R0 * f_b).normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(rotationAngle(R0.transpose() * R), 0.0, 1e-9);
}

// Pure IMU integration (no contacts): with dt_mode=measured the velocity after a
// constant horizontal acceleration a over a duration T is a*T even for
// irregular sampling; the offline fixed-dt behaviour would give a*N*nominal_dt.
class MeasuredDt : public ::testing::TestWithParam<std::string> {};

TEST_P(MeasuredDt, PropagationUsesMeasuredIntervals) {
  const RobotMapping m = go2Mapping();
  EstimatorSettings settings;
  settings.type = GetParam();
  settings.common.init_attitude = InitAttitude::kIdentity;
  settings.common.nominal_dt_s = 0.002;
  // gravity constant used by each implementation's propagation
  const double g = settings.type == "iekf" ? 9.80665 : 9.81;
  auto est = createEstimator(settings);
  StandingSequence seq(m);
  const Eigen::Vector3d acc(1.0, 0.0, g);
  std::int64_t t = 0;
  EstimatorOutput out;
  const int n = 200;
  for (int k = 0; k < n; ++k) {
    const std::int64_t dt_ns = k == 0 ? 0 : ((k % 2) ? 1 * kMs : 5 * kMs);  // mean 3 ms != nominal 2 ms
    t += dt_ns;
    StepTiming timing;
    timing.first = (k == 0);
    timing.dt_s = static_cast<double>(dt_ns) * 1e-9;
    timing.t_rel_s = static_cast<double>(t) * 1e-9;
    out = est->step(seq.make(k + 1, t, false, acc), timing);
  }
  const double T = static_cast<double>(t) * 1e-9;
  EXPECT_NEAR(out.v_wE.x(), 1.0 * T, 0.05 * T) << settings.type;
}

INSTANTIATE_TEST_SUITE_P(InvariantEstimators, MeasuredDt, ::testing::Values("iekf", "invariant_smoother"));

TEST(Smoother, ConfigurableWindowSizes) {
  const RobotMapping m = go2Mapping();
  StandingSequence seq(m);
  for (int w : {1, 2, 3, 5, 10}) {
    EstimatorSettings settings;
    settings.type = "invariant_smoother";
    settings.smoother.window_size = w;
    auto est = createEstimator(settings);
    std::int64_t t = 0;
    for (int k = 0; k < 3 * w + 5; ++k) {
      t += k == 0 ? 0 : 2 * kMs + (k % 3) * 100000;
      StepTiming timing;
      timing.first = (k == 0);
      timing.dt_s = k == 0 ? 0.0 : static_cast<double>(2 * kMs + (k % 3) * 100000) * 1e-9;
      timing.t_rel_s = static_cast<double>(t) * 1e-9;
      const EstimatorOutput out = est->step(seq.make(k + 1, t, true), timing);
      ASSERT_TRUE(out.v_wE.allFinite()) << "window " << w << " step " << k;
      EXPECT_EQ(out.window_frames, std::min(k + 1, w + 1));
      EXPECT_GE(out.window_duration_s, 0.0);
    }
    // Window duration: newest - oldest frame stamp, NOT the output delay.
    EstimatorOutput out;
    std::int64_t t_first_in_window = 0;
    std::vector<std::int64_t> stamps;
    est->reset();
    t = 0;
    for (int k = 0; k < w + 4; ++k) {
      t += k == 0 ? 0 : 2 * kMs;
      stamps.push_back(t);
      StepTiming timing;
      timing.first = (k == 0);
      timing.dt_s = k == 0 ? 0.0 : 0.002;
      timing.t_rel_s = static_cast<double>(t) * 1e-9;
      out = est->step(seq.make(k + 1, t, true), timing);
    }
    t_first_in_window = stamps[stamps.size() - static_cast<std::size_t>(w) - 1];
    EXPECT_NEAR(out.window_duration_s, static_cast<double>(t - t_first_in_window) * 1e-9, 1e-12) << "window " << w;
  }
}

// With uniform sampling the per-interval dt option must not change the result.
TEST(Smoother, PerIntervalDtIsIdenticalForUniformSampling) {
  const RobotMapping m = go2Mapping();
  StandingSequence seq(m);
  EstimatorSettings a, b;
  a.type = b.type = "invariant_smoother";
  a.smoother.per_interval_dt = true;
  b.smoother.per_interval_dt = false;
  auto ea = createEstimator(a);
  auto eb = createEstimator(b);
  std::int64_t t = 0;
  for (int k = 0; k < 300; ++k) {
    t += k == 0 ? 0 : 2 * kMs;
    StepTiming timing;
    timing.first = (k == 0);
    timing.dt_s = k == 0 ? 0.0 : 0.002;
    timing.t_rel_s = static_cast<double>(t) * 1e-9;
    const Eigen::Vector3d acc(0.3 * std::sin(0.05 * k), 0.0, 9.81);
    const auto m1 = seq.make(k + 1, t, k % 50 < 40, acc);
    const EstimatorOutput oa = ea->step(m1, timing);
    const EstimatorOutput ob = eb->step(m1, timing);
    ASSERT_EQ(oa.v_wE, ob.v_wE) << "step " << k;
    ASSERT_EQ(oa.p_wE, ob.p_wE) << "step " << k;
    ASSERT_EQ(oa.R_wE, ob.R_wE) << "step " << k;
  }
}

// ---------------------------------------------------------------------------
// Regression against the offline benchmark outputs
// ---------------------------------------------------------------------------
namespace {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(tok);
  return out;
}

struct Csv {
  explicit Csv(const std::string& path) : in(path) {
    std::string line;
    if (!in.is_open() || !std::getline(in, line)) return;
    const auto h = split(line);
    for (std::size_t i = 0; i < h.size(); ++i) idx[h[i]] = static_cast<int>(i);
    ok = true;
  }
  bool next() {
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      row = split(line);
      return true;
    }
    return false;
  }
  double d(const std::string& c) const { return std::stod(row.at(static_cast<std::size_t>(idx.at(c)))); }
  const std::string& s(const std::string& c) const { return row.at(static_cast<std::size_t>(idx.at(c))); }
  bool b(const std::string& c) const {
    const std::string& v = s(c);
    if (v == "True" || v == "true" || v == "1") return true;
    if (v == "False" || v == "false" || v == "0") return false;
    return std::stod(v) != 0.0;
  }
  std::ifstream in;
  std::unordered_map<std::string, int> idx;
  std::vector<std::string> row;
  bool ok = false;
};

std::string fmt(double v) {
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss << std::setprecision(9) << v;
  return ss.str();
}

void runRegression(const std::string& type, const std::string& reference_csv) {
  const char* ds = std::getenv("SEB_REGRESSION_DATASET");
  const char* ref = std::getenv("SEB_REGRESSION_REFERENCE");
  if (!ds || !ref) GTEST_SKIP() << "SEB_REGRESSION_DATASET / SEB_REGRESSION_REFERENCE not set";
  long max_samples = 20000;
  if (const char* m = std::getenv("SEB_REGRESSION_MAX_SAMPLES")) max_samples = std::atol(m);

  Csv sensor(std::string(ds) + "/sensor_data.csv");
  Csv feet(std::string(ds) + "/feet_kinematics.csv");
  Csv offline(std::string(ref) + "/" + reference_csv);
  ASSERT_TRUE(sensor.ok && feet.ok && offline.ok) << "cannot read regression data";

  // Offline benchmark configuration: ANYmal IMU mounted 180 deg about y
  // (b_R_imu in main_muse.cpp / sign flips in the KAIST mains), no lever arm,
  // identity initial attitude, offline time-step handling.
  Eigen::Matrix3d R_base_imu;
  R_base_imu << -1, 0, 0, 0, 1, 0, 0, 0, -1;
  EstimatorSettings settings;
  settings.type = type;
  settings.common.init_attitude = InitAttitude::kIdentity;
  settings.common.dt_mode = DtMode::kOfflineCompat;
  settings.common.nominal_dt_s = 0.0025;  // dt_init of the offline mains
  settings.muse.use_updated_attitude = false;
  auto est = createEstimator(settings);

  const char* legs[4] = {"LF", "RF", "LH", "RH"};
  const char* jn[3] = {"HAA", "HFE", "KFE"};
  double t_begin = 0.0, t_prev = 0.0;
  long n = 0;
  double max_rot_err = 0.0;
  while (sensor.next() && feet.next() && offline.next()) {
    if (max_samples > 0 && n >= max_samples) break;
    const double t_abs = sensor.d("t");
    auto s = std::make_shared<SensorSample>();
    s->sample_id = static_cast<std::uint64_t>(n + 1);
    const Eigen::Vector3d gyro(sensor.d("imu_wx"), sensor.d("imu_wy"), sensor.d("imu_wz"));
    const Eigen::Vector3d acc(sensor.d("imu_ax"), sensor.d("imu_ay"), sensor.d("imu_az"));
    s->gyro = R_base_imu * gyro;
    s->acc = R_base_imu * acc;
    for (int l = 0; l < 4; ++l) {
      for (int j = 0; j < 3; ++j) {
        s->q[3 * l + j] = sensor.d(std::string("joint_pos_") + legs[l] + "_" + jn[j]);
        s->dq[3 * l + j] = sensor.d(std::string("joint_vel_") + legs[l] + "_" + jn[j]);
      }
    }
    SynchronizedMeasurement m;
    m.sensor = s;
    for (int l = 0; l < 4; ++l) {
      const std::string L = legs[l];
      m.kinematics.p_foot[l] = Eigen::Vector3d(feet.d("p_" + L + "_x"), feet.d("p_" + L + "_y"), feet.d("p_" + L + "_z"));
      m.kinematics.v_foot[l] = Eigen::Vector3d(feet.d("v_" + L + "_x"), feet.d("v_" + L + "_y"), feet.d("v_" + L + "_z"));
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          m.kinematics.J_foot[l](r, c) = feet.d("J_" + L + "_" + std::to_string(r) + std::to_string(c));
        }
      }
      m.contacts.stance[l] = sensor.b("contact_" + L);
    }
    StepTiming timing;
    timing.first = (n == 0);
    if (n == 0) {
      t_begin = t_abs;
      t_prev = t_abs;
    }
    timing.dt_s = t_abs - t_prev;  // as in the offline mains
    timing.t_rel_s = t_abs - t_begin;
    t_prev = t_abs;

    const EstimatorOutput out = est->step(m, timing);

    // Positions/velocities: identical once printed like the offline CSV.
    const char* pc[3] = {"px", "py", "pz"};
    const char* vc[3] = {"vx", "vy", "vz"};
    for (int i = 0; i < 3; ++i) {
      ASSERT_EQ(fmt(out.p_wE[i]), offline.s(pc[i])) << type << " sample " << n << " " << pc[i];
      ASSERT_EQ(fmt(out.v_wE[i]), offline.s(vc[i])) << type << " sample " << n << " " << vc[i];
    }
    const Eigen::Quaterniond q_off(offline.d("qw"), offline.d("qx"), offline.d("qy"), offline.d("qz"));
    const double rot_err = rotationAngle(q_off.normalized().toRotationMatrix().transpose() * out.R_wE);
    max_rot_err = std::max(max_rot_err, rot_err);
    ASSERT_LT(rot_err, 1e-7) << type << " sample " << n;
    ++n;
  }
  EXPECT_GT(n, 0);
  std::cout << "[regression] " << type << ": " << n << " samples identical to the offline output "
            << "(position/velocity at 1e-9 print precision, max orientation difference " << max_rot_err
            << " rad)\n";
}

}  // namespace

TEST(OfflineRegression, Muse) { runRegression("muse", "muse/fused_state.csv"); }
TEST(OfflineRegression, Iekf) { runRegression("iekf", "iekf/fused_state.csv"); }
TEST(OfflineRegression, InvariantSmoother) {
  runRegression("invariant_smoother", "invariant_smoother/fused_state.csv");
}
