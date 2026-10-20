// End-to-end tests of the threaded pipeline with synthetic LowState input:
// real worker threads with deliberately different completion times, lost
// results, a slow estimator and time-base jumps.

#include "seb_online/pipeline.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <mutex>
#include <thread>

using namespace seb;
using namespace seb::test;

namespace {

constexpr std::int64_t kMs = 1000000;

struct Harness {
  explicit Harness(PipelineConfig cfg, std::unique_ptr<RecordingEstimator> est, Pipeline::Hooks hooks = {}) {
    estimator = est.get();
    Pipeline::Sinks sinks;
    sinks.output = [this](StateEstimate& s) {
      s.publish_steady_ns = steadyNowNs();
      s.publish_ros_ns = s.stamp_ns + 777;  // marker: publication time differs from state time
    };
    sinks.record = [this](const StateEstimate& s) {
      std::lock_guard<std::mutex> lock(mutex);
      states.push_back(s);
    };
    sinks.event = [this](const PipelineEvent& e) {
      std::lock_guard<std::mutex> lock(mutex);
      events.push_back(e);
    };
    // Default worker computations encode the sample in the result (see bundleIsCoherent).
    if (!hooks.kinematics) {
      hooks.kinematics = [](const SensorSample& s, KinematicsResult& out, bool) {
        const auto start = out.start_steady_ns;
        out = makeKinematics(s);
        out.start_steady_ns = start;
      };
    }
    if (!hooks.contact) {
      hooks.contact = [](const SensorSample& s, ContactResult& out, bool) {
        const auto start = out.start_steady_ns;
        out = makeContact(s);
        out.start_steady_ns = start;
      };
    }
    pipeline = std::make_unique<Pipeline>(cfg, std::move(est), sinks, hooks);
    pipeline->start();
  }

  ~Harness() { pipeline->stop(); }

  std::vector<PipelineEvent> eventsOf(EventType t) {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<PipelineEvent> r;
    for (const auto& e : events) {
      if (e.type == t) r.push_back(e);
    }
    return r;
  }

  std::mutex mutex;
  std::vector<StateEstimate> states;
  std::vector<PipelineEvent> events;
  RecordingEstimator* estimator = nullptr;
  std::unique_ptr<Pipeline> pipeline;
};

PipelineConfig baseConfig() {
  PipelineConfig cfg;
  cfg.mapping = RobotMapping::go2Defaults();
  cfg.sync.worker_timeout_ns = 200 * kMs;
  cfg.estimator_worker.max_input_age_s = 0.0;  // disabled unless a test enables it
  cfg.estimator_worker.max_gap_s = 1.0;
  cfg.estimator_worker.warmup_samples = 0;
  return cfg;
}

void busyFreeSleepUs(int us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

}  // namespace

// Workers finish in different orders (different per-sample delays); the estimator
// must receive every sample exactly once, in order, with sensor/kinematics/contact
// from the same sample, positive dt equal to the stamp difference, and the state
// stamped with the sample time.
TEST(Pipeline, ExactSynchronizationWithJitteredWorkers) {
  Pipeline::Hooks hooks;
  hooks.kinematics = [](const SensorSample& s, KinematicsResult& out, bool) {
    busyFreeSleepUs(static_cast<int>(s.sample_id % 3) * 300);
    const auto start = out.start_steady_ns;
    out = makeKinematics(s);
    out.start_steady_ns = start;
  };
  hooks.contact = [](const SensorSample& s, ContactResult& out, bool) {
    busyFreeSleepUs(static_cast<int>((s.sample_id + 1) % 4) * 250);
    const auto start = out.start_steady_ns;
    out = makeContact(s);
    out.start_steady_ns = start;
  };
  Harness h(baseConfig(), std::make_unique<RecordingEstimator>(), hooks);

  const int n = 300;
  std::map<std::uint64_t, std::int64_t> receive_of_acq;
  for (int i = 0; i < n; ++i) {
    ASSERT_TRUE(h.pipeline->waitForInFlightBelow(8, std::chrono::seconds(5)));
    const std::int64_t t = (i + 1) * 2 * kMs;
    h.pipeline->onLowState(makeRaw(i, t, 1000 + 2 * i));
    receive_of_acq[i + 1] = t;
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(10)));

  const auto calls = h.estimator->snapshot();
  ASSERT_EQ(calls.size(), static_cast<std::size_t>(n));
  for (std::size_t i = 0; i < calls.size(); ++i) {
    EXPECT_TRUE(calls[i].coherent) << "sample " << calls[i].sample_id << " mixed data from different samples";
    EXPECT_EQ(calls[i].sample_id, i + 1);
    EXPECT_EQ(calls[i].first, i == 0);
    if (i > 0) EXPECT_DOUBLE_EQ(calls[i].dt_s, 0.002);
  }
  EXPECT_EQ(h.estimator->concurrent_calls.load(), 0);

  std::lock_guard<std::mutex> lock(h.mutex);
  ASSERT_EQ(h.states.size(), static_cast<std::size_t>(n));
  for (const auto& s : h.states) {
    EXPECT_EQ(s.stamp_ns, receive_of_acq[s.acq_seq]) << "state stamp must be the source sample time";
    EXPECT_EQ(s.publish_ros_ns, s.stamp_ns + 777) << "publication time is reported separately";
    EXPECT_EQ(s.stamp_source, StampSource::kReceiveTime);
    EXPECT_GE(s.kin_end_steady_ns, s.kin_start_steady_ns);
    EXPECT_GE(s.release_steady_ns, std::max(s.kin_end_steady_ns, s.contact_end_steady_ns));
    EXPECT_GE(s.est_end_steady_ns, s.est_start_steady_ns);
  }
  const auto st = h.pipeline->syncStats();
  EXPECT_EQ(st.identity_mismatches, 0u);
  EXPECT_EQ(st.late_results, 0u);
}

// The contact result of one sample is lost: after the timeout the sample is
// reported as missing, the estimator never receives it, and the next update
// spans the gap explicitly (kGap, DEGRADED, dt = 2 periods).
TEST(Pipeline, MissingContactResultFollowsTimeoutPolicy) {
  PipelineConfig cfg = baseConfig();
  cfg.sync.worker_timeout_ns = 20 * kMs;
  Pipeline::Hooks hooks;
  hooks.deliver_contact = [](const ContactResult& c) { return c.sample_id != 10; };
  Harness h(cfg, std::make_unique<RecordingEstimator>(), hooks);

  for (int i = 0; i < 30; ++i) {
    ASSERT_TRUE(h.pipeline->waitForInFlightBelow(4, std::chrono::seconds(5)));
    h.pipeline->onLowState(makeRaw(i, (i + 1) * 2 * kMs, 1000 + 2 * i));
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(10)));

  const auto calls = h.estimator->snapshot();
  ASSERT_EQ(calls.size(), 29u);
  for (const auto& c : calls) {
    EXPECT_NE(c.sample_id, 10u);
    EXPECT_TRUE(c.coherent);
  }
  const auto missing = h.eventsOf(EventType::kMissingContact);
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0].sample_id, 10u);
  const auto gaps = h.eventsOf(EventType::kGap);
  ASSERT_EQ(gaps.size(), 1u);
  EXPECT_EQ(gaps[0].sample_id, 11u);

  std::lock_guard<std::mutex> lock(h.mutex);
  bool found = false;
  for (const auto& s : h.states) {
    if (s.sample_id == 11) {
      found = true;
      EXPECT_EQ(s.status, EstimateStatus::kDegraded);
      EXPECT_EQ(s.missing_before, 1u);
      EXPECT_NEAR(s.dt_s, 0.004, 1e-12);
    }
  }
  EXPECT_TRUE(found);
}

// Estimator slower than the input rate: bounded queue, explicit backlog drops,
// never a non-positive dt, every dispatched sample accounted for.
TEST(Pipeline, SlowEstimatorBoundedBacklog) {
  PipelineConfig cfg = baseConfig();
  cfg.sync.output_capacity = 5;
  cfg.sync.output_overflow = OverflowPolicy::kDropOldest;
  auto est = std::make_unique<RecordingEstimator>([] { busyFreeSleepUs(3000); });
  Harness h(cfg, std::move(est));

  const int n = 200;
  for (int i = 0; i < n; ++i) {
    h.pipeline->onLowState(makeRaw(i, (i + 1) * kMs, 1000 + i));
    busyFreeSleepUs(1000);
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(20)));

  const auto st = h.pipeline->syncStats();
  const auto c = h.pipeline->counters();
  const auto calls = h.estimator->snapshot();
  EXPECT_LE(st.max_output_depth, 5u);
  EXPECT_GT(st.backlog_drops, 0u) << "the overload must be visible";
  EXPECT_EQ(calls.size() + st.backlog_drops + c.events[static_cast<int>(EventType::kPipelineFull)],
            static_cast<std::size_t>(n));
  EXPECT_EQ(h.eventsOf(EventType::kEstimatorBacklog).size(), st.backlog_drops);
  for (std::size_t i = 1; i < calls.size(); ++i) {
    EXPECT_GT(calls[i].sample_id, calls[i - 1].sample_id);
    EXPECT_GT(calls[i].dt_s, 0.0);
    EXPECT_TRUE(calls[i].coherent);
  }
  EXPECT_EQ(h.estimator->concurrent_calls.load(), 0);
  std::size_t flagged = 0;
  {
    std::lock_guard<std::mutex> lock(h.mutex);
    for (const auto& s : h.states) {
      if (s.missing_before > 0) ++flagged;
    }
  }
  EXPECT_GT(flagged, 0u) << "updates after dropped samples are flagged";
}

TEST(Pipeline, StaleInputIsDroppedNotProcessed) {
  PipelineConfig cfg = baseConfig();
  cfg.sync.output_capacity = 100;
  cfg.estimator_worker.max_input_age_s = 0.005;
  auto est = std::make_unique<RecordingEstimator>([] { busyFreeSleepUs(10000); });
  Harness h(cfg, std::move(est));
  for (int i = 0; i < 20; ++i) {
    RawLowState raw = makeRaw(i, (i + 1) * kMs, 1000 + i);
    raw.receive_steady_ns = steadyNowNs();  // age is measured on the steady clock
    h.pipeline->onLowState(raw);
    busyFreeSleepUs(1000);
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(20)));
  EXPECT_GT(h.eventsOf(EventType::kStaleInput).size(), 0u);
  const auto calls = h.estimator->snapshot();
  EXPECT_LT(calls.size(), 20u);
  for (std::size_t i = 1; i < calls.size(); ++i) EXPECT_GT(calls[i].dt_s, 0.0);
}

// The time base jumps backwards (e.g. bag restarted): the pipeline is reset and
// the estimator restarts from a first sample instead of receiving a negative dt.
TEST(Pipeline, BackwardTimeJumpReinitialisesEstimator) {
  Harness h(baseConfig(), std::make_unique<RecordingEstimator>());
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(h.pipeline->waitForInFlightBelow(4, std::chrono::seconds(5)));
    h.pipeline->onLowState(makeRaw(i, 10000 * kMs + (i + 1) * 2 * kMs, 50000 + 2 * i));
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(5)));
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(h.pipeline->waitForInFlightBelow(4, std::chrono::seconds(5)));
    h.pipeline->onLowState(makeRaw(100 + i, (i + 1) * 2 * kMs, 10 + 2 * i));
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(5)));
  const auto calls = h.estimator->snapshot();
  ASSERT_EQ(calls.size(), 20u);
  EXPECT_TRUE(calls[0].first);
  EXPECT_TRUE(calls[10].first) << "new epoch must restart the estimator";
  for (const auto& c : calls) EXPECT_GE(c.dt_s, 0.0);
  EXPECT_EQ(h.eventsOf(EventType::kResetTimeJump).size(), 1u);
}

// Exact duplicates delivered by DDS are dropped before reaching the workers.
TEST(Pipeline, DuplicateMessagesNeverReachTheEstimator) {
  Harness h(baseConfig(), std::make_unique<RecordingEstimator>());
  for (int i = 0; i < 10; ++i) {
    RawLowState raw = makeRaw(i, (i + 1) * 2 * kMs, 1000 + 2 * i);
    h.pipeline->onLowState(raw);
    raw.receive_ros_ns += kMs / 2;
    h.pipeline->onLowState(raw);  // same payload and tick
  }
  ASSERT_TRUE(h.pipeline->drain(std::chrono::seconds(5)));
  EXPECT_EQ(h.estimator->snapshot().size(), 10u);
  EXPECT_EQ(h.eventsOf(EventType::kDuplicate).size(), 10u);
}

// Estimator worker in isolation: an earlier stamp than the last processed one can
// only come from a new time-base epoch; it must restart the estimator, never lock
// it into rejecting every following sample.
TEST(EstimatorWorker, EarlierStampRestartsInsteadOfBlocking) {
  BoundedQueue<SynchronizedMeasurement> queue(8);
  auto est = std::make_unique<RecordingEstimator>();
  RecordingEstimator* rec = est.get();
  std::vector<PipelineEvent> events;
  EstimatorWorkerConfig cfg;
  cfg.max_input_age_s = 0.0;
  EstimatorWorker worker(cfg, RobotMapping::go2Defaults(), std::move(est), queue, nullptr, nullptr,
                         [&](const PipelineEvent& e) { events.push_back(e); }, nullptr, nullptr);
  auto bundle = [](std::uint64_t id, std::int64_t stamp) {
    SynchronizedMeasurement m;
    m.sensor = makeSample(id, stamp);
    m.kinematics = makeKinematics(*m.sensor);
    m.contacts = makeContact(*m.sensor);
    return m;
  };
  const std::int64_t stamps[] = {100 * kMs, 102 * kMs, 104 * kMs, 2 * kMs, 4 * kMs, 6 * kMs};
  for (int i = 0; i < 6; ++i) {
    auto m = bundle(i + 1, stamps[i]);
    worker.processOne(m, 0);
  }
  const auto calls = rec->snapshot();
  ASSERT_EQ(calls.size(), 6u);
  EXPECT_TRUE(calls[0].first);
  EXPECT_TRUE(calls[3].first) << "earlier stamp starts a new estimator epoch";
  EXPECT_DOUBLE_EQ(calls[4].dt_s, 0.002);
  EXPECT_DOUBLE_EQ(calls[5].dt_s, 0.002);
  int jumps = 0;
  for (const auto& e : events) jumps += e.type == EventType::kResetTimeJump;
  EXPECT_EQ(jumps, 1);
}

// A sample with no leg in stance is published with zero velocity and the position
// of the previous estimate; the estimator state is forced the same way
// (holdStationary) and one event marks the episode.
TEST(EstimatorWorker, NoStanceForcesZeroVelocityAndHoldsPosition) {
  BoundedQueue<SynchronizedMeasurement> queue(8);
  auto est = std::make_unique<RecordingEstimator>();
  RecordingEstimator* rec = est.get();
  std::vector<PipelineEvent> events;
  std::vector<StateEstimate> outputs;
  EstimatorWorkerConfig cfg;
  cfg.max_input_age_s = 0.0;
  EstimatorWorker worker(cfg, RobotMapping::go2Defaults(), std::move(est), queue,
                         [&](StateEstimate& s) { outputs.push_back(s); }, nullptr,
                         [&](const PipelineEvent& e) { events.push_back(e); }, nullptr, nullptr);
  const bool stance[] = {true, true, false, false, false, true, false, true};
  for (int i = 0; i < 8; ++i) {
    // Estimator output that keeps moving: the hold must override it.
    rec->next_output.p_wE = Eigen::Vector3d(0.1 * (i + 1), 0.0, 0.3);
    rec->next_output.v_wE = Eigen::Vector3d(0.5, 0.2, -0.1);
    SynchronizedMeasurement m;
    m.sensor = makeSample(i + 1, (100 + 2 * i) * kMs);
    m.kinematics = makeKinematics(*m.sensor);
    m.contacts = makeContact(*m.sensor);
    m.contacts.stance.fill(false);
    m.contacts.stance[i % kNumLegs] = stance[i];  // one leg is enough to count as stance
    worker.processOne(m, 0);
  }
  ASSERT_EQ(outputs.size(), 8u);
  // Holds: samples 3, 4, 5 at the position of sample 2, sample 7 at that of sample 6.
  ASSERT_EQ(rec->held_positions.size(), 4u);
  for (int i : {2, 3, 4, 6}) {
    EXPECT_TRUE(outputs[i].v_wb.isZero()) << i;
    EXPECT_TRUE(outputs[i].v_b.isZero()) << i;
  }
  const Eigen::Vector3d r = -RobotMapping::go2Defaults().p_base_imu;  // identity attitude
  EXPECT_TRUE(outputs[2].p_wb.isApprox(Eigen::Vector3d(0.2, 0.0, 0.3) + r));
  EXPECT_TRUE(outputs[4].p_wb.isApprox(outputs[2].p_wb));
  EXPECT_TRUE(outputs[6].p_wb.isApprox(Eigen::Vector3d(0.6, 0.0, 0.3) + r));
  EXPECT_TRUE(outputs[5].v_wb.isApprox(Eigen::Vector3d(0.5, 0.2, -0.1))) << "estimator output again with stance";
  int holds = 0;
  for (const auto& e : events) holds += e.type == EventType::kNoStanceHold;
  EXPECT_EQ(holds, 2) << "one event per no-stance episode";
}

// Disabled, or an estimator without holdStationary (MUSE): output passes through unchanged.
TEST(EstimatorWorker, NoStanceHoldDisabledOrUnsupported) {
  for (const bool enabled : {false, true}) {
    BoundedQueue<SynchronizedMeasurement> queue(8);
    EstimatorWorkerConfig cfg;
    cfg.max_input_age_s = 0.0;
    cfg.hold_without_stance = enabled;
    auto est = std::make_unique<RecordingEstimator>();
    est->supports_hold = !enabled;  // enabled: estimator that does not implement the hold
    est->next_output.v_wE = Eigen::Vector3d(0.5, 0.0, 0.0);
    RecordingEstimator* rec = est.get();
    std::vector<StateEstimate> outputs;
    std::vector<PipelineEvent> events;
    EstimatorWorker worker(cfg, RobotMapping::go2Defaults(), std::move(est), queue,
                           [&](StateEstimate& s) { outputs.push_back(s); }, nullptr,
                           [&](const PipelineEvent& e) { events.push_back(e); }, nullptr, nullptr);
    SynchronizedMeasurement m;
    m.sensor = makeSample(1, 100 * kMs);
    m.kinematics = makeKinematics(*m.sensor);
    m.contacts = makeContact(*m.sensor);
    m.contacts.stance.fill(false);
    worker.processOne(m, 0);
    EXPECT_TRUE(rec->held_positions.empty()) << enabled;
    EXPECT_DOUBLE_EQ(outputs.at(0).v_wb.x(), 0.5) << enabled;
    for (const auto& e : events) EXPECT_NE(e.type, EventType::kNoStanceHold) << enabled;
  }
}

// Stance after a hold: a short hold just ends, a hold >= hold_restart_s restarts the
// estimator at the held position (next step is a first step) with roll/pitch from
// the accelerometer and the heading of the last estimate.
TEST(EstimatorWorker, LongHoldRestartsEstimatorAtHeldPose) {
  BoundedQueue<SynchronizedMeasurement> queue(8);
  auto est = std::make_unique<RecordingEstimator>();
  RecordingEstimator* rec = est.get();
  std::vector<PipelineEvent> events;
  EstimatorWorkerConfig cfg;
  cfg.max_input_age_s = 0.0;
  cfg.max_gap_s = 1.0;
  cfg.hold_restart_s = 0.5;
  EstimatorWorker worker(cfg, RobotMapping::go2Defaults(), std::move(est), queue, nullptr, nullptr,
                         [&](const PipelineEvent& e) { events.push_back(e); }, nullptr, nullptr);
  const double yaw = 0.7;
  rec->next_output.R_wE = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  rec->next_output.p_wE = Eigen::Vector3d(1.0, 2.0, 0.3);
  // stance: sample 1; no stance: samples 2-3 (0.2 s); stance: 4; no stance: 5-9 (0.8 s); stance: 10
  const std::int64_t stamps_ms[] = {100, 200, 300, 400, 500, 700, 900, 1100, 1300, 1400};
  const bool stance[] = {true, false, false, true, false, false, false, false, false, true};
  const Eigen::Vector3d acc(0.8, -0.5, 9.3);  // tilted robot at rest
  for (int i = 0; i < 10; ++i) {
    SynchronizedMeasurement m;
    auto sample = std::make_shared<SensorSample>(*makeSample(i + 1, stamps_ms[i] * kMs));
    sample->acc = acc;
    m.sensor = sample;
    m.kinematics = makeKinematics(*m.sensor);
    m.contacts = makeContact(*m.sensor);
    m.contacts.stance.fill(stance[i]);
    worker.processOne(m, 0);
  }
  ASSERT_EQ(rec->restarts.size(), 1u) << "only the hold of 0.8 s restarts";
  EXPECT_TRUE(rec->restarts[0].second.isApprox(Eigen::Vector3d(1.0, 2.0, 0.3)));
  const Eigen::Matrix3d& R = rec->restarts[0].first;
  EXPECT_NEAR(std::atan2(R(1, 0), R(0, 0)), yaw, 1e-9) << "heading kept";
  EXPECT_TRUE((R.transpose() * Eigen::Vector3d::UnitZ()).isApprox(acc.normalized(), 1e-9))
      << "roll/pitch from the accelerometer";
  const auto calls = rec->snapshot();
  ASSERT_EQ(calls.size(), 10u);
  EXPECT_TRUE(calls[9].first) << "restart: first step after it";
  EXPECT_FALSE(calls[3].first) << "short hold: no restart";
  int restarts = 0;
  for (const auto& e : events) restarts += e.type == EventType::kHoldRestart;
  EXPECT_EQ(restarts, 1);
}
