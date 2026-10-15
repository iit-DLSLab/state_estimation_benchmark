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
