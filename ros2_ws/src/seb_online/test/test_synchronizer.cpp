// Exact sample-id synchronisation: pairing, ordering, timeouts, bounds.

#include "seb_online/synchronizer.hpp"
#include "seb_online/thread_utils.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <condition_variable>
#include <map>
#include <random>

using namespace seb;
using namespace seb::test;

namespace {

constexpr std::int64_t kMs = 1000000;

struct Fixture {
  explicit Fixture(SynchronizerConfig cfg = SynchronizerConfig())
      : sync(cfg, [this](const PipelineEvent& e) { events.push_back(e); }, [this] { return now; }) {}

  SensorSamplePtr add(std::uint64_t id) {
    auto s = makeSample(id, static_cast<std::int64_t>(id) * 2 * kMs);
    samples[id] = s;
    EXPECT_TRUE(sync.registerSample(s, now));
    return s;
  }

  std::vector<SynchronizedMeasurement> drain() {
    std::vector<SynchronizedMeasurement> out;
    while (auto m = sync.output().tryPop()) out.push_back(std::move(*m));
    return out;
  }

  std::vector<PipelineEvent> eventsOf(EventType t) const {
    std::vector<PipelineEvent> r;
    for (const auto& e : events) {
      if (e.type == t) r.push_back(e);
    }
    return r;
  }

  std::int64_t now = 0;
  std::vector<PipelineEvent> events;
  std::map<std::uint64_t, SensorSamplePtr> samples;
  ExactSampleSynchronizer sync;
};

}  // namespace

// Many samples, the two workers complete in different random orders.
// Every released bundle must contain sensor, kinematics and contact of ONE sample,
// and bundles must come out in increasing sample order.
TEST(Synchronizer, ExactMatchUnderShuffledCompletionOrder) {
  SynchronizerConfig cfg;
  cfg.capacity = 128;
  cfg.output_capacity = 128;
  Fixture f(cfg);
  const int n = 100;
  for (int id = 1; id <= n; ++id) f.add(id);

  struct Job {
    bool kin;
    std::uint64_t id;
  };
  std::vector<Job> jobs;
  for (int id = 1; id <= n; ++id) {
    jobs.push_back({true, static_cast<std::uint64_t>(id)});
    jobs.push_back({false, static_cast<std::uint64_t>(id)});
  }
  std::mt19937 rng(42);
  std::shuffle(jobs.begin(), jobs.end(), rng);
  for (const auto& j : jobs) {
    const auto& s = *f.samples[j.id];
    if (j.kin) {
      f.sync.submitKinematics(makeKinematics(s));
    } else {
      f.sync.submitContact(makeContact(s));
    }
  }

  const auto out = f.drain();
  ASSERT_EQ(out.size(), static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    EXPECT_EQ(out[i].sensor->sample_id, static_cast<std::uint64_t>(i + 1));
    EXPECT_TRUE(bundleIsCoherent(out[i])) << "bundle " << i + 1 << " mixes samples";
  }
  const auto st = f.sync.stats();
  EXPECT_EQ(st.identity_mismatches, 0u);
  EXPECT_EQ(st.unknown_results, 0u);
  EXPECT_EQ(st.late_results, 0u);
  EXPECT_TRUE(f.events.empty());
}

// Kinematics of sample k is delayed while contact of k+1 (and kinematics of k+1)
// are already available: k+1 must not be paired with k, and nothing is released
// before k is complete.
TEST(Synchronizer, DelayedKinematicsIsNeverPairedWithNextSample) {
  Fixture f;
  auto k = f.add(1);
  auto k1 = f.add(2);

  f.sync.submitContact(makeContact(*k1));
  f.sync.submitKinematics(makeKinematics(*k1));
  f.sync.submitContact(makeContact(*k));
  EXPECT_EQ(f.sync.output().size(), 0u) << "sample k+1 released before k / paired with k";

  f.sync.submitKinematics(makeKinematics(*k));
  const auto out = f.drain();
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].sensor->sample_id, 1u);
  EXPECT_EQ(out[1].sensor->sample_id, 2u);
  EXPECT_TRUE(bundleIsCoherent(out[0]));
  EXPECT_TRUE(bundleIsCoherent(out[1]));
}

// A contact result that never arrives: after the timeout the sample is expired
// (recorded with its id), never released, and the following samples are released.
// A result arriving afterwards is discarded as late.
TEST(Synchronizer, MissingContactFollowsTimeoutPolicy) {
  SynchronizerConfig cfg;
  cfg.worker_timeout_ns = 10 * kMs;
  Fixture f(cfg);
  auto s1 = f.add(1);
  auto s2 = f.add(2);
  auto s3 = f.add(3);
  for (auto& s : {s1, s2, s3}) f.sync.submitKinematics(makeKinematics(*s));
  f.sync.submitContact(makeContact(*s1));
  f.sync.submitContact(makeContact(*s3));

  auto out = f.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor->sample_id, 1u);

  f.now = 9 * kMs;  // before the deadline: still waiting, 3 held back for order
  f.sync.expireOverdue(f.now);
  EXPECT_TRUE(f.drain().empty());
  EXPECT_TRUE(f.eventsOf(EventType::kMissingContact).empty());

  f.now = 11 * kMs;
  f.sync.expireOverdue(f.now);
  const auto missing = f.eventsOf(EventType::kMissingContact);
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0].sample_id, 2u);
  out = f.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor->sample_id, 3u);
  EXPECT_TRUE(bundleIsCoherent(out[0]));

  // The contact of sample 2 finally arrives: it must not be used.
  f.sync.submitContact(makeContact(*s2));
  EXPECT_TRUE(f.drain().empty());
  const auto late = f.eventsOf(EventType::kLateResult);
  ASSERT_EQ(late.size(), 1u);
  EXPECT_EQ(late[0].sample_id, 2u);
  EXPECT_EQ(f.sync.stats().expired_missing_contact, 1u);
}

TEST(Synchronizer, MissingBothResultsIsReported) {
  SynchronizerConfig cfg;
  cfg.worker_timeout_ns = 5 * kMs;
  Fixture f(cfg);
  f.add(1);
  f.now = 6 * kMs;
  f.sync.expireOverdue(f.now);
  const auto ev = f.eventsOf(EventType::kMissingBoth);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_EQ(ev[0].sample_id, 1u);
}

// Source timestamps are carried unchanged; results whose stamp differs from the
// sample they claim to belong to are rejected (never silently re-stamped).
TEST(Synchronizer, TimestampIntegrity) {
  Fixture f;
  auto s = f.add(1);
  f.sync.submitKinematics(makeKinematics(*s));
  ContactResult c = makeContact(*s);
  f.sync.submitContact(c);
  auto out = f.drain();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sensor->stamp_ns, s->stamp_ns);
  EXPECT_EQ(out[0].kinematics.stamp_ns, s->stamp_ns);
  EXPECT_EQ(out[0].contacts.stamp_ns, s->stamp_ns);
  EXPECT_EQ(out[0].sensor.get(), s.get()) << "the very same immutable sample object is forwarded";

  auto s2 = f.add(2);
  KinematicsResult bad = makeKinematics(*s2);
  bad.stamp_ns += 1;  // same id, different source time
  f.sync.submitKinematics(bad);
  f.sync.submitContact(makeContact(*s2));
  EXPECT_TRUE(f.drain().empty());
  EXPECT_EQ(f.sync.stats().identity_mismatches, 1u);
}

// Registration must follow the dispatch order; results for unknown ids are counted.
TEST(Synchronizer, OutOfOrderRegistrationAndUnknownResultsRejected) {
  Fixture f;
  auto s3 = makeSample(3, 6 * kMs);
  EXPECT_FALSE(f.sync.registerSample(s3, 0));  // expected id is 1
  f.add(1);
  auto s1_again = makeSample(1, 2 * kMs);
  EXPECT_FALSE(f.sync.registerSample(s1_again, 0));  // duplicate id
  f.sync.submitKinematics(makeKinematics(*s3));     // never registered
  EXPECT_EQ(f.sync.stats().unknown_results, 1u);
  EXPECT_EQ(f.sync.output().size(), 0u);
}

TEST(Synchronizer, DuplicateResultIsNotUsedTwice) {
  Fixture f;
  auto s = f.add(1);
  f.add(2);
  f.sync.submitKinematics(makeKinematics(*s));
  f.sync.submitKinematics(makeKinematics(*s));
  EXPECT_EQ(f.sync.stats().duplicate_results, 1u);
  f.sync.submitContact(makeContact(*s));
  EXPECT_EQ(f.drain().size(), 1u);
}

TEST(Synchronizer, BoundedPendingCapacity) {
  SynchronizerConfig cfg;
  cfg.capacity = 4;
  Fixture f(cfg);
  for (int id = 1; id <= 4; ++id) f.add(id);
  EXPECT_FALSE(f.sync.hasCapacity());
  EXPECT_FALSE(f.sync.registerSample(makeSample(5, 10 * kMs), 0));
  // Completing the head frees one slot.
  f.sync.submitKinematics(makeKinematics(*f.samples[1]));
  f.sync.submitContact(makeContact(*f.samples[1]));
  EXPECT_TRUE(f.sync.hasCapacity());
  f.add(5);
}

// Estimator slower than the input: the output queue never exceeds its capacity,
// and every discarded bundle is reported with its sample id.
TEST(Synchronizer, OutputOverflowDropOldestIsBoundedAndReported) {
  SynchronizerConfig cfg;
  cfg.capacity = 64;
  cfg.output_capacity = 4;
  cfg.output_overflow = OverflowPolicy::kDropOldest;
  Fixture f(cfg);
  for (int id = 1; id <= 10; ++id) {
    auto s = f.add(id);
    f.sync.submitKinematics(makeKinematics(*s));
    f.sync.submitContact(makeContact(*s));
    EXPECT_LE(f.sync.output().size(), 4u);
  }
  const auto out = f.drain();
  ASSERT_EQ(out.size(), 4u);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(out[i].sensor->sample_id, static_cast<std::uint64_t>(7 + i));
  const auto dropped = f.eventsOf(EventType::kEstimatorBacklog);
  ASSERT_EQ(dropped.size(), 6u);
  for (int i = 0; i < 6; ++i) EXPECT_EQ(dropped[i].sample_id, static_cast<std::uint64_t>(1 + i));
  EXPECT_EQ(f.sync.stats().max_output_depth, 4u);
}

TEST(Synchronizer, OutputOverflowRejectNewest) {
  SynchronizerConfig cfg;
  cfg.output_capacity = 4;
  cfg.output_overflow = OverflowPolicy::kRejectNewest;
  Fixture f(cfg);
  for (int id = 1; id <= 6; ++id) {
    auto s = f.add(id);
    f.sync.submitKinematics(makeKinematics(*s));
    f.sync.submitContact(makeContact(*s));
  }
  const auto out = f.drain();
  ASSERT_EQ(out.size(), 4u);
  EXPECT_EQ(out.front().sensor->sample_id, 1u);
  EXPECT_EQ(out.back().sensor->sample_id, 4u);
  const auto dropped = f.eventsOf(EventType::kEstimatorBacklog);
  ASSERT_EQ(dropped.size(), 2u);
  EXPECT_EQ(dropped[0].sample_id, 5u);
  EXPECT_EQ(dropped[1].sample_id, 6u);
}

TEST(Synchronizer, ResetDiscardsPendingAndKeepsIdsMonotonic) {
  Fixture f;
  auto s1 = f.add(1);
  f.add(2);
  f.sync.reset();
  EXPECT_EQ(f.sync.nextSampleId(), 3u);
  f.sync.submitKinematics(makeKinematics(*s1));  // from before the reset
  EXPECT_EQ(f.sync.stats().late_results, 1u);
  EXPECT_TRUE(f.drain().empty());
}

// The supervisor thread enforces deadlines without any external call.
TEST(Synchronizer, SupervisorExpiresWithoutExternalCalls) {
  std::mutex m;
  std::condition_variable cv;
  std::vector<PipelineEvent> events;
  SynchronizerConfig cfg;
  cfg.worker_timeout_ns = 5 * kMs;
  ExactSampleSynchronizer sync(
      cfg,
      [&](const PipelineEvent& e) {
        std::lock_guard<std::mutex> lock(m);
        events.push_back(e);
        cv.notify_all();
      },
      [] { return steadyNowNs(); });
  sync.startSupervisor();
  auto s = makeSample(1, 0);
  ASSERT_TRUE(sync.registerSample(s, steadyNowNs()));
  sync.submitKinematics(makeKinematics(*s));
  {
    std::unique_lock<std::mutex> lock(m);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&] { return !events.empty(); }));
    EXPECT_EQ(events[0].type, EventType::kMissingContact);
    EXPECT_EQ(events[0].sample_id, 1u);
    EXPECT_GE(events[0].value, 0.005);
  }
  sync.stopSupervisor();
}
