#include "seb_online/bounded_queue.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using seb::BoundedQueue;
using seb::OverflowPolicy;

TEST(BoundedQueue, FifoAndCapacity) {
  BoundedQueue<int> q(3);
  EXPECT_TRUE(q.tryPush(1));
  EXPECT_TRUE(q.tryPush(2));
  EXPECT_TRUE(q.tryPush(3));
  EXPECT_FALSE(q.tryPush(4));
  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(*q.pop(), 1);
  EXPECT_EQ(*q.pop(), 2);
  EXPECT_TRUE(q.tryPush(5));
  EXPECT_EQ(*q.pop(), 3);
  EXPECT_EQ(*q.pop(), 5);
  EXPECT_FALSE(q.tryPop().has_value());
}

TEST(BoundedQueue, OverflowPolicies) {
  BoundedQueue<int> q(2);
  std::optional<int> discarded;
  q.pushWithPolicy(1, OverflowPolicy::kDropOldest, &discarded);
  q.pushWithPolicy(2, OverflowPolicy::kDropOldest, &discarded);
  EXPECT_FALSE(discarded.has_value());
  EXPECT_TRUE(q.pushWithPolicy(3, OverflowPolicy::kDropOldest, &discarded));
  ASSERT_TRUE(discarded.has_value());
  EXPECT_EQ(*discarded, 1);
  discarded.reset();
  EXPECT_FALSE(q.pushWithPolicy(4, OverflowPolicy::kRejectNewest, &discarded));
  EXPECT_EQ(*discarded, 4);
  EXPECT_EQ(*q.pop(), 2);
  EXPECT_EQ(*q.pop(), 3);
}

TEST(BoundedQueue, CloseWakesBlockedConsumer) {
  BoundedQueue<int> q(2);
  std::thread consumer([&] { EXPECT_FALSE(q.pop().has_value()); });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  q.close();
  consumer.join();
  EXPECT_FALSE(q.tryPush(1));
}

TEST(BoundedQueue, BlockingPopReceivesItemsInOrderAcrossThreads) {
  BoundedQueue<int> q(8);
  std::vector<int> got;
  std::thread consumer([&] {
    while (auto v = q.pop()) got.push_back(*v);
  });
  for (int i = 0; i < 1000; ++i) {
    while (!q.tryPush(i)) std::this_thread::yield();
  }
  q.close();
  consumer.join();
  ASSERT_EQ(got.size(), 1000u);
  for (int i = 0; i < 1000; ++i) EXPECT_EQ(got[i], i);
}
