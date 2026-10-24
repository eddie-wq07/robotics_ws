#include "flexiv_cartesian_controller/target_freshness.hpp"
#include <gtest/gtest.h>

using flexiv_cartesian_controller::target_is_fresh;

TEST(TargetFreshness, AcceptsTargetReceivedAfterCycleStarted) {
  constexpr int64_t cycle_start = 1000000000;
  constexpr int64_t source_stamp = cycle_start + 20000;
  constexpr int64_t receipt_ros = cycle_start + 40000;
  constexpr int64_t receipt_steady = 5000000000;
  constexpr int64_t now_steady = receipt_steady + 10000;
  EXPECT_GT(source_stamp, cycle_start);  // The old comparison rejected this.
  EXPECT_TRUE(target_is_fresh(receipt_ros-source_stamp, receipt_steady, now_steady, 100000000));
}

TEST(TargetFreshness, CountsSourceAgeAndQueueAgeAgainstSameBudget) {
  EXPECT_TRUE(target_is_fresh(80000000, 1000000000, 1020000000, 100000000));
  EXPECT_FALSE(target_is_fresh(80000000, 1000000000, 1020000001, 100000000));
  EXPECT_FALSE(target_is_fresh(100000001, 1000000000, 1000000000, 100000000));
}

TEST(TargetFreshness, RejectsInvalidClockOrdering) {
  EXPECT_FALSE(target_is_fresh(-1, 100, 101, 100000000));
  EXPECT_FALSE(target_is_fresh(0, 100, 99, 100000000));
  EXPECT_FALSE(target_is_fresh(0, -1, 100, 100000000));
}
