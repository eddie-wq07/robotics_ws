#include <gtest/gtest.h>
#include "pen_execution/warmup.hpp"

TEST(Warmup, BriefFirstPressThenReleasedUntilThirtySeconds) {
  using pen_execution::warmup_step;
  EXPECT_TRUE(warmup_step(0).press);
  EXPECT_TRUE(warmup_step(.199).press);
  EXPECT_FALSE(warmup_step(.2).press);
  EXPECT_FALSE(warmup_step(29.999).complete);
  EXPECT_DOUBLE_EQ(warmup_step(15).progress,.5);
  EXPECT_TRUE(warmup_step(30).complete);
  EXPECT_FALSE(warmup_step(30).press);
  EXPECT_DOUBLE_EQ(warmup_step(31).progress,1);
}
