#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include "pen_execution/stop_on_unwind.hpp"

TEST(StopOnUnwind, StopsBeforeTrajectoryCleanup) {
  std::vector<int> events;
  struct Trajectory {
    std::vector<int>& events;
    ~Trajectory() { events.push_back(2); }
  };
  try {
    Trajectory data{events};
    pen_execution::StopOnUnwind guard([&] { events.push_back(1); });
    throw std::runtime_error("tracking failure");
  } catch (const std::exception&) { events.push_back(3); }
  EXPECT_EQ(events, (std::vector<int>{1, 2, 3}));
}

TEST(StopOnUnwind, DoesNotStopSuccessfulExecution) {
  int stops = 0;
  { pen_execution::StopOnUnwind guard([&] { ++stops; }); }
  EXPECT_EQ(stops, 0);
}

TEST(StopOnUnwind, PreservesOriginalFailureIfStopFails) {
  try {
    pen_execution::StopOnUnwind guard([] { throw std::runtime_error("stop failed"); });
    throw std::runtime_error("tracking failure");
  } catch (const std::exception& error) {
    EXPECT_STREQ(error.what(), "tracking failure");
  }
}
