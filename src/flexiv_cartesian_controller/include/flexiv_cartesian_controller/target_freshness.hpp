// Copyright 2026 Industrial Next. All rights reserved.
#pragma once

#include <cstdint>

namespace flexiv_cartesian_controller {
// Source age was validated using the ROS clock at receipt. Account for the
// remaining queue time using one monotonic clock, never the cycle-start time.
inline bool target_is_fresh(
    int64_t source_age_ns, int64_t receipt_ns, int64_t now_ns, int64_t timeout_ns) {
  return timeout_ns > 0 && source_age_ns >= 0 && source_age_ns <= timeout_ns && receipt_ns >= 0 &&
         now_ns >= receipt_ns && now_ns - receipt_ns <= timeout_ns - source_age_ns;
}
}  // namespace flexiv_cartesian_controller
