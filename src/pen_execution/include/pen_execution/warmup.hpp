#pragma once

#include <algorithm>

namespace pen_execution {
// Operator-selected cold-start delay, not a measured temperature guarantee.
constexpr double kWarmupSeconds = 30.0;
constexpr double kWarmupPressSeconds = 0.2;
struct WarmupStep {
  bool press;
  bool complete;
  double progress;
};
inline WarmupStep warmup_step(double elapsed) {
  return {elapsed < kWarmupPressSeconds, elapsed >= kWarmupSeconds,
          std::clamp(elapsed / kWarmupSeconds, 0.0, 1.0)};
}
}  // namespace pen_execution
