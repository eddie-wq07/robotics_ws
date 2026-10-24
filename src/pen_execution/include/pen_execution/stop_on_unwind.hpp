#pragma once

#include <exception>
#include <utility>

namespace pen_execution {
// Construct after command/trajectory storage so the stop request runs before
// expensive trajectory destruction when an execution exception unwinds.
template<class Stop>
class StopOnUnwind {
 public:
  explicit StopOnUnwind(Stop stop) : stop_(std::move(stop)), depth_(std::uncaught_exceptions()) {}
  StopOnUnwind(const StopOnUnwind&) = delete;
  StopOnUnwind& operator=(const StopOnUnwind&) = delete;
  ~StopOnUnwind() noexcept {
    if (std::uncaught_exceptions() > depth_) {
      try { stop_(); } catch (...) {}  // Never mask the original execution failure.
    }
  }
 private:
  Stop stop_;
  int depth_;
};
}  // namespace pen_execution
