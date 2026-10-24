#pragma once

#include <cstdint>

namespace pen_servo {

enum class State { Unconfigured, Disarmed, Ready, Pressed, Fault };
enum class Fault { None, PressTimeout, HeartbeatLost, ClockReversed };

struct Config {
  bool calibrated{false};
  uint32_t debounce_ms{0};
  uint32_t max_press_ms{0};
  uint32_t heartbeat_timeout_ms{0};  // zero selects DO0-only operation
};

// GPIO35 is electrically inverted; the ROS DO0 command is not.
constexpr bool press_requested_from_gpio35(int level) { return level == 0; }
constexpr bool do0_for_extrusion(bool extrude) { return extrude; }

class StateMachine {
 public:
  explicit StateMachine(Config config) : config_(config) {
    state_ = config.calibrated && config.debounce_ms > 0 && config.max_press_ms > 0
                 ? State::Disarmed : State::Unconfigured;
  }

  void heartbeat(uint64_t now_ms) { heartbeat_at_ = now_ms; has_heartbeat_ = true; }

  State update(bool press_request, uint64_t now_ms, bool explicit_rearm = false) {
    if (state_ == State::Unconfigured) return state_;
    if (initialized_ && now_ms < last_at_) {
      fail(Fault::ClockReversed);
      return state_;
    }
    last_at_ = now_ms;
    if (!initialized_ || press_request != raw_request_) {
      changed_at_ = now_ms;
      raw_request_ = press_request;
      initialized_ = true;
    }
    const bool stable = now_ms - changed_at_ >= config_.debounce_ms;
    const bool heartbeat_ok = config_.heartbeat_timeout_ms == 0 ||
        (has_heartbeat_ && now_ms >= heartbeat_at_ &&
         now_ms - heartbeat_at_ <= config_.heartbeat_timeout_ms);

    if ((state_ == State::Pressed || state_ == State::Ready) && !heartbeat_ok)
      fail(Fault::HeartbeatLost);
    // The press cap is never renewed by a repeated HIGH DO0 / LOW GPIO35.
    if (state_ == State::Pressed && now_ms - pressed_at_ >= config_.max_press_ms)
      fail(Fault::PressTimeout);
    if (state_ == State::Fault) {
      if (explicit_rearm && !press_request && stable && heartbeat_ok) {
        fault_ = Fault::None;
        state_ = State::Ready;
      }
      return state_;
    }

    // Release immediately on an inactive sample, debounce only subsequent arming/press.
    if (!press_request && state_ == State::Pressed) state_ = State::Disarmed;
    if (!press_request && stable && heartbeat_ok) state_ = State::Ready;
    if (press_request && stable && state_ == State::Ready && heartbeat_ok) {
      state_ = State::Pressed;
      pressed_at_ = now_ms;
    }
    return state_;
  }

  bool pressed() const { return state_ == State::Pressed; }
  State state() const { return state_; }
  Fault fault() const { return fault_; }

 private:
  void fail(Fault fault) { state_ = State::Fault; fault_ = fault; }
  Config config_;
  State state_{State::Unconfigured};
  Fault fault_{Fault::None};
  bool initialized_{false}, raw_request_{false}, has_heartbeat_{false};
  uint64_t changed_at_{0}, pressed_at_{0}, last_at_{0}, heartbeat_at_{0};
};

}  // namespace pen_servo
