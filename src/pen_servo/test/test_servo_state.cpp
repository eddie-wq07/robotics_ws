#include <cstdlib>
#include <iostream>
#include "pen_servo/servo_state.hpp"
#include "pen_servo/servo_angle.hpp"

#define CHECK(expr) do { if (!(expr)) { std::cerr << __LINE__ << ": " #expr "\n"; return EXIT_FAILURE; } } while (false)

int main() {
  using namespace pen_servo;
  CHECK(!angle_to_pulse_us(110, {}).has_value());
  CHECK(angle_to_pulse_us(0, {500, 2500}).value() == 500);
  CHECK(angle_to_pulse_us(180, {500, 2500}).value() == 2500);
  CHECK(angle_to_pulse_us(110, {500, 2500}).value() == 1722);
  CHECK(angle_to_pulse_us(130, {500, 2500}).value() == 1944);
  CHECK(!angle_to_pulse_us(181, {500, 2500}).has_value());
  CHECK(!angle_to_pulse_us(110, {2500, 500}).has_value());
  CHECK(angle_to_pulse_us(110, {500, 2400}).value() == 1661);
  CHECK(angle_to_pulse_us(130, {500, 2400}).value() == 1872);
  CHECK(angle_to_pulse_us(135, {500, 2400}).value() == 1925);
  CHECK(press_requested_from_gpio35(0));
  CHECK(!press_requested_from_gpio35(1));
  CHECK(do0_for_extrusion(true));
  StateMachine blank({});
  CHECK(blank.update(true, 1000) == State::Unconfigured);
  CHECK(!blank.pressed());
  StateMachine servo({true, 10, 100, 0});
  CHECK(servo.update(true, 0) == State::Disarmed);
  CHECK(servo.update(true, 1000) == State::Disarmed);  // stale-high startup
  servo.update(false, 1001);
  CHECK(servo.update(false, 1011) == State::Ready);
  servo.update(true, 1012);
  servo.update(false, 1015);  // bounce must not press
  CHECK(!servo.pressed());
  servo.update(true, 1016);
  CHECK(servo.update(true, 1026) == State::Pressed);
  CHECK(servo.update(false, 1027) == State::Disarmed);  // immediate release
  servo.update(false, 1037);
  servo.update(true, 1038);
  servo.update(true, 1048);
  CHECK(servo.update(true, 1148) == State::Fault);
  CHECK(servo.fault() == Fault::PressTimeout);
  CHECK(servo.update(true, 1200, true) == State::Fault);
  servo.update(false, 1201);
  CHECK(servo.update(false, 1211) == State::Fault);  // no automatic rearm
  CHECK(servo.update(false, 1212, true) == State::Ready);
  CHECK(servo.update(false, 1000) == State::Fault);
  CHECK(servo.fault() == Fault::ClockReversed);
  StateMachine hb({true, 10, 100, 25});
  hb.heartbeat(0);
  hb.update(false, 0);
  hb.update(false, 10);
  hb.update(true, 11);
  CHECK(hb.update(true, 21) == State::Pressed);
  CHECK(hb.update(true, 26) == State::Fault);
  CHECK(hb.fault() == Fault::HeartbeatLost);
  std::cout << "Servo startup, polarity, debounce, release, timeout, rearm, and heartbeat checks passed.\n";
}
