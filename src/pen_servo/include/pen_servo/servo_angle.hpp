#pragma once

#include <cstdint>
#include <optional>

namespace pen_servo {

// The commanded unit is degrees. PWM endpoints must match the actual servo
// or its previously calibrated Servo.attach(min_us, max_us) configuration.
struct PulseRange {
  uint32_t at_zero_deg_us{0};
  uint32_t at_180_deg_us{0};
};

constexpr std::optional<uint32_t> angle_to_pulse_us(uint32_t degrees, PulseRange range) {
  if (degrees > 180 || range.at_zero_deg_us < 400 ||
      range.at_180_deg_us > 3000 || range.at_180_deg_us <= range.at_zero_deg_us) {
    return std::nullopt;
  }
  return range.at_zero_deg_us +
         ((range.at_180_deg_us-range.at_zero_deg_us)*degrees+90U)/180U;
}

}  // namespace pen_servo
