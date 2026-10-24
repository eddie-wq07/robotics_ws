#include <cstdio>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "pen_servo/servo_state.hpp"
#include "pen_servo/servo_angle.hpp"

namespace {
constexpr char kTag[] = "pen_servo";
constexpr gpio_num_t kRequest = GPIO_NUM_35;
constexpr gpio_num_t kServo = GPIO_NUM_18;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr uint32_t kResolution = 1U << 16;
constexpr pen_servo::PulseRange kPulseRange{CONFIG_PEN_PWM_MIN_US, CONFIG_PEN_PWM_MAX_US};
constexpr auto kReleasePulse = pen_servo::angle_to_pulse_us(CONFIG_PEN_RELEASE_DEG, kPulseRange);
constexpr auto kPressPulse = pen_servo::angle_to_pulse_us(CONFIG_PEN_PRESS_DEG, kPulseRange);

uint32_t duty(uint32_t pulse_us) {
  return static_cast<uint32_t>(static_cast<uint64_t>(pulse_us) *
      CONFIG_PEN_PWM_HZ * kResolution / 1000000U);
}

void output(bool pressed) {
  ESP_ERROR_CHECK(ledc_set_duty(kMode, kChannel,
      duty(pressed ? kPressPulse.value() : kReleasePulse.value())));
  ESP_ERROR_CHECK(ledc_update_duty(kMode, kChannel));
}
}  // namespace

extern "C" void app_main() {
  ESP_LOGI(kTag, "ESP32 DevKit V1; reset reason %d", static_cast<int>(esp_reset_reason()));
  gpio_config_t input{};
  input.pin_bit_mask = 1ULL << kRequest;
  input.mode = GPIO_MODE_INPUT;
  input.pull_up_en = GPIO_PULLUP_DISABLE;  // GPIO35 has no internal pull resistors.
  input.pull_down_en = GPIO_PULLDOWN_DISABLE;
  input.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&input));

  bool configured = false;
#ifdef CONFIG_PEN_CALIBRATED
  configured = CONFIG_PEN_PWM_HZ > 0 && kReleasePulse.has_value() &&
      kPressPulse.has_value() && CONFIG_PEN_DEBOUNCE_MS > 0 &&
      CONFIG_PEN_MAX_PRESS_MS > 0 && CONFIG_PEN_PRESS_DEG != CONFIG_PEN_RELEASE_DEG &&
      kPressPulse.value() * CONFIG_PEN_PWM_HZ < 1000000 &&
      kReleasePulse.value() * CONFIG_PEN_PWM_HZ < 1000000;
#endif
  if (!configured) {
    // Without calibrated release pulses, no software can claim mechanical release.
    // Leave the signal unpowered and never guess a servo position.
    ESP_ERROR_CHECK(gpio_set_direction(kServo, GPIO_MODE_INPUT));
    ESP_LOGE(kTag, "UNCONFIGURED: measured settings TBD; PWM disabled, no press possible. "
             "Physical button release must be checked separately.");
    return;
  }

  ledc_timer_config_t timer{};
  timer.speed_mode = kMode;
  timer.duty_resolution = LEDC_TIMER_16_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = CONFIG_PEN_PWM_HZ;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  ledc_channel_config_t channel{};
  channel.gpio_num = kServo;
  channel.speed_mode = kMode;
  channel.channel = kChannel;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = duty(kReleasePulse.value());
  channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));
  ESP_LOGI(kTag, "Angle commands: RELEASE=%d deg, PRESS=%d deg",
           CONFIG_PEN_RELEASE_DEG, CONFIG_PEN_PRESS_DEG);

  pen_servo::StateMachine state({true, CONFIG_PEN_DEBOUNCE_MS, CONFIG_PEN_MAX_PRESS_MS, 0});
  auto previous = state.state();
  const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  ESP_LOGI(kTag, "Released/disarmed; observe stable GPIO35 HIGH to arm. "
           "After a fault send R with input HIGH to rearm.");
  for (;;) {
    char command = 0;
    const bool rearm = read(STDIN_FILENO, &command, 1) == 1 && command == 'R';
    const int level = gpio_get_level(kRequest);
    state.update(pen_servo::press_requested_from_gpio35(level), esp_timer_get_time() / 1000, rearm);
    if (state.state() != previous) {
      output(state.pressed());
      ESP_LOGI(kTag, "input=%d requested=%d state=%d servo_command=%s fault=%d", level,
               pen_servo::press_requested_from_gpio35(level), static_cast<int>(state.state()),
               state.pressed() ? "PRESS" : "RELEASE", static_cast<int>(state.fault()));
      previous = state.state();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
