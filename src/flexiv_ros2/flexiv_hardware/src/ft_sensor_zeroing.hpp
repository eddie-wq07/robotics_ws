#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>
#include <variant>

#include "flexiv/rdk/mode.hpp"

namespace flexiv_hardware {

// Startup only: never call from read/write, update, or controller switching.
// The caller must ensure the mounted tool is free of external contact.
template <class Robot, class KeepRunning>
void zero_ft_sensor(Robot& robot, KeepRunning keep_running,
    std::chrono::milliseconds timeout = std::chrono::seconds(10),
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds(20))
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const auto check_ready = [&]() {
        if (!keep_running() || !robot.connected() || !robot.operational() || robot.fault()) {
            throw std::runtime_error("F/T sensor zeroing interrupted: robot is not ready or shutdown requested");
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("F/T sensor zeroing exceeded its completion deadline");
        }
    };
    try {
        check_ready();
        if (!robot.stopped() || robot.busy()) {
            throw std::runtime_error("F/T sensor zeroing requires a stationary, idle robot");
        }
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        check_ready();
        robot.ExecutePrimitive("ZeroFTSensor", {});
        for (;;) {
            check_ready();
            const auto states = robot.primitive_states();
            // SDK calls can block: check the deadline again after each reply.
            check_ready();
            const auto state = states.find("terminated");
            if (state == states.end()) {
                throw std::runtime_error("ZeroFTSensor returned no terminated state");
            }
            const auto terminated = std::get_if<int>(&state->second);
            if (!terminated || (*terminated != 0 && *terminated != 1)) {
                throw std::runtime_error("ZeroFTSensor returned an invalid terminated state");
            }
            if (*terminated == 1) break;
            std::this_thread::sleep_for(poll_interval);
        }
        robot.Stop();
        check_ready();
        if (robot.mode() != flexiv::rdk::Mode::IDLE || !robot.stopped()) {
            throw std::runtime_error("Robot did not return to stopped IDLE after F/T sensor zeroing");
        }
    } catch (...) {
        // Best effort only; preserve the original failure and never clear faults or retry.
        try { robot.Stop(); } catch (...) {}
        throw;
    }
}

}  // namespace flexiv_hardware
