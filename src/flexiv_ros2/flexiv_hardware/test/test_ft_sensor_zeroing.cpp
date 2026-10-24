#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "flexiv/rdk/data.hpp"
#include "ft_sensor_zeroing.hpp"

namespace {
using flexiv::rdk::Mode;
struct Robot {
    bool ready = true, stationary = true, running = false, disconnected = false;
    bool failed = false, fail_during_poll = false, throw_on_execute = false;
    bool throw_on_stop = false, stay_in_primitive = false;
    int polls = 0, complete_after = 2, stops = 0;
    std::string result_kind = "valid";
    Mode current_mode = Mode::IDLE;
    std::vector<std::string> events;
    bool connected() { return !disconnected; }
    bool operational() { return ready; }
    bool fault() { return failed; }
    bool stopped() { return stationary; }
    bool busy() { return running; }
    Mode mode() { return current_mode; }
    void SwitchMode(Mode mode) {
        EXPECT_EQ(mode, Mode::NRT_PRIMITIVE_EXECUTION);
        current_mode = mode;
        events.push_back("primitive_mode");
    }
    void ExecutePrimitive(const std::string& name,
        const std::map<std::string, flexiv::rdk::FlexivDataTypes>& params) {
        EXPECT_EQ(name, "ZeroFTSensor");
        EXPECT_TRUE(params.empty());
        events.push_back("zero");
        if (throw_on_execute) throw std::runtime_error("execute failed");
    }
    std::map<std::string, flexiv::rdk::FlexivDataTypes> primitive_states() {
        events.push_back("poll");
        if (fail_during_poll) failed = true;
        if (result_kind == "missing") return {};
        if (result_kind == "wrong_type") return {{"terminated", std::string("1")}};
        if (result_kind == "invalid") return {{"terminated", 2}};
        return {{"terminated", ++polls >= complete_after ? 1 : 0}};
    }
    void Stop() {
        ++stops;
        events.push_back("stop");
        if (throw_on_stop) throw std::runtime_error("stop failed");
        if (!stay_in_primitive) current_mode = Mode::IDLE;
    }
};

void run(Robot& robot) {
    flexiv_hardware::zero_ft_sensor(robot, [] { return true; },
        std::chrono::seconds(1), std::chrono::milliseconds(0));
}

TEST(FTSensorZeroing, CompletesBeforeReturningToIdle) {
    Robot robot;
    run(robot);
    EXPECT_EQ(robot.events, (std::vector<std::string>{"primitive_mode", "zero", "poll", "poll", "stop"}));
    EXPECT_EQ(robot.mode(), Mode::IDLE);
}

TEST(FTSensorZeroing, RejectsUnsafeInitialStatesWithoutStartingPrimitive) {
    for (int condition = 0; condition < 5; ++condition) {
        Robot robot;
        if (condition == 0) robot.stationary = false;
        if (condition == 1) robot.running = true;
        if (condition == 2) robot.ready = false;
        if (condition == 3) robot.disconnected = true;
        if (condition == 4) robot.failed = true;
        EXPECT_THROW(run(robot), std::runtime_error);
        EXPECT_EQ(robot.events, (std::vector<std::string>{"stop"}));
    }
}

TEST(FTSensorZeroing, StopsOnFaultDuringPolling) {
    Robot robot;
    robot.fail_during_poll = true;
    EXPECT_THROW(run(robot), std::runtime_error);
    EXPECT_EQ(robot.stops, 1);
}

TEST(FTSensorZeroing, StopsWhenShutdownRequested) {
    Robot robot;
    EXPECT_THROW(flexiv_hardware::zero_ft_sensor(robot,
        [&] { return robot.polls == 0; }), std::runtime_error);
    EXPECT_EQ(robot.stops, 1);
}

TEST(FTSensorZeroing, StopsOnCompletionTimeout) {
    Robot robot;
    robot.complete_after = 1000000;
    EXPECT_THROW(flexiv_hardware::zero_ft_sensor(robot, [] { return true; },
        std::chrono::milliseconds(5), std::chrono::milliseconds(1)), std::runtime_error);
    EXPECT_GT(robot.polls, 0);
    EXPECT_EQ(robot.stops, 1);
}

TEST(FTSensorZeroing, RejectsMalformedPrimitiveReplies) {
    for (const auto& kind : {"missing", "wrong_type", "invalid"}) {
        Robot robot;
        robot.result_kind = kind;
        EXPECT_THROW(run(robot), std::runtime_error);
        EXPECT_EQ(robot.stops, 1);
    }
}

TEST(FTSensorZeroing, PreservesFailureWhenCleanupAlsoFails) {
    Robot robot;
    robot.throw_on_execute = robot.throw_on_stop = true;
    try {
        run(robot);
        FAIL() << "Expected failure";
    } catch (const std::runtime_error& error) {
        EXPECT_STREQ(error.what(), "execute failed");
    }
}

TEST(FTSensorZeroing, RequiresReturnToIdle) {
    Robot robot;
    robot.stay_in_primitive = true;
    EXPECT_THROW(run(robot), std::runtime_error);
}
}  // namespace
