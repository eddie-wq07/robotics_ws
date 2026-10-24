#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include <gtest/gtest.h>
#include "gpio_controller/gpio_controller.hpp"

TEST(GPIOController, PreservesUncommandedOutputsAndSupportsReactivation)
{
    rclcpp::init(0, nullptr);
    {
        gpio_controller::GPIOController controller;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("robot_sn", "Rizon4s-000000")});
        ASSERT_EQ(controller.init("gpio_test", "", options), controller_interface::return_type::OK);
        ASSERT_EQ(controller.on_configure(controller.get_state()), controller_interface::CallbackReturn::SUCCESS);
        std::array<double, 18> commands{}, states{};
        std::vector<hardware_interface::CommandInterface> command_handles;
        std::vector<hardware_interface::StateInterface> state_handles;
        command_handles.reserve(18);
        state_handles.reserve(18);
        for (size_t i = 0; i < 18; ++i) {
            command_handles.emplace_back("Rizon4s-000000_gpio", "digital_output_" + std::to_string(i), &commands[i]);
            state_handles.emplace_back("Rizon4s-000000_gpio", "digital_input_" + std::to_string(i), &states[i]);
        }
        std::vector<hardware_interface::LoanedCommandInterface> command_loans;
        std::vector<hardware_interface::LoanedStateInterface> state_loans;
        for (size_t i = 0; i < 18; ++i) {
            command_loans.emplace_back(command_handles[i]);
            state_loans.emplace_back(state_handles[i]);
        }
        controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
        auto node = std::make_shared<rclcpp::Node>("gpio_test_client");
        auto publisher = node->create_publisher<flexiv_msgs::msg::GPIOStates>(
            "/Rizon4s_000000/gpio_outputs", 10);
        size_t received = 0;
        auto subscriber = node->create_subscription<flexiv_msgs::msg::GPIOStates>(
            "/Rizon4s_000000/gpio_inputs", 10, [&received](flexiv_msgs::msg::GPIOStates::ConstSharedPtr msg) {
                if (msg->states.size() == 18) ++received;
            });
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(controller.get_node()->get_node_base_interface());
        auto pump = [&]() {
            for (int i = 0; i < 100; ++i) {
                executor.spin_some();
                EXPECT_EQ(controller.update(node->now(), rclcpp::Duration::from_seconds(.001)),
                    controller_interface::return_type::OK);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        ASSERT_EQ(controller.on_activate(controller.get_state()), controller_interface::CallbackReturn::SUCCESS);
        pump();
        for (double value : commands) EXPECT_TRUE(std::isnan(value));
        flexiv_msgs::msg::GPIOStates message;
        flexiv_msgs::msg::Digital pin;
        pin.pin = 0; pin.state = true; message.states.push_back(pin);
        pin.pin = 17; pin.state = false; message.states.push_back(pin);
        pin.pin = 255; pin.state = true; message.states.push_back(pin);
        publisher->publish(message);
        pump();
        EXPECT_EQ(commands[0], 1.0);
        EXPECT_EQ(commands[17], 0.0);
        for (size_t i = 1; i < 17; ++i) EXPECT_TRUE(std::isnan(commands[i]));
        ASSERT_GT(received, 0u);
        ASSERT_EQ(controller.on_deactivate(controller.get_state()), controller_interface::CallbackReturn::SUCCESS);
        publisher->publish(message);
        pump();
        ASSERT_EQ(controller.on_activate(controller.get_state()), controller_interface::CallbackReturn::SUCCESS);
        size_t before = received;
        pump();
        EXPECT_GT(received, before);
        for (double value : commands) EXPECT_TRUE(std::isnan(value));
        publisher->publish(message);
        pump();
        EXPECT_EQ(commands[0], 1.0);
        EXPECT_EQ(controller.on_deactivate(controller.get_state()), controller_interface::CallbackReturn::SUCCESS);
        controller.release_interfaces();
    }
    rclcpp::shutdown();
}
