/**
 * @file gpio_controller.cpp
 * @brief GPIO controller as ROS 2 controller. Adapted from
 * ros2_control_demos/example_10/gpio_controller
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include "gpio_controller/gpio_controller.hpp"

#include <string>
#include <limits>

namespace gpio_controller {

GPIOController::GPIOController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn GPIOController::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
        initMsgs();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    std::string robot_sn = params_.robot_sn;
    for (size_t i = 0; i < kIOPorts; ++i) {
        config.names.emplace_back(robot_sn + "_gpio/digital_output_" + std::to_string(i));
    }

    return config;
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    std::string robot_sn = params_.robot_sn;
    for (size_t i = 0; i < kIOPorts; ++i) {
        config.names.emplace_back(robot_sn + "_gpio/digital_input_" + std::to_string(i));
    }

    return config;
}

controller_interface::return_type GPIOController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!active_.load()) return controller_interface::return_type::OK;
    // Publish without blocking the controller-manager's shared update thread.
    if (realtime_inputs_publisher_->trylock()) {
        for (size_t i = 0; i < kIOPorts; ++i) {
            realtime_inputs_publisher_->msg_.states[i].pin = i;
            realtime_inputs_publisher_->msg_.states[i].state = state_interfaces_[i].get_value() == 1.0;
        }
        realtime_inputs_publisher_->unlockAndPublish();
    }

    const auto outputs = *outputs_buffer_.readFromRT();
    if (outputs.generation == generation_.load()) {
        for (size_t i = 0; i < command_interfaces_.size(); ++i) {
            command_interfaces_[i].set_value(outputs.values[i]);
        }
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GPIOController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    std::string robot_sn = params_.robot_sn;
    if (robot_sn.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'robot_sn' parameter has to be specified.");
        return CallbackReturn::ERROR;
    } else {
        // Replace "-" with "_" in robot_sn to match the topic name
        std::replace(robot_sn.begin(), robot_sn.end(), '-', '_');
    }

    try {
        // register publisher
        gpio_inputs_publisher_ = get_node()->create_publisher<CmdType>(
            "/" + robot_sn + kGPIOInputsTopic, rclcpp::SystemDefaultsQoS());
        realtime_inputs_publisher_ =
            std::make_unique<realtime_tools::RealtimePublisher<CmdType>>(gpio_inputs_publisher_);
        realtime_inputs_publisher_->msg_.states.resize(kIOPorts);

        // register subscriber
        gpio_outputs_command_
            = get_node()->create_subscription<CmdType>("/" + robot_sn + kGPIOOutputsTopic,
                rclcpp::SystemDefaultsQoS(), [this](const CmdType::SharedPtr msg) {
                    if (!active_.load()) return;
                    const auto generation = generation_.load();
                    if (pending_outputs_.generation != generation) {
                        pending_outputs_.values.fill(std::numeric_limits<double>::quiet_NaN());
                        pending_outputs_.generation = generation;
                    }
                    for (size_t i = 0; i < msg->states.size(); ++i) {
                        if (msg->states[i].pin >= kIOPorts) {
                            RCLCPP_WARN(get_node()->get_logger(),
                                "Received command for pin %d, but only pins 0-17 are supported.",
                                msg->states[i].pin);
                            continue;
                        } else {
                            pending_outputs_.values[msg->states[i].pin]
                                = static_cast<double>(msg->states[i].state);
                        }
                    }
                    outputs_buffer_.writeFromNonRT(pending_outputs_);
                });
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::initMsgs()
{
    pending_outputs_.values.fill(std::numeric_limits<double>::quiet_NaN());
    outputs_buffer_.initRT(pending_outputs_);
}

controller_interface::CallbackReturn GPIOController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (command_interfaces_.size() != kIOPorts || state_interfaces_.size() != kIOPorts) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    generation_.fetch_add(1);
    for (auto& command : command_interfaces_) {
        command.set_value(std::numeric_limits<double>::quiet_NaN());
    }
    active_.store(true);
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    active_.store(false);
    generation_.fetch_add(1);
    for (auto& command : command_interfaces_) {
        command.set_value(std::numeric_limits<double>::quiet_NaN());
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace gpio_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gpio_controller::GPIOController, controller_interface::ControllerInterface)
