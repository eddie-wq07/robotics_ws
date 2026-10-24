/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "ft_sensor_zeroing.hpp"

namespace {

constexpr std::array<const char*, 7> kPoseNames = {"pose_x", "pose_y", "pose_z",
    "pose_qw", "pose_qx", "pose_qy", "pose_qz"};

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

}

namespace flexiv_hardware {

hardware_interface::CallbackReturn FlexivHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info)
        != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    target_position_.resize(info_.joints.size());
    target_velocity_.resize(info_.joints.size());
    max_velocity_.assign(info_.joints.size(), kMaxJointVelocity);
    max_acceleration_.assign(info_.joints.size(), kMaxJointAcceleration);
    cartesian_state_.fill(std::numeric_limits<double>::quiet_NaN());
    cartesian_command_ = cartesian_state_;

    if (info_.joints.size() < 7) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected at least 7.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Get prefix for joint mapping
    std::string prefix;
    try {
        prefix = info_.hardware_parameters.at("prefix");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'prefix' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    cartesian_resource_ = prefix + "tcp";
    for (const auto& gpio : info_.gpios) {
        if (gpio.name == cartesian_resource_) {
            has_cartesian_interfaces_ = true;
        }
    }

    // Build RDK to ROS joint mapping
    std::vector<size_t> arm_indices;
    std::vector<size_t> ext_indices;

    // Find 7 arm joints in standard order
    for (int j = 1; j <= 7; ++j) {
        std::string arm_joint_name = prefix + "joint" + std::to_string(j);
        bool found = false;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (info_.joints[i].name == arm_joint_name) {
                arm_indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_FATAL(getLogger(), "Could not find arm joint '%s'", arm_joint_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Find external axis joints (any joint that is not an arm joint)
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        bool is_arm = false;
        for (size_t arm_idx : arm_indices) {
            if (i == arm_idx) {
                is_arm = true;
                break;
            }
        }
        if (!is_arm) {
            ext_indices.push_back(i);
        }
    }

    // Construct map: external joints first, then arm joints (RDK order)
    rdk_to_ros_map_.clear();
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), ext_indices.begin(), ext_indices.end());
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), arm_indices.begin(), arm_indices.end());

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld state interfaces found. 3 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    std::string robot_sn;
    try {
        robot_sn = info_.hardware_parameters.at("robot_sn");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
        } else {
            RCLCPP_FATAL(getLogger(),
                "Parameter 'rdk_control_mode' has invalid value '%s'. Options: joint_position, "
                "joint_impedance",
                rdk_control_mode_str.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'rdk_control_mode' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::StateInterface> FlexivHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    if (has_cartesian_interfaces_) {
        for (size_t i = 0; i < kPoseNames.size(); ++i) {
            state_interfaces.emplace_back(cartesian_resource_, kPoseNames[i], &cartesian_state_[i]);
        }
        state_interfaces.emplace_back(cartesian_resource_, "streaming", &cartesian_streaming_);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "gpio", "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    if (has_cartesian_interfaces_) {
        for (size_t i = 0; i < kPoseNames.size(); ++i) {
            command_interfaces.emplace_back(cartesian_resource_, kPoseNames[i], &cartesian_command_[i]);
        }
        command_interfaces.emplace_back(cartesian_resource_, "enabled", &cartesian_enabled_);
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
        if (robot_->operational_status() == flexiv::rdk::OperationalStatus::IN_MANUAL_MODE
            || robot_->operational_status() == flexiv::rdk::OperationalStatus::IN_AUTO_MODE) {
            RCLCPP_ERROR(getLogger(),
                "Robot is not in AUTO (REMOTE). Select AUTO (REMOTE) in Flexiv Elements, then restart bringup.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!robot_->estop_released()) {
            RCLCPP_ERROR(getLogger(), "Robot E-stop is pressed; release it when safe, then restart bringup.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Clear fault on robot server if any
        if (robot_->fault()) {
            RCLCPP_WARN(getLogger(), "Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            if (!robot_->ClearFault()) {
                RCLCPP_FATAL(getLogger(), "Fault cannot be cleared, exiting ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(getLogger(), "Fault on robot server is cleared");
        }

        // Check the DoF of the robot
        if (robot_->info().DoF != info_.joints.size()) {
            RCLCPP_FATAL(getLogger(), "Robot has %ld DoF. Expected %ld (from URDF).",
                robot_->info().DoF, info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Bound the readiness wait so startup failure does not hide ROS services indefinitely.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (!robot_->operational()) {
            if (!rclcpp::ok() || !robot_->connected() || robot_->fault()
                || !robot_->estop_released()
                || robot_->operational_status() == flexiv::rdk::OperationalStatus::IN_MANUAL_MODE
                || robot_->operational_status() == flexiv::rdk::OperationalStatus::IN_AUTO_MODE
                || std::chrono::steady_clock::now() >= deadline) {
                RCLCPP_ERROR(getLogger(),
                    "Robot did not become ready: status=%d, connected=%d, fault=%d, "
                    "estop_released=%d, reduced=%d, recovery=%d. Check Flexiv Elements.",
                    static_cast<int>(robot_->operational_status()), robot_->connected(),
                    robot_->fault(), robot_->estop_released(), robot_->reduced(), robot_->recovery());
                robot_->Stop();
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");

        // Unlock external axes if any
        if (robot_->info().DoF_e > 0) {
            robot_->LockExternalAxes(false);
        }

        RCLCPP_WARN(getLogger(),
            "Zeroing F/T sensor before controller startup. The robot and mounted tool "
            "must be stationary and free of external contact.");
        zero_ft_sensor(*robot_, [] { return rclcpp::ok(); });
        if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
            throw std::runtime_error("Could not refresh measured state after F/T sensor zeroing");
        }
        RCLCPP_INFO(getLogger(), "F/T sensor zeroing complete; robot is stopped in IDLE");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Robot startup failed; controllers must not start.");
        RCLCPP_FATAL(getLogger(), "%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    active_mode_ = CommandMode::None;
    cartesian_enabled_ = cartesian_streaming_ = 0.0;
    claimed_arm_interfaces_.clear();
    try {
        robot_->Stop();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "SDK stop during deactivation failed: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
try
{
    if (robot_->operational()) {
        // One SDK snapshot per cycle, rather than copying its vectors for every joint.
        hw_flexiv_robot_states_ = robot_->states();
        const auto count = rdk_to_ros_map_.size();
        if (hw_flexiv_robot_states_.q.size() != count
            || hw_flexiv_robot_states_.dtheta.size() != count
            || hw_flexiv_robot_states_.tau.size() != count) {
            throw std::runtime_error("SDK joint state dimensions do not match the hardware description");
        }
        for (size_t rdk_idx = 0; rdk_idx < rdk_to_ros_map_.size(); ++rdk_idx) {
            const size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            hw_states_joint_positions_[ros_idx] = hw_flexiv_robot_states_.q[rdk_idx];
            hw_states_joint_velocities_[ros_idx] = hw_flexiv_robot_states_.dtheta[rdk_idx];
            hw_states_joint_efforts_[ros_idx] = hw_flexiv_robot_states_.tau[rdk_idx];
        }
        cartesian_state_ = hw_flexiv_robot_states_.tcp_pose;
        const auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); ++i) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }
    }

    return hardware_interface::return_type::OK;
}
catch (const std::exception& e)
{
    RCLCPP_ERROR(getLogger(), "SDK read failed: %s", e.what());
    active_mode_ = CommandMode::None;
    stop_cartesian("SDK read failure");
    return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
try
{
    if (active_mode_ != CommandMode::None
        && (!robot_->connected() || !robot_->operational() || robot_->fault())) {
        active_mode_ = CommandMode::None;
        stop_cartesian("robot is no longer ready for commands");
        return hardware_interface::return_type::ERROR;
    }
    if (active_mode_ == CommandMode::Cartesian) {
        if (cartesian_stop_latched_) {
            return hardware_interface::return_type::OK;
        }
        if (cartesian_enabled_ != 1.0) {
            return stop_cartesian("controller disabled or target expired");
        }
        if (!robot_->connected() || !robot_->operational() || robot_->fault()
            || robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            return stop_cartesian("robot is no longer ready in Cartesian RT mode");
        }
        double norm = 0.0;
        for (size_t i = 0; i < cartesian_command_.size(); ++i) {
            if (!std::isfinite(cartesian_command_[i])) {
                return stop_cartesian("non-finite Cartesian command");
            }
            if (i >= 3) norm += cartesian_command_[i] * cartesian_command_[i];
        }
        if (std::abs(norm - 1.0) > 1e-3) {
            return stop_cartesian("invalid Cartesian quaternion");
        }
        try {
            robot_->StreamCartesianMotionForce(cartesian_command_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(getLogger(), "Cartesian stream failed: %s", e.what());
            return stop_cartesian("SDK streaming error");
        }
    }

    const auto finite = [](const auto& values) {
        return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
    };
    if (active_mode_ == CommandMode::Position && robot_->mode() == rdk_control_mode_
        && finite(hw_commands_joint_positions_)) {
        for (size_t i = 0; i < rdk_to_ros_map_.size(); ++i) {
            target_position_[i] = hw_commands_joint_positions_[rdk_to_ros_map_[i]];
        }
        std::fill(target_velocity_.begin(), target_velocity_.end(), 0.0);
        robot_->SendJointPosition(target_position_, target_velocity_, max_velocity_, max_acceleration_);
    } else if (active_mode_ == CommandMode::Velocity && robot_->mode() == rdk_control_mode_
        && finite(hw_commands_joint_velocities_)) {
        for (size_t i = 0; i < rdk_to_ros_map_.size(); ++i) {
            target_position_[i] = hw_states_joint_positions_[rdk_to_ros_map_[i]];
            target_velocity_[i] = hw_commands_joint_velocities_[rdk_to_ros_map_[i]];
        }
        robot_->SendJointPosition(target_position_, target_velocity_, max_velocity_, max_acceleration_);
    } else if (active_mode_ == CommandMode::Torque
        && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE
        && finite(hw_commands_joint_efforts_)) {
        for (size_t i = 0; i < rdk_to_ros_map_.size(); ++i) {
            target_position_[i] = hw_commands_joint_efforts_[rdk_to_ros_map_[i]];
        }
        robot_->StreamJointTorque(target_position_, true, true);
    }

    // Avoid constructing maps in healthy cycles. GPIO changes retain the existing
    // blocking SDK behavior and should not be commanded during a qualified RT run.
    bool digital_outputs_changed = false;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); ++i) {
        const double value = hw_commands_gpio_out_[i];
        if (!std::isfinite(value)) continue;
        const auto old = current_digital_outputs_.find(static_cast<unsigned int>(i));
        if (old == current_digital_outputs_.end() || old->second != static_cast<bool>(value)) {
            digital_outputs_changed = true;
            break;
        }
    }
    if (digital_outputs_changed) {
        std::map<unsigned int, bool> digital_outputs;
        for (size_t i = 0; i < hw_commands_gpio_out_.size(); ++i) {
            if (std::isfinite(hw_commands_gpio_out_[i]))
                digital_outputs[static_cast<unsigned int>(i)] = static_cast<bool>(hw_commands_gpio_out_[i]);
        }
        robot_->SetDigitalOutputs(digital_outputs);
        current_digital_outputs_ = std::move(digital_outputs);
    }

    return hardware_interface::return_type::OK;
}
catch (const std::exception& e)
{
    RCLCPP_ERROR(getLogger(), "SDK write failed: %s", e.what());
    active_mode_ = CommandMode::None;
    stop_cartesian("SDK write failure");
    return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type FlexivHardwareInterface::stop_cartesian(const char* reason)
{
    // Fault/stop path only. Stop() blocks; it is never called in healthy streaming.
    cartesian_stop_latched_ = true;
    cartesian_enabled_ = cartesian_streaming_ = 0.0;
    RCLCPP_WARN(getLogger(), "Stopping arm command stream: %s", reason);
    try {
        robot_->Stop();
        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "SDK stop failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    std::set<std::string> arm_interfaces;
    for (const auto& joint : info_.joints) {
        for (const auto* name : {"position", "velocity", "effort"}) {
            arm_interfaces.insert(joint.name + "/" + name);
        }
    }
    if (has_cartesian_interfaces_) {
        for (const auto* name : kPoseNames) arm_interfaces.insert(cartesian_resource_ + "/" + name);
        arm_interfaces.insert(cartesian_resource_ + "/enabled");
    }
    auto requested = claimed_arm_interfaces_;
    for (const auto& name : stop_interfaces) {
        if (arm_interfaces.count(name)) requested.erase(name);
    }
    for (const auto& name : start_interfaces) {
        if (arm_interfaces.count(name)) requested.insert(name);
    }
    std::set<std::string> position, velocity, torque, cartesian;
    for (const auto& joint : info_.joints) {
        position.insert(joint.name + "/position");
        velocity.insert(joint.name + "/velocity");
        torque.insert(joint.name + "/effort");
    }
    cartesian = position;  // Joint position resources also exclude the joint controller.
    for (const auto* name : kPoseNames) cartesian.insert(cartesian_resource_ + "/" + name);
    cartesian.insert(cartesian_resource_ + "/enabled");
    CommandMode mode;
    if (requested.empty()) mode = CommandMode::None;
    else if (requested == position) mode = CommandMode::Position;
    else if (requested == velocity) mode = CommandMode::Velocity;
    else if (requested == torque) mode = CommandMode::Torque;
    else if (has_cartesian_interfaces_ && requested == cartesian) mode = CommandMode::Cartesian;
    else {
        RCLCPP_ERROR(getLogger(), "Rejecting partial or conflicting arm command interfaces");
        return hardware_interface::return_type::ERROR;
    }
    if (mode == CommandMode::Cartesian && info_.joints.size() != 7) {
        RCLCPP_ERROR(getLogger(), "Cartesian controller supports one seven-joint arm only");
        return hardware_interface::return_type::ERROR;
    }
    pending_mode_ = mode;
    pending_arm_interfaces_ = std::move(requested);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (pending_mode_ == active_mode_ && pending_arm_interfaces_ == claimed_arm_interfaces_) {
        return hardware_interface::return_type::OK;
    }
    const auto previous_mode = active_mode_;
    active_mode_ = CommandMode::None;
    cartesian_enabled_ = cartesian_streaming_ = 0.0;
    try {
        if (previous_mode != CommandMode::None) robot_->Stop();
        if (pending_mode_ == CommandMode::Cartesian) {
            // Setters are legal in NRT Cartesian mode; configure before entering RT.
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
            robot_->SetForceControlAxis({false, false, false, false, false, false});
            robot_->SetNullSpacePosture(robot_->states().q);
            robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
            const auto state = robot_->states();
            cartesian_state_ = state.tcp_pose;
            cartesian_command_ = cartesian_state_;
            // Send a continuous first setpoint before returning control to the manager.
            robot_->StreamCartesianMotionForce(cartesian_command_);
            cartesian_enabled_ = cartesian_streaming_ = 1.0;
            cartesian_stop_latched_ = false;
        } else if (pending_mode_ == CommandMode::Position
            || pending_mode_ == CommandMode::Velocity) {
            robot_->SwitchMode(rdk_control_mode_);
            const auto state = robot_->states();
            for (size_t i = 0; i < rdk_to_ros_map_.size(); ++i) {
                hw_commands_joint_positions_[rdk_to_ros_map_[i]] = state.q[i];
            }
            std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(), 0.0);
        } else if (pending_mode_ == CommandMode::Torque) {
            robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);
            std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
                std::numeric_limits<double>::quiet_NaN());
        }
        active_mode_ = pending_mode_;
        claimed_arm_interfaces_ = pending_arm_interfaces_;
        return hardware_interface::return_type::OK;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Mode switch failed; streaming disabled: %s", e.what());
        // Preserve resulting claims so a subsequent deactivation can release them.
        claimed_arm_interfaces_ = pending_arm_interfaces_;
        stop_cartesian("mode switch failure");
        return hardware_interface::return_type::ERROR;
    }
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)
