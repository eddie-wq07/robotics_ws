// Copyright 2026 Industrial Next. All rights reserved.
#include "flexiv_cartesian_controller/cartesian_controller.hpp"
#include "flexiv_cartesian_controller/target_freshness.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sched.h>
#include <pluginlib/class_list_macros.hpp>

namespace flexiv_cartesian_controller {
namespace {
constexpr std::array<const char*, 7> kPoseNames = {
    "pose_x", "pose_y", "pose_z", "pose_qw", "pose_qx", "pose_qy", "pose_qz"};
using CallbackReturn = controller_interface::CallbackReturn;
}

int64_t FlexivCartesianController::steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

CallbackReturn FlexivCartesianController::on_init() {
  try {
    auto_declare<std::string>("resource", "");
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("world_frame", "world");
    auto_declare<std::string>("controller_manager", "/controller_manager");
    auto_declare<bool>("require_rt_scheduling", true);
    auto_declare<double>("command_timeout", 0.1);
    auto_declare<double>("max_translation_jump", 0.05);
    auto_declare<double>("max_rotation_jump", 0.35);
    auto_declare<double>("max_cycle_gap", 0.02);
    auto_declare<double>("state_rate", 100.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FlexivCartesianController::on_configure(const rclcpp_lifecycle::State&) {
  auto node = get_node();
  resource_ = node->get_parameter("resource").as_string();
  joints_ = node->get_parameter("joints").as_string_array();
  frame_ = node->get_parameter("world_frame").as_string();
  manager_ = node->get_parameter("controller_manager").as_string();
  require_rt_scheduling_ = node->get_parameter("require_rt_scheduling").as_bool();
  timeout_s_ = node->get_parameter("command_timeout").as_double();
  max_translation_ = node->get_parameter("max_translation_jump").as_double();
  max_rotation_ = node->get_parameter("max_rotation_jump").as_double();
  max_gap_s_ = node->get_parameter("max_cycle_gap").as_double();
  const double rate = node->get_parameter("state_rate").as_double();
  if (resource_.empty() || joints_.size() != 7 || frame_.empty() || manager_.empty()
      || !std::isfinite(timeout_s_) || timeout_s_ <= 0.0
      || !std::isfinite(max_translation_) || max_translation_ <= 0.0
      || !std::isfinite(max_rotation_) || max_rotation_ <= 0.0 || max_rotation_ > M_PI
      || !std::isfinite(max_gap_s_) || max_gap_s_ <= 0.001
      || !std::isfinite(rate) || rate <= 0.0 || rate > 1000.0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid Cartesian resource, joints, frame or limits");
    return CallbackReturn::ERROR;
  }
  auto sorted_joints = joints_;
  std::sort(sorted_joints.begin(), sorted_joints.end());
  if (std::adjacent_find(sorted_joints.begin(), sorted_joints.end()) != sorted_joints.end()
      || sorted_joints.front().empty()) return CallbackReturn::ERROR;
  if (node->get_parameter("use_sim_time").as_bool()) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian RT requires system ROS time, not /clock");
    return CallbackReturn::ERROR;
  }
  state_period_s_ = 1.0 / rate;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  qos.lifespan(rclcpp::Duration::from_seconds(timeout_s_));
  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { receive_target(*msg); });
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/tcp_pose", rclcpp::SensorDataQoS());
  pose_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(publisher);
  pose_pub_->msg_.header.frame_id = frame_;
  status_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/status", 1);
  switch_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(
      manager_ + "/switch_controller");
  supervisor_ = node->create_wall_timer(std::chrono::milliseconds(100), [this] { supervise(); });
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FlexivCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) result.names.push_back(joint + "/position");
  for (const auto* name : kPoseNames) result.names.push_back(resource_ + "/" + name);
  result.names.push_back(resource_ + "/enabled");
  return result;
}

controller_interface::InterfaceConfiguration
FlexivCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto* name : kPoseNames) result.names.push_back(resource_ + "/" + name);
  result.names.push_back(resource_ + "/streaming");
  return result;
}

std::array<double, 7> FlexivCartesianController::measured_pose() const {
  std::array<double, 7> result{};
  for (size_t i = 0; i < result.size(); ++i) result[i] = state_interfaces_[pose_states_[i]].get_value();
  return result;
}

CallbackReturn FlexivCartesianController::on_activate(const rclcpp_lifecycle::State&) {
  active_.store(false);
  const auto find_index = [](const auto& interfaces, const std::string& name) {
    for (size_t i = 0; i < interfaces.size(); ++i) if (interfaces[i].get_name() == name) return i;
    throw std::runtime_error("Missing interface: " + name);
  };
  bool have_enable = false;
  try {
    enabled_command_ = find_index(command_interfaces_, resource_ + "/enabled");
    have_enable = true;
    if (switch_pending_.load()) throw std::runtime_error("Previous stop request is still pending");
    if (require_rt_scheduling_ && sched_getscheduler(0) != SCHED_FIFO)
      throw std::runtime_error("Controller-manager update thread requires SCHED_FIFO; configure RT permissions before activation");
    for (size_t i = 0; i < kPoseNames.size(); ++i) {
      pose_commands_[i] = find_index(command_interfaces_, resource_ + "/" + kPoseNames[i]);
      pose_states_[i] = find_index(state_interfaces_, resource_ + "/" + kPoseNames[i]);
    }
    streaming_state_ = find_index(state_interfaces_, resource_ + "/streaming");
    target_pose_ = measured_pose();
    double norm = 0.0;
    for (size_t i = 0; i < target_pose_.size(); ++i) {
      if (!std::isfinite(target_pose_[i])) throw std::runtime_error("Invalid measured TCP pose");
      if (i >= 3) norm += target_pose_[i] * target_pose_[i];
    }
    if (std::abs(norm - 1.0) > 1e-3 || state_interfaces_[streaming_state_].get_value() != 1.0)
      throw std::runtime_error("Hardware did not enter Cartesian mode with a valid pose");
    for (size_t i = 0; i < target_pose_.size(); ++i)
      command_interfaces_[pose_commands_[i]].set_value(target_pose_[i]);
    command_interfaces_[enabled_command_].set_value(1.0);
  } catch (const std::exception& e) {
    if (have_enable) command_interfaces_[enabled_command_].set_value(0.0);
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  activation_ns_ = last_receipt_ns_ = steady_ns();
  activation_ros_ns_ = get_node()->now().nanoseconds();
  last_sequence_ = 0;
  generation_.fetch_add(1);
  stop_requested_.store(false);
  switch_pending_.store(false);
  publish_elapsed_s_ = 0.0;
  first_update_ = true;
  active_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

CallbackReturn FlexivCartesianController::on_deactivate(const rclcpp_lifecycle::State&) {
  active_.store(false, std::memory_order_release);
  generation_.fetch_add(1);
  if (enabled_command_ < command_interfaces_.size()) command_interfaces_[enabled_command_].set_value(0.0);
  return CallbackReturn::SUCCESS;
}

void FlexivCartesianController::receive_target(const geometry_msgs::msg::PoseStamped& msg) {
  if (!active_.load(std::memory_order_acquire)) return;
  Target target;
  target.generation = generation_.load();
  target.pose = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
      msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z};
  target.receipt_ns = steady_ns();
  const int64_t now = get_node()->now().nanoseconds();
  target.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  if (msg.header.frame_id != frame_) { reject(Rejection::invalid_frame); return; }
  if (msg.header.stamp.nanosec >= 1000000000U || target.stamp_ns <= 0 || target.stamp_ns > now) {
    reject(Rejection::invalid_timestamp); return;
  }
  target.source_age_ns = now - target.stamp_ns;
  if (target.source_age_ns > static_cast<int64_t>(timeout_s_ * 1e9)) {
    reject(Rejection::stale_timestamp); return;
  }
  double norm = 0.0;
  bool valid = true;
  for (size_t i = 0; i < target.pose.size(); ++i) {
    valid = valid && std::isfinite(target.pose[i]);
    if (i >= 3) norm += target.pose[i] * target.pose[i];
  }
  if (!valid || !std::isfinite(norm) || norm < 1e-12) {
    reject(Rejection::invalid_pose); return;
  }
  norm = std::sqrt(norm);
  for (size_t i = 3; i < 7; ++i) target.pose[i] /= norm;
  target.sequence = sequence_.fetch_add(1) + 1;
  target_buffer_.writeFromNonRT(target);
}

void FlexivCartesianController::reject(Rejection reason) {
  rejection_counts_[static_cast<size_t>(reason)].fetch_add(1, std::memory_order_relaxed);
  last_rejection_.store(reason, std::memory_order_relaxed);
  // Publish the reason before the supervisor observes an increased total.
  rejected_.fetch_add(1, std::memory_order_release);
}

void FlexivCartesianController::latch_stop() {
  command_interfaces_[enabled_command_].set_value(0.0);
  stop_requested_.store(true, std::memory_order_release);
}

controller_interface::return_type FlexivCartesianController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!active_.load(std::memory_order_acquire)) return controller_interface::return_type::OK;
  if (stop_requested_.load(std::memory_order_acquire)) {
    latch_stop(); return controller_interface::return_type::OK;
  }
  const auto measured = measured_pose();
  bool valid_state = state_interfaces_[streaming_state_].get_value() == 1.0;
  for (const double value : measured) valid_state = valid_state && std::isfinite(value);
  const int64_t period_ns = period.nanoseconds();
  if (!first_update_) {
    if (period_ns > 1500000) late_cycles_.fetch_add(1);
    if (period_ns > max_period_ns_.load()) max_period_ns_.store(period_ns);
    if (period.seconds() > max_gap_s_ || period_ns <= 0) valid_state = false;
  }
  first_update_ = false;
  if (!valid_state) { latch_stop(); return controller_interface::return_type::OK; }
  const Target target = *target_buffer_.readFromRT();
  // Read the clock after the buffer: a callback may have delivered a target
  // after this control cycle began, or even while measured state was read.
  const int64_t now = steady_ns();
  if (target.generation == generation_.load() && target.sequence > last_sequence_) {
    last_sequence_ = target.sequence;
    double delta_squared = 0.0, dot = 0.0;
    for (size_t i = 0; i < 3; ++i) delta_squared += std::pow(target.pose[i] - measured[i], 2);
    for (size_t i = 3; i < 7; ++i) dot += target.pose[i] * measured[i];
    const double angle = 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
    if (target.stamp_ns <= activation_ros_ns_) {
      reject(Rejection::before_activation);
    } else if (!target_is_fresh(target.source_age_ns, target.receipt_ns, now,
                                static_cast<int64_t>(timeout_s_ * 1e9))) {
      reject(Rejection::expired_target);
    } else if (delta_squared > max_translation_ * max_translation_) {
      reject(Rejection::translation_jump);
    } else if (angle > max_rotation_) {
      reject(Rejection::rotation_jump);
    } else {
      target_pose_ = target.pose;
      last_receipt_ns_ = target.receipt_ns;
    }
  }
  if (now - last_receipt_ns_ > static_cast<int64_t>(timeout_s_ * 1e9)) {
    latch_stop(); return controller_interface::return_type::OK;
  }
  for (size_t i = 0; i < target_pose_.size(); ++i)
    command_interfaces_[pose_commands_[i]].set_value(target_pose_[i]);
  command_interfaces_[enabled_command_].set_value(1.0);
  publish_elapsed_s_ += period.seconds();
  if (publish_elapsed_s_ >= state_period_s_ && pose_pub_->trylock()) {
    publish_elapsed_s_ = 0.0;
    auto& msg = pose_pub_->msg_;
    msg.header.stamp = time;
    msg.pose.position.x = measured[0]; msg.pose.position.y = measured[1]; msg.pose.position.z = measured[2];
    msg.pose.orientation.w = measured[3]; msg.pose.orientation.x = measured[4];
    msg.pose.orientation.y = measured[5]; msg.pose.orientation.z = measured[6];
    pose_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void FlexivCartesianController::supervise() {
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_node()->get_name();
  status.hardware_id = resource_;
  status.level = stop_requested_.load() ? 2 : 0;
  status.message = stop_requested_.load() ? "stop latched; explicit reactivation required"
      : active_.load() ? "active" : "inactive";
  for (const auto& pair : std::vector<std::pair<std::string, uint64_t>>{
      {"rejected_targets", rejected_.load()}, {"late_cycles_over_1_5ms", late_cycles_.load()},
      {"max_period_ns", static_cast<uint64_t>(max_period_ns_.load())}}) {
    diagnostic_msgs::msg::KeyValue value; value.key = pair.first; value.value = std::to_string(pair.second);
    status.values.push_back(value);
  }
  for (size_t i = 0; i < rejection_names_.size(); ++i) {
    diagnostic_msgs::msg::KeyValue value;
    value.key = std::string("rejected_") + rejection_names_[i];
    value.value = std::to_string(rejection_counts_[i].load(std::memory_order_relaxed));
    status.values.push_back(value);
  }
  diagnostic_msgs::msg::KeyValue reason;
  reason.key = "last_rejection_reason";
  const auto last = last_rejection_.load(std::memory_order_relaxed);
  reason.value = last == Rejection::count ? "none" : rejection_names_[static_cast<size_t>(last)];
  status.values.push_back(reason);
  msg.status.push_back(status); status_pub_->publish(msg);
  if (!active_.load() || !stop_requested_.load() || switch_pending_.load()
      || !switch_client_->service_is_ready()) return;
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = {get_node()->get_name()};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->timeout.sec = 2;
  switch_pending_.store(true);
  switch_client_->async_send_request(request, [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
    if (!future.get()->ok) RCLCPP_ERROR(get_node()->get_logger(), "Deactivation failed; stop remains latched");
    switch_pending_.store(false);
  });
}
}  // namespace flexiv_cartesian_controller
PLUGINLIB_EXPORT_CLASS(flexiv_cartesian_controller::FlexivCartesianController,
    controller_interface::ControllerInterface)
