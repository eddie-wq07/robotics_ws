// Copyright 2026 Industrial Next. All rights reserved.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace flexiv_cartesian_controller {
class FlexivCartesianController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

 private:
  struct Target {
    std::array<double, 7> pose{};
    int64_t receipt_ns = 0;
    int64_t stamp_ns = 0;
    int64_t source_age_ns = 0;
    uint64_t generation = 0;
    uint64_t sequence = 0;
  };
  static int64_t steady_ns();
  enum class Rejection : size_t {
    invalid_frame, invalid_timestamp, stale_timestamp, invalid_pose,
    before_activation, expired_target, translation_jump, rotation_jump, count
  };
  static constexpr std::array<const char*, static_cast<size_t>(Rejection::count)> rejection_names_ = {
    "invalid_frame", "invalid_timestamp", "stale_timestamp", "invalid_pose",
    "before_activation", "expired_target", "translation_jump", "rotation_jump"};
  void reject(Rejection reason);
  void receive_target(const geometry_msgs::msg::PoseStamped&);
  void supervise();
  void latch_stop();
  std::array<double, 7> measured_pose() const;

  std::string resource_, frame_, manager_;
  bool require_rt_scheduling_ = true;
  std::vector<std::string> joints_;
  double timeout_s_ = 0.1, max_translation_ = 0.05, max_rotation_ = 0.35;
  double max_gap_s_ = 0.02, state_period_s_ = 0.01;
  std::array<size_t, 7> pose_commands_{}, pose_states_{};
  size_t enabled_command_ = 0, streaming_state_ = 0;
  std::array<double, 7> target_pose_{};
  int64_t last_receipt_ns_ = 0, activation_ns_ = 0, activation_ros_ns_ = 0;
  uint64_t last_sequence_ = 0;
  double publish_elapsed_s_ = 0.0;
  bool first_update_ = true;
  std::atomic<bool> active_{false}, stop_requested_{false}, switch_pending_{false};
  std::atomic<uint64_t> generation_{0}, sequence_{0}, rejected_{0}, late_cycles_{0};
  std::array<std::atomic<uint64_t>, static_cast<size_t>(Rejection::count)> rejection_counts_{};
  std::atomic<Rejection> last_rejection_{Rejection::count};
  std::atomic<int64_t> max_period_ns_{0};
  realtime_tools::RealtimeBuffer<Target> target_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr supervisor_;
};
}  // namespace flexiv_cartesian_controller
