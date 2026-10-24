#include "flexiv_cartesian_controller/cartesian_controller.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <thread>

using namespace std::chrono_literals;
using flexiv_cartesian_controller::FlexivCartesianController;

class ControllerTest : public testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    static unsigned serial = 0;
    const auto name = "cartesian_test_" + std::to_string(++serial);
    controller = std::make_shared<FlexivCartesianController>();
    ASSERT_EQ(controller->init(name), controller_interface::return_type::OK);
    std::vector<std::string> joints;
    for (int i=1;i<=7;++i) joints.push_back("test_joint"+std::to_string(i));
    controller->get_node()->set_parameters({rclcpp::Parameter("resource", "test_tcp"),
      rclcpp::Parameter("joints", joints), rclcpp::Parameter("require_rt_scheduling", false)});
    ASSERT_EQ(controller->configure().label(), "inactive");
    const auto command_names = controller->command_interface_configuration().names;
    const auto state_names = controller->state_interface_configuration().names;
    command_values.assign(command_names.size(), 0.);
    state_values = {.5, 0., .3, 1., 0., 0., 0., 1.};
    commands.reserve(command_names.size()); states.reserve(state_names.size());
    for(size_t i=0;i<command_names.size();++i) {
      const auto slash=command_names[i].rfind('/');
      commands.emplace_back(command_names[i].substr(0,slash),command_names[i].substr(slash+1),&command_values[i]);
    }
    for(size_t i=0;i<state_names.size();++i) {
      const auto slash=state_names[i].rfind('/');
      states.emplace_back(state_names[i].substr(0,slash),state_names[i].substr(slash+1),&state_values[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for(auto& handle:commands) command_loans.emplace_back(handle);
    for(auto& handle:states) state_loans.emplace_back(handle);
    controller->assign_interfaces(std::move(command_loans),std::move(state_loans));
    x_index=static_cast<size_t>(std::find(command_names.begin(),command_names.end(),"test_tcp/pose_x")-command_names.begin());
    enable_index=static_cast<size_t>(std::find(command_names.begin(),command_names.end(),"test_tcp/enabled")-command_names.begin());
    peer=std::make_shared<rclcpp::Node>(name+"_peer");
    publisher=peer->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/"+name+"/target_pose",rclcpp::QoS(1).best_effort());
    status=peer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/"+name+"/status",10,[this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for(const auto& row:msg->status) for(const auto& value:row.values) diagnostics[value.key]=value.value;
      });
    executor.add_node(controller->get_node()->get_node_base_interface());
    executor.add_node(peer);
    ASSERT_TRUE(until([this]{return publisher->get_subscription_count()==1;},2s));
    ASSERT_EQ(controller->get_node()->activate().label(),"active");
  }

  void TearDown() override {
    if(controller) {
      controller->get_node()->deactivate();
      executor.remove_node(controller->get_node()->get_node_base_interface());
      controller->release_interfaces();
    }
    if(peer) executor.remove_node(peer);
  }

  template<class Predicate, class Duration>
  bool until(Predicate predicate, Duration duration) {
    const auto deadline=std::chrono::steady_clock::now()+duration;
    do {executor.spin_some(); if(predicate())return true; std::this_thread::sleep_for(100us);}
    while(std::chrono::steady_clock::now()<deadline);
    return predicate();
  }
  void pump(std::chrono::milliseconds duration) { until([]{return false;},duration); }
  geometry_msgs::msg::PoseStamped target(double x=.501) {
    geometry_msgs::msg::PoseStamped msg;msg.header.frame_id="world";
    msg.header.stamp=controller->get_node()->now();
    msg.pose.position.x=x;msg.pose.position.z=.3;msg.pose.orientation.w=1.;return msg;
  }
  void publish(const geometry_msgs::msg::PoseStamped& msg) {publisher->publish(msg);pump(5ms);}
  std::string diagnostic(const std::string& key) {
    diagnostics.erase(key);
    EXPECT_TRUE(until([&]{return diagnostics.count(key)>0;},500ms));
    return diagnostics[key];
  }

  // Loaned interfaces refer to these stable backing buffers; no hardware exists.
  std::vector<double> command_values,state_values;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  std::shared_ptr<FlexivCartesianController> controller;
  rclcpp::Node::SharedPtr peer;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::map<std::string,std::string> diagnostics;
  size_t x_index=0,enable_index=0;
};

TEST_F(ControllerTest, AcceptsFreshTargetNewerThanControlCycleTimestamp) {
  const auto msg=target();
  const auto cycle_time=rclcpp::Time(msg.header.stamp)-rclcpp::Duration::from_seconds(.001);
  publish(msg);
  controller->update(cycle_time,rclcpp::Duration::from_seconds(.001));
  EXPECT_DOUBLE_EQ(command_values[x_index],msg.pose.position.x);
  EXPECT_DOUBLE_EQ(command_values[enable_index],1.);
  EXPECT_EQ(diagnostic("rejected_targets"),"0");
}

TEST_F(ControllerTest, ReportsInvalidMessageReasons) {
  auto msg=target();msg.header.frame_id="wrong_frame";publish(msg);
  msg=target();msg.header.stamp=controller->get_node()->now()+rclcpp::Duration::from_seconds(1.);publish(msg);
  msg=target();msg.header.stamp=controller->get_node()->now()-rclcpp::Duration::from_seconds(.101);publish(msg);
  msg=target();msg.pose.orientation.w=0.;publish(msg);
  EXPECT_EQ(diagnostic("rejected_targets"),"4");
  EXPECT_EQ(diagnostic("rejected_invalid_frame"),"1");
  EXPECT_EQ(diagnostic("rejected_invalid_timestamp"),"1");
  EXPECT_EQ(diagnostic("rejected_stale_timestamp"),"1");
  EXPECT_EQ(diagnostic("rejected_invalid_pose"),"1");
}

TEST_F(ControllerTest, RejectsQueueExpiredTargetAndStops) {
  publish(target());pump(110ms);
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(.001));
  EXPECT_DOUBLE_EQ(command_values[enable_index],0.);
  diagnostics.clear();
  EXPECT_EQ(diagnostic("rejected_expired_target"),"1");
}

TEST_F(ControllerTest, RejectsPreActivationReplay) {
  auto msg=target();msg.header.stamp=controller->get_node()->now()-rclcpp::Duration::from_seconds(.05);
  publish(msg);controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(.001));
  EXPECT_EQ(diagnostic("rejected_before_activation"),"1");
}

TEST_F(ControllerTest, PreservesTranslationAndRotationJumpLimits) {
  publish(target(.56));
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(.001));
  auto msg=target(.5);msg.pose.orientation.w=std::cos(.2);msg.pose.orientation.z=std::sin(.2);
  publish(msg);controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(.001));
  EXPECT_DOUBLE_EQ(command_values[x_index],.5);
  EXPECT_EQ(diagnostic("rejected_translation_jump"),"1");
  EXPECT_EQ(diagnostic("rejected_rotation_jump"),"1");
}
