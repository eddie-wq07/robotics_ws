#include "pen_execution/job.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <flexiv_msgs/msg/gpio_states.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pen_interfaces/action/execute_drawing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace pen_execution {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Execute = pen_interfaces::action::ExecuteDrawing;
using GoalHandle = rclcpp_action::ServerGoalHandle<Execute>;
using JointAction = control_msgs::action::FollowJointTrajectory;
using Switch = controller_manager_msgs::srv::SwitchController;

int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

Pose from_message(const geometry_msgs::msg::Pose& msg) {
  return {msg.position.x, msg.position.y, msg.position.z,
          msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w};
}
geometry_msgs::msg::Pose to_message(const Pose& pose) {
  geometry_msgs::msg::Pose result;
  result.position.x = pose[0]; result.position.y = pose[1]; result.position.z = pose[2];
  result.orientation.x = pose[3]; result.orientation.y = pose[4];
  result.orientation.z = pose[5]; result.orientation.w = pose[6];
  return result;
}

class Executor : public rclcpp::Node {
 public:
  Executor() : Node("pen_execution") {
    mode_ = declare_parameter<std::string>("execution_mode", "preview");
    manager_ = declare_parameter<std::string>("controller_manager", "/controller_manager");
    serial_ = declare_parameter<std::string>("robot_sn", "Rizon4s-000000");
    jobs_root_ = declare_parameter<std::string>("jobs_root", "/home/ubuntu/robotics_ws/assets/outputs");
    commissioning_ = declare_parameter<bool>("commissioning_verified", false);
    require_heartbeat_ = declare_parameter<bool>("require_heartbeat", true);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 2.0);
    tracking_limit_ = declare_parameter<double>("tracking_limit_m", 0.015);
    start_tolerance_ = declare_parameter<double>("start_tolerance_m", 0.002);
    if (mode_ != "preview" && mode_ != "fake" && mode_ != "real")
      throw std::runtime_error("execution_mode must be preview, fake or real");
    if (!std::isfinite(heartbeat_timeout_) || heartbeat_timeout_<=0 || heartbeat_timeout_>5 ||
        !std::isfinite(tracking_limit_) || tracking_limit_<=0 || tracking_limit_>0.015 ||
        !std::isfinite(start_tolerance_) || start_tolerance_<=0 || start_tolerance_>0.005)
      throw std::runtime_error("Invalid heartbeat, tracking, or start-tolerance parameter");
    if (get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("Execution requires system ROS time");
    for (int i=1; i<=7; ++i) joint_names_.push_back(serial_ + "_joint" + std::to_string(i));
    std::string topic_serial = serial_;
    std::replace(topic_serial.begin(), topic_serial.end(), '-', '_');
    const auto sensor_qos = rclcpp::SensorDataQoS();
    auto target_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    target_qos.lifespan(rclcpp::Duration::from_seconds(0.1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/flexiv_cartesian_controller/target_pose", target_qos);
    view_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pen/execution_pose", sensor_qos);
    gpio_pub_ = create_publisher<flexiv_msgs::msg::GPIOStates>(
        declare_parameter<std::string>("gpio_topic", "/"+topic_serial+"/gpio_outputs"), 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/pen/execution_status", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/flexiv_cartesian_controller/tcp_pose", sensor_qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { receive_pose(*msg, true); });
    vendor_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/"+topic_serial+"/tcp_pose", sensor_qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { receive_pose(*msg, false); });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/flexiv_rizon_arm/joint_states", sensor_qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          joints_ = *msg; joints_at_ = steady_ns();
        });
    controller_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/flexiv_cartesian_controller/status", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          controller_at_ = steady_ns();
          controller_active_ = false;
          for (const auto& state : msg->status) {
            controller_active_ = state.level < 2 && state.message == "active";
            controller_fault_ = state.level >= 2;
            for (const auto& value : state.values) if (value.key == "rejected_targets") {
              try { rejected_count_ = std::stoull(value.value); } catch (...) {}
            }
          }
        });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>("/pen/heartbeat", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) { heartbeat_at_ = steady_ns(); });
    stop_service_ = create_service<std_srvs::srv::Trigger>("/pen/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancel_requested_ = true;
        command_enabled_ = false;
        response->success = true;
        response->message = busy_ ? "Stop requested; inspect final execution status" : "No active job";
      });
    list_client_ = create_client<controller_manager_msgs::srv::ListControllers>(manager_+"/list_controllers");
    hardware_client_ = create_client<controller_manager_msgs::srv::ListHardwareComponents>(manager_+"/list_hardware_components");
    switch_client_ = create_client<Switch>(manager_+"/switch_controller");
    joint_client_ = rclcpp_action::create_client<JointAction>(this, "/rizon_arm_controller/follow_joint_trajectory");
    server_ = rclcpp_action::create_server<Execute>(this, "/pen/execute",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Execute::Goal> goal) {
        if (mode_ == "preview" || goal->job_path.empty() || goal->expected_sha256.size() != 64)
          return rclcpp_action::GoalResponse::REJECT;
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
        cancel_requested_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        cancel_requested_ = true; command_enabled_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> goal) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, goal] { run(goal); });
      });
    command_timer_ = create_wall_timer(10ms, [this] { publish_target(); });
    status_timer_ = create_wall_timer(100ms, [this] { publish_status(); });
    RCLCPP_INFO(get_logger(), "Pen execution ready in %s mode; no robot command on startup", mode_.c_str());
  }

  ~Executor() override {
    stopping_ = true; cancel_requested_ = true; command_enabled_ = false;
    if (worker_.joinable()) worker_.join();
  }

 private:
  void set_phase(const std::string& phase, double progress = -1) {
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = phase;
    if (progress >= 0) progress_ = progress;
  }

  void fail(const std::string& error) {
    command_enabled_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_.empty()) error_ = error;
  }

  void receive_pose(const geometry_msgs::msg::PoseStamped& msg, bool cartesian) {
    if (cartesian != cartesian_active_.load()) return;
    if (cartesian && rclcpp::Time(msg.header.stamp).nanoseconds() <= activation_ros_ns_.load()) return;
    const auto pose = from_message(msg.pose);
    const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
    if (msg.header.frame_id != "world" || !valid_pose(pose) ||
        age < -0.05 || age > 0.25 || rclcpp::Time(msg.header.stamp).nanoseconds() <= 0) {
      if (cartesian_active_) fail("Invalid or stale measured TCP pose");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      measured_ = pose; pose_at_ = steady_ns(); have_pose_ = true;
    }
    view_pub_->publish(msg);
  }

  Pose measured() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_ || (steady_ns()-pose_at_)*1e-9 > 0.25)
      throw std::runtime_error("Fresh measured TCP pose unavailable");
    return measured_;
  }

  void target(const Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = pose; target_at_ = steady_ns();
  }

  void refresh() {
    if (!command_enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    target_at_ = steady_ns();
  }

  void publish_target() {
    if (!command_enabled_ || cancel_requested_ || stopping_) return;
    const auto current = steady_ns();
    Pose pose;
    bool stale;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = target_;
      stale = current-target_at_ > 80000000 ||
              (last_publish_ && current-last_publish_ > 80000000);
      last_publish_ = current;
    }
    if (stale) { fail("Target scheduler stalled beyond 80 ms"); return; }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world"; msg.header.stamp = now(); msg.pose = to_message(pose);
    pose_pub_->publish(msg);
  }

  void publish_status() {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus state;
    state.name = "pen_execution"; state.hardware_id = serial_;
    std::lock_guard<std::mutex> lock(mutex_);
    state.level = error_.empty() ? 0 : 2;
    state.message = phase_;
    const std::map<std::string,std::string> fields = {
      {"execution_mode",mode_}, {"busy",busy_ ? "true":"false"},
      {"progress",std::to_string(progress_)}, {"sample_index",std::to_string(sample_index_)},
      {"extrude_requested",extrude_ ? "true":"false"}, {"error",error_},
      {"job_sha256",job_digest_}, {"pen_feedback","request_only"}};
    for (const auto& pair : fields) {
      diagnostic_msgs::msg::KeyValue value; value.key=pair.first; value.value=pair.second;
      state.values.push_back(value);
    }
    message.status.push_back(state); status_pub_->publish(message);
  }

  void check(bool monitor = true) {
    if (stopping_ || !rclcpp::ok()) throw std::runtime_error("Executor shutting down");
    if (cancel_requested_) throw std::runtime_error("Execution canceled");
    if (require_heartbeat_ && (steady_ns()-heartbeat_at_.load())*1e-9 > heartbeat_timeout_)
      throw std::runtime_error("Client heartbeat expired");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error_.empty()) throw std::runtime_error(error_);
    if (monitor && cartesian_active_ && monitoring_) {
      if (!have_pose_ || (steady_ns()-pose_at_)*1e-9 > 0.25)
        throw std::runtime_error("Measured TCP feedback expired");
      if ((steady_ns()-controller_at_)*1e-9 > 0.5 || !controller_active_ || controller_fault_)
        throw std::runtime_error("Cartesian controller inactive, faulted or status stale");
      if (rejected_count_ > rejected_baseline_)
        throw std::runtime_error("Cartesian controller rejected targets");
      if (distance(target_,measured_) > tracking_limit_ || angular_distance(target_,measured_) > 0.25)
        throw std::runtime_error("TCP tracking error exceeded configured limit");
    }
  }

  void wait(double seconds, bool monitor = true) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (Clock::now() < end) { check(monitor); refresh(); std::this_thread::sleep_for(2ms); }
  }

  void wait_initial_heartbeat() {
    if (!require_heartbeat_) return;
    const auto deadline = Clock::now() + std::chrono::duration<double>(heartbeat_timeout_);
    while ((steady_ns() - heartbeat_at_.load()) * 1e-9 > heartbeat_timeout_) {
      // The first browser heartbeat and action goal travel independently. Allow
      // their delivery order to differ, without sending any hardware command.
      if (stopping_ || !rclcpp::ok()) throw std::runtime_error("Executor shutting down");
      if (cancel_requested_) throw std::runtime_error("Execution canceled");
      if (Clock::now() >= deadline)
        throw std::runtime_error("Client heartbeat unavailable during initial preflight");
      std::this_thread::sleep_for(2ms);
    }
  }

  template<class Future>
  auto wait_future(Future& future, double seconds, bool inspect = true) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (future.wait_for(0ms) != std::future_status::ready) {
      if (Clock::now() > end) throw std::runtime_error("ROS operation timed out");
      if (inspect) check();
      refresh(); std::this_thread::sleep_for(2ms);
    }
    return future.get();
  }

  template<class Service>
  typename Service::Response::SharedPtr call(
      const typename rclcpp::Client<Service>::SharedPtr& client,
      const typename Service::Request::SharedPtr& request, bool inspect = true) {
    if (!client->wait_for_service(2s)) throw std::runtime_error("Service unavailable: "+std::string(client->get_service_name()));
    auto future = client->async_send_request(request);
    return wait_future(future, 5.0, inspect);
  }

  std::map<std::string,std::string> controllers(bool inspect = true) {
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    const auto response = call<controller_manager_msgs::srv::ListControllers>(list_client_,request,inspect);
    std::map<std::string,std::string> result;
    for (const auto& controller : response->controller) result[controller.name] = controller.state;
    return result;
  }

  void switch_to(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate,
                 bool inspect = true) {
    auto request = std::make_shared<Switch::Request>();
    request->activate_controllers=activate; request->deactivate_controllers=deactivate;
    request->strictness=Switch::Request::STRICT; request->activate_asap=true;
    request->timeout.sec=3;
    if (!call<Switch>(switch_client_,request,inspect)->ok)
      throw std::runtime_error("Controller switch rejected; rollback is not assumed");
  }

  void pen(bool extrude, bool force=false) {
    if (extrude_ == extrude && !force) return;
    flexiv_msgs::msg::GPIOStates msg;
    flexiv_msgs::msg::Digital output;
    output.pin=0; output.state=extrude; msg.states.push_back(output);
    gpio_pub_->publish(msg); extrude_=extrude;
  }

  void hardware_preflight(const Job& job) {
    auto request=std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>();
    const auto response=call<controller_manager_msgs::srv::ListHardwareComponents>(hardware_client_,request);
    if (response->component.size()!=1)
      throw std::runtime_error("Expected exactly one robot hardware component");
    const auto& component=response->component.front();
    if (component.state.label!="active") throw std::runtime_error("Robot hardware is not active");
    const bool fake=component.class_type=="mock_components/GenericSystem";
    if ((mode_=="fake")!=fake) throw std::runtime_error("Execution mode does not match loaded hardware plugin");
    bool identity=false;
    for (const auto& interface:component.command_interfaces)
      if (interface.name==serial_+"_joint1/position") identity=true;
    if (!identity) throw std::runtime_error("Loaded robot identity does not match executor");
    if (!fake) {
      if (!commissioning_ || job.fake_only || !job.calibrated)
        throw std::runtime_error("Physical execution requires measured calibration and commissioning_verified=true");
      const auto qualified=job.source["qualification"];
      for (const char* field:{"rt", "clearance", "pen", "tool"})
        if (!qualified || !qualified[field] || !qualified[field].as<bool>())
          throw std::runtime_error(std::string("Physical qualification missing: ")+field);
      if (!job.source["profile_path"] || !job.source["profile_source_sha256"])
        throw std::runtime_error("Physical job lacks calibration profile source identity");
      std::ifstream input(job.source["profile_path"].as<std::string>(),std::ios::binary);
      std::string bytes((std::istreambuf_iterator<char>(input)),{});
      if (!input || sha256(bytes)!=job.source["profile_source_sha256"].as<std::string>())
        throw std::runtime_error("Calibration profile changed since preview");
    }
    if (pose_pub_->get_subscription_count()!=1 || count_publishers("/flexiv_cartesian_controller/target_pose")!=1)
      throw std::runtime_error("Expected one Cartesian subscriber and exclusive target publisher");
    const auto state=controllers();
    if (state.find("rizon_arm_controller")==state.end() || state.at("rizon_arm_controller")!="active" ||
        state.find("flexiv_cartesian_controller")==state.end() || state.at("flexiv_cartesian_controller")!="inactive")
      throw std::runtime_error("Start with joint active and Cartesian inactive");
    if (state.find("gpio_controller")==state.end() || state.at("gpio_controller")!="active")
      throw std::runtime_error("GPIO controller must be active");
  }

  std::vector<double> joint_positions() {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((steady_ns()-joints_at_)*1e-9>0.5) throw std::runtime_error("Joint feedback stale");
    std::vector<double> result;
    for (const auto& name:joint_names_) {
      auto found=std::find(joints_.name.begin(),joints_.name.end(),name);
      if (found==joints_.name.end()) throw std::runtime_error("Joint feedback identity mismatch");
      size_t i=static_cast<size_t>(found-joints_.name.begin());
      if (i>=joints_.position.size() || !std::isfinite(joints_.position[i]))
        throw std::runtime_error("Invalid joint feedback");
      result.push_back(joints_.position[i]);
    }
    return result;
  }

  void joint_route(const YAML::Node& route) {
    if (!route || !route["qualified"] || !route["qualified"].as<bool>())
      throw std::runtime_error("A qualified joint route is required");
    const auto start=route["start_positions"].as<std::vector<double>>();
    const auto current=joint_positions();
    if (start.size()!=7) throw std::runtime_error("Joint route requires seven start positions");
    for (size_t i=0;i<7;++i)
      if (!std::isfinite(start[i]) || std::abs(start[i]-current[i])>0.03)
        throw std::runtime_error("Robot is outside joint route start tolerance");
    JointAction::Goal goal; goal.trajectory.joint_names=joint_names_;
    double previous=0;
    const auto rows=route["points"];
    if (!rows.IsSequence() || rows.size()==0) throw std::runtime_error("Joint route has no points");
    for (const auto& row:rows) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions=row["positions"].as<std::vector<double>>();
      const double time=row["time_s"].as<double>();
      if (point.positions.size()!=7 || !std::isfinite(time) || time<=previous || time>120)
        throw std::runtime_error("Invalid joint route point");
      for (double p:point.positions) if (!std::isfinite(p)) throw std::runtime_error("Nonfinite joint route");
      point.time_from_start=rclcpp::Duration::from_seconds(time); previous=time;
      goal.trajectory.points.push_back(point);
    }
    if (!joint_client_->wait_for_action_server(2s)) throw std::runtime_error("Joint action unavailable");
    auto accepted=joint_client_->async_send_goal(goal);
    auto handle=wait_future(accepted,3.0);
    if (!handle) throw std::runtime_error("Joint route rejected");
    auto result=joint_client_->async_get_result(handle);
    const auto response=wait_future(result,previous+5.0);
    if (response.code!=rclcpp_action::ResultCode::SUCCEEDED || response.result->error_code!=0)
      throw std::runtime_error("Joint route failed");
  }

  void begin_cartesian() {
    set_phase("SWITCHING_TO_CARTESIAN");
    activation_ros_ns_=now().nanoseconds();
    cartesian_active_=true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_pose_=false; last_publish_=0;
      rejected_baseline_=rejected_count_;
    }
    owns_switch_=true;
    switch_to({"flexiv_cartesian_controller"},{"rizon_arm_controller"});
    const auto deadline=Clock::now()+65ms;
    while (true) {
      check(false);
      bool available;
      { std::lock_guard<std::mutex> lock(mutex_); available=have_pose_; }
      if (available) break;
      if (Clock::now()>deadline) throw std::runtime_error("No Cartesian feedback within activation grace");
      std::this_thread::sleep_for(1ms);
    }
    target(measured()); command_enabled_=true;
    wait(0.15,false);
    monitoring_=true;
    wait(0.1);
    const auto state=controllers();
    if (state.at("flexiv_cartesian_controller")!="active" || state.at("rizon_arm_controller")!="inactive")
      throw std::runtime_error("Controller ownership after switching is invalid");
  }

  void fake_position(const Pose& destination) {
    set_phase("FAKE_PREPOSITION");
    const auto origin=measured();
    const double duration=std::max({0.5,distance(origin,destination)/0.08,angular_distance(origin,destination)/1.0});
    const auto start=Clock::now();
    while (true) {
      const double u=std::min(1.0,std::chrono::duration<double>(Clock::now()-start).count()/duration);
      target(blend_pose(origin,destination,(1-std::cos(M_PI*u))/2));
      check(); if (u>=1) break; std::this_thread::sleep_for(3ms);
    }
    wait(0.15);
  }

  bool stop_after_error() {
    command_enabled_=false; monitoring_=false;
    if (outputs_owned_) {
      try { pen(false,true); }
      catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),"Could not send pen release: %s",e.what());
      }
    }
    if (owns_switch_) {
      try { joint_client_->async_cancel_all_goals(); } catch (...) {}
    }
    if (!owns_switch_) return true;
    try {
      const auto state=controllers(false);
      std::vector<std::string> deactivate;
      for (const auto& name:{"flexiv_cartesian_controller","rizon_arm_controller"})
        if (state.count(name) && state.at(name)=="active") deactivate.emplace_back(name);
      if (!deactivate.empty()) switch_to({},deactivate,false);
      owns_switch_=false; cartesian_active_=false;
      const auto after=controllers(false);
      return after.at("flexiv_cartesian_controller")!="active" && after.at("rizon_arm_controller")!="active";
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),"Could not confirm stop: %s",e.what()); return false;
    }
  }

  void run(std::shared_ptr<GoalHandle> goal) {
    auto result=std::make_shared<Execute::Result>();
    outputs_owned_=false; owns_switch_=false; cartesian_active_=false; monitoring_=false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_.clear(); progress_=0; sample_index_=0; job_digest_.clear();
    }
    try {
      set_phase("VALIDATING");
      const auto job=load_job(goal->get_goal()->job_path,jobs_root_,goal->get_goal()->expected_sha256);
      {
        std::lock_guard<std::mutex> lock(mutex_); job_digest_=job.digest;
      }
      if (goal->get_goal()->air_run && (!job.source["air_run"] || !job.source["air_run"].as<bool>()))
        throw std::runtime_error("Air run requires an independently compiled and previewed air-offset job");
      set_phase("PREFLIGHT"); wait_initial_heartbeat(); check(false); hardware_preflight(job);
      outputs_owned_=true; pen(false,true);
      if (mode_=="real") {
        set_phase("JOINT_APPROACH"); owns_switch_=true;
        joint_route(job.source["joint_routes"]["approach"]);
        const auto current=measured();
        if (distance(current,job.samples.front().pose)>start_tolerance_ ||
            angular_distance(current,job.samples.front().pose)>0.03)
          throw std::runtime_error("TCP is outside reviewed trajectory start tolerance");
      }
      begin_cartesian();
      if (mode_=="fake") fake_position(job.samples.front().pose);
      else if (distance(measured(),job.samples.front().pose)>start_tolerance_)
        throw std::runtime_error("TCP moved during controller switch");
      const auto began=Clock::now();
      int64_t feedback_at=0;
      std::string previous_phase;
      for (size_t i=0;i<job.samples.size();++i) {
        const auto& sample=job.samples[i];
        while (std::chrono::duration<double>(Clock::now()-began).count()<sample.time_s) {
          check(); refresh(); std::this_thread::sleep_for(1ms);
        }
        if (std::chrono::duration<double>(Clock::now()-began).count()-sample.time_s>0.04)
          throw std::runtime_error("Trajectory schedule missed by more than 40 ms");
        // At a semantic release/lift boundary resend OFF, even if already off.
        pen(sample.extrude, sample.phase!=previous_phase &&
            (sample.phase=="RELEASE" || sample.phase=="LIFT"));
        previous_phase=sample.phase;
        target(sample.pose); check();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          sample_index_=static_cast<uint32_t>(i);
        }
        set_phase(sample.phase, static_cast<double>(i)/(job.samples.size()-1));
        if (steady_ns()-feedback_at>100000000) {
          auto feedback=std::make_shared<Execute::Feedback>();
          feedback->phase=sample.phase; feedback->progress=static_cast<float>(i)/static_cast<float>(job.samples.size()-1);
          feedback->sample_index=static_cast<uint32_t>(i); goal->publish_feedback(feedback);
          feedback_at=steady_ns();
        }
      }
      pen(false,true); wait(0.2);
      set_phase("SWITCHING_TO_JOINT"); monitoring_=false;
      switch_to({"rizon_arm_controller"},{"flexiv_cartesian_controller"});
      command_enabled_=false; cartesian_active_=false;
      const auto state=controllers();
      if (state.at("rizon_arm_controller")!="active" || state.at("flexiv_cartesian_controller")!="inactive")
        throw std::runtime_error("Joint control restoration failed");
      if (mode_=="real") { set_phase("JOINT_PARK"); joint_route(job.source["joint_routes"]["park"]); }
      owns_switch_=false; outputs_owned_=false;
      result->success=true;
      result->message=mode_=="fake" ? "Fake hardware job completed; physical readiness not established" :
          "Trajectory completed; pen status is command-only unless independently observed";
      set_phase("COMPLETE",1); busy_=false; goal->succeed(result);
    } catch (const std::exception& e) {
      fail(e.what()); set_phase("STOPPING");
      const bool stopped=stop_after_error();
      result->success=false; result->message=std::string(e.what())+
          (stopped ? "; commands stopped" : "; robot stop unconfirmed");
      set_phase(stopped ? "STOPPED" : "FAULT");
      busy_=false;
      if (rclcpp::ok()) {
        if (goal->is_canceling()) goal->canceled(result); else goal->abort(result);
      }
    }
    command_enabled_=false;
  }

  std::string mode_,manager_,serial_,jobs_root_;
  bool commissioning_,require_heartbeat_;
  double heartbeat_timeout_,tracking_limit_,start_tolerance_;
  std::vector<std::string> joint_names_;
  std::atomic<bool> busy_{false},cancel_requested_{false},stopping_{false};
  std::atomic<bool> command_enabled_{false},cartesian_active_{false},monitoring_{false},extrude_{false};
  bool owns_switch_=false,outputs_owned_=false;
  std::atomic<int64_t> heartbeat_at_{0},activation_ros_ns_{0};
  std::thread worker_;
  std::mutex mutex_;
  Pose target_{0,0,0,0,0,0,1},measured_{0,0,0,0,0,0,1};
  int64_t target_at_=0,last_publish_=0,pose_at_=0,joints_at_=0,controller_at_=0;
  bool have_pose_=false,controller_active_=false,controller_fault_=false;
  uint64_t rejected_count_=0,rejected_baseline_=0;
  uint32_t sample_index_=0;
  double progress_=0;
  std::string phase_="IDLE",error_,job_digest_;
  sensor_msgs::msg::JointState joints_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_,view_pub_;
  rclcpp::Publisher<flexiv_msgs::msg::GPIOStates>::SharedPtr gpio_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_,vendor_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr controller_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr hardware_client_;
  rclcpp::Client<Switch>::SharedPtr switch_client_;
  rclcpp_action::Client<JointAction>::SharedPtr joint_client_;
  rclcpp_action::Server<Execute>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr command_timer_,status_timer_;
};
}  // namespace pen_execution

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<pen_execution::Executor>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
    executor.add_node(node); executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("pen_execution"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
