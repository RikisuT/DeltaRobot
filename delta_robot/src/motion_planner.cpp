/// @file motion_planner.cpp
/// @brief Motion Planning Implementation for Delta Robot
///
/// PARAMETERS:
///   None
///
/// PUBLISHERS:
///   ~/delta_motors/set_joints (deltarobot_interfaces::msg::DeltaJoints): Publishes joint position commands to the delta robot motors
///   ~/delta_motors/set_joint_vels (deltarobot_interfaces::msg::DeltaJointVels): Publishes joint velocity commands to the delta robot motors
///
/// SUBSCRIBERS:
///   None
///
/// SERVICES:
///   ~/delta_motion_planner/play_demo_trajectory (deltarobot_interfaces::srv::PlayDemoTrajectory): Service to play predefined demonstration trajectories (up_down, pringle, axes, circle, scan)
///   ~/delta_motion_planner/move_to_point (deltarobot_interfaces::srv::MoveToPoint): Service to move the end effector to a specific 3D point (only in TASK_MODE)
///   ~/delta_motion_planner/move_to_configuration (deltarobot_interfaces::srv::MoveToConfiguration): Service to move the robot to a specific joint configuration
///   ~/delta_motion_planner/motion_demo (deltarobot_interfaces::srv::MotionDemo): Service to start/stop the automatic demo mode
///   ~/delta_motion_planner/set_motion_mode (deltarobot_interfaces::srv::SetMotionMode): Service to switch between TASK_MODE and LIVE_TEACH_MODE
///
/// CLIENTS:
///   ~/delta_kinematics/delta_ik (deltarobot_interfaces::srv::DeltaIK): Client to compute inverse kinematics (joint angles from end effector position)
///   ~/delta_kinematics/delta_fk (deltarobot_interfaces::srv::DeltaFK): Client to compute forward kinematics (end effector position from joint angles)
///   ~/delta_kinematics/convert_to_joint_trajectory (deltarobot_interfaces::srv::ConvertToJointTrajectory): Client to convert end effector trajectory to joint trajectory
///   ~/delta_kinematics/convert_to_joint_vel_trajectory (deltarobot_interfaces::srv::ConvertToJointVelTrajectory): Client to convert end effector velocity trajectory to joint velocity trajectory

#include "rclcpp/rclcpp.hpp"
#include "motion_planner.hpp"
#include <numeric>  // add this at the top
// Include all the custom messages and services
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <ctime>

template<typename T>
using ServiceResponseFuture = typename rclcpp::Client<T>::SharedFuture;

using Point = geometry_msgs::msg::Point;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using PlayDemoTraj = deltarobot_interfaces::srv::PlayDemoTrajectory;
using PlayCustomTrajectory = deltarobot_interfaces::srv::PlayCustomTrajectory;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using ConvertToJointVelTrajectory = deltarobot_interfaces::srv::ConvertToJointVelTrajectory;

DeltaMotionPlanner::DeltaMotionPlanner() : Node("delta_motion_planner") {
  RCLCPP_INFO(get_logger(), "DeltaMotionPlanner node started");

  // Create clients first (non-blocking)
  this->delta_ik_client = create_client<DeltaIK>("delta_kinematics/delta_ik");
  this->delta_fk_client = create_client<DeltaFK>("delta_kinematics/delta_fk");
  this->convert_to_joint_trajectory_client = 
    create_client<ConvertToJointTrajectory>("delta_kinematics/convert_to_joint_trajectory");
  this->convert_to_joint_vel_trajectory_client = 
    create_client<ConvertToJointVelTrajectory>("delta_kinematics/convert_to_joint_vel_trajectory");
  
  // Declare and get parameters
  this->declare_parameter("traj_step_ms", 10);
  this->param_traj_step_ms = this->get_parameter("traj_step_ms").as_int();
  this->declare_parameter("live_controller_ms", 20);
  this->param_live_controller_ms = this->get_parameter("live_controller_ms").as_int();
  this->declare_parameter<std::string>("live_target_topic", "delta_motion_planner/live_target");
  this->live_target_topic = this->get_parameter("live_target_topic").as_string();
  this->declare_parameter<std::vector<std::string>>(
    "controller_joint_names",
    {"motor_joint_1", "motor_joint_2", "motor_joint_3", "differential_pinion_joint_1", "differential_pinion_joint_2", "differential_T_joint", "differential_EE_joint"});
  this->controller_joint_names = this->get_parameter("controller_joint_names").as_string_array();

  // Create publishers
  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  this->joint_pub = this->create_publisher<DeltaJoints>("delta_motors/set_joints", QOS_RKL10V);
  this->joint_vel_pub = this->create_publisher<DeltaJointVels>("delta_motors/set_joint_vels", QOS_RKL10V);
  this->trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory",
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());

  this->live_target_sub = this->create_subscription<Point>(
    this->live_target_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const Point::SharedPtr msg) {
      this->liveTargetCallback(msg);
    });

  // Defer service server + timer creation until kinematics services are available
  this->init_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() -> void {
      if (!this->delta_ik_client->service_is_ready() ||
          !this->delta_fk_client->service_is_ready() ||
          !this->convert_to_joint_trajectory_client->service_is_ready() ||
          !this->convert_to_joint_vel_trajectory_client->service_is_ready()) {
        RCLCPP_INFO(get_logger(), "Waiting for delta_kinematics services...");
        return;
      }

      // Cancel this timer - we're done waiting
      this->init_timer->cancel();
      RCLCPP_INFO(get_logger(), "delta_kinematics services found, initializing...");

      // Now safe to create servers and demo timer
      this->demo_traj_server = create_service<PlayDemoTraj>(
        "delta_motion_planner/play_demo_trajectory",
        std::bind(&DeltaMotionPlanner::playDemoTrajectory, this,
          std::placeholders::_1, std::placeholders::_2));

      this->move_to_point_server = create_service<MoveToPoint>(
        "delta_motion_planner/move_to_point",
        [this](const std::shared_ptr<MoveToPoint::Request> request,
               std::shared_ptr<MoveToPoint::Response> response) {
          response->success = this->moveToPoint(request->target, true);
        });

      this->move_to_configuration_server = create_service<MoveToConfiguration>(
        "delta_motion_planner/move_to_configuration",
        [this](const std::shared_ptr<MoveToConfiguration::Request> request,
               std::shared_ptr<MoveToConfiguration::Response> response) {
          response->success = this->moveToConfiguration(request->target_joint_angles, true);
        });

      this->motion_demo_server = create_service<MotionDemo>(
        "delta_motion_planner/motion_demo",
        [this](const std::shared_ptr<MotionDemo::Request> request,
               [[maybe_unused]] std::shared_ptr<MotionDemo::Response> response) {
          std::lock_guard<std::mutex> lock(this->mode_mutex);
          if (this->current_mode != TASK_MODE) {
            RCLCPP_WARN(get_logger(), "Demo mode is only available in TASK_MODE");
            this->playDemo = false;
            return;
          }
          this->playDemo = request->start;
          if (this->playDemo) {
            this->demo_sequence_index = 0;
          }
        });

      this->play_custom_trajectory_server = create_service<PlayCustomTrajectory>(
        "delta_motion_planner/play_custom_trajectory",
        std::bind(&DeltaMotionPlanner::playCustomTrajectory, this,
          std::placeholders::_1, std::placeholders::_2));

      this->set_motion_mode_server = create_service<SetMotionMode>(
        "delta_motion_planner/set_motion_mode",
        std::bind(&DeltaMotionPlanner::setMotionMode, this,
          std::placeholders::_1, std::placeholders::_2));

      this->initialized = true;

      // Live motion controller: runs at fixed rate (~50-100 Hz) for smooth real-time control
      // Reads latest target from buffer and executes immediately without service latency
      this->live_controller_timer = this->create_wall_timer(
        std::chrono::milliseconds(this->param_live_controller_ms),
        [this]() -> void {
          this->liveMotionController();
        });

      const float demoDelay = 32;
      this->demo_timer = this->create_wall_timer(
        std::chrono::duration<float>(demoDelay),
        [this]() -> void {
          bool task_mode = false;
          {
            std::lock_guard<std::mutex> lock(this->mode_mutex);
            task_mode = (this->current_mode == TASK_MODE);
          }
          if (this->playDemo && task_mode && !this->motion_active.load()) {
            switch (this->demo_sequence_index % 4) {
              case 0:
                RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: pringle");
                this->playTrajectory(this->pringleTrajectory());
                break;
              case 1:
                RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: circle");
                this->playTrajectory(this->circleTrajectory());
                break;
              case 2:
                RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: axes");
                this->playTrajectory(this->axesTrajectory());
                break;
              case 3:
              default:
                RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: random sample");
                this->playTrajectory(this->randomSampleTrajectory(20));
                break;
            }
            this->demo_sequence_index++;
          }
        });
    });
}

DeltaMotionPlanner::~DeltaMotionPlanner() {
  this->cancel_current_traj = true;
  if (this->traj_thread && this->traj_thread->joinable()) {
    this->traj_thread->join();
  }
}

void DeltaMotionPlanner::publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms) {
  // Build a single JointTrajectory message with all points and cumulative time stamps
  // The joint_trajectory_controller rejects single-point messages whose timestamp has already passed
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = rclcpp::Time(0);  // 0 = "execute immediately" for joint_trajectory_controller
  traj_msg.joint_names = this->controller_joint_names;

  const unsigned int step_ms = (delay_ms > 0) ? delay_ms : static_cast<unsigned int>(this->param_traj_step_ms);
  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    if (this->cancel_current_traj) break;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
      joint_traj[i].theta1, joint_traj[i].theta2, joint_traj[i].theta3,
      0.0, 0.0, 0.0, 0.0
    };
    // Each point is step_ms further in the future
    const uint64_t t_ns = static_cast<uint64_t>(i + 1) * step_ms * 1000000ULL;
    point.time_from_start.sec = static_cast<int32_t>(t_ns / 1000000000ULL);
    point.time_from_start.nanosec = static_cast<uint32_t>(t_ns % 1000000000ULL);
    traj_msg.points.push_back(point);
  }

  // Publish the full trajectory once to Gazebo
  this->trajectory_pub->publish(traj_msg);

  // Publish the joint commands to the physical motors with a delay per point
  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    if (this->cancel_current_traj) break;
    this->joint_pub->publish(joint_traj[i]);
    rclcpp::sleep_for(std::chrono::milliseconds(step_ms));
  }
}

void DeltaMotionPlanner::publishMotorVelocityCommands(const std::vector<DeltaJointVels>& joint_vel_traj, const unsigned int delay_ms) {
  // Publish the joint velocity trajectory to the motors with a small delay [ms] between each point
  for (unsigned int i = 0; i < joint_vel_traj.size(); i++) {
    this->joint_vel_pub->publish(joint_vel_traj[i]);
    rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

bool DeltaMotionPlanner::tryAcquireMotionSlot(const char* action_name) {
  bool expected = false;
  if (!this->motion_active.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "%s rejected: another motion is already active", action_name);
    return false;
  }
  return true;
}

void DeltaMotionPlanner::releaseMotionSlot() {
  this->motion_active = false;
}

bool DeltaMotionPlanner::moveToPoint(const Point& point, bool require_task_mode) {
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (require_task_mode && this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "move_to_point rejected: not in TASK_MODE");
      return false;
    }
    if (!require_task_mode && this->current_mode != LIVE_TEACH_MODE) {
      RCLCPP_WARN(get_logger(), "live target rejected: not in LIVE_TEACH_MODE");
      return false;
    }
  }

  if (!this->tryAcquireMotionSlot(require_task_mode ? "move_to_point" : "live target")) {
    return false;
  }

  // Perform IK to get the joint angles and to ensure if the point is reachable
  auto ik_request = std::make_shared<DeltaIK::Request>();
  ik_request->solution = point;

  auto future_result = this->delta_ik_client->async_send_request(
    ik_request,
    [this](ServiceResponseFuture<DeltaIK> future) {
    auto response = future.get();
    if (response->success) {
      // If the IK solution is valid, move to the point
      std::vector<DeltaJoints> joint_traj = {response->joint_angles};
      this->publishMotorCommands(joint_traj, 0);
    } else {
      RCLCPP_ERROR(get_logger(), "IK solution not found for the given end effector point");
    }
    this->releaseMotionSlot();
  }
  );

  (void)future_result;
  return true;
}

void DeltaMotionPlanner::liveTargetCallback(const Point::SharedPtr msg) {
  if (!this->initialized) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != LIVE_TEACH_MODE) {
      return;
    }
  }

  // Store the latest target in the buffer (overwrite semantics)
  // The fixed-rate live motion controller will read this and execute it
  {
    std::lock_guard<std::mutex> lock(this->live_target_mutex);
    this->live_target_buffer = *msg;
    this->live_target_updated = true;
  }
}

bool DeltaMotionPlanner::moveToConfiguration(const DeltaJoints& joints, bool require_task_mode) {
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (require_task_mode && this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "move_to_configuration rejected: not in TASK_MODE");
      return false;
    }
  }

  if (!this->tryAcquireMotionSlot("move_to_configuration")) {
    return false;
  }

  // Before publishing joint angles, ensure the request is valid using FK
  auto fk_request = std::make_shared<DeltaFK::Request>();
  fk_request->joint_angles = joints;

  auto future_result = this->delta_fk_client->async_send_request(
    fk_request,
    [this, joints](ServiceResponseFuture<DeltaFK> future) {
    auto response = future.get();
    if (response->success) {
      // If the FK solution is valid, move to the configuration
      std::vector<DeltaJoints> joint_traj = {joints};
      this->publishMotorCommands(joint_traj, 0);
    } else {
      RCLCPP_ERROR(get_logger(), "FK solution not found for the given joint angles");
    }
    this->releaseMotionSlot();
  }
  );

  (void)future_result;
  return true;
}

void DeltaMotionPlanner::moveThroughPoints(const std::vector<Point>& points) {
  // Plan a continuous trajectory through the given points using 3rd order polynomial interpolation
  (void)points;
}

bool DeltaMotionPlanner::playTrajectory(const std::vector<Point>& trajectory) {
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "Trajectory playback rejected: not in TASK_MODE");
      return false;
    }
  }

  if (!this->tryAcquireMotionSlot("trajectory playback")) {
    return false;
  }

  // Create a joint trajectory using the convert_to_joint_trajectory service
  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  // ---------- BEGIN_CITATION [1] ----------
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj](ServiceResponseFuture<ConvertToJointTrajectory> future) {
    auto response = future.get();
    if (response->joint_trajectory.empty()) {
      RCLCPP_ERROR(get_logger(), "Trajectory conversion failed");
      this->releaseMotionSlot();
      return;
    }
    *joint_traj = response->joint_trajectory;

    // Cancel existing trajectory if any
    this->cancel_current_traj = true;
    if (this->traj_thread && this->traj_thread->joinable()) {
      this->traj_thread->join();
    }
    this->cancel_current_traj = false;

    // Start new trajectory thread
    this->traj_thread = std::make_unique<std::thread>([this, joint_traj]() {
      // Reload parameter in case it changed
      this->param_traj_step_ms = this->get_parameter("traj_step_ms").as_int();
      this->publishMotorCommands(*joint_traj, this->param_traj_step_ms);
      this->releaseMotionSlot();
    });
  }
  );
  // ---------- END_CITATION [1] ----------

  (void)future_result;
  return true;
}

void DeltaMotionPlanner::playCustomTrajectory(
  const std::shared_ptr<PlayCustomTrajectory::Request> request,
  std::shared_ptr<PlayCustomTrajectory::Response> response) {
  if (request->trajectory.empty()) {
    RCLCPP_ERROR(get_logger(), "play_custom_trajectory received empty trajectory");
    response->success = false;
    return;
  }

  const int requested_step_ms = request->step_ms;
  const unsigned int step_ms = requested_step_ms > 0
    ? static_cast<unsigned int>(requested_step_ms)
    : static_cast<unsigned int>(this->get_parameter("traj_step_ms").as_int());

  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "Custom trajectory rejected: not in TASK_MODE");
      response->success = false;
      return;
    }
  }

  if (!this->tryAcquireMotionSlot("custom trajectory")) {
    response->success = false;
    return;
  }

  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = request->trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj, step_ms](ServiceResponseFuture<ConvertToJointTrajectory> future) {
      auto convert_response = future.get();
      if (convert_response->joint_trajectory.empty()) {
        RCLCPP_ERROR(get_logger(), "Custom trajectory conversion failed");
        this->releaseMotionSlot();
        return;
      }
      *joint_traj = convert_response->joint_trajectory;

      this->cancel_current_traj = true;
      if (this->traj_thread && this->traj_thread->joinable()) {
        this->traj_thread->join();
      }
      this->cancel_current_traj = false;

      this->traj_thread = std::make_unique<std::thread>([this, joint_traj, step_ms]() {
        this->publishMotorCommands(*joint_traj, step_ms);
        this->releaseMotionSlot();
      });
    }
  );
  (void)future_result;

  response->success = true;
}

void DeltaMotionPlanner::playDemoTrajectory(
  std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response) {

  std::string type = request->type.data;
  std::vector<Point> trajectory;
  const std::vector<std::string> available_demos = {"up_down", "pringle", "axes", "circle", "scan"};
  if (type == "up_down") {
    trajectory = this->straightUpDownTrajectory();
  } else if (type == "pringle") {
    trajectory = this->pringleTrajectory();
  } else if (type == "axes") {
    trajectory = this->axesTrajectory();
  } else if (type == "circle") {
    trajectory = this->circleTrajectory();
  } else if (type == "scan") {
    trajectory = this->scanTrajectory();
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid demo trajectory: %s", type.c_str());
    RCLCPP_ERROR(get_logger(), "Available demo trajectories: %s", std::accumulate(
      std::next(available_demos.begin()), available_demos.end(), available_demos[0],
      [](std::string a, std::string b) { return a + ", " + b; }
    ).c_str());
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Playing demo trajectory: %s", type.c_str());

  response->success = this->playTrajectory(trajectory);

  // Signal success
  if (!response->success) {
    RCLCPP_WARN(get_logger(), "Demo trajectory was rejected");
  }
}

std::vector<Point> DeltaMotionPlanner::scanTrajectory() {
  // Scan trajectory is saved in "scan_trajectory.csv" file
  return this->readCSV("scan_trajectory.csv");
}

std::vector<Point> DeltaMotionPlanner::straightUpDownTrajectory() {
  // Create a simple up down trajectory with 4 oscillations between
  // Z = -100 and Z = -200
  const int num_points = 300;
  std::vector<Point> trajectory;

  const float center = -180.0;
  const float amplitude = 75.0;
  const int cycles = 5;

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = 0.0;
    intermediate_pos.y = 0.0;
    intermediate_pos.z = center + amplitude * sin(2 * M_PI * cycles * t);
    trajectory.push_back(intermediate_pos);
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::pringleTrajectory() {
  // Circle Trajectory in XY plane while Z coordinate goes through 2 cycles of a sine wave
  const int num_points = 200;
  const float circle_center_z = -180.0;
  const float amplitude = 25.0;

  std::vector<float> t(num_points);
  float step = (2 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    t[i] = i * step;
  }

  std::vector<float> x_circle(num_points);
  std::vector<float> y_circle(num_points);
  std::vector<float> z_circle(num_points);
  for (int i = 0; i < num_points; ++i) {
    x_circle[i] = (2.0 * amplitude) * cos(t[i]);
    y_circle[i] = (2.0 * amplitude) * sin(t[i]);
    z_circle[i] = circle_center_z + amplitude * sin(2 * t[i]);
  }

  // Create trajectory
  std::vector<Point> trajectory(num_points);
  for (int i = 0; i < num_points; ++i) {
    trajectory[i].x = x_circle[i];
    trajectory[i].y = y_circle[i];
    trajectory[i].z = z_circle[i];
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::axesTrajectory() {
  // Trajectory showcasing the DOF of the DeltaRobot
  // Path will be a translation along X axis, then Y axis, then Z axis

  std::vector<Point> trajectory;

  const float x_start = 0.0;
  const float x_end = 60.0;
  const float y_start = 0.0;
  const float y_end = 60.0;
  const float z_start = -180.0;
  const float z_end = -240.0;
  const int num_points = 25;

  // X Axis Translation from (0, 0, -180) to (80, 0, -180)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start + t * (x_end - x_start);
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_end - t * (x_end - x_start);
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }

  // Y Axis translation from (0, 0, -180) to (0, 80, -180)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start + t * (y_end - y_start);
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_end - t * (y_end - y_start);
    intermediate_pos.z = z_start;
    trajectory.push_back(intermediate_pos);
  }

  // Z Axis translation from (0, 0, -180) to (0, 0, -220)
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_start + t * (z_end - z_start);
    trajectory.push_back(intermediate_pos);
  }
  // Go back to the starting point
  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / (num_points - 1);
    Point intermediate_pos;
    intermediate_pos.x = x_start;
    intermediate_pos.y = y_start;
    intermediate_pos.z = z_end - t * (z_end - z_start);
    trajectory.push_back(intermediate_pos);
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::circleTrajectory() {
  // Circle Trajectory in XY plane while Z coordinate remains constant
  const int num_points = 200;
  const float center_z = -180.0;
  const float radius = 40.0;

  std::vector<float> t(num_points);
  float step = (2 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    t[i] = i * step;
  }

  std::vector<float> x_circle(num_points);
  std::vector<float> y_circle(num_points);
  std::vector<float> z_circle(num_points);
  for (int i = 0; i < num_points; ++i) {
    x_circle[i] = radius * cos(t[i]);
    y_circle[i] = radius * sin(t[i]);
    z_circle[i] = center_z;
  }

  // Create trajectory
  std::vector<Point> trajectory(num_points);
  for (int i = 0; i < num_points; ++i) {
    trajectory[i].x = x_circle[i];
    trajectory[i].y = y_circle[i];
    trajectory[i].z = z_circle[i];
  }

  return trajectory;
}

std::vector<Point> DeltaMotionPlanner::randomSampleTrajectory(const int numPoints) {
  std::vector<Point> allPoints = this->readCSV("random_points.csv");
  if (allPoints.empty()) {  // ← add this
    RCLCPP_ERROR(get_logger(), "randomSampleTrajectory: no points loaded from CSV");
    return {};
  }
  std::vector<Point> sampledPoints;
  std::srand(std::time(0));
  for (int i = 0; i < numPoints; ++i) {
    int randomIndex = std::rand() % allPoints.size();
    sampledPoints.push_back(allPoints[randomIndex]);
  }
  return sampledPoints;
}

std::vector<Point> DeltaMotionPlanner::readCSV(const std::string& fileName) {
  const std::string user = std::getenv("USER");
  const std::string file_path = "/home/" + user + "/DeltaRobot/" + fileName;
  // The csv file has 3 columns: X, Y, Z
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
    return {};
  }
  std::vector<Point> trajectory;
  std::string line;
  bool first_line = true; // Skip the header
  while (std::getline(file, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }
    std::istringstream iss(line);
    std::string value;
    Point p;

    std::getline(iss, value, ',');
    p.x = std::stod(value);
    std::getline(iss, value, ',');
    p.y = std::stod(value);
    std::getline(iss, value, ',');
    p.z = std::stod(value);

    trajectory.push_back(p);
  }
  file.close();
  return trajectory;
}

void DeltaMotionPlanner::setMotionMode(
    const std::shared_ptr<SetMotionMode::Request> request,
    std::shared_ptr<SetMotionMode::Response> response) {
  
  // Validate mode value
  if (request->mode > static_cast<uint8_t>(LIVE_TEACH_MODE)) {
    response->success = false;
    response->message = "Invalid mode: must be 0 (TASK_MODE) or 1 (LIVE_TEACH_MODE)";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->motion_active.load()) {
      response->success = false;
      response->message = "Cannot switch modes while a motion is active";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    OperatingMode new_mode = static_cast<OperatingMode>(request->mode);
    
    // If switching from LIVE_TEACH to TASK, mark the buffer as not updated
    if (this->current_mode == LIVE_TEACH_MODE && new_mode == TASK_MODE) {
      this->live_target_updated = false;
    }

    if (new_mode == LIVE_TEACH_MODE) {
      this->playDemo = false;
    }
    
    this->current_mode = new_mode;
    
    const char* mode_name = (new_mode == TASK_MODE) ? "TASK_MODE" : "LIVE_TEACH_MODE";
    RCLCPP_INFO(get_logger(), "Motion mode switched to: %s", mode_name);
  }

  response->success = true;
  response->message = (request->mode == 0) ? "Switched to TASK_MODE" : "Switched to LIVE_TEACH_MODE";
}

void DeltaMotionPlanner::liveMotionController() {
  // Fixed-rate controller that processes live targets from the buffer
  // Only active when in LIVE_TEACH_MODE
  
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != LIVE_TEACH_MODE) {
      return;  // Only operate in LIVE_TEACH_MODE
    }
  }

  // Check if there's a new target to process
  bool has_target = false;
  Point target;
  {
    std::lock_guard<std::mutex> lock(this->live_target_mutex);
    if (this->live_target_updated) {
      has_target = true;
      target = this->live_target_buffer;
      this->live_target_updated = false;  // Mark as processed
    }
  }

  // If a new target was available, execute the motion immediately
  if (has_target) {
    // Call the internal motion path directly without waiting for the service callback.
    // This provides low-latency real-time motion for live teach mode.
    if (this->moveToPoint(target, false)) {
      std::lock_guard<std::mutex> lock(this->live_target_mutex);
      this->live_target_updated = false;
    }
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaMotionPlanner>());
  rclcpp::shutdown();
  return 0;
}