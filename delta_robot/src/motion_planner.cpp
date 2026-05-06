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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <ctime>
#include <chrono>

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

  this->current_commanded_pose.x = 0.0;
  this->current_commanded_pose.y = 0.0;
  this->current_commanded_pose.z = -300.0;
  this->current_commanded_tilt = 0.0;
  this->current_commanded_spin = 0.0;

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
  this->declare_parameter<std::string>("live_orientation_topic", "delta_motion_planner/live_orientation");
  this->live_orientation_topic = this->get_parameter("live_orientation_topic").as_string();
  this->declare_parameter<std::string>("commanded_tf_parent_frame", "world_link");
  this->commanded_tf_parent_frame = this->get_parameter("commanded_tf_parent_frame").as_string();
  this->declare_parameter<std::string>("commanded_tf_child_frame", "delta_robot/commanded_end_effector_pin");
  this->commanded_tf_child_frame = this->get_parameter("commanded_tf_child_frame").as_string();
  this->declare_parameter<std::string>("calculated_fk_tf_parent_frame", "world_link");
  this->calculated_fk_tf_parent_frame = this->get_parameter("calculated_fk_tf_parent_frame").as_string();
  this->declare_parameter<std::string>("calculated_fk_tf_child_frame", "delta_robot/calculated_fk_end_effector_pin");
  this->calculated_fk_tf_child_frame = this->get_parameter("calculated_fk_tf_child_frame").as_string();
  this->declare_parameter<std::string>("actual_fk_tf_parent_frame", "world_link");
  this->actual_fk_tf_parent_frame = this->get_parameter("actual_fk_tf_parent_frame").as_string();
  this->declare_parameter<std::string>("actual_fk_tf_child_frame", "delta_robot/actual_fk_end_effector_pin");
  this->actual_fk_tf_child_frame = this->get_parameter("actual_fk_tf_child_frame").as_string();
  this->declare_parameter("ee_to_tilt_axis_offset_m", 0.0);
  this->ee_to_tilt_axis_offset_m = this->get_parameter("ee_to_tilt_axis_offset_m").as_double();
  this->declare_parameter("tilt_axis_to_tool_tip_offset_m", 0.033);
  this->tilt_axis_to_tool_tip_offset_m = this->get_parameter("tilt_axis_to_tool_tip_offset_m").as_double();
  this->declare_parameter("tool_tip_to_object_center_offset_m", 0.0);
  this->tool_tip_to_object_center_offset_m =
    this->get_parameter("tool_tip_to_object_center_offset_m").as_double();
  this->declare_parameter("enable_tilt_axis_compensation", true);
  this->enable_tilt_axis_compensation =
    this->get_parameter("enable_tilt_axis_compensation").as_bool();
  this->parameters_callback_handle = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters) {
      return this->handleParameterUpdate(parameters);
    });
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
  this->commanded_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  this->calculated_fk_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  this->actual_fk_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(
    get_logger(),
    "Loaded offsets: ee_to_tilt_axis_offset_m=%.6f, tilt_axis_to_tool_tip_offset_m=%.6f, tool_tip_to_object_center_offset_m=%.6f, enable_tilt_axis_compensation=%s",
    this->ee_to_tilt_axis_offset_m,
    this->tilt_axis_to_tool_tip_offset_m,
    this->tool_tip_to_object_center_offset_m,
    this->enable_tilt_axis_compensation ? "true" : "false");

  this->live_target_sub = this->create_subscription<Point>(
    this->live_target_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const Point::SharedPtr msg) {
      this->liveTargetCallback(msg);
    });
  this->live_orientation_sub = this->create_subscription<Float64MultiArray>(
    this->live_orientation_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const Float64MultiArray::SharedPtr msg) {
      this->liveOrientationCallback(msg);
    });
  this->motor_feedback_sub = this->create_subscription<DeltaJoints>(
    "delta_motors/motor_position_feedback",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const DeltaJoints::SharedPtr msg) {
      this->publishActualFkTfFromJointFeedback(*msg);
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

      this->move_to_pose_server = create_service<MoveToPose>(
        "delta_motion_planner/move_to_pose",
        [this](const std::shared_ptr<MoveToPose::Request> request,
               std::shared_ptr<MoveToPose::Response> response) {
          response->success = this->moveToPose(
            request->target,
            request->tilt,
            request->spin,
            request->use_orientation,
            true);
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

      this->execute_trajectory_action_server = rclcpp_action::create_server<ExecuteTrajectory>(
        this,
        "delta_motion_planner/execute_trajectory",
        std::bind(&DeltaMotionPlanner::handleExecuteTrajectoryGoal, this,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(&DeltaMotionPlanner::handleExecuteTrajectoryCancel, this,
          std::placeholders::_1),
        std::bind(&DeltaMotionPlanner::handleExecuteTrajectoryAccepted, this,
          std::placeholders::_1));

      this->set_motion_mode_server = create_service<SetMotionMode>(
        "delta_motion_planner/set_motion_mode",
        std::bind(&DeltaMotionPlanner::setMotionMode, this,
          std::placeholders::_1, std::placeholders::_2));

      this->get_commanded_pose_service = create_service<GetCommandedPose>(
        "delta_motion_planner/get_commanded_pose",
        std::bind(&DeltaMotionPlanner::handleGetCommandedPose, this,
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
            static const std::vector<std::string> demo_names = {"pringle", "circle", "axes"};
            const auto& name = demo_names[this->demo_sequence_index % demo_names.size()];

            // Special case: random sample every 4th demo
            if (this->demo_sequence_index % 4 == 3) {
              RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: random sample");
              const std::string user = std::getenv("USER");
              const std::string file_path = "/home/" + user + "/DeltaRobot/random_points.csv";
              auto all_pts = delta_math::read_csv(file_path);
              if (!all_pts.empty()) {
                std::srand(std::time(0));
                auto sampled = delta_math::random_sample_trajectory(all_pts, 20);
                this->playTrajectory(toRosPoints(sampled));
              } else {
                RCLCPP_ERROR(get_logger(), "randomSampleTrajectory: no points loaded from CSV");
              }
            } else {
              RCLCPP_INFO(get_logger(), "Playing MSI Demo Trajectory: %s", name.c_str());
              auto traj = this->buildDemoTrajectory(name);
              if (!traj.empty()) {
                this->playTrajectory(traj);
              }
            }
            this->demo_sequence_index++;
          }
        });
    });
}

DeltaMotionPlanner::~DeltaMotionPlanner() {
  this->cancel_current_traj = true;
  if (this->traj_playback_timer) {
    this->traj_playback_timer->cancel();
  }
}

rcl_interfaces::msg::SetParametersResult DeltaMotionPlanner::handleParameterUpdate(
  const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(this->offset_mutex);
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "ee_to_tilt_axis_offset_m") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "ee_to_tilt_axis_offset_m must be a double";
        return result;
      }
      this->ee_to_tilt_axis_offset_m = parameter.as_double();
    } else if (parameter.get_name() == "tilt_axis_to_tool_tip_offset_m") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "tilt_axis_to_tool_tip_offset_m must be a double";
        return result;
      }
      this->tilt_axis_to_tool_tip_offset_m = parameter.as_double();
    } else if (parameter.get_name() == "tool_tip_to_object_center_offset_m") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "tool_tip_to_object_center_offset_m must be a double";
        return result;
      }
      this->tool_tip_to_object_center_offset_m = parameter.as_double();
    } else if (parameter.get_name() == "enable_tilt_axis_compensation") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enable_tilt_axis_compensation must be a bool";
        return result;
      }
      this->enable_tilt_axis_compensation = parameter.as_bool();
    }
  }

  return result;
}

void DeltaMotionPlanner::publishMotorCommands(
  const std::vector<DeltaJoints>& joint_traj,
  const unsigned int delay_ms,
  const double sim_tilt,
  const double sim_spin,
  const std::vector<Point>* ee_trajectory) {
  
  if (joint_traj.empty()) return;

  // Build a single JointTrajectory message with all points and cumulative time stamps
  // The joint_trajectory_controller rejects single-point messages whose timestamp has already passed
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = rclcpp::Time(0);  // 0 = "execute immediately" for joint_trajectory_controller
  traj_msg.joint_names = this->controller_joint_names;

  const unsigned int step_ms = (delay_ms > 0) ? delay_ms : static_cast<unsigned int>(this->param_traj_step_ms);
  const double sim_tilt_cmd = std::isfinite(sim_tilt) ? sim_tilt : 0.0;
  const double sim_spin_cmd = std::isfinite(sim_spin) ? sim_spin : 0.0;

  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    DeltaJoints command = joint_traj[i];
    if (!std::isfinite(command.theta1) || !std::isfinite(command.theta2) || !std::isfinite(command.theta3)) {
      continue;
    }
    if (!std::isfinite(command.theta4)) {
      command.theta4 = 0.0;
    }
    if (!std::isfinite(command.theta5)) {
      command.theta5 = 0.0;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
      command.theta1, command.theta2, command.theta3,
      command.theta4, command.theta5, sim_tilt_cmd, sim_spin_cmd
    };
    // Each point is step_ms further in the future
    const uint64_t t_ns = static_cast<uint64_t>(i + 1) * step_ms * 1000000ULL;
    point.time_from_start.sec = static_cast<int32_t>(t_ns / 1000000000ULL);
    point.time_from_start.nanosec = static_cast<uint32_t>(t_ns % 1000000000ULL);
    traj_msg.points.push_back(point);
  }

  // Publish the full trajectory once to Gazebo
  this->trajectory_pub->publish(traj_msg);

  // Setup state for timer-based physical motor command publishing
  this->active_joint_traj = joint_traj;
  if (ee_trajectory) {
    this->active_ee_traj = *ee_trajectory;
  } else {
    this->active_ee_traj.clear();
  }
  this->traj_playback_index = 0;
  this->traj_step_ms_active = step_ms;
  this->traj_sim_tilt = sim_tilt_cmd;
  this->traj_sim_spin = sim_spin_cmd;

  // Start or reset the timer
  if (this->traj_playback_timer) {
    this->traj_playback_timer->cancel();
  }
  this->traj_playback_timer = this->create_wall_timer(
    std::chrono::milliseconds(this->traj_step_ms_active),
    std::bind(&DeltaMotionPlanner::trajectoryPlaybackTick, this)
  );
  
  // Instantly fire the first point to avoid a 1-tick delay
  this->trajectoryPlaybackTick();
}

void DeltaMotionPlanner::trajectoryPlaybackTick() {
  if (this->cancel_current_traj || this->traj_playback_index >= this->active_joint_traj.size()) {
    if (this->traj_playback_timer) {
      this->traj_playback_timer->cancel();
    }
    this->releaseMotionSlot();
    return;
  }

  DeltaJoints command = this->active_joint_traj[this->traj_playback_index];
  if (std::isfinite(command.theta1) && std::isfinite(command.theta2) && std::isfinite(command.theta3)) {
    if (!std::isfinite(command.theta4)) command.theta4 = 0.0;
    if (!std::isfinite(command.theta5)) command.theta5 = 0.0;

    // Publish commanded target TF for this trajectory point if end-effector trajectory is available.
    if (this->traj_playback_index < this->active_ee_traj.size()) {
      const Point& ee_point = this->active_ee_traj[this->traj_playback_index];
      this->publishCommandedTargetTf(ee_point, this->traj_sim_tilt, this->traj_sim_spin);
      {
        std::lock_guard<std::mutex> lock(this->commanded_state_mutex);
        this->current_commanded_pose = ee_point;
        this->current_commanded_tilt = this->traj_sim_tilt;
        this->current_commanded_spin = this->traj_sim_spin;
      }
    }

    // Publish FK from commanded motor angles (mapped back into tool-point space).
    this->publishCalculatedFkTfFromJointCommand(command, this->traj_sim_tilt, this->traj_sim_spin);
    
    this->joint_pub->publish(command);
  }

  this->traj_playback_index++;
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
  // Backward-compatibility shim: 3DOF point command uses the same core path as pose commands.
  return this->moveToPose(point, 0.0, 0.0, false, require_task_mode);
}

bool DeltaMotionPlanner::moveToPose(
  const Point& point,
  double tilt,
  double spin,
  bool use_orientation,
  bool require_task_mode) {
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (require_task_mode && this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "move_to_pose rejected: not in TASK_MODE");
      return false;
    }
    if (!require_task_mode && this->current_mode != LIVE_TEACH_MODE) {
      RCLCPP_WARN(get_logger(), "live pose rejected: not in LIVE_TEACH_MODE");
      return false;
    }
  }

  if (!this->tryAcquireMotionSlot(require_task_mode ? "move_to_pose" : "live pose")) {
    return false;
  }

  const double tilt_cmd = use_orientation ? tilt : 0.0;
  const double spin_cmd = use_orientation ? spin : 0.0;
  this->publishCommandedTargetTf(point, tilt_cmd, spin_cmd);
  {
    std::lock_guard<std::mutex> lock(this->commanded_state_mutex);
    this->current_commanded_pose = point;
    this->current_commanded_tilt = tilt_cmd;
    this->current_commanded_spin = spin_cmd;
  }

  Point wrist = point;
  // Incoming Cartesian points are in millimeters; YAML offsets are configured in meters.
  double tool_offset_m = 0.0;
  double object_center_offset_m = 0.0;
  double axis_offset_m = 0.0;
  bool enable_axis_compensation = true;
  {
    std::lock_guard<std::mutex> lock(this->offset_mutex);
    tool_offset_m = this->tilt_axis_to_tool_tip_offset_m;
    object_center_offset_m = this->tool_tip_to_object_center_offset_m;
    axis_offset_m = this->ee_to_tilt_axis_offset_m;
    enable_axis_compensation = this->enable_tilt_axis_compensation;
  }
  const double tool_offset_mm = tool_offset_m * 1000.0;
  const double object_center_offset_mm = object_center_offset_m * 1000.0;
  const double axis_offset_mm = axis_offset_m * 1000.0;
  const double tool_plus_object_offset_mm = enable_axis_compensation
    ? (tool_offset_mm + object_center_offset_mm)
    : 0.0;
  // Apply wrist compensation for IK only. TF remains at the raw commanded point.
  wrist.x = point.x + (tool_plus_object_offset_mm * std::sin(tilt_cmd));
  wrist.y = point.y;
  if (enable_axis_compensation) {
    wrist.z = point.z - axis_offset_mm - (tool_plus_object_offset_mm * std::cos(tilt_cmd));
  } else {
    wrist.x = point.x;
    wrist.y = point.y;
    wrist.z = point.z - axis_offset_mm - (tool_offset_mm + object_center_offset_mm);
  }

  auto ik_request = std::make_shared<DeltaIK::Request>();
  ik_request->solution = wrist;

  auto future_result = this->delta_ik_client->async_send_request(
    ik_request,
    [this, tilt_cmd, spin_cmd](ServiceResponseFuture<DeltaIK> future) {
      auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(get_logger(), "IK solution not found for requested pose");
        this->releaseMotionSlot();
        return;
      }

      DeltaJoints command = response->joint_angles;
      command.theta4 = tilt_cmd + (2.0 * spin_cmd);
      command.theta5 = (2.0 * spin_cmd) - tilt_cmd;
      if (!std::isfinite(command.theta4) || !std::isfinite(command.theta5)) {
        RCLCPP_ERROR(get_logger(), "Differential mapping produced invalid theta4/theta5");
        this->releaseMotionSlot();
        return;
      }

      std::vector<DeltaJoints> joint_traj = {command};
      this->publishMotorCommands(joint_traj, 0, tilt_cmd, spin_cmd);
      this->releaseMotionSlot();
    });

  (void)future_result;
  return true;
}

void DeltaMotionPlanner::publishCommandedTargetTf(const Point& point_mm, double tilt_rad, double spin_rad) {
  this->publishTfStream(
    this->commanded_tf_broadcaster.get(),
    this->commanded_tf_parent_frame,
    this->commanded_tf_child_frame,
    point_mm,
    tilt_rad,
    spin_rad);
}

void DeltaMotionPlanner::publishTfStream(
  tf2_ros::TransformBroadcaster* broadcaster,
  const std::string& parent_frame,
  const std::string& child_frame,
  const Point& point_mm,
  double tilt_rad,
  double spin_rad) {
  if (broadcaster == nullptr) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;
  tf_msg.transform.translation.x = point_mm.x / 1000.0;
  tf_msg.transform.translation.y = point_mm.y / 1000.0;
  tf_msg.transform.translation.z = point_mm.z / 1000.0;

  double tool_offset_m = 0.0;
  double object_center_offset_m = 0.0;
  double axis_offset_m = 0.0;
  bool enable_axis_compensation = true;
  {
    std::lock_guard<std::mutex> lock(this->offset_mutex);
    tool_offset_m = this->tilt_axis_to_tool_tip_offset_m;
    object_center_offset_m = this->tool_tip_to_object_center_offset_m;
    axis_offset_m = this->ee_to_tilt_axis_offset_m;
    enable_axis_compensation = this->enable_tilt_axis_compensation;
  }

  const double total_offset_m = tool_offset_m + object_center_offset_m;
  if (!enable_axis_compensation) {
    // Project tool-axis offset into world XYZ using the commanded tilt/spin.
    const double offset_x = total_offset_m * std::sin(tilt_rad);
    const double offset_z = total_offset_m* std::cos(tilt_rad) - axis_offset_m -0.00513;
    tf_msg.transform.translation.x -= offset_x;
    tf_msg.transform.translation.z += offset_z;
  }

  tf2::Quaternion wrist_orientation;
  // Tilt is about Y and spin is about Z in the current wrist model.
  wrist_orientation.setRPY(0.0, tilt_rad, spin_rad);

  // Apply a fixed visualization-frame correction requested by the user:
  // rotate Z by +90 deg, then Y by +180 deg. This only changes orientation.
  constexpr double kPi = 3.14159265358979323846;
  tf2::Quaternion rot_z_180;
  rot_z_180.setRPY(kPi, 0.0, 0.0);

  // Fixed frame correction should be applied in parent/world frame space.
  tf2::Quaternion corrected_orientation = rot_z_180 * wrist_orientation;
  corrected_orientation.normalize();
  tf_msg.transform.rotation = tf2::toMsg(corrected_orientation);

  broadcaster->sendTransform(tf_msg);
}

Point DeltaMotionPlanner::convertWristToToolPoint(
  const Point& wrist_point_mm,
  double tilt_rad,
  double spin_rad) {
  double tool_offset_m = 0.0;
  double object_center_offset_m = 0.0;
  double axis_offset_m = 0.0;
  bool enable_axis_compensation = true;
  {
    std::lock_guard<std::mutex> lock(this->offset_mutex);
    tool_offset_m = this->tilt_axis_to_tool_tip_offset_m;
    object_center_offset_m = this->tool_tip_to_object_center_offset_m;
    axis_offset_m = this->ee_to_tilt_axis_offset_m;
    enable_axis_compensation = this->enable_tilt_axis_compensation;
  }

  const double tool_offset_mm = tool_offset_m * 1000.0;
  const double object_center_offset_mm = object_center_offset_m * 1000.0;
  const double axis_offset_mm = axis_offset_m * 1000.0;
  const double tool_plus_object_offset_mm = tool_offset_mm + object_center_offset_mm;

  Point tool_point = wrist_point_mm;
  if (enable_axis_compensation) {
    tool_point.x = wrist_point_mm.x -
      (tool_plus_object_offset_mm * std::sin(tilt_rad) * std::cos(spin_rad));
    tool_point.y = wrist_point_mm.y -
      (tool_plus_object_offset_mm * std::sin(tilt_rad) * std::sin(spin_rad));
    tool_point.z = wrist_point_mm.z + axis_offset_mm + (tool_plus_object_offset_mm * std::cos(tilt_rad));
  } else {
    tool_point.x = wrist_point_mm.x;
    tool_point.y = wrist_point_mm.y;
    tool_point.z = wrist_point_mm.z + axis_offset_mm + tool_plus_object_offset_mm;
  }

  return tool_point;
}

void DeltaMotionPlanner::publishCalculatedFkTf(const Point& point_mm, double tilt_rad, double spin_rad) {
  this->publishTfStream(
    this->calculated_fk_tf_broadcaster.get(),
    this->calculated_fk_tf_parent_frame,
    this->calculated_fk_tf_child_frame,
    point_mm,
    tilt_rad,
    spin_rad);
}

void DeltaMotionPlanner::publishCalculatedFkTfFromJointCommand(
  const DeltaJoints& joints,
  double tilt_rad,
  double spin_rad) {
  if (!this->delta_fk_client || !this->delta_fk_client->service_is_ready()) {
    return;
  }

  auto fk_request = std::make_shared<DeltaFK::Request>();
  fk_request->joint_angles = joints;

  auto future_result = this->delta_fk_client->async_send_request(
    fk_request,
    [this, tilt_rad, spin_rad](ServiceResponseFuture<DeltaFK> future) {
      auto response = future.get();
      if (!response->success) {
        return;
      }

      const Point tool_point = this->convertWristToToolPoint(response->solution, tilt_rad, spin_rad);
      this->publishCalculatedFkTf(tool_point, tilt_rad, spin_rad);
    });
  (void)future_result;
}

void DeltaMotionPlanner::publishActualFkTf(const Point& point_mm, double tilt_rad, double spin_rad) {
  this->publishTfStream(
    this->actual_fk_tf_broadcaster.get(),
    this->actual_fk_tf_parent_frame,
    this->actual_fk_tf_child_frame,
    point_mm,
    tilt_rad,
    spin_rad);
}

void DeltaMotionPlanner::publishActualFkTfFromJointFeedback(const DeltaJoints& joints) {
  if (!this->delta_fk_client || !this->delta_fk_client->service_is_ready()) {
    return;
  }
  if (!std::isfinite(joints.theta1) || !std::isfinite(joints.theta2) || !std::isfinite(joints.theta3)) {
    return;
  }

  // Threshold for zeroing out very small angles (noise elimination)
  const double ANGLE_NOISE_THRESHOLD = 1e-4;

  // Create a copy of joints and zero out very small angles to avoid numerical noise
  DeltaJoints cleaned_joints = joints;
  if (std::abs(cleaned_joints.theta1) < ANGLE_NOISE_THRESHOLD) {
    cleaned_joints.theta1 = 0.0;
  }
  if (std::abs(cleaned_joints.theta2) < ANGLE_NOISE_THRESHOLD) {
    cleaned_joints.theta2 = 0.0;
  }
  if (std::abs(cleaned_joints.theta3) < ANGLE_NOISE_THRESHOLD) {
    cleaned_joints.theta3 = 0.0;
  }
  if (std::abs(cleaned_joints.theta4) < ANGLE_NOISE_THRESHOLD) {
    cleaned_joints.theta4 = 0.0;
  }
  if (std::abs(cleaned_joints.theta5) < ANGLE_NOISE_THRESHOLD) {
    cleaned_joints.theta5 = 0.0;
  }

  // Differential inverse mapping from feedback joints:
  // theta4 = tilt + 2*spin, theta5 = 2*spin - tilt
  double tilt_rad = 0.0;
  double spin_rad = 0.0;
  if (std::isfinite(cleaned_joints.theta4) && std::isfinite(cleaned_joints.theta5)) {
    tilt_rad = 0.5 * (cleaned_joints.theta4 - cleaned_joints.theta5);
    spin_rad = 0.25 * (cleaned_joints.theta4 + cleaned_joints.theta5);
  }

  auto fk_request = std::make_shared<DeltaFK::Request>();
  fk_request->joint_angles = cleaned_joints;

  auto future_result = this->delta_fk_client->async_send_request(
    fk_request,
    [this, tilt_rad, spin_rad](ServiceResponseFuture<DeltaFK> future) {
      auto response = future.get();
      if (!response->success) {
        return;
      }

      const Point tool_point = this->convertWristToToolPoint(response->solution, tilt_rad, spin_rad);
      this->publishActualFkTf(tool_point, tilt_rad, spin_rad);
    });
  (void)future_result;
}

void DeltaMotionPlanner::handleGetCommandedPose(
  const std::shared_ptr<GetCommandedPose::Request>,
  std::shared_ptr<GetCommandedPose::Response> response) {
  std::lock_guard<std::mutex> lock(this->commanded_state_mutex);
  response->pose = this->current_commanded_pose;
  response->tilt = this->current_commanded_tilt;
  response->spin = this->current_commanded_spin;
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

void DeltaMotionPlanner::liveOrientationCallback(const Float64MultiArray::SharedPtr msg) {
  if (!this->initialized) {
    return;
  }
  if (msg->data.size() < 2) {
    RCLCPP_WARN(get_logger(), "live_orientation message requires [tilt, spin]");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != LIVE_TEACH_MODE) {
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->live_orientation_mutex);
    this->live_tilt_buffer = msg->data[0];
    this->live_spin_buffer = msg->data[1];
    this->live_orientation_updated = true;
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
      double tilt_cmd = 0.0;
      double spin_cmd = 0.0;
      if (std::isfinite(joints.theta4) && std::isfinite(joints.theta5)) {
        tilt_cmd = 0.5 * (joints.theta4 - joints.theta5);
        spin_cmd = 0.25 * (joints.theta4 + joints.theta5);
      }
      const Point tool_point = this->convertWristToToolPoint(response->solution, tilt_cmd, spin_cmd);
      {
        std::lock_guard<std::mutex> lock(this->commanded_state_mutex);
        this->current_commanded_pose = tool_point;
        this->current_commanded_tilt = tilt_cmd;
        this->current_commanded_spin = spin_cmd;
      }
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

std::vector<Point> DeltaMotionPlanner::applyZOffsetToTrajectory(const std::vector<Point>& trajectory) {
  // Apply z-offset adjustment so that when IK solves for the wrist position,
  // the end-effector will actually be at the commanded position.
  std::vector<Point> adjusted_trajectory = trajectory;
  
  double tool_offset_m = 0.0;
  double object_center_offset_m = 0.0;
  double axis_offset_m = 0.0;
  bool enable_axis_compensation = true;
  {
    std::lock_guard<std::mutex> lock(this->offset_mutex);
    tool_offset_m = this->tilt_axis_to_tool_tip_offset_m;
    object_center_offset_m = this->tool_tip_to_object_center_offset_m;
    axis_offset_m = this->ee_to_tilt_axis_offset_m;
    enable_axis_compensation = this->enable_tilt_axis_compensation;
  }
  
  if (enable_axis_compensation) {
    const double total_offset_mm = (tool_offset_m + object_center_offset_m + axis_offset_m) * 1000.0;
    for (auto& point : adjusted_trajectory) {
      point.z -= total_offset_mm;
    }
  }
  
  return adjusted_trajectory;
}

std::vector<Point> DeltaMotionPlanner::prependApproachSegment(
  const std::vector<Point>& trajectory,
  unsigned int step_ms) {
  if (trajectory.empty() || step_ms == 0) {
    return trajectory;
  }

  Point start_pose;
  {
    std::lock_guard<std::mutex> lock(this->commanded_state_mutex);
    start_pose = this->current_commanded_pose;
  }

  const Point& first = trajectory.front();
  constexpr double kMinApproachDistanceMm = 0.1;
  const double step_s = static_cast<double>(step_ms) / 1000.0;

  const double dx_start = first.x - start_pose.x;
  const double dy_start = first.y - start_pose.y;
  const double dz_start = first.z - start_pose.z;
  const double start_distance = std::sqrt(dx_start * dx_start + dy_start * dy_start + dz_start * dz_start);

  if (start_distance <= kMinApproachDistanceMm) {
    return trajectory;
  }

  double approach_speed = 0.0;
  if (trajectory.size() >= 2) {
    const Point& second = trajectory[1];
    const double dx_seg = second.x - first.x;
    const double dy_seg = second.y - first.y;
    const double dz_seg = second.z - first.z;
    const double seg_distance = std::sqrt(dx_seg * dx_seg + dy_seg * dy_seg + dz_seg * dz_seg);
    approach_speed = (step_s > 0.0) ? (seg_distance / step_s) : 0.0;
  }

  constexpr double kFallbackSpeedMmS = 100.0;
  if (!(approach_speed > 0.0)) {
    approach_speed = kFallbackSpeedMmS;
  }

  const double approach_duration_s = start_distance / approach_speed;
  const unsigned int steps = std::max(1u, static_cast<unsigned int>(std::ceil(approach_duration_s / step_s)));

  std::vector<Point> with_approach;
  with_approach.reserve(steps + trajectory.size());

  for (unsigned int step = 1; step <= steps; ++step) {
    const double alpha = static_cast<double>(step) / static_cast<double>(steps);
    Point approach_point;
    approach_point.x = start_pose.x + dx_start * alpha;
    approach_point.y = start_pose.y + dy_start * alpha;
    approach_point.z = start_pose.z + dz_start * alpha;
    with_approach.push_back(approach_point);
  }

  if (trajectory.size() >= 2) {
    with_approach.insert(with_approach.end(), std::next(trajectory.begin()), trajectory.end());
  }
  return with_approach;
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

  // Apply approach so we do not teleport to the first point.
  const unsigned int step_ms = static_cast<unsigned int>(this->get_parameter("traj_step_ms").as_int());
  const auto approach_trajectory = this->prependApproachSegment(trajectory, step_ms);

  // Apply z-offset adjustment so the end-effector reaches the commanded position
  const auto adjusted_trajectory = this->applyZOffsetToTrajectory(approach_trajectory);

  // Create a joint trajectory using the convert_to_joint_trajectory service
  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = adjusted_trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  auto ee_trajectory = std::make_shared<std::vector<Point>>(approach_trajectory);
  // ---------- BEGIN_CITATION [1] ----------
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj, ee_trajectory](ServiceResponseFuture<ConvertToJointTrajectory> future) {
    auto response = future.get();
    if (response->joint_trajectory.empty()) {
      RCLCPP_ERROR(get_logger(), "Trajectory conversion failed");
      this->releaseMotionSlot();
      return;
    }
    *joint_traj = response->joint_trajectory;

    // Cancel existing trajectory if any
    this->cancel_current_traj = true;
    if (this->traj_playback_timer) {
      this->traj_playback_timer->cancel();
    }
    this->cancel_current_traj = false;

    // Reload parameter in case it changed
    this->param_traj_step_ms = this->get_parameter("traj_step_ms").as_int();
    this->publishMotorCommands(*joint_traj, this->param_traj_step_ms, 0.0, 0.0, &(*ee_trajectory));
  }
  );
  // ---------- END_CITATION [1] ----------

  (void)future_result;
  return true;
}

void DeltaMotionPlanner::playCustomTrajectory(
  const std::shared_ptr<PlayCustomTrajectory::Request> request,
  std::shared_ptr<PlayCustomTrajectory::Response> response) {
  const int requested_step_ms = request->step_ms;
  const unsigned int step_ms = requested_step_ms > 0
    ? static_cast<unsigned int>(requested_step_ms)
    : static_cast<unsigned int>(this->get_parameter("traj_step_ms").as_int());
  response->success = this->queueCustomTrajectory(request->trajectory, step_ms);
  if (!response->success) {
    RCLCPP_WARN(get_logger(), "Custom trajectory rejected");
  }
}

bool DeltaMotionPlanner::queueCustomTrajectory(
  const std::vector<Point>& trajectory,
  unsigned int step_ms) {
  if (trajectory.empty()) {
    RCLCPP_ERROR(get_logger(), "play_custom_trajectory received empty trajectory");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "Custom trajectory rejected: not in TASK_MODE");
      return false;
    }
  }

  if (!this->tryAcquireMotionSlot("custom trajectory")) {
    return false;
  }

  // Apply approach so we do not teleport to the first point.
  const auto approach_trajectory = this->prependApproachSegment(trajectory, step_ms);
  const auto adjusted_trajectory = this->applyZOffsetToTrajectory(approach_trajectory);

  auto convert_request = std::make_shared<ConvertToJointTrajectory::Request>();
  convert_request->end_effector_trajectory = adjusted_trajectory;

  auto joint_traj = std::make_shared<std::vector<DeltaJoints>>();
  auto ee_trajectory = std::make_shared<std::vector<Point>>(approach_trajectory);
  auto future_result = this->convert_to_joint_trajectory_client->async_send_request(
    convert_request,
    [this, joint_traj, ee_trajectory, step_ms](ServiceResponseFuture<ConvertToJointTrajectory> future) {
      auto convert_response = future.get();
      if (convert_response->joint_trajectory.empty()) {
        RCLCPP_ERROR(get_logger(), "Custom trajectory conversion failed");
        this->releaseMotionSlot();
        return;
      }
      *joint_traj = convert_response->joint_trajectory;

      this->cancel_current_traj = true;
      if (this->traj_playback_timer) {
        this->traj_playback_timer->cancel();
      }
      this->cancel_current_traj = false;

      this->publishMotorCommands(*joint_traj, step_ms, 0.0, 0.0, &(*ee_trajectory));
    }
  );
  (void)future_result;

  return true;
}

double DeltaMotionPlanner::estimateTrajectoryDurationSec(
  std::size_t points,
  unsigned int step_ms) const {
  if (points == 0) {
    return 0.0;
  }
  const double step_s = static_cast<double>(step_ms) / 1000.0;
  return step_s * static_cast<double>(points);
}

rclcpp_action::GoalResponse DeltaMotionPlanner::handleExecuteTrajectoryGoal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const ExecuteTrajectory::Goal> goal) {
  if (goal->trajectory.empty()) {
    RCLCPP_WARN(get_logger(), "ExecuteTrajectory rejected: empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(this->mode_mutex);
    if (this->current_mode != TASK_MODE) {
      RCLCPP_WARN(get_logger(), "ExecuteTrajectory rejected: not in TASK_MODE");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeltaMotionPlanner::handleExecuteTrajectoryCancel(
  const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
  (void)goal_handle;
  this->cancel_current_traj = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DeltaMotionPlanner::handleExecuteTrajectoryAccepted(
  const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    const int requested_step_ms = goal->step_ms;
    const unsigned int step_ms = requested_step_ms > 0
      ? static_cast<unsigned int>(requested_step_ms)
      : static_cast<unsigned int>(this->get_parameter("traj_step_ms").as_int());

    if (!this->queueCustomTrajectory(goal->trajectory, step_ms)) {
      auto result = std::make_shared<ExecuteTrajectory::Result>();
      result->success = false;
      result->message = "trajectory rejected";
      goal_handle->abort(result);
      return;
    }

    const std::size_t total_points = goal->trajectory.size();
    const double total_duration_s = this->estimateTrajectoryDurationSec(total_points, step_ms);
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        this->cancel_current_traj = true;
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = false;
        result->message = "canceled";
        goal_handle->canceled(result);
        return;
      }

      auto now = std::chrono::steady_clock::now();
      const double elapsed_s = std::chrono::duration<double>(now - start).count();
      float progress = 0.0f;
      if (total_duration_s > 0.0) {
        progress = static_cast<float>(std::min(elapsed_s / total_duration_s, 1.0));
      }

      auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
      feedback->total_points = static_cast<uint32_t>(total_points);
      feedback->current_index = static_cast<uint32_t>(progress * total_points);
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);

      if (elapsed_s >= total_duration_s) {
        break;
      }
      rate.sleep();
    }

    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = true;
    result->message = "completed";
    goal_handle->succeed(result);
  }).detach();
}

void DeltaMotionPlanner::playDemoTrajectory(
  std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response) {

  std::string type = request->type.data;
  std::vector<Point> trajectory = this->buildDemoTrajectory(type);

  if (trajectory.empty()) {
    const std::vector<std::string> available_demos = {"up_down", "pringle", "axes", "circle", "scan"};
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

std::vector<Point> DeltaMotionPlanner::toRosPoints(const std::vector<delta_math::Point3D>& pts) {
  std::vector<Point> ros_pts;
  ros_pts.reserve(pts.size());
  for (const auto& p : pts) {
    Point rp;
    rp.x = p.x;
    rp.y = p.y;
    rp.z = p.z;
    ros_pts.push_back(rp);
  }
  return ros_pts;
}

std::vector<Point> DeltaMotionPlanner::buildDemoTrajectory(const std::string& name) {
  if (name == "up_down") {
    return toRosPoints(delta_math::straight_up_down_trajectory());
  } else if (name == "pringle") {
    return toRosPoints(delta_math::pringle_trajectory());
  } else if (name == "axes") {
    return toRosPoints(delta_math::axes_trajectory());
  } else if (name == "circle") {
    return toRosPoints(delta_math::circle_trajectory());
  } else if (name == "scan") {
    const std::string user = std::getenv("USER");
    const std::string file_path = "/home/" + user + "/DeltaRobot/scan_trajectory.csv";
    return toRosPoints(delta_math::read_csv(file_path));
  }
  return {};
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
      this->live_orientation_updated = false;
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
    bool use_orientation = false;
    double tilt = 0.0;
    double spin = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->live_orientation_mutex);
      if (this->live_orientation_updated) {
        use_orientation = true;
        tilt = this->live_tilt_buffer;
        spin = this->live_spin_buffer;
      }
    }

    // Call the internal motion path directly without waiting for the service callback.
    // This provides low-latency real-time motion for live teach mode.
    if (this->moveToPose(target, tilt, spin, use_orientation, false)) {
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