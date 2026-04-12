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
  this->declare_parameter<std::string>("live_orientation_topic", "delta_motion_planner/live_orientation");
  this->live_orientation_topic = this->get_parameter("live_orientation_topic").as_string();
  this->declare_parameter<std::string>("commanded_tf_parent_frame", "delta_robot/frame");
  this->commanded_tf_parent_frame = this->get_parameter("commanded_tf_parent_frame").as_string();
  this->declare_parameter<std::string>("commanded_tf_child_frame", "delta_robot/commanded_end_effector_pin");
  this->commanded_tf_child_frame = this->get_parameter("commanded_tf_child_frame").as_string();
  this->declare_parameter<std::string>("calculated_fk_tf_parent_frame", "delta_robot/frame");
  this->calculated_fk_tf_parent_frame = this->get_parameter("calculated_fk_tf_parent_frame").as_string();
  this->declare_parameter<std::string>("calculated_fk_tf_child_frame", "delta_robot/calculated_fk_end_effector_pin");
  this->calculated_fk_tf_child_frame = this->get_parameter("calculated_fk_tf_child_frame").as_string();
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
  // Build a single JointTrajectory message with all points and cumulative time stamps
  // The joint_trajectory_controller rejects single-point messages whose timestamp has already passed
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = rclcpp::Time(0);  // 0 = "execute immediately" for joint_trajectory_controller
  traj_msg.joint_names = this->controller_joint_names;

  const unsigned int step_ms = (delay_ms > 0) ? delay_ms : static_cast<unsigned int>(this->param_traj_step_ms);
  const double sim_tilt_cmd = std::isfinite(sim_tilt) ? sim_tilt : 0.0;
  const double sim_spin_cmd = std::isfinite(sim_spin) ? sim_spin : 0.0;

  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    if (this->cancel_current_traj) break;
    DeltaJoints command = joint_traj[i];
    if (!std::isfinite(command.theta1) || !std::isfinite(command.theta2) || !std::isfinite(command.theta3)) {
      RCLCPP_WARN(get_logger(), "Skipping joint command with invalid theta1-3 values");
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

  // Publish the joint commands to the physical motors with a delay per point
  for (unsigned int i = 0; i < joint_traj.size(); i++) {
    if (this->cancel_current_traj) break;
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
    
    // Publish commanded target TF for this trajectory point if end-effector trajectory is available.
    if (ee_trajectory && i < ee_trajectory->size()) {
      const Point& ee_point = (*ee_trajectory)[i];
      this->publishCommandedTargetTf(ee_point, sim_tilt_cmd, sim_spin_cmd);
    }

    // Publish FK from commanded motor angles (mapped back into tool-point space).
    this->publishCalculatedFkTfFromJointCommand(command, sim_tilt_cmd, sim_spin_cmd);
    
    this->joint_pub->publish(command);
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

  // Use the commanded Cartesian point directly for IK. Applying tool/axis offsets
  // here shifts the physical robot target (not just TF visualization), which can
  // introduce an apparent extra Z offset.
  Point wrist = point;

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

  tf2::Quaternion wrist_orientation;
  // Tilt is about Y and spin is about Z in the current wrist model.
  wrist_orientation.setRPY(0.0, tilt_rad, spin_rad);

  // Apply a fixed visualization-frame correction requested by the user:
  // rotate Z by +90 deg, then Y by +180 deg. This only changes orientation.
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kHalfPi = 0.5 * kPi;
  tf2::Quaternion rot_z_90;
  rot_z_90.setRPY(0.0, 0.0, kHalfPi);
  tf2::Quaternion rot_y_180;
  rot_y_180.setRPY(0.0, kPi, 0.0);

  // Fixed frame correction should be applied in parent/world frame space.
  tf2::Quaternion corrected_orientation = rot_y_180 * rot_z_90 * wrist_orientation;
  corrected_orientation.normalize();
  tf_msg.transform.rotation = tf2::toMsg(corrected_orientation);

  broadcaster->sendTransform(tf_msg);
}

Point DeltaMotionPlanner::convertWristToToolPoint(const Point& wrist_point_mm, double tilt_rad) {
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
    tool_point.x = wrist_point_mm.x - (tool_plus_object_offset_mm * std::sin(tilt_rad));
    tool_point.z = wrist_point_mm.z + axis_offset_mm + (tool_plus_object_offset_mm * std::cos(tilt_rad));
  } else {
    tool_point.x = wrist_point_mm.x;
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

      const Point tool_point = this->convertWristToToolPoint(response->solution, tilt_rad);
      this->publishCalculatedFkTf(tool_point, tilt_rad, spin_rad);
    });
  (void)future_result;
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
  auto ee_trajectory = std::make_shared<std::vector<Point>>(trajectory);
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
    if (this->traj_thread && this->traj_thread->joinable()) {
      this->traj_thread->join();
    }
    this->cancel_current_traj = false;

    // Start new trajectory thread
    this->traj_thread = std::make_unique<std::thread>([this, joint_traj, ee_trajectory]() {
      // Reload parameter in case it changed
      this->param_traj_step_ms = this->get_parameter("traj_step_ms").as_int();
      this->publishMotorCommands(*joint_traj, this->param_traj_step_ms, 0.0, 0.0, &(*ee_trajectory));
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
  auto ee_trajectory = std::make_shared<std::vector<Point>>(request->trajectory);
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
      if (this->traj_thread && this->traj_thread->joinable()) {
        this->traj_thread->join();
      }
      this->cancel_current_traj = false;

      this->traj_thread = std::make_unique<std::thread>([this, joint_traj, ee_trajectory, step_ms]() {
        this->publishMotorCommands(*joint_traj, step_ms, 0.0, 0.0, &(*ee_trajectory));
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