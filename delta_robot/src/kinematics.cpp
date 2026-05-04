/// @file kinematics.cpp
/// @brief Kinematics Implementation for Delta Robot
///
/// PARAMETERS:
///   base_triangle_side_length (float64): The side length of the equilateral triangle defining the base [mm]
///   active_link_length (float64): The center distance (joint to joint) of the actuated links [mm]
///   passive_link_length (float64): The center distance (joint to joint) of the passive links [mm]
///   passive_link_width (float64): The width of the passive links [mm]
///   end_effector_side_length (float64): The side length of the equilateral triangle defining the end effector [mm]
///   joint_min (float64): The minimum joint angle [rad]
///   joint_max (float64): The maximum joint angle [rad]
///   max_joint_velocity (float64): The maximum joint velocity [rad/s]
///   robot_config_freq (float64): The frequency at which the robot configuration is published [Hz]
///
/// PUBLISHERS:
///   ~/delta_robot/robot_config (deltarobot_interfaces::msg::RobotConfig): Publishes the current robot configuration including joint angles/velocities and end effector position
///
/// SUBSCRIBERS:
///   ~/delta_motors/motor_position_feedback (deltarobot_interfaces::msg::DeltaJoints): Subscribes to the current joint angles of the delta robot
///   ~/delta_motors/motor_velocity_feedback (deltarobot_interfaces::msg::DeltaJointVels): Subscribes to the current joint velocities of the delta robot
///
/// SERVICES:
///   ~/delta_fk (deltarobot_interfaces::srv::DeltaFK): Computes the end effector position given the joint angles (forward kinematics)
///   ~/delta_ik (deltarobot_interfaces::srv::DeltaIK): Computes the joint angles given the end effector position (inverse kinematics)
///   ~/convert_to_joint_trajectory (deltarobot_interfaces::srv::ConvertToJointTrajectory): Converts an end effector trajectory to a joint trajectory
///   ~/convert_to_joint_vel_trajectory (deltarobot_interfaces::srv::ConvertToJointVelTrajectory): Converts an end effector velocity trajectory to a joint velocity trajectory
///
/// CLIENTS:
///  ~/delta_motors/set_joint_limits (deltarobot_interfaces::srv::SetJointLimits): Client to set the joint limits for the delta robot motors


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "kinematics.hpp"
#include <vector>
#include <limits>

template<typename T>
using ServiceResponseFuture = typename rclcpp::Client<T>::SharedFuture;

DeltaKinematics::DeltaKinematics() : Node("delta_kinematics") {
  RCLCPP_INFO(this->get_logger(), "DeltaKinematics Started");

  // Declare parameters
  this->declare_parameter("base_triangle_side_length", 200.0);
  this->declare_parameter("end_effector_side_length", 100.0);
  this->declare_parameter("active_link_length", 100.0);
  this->declare_parameter("passive_link_length", 100.0);
  this->declare_parameter("passive_link_width", 30.0);
  this->declare_parameter("joint_min", 0.0);
  this->declare_parameter("joint_max", M_PI / 2.0);
  this->declare_parameter("max_joint_velocity", 1.0);
  this->declare_parameter("robot_config_freq", 10.0);

  // Populate math_params from YAML
  this->math_params.base_side     = this->get_parameter("base_triangle_side_length").as_double();
  this->math_params.ee_side       = this->get_parameter("end_effector_side_length").as_double();
  this->math_params.active_link   = this->get_parameter("active_link_length").as_double();
  this->math_params.passive_link  = this->get_parameter("passive_link_length").as_double();
  this->math_params.passive_width = this->get_parameter("passive_link_width").as_double();
  this->math_params.joint_min     = this->get_parameter("joint_min").as_double();
  this->math_params.joint_max     = this->get_parameter("joint_max").as_double();
  this->math_params.max_joint_vel = this->get_parameter("max_joint_velocity").as_double();
  const double robot_config_freq  = this->get_parameter("robot_config_freq").as_double();

  // Initialize robot_state with safe 45 degree angles to prevent FK failure on startup
  this->robot_state.theta1 = 0.785;
  this->robot_state.theta2 = 0.785;
  this->robot_state.theta3 = 0.785;
  this->robot_state.theta1_vel = 0.0;
  this->robot_state.theta2_vel = 0.0;
  this->robot_state.theta3_vel = 0.0;

  // Create FK and IK servers
  this->delta_fk_server = create_service<DeltaFK>(
    "delta_kinematics/delta_fk",
    std::bind(&DeltaKinematics::forwardKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );

  this->delta_ik_server = create_service<DeltaIK>(
    "delta_kinematics/delta_ik",
    std::bind(&DeltaKinematics::inverseKinematics, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create ConvertToJointTrajectory server
  this->convert_to_joint_trajectory_server = create_service<ConvertToJointTrajectory>(
    "delta_kinematics/convert_to_joint_trajectory",
    std::bind(&DeltaKinematics::convertToJointTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create ConvertToJointVelTrajectory server
  this->convert_to_joint_vel_trajectory_server = create_service<ConvertToJointVelTrajectory>(
    "delta_kinematics/convert_to_joint_vel_trajectory",
    std::bind(&DeltaKinematics::convertToJointVelTrajectory, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Wait for the set_joint_limits service to be available
  this->set_joint_limits_client = create_client<SetJointLimits>("delta_motors/set_joint_limits");
  if (this->set_joint_limits_client->wait_for_service(std::chrono::seconds(2))) {
    // Call the set_joint_limits service and set the joint limits
    auto set_joint_limits_request = std::make_shared<SetJointLimits::Request>();
    set_joint_limits_request->min_rad = this->math_params.joint_min;
    set_joint_limits_request->max_rad = this->math_params.joint_max;
    set_joint_limits_request->max_vel_rad_s = this->math_params.max_joint_vel;
    auto result = this->set_joint_limits_client->async_send_request(set_joint_limits_request);
  } else {
    RCLCPP_WARN(this->get_logger(), "delta_motors/set_joint_limits service not found. Skipping hardware joint limits config (assuming simulation only).");
  }

  // Create RobotConfig Publisher
  this->robot_config_publisher = create_publisher<RobotConfig>("delta_robot/robot_config", 10);

  // Create subscribers for motor positions and velocities
  this->motor_positions_sub = create_subscription<DeltaJoints>("delta_motors/motor_position_feedback", 10,
    [this](const DeltaJoints::SharedPtr msg) -> void {
    this->robot_state.theta1 = msg->theta1;
    this->robot_state.theta2 = msg->theta2;
    this->robot_state.theta3 = msg->theta3;
  });

  this->motor_velocities_sub = create_subscription<DeltaJointVels>("delta_motors/motor_velocity_feedback", 10,
    [this](const DeltaJointVels::SharedPtr msg) -> void {
    this->robot_state.theta1_vel = msg->theta1_vel;
    this->robot_state.theta2_vel = msg->theta2_vel;
    this->robot_state.theta3_vel = msg->theta3_vel;
  });

  // Create a timer to periodically get the motor positions and publish the robot configuration
  this->robot_config_timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / robot_config_freq),
    [this]() -> void {
    RobotConfig robot_config_msg;
    robot_config_msg.header.stamp = this->now();
    robot_config_msg.joint_angles.theta1 = this->robot_state.theta1;
    robot_config_msg.joint_angles.theta2 = this->robot_state.theta2;
    robot_config_msg.joint_angles.theta3 = this->robot_state.theta3;
    robot_config_msg.joint_velocities.theta1_vel = this->robot_state.theta1_vel;
    robot_config_msg.joint_velocities.theta2_vel = this->robot_state.theta2_vel;
    robot_config_msg.joint_velocities.theta3_vel = this->robot_state.theta3_vel;
    // Perform FK to get the end effector position
    auto fk = delta_math::forward_kinematics(this->math_params,
      robot_config_msg.joint_angles.theta1,
      robot_config_msg.joint_angles.theta2,
      robot_config_msg.joint_angles.theta3);
    robot_config_msg.end_effector_position.x = fk.x;
    robot_config_msg.end_effector_position.y = fk.y;
    robot_config_msg.end_effector_position.z = fk.z;
    this->robot_config_publisher->publish(robot_config_msg);
  }
  );
}

void DeltaKinematics::forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response) {
  auto fk = delta_math::forward_kinematics(this->math_params,
    request->joint_angles.theta1, request->joint_angles.theta2, request->joint_angles.theta3);

  response->solution.x = fk.x;
  response->solution.y = fk.y;
  response->solution.z = fk.z;
  response->success = true;
}

void DeltaKinematics::inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response) {
  auto ik = delta_math::inverse_kinematics(this->math_params,
    request->solution.x, request->solution.y, request->solution.z);

  response->joint_angles.theta1 = ik.theta1;
  response->joint_angles.theta2 = ik.theta2;
  response->joint_angles.theta3 = ik.theta3;
  response->joint_angles.theta4 = std::numeric_limits<double>::quiet_NaN();
  response->joint_angles.theta5 = std::numeric_limits<double>::quiet_NaN();
  response->success = std::isfinite(ik.theta1);
}

void DeltaKinematics::convertToJointTrajectory(const std::shared_ptr<ConvertToJointTrajectory::Request> request, std::shared_ptr<ConvertToJointTrajectory::Response> response) {
  std::vector<Point> trajectory = request->end_effector_trajectory;
  std::vector<DeltaJoints> joint_trajectory;

  for (const auto& point : trajectory) {
    auto ik = delta_math::inverse_kinematics(this->math_params, point.x, point.y, point.z);

    DeltaJoints joint_angles;
    joint_angles.theta1 = ik.theta1;
    joint_angles.theta2 = ik.theta2;
    joint_angles.theta3 = ik.theta3;
    joint_angles.theta4 = std::numeric_limits<double>::quiet_NaN();
    joint_angles.theta5 = std::numeric_limits<double>::quiet_NaN();

    joint_trajectory.push_back(joint_angles);
  }

  response->joint_trajectory = joint_trajectory;
}

void DeltaKinematics::convertToJointVelTrajectory(const std::shared_ptr<ConvertToJointVelTrajectory::Request> request, std::shared_ptr<ConvertToJointVelTrajectory::Response> response) {
  std::vector<Point> ref_traj = request->end_effector_trajectory;
  std::vector<DeltaJointVels> joint_velocities;
  const size_t N = ref_traj.size();
  if (N == 0 || N == 1) return;

  double dt = 1.0 / (N - 1);

  // Convert ROS Points to delta_math::Point3D for gradient computation
  std::vector<delta_math::Point3D> positions(N);
  for (size_t i = 0; i < N; ++i) {
    positions[i] = {ref_traj[i].x, ref_traj[i].y, ref_traj[i].z};
  }
  auto ee_vel = delta_math::compute_gradient(positions, dt);
  RCLCPP_INFO(this->get_logger(), "Computed Gradient");

  for (size_t i = 0; i < ref_traj.size(); ++i) {
    auto ik = delta_math::inverse_kinematics(this->math_params,
      ref_traj[i].x, ref_traj[i].y, ref_traj[i].z);
    auto jv = delta_math::calc_theta_dot(this->math_params,
      ik.theta1, ik.theta2, ik.theta3,
      ee_vel[i].x_vel, ee_vel[i].y_vel, ee_vel[i].z_vel);

    DeltaJointVels msg;
    msg.theta1_vel = jv.theta1_vel;
    msg.theta2_vel = jv.theta2_vel;
    msg.theta3_vel = jv.theta3_vel;
    joint_velocities.push_back(msg);
  }
  RCLCPP_INFO(this->get_logger(), "Created Joint Velocity Trajectory");

  response->joint_vel_trajectory = joint_velocities;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaKinematics>());
  rclcpp::shutdown();
  return 0;
}