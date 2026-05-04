#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/msg/delta_joint_vels.hpp"
#include "deltarobot_interfaces/msg/robot_config.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_vel_trajectory.hpp"
#include "deltarobot_interfaces/srv/set_joint_limits.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "delta_robot/delta_math.hpp"

#include <vector>

using DeltaFK = deltarobot_interfaces::srv::DeltaFK;
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using ConvertToJointVelTrajectory = deltarobot_interfaces::srv::ConvertToJointVelTrajectory;
using SetJointLimits = deltarobot_interfaces::srv::SetJointLimits;
using Point = geometry_msgs::msg::Point;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using RobotConfig = deltarobot_interfaces::msg::RobotConfig;

typedef struct {
  float theta1;
  float theta2;
  float theta3;
  float theta1_vel;
  float theta2_vel;
  float theta3_vel;
} DeltaRobotState;

class DeltaKinematics : public rclcpp::Node {
public:
  DeltaKinematics();
  ~DeltaKinematics() = default;

private:
  rclcpp::Service<DeltaFK>::SharedPtr delta_fk_server;
  rclcpp::Service<DeltaIK>::SharedPtr delta_ik_server;
  rclcpp::Service<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_server;
  rclcpp::Service<ConvertToJointVelTrajectory>::SharedPtr convert_to_joint_vel_trajectory_server;
  rclcpp::Client<SetJointLimits>::SharedPtr set_joint_limits_client;
  rclcpp::Subscription <DeltaJoints >::SharedPtr motor_positions_sub;
  rclcpp::Subscription <DeltaJointVels >::SharedPtr motor_velocities_sub;
  rclcpp::Publisher<RobotConfig>::SharedPtr robot_config_publisher;
  rclcpp::TimerBase::SharedPtr robot_config_timer;

  // Service Callbacks
  void forwardKinematics(const std::shared_ptr<DeltaFK::Request> request, std::shared_ptr<DeltaFK::Response> response);
  void inverseKinematics(const std::shared_ptr<DeltaIK::Request> request, std::shared_ptr<DeltaIK::Response> response);
  void convertToJointTrajectory(const std::shared_ptr<ConvertToJointTrajectory::Request> request, std::shared_ptr<ConvertToJointTrajectory::Response> response);
  void convertToJointVelTrajectory(const std::shared_ptr<ConvertToJointVelTrajectory::Request> request, std::shared_ptr<ConvertToJointVelTrajectory::Response> response);

  // Latest Robot Configuration from feedback subscribers
  DeltaRobotState robot_state;

  /// Robot geometric parameters — fed into delta_math functions
  delta_math::DeltaRobotParams math_params;
};

#endif  // KINEMATICS_HPP_