#ifndef DELTA_MOTOR_CONTROL_HPP_
#define DELTA_MOTOR_CONTROL_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/msg/delta_joint_vels.hpp"
#include "deltarobot_interfaces/srv/set_joint_limits.hpp"

using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using SetJointLimits = deltarobot_interfaces::srv::SetJointLimits;

class DeltaMotorControl : public rclcpp::Node {
public:

  DeltaMotorControl();
  ~DeltaMotorControl();

private:
  rclcpp::Subscription<DeltaJoints>::SharedPtr delta_joints_sub; // Subcriber to receive position commands
  rclcpp::Subscription<DeltaJointVels>::SharedPtr delta_joint_vels_sub; // Subscriber to receive velocity commands
  rclcpp::Publisher<DeltaJoints>::SharedPtr motor_positions_pub; // Publisher for real-time motor position feedback
  rclcpp::Publisher<DeltaJointVels>::SharedPtr motor_velocities_pub; // Publisher for real-time motor velocity feedback
  rclcpp::Service<SetJointLimits>::SharedPtr set_joint_limits_server;

  dynamixel::PortHandler* portHandler;
  dynamixel::PacketHandler* packetHandler;
  dynamixel::GroupSyncWrite* groupSyncWrite;

  uint8_t control_mode;
  uint8_t getControlMode() { return this->control_mode; }
  void setControlMode(uint8_t ctrl_mode);

  void initializeDynamixels();

  void disableTorque();
  void enableTorque();


  float convertToRadians(int motor_pos);
  uint32_t convertToMotorPosition(float theta);
  int convertToMotorVelocity(float theta_vel);
};

#endif  // DELTA_MOTOR_CONTROL_HPP_