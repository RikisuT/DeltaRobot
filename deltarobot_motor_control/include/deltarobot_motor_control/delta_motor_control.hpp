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
#include "deltarobot_interfaces/srv/get_dynamixel_positions.hpp"
#include "deltarobot_interfaces/srv/get_dynamixel_velocities.hpp"

using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using GetPositions = deltarobot_interfaces::srv::GetDynamixelPositions;
using GetVelocities = deltarobot_interfaces::srv::GetDynamixelVelocities;

class DeltaMotorControl : public rclcpp::Node {
public:

  DeltaMotorControl();
  ~DeltaMotorControl() = default;

private:
  rclcpp::Subscription<DeltaJoints>::SharedPtr delta_joints_sub;
  rclcpp::Subscription<DeltaJointVels>::SharedPtr delta_joint_vels_sub;
  rclcpp::Service<GetPositions>::SharedPtr get_positions_server;
  rclcpp::Service<GetVelocities>::SharedPtr get_velocities_server;

  dynamixel::PortHandler* portHandler;
  dynamixel::PacketHandler* packetHandler;
  dynamixel::GroupSyncWrite* groupSyncWrite;

  void initializeDynamixels();

  uint32_t convertToMotorPosition(float theta);
  uint32_t convertToMotorVelocity(float theta_vel);
};

#endif  // DELTA_MOTOR_CONTROL_HPP_