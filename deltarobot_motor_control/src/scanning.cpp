#include "scanning.hpp"

Scanning::Scanning() : Node("scanner") {
  RCLCPP_INFO(this->get_logger(), "Scanning Node Started");

  this->play_demo_trajectory_client = this->create_client<PlayDemoTrajectory>("play_demo_trajectory");
  this->robot_config_sub = this->create_subscription<RobotConfig>(
    "/robot_config",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const RobotConfig::SharedPtr msg) -> void
    {
      this->robot_config = *msg;
    }
  );

  this->range_sub = this->create_subscription<Range>(
    "vl53l1x/range",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const Range::SharedPtr msg) -> void
    {
      this->range = *msg;
    }
  );
}