#include "scanning.hpp"

Scanning::Scanning() : Node("scanner") {
  RCLCPP_INFO(this->get_logger(), "Scanning Node Started");

  // Create the subscriptions and update the recent values
  this->robot_config_sub = this->create_subscription<RobotConfig>(
    "/robot_config",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const RobotConfig::SharedPtr msg) -> void {this->robot_config = std::make_unique<RobotConfig>(*msg);}
  );

  this->range_sub = this->create_subscription<Range>(
    "vl53l1x/range",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [this](const Range::SharedPtr msg) -> void {this->range = std::make_unique<Range>(*msg);}
  );

  // Create play_demo_trajectory client
  this->play_demo_trajectory_client = this->create_client<PlayDemoTrajectory>("play_demo_trajectory");
  while (!this->play_demo_trajectory_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}