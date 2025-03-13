#ifndef SCANNING_HPP_
#define SCANNING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/play_demo_trajectory.hpp"
#include "deltarobot_interfaces/msg/robot_config.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/range.hpp"

using Point = geometry_msgs::msg::Point;
using Range = sensor_msgs::msg::Range;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using RobotConfig = deltarobot_interfaces::msg::RobotConfig;
using PlayDemoTrajectory = deltarobot_interfaces::srv::PlayDemoTrajectory;

class Scanning : public rclcpp::Node {
public:

  Scanning();
  ~Scanning() = default;

private:
  rclcpp::Client<PlayDemoTrajectory>::SharedPtr play_demo_trajectory_client;
  rclcpp::Subscription<RobotConfig>::SharedPtr robot_config_sub;
  rclcpp::Subscription<Range>::SharedPtr range_sub;

  // Recent values obtained from subscriptions
  std::unique_ptr<RobotConfig> robot_config;
  std::unique_ptr<Range> range;

};

#endif  // SCANNING_HPP_