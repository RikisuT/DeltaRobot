#ifndef RANGE_SCANNER_HPP_
#define RANGE_SCANNER_HPP_

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

typedef struct {
  double x;
  double y;
  double z;
  double range;
} ScanPoint;

class RangeScanner : public rclcpp::Node {
public:

  RangeScanner();
  ~RangeScanner() = default;

private:
  rclcpp::Client<PlayDemoTrajectory>::SharedPtr play_demo_trajectory_client;
  rclcpp::Subscription<RobotConfig>::SharedPtr robot_config_sub;
  rclcpp::Subscription<Range>::SharedPtr range_sub;
  rclcpp::TimerBase::SharedPtr scanning_timer;

  void startScanning();

  // Recent values obtained from subscriptions
  std::unique_ptr<RobotConfig> robot_config;
  std::unique_ptr<Range> range;

  // Scanning parameters
  std::vector<ScanPoint> scan_points;

  bool isScanning = false;
};

#endif  // RANGE_SCANNER_HPP_