#include "range_scanner.hpp"

RangeScanner::RangeScanner() : Node("range_scanner") {
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

  const double update_freq = 100.0; // [Hz]
  this->scanning_timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / update_freq),
    [this]() -> void {
    if (!this->isScanning) {
      this->startScanning();
    }
  }
  );
}

void RangeScanner::startScanning() {
  this->isScanning = true;
  this->scan_points.clear();

  // Get the latest robot configuration and range
  if (this->robot_config == nullptr || this->range == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Robot Config or Range not available. Skipping scan...");
    this->isScanning = false;
    return;
  }

  // Get the robot configuration
  RobotConfig robot_config = *this->robot_config;
  Point end_effector = robot_config.end_effector_position;

  // Get the range value
  Range range = *this->range;
  float range_val = range.range;

  // Create a scan point
  ScanPoint scan_point;
  scan_point.x = end_effector.x;
  scan_point.y = end_effector.y;
  scan_point.z = end_effector.z;
  scan_point.range = range_val;

  // Add the scan point to the list
  this->scan_points.push_back(scan_point);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeScanner>());
  rclcpp::shutdown();
  return 0;
}