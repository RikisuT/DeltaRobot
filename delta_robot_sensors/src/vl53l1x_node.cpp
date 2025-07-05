#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "vl53l1x_node.hpp"
#include "vl53l1x.hpp"  // Library for the range sensor

VL53L1XNode::VL53L1XNode() : Node("VL53L1X_Sensor") {
  // Declare Parameters
  float sensor_timeout_ms = this->declare_parameter("sensor_timeout_ms", 500);
  float sensor_freq = this->declare_parameter("sensor_frequency", 25.0);
  float timing_budget_ms = this->declare_parameter("timing_budget_ms", 20.0); // 20ms for Short distance mode
  // Get parameters from config file
  sensor_timeout_ms = this->get_parameter("sensor_timeout_ms").as_int();
  sensor_freq = this->get_parameter("sensor_frequency").as_double();
  timing_budget_ms = this->get_parameter("timing_budget_ms").as_double();

  // Ensure the sensor frequency does not exceed 50Hz
  if (sensor_freq > 50.0) { sensor_freq = 50.0; }

  if (!this->sensor.init()) {
    rclcpp::shutdown();
    throw std::runtime_error("VL53L1X Sensor offline! Shutting down node");
  }

  RCLCPP_INFO(this->get_logger(), "VL53L1X Sensor online!");

  // Set a timeout on the sensor. (Stop waiting and respond with an error)
  this->sensor.setTimeout(sensor_timeout_ms);
  this->sensor.setDistanceMode(VL53L1X::DistanceMode::Short);
  this->sensor.setMeasurementTimingBudget(timing_budget_ms * 1000);  // Convert ms to us
  this->sensor.startContinuous(static_cast<int>(1000.0 / sensor_freq));  // Hardcode testcase 100

  // Setup the publisher
  const std::string topic = "vl53l1x/raw_range";
  sensor_pub = this->create_publisher<sensor_msgs::msg::Range>(topic, 5);
  RCLCPP_INFO(this->get_logger(), "VL53L1X Sensor publishing on topic (%s) at %.1f Hz", topic.c_str(), sensor_freq);

  // Create a timer to publish the sensor data
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / sensor_freq)),
    [this]() -> void {
    uint16_t distance = this->sensor.read_range();
    if (this->sensor.timeoutOccurred()) {
      RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
      distance = 0;
    }
    rclcpp::Time now = this->now();
    sensor_msgs::msg::Range msg;
    msg.header.frame_id = "VL53L1X_frame";
    msg.header.stamp = now;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.471239;  // 27 degrees in radians
    // msg.min_range = 0.14;                  // 140 mm.  (It is actully much less, but this makes sense in the context
    // msg.max_range = 3.00;                  // 3.6 m. in the dark, down to 73cm in bright light
    msg.min_range = 0.04;          // 4 cm for short distance mode
    msg.max_range = 1.35;          // 135 cm for short distance mode
    msg.range = static_cast<float>(distance) / 1000.0;  // Convert mm to meters

    // from https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg
    // # (Note: values < range_min or > range_max should be discarded)
    if ((msg.range >= msg.min_range) && (msg.range <= msg.max_range)) {
      this->sensor_pub->publish(msg);
    }
  }
  );
}

VL53L1XNode::~VL53L1XNode() {
  RCLCPP_INFO(this->get_logger(), "VL53L1XNode Destructor");
  this->sensor.stopContinuous();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VL53L1XNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), e.what());
  }
  rclcpp::shutdown();
  return 0;
}