/**
 * @file vl53l1x_node.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Header file containing class definition for VL53L1XNode that publishes sensor data
 * on ROS topics.
 * @date 2025-03-18
 *
 *
 */

#ifndef VL53L1X_NODE_HPP
#define VL53L1X_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "vl53l1x.hpp"  // Library for the range sensor

 /**
  * @class VL53L1XNode
  * @brief A ROS2 Node for facilitating sensor readings from the VL53L1X range sensor over I2C
  *
  */
class VL53L1XNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the VL53L1XNode class.
   */
  VL53L1XNode();

  /**
   * @brief Destructor for the VL53L1XNode class.
   */
  ~VL53L1XNode();

private:

  /**
   * @brief The VL53L1X sensor object for interfacing with the sensor and requesting data.
   *
   */
  VL53L1X sensor;

  /**
   * @brief Timer for publishing sensor data at a fixed rate.
   *
   */
  rclcpp::TimerBase::SharedPtr timer;

  /**
   * @brief Publisher for range data.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensor_pub;
};

#endif // !VL53L1X_NODE_HPP