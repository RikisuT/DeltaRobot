/**
 * @file kalman.hpp
 * @brief Header file for the KalmanFilter class and AlphaBetaFilter struct.
 *
 * This file contains the definition of the KalmanFilter class, which is a ROS2 node
 * that subscribes to raw sensor data and publishes filtered data using an alpha-beta filter.
 * It also defines the AlphaBetaFilter struct used for filtering.
 *
 * @author Sharwin Patil
 * @date 2023-03-18
 */

#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "deltarobot_interfaces/msg/robot_config.hpp"

using IMU = sensor_msgs::msg::Imu;
using MagField = sensor_msgs::msg::MagneticField;
using Temp = sensor_msgs::msg::Temperature;
using Range = sensor_msgs::msg::Range;
using RobotConfig = deltarobot_interfaces::msg::RobotConfig;
using Float32 = std_msgs::msg::Float32;

/**
 * @struct AlphaBetaFilter
 * @brief Struct for storing alpha-beta filter parameters and state.
 *
 * This struct contains the parameters and state variables for an alpha-beta filter,
 * which is used to filter sensor data.
 */
typedef struct {

  /**
   * @brief Alpha value for filter
   *
   * @note Determines how much of the residual error is used to update the estimate.
   *
   */
  float alpha;

  /**
   * @brief Beta value for filter
   *
   * @note Determines how much of the residual error is used to update the rate estimate.
   */
  float beta;
  float estimate; // Current estimate
  float rateEstimate; // Current rate estimate
  float previousEstimate = 0; // Previous estimate
  float previousRateEstimate = 0; // Previous rate estimate
  rclcpp::Time prevMeasureTime = rclcpp::Time(0); // Time of the previous measurement
} AlphaBetaFilter;

/**
 * @class KalmanFilter
 * @brief A ROS2 node for filtering sensor data using a kalman filter and alpha-beta filters.
 *
 * The KalmanFilter class subscribes to raw sensor data topics and publishes
 * filtered data topics.
 *
 * It uses an alpha-beta filter to process the time of flight and temperaturesensor data.
 */
class KalmanFilter : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the KalmanFilter class.
   */
  KalmanFilter();

  /**
   * @brief Destructor for the KalmanFilter class.
   */
  ~KalmanFilter() = default;

private:
  float H; // Height from the ground to the fixed base of the robot [mm]
  AlphaBetaFilter rangeFilter; // Alpha-beta filter for range data
  AlphaBetaFilter tempFilter; // Alpha-beta filter for temperature data
  RobotConfig latestConfig; // Latest robot configuration data
  float tofOffsetMM; // Z Offset of the time of flight sensor from the EE Point [mm]

  /**
   * @brief Timer callback for running kalman filter.
   *
   */
  void timerCallback();

  /**
   * @brief Apply the alpha-beta filter to a measurement.
   * @param z The measurement value.
   * @param filter The alpha-beta filter to apply and update.
   */
  void applyAlphaBetaFilter(float z, AlphaBetaFilter& filter);

  /**
   * @brief Callback function for IMU data.
   * @param msg The IMU message.
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Callback function for magnetic field data.
   * @param msg The magnetic field message.
   */
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  /**
   * @brief Callback function for temperature data.
   * @param msg The temperature message.
   */
  void tempCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);

  /**
   * @brief Callback function for range data.
   * @param msg The range message.
   */
  void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer; // Timer for running the kalman filter

  // Raw data subscriptions
  rclcpp::Subscription<IMU>::SharedPtr raw_imu_sub; // Subscription for raw IMU data
  rclcpp::Subscription<MagField>::SharedPtr raw_mag_sub; // Subscription for raw magnetic field data
  rclcpp::Subscription<Temp>::SharedPtr raw_temp_sub; // Subscription for raw temperature data
  rclcpp::Subscription<Range>::SharedPtr raw_range_sub; // Subscription for raw range data
  rclcpp::Subscription<RobotConfig>::SharedPtr robot_config_sub; // Subscription for robot configuration data

  // Filtered data publishers
  rclcpp::Publisher<IMU>::SharedPtr filtered_imu_pub; // Publisher for filtered IMU data
  rclcpp::Publisher<MagField>::SharedPtr filtered_mag_pub; // Publisher for filtered magnetic field data
  rclcpp::Publisher<Temp>::SharedPtr filtered_temp_pub; // Publisher for filtered temperature data
  rclcpp::Publisher<Range>::SharedPtr filtered_range_pub; // Publisher for filtered range data
  rclcpp::Publisher<Float32>::SharedPtr filtered_range_error_pub; // Publisher for filtered range error data
};

#endif // !KALMAN_HPP